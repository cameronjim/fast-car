//! Racer desktop: a thin native shell around the web UI in `tools/racer_app/web`.
//!
//! It does three things the browser cannot: find the car on the LAN (mDNS, then a subnet scan,
//! reconnecting by itself when the Jetson reboots), keep the agent token in the OS keychain, and
//! make HTTP calls to the agent without CORS (through `tauri-plugin-http`).
//!
//! It adds no way to drive. The native side sends `GET /health` and nothing else; the UI's
//! foxglove client only subscribes (`web/src/ros.ts`, enforced by `web/src/guard.test.ts`, which
//! also reads this crate). The RC kill knob stays the real stop.

pub mod discovery;
pub mod net;
pub mod store;
pub mod token;

use discovery::{Car, Net, State, Tracker, DEFAULT_PORT};
use std::path::PathBuf;
use std::sync::mpsc::{channel, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tauri::{AppHandle, Emitter, Manager};

const EVENT: &str = "racer://state";
/// Poll the connected car this often. A reboot is noticed within one interval.
const CONNECTED_EVERY: Duration = Duration::from_secs(4);
/// While searching or lost, go again right after the last search finishes.
const SEARCHING_EVERY: Duration = Duration::from_millis(500);

struct Shared {
    tracker: Mutex<Tracker>,
    net: Arc<dyn Net>,
    secrets: Box<dyn token::Secrets>,
    data_dir: PathBuf,
    wake: Mutex<Sender<()>>,
}

type SharedState = Arc<Shared>;

fn snapshot(shared: &Shared) -> State {
    shared.tracker.lock().expect("tracker lock").state()
}

/// Write the current car to disk if it changed.
fn persist(shared: &Shared) {
    let current = shared
        .tracker
        .lock()
        .expect("tracker lock")
        .current()
        .cloned();
    if let Some(car) = current {
        if store::load(&shared.data_dir)
            .as_ref()
            .map(|c| (&c.host, c.port, &c.hostname))
            != Some((&car.host, car.port, &car.hostname))
        {
            let _ = store::save(&shared.data_dir, &car);
        }
    }
}

fn wake(shared: &Shared) {
    if let Ok(tx) = shared.wake.lock() {
        let _ = tx.send(());
    }
}

#[tauri::command]
fn discovery_state(shared: tauri::State<'_, SharedState>) -> State {
    snapshot(&shared)
}

#[tauri::command]
fn select_car(app: AppHandle, shared: tauri::State<'_, SharedState>, car: Car) -> State {
    shared.tracker.lock().expect("tracker lock").select(car);
    persist(&shared);
    let state = snapshot(&shared);
    let _ = app.emit(EVENT, &state);
    wake(&shared);
    state
}

#[tauri::command]
async fn add_manual(
    app: AppHandle,
    shared: tauri::State<'_, SharedState>,
    address: String,
) -> Result<State, String> {
    let shared = shared.inner().clone();
    // The probe blocks for up to a second or two: keep it off the async runtime.
    tauri::async_runtime::spawn_blocking(move || {
        let car = discovery::probe_manual(shared.net.as_ref(), &address)?;
        shared.tracker.lock().expect("tracker lock").select(car);
        persist(&shared);
        let state = snapshot(&shared);
        let _ = app.emit(EVENT, &state);
        wake(&shared);
        Ok(state)
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
fn rescan(shared: tauri::State<'_, SharedState>) {
    wake(&shared);
}

#[tauri::command]
fn forget_car(app: AppHandle, shared: tauri::State<'_, SharedState>) -> State {
    shared.tracker.lock().expect("tracker lock").clear_current();
    store::forget(&shared.data_dir);
    let state = snapshot(&shared);
    let _ = app.emit(EVENT, &state);
    wake(&shared);
    state
}

#[tauri::command]
fn token_get(shared: tauri::State<'_, SharedState>) -> Result<Option<String>, String> {
    shared.secrets.get()
}

#[tauri::command]
fn token_set(shared: tauri::State<'_, SharedState>, token: String) -> Result<(), String> {
    token::store(shared.secrets.as_ref(), &token)
}

#[tauri::command]
fn token_clear(shared: tauri::State<'_, SharedState>) -> Result<(), String> {
    shared.secrets.clear()
}

/// The discovery loop: probe the car, search when it is gone, tell the UI when anything changes.
fn run_loop(app: AppHandle, shared: SharedState, rx: std::sync::mpsc::Receiver<()>) {
    loop {
        if let Some(state) = discovery::step(&shared.tracker, shared.net.as_ref()) {
            persist(&shared);
            let _ = app.emit(EVENT, &state);
        }
        let pause = match snapshot(&shared).phase {
            discovery::Phase::Connected => CONNECTED_EVERY,
            _ => SEARCHING_EVERY,
        };
        match rx.recv_timeout(pause) {
            Ok(()) | Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return,
        }
    }
}

pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_http::init())
        .setup(|app| {
            let data_dir = app.path().app_data_dir()?;
            let last = store::load(&data_dir);
            let (tx, rx) = channel();
            let shared: SharedState = Arc::new(Shared {
                tracker: Mutex::new(Tracker::new(last, DEFAULT_PORT)),
                net: Arc::new(net::RealNet::new()),
                secrets: Box::new(token::Keychain),
                data_dir,
                wake: Mutex::new(tx),
            });
            app.manage(shared.clone());
            let handle = app.handle().clone();
            std::thread::Builder::new()
                .name("racer-discovery".into())
                .spawn(move || run_loop(handle, shared, rx))?;
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            discovery_state,
            select_car,
            add_manual,
            rescan,
            forget_car,
            token_get,
            token_set,
            token_clear,
        ])
        .run(tauri::generate_context!())
        .expect("error while running the Racer app");
}
