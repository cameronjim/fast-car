//! The agent's access token. It goes in the operating system's credential store (Keychain on
//! macOS, Credential Manager on Windows, the Secret Service on Linux) when that works, and in a
//! file in the app's own data folder (mode 0600) when it does not.
//!
//! Why the file: the owner lost the token three times. The app is rebuilt unsigned, so every new
//! build is a different program to the macOS keychain: it asks for the login keychain password
//! on the next read, and when that is refused or cancelled the saved token is simply gone and a
//! token that is shown only once cannot be recovered. So when the keychain refuses a read or a
//! write the token is kept in `<app data>/token` instead (and a marker file in the same folder
//! remembers that, so the keychain is not asked, and the password prompt not shown, on every
//! start). The UI says which of the two is in use.
//!
//! Trade-off, on purpose: the UI process reads the token over IPC and attaches it to requests
//! (the web UI's `Api` class does that in both builds), so a compromised page could read it. The
//! page only ever loads the bundled UI, the capability file grants no filesystem or shell access,
//! and the agent itself is LAN only with a per-IP lockout. A file with mode 0600 is readable by
//! the same user's other programs, which the keychain would also hand the token to once allowed.
//! It is not a weaker secret than the fallback the owner was already using (a note on paper).
//! On Windows the file inherits the folder's permissions (the user's profile).

use serde::Serialize;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

pub const SERVICE: &str = "com.cameronjim.racer";
pub const ACCOUNT: &str = "agent-token";
const TOKEN_FILE: &str = "token";
/// Present once the keychain has refused us. Its content is the reason, for the UI.
const MARKER_FILE: &str = "token-uses-file";

/// The slice of a credential store the app needs. The real one is the OS keychain.
pub trait Secrets: Send + Sync {
    fn get(&self) -> Result<Option<String>, String>;
    fn set(&self, value: &str) -> Result<(), String>;
    fn clear(&self) -> Result<(), String>;
}

pub struct Keychain;

impl Keychain {
    fn entry() -> Result<keyring::Entry, String> {
        keyring::Entry::new(SERVICE, ACCOUNT).map_err(|e| e.to_string())
    }
}

impl Secrets for Keychain {
    fn get(&self) -> Result<Option<String>, String> {
        match Self::entry()?.get_password() {
            Ok(p) => Ok(Some(p)),
            Err(keyring::Error::NoEntry) => Ok(None),
            Err(e) => Err(e.to_string()),
        }
    }

    fn set(&self, value: &str) -> Result<(), String> {
        Self::entry()?
            .set_password(value)
            .map_err(|e| e.to_string())
    }

    fn clear(&self) -> Result<(), String> {
        match Self::entry()?.delete_credential() {
            Ok(()) | Err(keyring::Error::NoEntry) => Ok(()),
            Err(e) => Err(e.to_string()),
        }
    }
}

/// Write `text` to `path` so that only the owner can read it, never leaving half a file.
fn write_private(path: &Path, text: &str) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        fs::create_dir_all(dir)?;
    }
    let tmp = path.with_extension("tmp");
    let mut opts = fs::OpenOptions::new();
    opts.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        opts.mode(0o600);
    }
    let mut file = opts.open(&tmp)?;
    #[cfg(unix)]
    {
        // The mode above only applies when the file is created; a leftover tmp may differ.
        use std::os::unix::fs::PermissionsExt;
        file.set_permissions(fs::Permissions::from_mode(0o600))?;
    }
    file.write_all(text.as_bytes())?;
    file.sync_all()?;
    drop(file);
    fs::rename(&tmp, path)
}

/// The token as a plain file in the app's data folder, mode 0600.
pub struct FileSecrets {
    path: PathBuf,
}

impl FileSecrets {
    pub fn new(dir: &Path) -> Self {
        FileSecrets {
            path: dir.join(TOKEN_FILE),
        }
    }
}

impl Secrets for FileSecrets {
    fn get(&self) -> Result<Option<String>, String> {
        match fs::read_to_string(&self.path) {
            Ok(text) => {
                let t = text.trim();
                Ok((!t.is_empty()).then(|| t.to_string()))
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(e.to_string()),
        }
    }

    fn set(&self, value: &str) -> Result<(), String> {
        write_private(&self.path, value).map_err(|e| e.to_string())
    }

    fn clear(&self) -> Result<(), String> {
        match fs::remove_file(&self.path) {
            Ok(()) => Ok(()),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
            Err(e) => Err(e.to_string()),
        }
    }
}

/// Where the token is kept right now. Serialised for the UI.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Where {
    Keychain,
    File,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct StorageInfo {
    pub storage: Where,
    /// Why the keychain is not used (empty when it is).
    pub note: String,
}

/// Keychain first, the data-folder file when the keychain refuses, and the choice remembered.
pub struct Store {
    primary: Box<dyn Secrets>,
    file: FileSecrets,
    marker: PathBuf,
    /// `Some(reason)` once the keychain has been given up on.
    fallback: Mutex<Option<String>>,
}

impl Store {
    pub fn new(primary: Box<dyn Secrets>, dir: &Path) -> Self {
        let marker = dir.join(MARKER_FILE);
        let fallback = fs::read_to_string(&marker)
            .ok()
            .map(|t| t.trim().to_string());
        Store {
            primary,
            file: FileSecrets::new(dir),
            marker,
            fallback: Mutex::new(fallback),
        }
    }

    fn reason(&self) -> Option<String> {
        self.fallback.lock().expect("fallback lock").clone()
    }

    /// Give up on the keychain from now on (this run and the next ones).
    fn use_file(&self, why: String) {
        let _ = write_private(&self.marker, &why);
        *self.fallback.lock().expect("fallback lock") = Some(why);
    }

    pub fn storage(&self) -> StorageInfo {
        match self.reason() {
            None => StorageInfo {
                storage: Where::Keychain,
                note: String::new(),
            },
            Some(note) => StorageInfo {
                storage: Where::File,
                note,
            },
        }
    }

    pub fn get(&self) -> Result<Option<String>, String> {
        if self.reason().is_some() {
            return self.file.get();
        }
        match self.primary.get() {
            Ok(Some(t)) => Ok(Some(t)),
            // Nothing in the keychain: a token saved by an earlier fallback may be in the file.
            Ok(None) => self.file.get(),
            Err(e) => {
                self.use_file(format!("keychain read refused: {e}"));
                self.file.get()
            }
        }
    }

    /// Save a token. Returns where it went.
    pub fn set(&self, token: &str) -> Result<StorageInfo, String> {
        let token = validate(token)?;
        if self.reason().is_none() {
            match self.primary.set(token) {
                Ok(()) => {
                    // The keychain has it now: no second copy lying around.
                    let _ = self.file.clear();
                    return Ok(self.storage());
                }
                Err(e) => self.use_file(format!("keychain write refused: {e}")),
            }
        }
        self.file
            .set(token)
            .map_err(|e| format!("the token could not be saved to the keychain or a file: {e}"))?;
        Ok(self.storage())
    }

    /// Forget the token everywhere. A keychain that refuses is ignored: the file is what counts
    /// once it has been given up on, and an unreadable keychain entry is not ours to fix.
    pub fn clear(&self) -> Result<(), String> {
        if self.reason().is_none() {
            let _ = self.primary.clear();
        }
        self.file.clear()
    }
}

/// Reject what can never be a token before it is stored anywhere.
pub fn validate(token: &str) -> Result<&str, String> {
    let t = token.trim();
    if t.is_empty() {
        return Err("the token is empty".into());
    }
    if t.len() > 512 || t.chars().any(|c| c.is_control() || c.is_whitespace()) {
        return Err("that does not look like a token".into());
    }
    Ok(t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

    fn temp_dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("racer-token-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&d);
        d
    }

    /// A keychain in memory that can be told to refuse, and counts how often it was asked.
    #[derive(Clone, Default)]
    struct Fake {
        value: Arc<Mutex<Option<String>>>,
        deny: Arc<Mutex<Option<String>>>,
        calls: Arc<AtomicUsize>,
    }

    impl Fake {
        fn denying(&self, why: Option<&str>) {
            *self.deny.lock().unwrap() = why.map(String::from);
        }
        fn check(&self) -> Result<(), String> {
            self.calls.fetch_add(1, Ordering::SeqCst);
            match self.deny.lock().unwrap().clone() {
                Some(why) => Err(why),
                None => Ok(()),
            }
        }
    }

    impl Secrets for Fake {
        fn get(&self) -> Result<Option<String>, String> {
            self.check()?;
            Ok(self.value.lock().unwrap().clone())
        }
        fn set(&self, v: &str) -> Result<(), String> {
            self.check()?;
            *self.value.lock().unwrap() = Some(v.to_string());
            Ok(())
        }
        fn clear(&self) -> Result<(), String> {
            self.check()?;
            *self.value.lock().unwrap() = None;
            Ok(())
        }
    }

    fn store(dir: &Path, fake: &Fake) -> Store {
        Store::new(Box::new(fake.clone()), dir)
    }

    #[test]
    fn round_trips_through_the_keychain_and_stores_trimmed() {
        let d = temp_dir("keychain");
        let fake = Fake::default();
        let s = store(&d, &fake);
        assert_eq!(s.get().unwrap(), None);
        let info = s.set("  abc123  ").unwrap();
        assert_eq!(info.storage, Where::Keychain);
        assert_eq!(s.get().unwrap().as_deref(), Some("abc123"));
        assert!(
            !d.join(TOKEN_FILE).exists(),
            "no file copy while the keychain works"
        );
        s.clear().unwrap();
        assert_eq!(s.get().unwrap(), None);
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn refuses_things_that_are_not_tokens() {
        let d = temp_dir("invalid");
        let fake = Fake::default();
        let s = store(&d, &fake);
        for bad in ["", "   ", "a b", "tab\there", "new\nline"] {
            assert!(s.set(bad).is_err(), "{bad:?}");
        }
        assert!(s.set(&"x".repeat(513)).is_err());
        assert_eq!(s.get().unwrap(), None, "nothing was written");
        assert!(!d.join(TOKEN_FILE).exists());
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn a_refused_keychain_write_falls_back_to_a_private_file() {
        let d = temp_dir("write-denied");
        let fake = Fake::default();
        fake.denying(Some("user cancelled"));
        let s = store(&d, &fake);
        let info = s.set("tok-1").unwrap();
        assert_eq!(info.storage, Where::File);
        assert!(info.note.contains("user cancelled"), "{info:?}");
        assert_eq!(s.get().unwrap().as_deref(), Some("tok-1"));
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = fs::metadata(d.join(TOKEN_FILE))
                .unwrap()
                .permissions()
                .mode();
            assert_eq!(mode & 0o777, 0o600, "token file is private to the owner");
        }
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn a_refused_keychain_read_is_remembered_and_the_keychain_is_not_asked_again() {
        let d = temp_dir("read-denied");
        let fake = Fake::default();
        fake.denying(Some("keychain locked"));
        let s = store(&d, &fake);
        assert_eq!(
            s.get().unwrap(),
            None,
            "nothing saved yet, and no error to the UI"
        );
        assert_eq!(s.storage().storage, Where::File);
        let asked = fake.calls.load(Ordering::SeqCst);
        s.set("tok-2").unwrap();
        s.get().unwrap();
        assert_eq!(
            fake.calls.load(Ordering::SeqCst),
            asked,
            "no more keychain prompts"
        );

        // a new launch of the app: the marker keeps the choice, and the token is still there
        let again = store(&d, &fake);
        assert_eq!(again.storage().storage, Where::File);
        assert!(again.storage().note.contains("keychain locked"));
        assert_eq!(again.get().unwrap().as_deref(), Some("tok-2"));
        assert_eq!(fake.calls.load(Ordering::SeqCst), asked);
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn a_token_survives_a_reinstall_that_locks_the_keychain_entry() {
        let d = temp_dir("reinstall");
        let fake = Fake::default();
        store(&d, &fake).set("old").unwrap(); // saved to the keychain by the old build
        fake.denying(Some("the new build may not read it"));
        let new_build = store(&d, &fake);
        assert_eq!(
            new_build.get().unwrap(),
            None,
            "the old keychain item is out of reach"
        );
        new_build.set("pasted-again").unwrap();
        let next_launch = store(&d, &fake);
        assert_eq!(next_launch.get().unwrap().as_deref(), Some("pasted-again"));
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn a_token_in_the_file_is_found_when_the_keychain_is_empty() {
        let d = temp_dir("file-first");
        FileSecrets::new(&d).set("from-file").unwrap();
        let s = store(&d, &Fake::default());
        assert_eq!(s.get().unwrap().as_deref(), Some("from-file"));
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn saving_to_a_working_keychain_removes_a_stale_file_copy() {
        let d = temp_dir("stale");
        FileSecrets::new(&d).set("stale").unwrap();
        let fake = Fake::default();
        let s = store(&d, &fake);
        s.set("fresh").unwrap();
        assert!(!d.join(TOKEN_FILE).exists());
        assert_eq!(s.get().unwrap().as_deref(), Some("fresh"));
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn clear_removes_the_token_from_both_places_even_if_the_keychain_refuses() {
        let d = temp_dir("clear");
        let fake = Fake::default();
        fake.denying(Some("no"));
        let s = store(&d, &fake);
        s.set("tok").unwrap();
        s.clear().unwrap();
        assert_eq!(s.get().unwrap(), None);
        assert!(!d.join(TOKEN_FILE).exists());
        s.clear().unwrap(); // clearing nothing is fine
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn an_unwritable_data_folder_is_reported_not_swallowed() {
        let d = temp_dir("unwritable");
        fs::create_dir_all(&d).unwrap();
        // a file where the folder should be: neither the keychain nor the file can take it
        let blocker = d.join("blocked");
        fs::write(&blocker, "x").unwrap();
        let fake = Fake::default();
        fake.denying(Some("no"));
        let s = Store::new(Box::new(fake.clone()), &blocker);
        let err = s.set("tok").unwrap_err();
        assert!(err.contains("could not be saved"), "{err}");
        let _ = fs::remove_dir_all(&d);
    }
}
