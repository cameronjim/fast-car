//! Finding the car, and keeping track of it across Jetson reboots.
//!
//! The logic here never touches the network itself. It goes through the [`Net`] trait, so the
//! tests drive it with a fake resolver and the app plugs in the real one (`net.rs`). The order
//! mirrors `tools/racer_app/find_car.py`: mDNS first (`_racer._tcp`, then the plain name
//! `racer-car.local`), and only if that finds no agent that answers `/health`, a scan of the
//! local /24. A host is only ever trusted if `/health` says it is the racer agent.
//!
//! Nothing in this module sends anything but `GET /health`. It cannot start, stop or drive.

use serde::{Deserialize, Serialize};
use std::net::Ipv4Addr;
use std::sync::Mutex;
use std::time::Duration;

pub const DEFAULT_NAME: &str = "racer-car.local";
pub const DEFAULT_PORT: u16 = 8080;
pub const AGENT_NAME: &str = "racer-agent";

/// How long one `/health` probe may take. LAN round trips are a few milliseconds.
pub const PROBE_TIMEOUT: Duration = Duration::from_millis(600);
/// How long to listen for mDNS answers.
pub const MDNS_WINDOW: Duration = Duration::from_millis(2500);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Source {
    Mdns,
    Name,
    Scan,
    Manual,
    Saved,
}

/// A host that might be the car, before it has answered `/health`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Candidate {
    pub host: String,
    pub port: u16,
    pub source: Source,
}

/// What `/health` told us about a verified agent.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Health {
    pub hostname: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Car {
    pub host: String,
    pub port: u16,
    /// The Jetson's host name from `/health`. It survives a DHCP change of address, so it is how
    /// a car is recognised again after a reboot.
    pub hostname: String,
    pub source: Source,
}

impl Car {
    /// The same physical car: same host name when both have one, otherwise the same address.
    pub fn same_car(&self, other: &Car) -> bool {
        if !self.hostname.is_empty() && !other.hostname.is_empty() {
            self.hostname == other.hostname
        } else {
            self.host == other.host && self.port == other.port
        }
    }
}

/// Everything discovery needs from the outside world.
pub trait Net: Send + Sync {
    /// Browse `_racer._tcp` for `window` and return what answered.
    fn browse_mdns(&self, window: Duration) -> Vec<Candidate>;
    /// Resolve a name through the system resolver (macOS and Windows speak mDNS natively).
    fn resolve_name(&self, name: &str) -> Vec<String>;
    /// This machine's LAN address.
    fn local_ipv4(&self) -> Option<Ipv4Addr>;
    /// `Some` only if `host:port` answers `/health` as the racer agent.
    fn probe(&self, host: &str, port: u16, timeout: Duration) -> Option<Health>;
    /// Probe many hosts. The real implementation runs these in parallel.
    fn probe_many(&self, hosts: &[String], port: u16, timeout: Duration) -> Vec<(String, Health)> {
        hosts
            .iter()
            .filter_map(|h| {
                self.probe(h, port, timeout)
                    .map(|health| (h.clone(), health))
            })
            .collect()
    }
}

/// Every host in the /24 around `ip`, except `ip` itself.
pub fn subnet_hosts(ip: Ipv4Addr) -> Vec<String> {
    let [a, b, c, own] = ip.octets();
    (1..=254u8)
        .filter(|h| *h != own)
        .map(|h| Ipv4Addr::new(a, b, c, h).to_string())
        .collect()
}

fn push_unique(list: &mut Vec<Candidate>, cand: Candidate) {
    if !list
        .iter()
        .any(|c| c.host == cand.host && c.port == cand.port)
    {
        list.push(cand);
    }
}

/// All cars on the network right now: mDNS first, the subnet scan only if mDNS found none.
pub fn discover(net: &dyn Net, port: u16) -> Vec<Car> {
    let mut cands: Vec<Candidate> = net.browse_mdns(MDNS_WINDOW);
    for addr in net.resolve_name(DEFAULT_NAME) {
        push_unique(
            &mut cands,
            Candidate {
                host: addr,
                port,
                source: Source::Name,
            },
        );
    }
    let mut cars: Vec<Car> = cands
        .into_iter()
        .filter_map(|c| {
            net.probe(&c.host, c.port, PROBE_TIMEOUT).map(|h| Car {
                host: c.host,
                port: c.port,
                hostname: h.hostname,
                source: c.source,
            })
        })
        .collect();
    if cars.is_empty() {
        if let Some(ip) = net.local_ipv4() {
            let hosts = subnet_hosts(ip);
            cars = net
                .probe_many(&hosts, port, Duration::from_millis(400))
                .into_iter()
                .map(|(host, h)| Car {
                    host,
                    port,
                    hostname: h.hostname,
                    source: Source::Scan,
                })
                .collect();
        }
    }
    cars.sort_by(|a, b| a.host.cmp(&b.host));
    cars
}

/// Turn what the user typed into a host and port. Accepts `192.168.1.5`, `racer-car.local:8080`,
/// `http://192.168.1.5:8080/`, and `[fe80::1]:8080`.
pub fn parse_address(input: &str) -> Result<(String, u16), String> {
    let s = input.trim();
    let s = s.strip_prefix("http://").unwrap_or(s);
    let s = s.split('/').next().unwrap_or("");
    if s.is_empty() {
        return Err("enter an address".into());
    }
    let (host, port_text) = if let Some(rest) = s.strip_prefix('[') {
        let (h, tail) = rest
            .split_once(']')
            .ok_or("missing ] in the IPv6 address")?;
        (h.to_string(), tail.strip_prefix(':'))
    } else if s.matches(':').count() == 1 {
        let (h, p) = s.split_once(':').unwrap_or((s, ""));
        (h.to_string(), Some(p))
    } else if s.contains(':') {
        return Err("put an IPv6 address in square brackets".into());
    } else {
        (s.to_string(), None)
    };
    let port = match port_text {
        None => DEFAULT_PORT,
        Some(p) => p
            .parse::<u16>()
            .map_err(|_| format!("{p:?} is not a port number"))?,
    };
    if port == 0 {
        return Err("port 0 is not valid".into());
    }
    let ok = !host.is_empty()
        && host
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '-' | '_' | ':'));
    if !ok {
        return Err(format!("{host:?} is not a valid host name or address"));
    }
    Ok((host, port))
}

/// Check an address the user typed: it must answer as the racer agent.
pub fn probe_manual(net: &dyn Net, input: &str) -> Result<Car, String> {
    let (host, port) = parse_address(input)?;
    match net.probe(&host, port, Duration::from_millis(1500)) {
        Some(h) => Ok(Car {
            host,
            port,
            hostname: h.hostname,
            source: Source::Manual,
        }),
        None => Err(format!(
            "nothing on {host}:{port} answered as the racer agent (GET /health)"
        )),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Phase {
    /// No car chosen yet and none found, or still looking.
    Searching,
    /// The chosen car answers `/health`.
    Connected,
    /// The chosen car stopped answering (rebooting, off, Wi-Fi). The loop keeps looking for it.
    Lost,
}

/// What the UI is told. `car` is the one the app is using; `cars` is everything found.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct State {
    pub phase: Phase,
    pub car: Option<Car>,
    pub cars: Vec<Car>,
}

/// The discovery state machine. One `tick` is one round of probing and, if needed, searching.
#[derive(Clone, Debug)]
pub struct Tracker {
    /// The car the app is connected to or last used. Loaded from disk at start.
    current: Option<Car>,
    phase: Phase,
    cars: Vec<Car>,
    port: u16,
    /// Bumped by every user action, so a slow tick that raced with one is discarded.
    generation: u64,
}

impl Tracker {
    pub fn new(last_used: Option<Car>, port: u16) -> Self {
        let cars = last_used.iter().cloned().collect();
        Tracker {
            current: last_used,
            phase: Phase::Searching,
            cars,
            port,
            generation: 0,
        }
    }

    pub fn state(&self) -> State {
        State {
            phase: self.phase,
            car: self.current.clone(),
            cars: self.cars.clone(),
        }
    }

    pub fn current(&self) -> Option<&Car> {
        self.current.as_ref()
    }

    /// The user picked a car (from the list, or a manual address that answered).
    pub fn select(&mut self, car: Car) {
        self.generation += 1;
        if !self.cars.iter().any(|c| c.same_car(&car)) {
            self.cars.push(car.clone());
        }
        self.current = Some(car);
        self.phase = Phase::Connected;
    }

    /// Forget the current car, back to the picker.
    pub fn clear_current(&mut self) {
        self.generation += 1;
        self.current = None;
        self.phase = Phase::Searching;
    }

    /// One round. Returns true if the state changed.
    pub fn tick(&mut self, net: &dyn Net) -> bool {
        let before = self.state();
        match self.current.clone() {
            Some(car) => self.tick_with_car(net, car),
            None => self.tick_without_car(net),
        }
        self.state() != before
    }

    fn tick_with_car(&mut self, net: &dyn Net, car: Car) {
        if let Some(h) = net.probe(&car.host, car.port, PROBE_TIMEOUT) {
            // Something else may have taken the old address (DHCP): only our own host name counts.
            if car.hostname.is_empty() || h.hostname.is_empty() || h.hostname == car.hostname {
                let mut up = car;
                if !h.hostname.is_empty() {
                    up.hostname = h.hostname;
                }
                self.phase = Phase::Connected;
                self.cars = vec![up.clone()];
                self.current = Some(up);
                return;
            }
        }
        // The car did not answer as itself: it may be rebooting, or it may have a new address.
        self.phase = Phase::Lost;
        let found = discover(net, self.port);
        if let Some(again) = found.iter().find(|c| c.same_car(&car)) {
            self.current = Some(again.clone());
            self.phase = Phase::Connected;
        }
        if !found.is_empty() {
            self.cars = found;
        }
    }

    fn tick_without_car(&mut self, net: &dyn Net) {
        self.phase = Phase::Searching;
        self.cars = discover(net, self.port);
        // With nothing saved, a single car on the network is the car.
        if self.cars.len() == 1 {
            self.current = Some(self.cars[0].clone());
            self.phase = Phase::Connected;
        }
    }
}

/// Run one tick against the shared tracker without holding the lock during network I/O. If the
/// user acted meanwhile (the generation moved), the result is dropped. Returns the new state if
/// it changed.
pub fn step(shared: &Mutex<Tracker>, net: &dyn Net) -> Option<State> {
    let mut work = shared.lock().ok()?.clone();
    let changed = work.tick(net);
    let mut guard = shared.lock().ok()?;
    if guard.generation != work.generation {
        return None;
    }
    *guard = work;
    changed.then(|| guard.state())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use std::sync::Mutex as StdMutex;

    /// A fake network. `agents` maps "host:port" to the host name its /health returns.
    #[derive(Default)]
    struct Fake {
        mdns: Vec<Candidate>,
        names: Vec<String>,
        local: Option<Ipv4Addr>,
        agents: StdMutex<HashMap<String, String>>,
        probes: StdMutex<Vec<String>>,
    }

    impl Fake {
        fn agent(self, host: &str, port: u16, hostname: &str) -> Self {
            self.agents
                .lock()
                .unwrap()
                .insert(format!("{host}:{port}"), hostname.into());
            self
        }
        fn remove(&self, host: &str, port: u16) {
            self.agents
                .lock()
                .unwrap()
                .remove(&format!("{host}:{port}"));
        }
        fn probed(&self) -> usize {
            self.probes.lock().unwrap().len()
        }
    }

    impl Net for Fake {
        fn browse_mdns(&self, _: Duration) -> Vec<Candidate> {
            self.mdns.clone()
        }
        fn resolve_name(&self, _: &str) -> Vec<String> {
            self.names.clone()
        }
        fn local_ipv4(&self) -> Option<Ipv4Addr> {
            self.local
        }
        fn probe(&self, host: &str, port: u16, _: Duration) -> Option<Health> {
            let key = format!("{host}:{port}");
            self.probes.lock().unwrap().push(key.clone());
            self.agents.lock().unwrap().get(&key).map(|h| Health {
                hostname: h.clone(),
            })
        }
    }

    fn cand(host: &str, source: Source) -> Candidate {
        Candidate {
            host: host.into(),
            port: 8080,
            source,
        }
    }

    fn car(host: &str, hostname: &str) -> Car {
        Car {
            host: host.into(),
            port: 8080,
            hostname: hostname.into(),
            source: Source::Saved,
        }
    }

    #[test]
    fn mdns_answer_that_passes_health_is_a_car_and_no_scan_runs() {
        let net = Fake {
            mdns: vec![cand("192.168.1.50", Source::Mdns)],
            local: Some(Ipv4Addr::new(192, 168, 1, 10)),
            ..Fake::default()
        }
        .agent("192.168.1.50", 8080, "jetson");
        let cars = discover(&net, 8080);
        assert_eq!(cars.len(), 1);
        assert_eq!(cars[0].hostname, "jetson");
        assert_eq!(cars[0].source, Source::Mdns);
        assert_eq!(net.probed(), 1, "no subnet scan when mDNS found the car");
    }

    #[test]
    fn mdns_host_that_is_not_the_agent_is_ignored() {
        let net = Fake {
            mdns: vec![cand("192.168.1.77", Source::Mdns)],
            ..Fake::default()
        };
        assert!(discover(&net, 8080).is_empty());
    }

    #[test]
    fn the_plain_name_is_tried_and_deduplicated_against_mdns() {
        let net = Fake {
            mdns: vec![cand("192.168.1.50", Source::Mdns)],
            names: vec!["192.168.1.50".into(), "192.168.1.51".into()],
            ..Fake::default()
        }
        .agent("192.168.1.50", 8080, "a")
        .agent("192.168.1.51", 8080, "b");
        let cars = discover(&net, 8080);
        let hosts: Vec<_> = cars.iter().map(|c| c.host.as_str()).collect();
        assert_eq!(hosts, ["192.168.1.50", "192.168.1.51"]);
        assert_eq!(cars[0].source, Source::Mdns);
        assert_eq!(cars[1].source, Source::Name);
    }

    #[test]
    fn falls_back_to_scanning_the_subnet() {
        let net = Fake {
            local: Some(Ipv4Addr::new(10, 0, 5, 20)),
            ..Fake::default()
        }
        .agent("10.0.5.99", 8080, "jetson");
        let cars = discover(&net, 8080);
        assert_eq!(cars.len(), 1);
        assert_eq!(cars[0].host, "10.0.5.99");
        assert_eq!(cars[0].source, Source::Scan);
        assert_eq!(
            net.probed(),
            253,
            "every other host in the /24 is probed once"
        );
    }

    #[test]
    fn no_local_address_and_no_mdns_means_nothing_found() {
        assert!(discover(&Fake::default(), 8080).is_empty());
    }

    #[test]
    fn subnet_excludes_own_address_and_network_edges() {
        let hosts = subnet_hosts(Ipv4Addr::new(192, 168, 1, 10));
        assert_eq!(hosts.len(), 253);
        assert!(!hosts.contains(&"192.168.1.10".to_string()));
        assert!(!hosts.contains(&"192.168.1.0".to_string()));
        assert!(!hosts.contains(&"192.168.1.255".to_string()));
        assert_eq!(hosts[0], "192.168.1.1");
    }

    #[test]
    fn parses_addresses_people_type() {
        assert_eq!(
            parse_address("192.168.1.5"),
            Ok(("192.168.1.5".into(), 8080))
        );
        assert_eq!(
            parse_address(" racer-car.local:9000 "),
            Ok(("racer-car.local".into(), 9000))
        );
        assert_eq!(
            parse_address("http://10.0.0.2:8080/"),
            Ok(("10.0.0.2".into(), 8080))
        );
        assert_eq!(
            parse_address("[fe80::1]:8081"),
            Ok(("fe80::1".into(), 8081))
        );
        assert_eq!(parse_address("[fe80::1]"), Ok(("fe80::1".into(), 8080)));
    }

    #[test]
    fn rejects_bad_addresses() {
        for bad in [
            "",
            "   ",
            "host:abc",
            "host:0",
            "host:99999",
            "fe80::1",
            "[fe80::1",
            "a b",
            "a:b:c",
        ] {
            assert!(parse_address(bad).is_err(), "{bad:?} should be rejected");
        }
    }

    #[test]
    fn manual_address_must_answer_as_the_agent() {
        let net = Fake::default().agent("192.168.1.9", 8080, "jetson");
        let car = probe_manual(&net, "192.168.1.9").unwrap();
        assert_eq!(car.source, Source::Manual);
        assert_eq!(car.hostname, "jetson");
        assert!(probe_manual(&net, "192.168.1.10")
            .unwrap_err()
            .contains("racer agent"));
        assert!(probe_manual(&net, "").is_err());
    }

    #[test]
    fn auto_selects_a_lone_car_when_nothing_was_saved() {
        let net = Fake {
            mdns: vec![cand("192.168.1.50", Source::Mdns)],
            ..Fake::default()
        }
        .agent("192.168.1.50", 8080, "jetson");
        let mut t = Tracker::new(None, 8080);
        assert!(t.tick(&net));
        let s = t.state();
        assert_eq!(s.phase, Phase::Connected);
        assert_eq!(s.car.unwrap().host, "192.168.1.50");
    }

    #[test]
    fn two_cars_and_nothing_saved_waits_for_the_user() {
        let net = Fake {
            mdns: vec![
                cand("192.168.1.50", Source::Mdns),
                cand("192.168.1.51", Source::Mdns),
            ],
            ..Fake::default()
        }
        .agent("192.168.1.50", 8080, "a")
        .agent("192.168.1.51", 8080, "b");
        let mut t = Tracker::new(None, 8080);
        t.tick(&net);
        let s = t.state();
        assert_eq!(s.phase, Phase::Searching);
        assert!(s.car.is_none());
        assert_eq!(s.cars.len(), 2);
    }

    #[test]
    fn reconnects_to_the_last_used_car_on_start() {
        let net = Fake::default().agent("192.168.1.50", 8080, "jetson");
        let mut t = Tracker::new(Some(car("192.168.1.50", "jetson")), 8080);
        t.tick(&net);
        assert_eq!(t.state().phase, Phase::Connected);
        assert_eq!(
            net.probed(),
            1,
            "the saved car is probed directly, no search"
        );
    }

    #[test]
    fn follows_the_car_to_a_new_address_after_a_reboot() {
        // It was .50, comes back as .60 with the same host name.
        let net = Fake {
            mdns: vec![cand("192.168.1.60", Source::Mdns)],
            ..Fake::default()
        }
        .agent("192.168.1.60", 8080, "jetson");
        let mut t = Tracker::new(Some(car("192.168.1.50", "jetson")), 8080);
        assert!(t.tick(&net));
        let s = t.state();
        assert_eq!(s.phase, Phase::Connected);
        assert_eq!(s.car.unwrap().host, "192.168.1.60");
    }

    #[test]
    fn goes_lost_while_the_jetson_is_down_and_recovers_on_its_own() {
        let net = Fake::default().agent("192.168.1.50", 8080, "jetson");
        let mut t = Tracker::new(Some(car("192.168.1.50", "jetson")), 8080);
        t.tick(&net);
        assert_eq!(t.state().phase, Phase::Connected);

        net.remove("192.168.1.50", 8080);
        assert!(t.tick(&net));
        assert_eq!(t.state().phase, Phase::Lost);
        assert_eq!(
            t.state().car.unwrap().host,
            "192.168.1.50",
            "the car is kept, not forgotten"
        );

        net.agents
            .lock()
            .unwrap()
            .insert("192.168.1.50:8080".into(), "jetson".into());
        assert!(t.tick(&net));
        assert_eq!(t.state().phase, Phase::Connected);
    }

    #[test]
    fn a_different_car_at_the_old_address_is_not_taken_for_ours() {
        // DHCP gave .50 to another racer. Ours is now at .60.
        let net = Fake {
            mdns: vec![
                cand("192.168.1.50", Source::Mdns),
                cand("192.168.1.60", Source::Mdns),
            ],
            ..Fake::default()
        }
        .agent("192.168.1.50", 8080, "someone-elses-car")
        .agent("192.168.1.60", 8080, "jetson");
        let mut t = Tracker::new(Some(car("192.168.1.50", "jetson")), 8080);
        t.tick(&net);
        let s = t.state();
        assert_eq!(s.phase, Phase::Connected);
        assert_eq!(s.car.unwrap().host, "192.168.1.60");
    }

    #[test]
    fn if_only_a_different_car_is_there_we_stay_lost_rather_than_switch() {
        let net = Fake {
            mdns: vec![cand("192.168.1.50", Source::Mdns)],
            ..Fake::default()
        }
        .agent("192.168.1.50", 8080, "someone-elses-car");
        let mut t = Tracker::new(Some(car("192.168.1.50", "jetson")), 8080);
        t.tick(&net);
        let s = t.state();
        assert_eq!(s.phase, Phase::Lost);
        assert_eq!(s.car.unwrap().hostname, "jetson");
        assert_eq!(s.cars.len(), 1, "the other car is offered in the picker");
    }

    /// A network that makes the user pick a car in the middle of a tick.
    struct Racing<'a> {
        inner: Fake,
        shared: &'a Mutex<Tracker>,
    }

    impl Net for Racing<'_> {
        fn browse_mdns(&self, w: Duration) -> Vec<Candidate> {
            self.inner.browse_mdns(w)
        }
        fn resolve_name(&self, n: &str) -> Vec<String> {
            self.inner.resolve_name(n)
        }
        fn local_ipv4(&self) -> Option<Ipv4Addr> {
            self.inner.local_ipv4()
        }
        fn probe(&self, host: &str, port: u16, t: Duration) -> Option<Health> {
            self.shared
                .lock()
                .unwrap()
                .select(car("192.168.1.99", "chosen"));
            self.inner.probe(host, port, t)
        }
    }

    #[test]
    fn a_user_pick_wins_over_a_tick_that_raced_with_it() {
        let shared = Mutex::new(Tracker::new(Some(car("192.168.1.50", "jetson")), 8080));
        let net = Racing {
            inner: Fake::default().agent("192.168.1.50", 8080, "jetson"),
            shared: &shared,
        };
        assert!(step(&shared, &net).is_none(), "the stale result is dropped");
        assert_eq!(
            shared.lock().unwrap().current().unwrap().host,
            "192.168.1.99"
        );
    }

    #[test]
    fn step_reports_changes_only() {
        let net = Fake::default().agent("192.168.1.50", 8080, "jetson");
        let shared = Mutex::new(Tracker::new(Some(car("192.168.1.50", "jetson")), 8080));
        assert!(
            step(&shared, &net).is_some(),
            "first tick: searching to connected"
        );
        assert!(step(&shared, &net).is_none(), "nothing changed");
    }

    #[test]
    fn clearing_the_car_goes_back_to_searching() {
        let mut t = Tracker::new(Some(car("192.168.1.50", "jetson")), 8080);
        t.clear_current();
        assert!(t.current().is_none());
        assert_eq!(t.state().phase, Phase::Searching);
    }

    #[test]
    fn same_car_prefers_host_name_over_address() {
        assert!(car("1.1.1.1", "x").same_car(&car("2.2.2.2", "x")));
        assert!(!car("1.1.1.1", "x").same_car(&car("1.1.1.1", "y")));
        assert!(car("1.1.1.1", "").same_car(&car("1.1.1.1", "")));
        assert!(!car("1.1.1.1", "").same_car(&car("2.2.2.2", "")));
    }
}
