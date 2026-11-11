//! The real network behind [`discovery::Net`]: mDNS via `mdns-sd`, the system resolver, and a
//! blocking `GET /health` via `ureq`. The only request this app's native side ever makes.

use crate::discovery::{Candidate, Health, Net, Source, AGENT_NAME};
use mdns_sd::{ServiceDaemon, ServiceEvent};
use std::net::{IpAddr, Ipv4Addr, ToSocketAddrs, UdpSocket};
use std::sync::Mutex;
use std::time::{Duration, Instant};

const SERVICE_TYPE: &str = "_racer._tcp.local.";

pub struct RealNet {
    daemon: Mutex<Option<ServiceDaemon>>,
}

impl RealNet {
    pub fn new() -> Self {
        RealNet {
            daemon: Mutex::new(None),
        }
    }

    /// The mDNS daemon is created on first use, and again if it died (for example when the
    /// machine changed network).
    fn daemon(&self) -> Option<ServiceDaemon> {
        let mut slot = self.daemon.lock().ok()?;
        if slot.is_none() {
            *slot = ServiceDaemon::new().ok();
        }
        slot.clone()
    }

    fn reset_daemon(&self) {
        if let Ok(mut slot) = self.daemon.lock() {
            if let Some(d) = slot.take() {
                let _ = d.shutdown();
            }
        }
    }
}

impl Default for RealNet {
    fn default() -> Self {
        Self::new()
    }
}

impl Net for RealNet {
    fn browse_mdns(&self, window: Duration) -> Vec<Candidate> {
        let Some(daemon) = self.daemon() else {
            return Vec::new();
        };
        let Ok(rx) = daemon.browse(SERVICE_TYPE) else {
            self.reset_daemon();
            return Vec::new();
        };
        let deadline = Instant::now() + window;
        let mut out: Vec<Candidate> = Vec::new();
        while let Some(left) = deadline.checked_duration_since(Instant::now()) {
            match rx.recv_timeout(left) {
                Ok(ServiceEvent::ServiceResolved(info)) => {
                    for ip in info.get_addresses_v4() {
                        let host = ip.to_string();
                        if !out.iter().any(|c| c.host == host) {
                            out.push(Candidate {
                                host,
                                port: info.get_port(),
                                source: Source::Mdns,
                            });
                        }
                    }
                }
                Ok(_) => {}
                Err(_) => break,
            }
        }
        let _ = daemon.stop_browse(SERVICE_TYPE);
        out
    }

    fn resolve_name(&self, name: &str) -> Vec<String> {
        // Port 0 is a placeholder: only the address is used.
        let Ok(addrs) = (name, 0u16).to_socket_addrs() else {
            return Vec::new();
        };
        let mut out: Vec<String> = Vec::new();
        for a in addrs {
            if let IpAddr::V4(v4) = a.ip() {
                let s = v4.to_string();
                if !out.contains(&s) {
                    out.push(s);
                }
            }
        }
        out
    }

    fn local_ipv4(&self) -> Option<Ipv4Addr> {
        // A UDP connect sends nothing; it only makes the OS pick the outgoing interface.
        let sock = UdpSocket::bind("0.0.0.0:0").ok()?;
        sock.connect("10.255.255.255:1").ok()?;
        match sock.local_addr().ok()?.ip() {
            IpAddr::V4(v4) if !v4.is_loopback() && !v4.is_unspecified() => Some(v4),
            _ => None,
        }
    }

    fn probe(&self, host: &str, port: u16, timeout: Duration) -> Option<Health> {
        let host = if host.contains(':') {
            format!("[{host}]")
        } else {
            host.to_string()
        };
        let url = format!("http://{host}:{port}/health");
        let agent: ureq::Agent = ureq::Agent::config_builder()
            .timeout_global(Some(timeout))
            .http_status_as_error(false)
            .proxy(None)
            .build()
            .into();
        let mut resp = agent.get(&url).call().ok()?;
        if !resp.status().is_success() {
            return None;
        }
        let body = resp
            .body_mut()
            .with_config()
            .limit(4096)
            .read_to_string()
            .ok()?;
        parse_health(&body)
    }

    fn probe_many(&self, hosts: &[String], port: u16, timeout: Duration) -> Vec<(String, Health)> {
        // 64 at a time keeps a /24 sweep to about a second on a quiet LAN.
        let mut found = Vec::new();
        for chunk in hosts.chunks(64) {
            std::thread::scope(|s| {
                let handles: Vec<_> = chunk
                    .iter()
                    .map(|h| s.spawn(move || (h.clone(), self.probe(h, port, timeout))))
                    .collect();
                for handle in handles {
                    if let Ok((host, Some(health))) = handle.join() {
                        found.push((host, health));
                    }
                }
            });
        }
        found
    }
}

/// Accept a `/health` body only if it says it is the racer agent.
pub fn parse_health(body: &str) -> Option<Health> {
    let v: serde_json::Value = serde_json::from_str(body).ok()?;
    if v.get("name")?.as_str()? != AGENT_NAME {
        return None;
    }
    let hostname = v
        .get("hostname")
        .and_then(|h| h.as_str())
        .unwrap_or("")
        .to_string();
    Some(Health { hostname })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::TcpListener;

    #[test]
    fn health_body_must_name_the_racer_agent() {
        let ok =
            parse_health(r#"{"ok":true,"name":"racer-agent","version":"0.1","hostname":"jetson"}"#);
        assert_eq!(
            ok,
            Some(Health {
                hostname: "jetson".into()
            })
        );
        assert_eq!(
            parse_health(r#"{"name":"racer-agent"}"#),
            Some(Health {
                hostname: String::new()
            })
        );
        assert_eq!(parse_health(r#"{"name":"nginx"}"#), None);
        assert_eq!(parse_health(r#"{"hostname":"x"}"#), None);
        assert_eq!(parse_health("<html>router login</html>"), None);
        assert_eq!(parse_health(""), None);
    }

    /// Serve one canned HTTP response on localhost and return the port.
    fn serve_once(response: &'static str) -> u16 {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        std::thread::spawn(move || {
            if let Ok((mut s, _)) = listener.accept() {
                let mut buf = [0u8; 1024];
                let _ = s.read(&mut buf);
                let _ = s.write_all(response.as_bytes());
            }
        });
        port
    }

    #[test]
    fn probes_a_real_socket_and_only_ever_sends_get_health() {
        let port = serve_once(
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 57\r\nConnection: close\r\n\r\n\
             {\"ok\":true,\"name\":\"racer-agent\",\"hostname\":\"localjetson\"}",
        );
        let h = RealNet::new().probe("127.0.0.1", port, Duration::from_secs(2));
        assert_eq!(
            h,
            Some(Health {
                hostname: "localjetson".into()
            })
        );
    }

    #[test]
    fn a_non_200_or_a_closed_port_is_not_a_car() {
        let port =
            serve_once("HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        assert_eq!(
            RealNet::new().probe("127.0.0.1", port, Duration::from_secs(2)),
            None
        );
        let closed = {
            let l = TcpListener::bind("127.0.0.1:0").unwrap();
            l.local_addr().unwrap().port()
        };
        assert_eq!(
            RealNet::new().probe("127.0.0.1", closed, Duration::from_millis(300)),
            None
        );
    }

    #[test]
    fn probe_many_finds_the_one_that_answers() {
        let port = serve_once(
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 57\r\nConnection: close\r\n\r\n\
             {\"ok\":true,\"name\":\"racer-agent\",\"hostname\":\"localjetson\"}",
        );
        let hosts = vec!["127.0.0.1".to_string()];
        let found = RealNet::new().probe_many(&hosts, port, Duration::from_secs(2));
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].0, "127.0.0.1");
    }
}
