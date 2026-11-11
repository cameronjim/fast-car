//! The agent's access token, kept in the operating system's credential store: Keychain on macOS,
//! Credential Manager on Windows, the Secret Service (GNOME Keyring, KWallet) on Linux.
//!
//! Trade-off, on purpose: the UI process reads the token over IPC and attaches it to requests
//! (the web UI's `Api` class does that in both builds), so a compromised page could read it.
//! The page only ever loads the bundled UI, the capability file grants no filesystem or shell
//! access, and the agent itself is LAN only with a per-IP lockout. What the keychain buys over
//! localStorage is that the token is not a plain file in the app's profile folder. On a Linux
//! machine with no Secret Service the commands fail and the UI keeps the token in memory for
//! the session and asks again next launch.

pub const SERVICE: &str = "com.cameronjim.racer";
pub const ACCOUNT: &str = "agent-token";

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

/// Reject what can never be a token before it reaches the keychain.
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

pub fn store(secrets: &dyn Secrets, token: &str) -> Result<(), String> {
    secrets.set(validate(token)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    #[derive(Default)]
    struct Memory(Mutex<Option<String>>);

    impl Secrets for Memory {
        fn get(&self) -> Result<Option<String>, String> {
            Ok(self.0.lock().unwrap().clone())
        }
        fn set(&self, v: &str) -> Result<(), String> {
            *self.0.lock().unwrap() = Some(v.to_string());
            Ok(())
        }
        fn clear(&self) -> Result<(), String> {
            *self.0.lock().unwrap() = None;
            Ok(())
        }
    }

    #[test]
    fn round_trips_and_clears() {
        let m = Memory::default();
        assert_eq!(m.get().unwrap(), None);
        store(&m, "  abc123  ").unwrap();
        assert_eq!(
            m.get().unwrap().as_deref(),
            Some("abc123"),
            "stored trimmed"
        );
        m.clear().unwrap();
        assert_eq!(m.get().unwrap(), None);
    }

    #[test]
    fn refuses_things_that_are_not_tokens() {
        let m = Memory::default();
        for bad in ["", "   ", "a b", "tab\there", "new\nline"] {
            assert!(store(&m, bad).is_err(), "{bad:?}");
        }
        assert!(store(&m, &"x".repeat(513)).is_err());
        assert_eq!(m.get().unwrap(), None, "nothing was written");
    }
}
