//! The last-used car, as a small JSON file in the app's data folder. Not secret: an address and
//! a host name. The token is separate and lives in the keychain (`token.rs`).

use crate::discovery::{Car, Source};
use std::fs;
use std::path::Path;

const FILE: &str = "last_car.json";

pub fn load(dir: &Path) -> Option<Car> {
    let text = fs::read_to_string(dir.join(FILE)).ok()?;
    let mut car: Car = serde_json::from_str(&text).ok()?;
    car.source = Source::Saved;
    Some(car)
}

pub fn save(dir: &Path, car: &Car) -> std::io::Result<()> {
    fs::create_dir_all(dir)?;
    let text = serde_json::to_string_pretty(car).map_err(std::io::Error::other)?;
    // Write then rename so a crash never leaves half a file.
    let tmp = dir.join(format!("{FILE}.tmp"));
    fs::write(&tmp, text)?;
    fs::rename(tmp, dir.join(FILE))
}

pub fn forget(dir: &Path) {
    let _ = fs::remove_file(dir.join(FILE));
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_dir(name: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("racer-store-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&d);
        d
    }

    #[test]
    fn saves_and_loads_the_last_car() {
        let d = temp_dir("roundtrip");
        let car = Car {
            host: "192.168.1.50".into(),
            port: 8080,
            hostname: "jetson".into(),
            source: Source::Mdns,
        };
        save(&d, &car).unwrap();
        let back = load(&d).unwrap();
        assert_eq!(back.host, "192.168.1.50");
        assert_eq!(back.hostname, "jetson");
        assert_eq!(
            back.source,
            Source::Saved,
            "a loaded car is marked as saved"
        );
        forget(&d);
        assert!(load(&d).is_none());
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn a_missing_or_corrupt_file_is_just_no_saved_car() {
        let d = temp_dir("corrupt");
        assert!(load(&d).is_none());
        fs::create_dir_all(&d).unwrap();
        fs::write(d.join(FILE), "{not json").unwrap();
        assert!(load(&d).is_none());
        let _ = fs::remove_dir_all(&d);
    }
}
