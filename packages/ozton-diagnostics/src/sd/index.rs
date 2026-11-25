//! Allocate a new run file and persist its high-water mark.

use std::{
    fs::{self, File, OpenOptions},
    io::{self, Write},
    path::Path,
};

const INDEX: &str = "index.log";

pub(super) fn open_log(directory: &Path) -> io::Result<(String, File)> {
    let index = directory.join(INDEX);
    let mut highest: u64 = match fs::read_to_string(&index) {
        Ok(value) => value
            .trim()
            .parse()
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "invalid SD log index"))?,
        Err(error) if error.kind() == io::ErrorKind::NotFound => 0,
        Err(error) => return Err(error),
    };
    loop {
        highest = highest
            .checked_add(1)
            .ok_or_else(|| io::Error::other("SD log index exhausted"))?;
        let name = format!("{highest}.log");
        // VEXos checks create and create_new independently: create_new alone
        // rejects missing files. Set both, retaining the existing-file check.
        let file = match OpenOptions::new()
            .write(true)
            .create(true)
            .create_new(true)
            .truncate(true)
            .open(directory.join(&name))
        {
            Ok(file) => file,
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(error),
        };
        // Publish the reservation before recording any telemetry. A failed
        // update leaves an empty numbered file, which is never overwritten.
        let mut counter = File::create(&index)?;
        writeln!(counter, "{highest}")?;
        counter.sync_all()?;
        return Ok((name, file));
    }
}

#[cfg(test)]
mod tests {
    use std::{
        path::PathBuf,
        sync::atomic::{AtomicUsize, Ordering},
    };

    use super::*;

    struct TempDir(PathBuf);

    impl TempDir {
        fn new() -> Self {
            static NEXT: AtomicUsize = AtomicUsize::new(0);
            let path = std::env::temp_dir().join(format!(
                "ozton-sd-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            fs::create_dir(&path).unwrap();
            Self(path)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn each_startup_allocates_a_new_file_even_after_deletion() {
        let dir = TempDir::new();
        let (first, mut file) = open_log(&dir.0).unwrap();
        assert_eq!(first, "1.log");
        file.write_all(b"first run").unwrap();
        drop(file);
        let (second, file) = open_log(&dir.0).unwrap();
        assert_eq!(second, "2.log");
        drop(file);
        fs::remove_file(dir.0.join(second)).unwrap();
        let (third, _) = open_log(&dir.0).unwrap();
        assert_eq!(third, "3.log");
        assert_eq!(fs::read(dir.0.join(first)).unwrap(), b"first run");
        assert_eq!(fs::read_to_string(dir.0.join(INDEX)).unwrap(), "3\n");
    }

    #[test]
    fn stale_index_skips_occupied_names_without_overwriting() {
        let dir = TempDir::new();
        fs::write(dir.0.join(INDEX), "1\n").unwrap();
        fs::write(dir.0.join("2.log"), "previous run").unwrap();
        let (name, _) = open_log(&dir.0).unwrap();
        assert_eq!(name, "3.log");
        assert_eq!(
            fs::read_to_string(dir.0.join("2.log")).unwrap(),
            "previous run"
        );
    }

    #[test]
    fn corrupt_index_fails_without_touching_existing_logs() {
        let dir = TempDir::new();
        fs::write(dir.0.join(INDEX), "invalid").unwrap();
        fs::write(dir.0.join("1.log"), "previous run").unwrap();
        assert_eq!(
            open_log(&dir.0).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
        assert_eq!(
            fs::read_to_string(dir.0.join("1.log")).unwrap(),
            "previous run"
        );
    }
}
