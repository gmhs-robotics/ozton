//! Allocate a new run file and persist its high-water mark.

use std::{
    fs::{self, File, OpenOptions},
    io::{self, Write},
    path::Path,
};

const INDEX: &str = "index.log";
const BACKUP: &str = "index.bak";

fn read_counter(path: &Path) -> io::Result<Option<u64>> {
    match fs::read_to_string(path) {
        Ok(value) => value
            .trim()
            .parse()
            .map(Some)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "invalid SD log index")),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(error),
    }
}

fn write_counter(path: &Path, highest: u64) -> io::Result<()> {
    let mut counter = File::create(path)?;
    writeln!(counter, "{highest}")?;
    counter.sync_all()
}

pub(super) fn open_log(directory: &Path) -> io::Result<(String, File, Option<io::Error>)> {
    let index = directory.join(INDEX);
    let backup = directory.join(BACKUP);
    let primary = read_counter(&index);
    let mirror = read_counter(&backup);
    let primary_value = primary.as_ref().ok().and_then(|value| *value);
    let mirror_value = mirror.as_ref().ok().and_then(|value| *value);
    let primary_is_latest = primary_value >= mirror_value;
    let mut highest = match (primary, mirror) {
        (Ok(a), Ok(b)) => a.into_iter().chain(b).max().unwrap_or(0),
        (Ok(Some(value)), Err(_)) | (Err(_), Ok(Some(value))) => value,
        (Err(error), _) | (_, Err(error)) => return Err(error),
    };
    loop {
        highest = highest
            .checked_add(1)
            .ok_or_else(|| io::Error::other("SD log index exhausted"))?;
        let name = format!("{highest}.log");
        // VEXos checks create and create_new independently: create_new alone
        // rejects missing files. Set both, retaining the existing-file check.
        let mut file = match OpenOptions::new()
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
        // Persist the session before advancing either counter. A numbered
        // file is never reused, even if publishing its reservation fails.
        file.write_all(super::SESSION)?;
        file.sync_all()?;
        // Update the older or invalid copy first. Never risk the latest
        // valid counter until another copy has been written.
        let (first, second) = if primary_is_latest {
            (&backup, &index)
        } else {
            (&index, &backup)
        };
        let index_error = write_counter(first, highest)
            .and_then(|()| write_counter(second, highest))
            .err();
        return Ok((name, file, index_error));
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
        let (first, mut file, error) = open_log(&dir.0).unwrap();
        assert_eq!(first, "1.log");
        assert!(error.is_none());
        assert_eq!(fs::read(dir.0.join(&first)).unwrap(), super::super::SESSION);
        file.write_all(b"first run").unwrap();
        drop(file);
        let (second, file, _) = open_log(&dir.0).unwrap();
        assert_eq!(second, "2.log");
        drop(file);
        fs::remove_file(dir.0.join(second)).unwrap();
        let (third, _, _) = open_log(&dir.0).unwrap();
        assert_eq!(third, "3.log");
        assert_eq!(
            fs::read(dir.0.join(first)).unwrap(),
            [super::super::SESSION, b"first run"].concat()
        );
        assert_eq!(fs::read_to_string(dir.0.join(INDEX)).unwrap(), "3\n");
    }

    #[test]
    fn stale_index_skips_occupied_names_without_overwriting() {
        let dir = TempDir::new();
        fs::write(dir.0.join(INDEX), "1\n").unwrap();
        fs::write(dir.0.join("2.log"), "previous run").unwrap();
        let (name, _, _) = open_log(&dir.0).unwrap();
        assert_eq!(name, "3.log");
        assert_eq!(
            fs::read_to_string(dir.0.join("2.log")).unwrap(),
            "previous run"
        );
    }

    #[test]
    fn backup_recovers_a_torn_index_without_reusing_deleted_numbers() {
        let dir = TempDir::new();
        fs::write(dir.0.join(INDEX), "invalid").unwrap();
        fs::write(dir.0.join(BACKUP), "7\n").unwrap();
        let (name, _, error) = open_log(&dir.0).unwrap();
        assert_eq!(name, "8.log");
        assert!(error.is_none());
        assert_eq!(fs::read_to_string(dir.0.join(INDEX)).unwrap(), "8\n");
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
