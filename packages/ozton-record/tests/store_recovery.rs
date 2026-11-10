use std::{
    fs,
    io::Write,
    path::PathBuf,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use ozton_record::{
    frame::Recording,
    routes::RouteIndex,
    selector::RecordTarget,
    store::{Pending, RouteStore},
};

struct Directory(PathBuf);

impl Directory {
    fn new() -> Self {
        let id = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path =
            std::env::temp_dir().join(format!("ozton-route-test-{}-{id}", std::process::id()));
        fs::create_dir(&path).unwrap();
        Self(path)
    }
}

impl Drop for Directory {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn recording(value: f64) -> Recording<f64> {
    let mut recording = Recording::default();
    recording.push_timed(Duration::ZERO, value);
    recording.push_timed(Duration::from_millis(10), value);
    recording
}

#[test]
fn reads_legacy_route_and_recovers_an_interrupted_overwrite() {
    let directory = Directory::new();
    let root = &directory.0;
    let index_path = root.join(RouteIndex::INDEX_PATH);
    let legacy_path = root.join(RouteIndex::path_for(1));
    fs::write(&index_path, "1\tOriginal\n").unwrap();
    recording(0.25).save(&legacy_path).unwrap();
    let old = fs::read(&legacy_path).unwrap();
    let maximum = Duration::from_secs(15);
    let check = |value: &f64| value.is_finite();
    let mut store = RouteStore::open(root).unwrap();
    let mut pending = Pending::new(RecordTarget::Overwrite(1), recording(0.75));

    let index_copy = root.join("old-index");
    fs::rename(&index_path, &index_copy).unwrap();
    fs::create_dir(&index_path).unwrap();
    assert!(store.save(&mut pending, maximum, check).is_err());
    assert_eq!(
        store.load::<f64>(1, maximum, check).unwrap().frames[0].frame,
        0.25
    );
    let prepared = fs::read(root.join(RouteIndex::path_for(2))).unwrap();
    fs::remove_dir(&index_path).unwrap();
    fs::rename(&index_copy, &index_path).unwrap();
    let mut index = fs::OpenOptions::new()
        .append(true)
        .open(&index_path)
        .unwrap();
    index.write_all(b"1\tOriginal\t2\tCOMM").unwrap();
    drop(index);
    let recovered = RouteStore::open(root).unwrap();
    assert_eq!(
        recovered.load::<f64>(1, maximum, check).unwrap().frames[0].frame,
        0.25
    );
    assert_eq!(store.save(&mut pending, maximum, check).unwrap(), 1);
    assert_eq!(fs::read(legacy_path).unwrap(), old);
    assert_eq!(
        fs::read(root.join(RouteIndex::path_for(2))).unwrap(),
        prepared
    );
    let committed = RouteStore::open(root).unwrap();
    assert_eq!(
        committed.load::<f64>(1, maximum, check).unwrap().frames[0].frame,
        0.75
    );
}
