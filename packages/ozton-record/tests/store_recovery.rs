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
    let legacy =
        ozton_record::rkyv::to_bytes::<ozton_record::rkyv::rancor::Error>(&recording(0.25))
            .unwrap();
    fs::write(&legacy_path, &legacy).unwrap();
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

#[test]
fn saving_another_route_does_not_commit_an_unterminated_overwrite() {
    let directory = Directory::new();
    let root = &directory.0;
    let index_path = root.join(RouteIndex::INDEX_PATH);
    fs::write(&index_path, "1\tOriginal\nV1\t1\t2\tOriginal\tCOMMIT").unwrap();
    recording(0.25)
        .save(root.join(RouteIndex::path_for(1)))
        .unwrap();
    recording(0.75)
        .save(root.join(RouteIndex::path_for(2)))
        .unwrap();
    let maximum = Duration::from_secs(15);
    let check = |value: &f64| value.is_finite();
    let mut recovered = RouteStore::open(root).unwrap();
    let mut pending = Pending::new(RecordTarget::New, recording(0.5));
    let new = recovered.save(&mut pending, maximum, check).unwrap();

    let reopened = RouteStore::open(root).unwrap();
    assert_eq!(
        reopened.load::<f64>(1, maximum, check).unwrap().frames[0].frame,
        0.25
    );
    assert_eq!(
        reopened.load::<f64>(new, maximum, check).unwrap().frames[0].frame,
        0.5
    );
}

#[test]
fn versioned_recording_rejects_an_equally_sized_different_frame() {
    let directory = Directory::new();
    let path = directory.0.join("schema.route");
    recording(1.0).save(&path).unwrap();
    assert!(matches!(
        Recording::<u64>::load(&path),
        Err(ozton_record::frame::RecordingError::IncompatibleSchema),
    ));
}

#[test]
fn a_prepared_save_cannot_be_retried_against_another_store() {
    let first = Directory::new();
    let second = Directory::new();
    let mut source = RouteStore::open(&first.0).unwrap();
    let mut destination = RouteStore::open(&second.0).unwrap();
    let index = first.0.join(RouteIndex::INDEX_PATH);
    fs::create_dir(&index).unwrap();
    let mut pending = Pending::new(RecordTarget::New, recording(0.5));
    let maximum = Duration::from_secs(1);
    let valid = |value: &f64| value.is_finite();

    assert!(source.save(&mut pending, maximum, valid).is_err());
    assert!(first.0.join(RouteIndex::path_for(1)).is_file());
    assert!(matches!(
        destination.save(&mut pending, maximum, valid),
        Err(ozton_record::store::StoreError::StalePending),
    ));
    assert_eq!(destination.iter().count(), 0);
    fs::remove_dir(&index).unwrap();
    assert_eq!(source.save(&mut pending, maximum, valid).unwrap(), 1);
}
