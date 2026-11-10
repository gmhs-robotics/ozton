//! Transactional recording storage over VEXos's append-capable filesystem.

use std::{
    collections::BTreeMap,
    fs::{File, OpenOptions},
    io::{self, Read, Write},
    path::{Path, PathBuf},
    time::Duration,
};

use crate::{
    frame::{Frameable, Recording, RecordingError},
    routes::RouteIndex,
    selector::RecordTarget,
};

const MAX_INDEX_BYTES: u64 = 64 * 1024;
const MAX_RECORDING_BYTES: u64 = 2 * 1024 * 1024;
const MAX_ROUTE_ID: u32 = 4096;
const MAX_ROUTES: usize = 32;
pub(crate) const MAX_FRAMES: usize = 6001;

#[derive(Debug)]
pub enum StoreError {
    Io(io::Error),
    Archive(RecordingError),
    InvalidRecording,
    MissingRoute,
    StorageFull,
}

impl From<io::Error> for StoreError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

impl From<RecordingError> for StoreError {
    fn from(error: RecordingError) -> Self {
        Self::Archive(error)
    }
}

#[derive(Debug, Clone)]
struct Entry {
    label: String,
    file: u32,
}

pub struct Pending<F: Frameable> {
    target: RecordTarget,
    recording: Recording<F>,
    prepared: Option<(u32, Entry)>,
}

impl<F: Frameable> Pending<F> {
    pub fn new(target: RecordTarget, recording: Recording<F>) -> Self {
        Self {
            target,
            recording,
            prepared: None,
        }
    }
}

/// Immutable recording files plus an append-only index. One owner writes a store.
pub struct RouteStore {
    root: PathBuf,
    routes: BTreeMap<u32, Entry>,
    next_file: u32,
}

impl RouteStore {
    pub fn open(root: impl Into<PathBuf>) -> Result<Self, StoreError> {
        let root = root.into();
        let mut routes = BTreeMap::new();
        let mut next_file = 1;
        match File::open(root.join(RouteIndex::INDEX_PATH)) {
            Ok(file) => {
                let mut bytes = Vec::new();
                file.take(MAX_INDEX_BYTES + 1).read_to_end(&mut bytes)?;
                if bytes.len() as u64 > MAX_INDEX_BYTES {
                    return Err(StoreError::StorageFull);
                }
                for line in bytes.split_inclusive(|byte| *byte == b'\n') {
                    if !line.ends_with(b"\n") {
                        continue;
                    }
                    let Ok(line) = std::str::from_utf8(line) else {
                        continue;
                    };
                    let Some((id, label, file)) = index_row(line) else {
                        continue;
                    };
                    if !routes.contains_key(&id) && routes.len() == MAX_ROUTES {
                        continue;
                    }
                    if file <= MAX_ROUTE_ID {
                        next_file = next_file.max(file + 1);
                    }
                    routes.insert(
                        id,
                        Entry {
                            label: label.to_owned(),
                            file,
                        },
                    );
                }
            }
            Err(error) if error.kind() == io::ErrorKind::NotFound => {}
            Err(error) => return Err(error.into()),
        }
        Ok(Self {
            root,
            routes,
            next_file,
        })
    }

    pub fn iter(&self) -> impl Iterator<Item = (u32, &str)> {
        self.routes
            .iter()
            .map(|(&id, entry)| (id, entry.label.as_str()))
    }

    pub(crate) fn file_id(&self, route: u32) -> Option<u32> {
        self.routes.get(&route).map(|entry| entry.file)
    }

    pub fn load<F: Frameable>(
        &self,
        id: u32,
        max_duration: Duration,
        valid: impl Fn(&F) -> bool,
    ) -> Result<Recording<F>, StoreError> {
        let entry = self.routes.get(&id).ok_or(StoreError::MissingRoute)?;
        load_file(
            &self.root.join(RouteIndex::path_for(entry.file)),
            max_duration,
            valid,
        )
    }

    pub fn save<F: Frameable>(
        &mut self,
        pending: &mut Pending<F>,
        max_duration: Duration,
        valid: impl Fn(&F) -> bool,
    ) -> Result<u32, StoreError> {
        validate(&pending.recording, max_duration, &valid)?;
        if pending.prepared.is_none() {
            let (id, label) = match pending.target {
                RecordTarget::Off => return Err(StoreError::InvalidRecording),
                RecordTarget::New => {
                    if self.routes.len() == MAX_ROUTES {
                        return Err(StoreError::StorageFull);
                    }
                    let id = (1..=MAX_ROUTE_ID)
                        .find(|id| !self.routes.contains_key(id))
                        .ok_or(StoreError::StorageFull)?;
                    (id, format!("Route {id}"))
                }
                RecordTarget::Overwrite(id) => (
                    id,
                    self.routes
                        .get(&id)
                        .ok_or(StoreError::MissingRoute)?
                        .label
                        .clone(),
                ),
            };
            if label.contains(['\t', '\n', '\r']) {
                return Err(StoreError::InvalidRecording);
            }
            let bytes = rkyv::to_bytes::<rkyv::rancor::Error>(&pending.recording)
                .map_err(RecordingError::Rkyv)?;
            if bytes.len() as u64 > MAX_RECORDING_BYTES {
                return Err(StoreError::InvalidRecording);
            }
            let (file_id, path, mut file) = self.reserve()?;
            file.write_all(&bytes)?;
            file.sync_all()?;
            drop(file);
            load_file::<F>(&path, max_duration, &valid)?;
            pending.prepared = Some((
                id,
                Entry {
                    label,
                    file: file_id,
                },
            ));
        }
        let (id, entry) = pending
            .prepared
            .as_ref()
            .ok_or(StoreError::InvalidRecording)?;
        if self
            .routes
            .get(id)
            .is_some_and(|current| current.file == entry.file)
        {
            return Ok(*id);
        }
        // A leading newline isolates an interrupted previous append.
        let row = format!("\n{}", commit_row(*id, &entry.label, entry.file));
        let index_path = self.root.join(RouteIndex::INDEX_PATH);
        let length = match File::open(&index_path) {
            Ok(index) => index.metadata()?.len(),
            Err(error) if error.kind() == io::ErrorKind::NotFound => 0,
            Err(error) => return Err(error.into()),
        };
        if length.saturating_add(row.len() as u64) > MAX_INDEX_BYTES {
            return Err(StoreError::StorageFull);
        }
        let mut index = OpenOptions::new()
            .append(true)
            .create(true)
            .open(index_path)?;
        index.write_all(row.as_bytes())?;
        index.sync_all()?;
        self.routes.insert(*id, entry.clone());
        Ok(*id)
    }

    fn reserve(&mut self) -> Result<(u32, PathBuf, File), StoreError> {
        for id in self.next_file..=MAX_ROUTE_ID {
            let path = self.root.join(RouteIndex::path_for(id));
            match OpenOptions::new().write(true).create_new(true).open(&path) {
                Ok(file) => {
                    self.next_file = id + 1;
                    return Ok((id, path, file));
                }
                Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {}
                Err(error) => return Err(error.into()),
            }
        }
        Err(StoreError::StorageFull)
    }
}

pub(crate) fn commit_row(id: u32, label: &str, file: u32) -> String {
    format!("V1\t{id}\t{file}\t{label}\tCOMMIT\n")
}

fn index_row(line: &str) -> Option<(u32, &str, u32)> {
    let mut fields = line.trim_end_matches(['\r', '\n']).split('\t');
    let first = fields.next()?;
    let positive = |value: &str| value.trim().parse::<u32>().ok().filter(|id| *id > 0);
    let (id, label, file) = if first == "V1" {
        let id = positive(fields.next()?)?;
        let file = positive(fields.next()?)?;
        let label = fields.next()?;
        if fields.next() != Some("COMMIT") {
            return None;
        }
        (id, label, file)
    } else {
        let id = positive(first)?;
        let label = fields.next().unwrap_or(first);
        let file = if let Some(file) = fields.next() {
            if fields.next() != Some("COMMIT") {
                return None;
            }
            positive(file)?
        } else {
            id
        };
        (id, label, file)
    };
    let label = label.trim();
    (!label.is_empty() && fields.next().is_none()).then_some((id, label, file))
}

fn load_file<F: Frameable>(
    path: &Path,
    max_duration: Duration,
    valid: impl Fn(&F) -> bool,
) -> Result<Recording<F>, StoreError> {
    let file = File::open(path)?;
    let length = file.metadata()?.len();
    if length == 0 || length > MAX_RECORDING_BYTES {
        return Err(StoreError::InvalidRecording);
    }
    let mut bytes = rkyv::util::AlignedVec::<16>::with_capacity(length as usize);
    let mut reader = file.take(MAX_RECORDING_BYTES + 1);
    let mut buffer = [0; 1024];
    loop {
        let count = reader.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        bytes.extend_from_slice(&buffer[..count]);
    }
    if bytes.len() as u64 != length {
        return Err(StoreError::InvalidRecording);
    }
    let recording = rkyv::from_bytes::<Recording<F>, rkyv::rancor::Error>(&bytes)
        .map_err(RecordingError::Rkyv)?;
    validate(&recording, max_duration, valid)?;
    Ok(recording)
}

fn validate<F: Frameable>(
    recording: &Recording<F>,
    max_duration: Duration,
    valid: impl Fn(&F) -> bool,
) -> Result<(), StoreError> {
    if recording.frames.is_empty()
        || recording.frames.len() > MAX_FRAMES
        || recording.duration() > max_duration
        || recording.frames.iter().any(|timed| !valid(&timed.frame))
    {
        return Err(StoreError::InvalidRecording);
    }
    Ok(())
}
