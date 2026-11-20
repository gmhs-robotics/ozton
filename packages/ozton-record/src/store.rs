//! Transactional recording storage over VEXos's append-capable filesystem.

use std::{
    collections::BTreeMap,
    fs::{File, OpenOptions},
    io::{self, Read, Write},
    path::{Path, PathBuf},
    rc::Rc,
    time::Duration,
};

use crate::{
    frame::{
        FrameSchema, Frameable, Recording, RecordingError, archive_header, decode_archive,
        write_archive,
    },
    routes::RouteIndex,
    selector::RecordTarget,
};

const MAX_INDEX_BYTES: u64 = 64 * 1024;
pub(crate) const MAX_RECORDING_BYTES: u64 = 2 * 1024 * 1024;
const MAX_ROUTE_ID: u32 = 4096;
const MAX_ROUTES: usize = 32;
pub(crate) const MAX_FRAMES: usize = 6001;
pub(crate) const MAX_FRAME_GAP: Duration = Duration::from_millis(100);

#[derive(Debug)]
pub enum StoreError {
    Io(io::Error),
    Archive(RecordingError),
    InvalidRecording,
    MissingRoute,
    StorageFull,
    StalePending,
}

impl std::fmt::Display for StoreError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(f, "{error}"),
            Self::Archive(error) => write!(f, "{error}"),
            Self::InvalidRecording => f.write_str("invalid recording"),
            Self::MissingRoute => f.write_str("route does not exist"),
            Self::StorageFull => f.write_str("recording storage is full"),
            Self::StalePending => {
                f.write_str("pending save belongs to another store or route revision")
            }
        }
    }
}

impl std::error::Error for StoreError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Archive(error) => Some(error),
            _ => None,
        }
    }
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

#[derive(Debug, Clone, PartialEq, Eq)]
struct Entry {
    label: String,
    file: u32,
}

struct Prepared {
    owner: Rc<()>,
    id: u32,
    previous: Option<Entry>,
    entry: Entry,
}

pub struct Pending<F: Frameable> {
    target: RecordTarget,
    recording: Recording<F>,
    prepared: Option<Prepared>,
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
    owner: Rc<()>,
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
            owner: Rc::new(()),
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

    /// Append a label update for a committed recording without replacing its file.
    pub fn rename_route(&mut self, route: u32, label: &str) -> Result<(), StoreError> {
        if label.trim().is_empty() || label.contains(['\t', '\n', '\r']) {
            return Err(StoreError::InvalidRecording);
        }
        let entry = self
            .routes
            .get_mut(&route)
            .ok_or(StoreError::MissingRoute)?;
        if entry.label == label {
            return Ok(());
        }
        let mut index = OpenOptions::new()
            .append(true)
            .open(self.root.join(RouteIndex::INDEX_PATH))?;
        let row = journal_append(route, label, entry.file, index.metadata()?.len())?;
        index.write_all(row.as_bytes())?;
        index.sync_all()?;
        entry.label = label.to_owned();
        Ok(())
    }

    pub fn load<F: Frameable + FrameSchema>(
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

    pub fn save<F: Frameable + FrameSchema>(
        &mut self,
        pending: &mut Pending<F>,
        max_duration: Duration,
        valid: impl Fn(&F) -> bool,
    ) -> Result<u32, StoreError> {
        validate(&pending.recording, max_duration, &valid)?;
        let index_path = self.root.join(RouteIndex::INDEX_PATH);
        let index_length = match File::open(&index_path) {
            Ok(index) => index.metadata()?.len(),
            Err(error) if error.kind() == io::ErrorKind::NotFound => 0,
            Err(error) => return Err(error.into()),
        };
        let retrying = pending.prepared.is_some();
        if !retrying {
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
            let header = archive_header::<F>();
            if bytes.len() as u64 + header.len() as u64 > MAX_RECORDING_BYTES {
                return Err(StoreError::InvalidRecording);
            }
            let (file_id, path, mut file) = self.reserve(id, &label, index_length)?;
            let written = write_archive(&mut file, &header, &bytes)
                .and_then(|()| file.sync_all().map_err(RecordingError::Io));
            drop(file);
            if let Err(error) = written {
                let _ = std::fs::remove_file(&path);
                return Err(error.into());
            }
            drop(bytes);
            if let Err(error) = load_file::<F>(&path, max_duration, &valid) {
                let _ = std::fs::remove_file(&path);
                return Err(error);
            }
            pending.prepared = Some(Prepared {
                owner: Rc::clone(&self.owner),
                id,
                previous: self.routes.get(&id).cloned(),
                entry: Entry {
                    label,
                    file: file_id,
                },
            });
        }
        let Prepared {
            owner,
            id,
            previous,
            entry,
        } = pending
            .prepared
            .as_ref()
            .ok_or(StoreError::InvalidRecording)?;
        if !Rc::ptr_eq(owner, &self.owner) {
            return Err(StoreError::StalePending);
        }
        let current = self.routes.get(id);
        if current == Some(entry) {
            return Ok(*id);
        }
        if current != previous.as_ref() {
            return Err(StoreError::StalePending);
        }
        if retrying {
            load_file::<F>(
                &self.root.join(RouteIndex::path_for(entry.file)),
                max_duration,
                &valid,
            )?;
        }
        let row = journal_append(*id, &entry.label, entry.file, index_length)?;
        let mut index = OpenOptions::new()
            .append(true)
            .create(true)
            .open(index_path)?;
        index.write_all(row.as_bytes())?;
        index.sync_all()?;
        self.routes.insert(*id, entry.clone());
        Ok(*id)
    }

    fn reserve(
        &mut self,
        route: u32,
        label: &str,
        index_length: u64,
    ) -> Result<(u32, PathBuf, File), StoreError> {
        for file_id in self.next_file..=MAX_ROUTE_ID {
            journal_append(route, label, file_id, index_length)?;
            let path = self.root.join(RouteIndex::path_for(file_id));
            match OpenOptions::new().write(true).create_new(true).open(&path) {
                Ok(file) => {
                    self.next_file = file_id + 1;
                    return Ok((file_id, path, file));
                }
                Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {}
                Err(error) => return Err(error.into()),
            }
        }
        Err(StoreError::StorageFull)
    }
}

fn journal_append(id: u32, label: &str, file: u32, length: u64) -> Result<String, StoreError> {
    // The tab invalidates an unterminated row, even if its COMMIT marker was fully written.
    let row = format!("\t\n{}", commit_row(id, label, file));
    if length.saturating_add(row.len() as u64) > MAX_INDEX_BYTES {
        return Err(StoreError::StorageFull);
    }
    Ok(row)
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

fn load_file<F: Frameable + FrameSchema>(
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
    let recording = decode_archive::<F>(&bytes)?;
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
        || recording.frames.iter().any(|timed| {
            timed.delta_time_micros > MAX_FRAME_GAP.as_micros() as u64 || !valid(&timed.frame)
        })
    {
        return Err(StoreError::InvalidRecording);
    }
    Ok(())
}
