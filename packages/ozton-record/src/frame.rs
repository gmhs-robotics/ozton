use std::{
    path::Path,
    time::{Duration, Instant},
};

use async_trait::async_trait;
use rkyv::{
    Archive, Deserialize, Serialize,
    api::high::{HighDeserializer, HighSerializer, HighValidator},
    bytecheck::CheckBytes,
    from_bytes,
    rancor::Error,
    ser::allocator::ArenaHandle,
    to_bytes,
    util::AlignedVec,
};
use vexide::{smart::PortError, time::sleep_until};

use crate::store::{MAX_FRAME_GAP, MAX_RECORDING_BYTES};

#[derive(Archive, Serialize, Deserialize, Default, Clone, Debug)]
pub struct TimedFrame<F: Frameable> {
    pub delta_time_micros: u64,
    pub frame: F,
}

#[derive(Archive, Serialize, Deserialize, Default, Clone, Debug)]
pub struct Recording<F: Frameable> {
    pub frames: Vec<TimedFrame<F>>,
}

#[derive(Debug)]
pub enum RecordingError {
    Io(std::io::Error),
    Rkyv(Error),
    InvalidSize,
    IncompatibleSchema,
}

impl std::fmt::Display for RecordingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RecordingError::Io(e) => write!(f, "I/O error: {e}"),
            RecordingError::Rkyv(e) => write!(f, "rkyv error: {e}"),
            RecordingError::InvalidSize => f.write_str("recording exceeds its size limit"),
            RecordingError::IncompatibleSchema => {
                f.write_str("incompatible recording schema or format version")
            }
        }
    }
}

impl std::error::Error for RecordingError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Rkyv(error) => Some(error),
            Self::InvalidSize | Self::IncompatibleSchema => None,
        }
    }
}

pub(crate) fn recording_interval(interval: Duration) -> Duration {
    interval.clamp(Duration::from_millis(1), MAX_FRAME_GAP)
}

impl From<std::io::Error> for RecordingError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

impl From<Error> for RecordingError {
    fn from(e: Error) -> Self {
        Self::Rkyv(e)
    }
}

/// Selects how a frame should be applied to a device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RecordMode {
    /// Apply the just-sampled driver command directly to the robot.
    Live,
    /// Apply a previously recorded frame during autonomous playback.
    Playback,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaybackOutcome {
    Completed,
    TrackingLost,
    OutputFailed,
}

#[derive(Debug)]
pub enum PlaybackError {
    Device(PortError),
    TrackingUnavailable,
    InvalidCorrection,
}

impl From<PortError> for PlaybackError {
    fn from(error: PortError) -> Self {
        Self::Device(error)
    }
}

/// A robot whose device fields know how to finalize and apply generated frames.
#[async_trait(?Send)]
pub trait FrameRobot {
    type Frame: Frameable + FrameSchema;

    /// Enriches a sampled frame before it is persisted to disk.
    ///
    /// Most fields simply clone the sampled value. Tracked drivetrains can use this hook to attach
    /// odometry state needed for corrected playback.
    async fn finalize_frame(&self, frame: &Self::Frame) -> Self::Frame;

    /// Applies a frame to the live robot.
    async fn apply_frame(&mut self, frame: &Self::Frame, mode: RecordMode)
    -> Result<(), PortError>;

    /// Checks all derived device fields before a frame is captured or played.
    fn devices_ready(&self) -> bool {
        true
    }

    async fn apply_playback_frame(&mut self, frame: &Self::Frame) -> Result<(), PlaybackError> {
        self.apply_frame(frame, RecordMode::Playback)
            .await
            .map_err(Into::into)
    }

    /// Resets any motor outputs that should be stopped after playback completes.
    async fn stop_playback(&mut self) -> Result<(), PortError>;
}

#[async_trait(?Send)]
pub trait Recordable: FrameRobot {
    const UPDATE_INTERVAL: Duration;
    const MAX_CAPTURE_DURATION: Duration = Duration::from_secs(15);

    /// Reject application-specific invalid values before capture or playback.
    fn valid_frame(_frame: &Self::Frame) -> bool {
        true
    }

    async fn get_new_frame(&self) -> Self::Frame;

    /// Whether localization is valid enough to record or replay a corrected route.
    fn playback_ready(&self) -> bool {
        true
    }

    /// Immediate stop when playback loses localization or hardware output fails.
    ///
    /// This hook cannot return an error, so the default implementation reports a failed stop.
    async fn on_playback_abort(&mut self) {
        if let Err(error) = self.stop_playback().await {
            crate::log_error!("recording.abort: failed to stop outputs: {error:?}");
        }
    }

    /// Runs after a recording and route index entry have been successfully saved.
    async fn on_save(&mut self) {}
}

type FrameSerializer<'a> = HighSerializer<AlignedVec, ArenaHandle<'a>, Error>;
type FrameDeserializer = HighDeserializer<Error>;
type FrameValidator<'a> = HighValidator<'a, Error>;

pub trait Frameable =
    Archive + Default + Clone + std::fmt::Debug + for<'a> Serialize<FrameSerializer<'a>>
    where
        <Self as Archive>::Archived:
            for<'a> CheckBytes<FrameValidator<'a>> + Deserialize<Self, FrameDeserializer>;

/// Stable ID for the persisted shape and meaning of a frame.
pub trait FrameSchema {
    const SCHEMA_ID: u64;
}

/// Hash a stable schema description. Include the names, order, and meaning of fields.
pub const fn schema_id(description: &str) -> u64 {
    let mut hash = 0xcbf29ce484222325_u64;
    let bytes = description.as_bytes();
    let mut index = 0;
    while index < bytes.len() {
        hash ^= bytes[index] as u64;
        hash = hash.wrapping_mul(0x100000001b3);
        index += 1;
    }
    hash
}

macro_rules! primitive_schemas {
    ($($ty:ty),* $(,)?) => {$(
        impl FrameSchema for $ty {
            const SCHEMA_ID: u64 = schema_id(stringify!($ty));
        }
    )*};
}
primitive_schemas!(
    bool, i8, i16, i32, i64, i128, isize, u8, u16, u32, u64, u128, usize, f32, f64
);

const ARCHIVE_PREFIX: &[u8; 7] = b"OZTONR!";
const ARCHIVE_VERSION: u8 = 1;
const HEADER_SIZE: usize = 16;

pub(crate) fn archive_header<F: FrameSchema>() -> [u8; HEADER_SIZE] {
    let mut header = [0; HEADER_SIZE];
    header[..7].copy_from_slice(ARCHIVE_PREFIX);
    header[7] = ARCHIVE_VERSION;
    header[8..].copy_from_slice(&F::SCHEMA_ID.to_le_bytes());
    header
}

pub(crate) fn write_archive(
    mut writer: impl std::io::Write,
    header: &[u8; HEADER_SIZE],
    payload: &[u8],
) -> Result<(), RecordingError> {
    if payload.len() as u64 > MAX_RECORDING_BYTES - HEADER_SIZE as u64 {
        return Err(RecordingError::InvalidSize);
    }
    writer.write_all(header)?;
    writer.write_all(payload)?;
    Ok(())
}

pub(crate) fn decode_archive<F: Frameable + FrameSchema>(
    bytes: &[u8],
) -> Result<Recording<F>, RecordingError> {
    let payload = if bytes.starts_with(ARCHIVE_PREFIX) {
        if !bytes.starts_with(&archive_header::<F>()) {
            return Err(RecordingError::IncompatibleSchema);
        }
        &bytes[HEADER_SIZE..]
    } else {
        // Legacy recordings predate the schema header. Validate their archive as before.
        bytes
    };
    Ok(from_bytes::<Recording<F>, Error>(payload)?)
}

/// Keeps playback lookup linear in total frame count as elapsed time advances.
struct PlaybackCursor<'a, F: Frameable> {
    frames: &'a [TimedFrame<F>],
    index: usize,
    frame_time: Duration,
}

impl<'a, F: Frameable> PlaybackCursor<'a, F> {
    fn new(frames: &'a [TimedFrame<F>]) -> Self {
        Self {
            frames,
            index: 0,
            frame_time: frames.first().map_or(Duration::ZERO, |first| {
                Duration::from_micros(first.delta_time_micros)
            }),
        }
    }

    fn frame_at(&mut self, elapsed: Duration) -> Option<&'a F> {
        if self.index == 0 && elapsed <= self.frame_time {
            return self.frames.first().map(|timed| &timed.frame);
        }
        while let Some(next) = self.frames.get(self.index + 1) {
            let next_time = self
                .frame_time
                .saturating_add(Duration::from_micros(next.delta_time_micros));
            if elapsed < next_time {
                break;
            }
            self.index += 1;
            self.frame_time = next_time;
        }
        self.frames.get(self.index).map(|timed| &timed.frame)
    }
}

impl<F: Frameable> Recording<F> {
    #[must_use]
    pub fn with_frame_capacity(frame_capacity: usize) -> Self {
        Self {
            frames: Vec::with_capacity(frame_capacity),
        }
    }

    #[allow(dead_code)]
    pub fn push_timed(&mut self, delta: Duration, frame: F) {
        self.frames.push(TimedFrame {
            delta_time_micros: u64::try_from(delta.as_micros()).unwrap_or(u64::MAX),
            frame,
        });
    }

    #[allow(dead_code)]
    pub fn duration(&self) -> Duration {
        self.frames.iter().fold(Duration::ZERO, |duration, frame| {
            duration.saturating_add(Duration::from_micros(frame.delta_time_micros))
        })
    }

    pub fn frame_at(&self, elapsed: Duration) -> Option<F> {
        self.frame_at_ref(elapsed).cloned()
    }

    pub fn frame_at_ref(&self, elapsed: Duration) -> Option<&F> {
        PlaybackCursor::new(&self.frames).frame_at(elapsed)
    }

    #[allow(dead_code)]
    pub fn save<P: AsRef<Path>>(&self, path: P) -> Result<(), RecordingError>
    where
        F: FrameSchema,
    {
        crate::log!(
            "recording.save: path={} frames={}",
            path.as_ref().display(),
            self.frames.len()
        );
        let bytes = to_bytes::<Error>(self)?;
        if bytes.len() as u64 > MAX_RECORDING_BYTES - HEADER_SIZE as u64 {
            return Err(RecordingError::InvalidSize);
        }
        write_archive(std::fs::File::create(path)?, &archive_header::<F>(), &bytes)?;
        crate::log!("recording.save: success");
        Ok(())
    }

    #[allow(dead_code)]
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self, RecordingError>
    where
        F: FrameSchema,
    {
        crate::log!("recording.load: path={}", path.as_ref().display());
        use std::io::Read;
        let file = std::fs::File::open(path)?;
        let mut aligned = AlignedVec::<16>::new();
        let mut reader = file.take(MAX_RECORDING_BYTES + 1);
        let mut buffer = [0; 1024];
        loop {
            let count = reader.read(&mut buffer)?;
            if count == 0 {
                break;
            }
            if aligned.len() as u64 + count as u64 > MAX_RECORDING_BYTES {
                return Err(RecordingError::InvalidSize);
            }
            aligned.extend_from_slice(&buffer[..count]);
        }
        let recording = decode_archive::<F>(&aligned)?;
        crate::log!(
            "recording.load: success frames={} duration={}us",
            recording.frames.len(),
            recording.duration().as_micros()
        );
        Ok(recording)
    }

    #[allow(dead_code)]
    pub async fn playback<R: Recordable<Frame = F>>(self, robot: &mut R) -> PlaybackOutcome {
        if self.frames.is_empty() {
            crate::log!("recording.playback: skipped empty recording");
            return match robot.stop_playback().await {
                Ok(()) => PlaybackOutcome::Completed,
                Err(error) => {
                    crate::log_error!("recording.playback: failed to stop empty route: {error:?}");
                    robot.on_playback_abort().await;
                    PlaybackOutcome::OutputFailed
                }
            };
        }

        let total_duration = self.duration();
        crate::log!(
            "recording.playback: start frames={} duration={}us",
            self.frames.len(),
            total_duration.as_micros()
        );
        let start = Instant::now();
        let interval = recording_interval(R::UPDATE_INTERVAL);
        let mut outcome = PlaybackOutcome::Completed;
        let mut cursor = PlaybackCursor::new(&self.frames);

        loop {
            if !robot.playback_ready() || !robot.devices_ready() {
                crate::log!("recording.playback: localization unavailable; aborting route");
                outcome = PlaybackOutcome::TrackingLost;
                break;
            }
            let tick = Instant::now();
            let elapsed = tick.saturating_duration_since(start).min(total_duration);

            if let Some(frame) = cursor.frame_at(elapsed)
                && let Err(error) = robot.apply_playback_frame(frame).await
            {
                crate::log_error!("recording.playback: apply failed; aborting route: {error:?}");
                outcome = match error {
                    PlaybackError::TrackingUnavailable => PlaybackOutcome::TrackingLost,
                    _ => PlaybackOutcome::OutputFailed,
                };
                break;
            }

            if elapsed >= total_duration {
                break;
            }

            let remaining = total_duration.saturating_sub(elapsed);
            let Some(next_deadline) = tick.checked_add(interval.min(remaining)) else {
                crate::log_error!("recording.playback: deadline overflow; aborting route");
                outcome = PlaybackOutcome::OutputFailed;
                break;
            };
            sleep_until(next_deadline).await;
        }

        if outcome != PlaybackOutcome::Completed {
            robot.on_playback_abort().await;
        } else if let Err(error) = robot.stop_playback().await {
            crate::log_error!("recording.playback: failed to stop outputs: {error:?}");
            robot.on_playback_abort().await;
            outcome = PlaybackOutcome::OutputFailed;
        }
        crate::log!("recording.playback: outcome={outcome:?}");
        outcome
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{PlaybackCursor, Recording};

    #[derive(Default)]
    struct FailingRobot {
        apply_failed: bool,
        stop_failed: bool,
        stops: usize,
    }

    #[async_trait::async_trait(?Send)]
    impl super::FrameRobot for FailingRobot {
        type Frame = f64;

        async fn finalize_frame(&self, frame: &f64) -> f64 {
            *frame
        }

        async fn apply_frame(
            &mut self,
            _frame: &f64,
            _mode: super::RecordMode,
        ) -> Result<(), vexide::smart::PortError> {
            if self.apply_failed {
                Err(vexide::smart::PortError::Disconnected { port: 1 })
            } else {
                Ok(())
            }
        }

        async fn stop_playback(&mut self) -> Result<(), vexide::smart::PortError> {
            self.stops += 1;
            if self.stop_failed {
                Err(vexide::smart::PortError::Disconnected { port: 1 })
            } else {
                Ok(())
            }
        }
    }

    #[async_trait::async_trait(?Send)]
    impl super::Recordable for FailingRobot {
        const UPDATE_INTERVAL: Duration = Duration::from_millis(10);

        async fn get_new_frame(&self) -> f64 {
            0.0
        }
    }

    #[test]
    fn failed_playback_apply_stops_outputs_and_reports_failure() {
        let mut robot = FailingRobot {
            apply_failed: true,
            ..FailingRobot::default()
        };
        let mut recording = Recording::default();
        recording.push_timed(Duration::ZERO, 1.0);

        let outcome = futures::executor::block_on(recording.playback(&mut robot));

        assert_eq!(outcome, super::PlaybackOutcome::OutputFailed);
        assert_eq!(robot.stops, 1);
    }

    #[test]
    fn failed_playback_stop_retries_through_abort_and_reports_failure() {
        // An empty recording still has to stop outputs. A stop failure must not become success.
        for empty in [true, false] {
            let mut robot = FailingRobot {
                stop_failed: true,
                ..FailingRobot::default()
            };
            let mut recording = Recording::default();
            if !empty {
                recording.push_timed(Duration::ZERO, 1.0);
            }

            let outcome = futures::executor::block_on(recording.playback(&mut robot));

            assert_eq!(outcome, super::PlaybackOutcome::OutputFailed);
            assert_eq!(robot.stops, 2);
        }
    }

    #[test]
    fn playback_cursor_matches_frame_lookup_across_skipped_frames() {
        let mut recording = Recording::<f64>::default();
        recording.push_timed(Duration::ZERO, 1.0);
        recording.push_timed(Duration::from_millis(10), 2.0);
        recording.push_timed(Duration::ZERO, 3.0);
        recording.push_timed(Duration::from_millis(20), 4.0);

        let mut cursor = PlaybackCursor::new(&recording.frames);
        for millis in [0, 5, 10, 11, 29, 30, 50] {
            let elapsed = Duration::from_millis(millis);
            assert_eq!(
                cursor.frame_at(elapsed).copied(),
                recording.frame_at(elapsed)
            );
        }
    }
}
