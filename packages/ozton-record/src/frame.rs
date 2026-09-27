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

use crate::frame_types::Interpolate;

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
}

impl std::fmt::Display for RecordingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RecordingError::Io(e) => write!(f, "I/O error: {e}"),
            RecordingError::Rkyv(e) => write!(f, "rkyv error: {e}"),
        }
    }
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

/// A robot whose device fields know how to finalize and apply generated frames.
#[async_trait(?Send)]
pub trait FrameRobot {
    type Frame: Frameable;

    /// Enriches a sampled frame before it is persisted to disk.
    ///
    /// Most fields simply clone the sampled value. Tracked drivetrains can use this hook to attach
    /// odometry state needed for corrected playback.
    async fn finalize_frame(&self, frame: &Self::Frame) -> Self::Frame;

    /// Applies a frame to the live robot.
    async fn apply_frame(&mut self, frame: &Self::Frame, mode: RecordMode)
    -> Result<(), PortError>;

    /// Resets any motor outputs that should be stopped after playback completes.
    async fn stop_playback(&mut self) -> Result<(), PortError>;
}

#[async_trait(?Send)]
pub trait Recordable: FrameRobot {
    const UPDATE_INTERVAL: Duration;

    /// Optional built-in autonomous route shown alongside saved recordings.
    const PREDETERMINED_ROUTE_NAME: Option<&'static str> = None;

    /// Additional built-in routes, listed after `PREDETERMINED_ROUTE_NAME`.
    const ADDITIONAL_PREDETERMINED_ROUTE_NAMES: &'static [&'static str] = &[];

    async fn get_new_frame(&self) -> Self::Frame;

    /// Runs the built-in route when selected. Implement this when setting its name.
    async fn run_predetermined_route(&mut self) -> PlaybackOutcome {
        PlaybackOutcome::Completed
    }

    /// Runs an additional built-in route by its index in
    /// `ADDITIONAL_PREDETERMINED_ROUTE_NAMES`.
    async fn run_additional_predetermined_route(&mut self, _index: usize) -> PlaybackOutcome {
        PlaybackOutcome::Completed
    }

    /// Whether localization is valid enough to record or replay a corrected route.
    fn playback_ready(&self) -> bool {
        true
    }

    /// Immediate stop when playback loses localization or hardware output fails.
    async fn on_playback_abort(&mut self) {
        let _ = self.stop_playback().await;
    }

    /// Runs after a recording and route index entry have been successfully saved.
    async fn on_save(&mut self) {}
}

type FrameSerializer<'a> = HighSerializer<AlignedVec, ArenaHandle<'a>, Error>;
type FrameDeserializer = HighDeserializer<Error>;
type FrameValidator<'a> = HighValidator<'a, Error>;

pub trait Frameable = Archive
    + Default
    + Clone
    + std::fmt::Debug
    + Interpolate
    + for<'a> Serialize<FrameSerializer<'a>>
where
    <Self as Archive>::Archived:
        for<'a> CheckBytes<FrameValidator<'a>> + Deserialize<Self, FrameDeserializer>;

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

    #[allow(dead_code)]
    pub fn frame_at(&self, elapsed: Duration) -> Option<F> {
        let first = self.frames.first()?;
        let mut current = first;
        let mut current_time = Duration::from_micros(first.delta_time_micros);

        if elapsed <= current_time {
            crate::log!(
                "recording.stepped_frame_at: elapsed={}us frame={:?}",
                elapsed.as_micros(),
                current.frame
            );
            return Some(current.frame.clone());
        }

        for next in self.frames.iter().skip(1) {
            let next_time =
                current_time.saturating_add(Duration::from_micros(next.delta_time_micros));

            if elapsed < next_time {
                crate::log!(
                    "recording.stepped_frame_at: elapsed={}us frame={:?}",
                    elapsed.as_micros(),
                    current.frame
                );
                return Some(current.frame.clone());
            }

            current = next;
            current_time = next_time;
        }

        crate::log!(
            "recording.stepped_frame_at: elapsed={}us frame={:?}",
            elapsed.as_micros(),
            current.frame
        );
        Some(current.frame.clone())
    }

    #[allow(dead_code)]
    pub fn save<P: AsRef<Path>>(&self, path: P) -> Result<(), RecordingError> {
        crate::log!(
            "recording.save: path={} frames={}",
            path.as_ref().display(),
            self.frames.len()
        );
        let bytes = to_bytes::<Error>(self)?;
        std::fs::write(path, bytes.as_slice())?;
        crate::log!("recording.save: success");
        Ok(())
    }

    #[allow(dead_code)]
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self, RecordingError> {
        crate::log!("recording.load: path={}", path.as_ref().display());
        let bytes = std::fs::read(path)?;
        let mut aligned = AlignedVec::<16>::with_capacity(bytes.len());
        aligned.extend_from_slice(&bytes);
        let recording = from_bytes::<Self, Error>(&aligned)?;
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
            return if robot.stop_playback().await.is_ok() {
                PlaybackOutcome::Completed
            } else {
                robot.on_playback_abort().await;
                PlaybackOutcome::OutputFailed
            };
        }

        let total_duration = self.duration();
        crate::log!(
            "recording.playback: start frames={} duration={}us",
            self.frames.len(),
            total_duration.as_micros()
        );
        let start = Instant::now();
        let mut outcome = PlaybackOutcome::Completed;
        let mut cursor = PlaybackCursor::new(&self.frames);

        loop {
            if !robot.playback_ready() {
                crate::log!("recording.playback: localization unavailable; aborting route");
                outcome = PlaybackOutcome::TrackingLost;
                break;
            }
            let elapsed = start.elapsed().min(total_duration);

            if let Some(frame) = cursor.frame_at(elapsed) {
                if let Err(error) = robot.apply_frame(frame, RecordMode::Playback).await {
                    crate::log!("recording.playback: apply error: {error:?}");
                    outcome = PlaybackOutcome::OutputFailed;
                    break;
                }
            }

            if elapsed >= total_duration {
                break;
            }

            let interval = R::UPDATE_INTERVAL.max(Duration::from_millis(1));
            let Some(next_deadline) = Instant::now().checked_add(interval) else {
                crate::log!("recording.playback: deadline overflow, aborting");
                outcome = PlaybackOutcome::OutputFailed;
                break;
            };
            sleep_until(next_deadline).await;
        }

        if outcome != PlaybackOutcome::Completed {
            robot.on_playback_abort().await;
        } else if let Err(error) = robot.stop_playback().await {
            crate::log!("recording.playback: stop_playback error: {error:?}");
            robot.on_playback_abort().await;
            outcome = PlaybackOutcome::OutputFailed;
        }
        crate::log!("recording.playback: complete");
        outcome
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{PlaybackCursor, Recording};

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
