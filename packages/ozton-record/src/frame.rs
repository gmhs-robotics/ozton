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

    async fn get_new_frame(&self) -> Self::Frame;

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

impl<F: Frameable> Recording<F> {
    #[must_use]
    pub fn with_frame_capacity(frame_capacity: usize) -> Self {
        Self {
            frames: Vec::with_capacity(frame_capacity),
        }
    }

    #[allow(dead_code)]
    pub fn push_timed(&mut self, delta: Duration, frame: F) {
        crate::log!(
            "recording.push_timed: delta={}us frame={frame:?}",
            delta.as_micros()
        );
        self.frames.push(TimedFrame {
            delta_time_micros: delta.as_micros() as u64,
            frame,
        });
    }

    #[allow(dead_code)]
    pub fn duration(&self) -> Duration {
        self.frames.iter().fold(Duration::ZERO, |duration, frame| {
            duration + Duration::from_micros(frame.delta_time_micros)
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
            let next_time = current_time + Duration::from_micros(next.delta_time_micros);

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
    pub async fn playback<R: Recordable<Frame = F>>(self, robot: &mut R) {
        if self.frames.is_empty() {
            crate::log!("recording.playback: skipped empty recording");
            return;
        }

        let total_duration = self.duration();
        crate::log!(
            "recording.playback: start frames={} duration={}us",
            self.frames.len(),
            total_duration.as_micros()
        );
        let start = Instant::now();
        let mut deadline = start;

        loop {
            let elapsed = start.elapsed().min(total_duration);

            if let Some(frame) = self.frame_at(elapsed) {
                crate::log!(
                    "recording.playback: apply frame elapsed={}us frame={frame:?}",
                    elapsed.as_micros(),
                );
                if let Err(error) = robot.apply_frame(&frame, RecordMode::Playback).await {
                    crate::log!("recording.playback: apply error: {error:?}");
                }
            }

            if elapsed >= total_duration {
                break;
            }

            let Some(next_deadline) = deadline.checked_add(R::UPDATE_INTERVAL) else {
                crate::log!("recording.playback: deadline overflow, aborting");
                break;
            };
            deadline = next_deadline;
            sleep_until(deadline).await;
        }

        if let Some(last) = self.frames.last() {
            crate::log!("recording.playback: apply final frame {:?}", last.frame);
            if let Err(error) = robot.apply_frame(&last.frame, RecordMode::Playback).await {
                crate::log!("recording.playback: final apply error: {error:?}");
            }
        }

        if let Err(error) = robot.stop_playback().await {
            crate::log!("recording.playback: stop_playback error: {error:?}");
        }
        crate::log!("recording.playback: complete");
    }
}
