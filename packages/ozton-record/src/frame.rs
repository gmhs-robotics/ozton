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

#[async_trait(?Send)]
pub trait Recordable {
    type Frame: Frameable;
    const UPDATE_INTERVAL: Duration;

    async fn transform_to_frame(&mut self, frame: &Self::Frame) -> Result<(), PortError>;
    async fn get_new_frame(&self) -> Self::Frame;
}

type FrameSerializer<'a> = HighSerializer<AlignedVec, ArenaHandle<'a>, Error>;
type FrameDeserializer = HighDeserializer<Error>;
type FrameValidator<'a> = HighValidator<'a, Error>;

pub trait Frameable = Archive + Default + Clone + for<'a> Serialize<FrameSerializer<'a>>
where
    <Self as Archive>::Archived:
        for<'a> CheckBytes<FrameValidator<'a>> + Deserialize<Self, FrameDeserializer>;

impl<F: Frameable> Recording<F> {
    #[allow(dead_code)]
    pub fn push_timed(&mut self, delta: Duration, frame: F) {
        self.frames.push(TimedFrame {
            delta_time_micros: delta.as_micros() as u64,
            frame,
        });
    }

    #[allow(dead_code)]
    pub fn save<P: AsRef<Path>>(&self, path: P) -> Result<(), RecordingError> {
        let bytes = to_bytes::<Error>(self)?;
        std::fs::write(path, bytes.as_slice())?;
        Ok(())
    }

    #[allow(dead_code)]
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self, RecordingError> {
        let bytes = std::fs::read(path)?;
        let recording = from_bytes::<Self, Error>(&bytes)?;
        Ok(recording)
    }

    #[allow(dead_code)]
    pub async fn playback<R: Recordable<Frame = F>>(self, robot: &mut R) {
        let mut deadline = Instant::now();

        for tf in self.frames {
            let frame_delta = Duration::from_micros(tf.delta_time_micros);
            let Some(next_deadline) = deadline.checked_add(frame_delta) else {
                break;
            };
            deadline = next_deadline;

            sleep_until(deadline).await;

            // Apply each frame at its recorded timestamp rather than one frame early.
            let _ = robot.transform_to_frame(&tf.frame).await;
        }
    }
}
