//! Exclusive recording and playback competition workflow.

use std::{
    path::Path,
    time::{Duration, Instant},
};

use ozton_runtime::{
    clock::CommandClock,
    selector::{Selector, SelectorItem, StatusHandle},
};
use vexide::{
    competition::{Compete, CompeteExt},
    display::Display,
    time::sleep,
};

use crate::{
    frame::{Frameable, PlaybackOutcome, RecordMode, Recordable, Recording},
    selector::RecordTarget,
    store::{MAX_FRAMES, Pending, RouteStore, StoreError},
};

const MAX_GAP: Duration = Duration::from_millis(100);

/// Bounded capture. On a capacity limit, finish capture and keep driving.
pub struct RecordingSession<F: Frameable> {
    recording: Recording<F>,
    previous: Option<Instant>,
    elapsed: Duration,
    maximum: Duration,
    capacity: usize,
}

impl<F: Frameable> RecordingSession<F> {
    pub fn new(maximum: Duration, capacity: usize) -> Self {
        let capacity = capacity.min(MAX_FRAMES);
        Self {
            recording: Recording::with_frame_capacity(capacity),
            previous: None,
            elapsed: Duration::ZERO,
            maximum,
            capacity,
        }
    }

    /// Returns false when no more frames fit. An error indicates a broken timing reference.
    fn push(&mut self, frame: F, now: Instant) -> Result<bool, ()> {
        let delta = self
            .previous
            .map_or(Duration::ZERO, |last| now.saturating_duration_since(last));
        if delta > MAX_GAP {
            return Err(());
        }
        let elapsed = self.elapsed.saturating_add(delta);
        if elapsed > self.maximum || self.recording.frames.len() >= self.capacity {
            return Ok(false);
        }
        self.recording.push_timed(delta, frame);
        self.previous = Some(now);
        self.elapsed = elapsed;
        Ok(self.recording.frames.len() < self.capacity && self.elapsed < self.maximum)
    }

    fn finish(self) -> Option<Recording<F>> {
        (!self.recording.frames.is_empty()).then_some(self.recording)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Action {
    Disabled,
    Capture(RecordTarget),
    Play(u32),
    Retry,
}

#[derive(Clone)]
struct Choice {
    label: String,
    action: Action,
}

impl SelectorItem for Choice {
    fn label(&self) -> &str {
        &self.label
    }
}

/// Select Record New, Record Over, or Play on the Brain.
/// Capture takes place in driver mode, save while disabled, and replay in autonomous.
pub struct RecordingAutonomous<R: Recordable + 'static> {
    robot: R,
    store: Option<RouteStore>,
    pending: Option<Pending<R::Frame>>,
    capture: Option<(RecordTarget, RecordingSession<R::Frame>)>,
    status: StatusHandle<Choice>,
    _selector: Selector<Choice>,
}

impl<R: Recordable + 'static> RecordingAutonomous<R> {
    pub async fn compete(robot: R, display: Display) -> ! {
        let store = RouteStore::open(Path::new(".")).ok();
        let selector = Selector::new(display, choices(store.as_ref(), false), 0);
        let status = selector.status_handle();
        if store.is_none() {
            status.show_status_persistent("SD unavailable");
        }
        Self {
            robot,
            store,
            pending: None,
            capture: None,
            status,
            _selector: selector,
        }
        .compete()
        .await
    }

    fn interval() -> Duration {
        R::UPDATE_INTERVAL.clamp(Duration::from_millis(1), MAX_GAP)
    }

    fn maximum() -> Duration {
        R::MAX_CAPTURE_DURATION.min(Duration::from_secs(60))
    }

    fn finish_capture(&mut self) {
        if let Some((target, session)) = self.capture.take()
            && let Some(recording) = session.finish()
        {
            self.pending = Some(Pending::new(target, recording));
        }
    }

    fn replace_menu(&self, selected: Action) {
        let choices = choices(self.store.as_ref(), self.pending.is_some());
        let index = choices
            .iter()
            .position(|choice| choice.action == selected)
            .unwrap_or(0);
        self.status.replace_options(choices, index);
    }

    fn commit(&mut self) -> Result<Option<u32>, StoreError> {
        let Some(pending) = self.pending.as_mut() else {
            return Ok(None);
        };
        if self.store.is_none() {
            self.store = Some(RouteStore::open(Path::new("."))?);
        }
        let id = self.store.as_mut().expect("store opened").save(
            pending,
            Self::maximum(),
            R::valid_frame,
        )?;
        self.pending = None;
        Ok(Some(id))
    }

    async fn save(&mut self) {
        match self.commit() {
            Ok(Some(id)) => {
                self.replace_menu(Action::Play(id));
                self.status
                    .show_status_persistent(format!("Saved route {id}"));
                self.robot.on_save().await;
            }
            Ok(None) => {}
            Err(error) => {
                self.replace_menu(Action::Disabled);
                self.status
                    .show_status_persistent("Save failed. Select Retry save.");
                crate::log_error!("recording save failed: {error:?}");
            }
        }
    }
}

impl<R: Recordable + 'static> Compete for RecordingAutonomous<R> {
    async fn driver(&mut self) {
        self.robot.on_playback_abort().await;
        self.finish_capture();
        if self.pending.is_none()
            && let Action::Capture(target) = self.status.with_selection(|choice| choice.action)
        {
            let maximum = Self::maximum();
            let capacity = maximum.as_micros().div_ceil(Self::interval().as_micros()) + 1;
            self.capture = Some((target, RecordingSession::new(maximum, capacity as usize)));
            self.status.show_status_persistent("Recording");
        }
        let mut clock = CommandClock::new(Self::interval()).expect("nonzero recording interval");
        loop {
            clock.tick().await;
            let now = Instant::now();
            let frame = self.robot.get_new_frame().await;
            let capture = if self.capture.is_some() {
                let valid = self.robot.playback_ready();
                if valid {
                    let finalized = self.robot.finalize_frame(&frame).await;
                    R::valid_frame(&finalized).then_some(finalized)
                } else {
                    None
                }
            } else {
                None
            };
            let result = self.robot.apply_frame(&frame, RecordMode::Live).await;
            if let Some((_, session)) = &mut self.capture {
                let result = match (capture, result) {
                    (Some(frame), Ok(())) => session.push(frame, now),
                    _ => Err(()),
                };
                match result {
                    Ok(true) => {}
                    Ok(false) => {
                        self.finish_capture();
                        self.status
                            .show_status_persistent("Capture full. Disable to save.");
                    }
                    Err(()) => {
                        self.capture = None;
                        self.status
                            .show_status_persistent("Capture discarded: feedback or output");
                    }
                }
            }
        }
    }

    async fn disabled(&mut self) {
        self.robot.on_playback_abort().await;
        self.finish_capture();
        self.save().await;
        loop {
            if self.status.with_selection(|choice| choice.action) == Action::Retry {
                self.save().await;
            }
            self.robot.on_playback_abort().await;
            sleep(Self::interval()).await;
        }
    }

    async fn autonomous(&mut self) {
        self.robot.on_playback_abort().await;
        self.finish_capture();
        if let Action::Play(id) = self.status.with_selection(|choice| choice.action) {
            let recording = self
                .store
                .as_ref()
                .ok_or(StoreError::MissingRoute)
                .and_then(|store| store.load(id, Self::maximum(), R::valid_frame));
            match recording {
                Ok(recording) => {
                    self.status
                        .show_status_persistent(format!("Playing route {id}"));
                    let status = match recording.playback(&mut self.robot).await {
                        PlaybackOutcome::Completed => "Playback complete",
                        PlaybackOutcome::TrackingLost => "Playback stopped: tracking",
                        PlaybackOutcome::OutputFailed => "Playback stopped: output",
                    };
                    self.status.show_status_persistent(status);
                }
                Err(error) => {
                    self.status
                        .show_status_persistent("Route could not be loaded");
                    crate::log_error!("recording load failed: {error:?}");
                }
            }
        } else {
            self.status.show_status_persistent("Playback disabled");
        }
        loop {
            self.robot.on_playback_abort().await;
            sleep(Self::interval()).await;
        }
    }

    async fn disconnected(&mut self) {
        self.robot.on_playback_abort().await;
        self.finish_capture();
    }
}

fn choices(store: Option<&RouteStore>, pending: bool) -> Vec<Choice> {
    let mut choices = vec![Choice {
        label: "Disable".to_owned(),
        action: Action::Disabled,
    }];
    if pending {
        choices.push(Choice {
            label: "Retry save".to_owned(),
            action: Action::Retry,
        });
    } else {
        choices.push(Choice {
            label: "Record new route".to_owned(),
            action: Action::Capture(RecordTarget::New),
        });
    }
    if let Some(store) = store {
        for (id, label) in store.iter() {
            choices.push(Choice {
                label: format!("Play {label}"),
                action: Action::Play(id),
            });
            if !pending {
                choices.push(Choice {
                    label: format!("Record over {label}"),
                    action: Action::Capture(RecordTarget::Overwrite(id)),
                });
            }
        }
    }
    choices
}
