use std::time::{Duration, Instant};

use vexide::{
    competition::{Compete, CompeteExt},
    prelude::*,
    time::sleep,
};

use super::{
    frame::{Frameable, PlaybackOutcome, RecordMode, Recordable, Recording, RecordingError},
    routes::RouteIndex,
    selector::{
        PlaybackChoice, PlaybackSource, RecordOption, RecordTarget, RecorderSelect,
        SelectionController,
    },
};

const AUTONOMOUS_DURATION: Duration = Duration::from_secs(15);

#[allow(dead_code)]
#[derive(Debug, Default)]
enum RecorderState<F: Frameable> {
    #[default]
    Idle,
    Recording {
        target: RecordTarget,
        current: Recording<F>,
        last_frame_time: Option<Instant>,
    },
}

#[allow(dead_code)]
pub struct RecordingSession<F: Frameable> {
    state: RecorderState<F>,
    frame_capacity: usize,
}

#[allow(dead_code)]
impl<F: Frameable> RecordingSession<F> {
    pub fn new() -> Self {
        Self::with_frame_capacity(0)
    }

    pub fn with_frame_capacity(frame_capacity: usize) -> Self {
        crate::log!("runtime.recording_session.new");
        Self {
            state: RecorderState::default(),
            frame_capacity,
        }
    }

    pub fn set_target(&mut self, target: RecordTarget) {
        crate::log!("runtime.recording_session.set_target: {:?}", target);
        self.state = match target {
            RecordTarget::Off => RecorderState::Idle,
            other => RecorderState::Recording {
                target: other,
                current: Recording::with_frame_capacity(self.frame_capacity),
                last_frame_time: None,
            },
        };
    }

    pub fn target(&self) -> RecordTarget {
        match &self.state {
            RecorderState::Idle => RecordTarget::Off,
            RecorderState::Recording { target, .. } => *target,
        }
    }

    pub fn is_recording(&self) -> bool {
        matches!(self.state, RecorderState::Recording { .. })
    }

    pub fn push_frame(&mut self, frame: F) {
        let RecorderState::Recording {
            current,
            last_frame_time,
            ..
        } = &mut self.state
        else {
            return;
        };

        let now = Instant::now();
        let delta = if let Some(last) = last_frame_time.replace(now) {
            now.saturating_duration_since(last)
        } else {
            Default::default()
        };

        current.push_timed(delta, frame);
    }

    pub fn finish(&mut self) -> Option<(RecordTarget, Recording<F>)> {
        let RecorderState::Recording {
            target, current, ..
        } = core::mem::replace(&mut self.state, RecorderState::Idle)
        else {
            return None;
        };

        if current.frames.is_empty() {
            crate::log!("runtime.recording_session.finish: no frames recorded");
            return None;
        }

        crate::log!(
            "runtime.recording_session.finish: target={target:?} frames={}",
            current.frames.len()
        );
        Some((target, current))
    }
}

#[allow(dead_code)]
pub struct RecordingAutonomous<R: Recordable + 'static> {
    pub robot: R,
    pub index: RouteIndex,
    recorder: RecordingSession<R::Frame>,
    selection: SelectionController<RecordOption>,
    _selector: RecorderSelect<RecordOption>,
}

#[allow(dead_code)]
impl<R: Recordable + 'static> RecordingAutonomous<R> {
    pub async fn compete(robot: R, display: Display) -> ! {
        crate::log!("runtime.recording_autonomous.compete: start");
        let index = RouteIndex::load();

        let selector = RecorderSelect::new(display, record_options(&index), 0);

        let selection = SelectionController::new(selector.status_handle());
        let recorder =
            RecordingSession::with_frame_capacity(estimated_frame_capacity(R::UPDATE_INTERVAL));

        Self {
            robot,
            index,
            recorder,
            selection,
            _selector: selector,
        }
        .compete()
        .await;
    }

    async fn save_recording(&mut self, target: RecordTarget, recording: Recording<R::Frame>) {
        crate::log!(
            "runtime.recording_autonomous.save_recording: target={target:?} frames={}",
            recording.frames.len()
        );
        let Some(route_id) = (match target {
            RecordTarget::Off => None,
            RecordTarget::New => Some(self.index.next_id()),
            RecordTarget::Overwrite(id) => Some(id),
        }) else {
            crate::log!("runtime.recording_autonomous.save_recording: target off, skipping");
            return;
        };

        let display_name = self.index.display_name_or_id(route_id);
        let path = RouteIndex::path_for(route_id);

        if let Err(error) = recording.save(&path) {
            crate::log!(
                "runtime.recording_autonomous.save_recording: save failed path={} error={error}",
                path.display()
            );
            self.selection
                .status()
                .show_status(prefixed_status("Failed to save ", &display_name));
            return;
        }

        let display_name = display_name.into_owned();
        self.index.update(route_id, &display_name);
        if let Err(error) = self.index.save() {
            crate::log!(
                "runtime.recording_autonomous.save_recording: index save failed route={} error={error}",
                route_id
            );
            self.selection.status().show_status(prefixed_status(
                "Saved route, failed index update for ",
                &display_name,
            ));
            return;
        }

        crate::log!(
            "runtime.recording_autonomous.save_recording: success route={} name={display_name}",
            route_id
        );
        self.robot.on_save().await;
        self.selection
            .status()
            .show_status(prefixed_status("Saved ", &display_name));
    }

    async fn arm_recording(&mut self, option: RecordOption) {
        crate::log!(
            "runtime.recording_autonomous.arm_recording: label={} target={:?}",
            option.label,
            option.target
        );
        self.recorder.set_target(option.target);

        if let RecordTarget::Off = option.target {
            self.selection.status().show_status("Recording off");
        } else {
            self.selection
                .status()
                .show_status(prefixed_status("Armed: ", &option.label));
        }
    }

    async fn update_selection(&mut self) {
        if let Some(option) = self.selection.consume_selection_change() {
            self.arm_recording(option).await;
        }
    }
}

#[allow(dead_code)]
pub struct PlaybackAutonomous<R: Recordable + 'static> {
    pub robot: R,
    pub index: RouteIndex,
    active_route: PlaybackSource,
    route_played_this_autonomous: bool,
    selection: SelectionController<PlaybackChoice>,
    _selector: RecorderSelect<PlaybackChoice>,
}

#[allow(dead_code)]
impl<R: Recordable + 'static> PlaybackAutonomous<R> {
    pub async fn compete(robot: R, display: Display) -> ! {
        crate::log!("runtime.playback_autonomous.compete: start");
        let index = RouteIndex::load();

        let selector = RecorderSelect::new(
            display,
            playback_choices(
                &index,
                R::PREDETERMINED_ROUTE_NAME,
                R::ADDITIONAL_PREDETERMINED_ROUTE_NAMES,
            ),
            0,
        );
        let selection = SelectionController::new(selector.status_handle());

        Self {
            robot,
            index,
            active_route: PlaybackSource::Disabled,
            route_played_this_autonomous: false,
            selection,
            _selector: selector,
        }
        .compete()
        .await;
    }

    async fn play_selected(&mut self, choice: PlaybackChoice) {
        crate::log!(
            "runtime.playback_autonomous.play_selected: label={} source={:?}",
            choice.label,
            choice.source
        );
        self.active_route = choice.source;
    }

    async fn update_selection(&mut self) {
        if let Some(choice) = self.selection.consume_selection_change() {
            self.play_selected(choice).await;
        }
    }
}

impl<R: Recordable + 'static> Compete for RecordingAutonomous<R> {
    async fn driver(&mut self) {
        crate::log!("runtime.recording_autonomous.driver: enter");
        self.robot.on_playback_abort().await;
        loop {
            self.update_selection().await;

            let frame = self.robot.get_new_frame().await;
            if self.recorder.is_recording() && !self.robot.playback_ready() {
                self.recorder.set_target(RecordTarget::Off);
                self.selection
                    .status()
                    .show_status("Tracking lost: recording discarded");
            }
            if self.recorder.is_recording() {
                let finalized = self.robot.finalize_frame(&frame).await;
                self.recorder.push_frame(finalized);
            }

            if let Err(error) = self.robot.apply_frame(&frame, RecordMode::Live).await {
                crate::log!("runtime.recording_autonomous.driver: live apply error: {error:?}");
            }

            sleep(effective_update_interval(R::UPDATE_INTERVAL)).await;
        }
    }

    async fn disabled(&mut self) {
        crate::log!("runtime.recording_autonomous.disabled");
        self.robot.on_playback_abort().await;

        if let Some((target, recording)) = self.recorder.finish() {
            self.save_recording(target, recording).await;
        }
        self.update_selection().await;
    }

    async fn autonomous(&mut self) {
        self.robot.on_playback_abort().await;
        self.update_selection().await;
        sleep(effective_update_interval(R::UPDATE_INTERVAL)).await;
    }
}

impl<R: Recordable + 'static> Compete for PlaybackAutonomous<R> {
    async fn driver(&mut self) {
        self.route_played_this_autonomous = false;
        crate::log!("runtime.playback_autonomous.driver: enter");
        self.robot.on_playback_abort().await;

        loop {
            self.update_selection().await;

            let frame = self.robot.get_new_frame().await;

            if let Err(error) = self.robot.apply_frame(&frame, RecordMode::Live).await {
                crate::log!("runtime.playback_autonomous.driver: live apply error: {error:?}");
            }

            sleep(effective_update_interval(R::UPDATE_INTERVAL)).await;
        }
    }

    async fn disabled(&mut self) {
        self.route_played_this_autonomous = false;
        crate::log!("runtime.playback_autonomous.disabled");
        self.robot.on_playback_abort().await;
        self.update_selection().await;
    }

    async fn autonomous(&mut self) {
        self.robot.on_playback_abort().await;
        crate::log!(
            "runtime.playback_autonomous.before_route: active_route={:?} already_played={}",
            self.active_route,
            self.route_played_this_autonomous
        );
        self.update_selection().await;

        if self.route_played_this_autonomous {
            sleep(effective_update_interval(R::UPDATE_INTERVAL)).await;
            return;
        }

        let route_id = match self.active_route {
            PlaybackSource::Disabled => {
                crate::log!("runtime.playback_autonomous.before_route: playback disabled");
                self.selection.status().show_status("Playback disabled");
                sleep(effective_update_interval(R::UPDATE_INTERVAL)).await;
                return;
            }
            PlaybackSource::Predetermined(index) => {
                self.route_played_this_autonomous = true;
                let name = if index == 0 {
                    R::PREDETERMINED_ROUTE_NAME
                } else {
                    R::ADDITIONAL_PREDETERMINED_ROUTE_NAMES
                        .get(index - 1)
                        .copied()
                };
                if let Some(name) = name {
                    self.selection
                        .status()
                        .show_status(prefixed_status("Playing ", name));
                    let outcome = if index == 0 {
                        self.robot.run_predetermined_route().await
                    } else {
                        self.robot
                            .run_additional_predetermined_route(index - 1)
                            .await
                    };
                    match outcome {
                        PlaybackOutcome::Completed => {}
                        PlaybackOutcome::TrackingLost => self
                            .selection
                            .status()
                            .show_status("Route stopped: tracking lost"),
                        PlaybackOutcome::OutputFailed => self
                            .selection
                            .status()
                            .show_status("Route stopped: output failed"),
                    }
                }
                return;
            }
            PlaybackSource::Recorded(id) => id,
        };

        self.route_played_this_autonomous = true;

        let path = RouteIndex::path_for(route_id);
        let display_name = self.index.display_name_or_id(route_id);
        crate::log!(
            "runtime.playback_autonomous.before_route: loading route={} path={} name={display_name}",
            route_id,
            path.display()
        );

        match Recording::load(&path) {
            Ok(recording) => {
                self.selection
                    .status()
                    .show_status(prefixed_status("Playing ", &display_name));
                crate::log!("runtime.playback_autonomous.before_route: playback starting");
                match recording.playback(&mut self.robot).await {
                    PlaybackOutcome::Completed => {}
                    PlaybackOutcome::TrackingLost => self
                        .selection
                        .status()
                        .show_status("Playback stopped: tracking lost"),
                    PlaybackOutcome::OutputFailed => self
                        .selection
                        .status()
                        .show_status("Playback stopped: output failed"),
                }
            }
            Err(RecordingError::Io(error)) if error.kind() == std::io::ErrorKind::NotFound => {
                crate::log!(
                    "runtime.playback_autonomous.before_route: missing route file path={} error={error}",
                    path.display()
                );
                self.selection
                    .status()
                    .show_status(prefixed_status("Missing route ", &display_name));
            }
            Err(error) => {
                crate::log!(
                    "runtime.playback_autonomous.before_route: load failed path={} error={error}",
                    path.display()
                );
                self.selection
                    .status()
                    .show_status(prefixed_status("Load failed ", &display_name));
            }
        }
    }
}

fn record_options(index: &RouteIndex) -> Vec<RecordOption> {
    let mut options: Vec<_> = vec![
        RecordOption {
            label: "Record Off".to_owned(),
            target: RecordTarget::Off,
        },
        RecordOption {
            label: "Record New Route".to_owned(),
            target: RecordTarget::New,
        },
    ];
    options.reserve(index.len());
    options.extend(index.iter().map(|(id, display_name)| RecordOption {
        label: prefixed_status("Record over ", display_name),
        target: RecordTarget::Overwrite(id),
    }));
    crate::log!("runtime.record_options: {} options", options.len());
    options
}

fn playback_choices(
    index: &RouteIndex,
    predetermined_name: Option<&str>,
    additional_names: &[&str],
) -> Vec<PlaybackChoice> {
    let mut playback_choices = Vec::with_capacity(index.len() + additional_names.len() + 2);
    playback_choices.push(PlaybackChoice {
        label: "Disable".to_string(),
        source: PlaybackSource::Disabled,
    });

    if let Some(name) = predetermined_name {
        playback_choices.push(PlaybackChoice {
            label: name.to_owned(),
            source: PlaybackSource::Predetermined(0),
        });
    }

    playback_choices.extend(additional_names.iter().enumerate().map(|(index, name)| {
        PlaybackChoice {
            label: (*name).to_owned(),
            source: PlaybackSource::Predetermined(index + 1),
        }
    }));

    playback_choices.extend(index.iter().map(|(id, display_name)| PlaybackChoice {
        label: display_name.to_owned(),
        source: PlaybackSource::Recorded(id),
    }));

    crate::log!(
        "runtime.playback_choices: {} choices",
        playback_choices.len()
    );
    playback_choices
}

fn estimated_frame_capacity(update_interval: Duration) -> usize {
    let interval_micros = effective_update_interval(update_interval).as_micros();
    let duration_micros = AUTONOMOUS_DURATION.as_micros();
    ((duration_micros + interval_micros - 1) / interval_micros + 1) as usize
}

fn effective_update_interval(interval: Duration) -> Duration {
    interval.max(Duration::from_millis(1))
}

fn prefixed_status(prefix: &str, suffix: &str) -> String {
    let mut text = String::with_capacity(prefix.len() + suffix.len());
    text.push_str(prefix);
    text.push_str(suffix);
    text
}

#[cfg(test)]
mod tests {
    use super::{RouteIndex, playback_choices};
    use crate::selector::PlaybackSource;

    #[test]
    fn predetermined_route_appears_before_saved_routes() {
        let mut index = RouteIndex::default();
        index.update(7, "Saved route");

        let choices = playback_choices(
            &index,
            Some("Move Off Wall"),
            &["Red Toggle", "Blue Toggle"],
        );
        assert_eq!(choices.len(), 5);
        assert_eq!(choices[0].source, PlaybackSource::Disabled);
        assert_eq!(choices[1].label, "Move Off Wall");
        assert_eq!(choices[1].source, PlaybackSource::Predetermined(0));
        assert_eq!(choices[2].label, "Red Toggle");
        assert_eq!(choices[2].source, PlaybackSource::Predetermined(1));
        assert_eq!(choices[3].label, "Blue Toggle");
        assert_eq!(choices[3].source, PlaybackSource::Predetermined(2));
        assert_eq!(choices[4].source, PlaybackSource::Recorded(7));
    }
}
