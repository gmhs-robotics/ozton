use core::{future::Future, pin::Pin};
use std::time::Instant;

use autons::prelude::{SelectCompete, SelectCompeteExt};
use vexide::{prelude::*, time::sleep};

use super::{
    frame::{Frameable, RecordMode, Recordable, Recording},
    routes::RouteIndex,
    selector::{PlaybackChoice, RecordOption, RecordTarget, RecorderSelect, SelectionController},
};

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
#[derive(Debug, Default)]
pub struct RecordingSession<F: Frameable> {
    state: RecorderState<F>,
}

#[allow(dead_code)]
impl<F: Frameable> RecordingSession<F> {
    pub fn new() -> Self {
        crate::log!("runtime.recording_session.new");
        Self::default()
    }

    pub fn set_target(&mut self, target: RecordTarget) {
        crate::log!("runtime.recording_session.set_target: {:?}", target);
        self.state = match target {
            RecordTarget::Off => RecorderState::Idle,
            other => RecorderState::Recording {
                target: other,
                current: Recording::default(),
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

        crate::log!(
            "runtime.recording_session.push_frame: delta={}us frame={frame:?}",
            delta.as_micros()
        );
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
}

#[allow(dead_code)]
impl<R: Recordable + 'static> RecordingAutonomous<R> {
    pub async fn compete(robot: R, display: Display) -> ! {
        crate::log!("runtime.recording_autonomous.compete: start");
        let index = RouteIndex::load();

        let selector = RecorderSelect::new(
            display,
            record_options(&index),
            0,
            Self::arm_recording_callback,
        );

        let selection = SelectionController::new(selector.status_handle());
        let recorder = RecordingSession::new();

        Self {
            robot,
            index,
            recorder,
            selection,
        }
        .compete(selector)
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

        let display_name = self.index.display_name(route_id);
        let path = RouteIndex::path_for(route_id);

        if let Err(error) = recording.save(&path) {
            crate::log!(
                "runtime.recording_autonomous.save_recording: save failed path={} error={error}",
                path.display()
            );
            self.selection
                .status()
                .show_status(format!("Failed to save {display_name}"));
            return;
        }

        self.index.update(route_id, &display_name);
        if let Err(error) = self.index.save() {
            crate::log!(
                "runtime.recording_autonomous.save_recording: index save failed route={} error={error}",
                route_id
            );
            self.selection.status().show_status(format!(
                "Saved route, failed index update for {display_name}"
            ));
            return;
        }

        crate::log!(
            "runtime.recording_autonomous.save_recording: success route={} name={display_name}",
            route_id
        );
        self.selection
            .status()
            .show_status(format!("Saved {display_name}"));
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
                .show_status(format!("Armed: {}", option.label));
        }
    }

    async fn update_selection(&mut self) {
        if let Some(option) = self.selection.consume_selection_change() {
            self.arm_recording(option).await;
        }
    }

    fn arm_recording_callback(
        &mut self,
        option: RecordOption,
    ) -> Pin<Box<dyn Future<Output = ()> + '_>> {
        Box::pin(self.arm_recording(option))
    }
}

#[allow(dead_code)]
pub struct PlaybackAutonomous<R: Recordable + 'static> {
    pub robot: R,
    pub index: RouteIndex,
    active_route: Option<u32>,
    route_played_this_autonomous: bool,
    selection: SelectionController<PlaybackChoice>,
}

#[allow(dead_code)]
impl<R: Recordable + 'static> PlaybackAutonomous<R> {
    pub async fn compete(robot: R, display: Display) -> ! {
        crate::log!("runtime.playback_autonomous.compete: start");
        let index = RouteIndex::load();

        let selector = RecorderSelect::new(
            display,
            playback_choices(&index),
            0,
            Self::play_selected_callback,
        );
        let selection = SelectionController::new(selector.status_handle());

        Self {
            robot,
            index,
            active_route: None,
            route_played_this_autonomous: false,
            selection,
        }
        .compete(selector)
        .await;
    }

    async fn play_selected(&mut self, choice: PlaybackChoice) {
        crate::log!(
            "runtime.playback_autonomous.play_selected: label={} route_id={:?}",
            choice.label,
            choice.route_id
        );
        self.active_route = choice.route_id;
    }

    async fn update_selection(&mut self) {
        if let Some(choice) = self.selection.consume_selection_change() {
            self.play_selected(choice).await;
        }
    }

    fn play_selected_callback(
        &mut self,
        choice: PlaybackChoice,
    ) -> Pin<Box<dyn Future<Output = ()> + '_>> {
        Box::pin(self.play_selected(choice))
    }
}

impl<R: Recordable + 'static> SelectCompete for RecordingAutonomous<R> {
    async fn driver(&mut self) {
        crate::log!("runtime.recording_autonomous.driver: enter");
        loop {
            self.update_selection().await;

            let frame = self.robot.get_new_frame().await;
            crate::log!("runtime.recording_autonomous.driver: raw frame {frame:?}");
            if self.recorder.is_recording() {
                let finalized = self.robot.finalize_frame(&frame).await;
                crate::log!("runtime.recording_autonomous.driver: finalized frame {finalized:?}");
                self.recorder.push_frame(finalized);
            }

            if let Err(error) = self.robot.apply_frame(&frame, RecordMode::Live).await {
                crate::log!("runtime.recording_autonomous.driver: live apply error: {error:?}");
            }

            sleep(R::UPDATE_INTERVAL).await;
        }
    }

    async fn disabled(&mut self) {
        crate::log!("runtime.recording_autonomous.disabled");
        self.update_selection().await;

        if let Some((target, recording)) = self.recorder.finish() {
            self.save_recording(target, recording).await;
        }
    }
}

impl<R: Recordable + 'static> SelectCompete for PlaybackAutonomous<R> {
    async fn driver(&mut self) {
        self.route_played_this_autonomous = false;
        crate::log!("runtime.playback_autonomous.driver: enter");

        loop {
            self.update_selection().await;

            let frame = self.robot.get_new_frame().await;
            crate::log!("runtime.playback_autonomous.driver: live frame {frame:?}");

            if let Err(error) = self.robot.apply_frame(&frame, RecordMode::Live).await {
                crate::log!("runtime.playback_autonomous.driver: live apply error: {error:?}");
            }

            sleep(R::UPDATE_INTERVAL).await;
        }
    }

    async fn disabled(&mut self) {
        self.route_played_this_autonomous = false;
        crate::log!("runtime.playback_autonomous.disabled");
        self.update_selection().await;
    }

    async fn before_route(&mut self) {
        crate::log!(
            "runtime.playback_autonomous.before_route: active_route={:?} already_played={}",
            self.active_route,
            self.route_played_this_autonomous
        );
        self.update_selection().await;

        if self.route_played_this_autonomous {
            sleep(R::UPDATE_INTERVAL).await;
            return;
        }

        let Some(route_id) = self.active_route else {
            crate::log!("runtime.playback_autonomous.before_route: playback disabled");
            self.selection.status().show_status("Playback disabled");
            sleep(R::UPDATE_INTERVAL).await;
            return;
        };

        self.route_played_this_autonomous = true;

        let path = RouteIndex::path_for(route_id);
        let display_name = self.index.display_name(route_id);
        crate::log!(
            "runtime.playback_autonomous.before_route: loading route={} path={} name={display_name}",
            route_id,
            path.display()
        );

        if let Ok(recording) = Recording::load(&path) {
            self.selection
                .status()
                .show_status(format!("Playing {display_name}"));
            crate::log!("runtime.playback_autonomous.before_route: playback starting");
            recording.playback(&mut self.robot).await;
            return;
        }

        crate::log!(
            "runtime.playback_autonomous.before_route: route missing or failed to load"
        );
        self.selection
            .status()
            .show_status(format!("Missing route {display_name}"));
    }
}

fn record_options(index: &RouteIndex) -> Vec<RecordOption> {
    let options: Vec<_> = [
        RecordOption {
            label: "Record Off".to_owned(),
            target: RecordTarget::Off,
        },
        RecordOption {
            label: "Record New Route".to_owned(),
            target: RecordTarget::New,
        },
    ]
    .into_iter()
    .chain(index.entries().iter().map(|entry| RecordOption {
        label: format!("Record over {}", entry.display_name),
        target: RecordTarget::Overwrite(entry.id),
    }))
    .collect();
    crate::log!("runtime.record_options: {} options", options.len());
    options
}

fn playback_choices(index: &RouteIndex) -> Vec<PlaybackChoice> {
    let mut playback_choices = vec![PlaybackChoice {
        label: "Disable".to_string(),
        route_id: None,
    }];

    playback_choices.extend(index.entries().into_iter().map(|entry| PlaybackChoice {
        label: entry.display_name,
        route_id: Some(entry.id),
    }));

    crate::log!(
        "runtime.playback_choices: {} choices",
        playback_choices.len()
    );
    playback_choices
}
