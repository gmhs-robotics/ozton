//! Bounded odometry history for acquisition-time range alignment.
use std::{collections::VecDeque, time::Duration};

use ozton_math::geometry::{Pose, wrap_heading};

const HISTORY_CAPACITY: usize = 128;
const HISTORY_AGE: Duration = Duration::from_millis(500);

/// Store only uncorrected odometry so a map correction cannot be mistaken
/// for physical travel between the measurement and current filter state.
pub(super) struct PoseHistory {
    samples: VecDeque<(Duration, Pose)>,
}

impl Default for PoseHistory {
    fn default() -> Self {
        Self {
            samples: VecDeque::with_capacity(HISTORY_CAPACITY),
        }
    }
}

impl PoseHistory {
    pub(super) fn clear(&mut self) {
        self.samples.clear();
    }

    pub(super) fn record(&mut self, time: Duration, pose: Pose) {
        while self.samples.len() >= HISTORY_CAPACITY
            || self
                .samples
                .front()
                .is_some_and(|&(old, _)| time.saturating_sub(old) > HISTORY_AGE)
        {
            self.samples.pop_front();
        }
        self.samples.push_back((time, pose));
    }

    fn at(&self, time: Duration) -> Option<Pose> {
        let &(first, _) = self.samples.front()?;
        let &(last, _) = self.samples.back()?;
        if time < first || time > last {
            return None;
        }
        let upper = self.samples.partition_point(|&(stamp, _)| stamp < time);
        let &(after, pose) = self.samples.get(upper)?;
        if after == time {
            return Some(pose);
        }
        let &(before, prior) = self.samples.get(upper.checked_sub(1)?)?;
        let elapsed = time.checked_sub(before)?;
        let interval = after.checked_sub(before)?;
        let fraction = elapsed.div_duration_f64(interval);
        Some(prior.interpolate(pose, fraction))
    }

    /// Express the physical sensor pose at acquisition in today's chassis
    /// frame. Raycasting this virtual mount at a current particle is equivalent
    /// to moving that particle backward by the measured relative odometry.
    pub(super) fn sensor_mount(&self, time: Duration, current: Pose, mount: Pose) -> Option<Pose> {
        let past = self.at(time)?;
        if past == current {
            return Some(Pose::new(mount.position, wrap_heading(mount.heading)));
        }
        Some(Pose::new(
            current.to_local(past.transform_point(mount.position)),
            wrap_heading(past.heading + mount.heading - current.heading),
        ))
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::{geometry::DVec2, qty};

    use super::*;

    #[test]
    fn delayed_beam_keeps_its_world_origin_and_bearing() {
        let mut history = PoseHistory::default();
        let past = Pose::default();
        let current = Pose::new(DVec2::new(0.1, 0.2), qty!(90 deg));
        history.record(Duration::from_millis(10), past);
        history.record(Duration::from_millis(30), current);
        let mount = Pose::new(DVec2::new(0.04, 0.08), qty!(-90 deg));
        let aligned = history
            .sensor_mount(Duration::from_millis(10), current, mount)
            .unwrap();
        assert!(
            current
                .transform_point(aligned.position)
                .distance(mount.position)
                < 1e-12
        );
        assert!(
            wrap_heading(current.heading + aligned.heading - mount.heading)
                .value
                .abs()
                < 1e-12
        );
        assert!(history.at(Duration::from_millis(5)).is_none());
        assert!(history.at(Duration::from_millis(40)).is_none());
    }
}
