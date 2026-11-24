//! Allocation-free execution timing. Histograms are approximate and inclusive.
use std::time::Duration;

#[derive(Clone, Copy, Debug)]
#[repr(usize)]
pub enum Stage {
    Tracking,
    Localization,
    LocalizationLog,
    MclPredict,
    MclObserve,
    MotionTick,
    ControlInterval,
    Planner,
    HardwareReport,
    SdFlush,
}
const STAGES: [Stage; 10] = [
    Stage::Tracking, Stage::Localization, Stage::LocalizationLog, Stage::MclPredict,
    Stage::MclObserve, Stage::MotionTick, Stage::ControlInterval, Stage::Planner,
    Stage::HardwareReport, Stage::SdFlush,
];
const BOUNDS: [u64; 12] = [10, 25, 50, 100, 250, 500, 1000, 2000, 5000, 10000, 20000, 50000];

#[derive(Clone, Copy)]
struct Stats {
    count: u64,
    total: u64,
    max: u64,
    over_budget: u64,
    histogram: [u64; BOUNDS.len() + 1],
}

impl Stats {
    const EMPTY: Self = Self { count: 0, total: 0, max: 0, over_budget: 0, histogram: [0; BOUNDS.len() + 1] };

    fn add(&mut self, micros: u64, budget: u64) {
        self.count = self.count.saturating_add(1);
        self.total = self.total.saturating_add(micros);
        self.max = self.max.max(micros);
        self.over_budget += u64::from(micros > budget);
        self.histogram[BOUNDS.partition_point(|bound| *bound < micros)] += 1;
    }

    fn p95_upper(&self) -> u64 {
        let target = self.count.saturating_mul(95).div_ceil(100);
        let mut count = 0;
        for (i, n) in self.histogram.iter().enumerate() {
            count += n;
            if count >= target {
                return BOUNDS.get(i).copied().unwrap_or(self.max).min(self.max);
            }
        }
        self.max
    }
}

#[cfg(feature = "logging")]
static STATS: std::sync::Mutex<[Stats; STAGES.len()]> = std::sync::Mutex::new([Stats::EMPTY; STAGES.len()]);

/// Measure synchronous work, never hold a span across an await unless wall time is intended.
#[must_use]
pub struct Span {
    #[cfg(feature = "logging")]
    started: Option<std::time::Instant>,
    #[cfg(feature = "logging")]
    stage: Stage,
}

impl Span {
    #[inline]
    pub fn new(stage: Stage) -> Self {
        #[cfg(not(feature = "logging"))]
        let _ = stage;
        Self {
            #[cfg(feature = "logging")]
            started: crate::serial::sd_enabled().then(std::time::Instant::now),
            #[cfg(feature = "logging")]
            stage,
        }
    }
}

impl Drop for Span {
    #[inline]
    fn drop(&mut self) {
        #[cfg(feature = "logging")]
        if let Some(started) = self.started {
            record(self.stage, started.elapsed(), Duration::from_millis(10));
        }
    }
}

/// Record observed period or work against its own deadline.
#[inline]
pub fn record(stage: Stage, elapsed: Duration, budget: Duration) {
    #[cfg(feature = "logging")]
    if crate::serial::sd_enabled() && let Ok(mut stats) = STATS.try_lock() {
        stats[stage as usize].add(
            elapsed.as_micros().min(u128::from(u64::MAX)) as u64,
            budget.as_micros().min(u128::from(u64::MAX)) as u64,
        );
    }
    #[cfg(not(feature = "logging"))]
    let _ = (stage, elapsed, budget);
}

/// Emit one summary per active stage, then start a new window.
pub fn flush_summary() {
    #[cfg(feature = "logging")]
    {
        if !crate::serial::sd_enabled() { return; }
        let snapshot = {
            let Ok(mut stats) = STATS.try_lock() else { return; };
            core::mem::replace(&mut *stats, [Stats::EMPTY; STAGES.len()])
        };
        for (stage, stats) in STAGES.into_iter().zip(snapshot) {
            if stats.count == 0 { continue; }
            crate::log_row!(Trace, "PROFILE", "stage={stage:?},count={},total_us={},mean_us={:.2},max_us={},p95_upper_us={},over_budget={}",
                stats.count, stats.total, stats.total as f64 / stats.count as f64,
                stats.max, stats.p95_upper(), stats.over_budget);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn percentile_and_deadline_counts() {
        let mut s = Stats::EMPTY;
        for _ in 0..95 { s.add(120, 1000); }
        for _ in 0..5 { s.add(12000, 1000); }
        assert_eq!(s.count, 100);
        assert_eq!(s.max, 12000);
        assert_eq!(s.p95_upper(), 250);
        assert_eq!(s.over_budget, 5);
    }
}
