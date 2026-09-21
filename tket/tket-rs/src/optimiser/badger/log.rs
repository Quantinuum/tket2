//! Logging utilities for the Badger optimiser.

use std::time::{Duration, Instant};
use std::{fmt::Debug, io};

/// Logging configuration for the Badger optimiser.
pub struct BadgerLogger<'w> {
    circ_candidates_csv: Option<csv::Writer<Box<dyn io::Write + Send + Sync + 'w>>>,
    last_circ_processed: usize,
    last_progress_time: Instant,
    branching_factor: UsizeAverage,
}

impl Default for BadgerLogger<'_> {
    fn default() -> Self {
        Self {
            circ_candidates_csv: Default::default(),
            last_circ_processed: Default::default(),
            // Ensure the first progress message is printed.
            last_progress_time: Instant::now() - Duration::from_secs(60),
            branching_factor: UsizeAverage::new(),
        }
    }
}

/// The logging target for general events.
pub const LOG_TARGET: &str = "badger::log";
/// The logging target for progress events. More verbose than the general log.
pub const PROGRESS_TARGET: &str = "badger::progress";
/// The logging target for function spans.
pub const METRICS_TARGET: &str = "badger::metrics";

impl<'w> BadgerLogger<'w> {
    /// Create a new logging configuration.
    ///
    /// Two writer objects must be provided:
    /// - best_progress_csv_writer: for a log of the successive best candidate
    ///   circuits,
    ///
    /// Regular events are logged with [`tracing`], with targets [`LOG_TARGET`]
    /// or [`PROGRESS_TARGET`].
    ///
    /// [`log`]: <https://docs.rs/log/latest/log/>
    pub fn new(best_progress_csv_writer: impl io::Write + Send + Sync + 'w) -> Self {
        let boxed_candidates_writer: Box<dyn io::Write + Send + Sync + 'w> =
            Box::new(best_progress_csv_writer);
        Self {
            circ_candidates_csv: Some(csv::Writer::from_writer(boxed_candidates_writer)),
            ..Default::default()
        }
    }

    /// Log a new best candidate
    #[inline]
    pub fn log_best<C: Debug + serde::Serialize>(
        &mut self,
        best_cost: C,
        num_rewrites: Option<usize>,
    ) {
        match num_rewrites {
            Some(rs) => self.log(format!(
                "new best of size {best_cost:?} after {rs} rewrites"
            )),
            None => self.log(format!("new best of size {best_cost:?}")),
        }
        if let Some(csv_writer) = self.circ_candidates_csv.as_mut() {
            csv_writer.serialize(BestCircSer::new(best_cost)).unwrap();
            csv_writer.flush().unwrap();
        };
    }

    /// Log the final optimised circuit
    #[inline]
    pub fn log_processing_end<C: Debug>(
        &self,
        circuits_processed: usize,
        circuits_seen: Option<usize>,
        best_cost: C,
        needs_joining: bool,
        timeout: bool,
        elapsed_time: Duration,
    ) {
        let elapsed_secs = elapsed_time.as_secs_f32();
        match timeout {
            true => self.log(format!(
                "Optimisation finished in {elapsed_secs:.2}s (timeout)."
            )),
            false => self.log(format!("Optimisation finished in {elapsed_secs:.2}s.")),
        };
        match circuits_seen {
            Some(circuits_seen) => self.log(format!(
                "Processed {circuits_processed} circuits (out of {circuits_seen} seen)."
            )),
            None => self.log(format!("Processed {circuits_processed} circuits.")),
        }
        self.log_avg_branching_factor();
        self.log(format!("---- END RESULT: {best_cost:?} ----"));
        if needs_joining {
            self.log("Joining worker threads.");
        }
    }

    /// Log the progress of the optimisation.
    #[inline(always)]
    pub fn log_progress(
        &mut self,
        circuits_processed: usize,
        workqueue_len: Option<usize>,
        seen_hashes: usize,
    ) {
        if circuits_processed > self.last_circ_processed
            && Instant::now() - self.last_progress_time > Duration::from_secs(1)
        {
            self.last_circ_processed = circuits_processed;
            self.last_progress_time = Instant::now();

            self.progress(format!("Processed {circuits_processed} circuits..."));
            if let Some(workqueue_len) = workqueue_len {
                self.progress(format!("Queue size: {workqueue_len} circuits."));
            }
            self.progress(format!("Total seen: {seen_hashes} circuits."));
        }
    }

    /// Log general events, normally printed to stdout.
    #[inline]
    pub fn log(&self, msg: impl AsRef<str>) {
        tracing::info!(target: LOG_TARGET, "{}", msg.as_ref());
    }

    /// Log a warning message.
    #[inline]
    pub fn warn(&self, msg: impl AsRef<str>) {
        tracing::warn!(target: LOG_TARGET, "{}", msg.as_ref());
    }

    /// Log verbose information on the progress of the optimization.
    #[inline]
    pub fn progress(&self, msg: impl AsRef<str>) {
        tracing::info!(target: PROGRESS_TARGET, "{}", msg.as_ref());
    }

    /// Append a new branching factor to the average.
    pub fn register_branching_factor(&mut self, branching_factor: usize) {
        self.branching_factor.append(branching_factor);
    }

    /// Log the average branching factor so far.
    pub fn log_avg_branching_factor(&self) {
        if let Some(avg) = self.branching_factor.average() {
            self.log(format!("Average branching factor: {avg}"));
        }
    }
}

/// A helper struct for logging improvements in circuit size seen during the
/// Badger execution.
//
// TODO: Replace this fixed logging. Report back intermediate results.
#[derive(serde::Serialize, Clone, Debug)]
struct BestCircSer<C> {
    circ_cost: C,
    time: String,
}

impl<C> BestCircSer<C> {
    fn new(circ_cost: C) -> Self {
        let time = chrono::Local::now().to_rfc3339();
        Self { circ_cost, time }
    }
}

struct UsizeAverage {
    sum: usize,
    count: usize,
}

impl UsizeAverage {
    pub fn new() -> Self {
        Self {
            sum: Default::default(),
            count: 0,
        }
    }

    pub fn append(&mut self, value: usize) {
        self.sum += value;
        self.count += 1;
    }

    pub fn average(&self) -> Option<f64> {
        if self.count > 0 {
            Some(self.sum as f64 / self.count as f64)
        } else {
            None
        }
    }
}
