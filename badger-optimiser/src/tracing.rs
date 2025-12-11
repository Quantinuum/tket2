//! Setup routines for tracing and logging of the optimisation process.
use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;

use tket::optimiser::badger::log::{LOG_TARGET, METRICS_TARGET, PROGRESS_TARGET};

use tracing::{Metadata, Subscriber};
use tracing_appender::non_blocking;
use tracing_subscriber::filter::filter_fn;
use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use tracing_subscriber::Layer;

fn log_filter(metadata: &Metadata<'_>) -> bool {
    metadata.target().starts_with(LOG_TARGET)
}

fn verbose_filter(metadata: &Metadata<'_>) -> bool {
    metadata.target().starts_with(LOG_TARGET) || metadata.target().starts_with(PROGRESS_TARGET)
}

#[allow(unused)]
fn metrics_filter(metadata: &Metadata<'_>) -> bool {
    metadata.target().starts_with(METRICS_TARGET)
}

#[derive(Debug, Default)]
pub struct Tracer {
    pub logfile: Option<non_blocking::WorkerGuard>,
}

impl Tracer {
    /// Setup tracing subscribers for stdout and file logging.
    pub fn setup_tracing(logfile: Option<PathBuf>, show_threads: bool) -> Self {
        let mut tracer = Self::default();
        tracing_subscriber::registry()
            .with(tracer.stdout_layer(show_threads))
            .with(logfile.map(|f| tracer.logfile_layer(f, show_threads)))
            .init();
        tracer
    }

    /// Initialize a file logger handle and non-blocking worker.
    fn init_writer(&self, file: PathBuf) -> (non_blocking::NonBlocking, non_blocking::WorkerGuard) {
        let writer = BufWriter::new(File::create(file).unwrap());
        non_blocking(writer)
    }

    /// Clean log with the most important events.
    fn stdout_layer<S>(&mut self, show_threads: bool) -> impl Layer<S> + use<S>
    where
        S: Subscriber + for<'span> tracing_subscriber::registry::LookupSpan<'span>,
    {
        tracing_subscriber::fmt::layer()
            .without_time()
            .with_target(false)
            .with_level(false)
            .with_thread_names(show_threads)
            .with_filter(filter_fn(log_filter))
    }

    fn logfile_layer<S>(&mut self, logfile: PathBuf, show_threads: bool) -> impl Layer<S> + use<S>
    where
        S: Subscriber + for<'span> tracing_subscriber::registry::LookupSpan<'span>,
    {
        let (non_blocking, guard) = self.init_writer(logfile);
        self.logfile = Some(guard);
        tracing_subscriber::fmt::layer()
            .with_ansi(false)
            .with_thread_names(show_threads)
            .with_writer(non_blocking)
            .with_filter(filter_fn(verbose_filter))
    }
}
