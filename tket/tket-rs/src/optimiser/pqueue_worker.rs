//! A priority queue of states, running in a dedicated thread.
//!
//! Communication with other threads is achieved over multi-producer
//! multi-consumer channels.

use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Instant;

use crossbeam_channel::{Receiver, RecvError, SendError, Sender, select};

use crate::circuit::cost::CircuitCost;

use crate::optimiser::pqueue::{Entry, StatePQueue};

/// A unit of work for a worker, consisting of a state to process, along its
/// hash and cost.
pub type Work<S, P> = Entry<S, P, u64>;

/// A priority queue [`StatePQueue`] running in a dedicated thread.
///
/// When initiated, provides channels that other threads and workers can use to
/// add and remove states from the queue.
#[derive(Debug, Clone)]
pub struct StatePQWorker<S, P: Ord> {
    /// Channel to add states to the queue.
    push: Receiver<Vec<Work<S, P>>>,
    /// Channel to pop states from the queue.
    pop: Sender<Work<S, P>>,
    /// Outbound channel to log to main thread.
    log: Sender<LogMessage<S, P>>,
    /// Timestamp of the last progress log.
    /// Used to avoid spamming the log.
    last_progress_log: Instant,
    /// The priority queue data structure.
    pq: StatePQueue<S, P>,
    /// The minimum cost we've seen.
    min_cost: Option<P>,
    /// The number of states we've processed.
    state_cnt: usize,
    /// The maximum cost in the queue. Shared with the workers so they can cull
    /// the states they generate.
    max_cost: Arc<RwLock<Option<P>>>,
    /// Local copy of `max_cost`, used to avoid locking when checking the value.
    local_max_cost: Option<P>,
}

/// Logging information from the priority channel.
#[derive(Debug, Clone)]
pub enum LogMessage<S, P> {
    NewBestState(S, P),
    StateCount {
        processed_count: usize,
        seen_count: usize,
        queue_length: usize,
    },
}

/// Channels for communication with the priority queue of [`StatePQWorker`].
#[derive(Clone)]
pub struct StatePQueueChannels<S, P> {
    /// A channel to add batches of states to the queue.
    push: Sender<Vec<Work<S, P>>>,
    /// A channel to remove the best candidate state from the queue.
    pop: Receiver<Work<S, P>>,
    /// A maximum accepted cost for the queue. States with higher costs will
    /// be dropped.
    ///
    /// Shared with the workers so they can cull the states they generate.
    max_cost: Arc<RwLock<Option<P>>>,
}

impl<S, P: CircuitCost> StatePQueueChannels<S, P> {
    /// Signal the priority channel to stop.
    ///
    /// This will in turn signal the workers to stop.
    pub fn close(&self) -> Result<(), SendError<Vec<Work<S, P>>>> {
        self.push.send(Vec::new())
    }

    /// Send a lot of states to the priority channel.
    pub fn send(&self, work: Vec<Work<S, P>>) -> Result<(), SendError<Vec<Work<S, P>>>> {
        if work.is_empty() {
            return Ok(());
        }
        self.push.send(work)
    }

    /// Receive a state from the priority channel.
    ///
    /// Blocks until a state is available.
    pub fn recv(&self) -> Result<Work<S, P>, RecvError> {
        self.pop.recv()
    }

    /// Get the maximum accepted state cost.
    ///
    /// This function requires locking, so its value should be cached where
    /// appropriate.
    pub fn max_cost(&self) -> Option<P> {
        self.max_cost.read().as_deref().ok().cloned().flatten()
    }
}

impl<S, P> StatePQWorker<S, P>
where
    P: CircuitCost + Send + Sync + 'static,
    S: Clone + Send + Sync + 'static,
{
    /// Initialize the queueing system.
    ///
    /// Start the state priority queue in a new thread.
    ///
    /// Get back a [`StatePQueueChannels`] for adding and removing states
    /// to/from the queue, and a channel receiver to receive logging
    /// information.
    pub fn init(queue_capacity: usize) -> (StatePQueueChannels<S, P>, Receiver<LogMessage<S, P>>) {
        // Shared maximum cost in the queue.
        let max_cost = Arc::new(RwLock::new(None));
        // Channels for pushing and popping states from pqueue
        let (tx_push, rx_push) = crossbeam_channel::unbounded();
        let (tx_pop, rx_pop) = crossbeam_channel::bounded(0);
        // Channel for logging results and statistics to the main thread.
        let (tx_log, rx_log) = crossbeam_channel::unbounded();

        let pq = Self::new(rx_push, tx_pop, tx_log, max_cost.clone(), queue_capacity);
        pq.run();
        (
            StatePQueueChannels {
                push: tx_push,
                pop: rx_pop,
                max_cost,
            },
            rx_log,
        )
    }

    fn new(
        push: Receiver<Vec<Work<S, P>>>,
        pop: Sender<Work<S, P>>,
        log: Sender<LogMessage<S, P>>,
        max_cost: Arc<RwLock<Option<P>>>,
        queue_capacity: usize,
    ) -> Self {
        // The priority queue, local to this thread.
        let pq = StatePQueue::new(queue_capacity, None);
        // The minimum cost we've seen.
        let min_cost = None;
        // The number of states we've seen (for logging).
        let state_cnt = 0;

        StatePQWorker {
            push,
            pop,
            log,
            // Ensure we log the first progress.
            last_progress_log: Instant::now() - std::time::Duration::from_secs(60),
            pq,
            min_cost,
            state_cnt,
            max_cost,
            local_max_cost: None,
        }
    }

    /// Run the queuer as a thread.
    fn run(mut self) {
        let builder = thread::Builder::new().name("priority queueing".into());
        let _ = builder
            .name("priority-channel".into())
            .spawn(move || {
                'main: loop {
                    while self.pq.is_empty() {
                        let Ok(new_circs) = self.push.recv() else {
                            // Something went wrong
                            break 'main;
                        };
                        if new_circs.is_empty() {
                            // The main thread signalled us to stop.
                            break 'main;
                        }
                        self.enqueue_circs(new_circs);
                    }
                    select! {
                        recv(self.push) -> result => {
                            let Ok(new_circs) = result else {
                                // Something went wrong
                                break 'main;
                            };
                            if new_circs.is_empty() {
                                // The main thread signalled us to stop.
                                break 'main;
                            }
                            self.enqueue_circs(new_circs);
                        }
                        send(self.pop, self.pq.pop().unwrap()) -> result => {
                            if result.is_err() {
                                // Something went wrong.
                                break 'main;
                            }
                            self.update_max_cost();
                        }
                    }
                }
                // Send a last set of logs before terminating.
                self.log
                    .send(LogMessage::StateCount {
                        processed_count: self.state_cnt,
                        seen_count: self.pq.num_seen_hashes(),
                        queue_length: self.pq.len(),
                    })
                    .unwrap();
            })
            .unwrap();
    }

    /// Add states to queue.
    #[tracing::instrument(target = "badger::metrics", skip(self, states))]
    fn enqueue_circs(&mut self, states: Vec<Work<S, P>>) {
        for Work { cost, hash, state } in states {
            // A new best state
            if self.min_cost.is_none() || Some(&cost) < self.min_cost.as_ref() {
                self.min_cost = Some(cost.clone());
                self.log
                    .send(LogMessage::NewBestState(state.clone(), cost.clone()))
                    .unwrap();
            }

            self.pq.push_unchecked(state, hash, cost);
        }
        self.update_max_cost();

        // This is the result from processing a state. Add it to the count.
        self.state_cnt += 1;
        if Instant::now() - self.last_progress_log > std::time::Duration::from_millis(100) {
            self.log
                .send(LogMessage::StateCount {
                    processed_count: self.state_cnt,
                    seen_count: self.pq.num_seen_hashes(),
                    queue_length: self.pq.len(),
                })
                .unwrap();
        }
    }

    /// Update the shared `max_cost` value.
    ///
    /// If the priority queue is full, set the `max_cost` to the maximum cost.
    /// Otherwise, leave it as `None`.
    #[inline]
    fn update_max_cost(&mut self) {
        if !self.pq.is_full() || self.pq.is_empty() {
            return;
        }
        let queue_max = self.pq.max_cost().unwrap().clone();
        let local_max = self.local_max_cost.clone();
        if local_max.is_some() && queue_max < local_max.unwrap() {
            self.local_max_cost = Some(queue_max.clone());
            *self.max_cost.write().unwrap() = Some(queue_max);
        }
    }
}
