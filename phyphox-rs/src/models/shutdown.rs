use log::{error, info};
use std::sync::Arc;
use tokio::signal;
use tokio::sync::Notify;

const SHUTDOWN_PERIOD_MS: u64 = 100;
const MAX_RETRIES: u64 = 10;

pub(crate) struct ShutdownSignal {
    notify: Arc<Notify>,
}

impl ShutdownSignal {
    fn new(notify: Arc<Notify>) -> Self {
        Self { notify }
    }

    async fn listen_for_shutdown(&self, run_for_millis: Option<u64>) {
        let mut retries = 0;
        loop {
            if let Some(time_to_live_millis) = run_for_millis {
                tokio::time::sleep(std::time::Duration::from_millis(time_to_live_millis)).await;
            } else {
                if let Err(e) = signal::ctrl_c().await {
                    error!("Error while waiting for Ctrl+C: {}", e);
                    continue;
                }
                info!("Ctrl+C received. Sending stop signal...");
            }
            self.notify.notify_waiters();
            tokio::time::sleep(std::time::Duration::from_millis(SHUTDOWN_PERIOD_MS)).await;
            retries += 1;
            if retries >= MAX_RETRIES {
                info!("Maximum retries reached, giving up on shutdown signal");
                break;
            }
        }
    }
}

pub(crate) fn listen_for_shutdown(
    notify: Arc<Notify>,
    run_for_millis: Option<u64>,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let shutdown_signal = ShutdownSignal::new(notify);
        shutdown_signal.listen_for_shutdown(run_for_millis).await;
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::time::{timeout, Duration};

    #[tokio::test]
    async fn test_shutdown_signal_with_timeout() {
        let notify = Arc::new(Notify::new());
        let shutdown_signal = ShutdownSignal::new(notify.clone());

        let handle = tokio::spawn(async move {
            shutdown_signal.listen_for_shutdown(Some(200)).await;
        });

        // Wait for the notify to be called
        let result = timeout(Duration::from_millis(300), notify.notified()).await;
        assert!(result.is_ok(), "Shutdown signal was not received in time");

        handle.abort();
    }
}
