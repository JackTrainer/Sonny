use std::error::Error;

use SONNY::chaos::chaos_test;

#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    chaos_test::run().await
}
