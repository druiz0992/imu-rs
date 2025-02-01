//! Module errors

/// Represents the different types of errors that can occur in the Phyphox library.
#[derive(Debug)]
pub enum PhyphoxError {
    /// Error indicating that the listener was not found.
    ListenerNotFound(String),

    /// Error indicating that there was an issue building the client.
    ClientBuild(String),

    /// Error indicating that there was an issue fetching data.
    FetchData(String),

    /// Error indicating that the received data format is incorrect.
    IncorrectDataFormat(String),

    Other(String),
}
