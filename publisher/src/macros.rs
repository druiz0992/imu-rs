#[macro_export]
/// A macro to create a new `AsyncListener` with a cloned handler and an asynchronous method call.
macro_rules! listener {
    ($handler:ident.$method:ident) => {
        AsyncListener::new({
            let handler = $handler.clone(); // Clone the handler
            move |id, value| {
                let handler = handler.clone(); // Clone inside the closure
                async move {
                    handler.$method(id, value).await; // Call the method on the handler
                }
            }
        })
    };
}
