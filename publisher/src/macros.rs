#[macro_export]
/// A macro to create a new `Listener` with a cloned handler and an asynchronous method call.
macro_rules! listener {
    ($handler:ident.$method:ident) => {
        Listener::new({
            let handler = $handler.clone(); // Clone the handler
            move |id, value| {
                handler.$method(id, value); // Call the method on the handler
            }
        })
    };
}
