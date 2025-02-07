#[macro_export]
macro_rules! listener {
    ($handler:ident.$method:ident) => {
        Listener::new({
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
