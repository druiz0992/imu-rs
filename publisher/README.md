 # Crate publisher

 ## publisher

 The `publisher` crate provides a mechanism for registering and notifying listeners
 of new events of type `T`.

 This crate is designed to handle dynamic registration of callback functions (`Fn(T)`) as listeners,
 ensuring that all registered listeners receive updates when an event occurs.

 ### Example

 ```rust
 fn main() {
     let publisher = Arc::new(Publisher::new());

     // Register a listener
     let listener_id = publisher.register(|data: String| {
         println!("Listener received: {}", data);
     });

     // Notify all listeners
     publisher.notify("Hello, World!".to_string());

     // Unregister the listener
     publisher.unregister(listener_id).expect("Failed to unregister listener");

     // Verify that no listeners are left
     assert!(publisher.is_empty());
 }
 ```