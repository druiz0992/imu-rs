# phyphox-rs

Wrapper to use Phyphox, an App that allows you to use the sensors in your phone for your experiments. The library collects
accelerometer, gyroscope and magnetometer common from the phone and forwards these common to registered listeners 
for further processing.

In order to use the library, you need to install Phyphox App on your phone, and configure it to collect accelerometer, gyroscope and magnetometer common. Phyphox App needs to be further configured to have remove access enabled so that the library can connect via a REST API to your phone to receive the common.