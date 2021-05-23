# AiRadar device

### Run on development mode

The main file of the device is `main_controller.ino`. Open the file on Arduino IDE then installing require libraries.
Some libraries are already register with the Ardunio Library Manager. SOme need to downloading from external source based on the device specificaiton.

The WiFi will be config in the `main_controller.ino` file. Changing the `SSID` and `Password` of WiFi to match the local WiFi around the device.

Changing the server IP or domain to match the usage `Server IP: 139.59.126.32, Server Port: 8080` or changing the Server IP and Port on difference hosting platform for `Blynk`.
The `auth[]` is a device key which provide by `Blynk` platform on difference project. Changing to matching the server domain.
