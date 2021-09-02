# ESP-Now-Serial-Bridge

ESP32 based serial bridge for transmitting serial data between two boards

The primary purpose of this sketch was to enable a MAVLink serial connection, which I successfully tested at 57600 bps.  In theory, much faster buad rates should work fine, but I have not tested faster than 115200.
 
Range is easily better than regular WiFi, however an external antenna may be required for truly long range messaging, to combat obstacles/walls, and/or to achieve success in an area saturated with 2.4GHz traffic.

To find the MAC address of each board, uncomment the `#define DEBUG` line, and monitor serial output on boot.  Set the OPPOSITE board's IP address as the value for RECVR_MAC in the macros at the top of the sketch.

When uploading the sketch, be sure to define `BOARD1` or `BOARD2` as appropriate before compiling.

-- Yuri - Sep 2021

Based this example - https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

### License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.