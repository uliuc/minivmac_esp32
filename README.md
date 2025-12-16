This is a port of the minivmac project to an ESP32S3. minivmac (https://www.gryphel.com/c/minivmac/) is a Macintosh emulator for early Macs. The emulator was ported to an ESP32S3 board with a display from Waveshare (https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8B). The port works very well and is fast enough to actually use the Mac, if it weren't for the very small display. 2.8 inches is very small, and you need very good eyes to see everything. 

Usage:
- The sources must be compiled with the IDF from Espressif (version 5.5). (idf.py build).
- A MacPlus ROM (file name: vMac.ROM) and various disks (disk1.dsk to disk6.dsk) must be placed in the spiffs directory. The ROM is required for the Macintosh to start (firmware). The disks appear on the desktop after the operating system has started. disk1.dsk should, of course, be a bootable MacOS (e.g., System 7.x) startup disk.
- Once the files are in the spiffs directory, everything can be flashed.
- idf.py flash
- The emulator then starts directly.
- For mouse and keyboard support (see next chapter)

Mouse and keyboard:

The mouse and keyboard inputs are made via Bluetooth. Unfortunately, the ESP32S3 cannot use classic Bluetooth, only BLE. However, since I don't have any BLE mice or keyboards, the support was solved a little differently. I use a cheap ESP32 (not S3) board and run firmware on it that connects to all HID input devices in the vicinity (they must be in pairing mode to start). Once the devices are connected, all HID events (i.e., all mouse movements or keyboard inputs) are output via the serial interface (UART). This ESP32 board is connected to the Waveshare display via UART (4 cables) (GND -> GND, 3.3V -> 3.3V, ESP32 RX -> ESP32 TX, and ESP32 TX -> ESP32 RX). The ESP32 receives the data from the ESP32 and forwards it to the emulator (see the process in the video). 
The Bluetooth firmware for the ESP32 is located in a separate project. ()
