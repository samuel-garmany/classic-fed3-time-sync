# Classic FED3 with FNT Support

> [!WARNING]
> This software has only been tested on the FED 3.2 revision, your milage may vary when attempting to use a different model.

This repository contains modified firmware for the **Feeding Experimentation Device 3 (FED3)**, customized to enable real-time event tracking, automated clock synchronization, and on-the-fly behavioral protocol switching when paired with the [FieldNeuroToolbox (FNT)](https://github.com/calebvogt/fnt).

This sketch is modified from the original `ClassicFED3` script from the official [KravitzLabDevices/FED3_library](https://github.com/KravitzLabDevices/FED3_library).

## Installation & Setup

**System & IDE Setup:** Follow the official [FED3 Setup Guide](https://github.com/KravitzLabDevices/FED3_library/wiki/Get-started) to configure your computer and Arduino IDE.

The modified sketch is located at:
* [ClassicFed3withTimeSync.ino](file:///home/user/Documents/Donaldson%20Lab/classic-fed3-time-sync/ClassicFed3withTimeSync/ClassicFed3withTimeSync.ino)

> [!NOTE]
> Because these changes require a continuous wired USB connection to a host computer running FNT, sleep mode is disabled in this firmware. This will increase battery consumption.

## Key Modifications

Compared to the standard Classic FED3 firmware, this version introduces:
1. **Disabled Sleep Mode:** Disables the standard FED3 low-power sleep state to ensure the microcontroller remains active and responsive to incoming serial messages.
2. **Serial Connection Setup:** Initializes a serial connection at `115200` baud.
3. **Live Event Tracking:** Automatically broadcasts nosepoke and pellet dispensing events over the serial port as they occur.
4. **Interactive Time Synchronization:** Adds a protocol to sync the FED3's real-time clock (RTC) dynamically via serial commands.


## Serial Communication Protocol

The firmware communicates using simple text-based commands over the serial interface:

### 1. Auto-Discovery & Handshake
* **Query:** `PING`
* **Response:** `PONG_FED3`
* *Purpose:* Allows the FNT GUI to automatically detect and identify connected FED3 devices on active COM ports.

### 2. Time Synchronization
* **Command:** `SYNC:YYYY,MM,DD,HH,MM,SS` (e.g., `SYNC:2026,06,03,13,15,00`)
* **Response:** `Time synced successfully.`
* *Action:* Sets the RTC (`RTC_PCF8523`) to the specified date and time, and triggers a display redraw (`fed3.UpdateDisplay()`) to show the updated time on the OLED screen.

### 3. Live Tracking Broadcasts
When events occur, the FED3 outputs the following strings instantly:
* **Left Nosepoke:** `Left Poke, Total: <count>`
* **Right Nosepoke:** `Right Poke, Total: <count>`
* **Pellet Dispensing:** `Pellet Dispensed, Total: <count>`
