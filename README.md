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
3. **Live Event Tracking:** Automatically broadcasts nosepoke and pellet dispensing events over the serial port as they occur, timestamped from the device RTC.
4. **Interactive Time Synchronization:** Adds a protocol to sync the FED3's real-time clock (RTC) dynamically via serial commands.
5. **Non-blocking SD card transfers:** Log files stream to the host a slice at a time, so the behavioural task keeps running during a download.

### Protocol 2.0 — why the transfer path changed

Protocol 1.x pushed an entire CSV inside the serial command handler. That starved
`fed3.run()` for the length of the transfer, so the display froze and nosepokes
went unlogged. Worse, `Serial.write()` on the SAMD21 USB CDC endpoint **blocks
indefinitely** once the host stops draining the buffer — any interruption on the
host side (a cancelled download, a busy GUI, a transfer timeout) left the device
wedged until it was power-cycled, with the trial's data recoverable only by
pulling the SD card by hand.

In 2.0, transfers are a state machine serviced once per `loop()`. Each pass moves
at most 64 bytes, and only as many as the USB buffer will accept right now, so
`loop()` always returns. If the host stops reading for 15 seconds the device
aborts the transfer by itself and resumes normal operation. Transfers are also
range-based, so an interrupted download resumes from a byte offset instead of
starting over.

> [!IMPORTANT]
> FNT reports the firmware version reported by `PING`. Devices running 1.x still
> stream live events, but SD-card mirroring and resumable transfers require this
> firmware.

## Serial Communication Protocol

Text commands, newline terminated, at 115200 baud.

### Host → device

| Command | Purpose |
| --- | --- |
| `PING` | Auto-discovery handshake |
| `STATUS` | Full device state snapshot |
| `SYNC:YYYY,MM,DD,HH,MM,SS` | Set the RTC (`RTC_PCF8523`) and redraw the display |
| `LIST_FILES` | Enumerate `FED*.CSV` on the SD card |
| `FSIZE:<name>` | Current size of one file, without transferring it |
| `GET_FILE:<name>[,<offset>]` | Stream a file from a byte offset |
| `ABORT` | Cancel an in-flight transfer |
| `NEW_TRIAL` | Zero the counters and roll a new SD file |
| `FEED` | Dispense one pellet |
| `LIGHTS:ON` / `LIGHTS:OFF` | Toggle the NeoPixels |
| `MODE:<spec>` | Switch behavioural program (see below) |

### Device → host

**Handshake.** `PONG_FED3,ID:<n>,FW:<version>`

**Status.** `STATUS,FW:..,ID:..,TIME:..,MODE:..,SESSION:..,FR:..,L:..,R:..,P:..,FILE:..`

**Events.** One line per behavioural event, stamped with the device RTC:

```
EVT,<iso8601>,<LEFT|RIGHT|PELLET>,<left>,<right>,<pellet>,<millis>
```

Counts are absolute running totals rather than deltas, so a host that dropped a
line — or reconnected mid-session — resynchronizes on the next event instead of
drifting.

**Time sync.** `SYNCED,<iso8601>` echoes the clock the device ended up with, so
the host can record the residual offset instead of assuming the write landed.

**File listing.** `FILE:<name>,<bytes>` per entry, terminated by `END_LIST`.

**File transfer.** `FILE_DATA_START:<name>,<offset>,<size>`, then the raw bytes
from `offset`, then `0x04` (EOT) and `CRC32:<hex>`. The CRC covers only the bytes
sent in this range and uses the same polynomial as Python's `zlib.crc32`.

**Errors.** `ERROR:<code>[:<detail>]`, including `ERROR:STREAM_ABORTED:<reason>`
mid-transfer when the device gives up on a stalled host.

### Mode commands

`MODE:FR1` · `MODE:FR3` · `MODE:FR5` · `MODE:FR,<ratio>` · `MODE:PR` ·
`MODE:RR,<ratio>` · `MODE:FRTO,<ratio>,<timeout_s>` · `MODE:FREE` ·
`MODE:EXTINCT` · `MODE:LIGHTTRK` · `MODE:FR1_R` · `MODE:FR_R,<ratio>` ·
`MODE:PR_R` · `MODE:OPTO` · `MODE:OPTO_R` · `MODE:TIMED`
