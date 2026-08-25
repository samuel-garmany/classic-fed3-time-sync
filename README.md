# Classic FED3 with FNT Support

> [!WARNING]
> This software has only been tested on the FED 3.2 revision, your milage may vary when attempting to use a different model.

This repository contains modified firmware for the **Feeding Experimentation Device 3 (FED3)**, customized to enable real-time event tracking, automated clock synchronization, and on-the-fly behavioral protocol switching when paired with the [FieldNeuroToolbox (FNT)](https://github.com/calebvogt/fnt).

This sketch is modified from the original `ClassicFED3` script from the official [KravitzLabDevices/FED3_library](https://github.com/KravitzLabDevices/FED3_library).

## Installation & Setup

**System & IDE Setup:** Follow the official [FED3 Setup Guide](https://github.com/KravitzLabDevices/FED3_library/wiki/Get-started) to configure your computer and Arduino IDE.

### Building from the command line

The sketch carries a [project file](https://arduino.github.io/arduino-cli/latest/sketch-project-file/)
pinning the board platform and every library version, so a build does not depend
on whatever happens to be installed locally:

```bash
arduino-cli compile ClassicFed3withTimeSync
arduino-cli upload -p /dev/ttyACM0 ClassicFed3withTimeSync
```

The first build needs the Adafruit board index:

```bash
arduino-cli config add board_manager.additional_urls \
  https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
arduino-cli core update-index
```

After changing a platform or library version, regenerate the pins with
`arduino-cli compile --dump-profile ClassicFed3withTimeSync` and commit the result.

The modified sketch is located at
[`ClassicFed3withTimeSync/ClassicFed3withTimeSync.ino`](ClassicFed3withTimeSync/ClassicFed3withTimeSync.ino).

> [!IMPORTANT]
> **Reflash every device before an experiment.** FNT refuses any board whose
> `PING` reply carries no `FW:` field, or a version below 2.0 — it will connect,
> report "firmware too old", and disconnect. This is deliberate: older firmware
> parses `GET_FILE:<name>,<offset>` as a request for a file literally named
> `"<name>,0"`, so SD mirroring and exports fail, and they fail silently
> mid-experiment. Check a board by sending `PING` in a serial monitor; the reply
> should read `PONG_FED3,ID:<n>,FW:2.0`.

> [!NOTE]
> Because these changes require a continuous wired USB connection to a host computer running FNT, sleep mode is disabled in this firmware. This will increase battery consumption.

## Key Modifications

Compared to the standard Classic FED3 firmware, this version introduces:
1. **Disabled Sleep Mode:** Disables the standard FED3 low-power sleep state to ensure the microcontroller remains active and responsive to incoming serial messages.
2. **Serial Connection Setup:** Initializes a serial connection at `115200` baud.
3. **Live Event Tracking:** Automatically broadcasts nosepoke and pellet dispensing events over the serial port as they occur, timestamped from the device RTC.
4. **Interactive Time Synchronization:** Adds a protocol to sync the FED3's real-time clock (RTC) dynamically via serial commands.
5. **Non-blocking SD card transfers:** Log files stream to the host a slice at a time, so the behavioural task keeps running during a download.
6. **Uninterrupted payloads:** Nothing else writes to the serial port while a file is streaming. Nosepokes are queued and flushed once the transfer closes, and commands other than `ABORT` are held until then. Without this, a poke during a download splices an `EVT` line into the middle of the file, which fails the CRC and costs both the chunk and the event.
7. **Non-blocking command reads:** Commands are accumulated a byte at a time rather than with `Serial.readStringUntil()`, which returns a truncated command when a line is split across two USB packets — quietly turning `MODE:FR,10` into `MODE:FR,1`.
8. **Version reporting:** `PING` and `STATUS` carry `FW:`, which is how the host decides whether ranged transfers are available.
9. **Bounded pellet dispensing:** `feedPellet()` replaces the library's `FED3::Feed()` at every call site. The library's version is `do { ...jam clearing... } while (PelletAvailable == false)` with no exit but a pellet reaching the well, plus a second unbounded wait for the pellet to be *collected* that calls `run()` — and therefore `goToSleep()` — inside the loop. An empty hopper or an uncollected pellet meant `loop()` never ran again: frozen display, unlogged pokes, stalled transfers, and no USB until a power cycle. `feedPellet()` keeps the same rotation, the same jam-clearing escalation, the same counter increment and the same `logdata()` row, but services the serial layer between movements and gives up after `FEED_MAX_TURNS`, reporting `EVT,...,JAM` instead of spinning forever.

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

The `FW:` field gates everything else. A reply without one, or reporting less
than 2.0, means the host disconnects rather than falling back — FNT supports one
protocol, not two.

**Status.** `STATUS,ID:..,FW:..,TIME:..,MODE:..,SESSION:..,FR:..,L:..,R:..,P:..,FILE:..`

**Events.** One line per behavioural event, stamped with the device RTC:

```
EVT,<iso8601>,<LEFT|RIGHT|PELLET|JAM>,<left>,<right>,<pellet>,<millis>
```

`JAM` is not a behaviour: it means the device tried to dispense and gave up
because the hopper is empty or the disk is stuck. It travels as an event so that
it is stamped by the device, queued rather than spliced during a transfer, and
lands in the host's `events.csv` beside the pokes it interrupts. The device keeps
running and logging afterwards.

Counts are absolute running totals rather than deltas, so a host that dropped a
line — or reconnected mid-session — resynchronizes on the next event instead of
drifting.

**Time sync.** `SYNCED,<iso8601>` echoes the clock the device ended up with, so
the host can record the residual offset instead of assuming the write landed.

**File listing.** `FILE:<name>,<bytes>` per entry, terminated by `END_LIST`.

**File transfer.** `FILE_DATA_START:<name>,<offset>,<size>`, then the raw bytes
from `offset`, then `0x04` (EOT) and `CRC32:<hex>`. The CRC covers only the bytes
sent in this range and uses the same polynomial as Python's `zlib.crc32`.

**Errors.** `ERROR:<code>[:<detail>]`. Codes:

| Code | Meaning |
| --- | --- |
| `STREAM_ABORTED:<reason>` | Transfer given up mid-flight (`HOST_STALLED`, `USB_LOST`, `HOST_REQUEST`, `NEW_TRIAL`) |
| `STREAM_BUSY` | A transfer was requested while one was already running |
| `FILE_NOT_FOUND:<name>` | No such file on the SD card |
| `CANNOT_OPEN_ROOT` | SD card root directory unreadable |
| `BAD_SYNC` | `SYNC:` arguments missing or out of range |
| `EVENT_QUEUE_OVERFLOW` | More than 12 pokes occurred during one transfer; the counts in later events are still absolute and correct, only the individual timestamps of the dropped ones were lost |
| `COMMAND_TOO_LONG` | Command exceeded the 64-byte input buffer |
| `UNKNOWN_COMMAND:<text>` | Command not recognised by this firmware |
| `UNKNOWN_MODE:<spec>` | `MODE:` argument not recognised; the mode is unchanged |

**Ordering guarantee.** Between `FILE_DATA_START` and the `0x04` terminator the
device emits *only* file bytes. Events and command replies that occur during a
transfer are queued and sent afterwards, so the CRC always covers exactly the
requested byte range.

### Mode commands

`MODE:FR1` · `MODE:FR3` · `MODE:FR5` · `MODE:FR,<ratio>` · `MODE:PR` ·
`MODE:RR,<ratio>` · `MODE:FRTO,<ratio>,<timeout_s>` · `MODE:FREE` ·
`MODE:EXTINCT` · `MODE:LIGHTTRK` · `MODE:FR1_R` · `MODE:FR_R,<ratio>` ·
`MODE:PR_R` · `MODE:OPTO` · `MODE:OPTO_R` · `MODE:TIMED`
