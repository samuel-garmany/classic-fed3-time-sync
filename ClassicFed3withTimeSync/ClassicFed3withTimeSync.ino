/*
  Feeding experimentation device 3 (FED3)
  Classic FED3 script with Time Sync (FNT compatible)
  
  This script merges the original FED3 Classic script with time synchronization
  and dynamic control capabilities for the Feeding Network Tracker (FNT).
  
  Original Author: 
  alexxai@wustl.edu
  December, 2020

  FNT Modifications:
  samuel.garmany@colorado.edu
  June, 2026
*/

#include <FED3.h>                //Include the FED3 library
#include <RTClib.h>              //Added for time sync
String sketch = "Classic";       //Unique identifier text for each sketch

// Firmware version, reported in PING and STATUS. FNT refuses to drive a board
// that does not announce one, because older firmware folds a ",<offset>"
// argument into the filename and answers FILE_NOT_FOUND to every ranged
// transfer. Bump the minor when adding a command or changing device behaviour
// the host can observe; bump the major when changing the shape of an existing
// line, which is what FNT parses against.
//
// 2.4 — a declared jam no longer calls fed3.DisplayJammed(), which sleeps the
//       MCU forever and recurses; the JAM event is emitted before any display
//       work and the device stays awake and reachable.
// 2.3 — per-attempt FEEDING: progress, a wall-clock ceiling on one dispense,
//       and a throttled retrieval repaint.
// 2.2 — commands are answered *during* a dispense rather than parked; 2.1 read
//       one and then stopped reading, so the board went silent for minutes and
//       the host's writes backed up until the USB endpoint stalled.
// 2.1 — dispensing is bounded and services the serial layer throughout, and
//       reports EVT,...,JAM instead of spinning forever on an empty hopper.
// 2.0 — ranged SD transfers, queued events, non-blocking command reads.
#define FW_VERSION "2.4"

FED3 fed3 (sketch);              //Start the FED3 object
extern RTC_PCF8523 rtc;          //Connect to the clock

//variables for PR tasks
int poke_num = 0;                                      // this variable is the number of pokes since last pellet
int pokes_required = 1;                                // increase the number of pokes required each time a pellet is received using an exponential equation

//variables for Timeout task (FNT mode 12)
unsigned long lastPelletTime = 0;
unsigned long timeoutDuration = 0;                     // in milliseconds

// variables for tracking
int lastLeftCount = 0;
int lastRightCount = 0;
int lastPelletCount = 0;

////////////////////////////////////////////////////////////////////////////////
//  Non-blocking SD file streaming
//
//  Earlier firmware pushed a whole CSV inside the command handler. That starved
//  fed3.run() for the length of the transfer (frozen display, unlogged pokes),
//  and because Serial.write() blocks once the host stops draining the USB CDC
//  buffer, any interruption on the host wedged the device until a power cycle.
//
//  Streaming is now a state machine serviced once per loop(): each pass moves at
//  most STREAM_CHUNK bytes and only as many as the CDC buffer can accept right
//  now, so loop() always returns and the device stays responsive. If the host
//  stops reading for STREAM_IDLE_TIMEOUT the transfer aborts itself.
////////////////////////////////////////////////////////////////////////////////

#define STREAM_CHUNK 64                                // max bytes handed to Serial per loop() pass
#define STREAM_IDLE_TIMEOUT 15000UL                    // ms with a full TX buffer before we give up

File streamFile;
bool streamActive = false;
uint32_t streamCrc = 0xFFFFFFFF;
unsigned long streamLastProgress = 0;

////////////////////////////////////////////////////////////////////////////////
//  Keeping the payload clean
//
//  serviceFileStream() writes raw file bytes straight to Serial, and it runs in
//  loop() next to the event reporter and the command handler. Anything else that
//  prints during a transfer therefore lands *inside* the file the host is
//  reassembling: the host writes a nosepoke line into the mirrored CSV, the CRC
//  no longer matches, the chunk is discarded, and the mirror retries forever on
//  a busy cage.
//
//  So while a stream is active nothing else writes to Serial. Events are queued
//  and flushed afterwards (they carry absolute counts and their own RTC stamp,
//  so a short delay costs no information), and commands other than ABORT are
//  held until the transfer finishes.
////////////////////////////////////////////////////////////////////////////////

#define EVENT_QUEUE_LEN 12                             // pokes buffered during a transfer
#define EVENT_LINE_LEN 72
#define PENDING_CMD_LEN 48

char eventQueue[EVENT_QUEUE_LEN][EVENT_LINE_LEN];
uint8_t eventQueueCount = 0;
bool eventQueueOverflowed = false;

char pendingCommand[PENDING_CMD_LEN];
bool hasPendingCommand = false;

#define CMD_BUF_LEN 64                                 // longest command is ~40 chars
char cmdBuf[CMD_BUF_LEN];
uint8_t cmdLen = 0;
bool cmdOverflow = false;

// Declared up front: the .ino preprocessor generates prototypes automatically,
// but not reliably for functions taking a reference, and finishFileStream()
// calls flushEventQueue() before it is defined.
void flushEventQueue();
bool readCommandLine(String &out);
void handleCommand(const String &command);

// Helper to draw the animated mouse at a specific X position
void drawMouse(int x) {
  fed3.display.fillRoundRect(x + 25, 82, 15, 10, 6, BLACK);    //head
  fed3.display.fillRoundRect(x + 22, 80, 8, 5, 3, BLACK);      //ear
  fed3.display.fillRoundRect(x + 30, 84, 1, 1, 1, WHITE);      //eye
  
  if ((x / 10) % 2 == 0) {
    fed3.display.fillRoundRect(x, 84, 32, 17, 10, BLACK);      //body
    fed3.display.drawFastHLine(x - 8, 85, 18, BLACK);           //tail
    fed3.display.drawFastHLine(x - 8, 86, 18, BLACK);
    fed3.display.drawFastHLine(x - 14, 84, 8, BLACK);
    fed3.display.drawFastHLine(x - 14, 85, 8, BLACK);
    fed3.display.fillRoundRect(x + 22, 99, 8, 4, 3, BLACK);    //front foot
    fed3.display.fillRoundRect(x, 97, 8, 6, 3, BLACK);          //back foot
  } else {
    fed3.display.fillRoundRect(x + 2, 82, 30, 17, 10, BLACK);  //body
    fed3.display.drawFastHLine(x - 6, 91, 18, BLACK);           //tail
    fed3.display.drawFastHLine(x - 6, 90, 18, BLACK);
    fed3.display.drawFastHLine(x - 12, 92, 8, BLACK);
    fed3.display.drawFastHLine(x - 12, 91, 8, BLACK);
    fed3.display.fillRoundRect(x + 15, 99, 8, 4, 3, BLACK);    //foot
    fed3.display.fillRoundRect(x + 8, 97, 8, 6, 3, BLACK);     //back foot
  }
}

// Incremental CRC-32 (same polynomial the host's zlib.crc32 uses).
uint32_t crc32Update(uint32_t crc, uint8_t c) {
  crc ^= c;
  for (uint8_t j = 0; j < 8; j++) {
    crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320) : (crc >> 1);
  }
  return crc;
}

// Current RTC time as ISO-8601, so every serial line carries the device's own
// clock and the host can verify its sync offset rather than assuming it.
String isoNow() {
  DateTime n = rtc.now();
  // Sized for the worst case snprintf can produce, not for the 19 characters a
  // valid date yields: the RTC fields are bytes and year is an int, so a FED3
  // whose coin cell has died reads back values that formatted to 25 characters
  // and overran a char[20] on the stack. snprintf also truncates rather than
  // overflowing if that ever stops being enough.
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
           n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second());
  return String(buf);
}

void closeFileStream() {
  if (streamActive) {
    streamFile.close();
    streamActive = false;
  }
}

// Abort without ever blocking: the notification is only emitted if a transfer
// was actually running and the TX buffer has room. The host has its own
// transfer timeout and recovers either way.
void abortFileStream(const char* reason) {
  if (!streamActive) return;
  closeFileStream();
  if (Serial && Serial.availableForWrite() > 32) {
    Serial.print("ERROR:STREAM_ABORTED:");
    Serial.println(reason);
  }
  flushEventQueue();                                   // pokes that happened mid-transfer
}

void finishFileStream() {
  closeFileStream();
  Serial.write(0x04);                                  // EOT terminates the data section
  Serial.println();
  Serial.print("CRC32:");
  Serial.println(~streamCrc, HEX);
  flushEventQueue();                                   // safe now: the payload is closed
}

// GET_FILE:<name>[,<offset>] — offset lets the host resume an interrupted
// transfer, and lets it tail a growing log by asking only for the bytes it has
// not already mirrored.
void startFileStream(String name, uint32_t offset) {
  if (streamActive) {
    Serial.println("ERROR:STREAM_BUSY");
    return;
  }
  streamFile = fed3.SD.open(name, FILE_READ);
  if (!streamFile) {
    Serial.print("ERROR:FILE_NOT_FOUND:");
    Serial.println(name);
    return;
  }
  uint32_t size = streamFile.fileSize();
  if (offset > size) offset = size;                    // host mirror is ahead (file rotated): resend nothing
  if (offset > 0) streamFile.seek(offset);

  streamCrc = 0xFFFFFFFF;
  streamActive = true;
  streamLastProgress = millis();

  // Header carries the range being sent so the host can place the bytes exactly.
  Serial.print("FILE_DATA_START:");
  Serial.print(name);
  Serial.print(",");
  Serial.print(offset);
  Serial.print(",");
  Serial.println(size);

  if (offset == size) finishFileStream();              // already up to date: empty payload + CRC of nothing
}

void serviceFileStream() {
  if (!streamActive) return;

  if (!Serial) {                                       // host closed the port mid-transfer
    abortFileStream("USB_LOST");
    return;
  }

  int room = Serial.availableForWrite();
  if (room <= 0) {
    if (millis() - streamLastProgress > STREAM_IDLE_TIMEOUT) {
      abortFileStream("HOST_STALLED");
    }
    return;                                            // buffer full — yield, retry next loop()
  }

  int budget = min(room, STREAM_CHUNK);
  while (budget > 0 && streamFile.available()) {
    char c = streamFile.read();
    Serial.write(c);
    streamCrc = crc32Update(streamCrc, (uint8_t)c);
    budget--;
  }
  streamLastProgress = millis();

  if (!streamFile.available()) finishFileStream();
}

// One structured line per behavioural event, stamped with the device RTC and
// carrying all three running counts. Counts are absolute rather than deltas so a
// host that missed a line (or reconnected mid-session) resynchronizes on the
// next event instead of drifting.
//
// During a file transfer the line is queued rather than printed, so it cannot be
// spliced into the payload. The RTC stamp is taken here, at the moment of the
// poke, so queueing delays delivery without distorting the timestamp.
void emitEvent(const char* type) {
  if (!Serial) return;

  char line[EVENT_LINE_LEN];
  snprintf(line, sizeof(line), "EVT,%s,%s,%d,%d,%d,%lu",
           isoNow().c_str(), type,
           fed3.LeftCount, fed3.RightCount, fed3.PelletCount, millis());

  if (streamActive) {
    if (eventQueueCount < EVENT_QUEUE_LEN) {
      strncpy(eventQueue[eventQueueCount], line, EVENT_LINE_LEN - 1);
      eventQueue[eventQueueCount][EVENT_LINE_LEN - 1] = '\0';
      eventQueueCount++;
    } else {
      // Counts are absolute, so the events that do get through still carry the
      // correct totals; only the individual timestamps of the dropped pokes are
      // lost, and those are on the SD card regardless. The host is told.
      eventQueueOverflowed = true;
    }
    return;
  }

  Serial.println(line);
}

////////////////////////////////////////////////////////////////////////////////
//  Bounded, serial-servicing pellet dispensing
//
//  FED3::Feed() is
//
//      do { ...rotate, jam-clear... } while (PelletAvailable == false);
//
//  with no exit but a pellet reaching the well, and a second unbounded
//  `while (digitalRead(PELLET_WELL) == LOW) { run(); }` that waits — forever —
//  for the pellet to be collected, calling run() (and therefore goToSleep())
//  inside the wait.
//
//  Either loop is terminal. An empty hopper spins the disk and escalates jam
//  clearing until numMotorTurns > 100, prints "JAMMED" on the display, and then
//  keeps going; an uncollected pellet parks the MCU in sleep. In both cases
//  loop() never runs again: the display freezes, pokes go unlogged,
//  serviceFileStream() stops pushing bytes, commands are never read, and the
//  board stops answering USB until it is power cycled. This is the "device
//  locked up and FNT cannot talk to it" failure.
//
//  feedPellet() below keeps the library's mechanics — the same rotation, the
//  same jam-clearing escalation in the same order, the same counter increment
//  and the same logdata() row at the same point — so every call site and the
//  CSV schema are unchanged. What differs is that every wait is bounded and
//  services the serial layer, so a transfer in flight keeps moving, the host
//  keeps getting answers, and a hopper that cannot deliver is *reported* rather
//  than silently taking the device off the bus.
////////////////////////////////////////////////////////////////////////////////

#define FEED_MAX_TURNS 60                              // dispense attempts before calling it a jam
// A wall-clock ceiling as well as a turn count. The turn count assumes the
// library's motor routines take about as long as their step counts suggest; this
// holds regardless of what they actually do, so no combination of jam clearing
// can keep the dispenser running indefinitely.
#define FEED_MAX_MS 240000UL
#define FEED_RETRIEVAL_TIMEOUT 300000UL                // ms to wait for the pellet to be collected
#define FEED_POKE_HOLD_TIMEOUT 5000UL                  // ms a poke beam may stay broken before we stop waiting

bool feedInProgress = false;
bool commandDropped = false;                           // reported once printing is safe

// Commands that must not run from inside a dispense: they would re-enter the
// dispenser (FEED) or change the task underneath it (MODE, NEW_TRIAL).
bool deferDuringFeed(const String &command) {
  return command == "FEED" || command == "NEW_TRIAL" || command.startsWith("MODE:");
}

// Keep the serial layer alive from inside a dispense.
//
// Deliberately does not run handleCommand(): FEED and MODE both dispense, so
// executing a command here would re-enter the dispenser from inside itself. A
// command that arrives mid-feed is parked in the pending slot the main loop
// already drains. ABORT is the exception, and is applied directly rather than
// through handleCommand() so this stays free of that reentrancy.
void serviceSerialDuringFeed() {
  serviceFileStream();

  // Pokes counted by the library's own inner loops are reported as they happen
  // rather than after the motor stops.
  if (fed3.LeftCount > lastLeftCount) {
    emitEvent("LEFT");
    lastLeftCount = fed3.LeftCount;
  }
  if (fed3.RightCount > lastRightCount) {
    emitEvent("RIGHT");
    lastRightCount = fed3.RightCount;
  }

  // Input is drained on every pass, whatever else happens. An earlier version
  // parked the first command it saw and then returned early on `hasPendingCommand`
  // for the rest of the dispense: the device answered nothing for minutes, the
  // unread commands backed up until the USB endpoint stalled, and the host saw
  // writes start failing ~40s in. Not reading is never the safe option.
  String command;
  while (readCommandLine(command)) {
    if (command == "ABORT") {
      abortFileStream("HOST_REQUEST");
      if (!streamActive && Serial.availableForWrite() > 8) Serial.println("ABORT_OK");
      continue;
    }

    // Two reasons to hold a command rather than run it here. A reply printed
    // while a file is streaming lands inside the payload the host is
    // reassembling; and FEED, MODE and NEW_TRIAL would re-enter or reconfigure
    // the dispenser this call is standing inside. Everything else — PING,
    // STATUS, LIST_FILES, GET_FILE, SYNC, LIGHTS — is answered immediately, so
    // the device stays visibly alive for the whole dispense.
    if (streamActive || deferDuringFeed(command)) {
      if (hasPendingCommand) {
        // Reported by the main loop once it is safe to print. Dropping one
        // command is recoverable; going deaf to keep it is not.
        commandDropped = true;
      } else {
        strncpy(pendingCommand, command.c_str(), PENDING_CMD_LEN - 1);
        pendingCommand[PENDING_CMD_LEN - 1] = '\0';
        hasPendingCommand = true;
      }
      continue;
    }

    handleCommand(command);
  }
}

// dispenseTimer_ms() with the busy-wait replaced by a serviced one. Same
// debounced read of the pellet well, same "true as soon as a pellet appears".
bool feedWaitForPellet(unsigned long ms) {
  unsigned long deadline = millis() + ms;
  while ((long)(millis() - deadline) < 0) {
    if (digitalRead(PELLET_WELL) == LOW) {
      delayMicroseconds(100);
      if (digitalRead(PELLET_WELL) == LOW) return true;  // debounce
    }
    serviceSerialDuringFeed();
  }
  return false;
}

// The library logs pokes that happen while a pellet is sitting in the well, and
// blocks on `while (digitalRead(LEFT_POKE) == LOW) {}` until the beam clears. A
// mouse resting in the nosepoke would hang that too, so the hold is bounded.
void feedLogPokeIfAny(int pin, bool isLeft) {
  if (digitalRead(pin) != LOW) return;

  unsigned long pokeStart = millis();
  if (isLeft) {
    if (fed3.countAllPokes) fed3.LeftCount++;
    fed3.leftInterval = 0.0;
  } else {
    fed3.RightCount++;
    fed3.rightInterval = 0.0;
  }

  while (digitalRead(pin) == LOW &&
         millis() - pokeStart < FEED_POKE_HOLD_TIMEOUT) {
    serviceSerialDuringFeed();
  }

  if (isLeft) {
    fed3.leftInterval = (millis() - pokeStart);
    fed3.Event = "LeftWithPellet";
  } else {
    fed3.rightInterval = (millis() - pokeStart);
    fed3.Event = "RightWithPellet";
  }
  fed3.UpdateDisplay();
  fed3.logdata();
}

// Rotation that yields.
//
// fed3.RotateDisk() steps the motor without ever touching the serial layer, and
// the library's jam-clearing routines are built out of it: VibrateJam() is 30
// pairs of RotateDisk(120)/RotateDisk(-60) — around eleven seconds of continuous
// motion at the library's 250 RPM — and ClearJam() is comparable. Those are the
// windows in which the device stops draining its USB endpoint, and from the host
// they are indistinguishable from a dead board.
//
// Turning the same arc in short segments with a service call between them keeps
// the total movement, the direction and the early exit on a detected pellet
// identical, while capping the unserviced window at a few tens of milliseconds.
// RotateDisk() already releases the motor at the end of every call, and the
// library's own jam routines call it dozens of times in a row, so segmenting
// changes nothing mechanically that the library does not already do.
#define ROTATE_SEGMENT 25

bool rotateServiced(int steps) {
  int remaining = steps > 0 ? steps : -steps;
  int direction = steps > 0 ? 1 : -1;
  while (remaining > 0) {
    int segment = remaining < ROTATE_SEGMENT ? remaining : ROTATE_SEGMENT;
    if (fed3.RotateDisk(direction * segment)) return true;
    remaining -= segment;
    serviceSerialDuringFeed();
  }
  return false;
}

// Erase the "Jam clear" banner without repainting the whole screen, as the
// library does at each of its early exits.
void clearJamBanner() {
  fed3.display.fillRect(5, 15, 120, 15, WHITE);
}

// Say "jammed" on the screen without ending the device's life.
//
// fed3.DisplayJammed() cannot be used: it paints the banner, calls
// LowPower.sleep() with no wake time, and then recurses into itself. It is a
// deliberate terminal state — the board sleeps forever showing "JAMMED, PLEASE
// CHECK" and never services USB again. That is the whole failure this firmware
// exists to remove, so the banner is painted here instead and the device stays
// awake, keeps logging pokes, and keeps answering the host.
void showJammedBanner() {
  fed3.display.fillRect(6, 20, 200, 22, WHITE);
  fed3.display.setCursor(6, 36);
  fed3.display.print("JAMMED - CHECK HOPPER");
  fed3.display.refresh();
}

bool minorJamServiced() {
  return rotateServiced(100);
}

bool vibrateJamServiced() {
  fed3.DisplayJamClear();
  if (feedWaitForPellet(250)) { clearJamBanner(); return true; }
  for (int i = 0; i < 30; i++) {
    if (rotateServiced(120)) { clearJamBanner(); return true; }
    if (rotateServiced(-60)) { clearJamBanner(); return true; }
  }
  return false;
}

bool clearJamServiced() {
  fed3.DisplayJamClear();
  if (feedWaitForPellet(250)) { clearJamBanner(); return true; }
  for (int i = 0; i < 21 + random(0, 20); i++) {
    if (rotateServiced(-i * 4)) { clearJamBanner(); return true; }
  }
  if (feedWaitForPellet(250)) { clearJamBanner(); return true; }
  for (int i = 0; i < 21 + random(0, 20); i++) {
    if (rotateServiced(i * 4)) { clearJamBanner(); return true; }
  }
  return false;
}

// Drop-in replacement for the library's Feed(). Returns false if the hopper
// could not deliver, where the library would have spun forever.
bool feedPellet(int pulse = 0, bool pixelsoff = true) {
  if (feedInProgress) return false;                    // never re-enter
  feedInProgress = true;

  bool pelletDispensed = false;
  fed3.numMotorTurns = 0;
  unsigned long feedStarted = millis();

  while (!pelletDispensed && fed3.numMotorTurns <= FEED_MAX_TURNS &&
         millis() - feedStarted < FEED_MAX_MS) {
    // One line per attempt. A dispense that takes minutes should look like
    // progress to the host rather than like silence, and it is what locates a
    // hang inside the library's motor routines when one happens.
    if (!streamActive && Serial && Serial.availableForWrite() > 24) {
      Serial.print("FEEDING:");
      Serial.print(fed3.numMotorTurns);
      Serial.print("/");
      Serial.println(FEED_MAX_TURNS);
    }
    pelletDispensed = rotateServiced(-300);
    if (pixelsoff) fed3.pixelsOff();
    serviceSerialDuringFeed();
    if (pelletDispensed) break;

    pelletDispensed = feedWaitForPellet(1500);
    fed3.numMotorTurns++;

    // The library's escalation, unchanged in order and cadence.
    if (!pelletDispensed && fed3.numMotorTurns % 5 == 0) {
      pelletDispensed = minorJamServiced();
    }
    if (!pelletDispensed && fed3.numMotorTurns % 10 == 0 &&
        fed3.numMotorTurns % 20 != 0) {
      pelletDispensed = vibrateJamServiced();
    }
    if (!pelletDispensed && fed3.numMotorTurns % 20 == 0) {
      pelletDispensed = clearJamServiced();
    }
    serviceSerialDuringFeed();
  }

  if (!pelletDispensed) {
    // Where the library would spin forever. The device stays online and says so,
    // which is the whole point: a jam during an unattended run is recoverable
    // information, not a dead cage discovered hours later.
    fed3.ReleaseMotor();
    // Told to the host first. The banner is only pixels; the event is what
    // reaches whoever is not standing in front of the cage.
    emitEvent("JAM");
    showJammedBanner();
    feedInProgress = false;
    return false;
  }

  fed3.ReleaseMotor();
  fed3.pelletTime = millis();
  fed3.display.fillCircle(25, 99, 5, BLACK);
  fed3.display.refresh();

  // Wait for collection, servicing serial throughout and giving up eventually,
  // instead of sleeping the MCU until a human intervenes.
  fed3.retInterval = 0;
  unsigned long lastRetrievalDraw = 0;
  while (digitalRead(PELLET_WELL) == LOW &&
         millis() - fed3.pelletTime < FEED_RETRIEVAL_TIMEOUT) {
    fed3.retInterval = (millis() - fed3.pelletTime);
    // Throttled: DisplayRetrievalInt() ends in display.refresh(), which pushes
    // the entire framebuffer over SPI. Called every pass of this loop it is
    // thousands of full repaints a minute, and it is the slowest thing between
    // one serial service call and the next.
    if (millis() - lastRetrievalDraw >= 250) {
      lastRetrievalDraw = millis();
      fed3.DisplayRetrievalInt();
    }
    feedLogPokeIfAny(LEFT_POKE, true);
    feedLogPokeIfAny(RIGHT_POKE, false);
    serviceSerialDuringFeed();
  }

  // Counted and logged here, exactly where the library does it, so the CSV and
  // every mode's post-Feed() arithmetic see what they saw before.
  fed3.ReleaseMotor();
  fed3.PelletCount++;

  if (pulse > 0) fed3.BNC(pulse, 1);

  fed3.Left = false;
  fed3.Right = false;
  fed3.Event = "Pellet";

  DateTime now = rtc.now();
  fed3.interPelletInterval = now.unixtime() - fed3.lastPellet;
  fed3.lastPellet = now.unixtime();

  fed3.logdata();
  fed3.numMotorTurns = 0;
  fed3.PelletAvailable = true;
  fed3.UpdateDisplay();

  feedInProgress = false;
  return true;
}

// Emitted once a transfer completes, in the order the pokes happened.
void flushEventQueue() {
  for (uint8_t i = 0; i < eventQueueCount; i++) {
    Serial.println(eventQueue[i]);
  }
  eventQueueCount = 0;
  if (eventQueueOverflowed) {
    eventQueueOverflowed = false;
    Serial.println("ERROR:EVENT_QUEUE_OVERFLOW");
  }
  if (commandDropped) {
    commandDropped = false;
    Serial.println("ERROR:COMMAND_DROPPED");
  }
}

void emitStatus() {
  Serial.print("STATUS,ID:");
  Serial.print(fed3.FED);
  Serial.print(",FW:");
  Serial.print(FW_VERSION);
  Serial.print(",TIME:");
  Serial.print(isoNow());
  Serial.print(",MODE:");
  Serial.print(fed3.FEDmode);
  Serial.print(",SESSION:");
  Serial.print(fed3.sessiontype);
  Serial.print(",FR:");
  Serial.print(fed3.FR);
  Serial.print(",L:");
  Serial.print(fed3.LeftCount);
  Serial.print(",R:");
  Serial.print(fed3.RightCount);
  Serial.print(",P:");
  Serial.print(fed3.PelletCount);
  Serial.print(",FILE:");
  Serial.println(fed3.filename);
}

void setup() {
  Serial.begin(115200);                                // start listening on the serial port
  // Commands are read byte-at-a-time by readCommandLine(), so no read here ever
  // blocks on the timeout; it is set low only to bound the library's own reads.
  Serial.setTimeout(10);
  randomSeed(analogRead(0));                           // Seed the random number generator
  fed3.ClassicFED3 = false;                            // Bypasses the boot menu
  fed3.begin();                                        //Setup the FED3 hardware
  if (fed3.FED < 1) {
    fed3.FED = 1;
  }
  fed3.disableSleep();                                 //disable sleep for time sync purposes, this will hurt battery but sync requires a wired connection anyways
  fed3.FEDmode = 1;                                    //Default to Fixed Ratio (FR)
  fed3.FR = 1;                                         //Default to FR1
  fed3.sessiontype = "FR1";
  fed3.DisplayPokes = true;
  fed3.DisplayTimed = false;
 
  // Custom Device ID configuration menu on boot (modeled after the original FED3 Classic menu)
  int initialFED = fed3.FED;
  bool displayNeedsUpdate = true;
  int mouseX = -50;
  
  // Set up screen elements before animation loop
  fed3.display.clearDisplay();
  fed3.display.setTextColor(BLACK);
  fed3.display.setTextSize(1);
  
  // Top header text
  fed3.display.setCursor(10, 40);
  fed3.display.print("Set Device ID:");
  
  // Filename at the bottom
  fed3.display.setCursor(1, 135);
  fed3.display.print(fed3.filename);
  
  // Only display the interactive boot menu if USB Serial is NOT active.
  // This bypasses boot delay when FNT scanner pings the board, preventing SAMD21 USB controller hangs.
  if (!Serial) {
    // The mouse animation acts as a loading bar timeout. Resetting mouseX resets the timer.
    while (mouseX < 200) {
      if (displayNeedsUpdate) {
        // Erase previous ID text area
        fed3.display.fillRect(10, 48, 140, 20, WHITE);
        
        // Draw selected ID text
        fed3.display.setCursor(10, 60);
        if (fed3.FED < 100 && fed3.FED >= 10) {
          fed3.display.print("0");
        }
        if (fed3.FED < 10) {
          fed3.display.print("00");
        }
        fed3.display.print(fed3.FED);
        
        fed3.display.refresh();
        displayNeedsUpdate = false;
      }
      
      // Draw the running mouse animation frame by frame
      drawMouse(mouseX);
      
      fed3.display.refresh();
      delay(80);
      
      // Erase the mouse area
      fed3.display.fillRect (mouseX - 25, 73, 95, 33, WHITE);
      
      // Update mouse position (loading bar progress, original mouse speed)
      mouseX += 15;
      
      // Poll the buttons directly (digitalRead == LOW is pressed)
      // Left Poke -> Decrement Device ID
      if (digitalRead(LEFT_POKE) == LOW) {
        if (fed3.FED > 1) {
          fed3.FED--;
          // Play tones & neopixel animations like in SelectMode()
          tone(BUZZER, 2500, 200);
          fed3.colorWipe(fed3.strip.Color(2, 0, 2), 40); // Purple wipe
          fed3.colorWipe(fed3.strip.Color(0, 0, 0), 20); // OFF
          mouseX = -50; // Reset loading bar/timeout
          displayNeedsUpdate = true;
        }
      }
      
      // Right Poke -> Increment Device ID
      if (digitalRead(RIGHT_POKE) == LOW) {
        if (fed3.FED < 999) {
          fed3.FED++;
          // Play tones & neopixel animations like in SelectMode()
          tone(BUZZER, 2500, 200);
          fed3.colorWipe(fed3.strip.Color(2, 2, 0), 40); // Yellow/Green wipe
          fed3.colorWipe(fed3.strip.Color(0, 0, 0), 20); // OFF
          mouseX = -50; // Reset loading bar/timeout
          displayNeedsUpdate = true;
        }
      }
    }
  }
  
  // If the device ID was changed, recreate the datafile with the new ID
  if (fed3.FED != initialFED) {
    // Print "...Selected!" like in the original program selection menu
    fed3.display.setCursor(10, 100);
    fed3.display.println("...Selected!");
    fed3.display.refresh();
    delay(500);

    fed3.SD.remove(fed3.filename);                     // Delete the initial empty file created with old ID
    fed3.writeConfigFile();                            // Write the new config
    fed3.CreateDataFile();                             // Create the new datafile using new ID
    fed3.writeHeader();                                // Write header to the new file
  }
  
  // Clear display, clear nosepoke flags, and show the normal FR1 screen
  fed3.Left = false;
  fed3.Right = false;
  fed3.display.clearDisplay();
  fed3.UpdateDisplay();
}

void loop() {
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                     Original FED3 Classic Modes
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Mode 0: Free feeding
  if (fed3.FEDmode == 0) {
    fed3.sessiontype = "Free_feed";                     //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.DisplayPokes = false;                          //Turn off poke indicators for free feeding mode
    fed3.UpdateDisplay();                               //Update display for free feeding session to remove poke display (they are on by default)
    feedPellet();
    fed3.Timeout(5);                                    //5s timeout
  }

  // Modes 1-3: Fixed Ratio Programs FR1, FR3, FR5
  if ((fed3.FEDmode == 1) or (fed3.FEDmode == 2) or (fed3.FEDmode == 3)) {
    fed3.sessiontype = "FR" + String(fed3.FR);          //The text in "sessiontype" will appear on the screen and in the logfile
    if (fed3.Left) {
      fed3.logLeftPoke();                               //Log left poke
      if (fed3.LeftCount % fed3.FR == 0) {              //if fixed ratio is  met
        fed3.ConditionedStimulus();                     //deliver conditioned stimulus (tone and lights)
        feedPellet();                                    //deliver pellet
      }
    }
    if (fed3.Right) {                                    //If right poke is triggered
      fed3.logRightPoke();
    }
  }

  // Mode 4: Progressive Ratio
  if (fed3.FEDmode == 4) {
    fed3.sessiontype = "ProgRatio";                      //The text in "sessiontype" will appear on the screen and in the logfile
    if (fed3.Left) {                                     //If left poke is triggered and pellet is not in the well
      fed3.logLeftPoke();                                //Log left poke
      fed3.Click();                                      //Click
      poke_num++;                                        //store this new poke number as current poke number.
      if (poke_num == pokes_required) {                  //check to see if the mouse has acheived the correct number of pokes in order to receive the pellet
        fed3.ConditionedStimulus();                      //Deliver conditioned stimulus (tone and lights)
        feedPellet();                                     //Deliver pellet
        pokes_required = round((5 * exp((fed3.PelletCount + 1) * 0.2)) - 5);  //increment the number of pokes required according to the progressive ratio:
        fed3.FR = pokes_required;
        poke_num = 0;                                    //reset poke_num back to 0 for the next trial
      }
    }
    if (fed3.Right) {                                    //If right poke is triggered and pellet is not in the well
      fed3.logRightPoke();
    }
  }

  // Mode 5: Extinction
  if (fed3.FEDmode == 5) {
    fed3.sessiontype = "Extinct";                        //The text in "sessiontype" will appear on the screen and in the logfile
    if (fed3.Left) {
      fed3.logLeftPoke();                                //Log left poke
      fed3.ConditionedStimulus();                        //deliver conditioned stimulus (tone and lights)
    }
    if (fed3.Right) {                                    //If right poke is triggered
      fed3.logRightPoke();
    }
  }

  // Mode 6: Light tracking FR1 task
  if (fed3.FEDmode == 6) {
    fed3.sessiontype = "Light Trk";                       //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.disableSleep();                                  //Sleep mode shuts the NeoPixels off to save power.  Therefore to leave pixels on during this task we must disable sleep mode.

    //If left poke is active, run FR1 session with left active
    if (fed3.activePoke == 1) {
      fed3.leftPokePixel(5,5,5,0) ;                       //turn on pixel inside left nosepoke dim white
      if (fed3.Left) {
        fed3.logLeftPoke();                               //Log left poke
        fed3.ConditionedStimulus();                       //deliver conditioned stimulus (tone and lights)
        feedPellet();
        fed3.randomizeActivePoke(3);                      //randomize which poke is active, specifying maximum on the same poke before forcing a switch
      }
      if (fed3.Right) {                                   //If right poke is triggered
        fed3.logRightPoke();
      }
    }
    //If right poke is active, run FR1 session with right active
    if (fed3.activePoke == 0) {
      fed3.rightPokePixel(5,5,5,0) ;                    //turn on pixel inside right nosepoke dim white
      if (fed3.Right) {
        fed3.logRightPoke();                              //Log left poke
        fed3.ConditionedStimulus();                       //deliver conditioned stimulus (tone and lights)
        feedPellet();                                      //deliver pellet
        fed3.randomizeActivePoke(3);                      //randomize which poke is active, specifying maximum on the same poke before forcing a switch
      }
      if (fed3.Left) {                                    //If right poke is triggered
        fed3.logLeftPoke();
      }
    }
  }

  // Mode 7: FR (reversed)
  if (fed3.FEDmode == 7) {
    if (fed3.FR == 1) {
      fed3.sessiontype = "FR1_R";
    } else {
      fed3.sessiontype = "FR" + String(fed3.FR) + "_R";
    }
    fed3.activePoke = 0;                                  //Set activePoke to 0 to make right poke active
    if (fed3.Left) {                                      //If left poke
      fed3.logLeftPoke();                                 //Log left poke
    }
    if (fed3.Right) {                                     //If right poke is triggered
      fed3.logRightPoke();                                //Log Right Poke
      if (fed3.RightCount % fed3.FR == 0) {               //if fixed ratio is met
        fed3.ConditionedStimulus();                         //Deliver conditioned stimulus (tone and lights)
        feedPellet();                                        //deliver pellet
      }
    }
  }

  // Mode 8: PR (reversed)
  if (fed3.FEDmode == 8) {
    fed3.sessiontype = "PR_R";                          //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.activePoke = 0;                                //Right poke is active
    if (fed3.Right) {                                   //If Right poke is triggered
      fed3.logRightPoke();                              //Log Right poke
      poke_num++;                                       //store this new poke number as current poke number.
      if (poke_num == pokes_required) {                 //check to see if the mouse has acheived the correct number of pokes in order to receive the pellet
        fed3.ConditionedStimulus();                     //Deliver conditioned stimulus (tone and lights)
        feedPellet();                                    //Deliver pellet
        pokes_required = round((5 * exp((fed3.PelletCount + 1) * 0.2)) - 5);
        fed3.FR = pokes_required;
        poke_num = 0;                                   //reset the number of pokes back to 0, for the next trial
        fed3.Right = false;
      }
      else {
        fed3.Click();                                   //If not enough pokes, just do a Click
      }
    }
    if (fed3.Left) {                                    //If left poke is triggered and pellet is not in the well
      fed3.logLeftPoke();
    }
  }

  // Mode 9: Optogenetic stimulation
  if (fed3.FEDmode == 9) {
    fed3.sessiontype = "OptoStim";                      //The text in "sessiontype" will appear on the screen and in the logfile
    if (fed3.Left) {                                    //If left poke
      fed3.logLeftPoke();                               //Log left poke
      fed3.ConditionedStimulus();                       //Deliver conditioned stimulus (tone and lights)
      fed3.BNC(25, 20);                                 //Deliver 20 pulses at 20Hz (25ms HIGH, 25ms LOW), lasting 1 second
    }
    if (fed3.Right) {                                   //If right poke is triggered
      fed3.logRightPoke();                              //Log Right Poke
    }
  }

  // Mode 10: Optogenetic stimulation (reversed)
  if (fed3.FEDmode == 10) {
    fed3.sessiontype = "OptoStim_R";                     //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.activePoke = 0;                                 //Set activePoke to 0 to make right poke active
    if (fed3.Right) {                                    //If Right poke
      fed3.logRightPoke();                               //Log Right poke
      fed3.ConditionedStimulus();                        //Deliver conditioned stimulus (tone and lights)
      fed3.BNC(25, 20);                                  //Deliver 20 pulses at 20Hz (25ms HIGH, 25ms LOW), lasting 1 second
    }
    if (fed3.Left) {                                     //If Left poke is triggered
      fed3.logLeftPoke();                                //Log LeftPoke
    }
  }

  // Mode 11: Timed Feeding
  if (fed3.FEDmode == 11) {
    fed3.sessiontype = "Timed";                         //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.DisplayPokes = false;                          //Turn off poke indicators for free feeding mode
    fed3.DisplayTimed = true;                           //Display timed feeding info
    fed3.UpdateDisplay();
    if ((int)fed3.currentHour >= fed3.timedStart &&
        (int)fed3.currentHour < fed3.timedEnd) {
      feedPellet();
      fed3.Timeout(5);                                  //5s timeout
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                     FNT Custom Modes
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Mode 12: Fixed Ratio with Timeout (FRTO)
  if (fed3.FEDmode == 12) {
    if (fed3.Left) {
      fed3.logLeftPoke();                               //Log left poke
      if (lastPelletTime == 0 || (millis() - lastPelletTime >= timeoutDuration)) {
        if (fed3.LeftCount % fed3.FR == 0) {
          fed3.ConditionedStimulus();
          feedPellet();
          lastPelletTime = millis();
        }
      } else {
        fed3.Click();                                   // Play sound click to indicate registered poke during timeout
      }
    }
    if (fed3.Right) {
      fed3.logRightPoke();
    }
  }

  // Mode 13: Random Ratio (RR)
  if (fed3.FEDmode == 13) {
    if (fed3.Left) {
      fed3.logLeftPoke();                               //Log left poke
      if (random(1, fed3.FR + 1) == 1) {
        fed3.ConditionedStimulus();
        feedPellet();
      } else {
        fed3.Click();
      }
    }
    if (fed3.Right) {
      fed3.logRightPoke();
    }
  }

  // Call fed.run at least once per loop
  fed3.run();

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                     FNT Serial Comm and Live Tracking
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Push a slice of any in-flight file transfer. Bounded and non-blocking, so
  // the behavioural task above keeps running throughout an SD card download.
  serviceFileStream();

  // Live Tracking System over Serial
  if (fed3.LeftCount > lastLeftCount) {
    emitEvent("LEFT");
    lastLeftCount = fed3.LeftCount;
  }

  if (fed3.RightCount > lastRightCount) {
    emitEvent("RIGHT");
    lastRightCount = fed3.RightCount;
  }

  if (fed3.PelletCount > lastPelletCount) {
    emitEvent("PELLET");
    lastPelletCount = fed3.PelletCount;
  }

  // Serial command handling. While a transfer is in flight, only ABORT is acted
  // on: every other command prints a reply, and a reply emitted mid-stream lands
  // inside the file the host is reassembling. One command is held and run as
  // soon as the payload is closed.
  if (commandDropped && !streamActive) {
    commandDropped = false;
    Serial.println("ERROR:COMMAND_DROPPED");
  }

  if (hasPendingCommand && !streamActive) {
    hasPendingCommand = false;
    handleCommand(String(pendingCommand));
  } else if (!hasPendingCommand) {
    String command;
    if (readCommandLine(command)) {
      if (streamActive && command != "ABORT") {
        strncpy(pendingCommand, command.c_str(), PENDING_CMD_LEN - 1);
        pendingCommand[PENDING_CMD_LEN - 1] = '\0';
        hasPendingCommand = true;
      } else {
        handleCommand(command);
      }
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
//  Command handling
////////////////////////////////////////////////////////////////////////////////

// Accumulate one newline-terminated command without blocking.
//
// Serial.readStringUntil() was doing this before. It blocks for the serial
// timeout on every call, and — the reason it had to go — it returns whatever
// arrived so far when the timeout expires, so a command split across two USB
// packets is silently truncated: "MODE:FR,10" applied as "MODE:FR,1" changes
// the reinforcement schedule of a running experiment with no error anywhere.
bool readCommandLine(String &out) {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen == 0) {
        cmdOverflow = false;
        continue;                                    // bare CR of a CRLF pair
      }
      bool poisoned = cmdOverflow;
      cmdBuf[cmdLen] = '\0';
      cmdLen = 0;
      cmdOverflow = false;
      if (poisoned) {
        Serial.println("ERROR:COMMAND_TOO_LONG");
        continue;
      }
      out = String(cmdBuf);
      out.trim();
      return out.length() > 0;
    }
    if (cmdLen < CMD_BUF_LEN - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdOverflow = true;                            // drop the rest of this line
    }
  }
  return false;
}

void handleCommand(const String &command) {
  // Handshake for GUI auto-discovery
  if (command == "PING") {
      // The FW field is what lets FNT tell this firmware apart from the older
      // one, whose GET_FILE cannot take an offset. Without it the host has to
      // assume the worst and every ranged transfer fails as FILE_NOT_FOUND.
      Serial.print("PONG_FED3,ID:");
      Serial.print(fed3.FED);
      Serial.print(",FW:");
      Serial.println(FW_VERSION);
  }
  else if (command == "STATUS") {
      emitStatus();
  }
  else if (command == "ABORT") {                     // host-side escape hatch out of a wedged transfer
      if (streamActive) {
        abortFileStream("HOST_REQUEST");
      } else {
        Serial.println("ABORT_OK");
      }
  }
  else if (command == "LIST_FILES") {
      File root = fed3.SD.open("/");
      if (!root) {
          Serial.println("ERROR:CANNOT_OPEN_ROOT");
      } else {
          root.rewind();
          while (true) {
              if (!Serial) break;                    // host vanished; don't block on a dead port
              File entry = root.openNextFile();
              if (!entry) {
                  break;
              }
              char name[32];
              entry.getName(name, sizeof(name));
              String nameStr = String(name);
              if (!entry.isDirectory() && nameStr.startsWith("FED") && nameStr.endsWith(".CSV")) {
                  Serial.print("FILE:");
                  Serial.print(nameStr);
                  Serial.print(",");
                  Serial.println(entry.fileSize());
              }
              entry.close();
          }
          root.close();
          Serial.println("END_LIST");
      }
  }
  else if (command.startsWith("GET_FILE:")) {
      // GET_FILE:<name>[,<offset>] — arms the non-blocking streamer above.
      String args = command.substring(9);
      args.trim();

      uint32_t offset = 0;
      int comma = args.indexOf(',');
      String reqFilename = args;
      if (comma != -1) {
          reqFilename = args.substring(0, comma);
          offset = (uint32_t)args.substring(comma + 1).toInt();
      }
      reqFilename.trim();

      startFileStream(reqFilename, offset);
  }
  else if (command.startsWith("FSIZE:")) {
      // Cheap "is there anything new?" probe — lets the host skip a transfer
      // entirely when its mirror is already current.
      String reqFilename = command.substring(6);
      reqFilename.trim();
      File f = fed3.SD.open(reqFilename, FILE_READ);
      if (!f) {
          Serial.print("ERROR:FILE_NOT_FOUND:");
          Serial.println(reqFilename);
      } else {
          Serial.print("FSIZE:");
          Serial.print(reqFilename);
          Serial.print(",");
          Serial.println(f.fileSize());
          f.close();
      }
  }
  else if (command == "NEW_TRIAL") {
      abortFileStream("NEW_TRIAL");                  // the file being streamed is about to be superseded
      fed3.LeftCount = 0;
      fed3.RightCount = 0;
      fed3.PelletCount = 0;
      fed3.BlockPelletCount = 0;
      
      lastLeftCount = 0;
      lastRightCount = 0;
      lastPelletCount = 0;
      
      // Reset PR variables
      poke_num = 0;
      pokes_required = 1;
      fed3.FR = 1;
      
      // Create new data file on SD card
      fed3.CreateDataFile();
      fed3.writeHeader();
      
      // Update display to reflect new counters
      fed3.UpdateDisplay();
      
      Serial.print("NEW_TRIAL_STARTED:");
      Serial.println(fed3.filename);
  }
  else if (command == "FEED") {
      feedPellet();
      Serial.println("Pellet dispensed manually.");
  }
  else if (command == "LIGHTS:ON") {
      fed3.pixelsOn(30, 30, 30, 30);
      Serial.println("Lights turned ON.");
  }
  else if (command == "LIGHTS:OFF") {
      fed3.pixelsOff();
      Serial.println("Lights turned OFF.");
  }
  // check if the command starts with SYNC:
  else if (command.startsWith("SYNC:")) {
      // SYNC:YYYY,MM,DD,HH,MM,SS — tokenized rather than sliced at fixed
      // offsets, so an unpadded field can't silently set the clock wrong.
      String args = command.substring(5);
      int f[6] = {0, 0, 0, 0, 0, 0};
      int idx = 0;
      int start = 0;
      while (idx < 6) {
          int comma = args.indexOf(',', start);
          String tok = (comma == -1) ? args.substring(start) : args.substring(start, comma);
          f[idx++] = tok.toInt();
          if (comma == -1) break;
          start = comma + 1;
      }

      if (idx < 6 || f[0] < 2000 || f[1] < 1 || f[1] > 12 || f[2] < 1 || f[2] > 31) {
          Serial.println("ERROR:BAD_SYNC");
      } else {
          rtc.adjust(DateTime(f[0], f[1], f[2], f[3], f[4], f[5]));
          // Echo the clock we ended up with so the host can record the true
          // device time against its own and measure the residual offset.
          Serial.print("SYNCED,");
          Serial.println(isoNow());
          fed3.UpdateDisplay();
      }
  }
  // check if the command starts with MODE:
  else if (command.startsWith("MODE:")) {
      String mode_params = command.substring(5);
      if (mode_params.startsWith("FRTO")) {
          int comma1 = mode_params.indexOf(',');
          int ratio = 1;
          int timeout_s = 30;
          if (comma1 != -1) {
              int comma2 = mode_params.indexOf(',', comma1 + 1);
              if (comma2 != -1) {
                  ratio = mode_params.substring(comma1 + 1, comma2).toInt();
                  timeout_s = mode_params.substring(comma2 + 1).toInt();
              } else {
                  ratio = mode_params.substring(comma1 + 1).toInt();
              }
          }
          if (ratio < 1) ratio = 1;
          if (timeout_s < 0) timeout_s = 0;
          
          fed3.FEDmode = 12; 
          fed3.FR = ratio;
          timeoutDuration = (unsigned long)timeout_s * 1000;
          lastPelletTime = 0; 
          
          fed3.sessiontype = "FR" + String(ratio) + " TO" + String(timeout_s) + "s";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.UpdateDisplay();
          Serial.print("Mode set to Fixed Ratio with Timeout. FR: ");
          Serial.print(fed3.FR);
          Serial.print(", Timeout: ");
          Serial.print(timeout_s);
          Serial.println("s");
      }
      else if (mode_params == "FR1") {
          fed3.FEDmode = 1;
          fed3.FR = 1;
          fed3.sessiontype = "FR1";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.UpdateDisplay();
          Serial.println("Mode set to FR1");
      }
      else if (mode_params == "FR3") {
          fed3.FEDmode = 2;
          fed3.FR = 3;
          fed3.sessiontype = "FR3";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.UpdateDisplay();
          Serial.println("Mode set to FR3");
      }
      else if (mode_params == "FR5") {
          fed3.FEDmode = 3;
          fed3.FR = 5;
          fed3.sessiontype = "FR5";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.UpdateDisplay();
          Serial.println("Mode set to FR5");
      }
      else if (mode_params == "FR" || mode_params.startsWith("FR,")) {
          int comma_idx = mode_params.indexOf(',');
          int ratio = 1;
          if (comma_idx != -1) {
              ratio = mode_params.substring(comma_idx + 1).toInt();
          }
          if (ratio < 1) ratio = 1;
          fed3.FEDmode = 1; 
          fed3.FR = ratio;
          fed3.sessiontype = "FR" + String(ratio);
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.UpdateDisplay();
          Serial.print("Mode set to Fixed Ratio, FR: ");
          Serial.println(fed3.FR);
      }
      else if (mode_params.startsWith("RR")) {
          int comma_idx = mode_params.indexOf(',');
          int ratio = 5; 
          if (comma_idx != -1) {
              ratio = mode_params.substring(comma_idx + 1).toInt();
          }
          if (ratio < 1) ratio = 1;
          fed3.FEDmode = 13; 
          fed3.FR = ratio;
          fed3.sessiontype = "RR" + String(ratio);
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.UpdateDisplay();
          Serial.print("Mode set to Random Ratio, RR: ");
          Serial.println(fed3.FR);
      }
      else if (mode_params == "PR") {
          fed3.FEDmode = 4;
          fed3.sessiontype = "ProgRatio";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          poke_num = 0;
          pokes_required = 1;
          fed3.FR = 1;
          fed3.UpdateDisplay();
          Serial.println("Mode set to Progressive Ratio");
      }
      else if (mode_params == "FREE") {
          fed3.FEDmode = 0;
          fed3.sessiontype = "Free_feed";
          fed3.DisplayPokes = false;
          fed3.DisplayTimed = false;
          fed3.UpdateDisplay();
          Serial.println("Mode set to Free Feeding");
      }
      else if (mode_params == "EXTINCT") {
          fed3.FEDmode = 5;
          fed3.sessiontype = "Extinct";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.UpdateDisplay();
          Serial.println("Mode set to Extinction");
      }
      else if (mode_params == "LIGHTTRK") {
          fed3.FEDmode = 6;
          fed3.sessiontype = "Light Trk";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.disableSleep();
          fed3.UpdateDisplay();
          Serial.println("Mode set to Light Tracking");
      }
      else if (mode_params.startsWith("FR1_R") || mode_params.startsWith("FR_R")) {
          int comma_idx = mode_params.indexOf(',');
          int ratio = 1;
          if (comma_idx != -1) {
              ratio = mode_params.substring(comma_idx + 1).toInt();
          }
          if (ratio < 1) ratio = 1;
          fed3.FEDmode = 7;
          fed3.FR = ratio;
          if (ratio == 1) {
              fed3.sessiontype = "FR1_R";
          } else {
              fed3.sessiontype = "FR" + String(ratio) + "_R";
          }
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.activePoke = 0;
          fed3.UpdateDisplay();
          Serial.print("Mode set to FR Reversed. FR: ");
          Serial.println(fed3.FR);
      }
      else if (mode_params == "PR_R") {
          fed3.FEDmode = 8;
          fed3.sessiontype = "PR_R";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.activePoke = 0;
          poke_num = 0;
          pokes_required = 1;
          fed3.FR = 1;
          fed3.UpdateDisplay();
          Serial.println("Mode set to PR Reversed");
      }
      else if (mode_params == "OPTO") {
          fed3.FEDmode = 9;
          fed3.sessiontype = "OptoStim";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.UpdateDisplay();
          Serial.println("Mode set to Opto Stimulation");
      }
      else if (mode_params == "OPTO_R") {
          fed3.FEDmode = 10;
          fed3.sessiontype = "OptoStim_R";
          fed3.DisplayPokes = true;
          fed3.DisplayTimed = false;
          fed3.activePoke = 0;
          fed3.UpdateDisplay();
          Serial.println("Mode set to Opto Stimulation Reversed");
      }
      else if (mode_params == "TIMED") {
          fed3.FEDmode = 11;
          fed3.sessiontype = "Timed";
          fed3.DisplayPokes = false;
          fed3.DisplayTimed = true;
          fed3.UpdateDisplay();
          Serial.println("Mode set to Timed Feeding");
      }
      else {
          // An unknown spec used to leave the device silently in its previous
          // mode, so a mistyped scheduled event looked like it had been applied.
          Serial.print("ERROR:UNKNOWN_MODE:");
          Serial.println(mode_params);
      }
  }
  else {
    // Previously an unrecognised command produced no output at all, so the host
    // could not distinguish "not supported by this firmware" from "device has
    // stopped responding" and simply waited for its timeout.
    Serial.print("ERROR:UNKNOWN_COMMAND:");
    Serial.println(command);
  }
}
