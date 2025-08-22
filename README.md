# Com-Prot (StarWire) Library

A tiny two–wire communication protocol for ESP8266 style devices.  The master
drives a **clock** line while all devices share a single **open‑drain data**
line.  Slaves speak only when explicitly addressed by the master.

## Protocol overview

* **Encoding** – every logical bit is transmitted as two identical
  "cells": `0 → 00`, `1 → 11`.
* **Sync** – frames begin with the 15‑cell sequence
  `000111000111000`.  Because encoded data never contains an odd run of
  bits, this pattern cannot appear in payload and clearly marks the start of a
  message.
* **Payload format** – after SYNC the master sends 16 logical bits
  (32 cells) packed as `MTYPE(2) | A(6) | CMD(4) | CRC(4)` followed by two guard
  cells "11".
  * `MTYPE` values: `0` = `POLL_ID`, `1` = `CMD_TO_TYPE`, `2` = `CMD_TO_ID`.
  * `A` is either the slave ID or type depending on `MTYPE`.
  * `CMD` is a 4‑bit command.  `CMD=0` in `CMD_TO_ID` means **WHOAREYOU**.
  * `CRC` is a 4‑bit checksum of the preceding 12 bits (polynomial `x⁴+x+1`)
    used to detect corrupted frames.
* **Response windows** – after the guard cells the master releases the data
  line and optionally samples a reply:
  * After `POLL_ID` the addressed slave may answer with two cells: `00`
    means *present*, `11` means *no device*.
  * After `WHOAREYOU` the slave returns its 6‑bit type encoded as six dibits
    (12 cells).

## Timing & throughput

The default cell period is **1 ms** (1 kHz clock).  A command frame consists of
15 sync cells, 32 payload cells, and 2 guard cells – 49 cells total – so one
frame takes 49 ms.  That yields roughly **20 frames per second** carrying
16 payload bits each, for a raw throughput of about **320 bit/s** (~40 B/s).
Presence (2‑cell) or type (12‑cell) response windows slightly reduce this rate.

The odd‑length SYNC sequence and dibit encoding allow slaves to resynchronise
after resets without false triggers, while the 4‑bit CRC guards against
corrupted payloads.

## Getting started

```
#include <com-prot.h>

// Master on pins D2 (DATA) and D1 (CLK)
StarWire::ComProtMaster master(1, D2, D1);

void setup() {
  Serial.begin(115200);
  master.begin();
}

void loop() {
  master.update();       // polls for presence and handles responses
  // ... send commands with master.sendCommandToSlaveId(...) etc.
}
```

For a complete example see `examples/`.

## Installation

### PlatformIO

```ini
lib_deps =
    https://github.com/EnergetickaAkademie/com-prot.git
```

### Arduino IDE

1. Download this repository as ZIP.
2. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library**.
3. Select the downloaded file.

## License

MIT

