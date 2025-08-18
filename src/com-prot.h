#pragma once
#include <Arduino.h>
#include <vector>
#include <functional>

/*
  StarWire (CLK+DATA) protocol for ESP8266
  - DATA open-drain: 1=release (INPUT), 0=drive low (OUTPUT LOW)
  - Bit symbol: logical 0 -> "00", logical 1 -> "11" (cells)
  - SYNC: "000111000111000" (15 cells, odd-length runs)
  - Frame: MTYPE(2) + A(6) + CMD(4)  => 12 logical bits => 24 cells
  - Reply window: 2 cells; YES="00" (pull low), NO="11" (release)
*/

namespace StarWire {

// ---------- Low-level config ----------
struct Pins {
  uint8_t clk;   // master: OUTPUT; slave: INPUT w/ interrupt
  uint8_t data;  // open-drain
};

static const uint16_t CELL_US_DEFAULT = 40;    // 25 kHz
static const uint8_t  MAX_SLAVES_HINT = 20;    // only for vector reserve

// ---------- Encoding helpers ----------
inline uint8_t encCellBit(uint8_t logicalBit) { return logicalBit ? 1 : 0; } // just for clarity
// Encode 12 logical bits -> 24 cells into buffer
inline void encode12_to_24cells(uint16_t bits12, uint8_t *cells24 /* bytes 0/1 per cell */) {
  // MSB first across 12 bits
  for (int i=11, w=0; i>=0; --i) {
    uint8_t b = (bits12 >> i) & 1;
    cells24[w++] = b; // first cell
    cells24[w++] = b; // second cell
  }
}

static const uint8_t SYNC_LEN = 15;
static const uint8_t SYNC_PATTERN[SYNC_LEN] = {
  0,0,0, 1,1,1, 0,0,0, 1,1,1, 0,0,0
};

// ---------- Message types ----------
enum MType : uint8_t {
  POLL_ID      = 0b00,
  CMD_TO_TYPE  = 0b01,
  CMD_TO_ID    = 0b10
};

// Pack fields to 12 logical bits: MTYPE(2), A(6), CMD(4)
inline uint16_t pack12(uint8_t mtype2, uint8_t A6, uint8_t cmd4) {
  return ((uint16_t)(mtype2 & 0x3) << 10) |
         ((uint16_t)(A6 & 0x3F)  << 4)  |
         ((uint16_t)(cmd4 & 0x0F));
}

// ---------- Open-drain helpers ----------
inline void data_release(uint8_t pin) { pinMode(pin, INPUT); }               // bus HIGH via pull-up
inline void data_drive0(uint8_t pin)  { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); } // pull low
inline int  data_read(uint8_t pin)    { return digitalRead(pin); }

// ---------- Event structs ----------
struct SlaveInfo {
  uint8_t id;
  uint8_t type;
  unsigned long lastSeenMs;
};

using CommandHandler = std::function<void(uint8_t cmd4, const uint8_t senderId)>; // senderId=masterId for us
using DebugReceiveHandler = std::function<void(uint8_t mtype, uint8_t A6, uint8_t cmd4, uint8_t sender)>;

// ---------------------------------------------------------------------------
// Base class (stats + debug)
// ---------------------------------------------------------------------------
class ComProtBase {
protected:
  Pins pins;
  uint16_t cellUs;
  DebugReceiveHandler debugHandler;

  // stats
  unsigned long lastEdgeMicros = 0;
  unsigned long edges = 0;

public:
  explicit ComProtBase(Pins p, uint16_t cell_us = CELL_US_DEFAULT)
  : pins(p), cellUs(cell_us), debugHandler(nullptr) {}

  virtual ~ComProtBase() {}

  void setDebugReceiveHandler(DebugReceiveHandler cb) { debugHandler = cb; }
  void removeDebugReceiveHandler() { debugHandler = nullptr; }

  void setCellPeriodUs(uint16_t us) { cellUs = us; }
};

// ---------------------------------------------------------------------------
// Master
// ---------------------------------------------------------------------------
class ComProtMaster : public ComProtBase {
public:
  // API compatibility with your existing code
  explicit ComProtMaster(uint8_t masterId, uint8_t dataPin, uint8_t clkPin, unsigned long heartbeatTimeoutMs = 3100,
                         uint16_t cell_us = CELL_US_DEFAULT);

  ~ComProtMaster();

  void begin();
  void update(); // non-blocking: scanning & timeouts
  void setHeartbeatTimeout(unsigned long ms) { heartbeatTimeout = ms; }

  // scanning/presence
  std::vector<SlaveInfo> getConnectedSlaves();
  std::vector<SlaveInfo> getSlavesByType(uint8_t type);
  bool isSlaveConnected(uint8_t id);

  // commands
  bool sendCommandToSlaveType(uint8_t slaveType, uint8_t cmd /* 4 bits */);
  bool sendCommandToSlaveId(uint8_t slaveId, uint8_t cmd /* 4 bits */);

  uint8_t getMasterId() const { return masterId; }
  size_t  getSlaveCount() const { return slaves.size(); }

private:
  uint8_t masterId;
  unsigned long heartbeatTimeout;

  // scanning state
  uint8_t scanNextId = 0;

  // ISR state
  static ComProtMaster* self;
  volatile bool ticking = false;

  // prepared TX bit-cells (SYNC + 24 cells payload + GUARD(2x1) + RESP(2 read) total scheduling)
  static const uint16_t MAX_CELLS = 15 + 24 + 2;
  volatile uint8_t  txCells[MAX_CELLS];
  volatile uint16_t txLen = 0;
  volatile uint16_t cellIdx = 0;
  volatile bool     inRespWindow = false;
  volatile uint8_t  respSample[2];
  volatile uint8_t  respSampleIdx = 0;

  // list
  std::vector<SlaveInfo> slaves;

  // pins cache
  uint8_t DATA, CLK;

  // ticker/timer
  void startTicker();
  void stopTicker();
  static void IRAM_ATTR onTickISR();

  // internal helpers
  void buildFrameAndKick(uint8_t mtype, uint8_t A6, uint8_t cmd4);
  void handlePresenceResult(uint8_t polledId, bool present);
  std::vector<SlaveInfo>::iterator findSlave(uint8_t id);
  void checkTimeouts();
};

// ---------------------------------------------------------------------------
// Slave
// ---------------------------------------------------------------------------
class ComProtSlave : public ComProtBase {
public:
  explicit ComProtSlave(uint8_t slaveId, uint8_t slaveType, uint8_t dataPin, uint8_t clkPin,
                        uint8_t masterId = 1, unsigned long heartbeatIntervalMs = 1000,
                        uint16_t cell_us = CELL_US_DEFAULT);

  ~ComProtSlave();

  void begin();
  void update(); // handles heartbeat scheduling (via master polling model we don't transmit here)

  void setHeartbeatInterval(unsigned long) {/* kept for API parity, unused */}
  void setCommandHandler(uint8_t cmdType /* 0..15 */, CommandHandler h);
  void removeCommandHandler(uint8_t cmdType);

  // NOTE: star-wire is strictly master-driven; slaves do not push spontaneous frames.
  // sendResponse is a no-op kept for API compatibility.
  bool sendResponse(uint8_t /*commandType*/, uint8_t* /*data*/=nullptr, uint16_t /*len*/=0) { return false; }

  uint8_t getSlaveId() const { return slaveId; }
  uint8_t getSlaveType() const { return slaveType; }
  uint8_t getMasterId() const { return masterId; }

private:
  uint8_t slaveId, slaveType, masterId;

  // ring buffer of last 15 cells for SYNC detection
  volatile uint16_t syncWindow = 0; // use 16-bit, we only need 15 bits

  // frame reception
  volatile uint8_t  collecting = 0;
  volatile uint8_t  recvCells[24];
  volatile uint8_t  recvIdx = 0;
  volatile bool     expectReplyWindow = false;
  volatile bool     mustReplyYes = false;

  // decoded last frame fields
  volatile uint8_t last_mtype = 0, last_A6 = 0, last_cmd4 = 0;

  // command handlers
  CommandHandler handlers[16]; // simple fixed array

  // pins cache
  uint8_t DATA, CLK;

  // ISR
  static ComProtSlave* self;
  static void IRAM_ATTR onClkRiseISR();

  // helpers
  static bool matchSYNC(uint16_t w);
  void handleDecoded(uint8_t mtype, uint8_t A6, uint8_t cmd4);
static uint8_t decode12_from_24cells(const volatile uint8_t *cells24, uint16_t &bits12_out);
};

} // namespace StarWire
