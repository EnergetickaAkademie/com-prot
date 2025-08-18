#pragma once
#include <Arduino.h>
#include <vector>
#include <functional>

/*
  StarWire (CLK+DATA) protocol for ESP8266
  - DATA open-drain: 1=release (INPUT / open-drain HIGH), 0=drive low (OUTPUT LOW)
  - Bit symbol (payload): logical 0 -> "00", logical 1 -> "11" (cells)
  - SYNC (cells): "000111000111000" (15 cells, odd-length runs; cannot occur in data)
  - Frame: MTYPE(2) + A(6) + CMD(4)  => 12 logical bits => 24 cells
  - Reply windows:
      PRESENCE2 (after POLL_ID): 2 cells; YES="00", NO="11"
      TYPE12    (after CMD_TO_ID with CMD=0): 12 cells; TYPE(6) encoded as dibits
*/

namespace StarWire {

// ---------- Low-level config ----------
struct Pins {
  uint8_t clk;   // master: OUTPUT; slave: INPUT (RISING ISR)
  uint8_t data;  // open-drain
};

static const uint16_t CELL_US_DEFAULT = 40;    // 25 kHz cell clock
static const uint8_t  MAX_SLAVES_HINT = 20;

// ---------- Encoding helpers ----------
inline void encode12_to_24cells(uint16_t bits12, uint8_t *cells24) {
  for (int i=11, w=0; i>=0; --i) {
    uint8_t b = (bits12 >> i) & 1;
    cells24[w++] = b; cells24[w++] = b;
  }
}

static const uint8_t SYNC_LEN = 15;
static const uint8_t SYNC_PATTERN[SYNC_LEN] = { 0,0,0, 1,1,1, 0,0,0, 1,1,1, 0,0,0 };

// ---------- Message types ----------
enum MType : uint8_t {
  POLL_ID      = 0b00,
  CMD_TO_TYPE  = 0b01,
  CMD_TO_ID    = 0b10
};

// Special CMDs
static const uint8_t CMD_WHOAREYOU = 0x0;

// Pack fields to 12 logical bits: MTYPE(2), A(6), CMD(4)
inline uint16_t pack12(uint8_t mtype2, uint8_t A6, uint8_t cmd4) {
  return ((uint16_t)(mtype2 & 0x3) << 10) |
         ((uint16_t)(A6 & 0x3F)  << 4)  |
         ((uint16_t)(cmd4 & 0x0F));
}

// ---------- Open-drain helpers (non-ISR use OK) ----------
inline void data_release(uint8_t pin) { pinMode(pin, INPUT); }
inline void data_drive0(uint8_t pin)  { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
inline int  data_read(uint8_t pin)    { return digitalRead(pin); }

// ---------- Event structs ----------
struct SlaveInfo {
  uint8_t id;
  uint8_t type;          // 0xFF = unknown
  unsigned long lastSeenMs;
};

using CommandHandler = std::function<void(uint8_t cmd4, const uint8_t senderId)>;
using DebugReceiveHandler = std::function<void(uint8_t mtype, uint8_t A6, uint8_t cmd4, uint8_t sender)>;

// ---------------------------------------------------------------------------
// Base
// ---------------------------------------------------------------------------
class ComProtBase {
protected:
  Pins pins;
  uint16_t cellUs;
  DebugReceiveHandler debugHandler;

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
  explicit ComProtMaster(uint8_t masterId, uint8_t dataPin, uint8_t clkPin,
                         unsigned long heartbeatTimeoutMs = 3100,
                         uint16_t cell_us = CELL_US_DEFAULT);
  ~ComProtMaster();

  void begin();
  void update(); // non-blocking: polling, presence & type events, timeouts

  void setHeartbeatTimeout(unsigned long ms) { heartbeatTimeout = ms; }

  std::vector<SlaveInfo> getConnectedSlaves();
  std::vector<SlaveInfo> getSlavesByType(uint8_t type);
  bool isSlaveConnected(uint8_t id);

  // 4-bit commands (native)
  bool sendCommandToSlaveType(uint8_t slaveType, uint8_t cmd /* 0..15 */);
  bool sendCommandToSlaveId(uint8_t slaveId, uint8_t cmd /* 0..15 */);

  // API-compat overloads (payload unsupported -> return false if dataLen>0)
  bool sendCommandToSlaveType(uint8_t slaveType, uint8_t command, uint8_t* data, uint16_t dataLen);
  bool sendCommandToSlaveId(uint8_t slaveId, uint8_t command, uint8_t* data, uint16_t dataLen);

  uint8_t getMasterId() const { return masterId; }
  size_t  getSlaveCount() const { return slaves.size(); }

private:
  uint8_t masterId;
  unsigned long heartbeatTimeout;

  // scanning
  uint8_t scanNextId = 0;
  volatile uint8_t currentlyPollingId = 0xFF;
  volatile uint8_t pendingWhoId = 0xFF; // schedule WHOAREYOU for this id
  volatile uint8_t whoTargetId = 0xFF;  // ID pro právě běžící WHOAREYOU/TYPE12

  // ISR state
  static ComProtMaster* self;
  volatile bool ticking = false;

  enum RespMode : uint8_t { RESP_NONE=0, RESP_PRESENCE2=1, RESP_TYPE12=2 };
  volatile RespMode respMode = RESP_NONE;

  static const uint16_t MAX_CELLS = 15 + 24 + 2; // SYNC + payload + GUARD
  volatile uint8_t  txCells[MAX_CELLS];
  volatile uint16_t txLen = 0;
  volatile uint16_t cellIdx = 0;

  // Response reading
  volatile uint8_t  respCells[12];
  volatile uint8_t  respNeed = 0;
  volatile uint8_t  respHave = 0;

  // Events from ISR
  volatile bool     presenceEvtPending = false;
  volatile uint8_t  presenceEvtId = 0;
  volatile uint8_t  presenceEvtYes = 0;

  volatile bool     typeEvtPending = false;
  volatile uint8_t  typeEvtId = 0;
  volatile uint8_t  typeEvtVal = 0;

  // list
  std::vector<SlaveInfo> slaves;

  // pins
  uint8_t DATA, CLK;

  // timer
  void startTicker();
  void stopTicker();
  static void IRAM_ATTR onTickISR();

  // helpers
  void buildFrameAndKick(uint8_t mtype, uint8_t A6, uint8_t cmd4, RespMode rm);
  void schedulePollIfIdle();
  void scheduleWhoIfNeeded();
  void handlePresenceResult(uint8_t polledId, bool present);
  void handleTypeResult(uint8_t id, uint8_t type);
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
  void update();

  void setHeartbeatInterval(unsigned long) {/* API parity, unused */}
  void setCommandHandler(uint8_t cmdType /* 0..15 */, CommandHandler h);
  void removeCommandHandler(uint8_t cmdType);

  bool sendResponse(uint8_t /*commandType*/, uint8_t* /*data*/=nullptr, uint16_t /*len*/=0) { return false; }

  uint8_t getSlaveId() const { return slaveId; }
  uint8_t getSlaveType() const { return slaveType; }
  uint8_t getMasterId() const { return masterId; }

private:
  uint8_t slaveId, slaveType, masterId;

  // RX state machine
  enum RxState : uint8_t { RX_WAIT_SYNC=0, RX_PAYLOAD, RX_POST_GUARD, RX_RESP_WINDOW };
  volatile RxState rxState = RX_WAIT_SYNC;

  // SYNC window (15 bits)
  volatile uint16_t syncWindow = 0;

  // payload
  volatile uint8_t  recvCells[24];
  volatile uint8_t  recvIdx = 0;

  // counters
  volatile uint8_t  postCount = 0;     // counts guard cells
  volatile uint8_t  replyCount = 0;    // counts reply cells

  // reply modes
  volatile bool     expectPresenceReply = false; // 2 cells "00"
  volatile bool     expectTypeReply = false;     // 12 cells encoded type
  volatile uint8_t  typeReplyCells[12];
  volatile uint8_t  typeReplyLen = 0;
  volatile uint8_t  typeReplyIdx = 0;

  // decoded last (debug)
  volatile uint8_t last_mtype = 0, last_A6 = 0, last_cmd4 = 0;

  // pending user command (mimo ISR)
  volatile bool     pendingCmdValid = false;
  volatile uint8_t  pendingCmd4 = 0;

  // command handlers
  CommandHandler handlers[16];

  // pins
  uint8_t DATA, CLK;

  // ISR
  static ComProtSlave* self;
  static void IRAM_ATTR onClkRiseISR();

  // helpers
  void handleDecoded(uint8_t mtype, uint8_t A6, uint8_t cmd4);
  static uint8_t IRAM_ATTR decode12_from_24cells(const volatile uint8_t *cells24, uint16_t &bits12_out);
};

} // namespace StarWire
