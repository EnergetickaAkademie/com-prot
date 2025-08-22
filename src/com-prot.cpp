#include "com-prot.h"
using namespace StarWire;

extern "C" {
  #include "ets_sys.h"
  #include "os_type.h"
  #include "osapi.h"
  #include "user_interface.h"
  #include "gpio.h"     // fast GPIO: GPOS/GPOC, GPIO_REG_READ
}

// ---------- Fast GPIO helpers (IRAM-safe) ----------
static inline uint32_t gpio_mask(uint8_t gpio){ return (1U << gpio); }

static inline void IRAM_ATTR fast_clk_high(uint8_t gpio){ GPOS = gpio_mask(gpio); }
static inline void IRAM_ATTR fast_clk_low (uint8_t gpio){ GPOC = gpio_mask(gpio); }

static inline void IRAM_ATTR fast_data_release(uint8_t gpio){ GPOS = gpio_mask(gpio); } // open-drain: HIGH = release
static inline void IRAM_ATTR fast_data_low    (uint8_t gpio){ GPOC = gpio_mask(gpio); } // drive LOW

static inline uint8_t IRAM_ATTR fast_data_read(uint8_t gpio){
  return (GPIO_REG_READ(GPIO_IN_ADDRESS) & gpio_mask(gpio)) ? 1 : 0;
}

// SYNC window constants (15 bits)
static const uint16_t SYNC_MASK = 0x7FFF;
static const uint16_t SYNC_WORD = 0b000111000111000;

// Timer1 ticks: CPU/16 per microsecond base
static inline uint32_t ticks_per_us() { return (uint32_t)(system_get_cpu_freq() / 16); }
static inline uint32_t cell_ticks_us(uint16_t us) {
  uint32_t tpu = ticks_per_us();
  uint32_t ticks = (uint32_t)us * (tpu ? tpu : 5);
  if (ticks < 2) ticks = 2;
  return ticks;
}

// ===================== Master =====================
ComProtMaster* ComProtMaster::self = nullptr;

ComProtMaster::ComProtMaster(uint8_t _masterId, uint8_t dataPin, uint8_t clkPin,
                             unsigned long heartbeatTimeoutMs, uint16_t cell_us)
: ComProtBase({clkPin, dataPin}, cell_us),
  masterId(_masterId),
  heartbeatTimeout(heartbeatTimeoutMs),
  DATA(dataPin), CLK(clkPin)
{
  slaves.reserve(MAX_SLAVES_HINT);
}

ComProtMaster::~ComProtMaster() {
  stopTicker();
  if (self == this) self = nullptr;
}

void ComProtMaster::begin() {
  // CLK as push-pull output (we toggle via fast_* macros)
  pinMode(CLK, OUTPUT);
  fast_clk_low(CLK);

  // DATA as hardware open-drain, released high by default
  pinMode(DATA, OUTPUT_OPEN_DRAIN);
  fast_data_release(DATA);

  self = this;
  startTicker();
}

void ComProtMaster::startTicker() {
  if (ticking) return;
  ticking = true;
  timer1_isr_init();
  timer1_attachInterrupt(onTickISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  // generate one rising edge per cell: toggle at half-cell
  uint16_t halfCellUs = (cellUs >= 2) ? (cellUs / 2) : 1;
  timer1_write(cell_ticks_us(halfCellUs));
}

void ComProtMaster::stopTicker() {
  if (!ticking) return;
  timer1_disable();
  ticking = false;
}

// Build: SYNC(15) + payload+CRC(32) + GUARD("11"); choose response mode
void ComProtMaster::buildFrameAndKick(uint8_t mtype, uint8_t A6, uint8_t cmd4, RespMode rm) {
  uint16_t bits16 = pack16_with_crc(mtype, A6, cmd4);
  uint8_t payload[32];
  encode16_to_32cells(bits16, payload);

  uint16_t i=0;
  for (uint8_t k=0;k<SYNC_LEN;++k) txCells[i++] = SYNC_PATTERN[k];
  for (uint8_t k=0;k<32;++k)       txCells[i++] = payload[k];
  txCells[i++] = 1; txCells[i++] = 1; // GUARD

  txLen = i;
  cellIdx = 0;

  respMode = rm;
  respNeed = (rm==RESP_PRESENCE2 ? 2 : (rm==RESP_TYPE12 ? 12 : 0));
  respHave = 0;

  if (mtype == POLL_ID) {
    currentlyPollingId = (A6 & 0x3F);
    whoTargetId = 0xFF;
  } else if (rm == RESP_TYPE12) {
    // remember WHO target to attach TYPE to correct ID
    whoTargetId = (A6 & 0x3F);
    currentlyPollingId = 0xFF;
  } else {
    currentlyPollingId = 0xFF;
    whoTargetId = 0xFF;
  }
}

void ComProtMaster::schedulePollIfIdle() {
  if (respNeed==0 && cellIdx >= txLen) {
    uint8_t id = scanNextId;
    buildFrameAndKick(POLL_ID, id, 0, RESP_PRESENCE2);
    scanNextId = (scanNextId + 1) & 0x3F;
  }
}

void ComProtMaster::scheduleWhoIfNeeded() {
  // schedule WHO only when idle
  if (pendingWhoId != 0xFF && respNeed==0 && cellIdx >= txLen) {
    uint8_t id = pendingWhoId;
    pendingWhoId = 0xFF;
    buildFrameAndKick(CMD_TO_ID, id, CMD_WHOAREYOU, RESP_TYPE12);
  }
}

void ComProtMaster::update() {
  // 1) consume presence event
  if (presenceEvtPending) {
    noInterrupts();
    uint8_t id   = presenceEvtId;
    bool    yes  = presenceEvtYes;
    presenceEvtPending = false;
    interrupts();

    if (yes) {
      handlePresenceResult(id, true);
      // if type unknown, schedule WHO
      auto it = findSlave(id);
      if (it != slaves.end() && it->type == 0xFF) pendingWhoId = id;
    }
  }

  // 2) consume type event
  if (typeEvtPending) {
    noInterrupts();
    uint8_t id = typeEvtId;
    uint8_t tp = typeEvtVal;
    typeEvtPending = false;
    interrupts();
    handleTypeResult(id, tp);
  }

  // 3) prefer WHO over POLL when idle
  scheduleWhoIfNeeded();
  schedulePollIfIdle();

  // 4) timeouts
  checkTimeouts();
}

// ISR: toggle CLK each half-cell; act on rising edges (one per cell)
void IRAM_ATTR ComProtMaster::onTickISR() {
  if (!self) return;

  static bool clkHigh = false;
  clkHigh = !clkHigh;
  if (clkHigh) fast_clk_high(self->CLK); else fast_clk_low(self->CLK);
  if (!clkHigh) return; // act on rising edge (one per cell)

  if (self->cellIdx < self->txLen) {
    uint8_t b = self->txCells[self->cellIdx++];
    if (b) fast_data_release(self->DATA); else fast_data_low(self->DATA);
    if (self->cellIdx >= self->txLen) {
      // payload+guard done -> release for response window if any
      fast_data_release(self->DATA);
    }
  } else if (self->respNeed) {
    // read response cells
    if (self->respHave < self->respNeed) {
      self->respCells[self->respHave++] = fast_data_read(self->DATA);
      if (self->respHave >= self->respNeed) {
        // response complete
        if (self->respMode == RESP_PRESENCE2) {
          bool present = (self->respCells[0]==0 && self->respCells[1]==0);
          if (self->currentlyPollingId != 0xFF) {
            self->presenceEvtId = self->currentlyPollingId;
            self->presenceEvtYes = present ? 1 : 0;
            self->presenceEvtPending = true;
          }
        } else if (self->respMode == RESP_TYPE12) {
          // decode 6 bits from dibits. Treat a response that never pulls the
          // line LOW ("111111" -> type 63) as invalid to avoid false positives
          // when no slave replies.
          uint8_t t = 0;
          bool ok = true;
          bool sawZero = false;
          for (int i=0;i<12;i+=2) {
            uint8_t a = self->respCells[i];
            uint8_t b = self->respCells[i+1];
            if (a!=b) { ok=false; break; }
            if (a==0) sawZero = true;
            t = (uint8_t)((t<<1) | (a?1:0));
          }
          if (ok && sawZero && self->whoTargetId != 0xFF) {
            self->typeEvtId  = self->whoTargetId;
            self->typeEvtVal = (t & 0x3F);
            self->typeEvtPending = true;
          }
          self->whoTargetId = 0xFF; // always reset after TYPE
        }
        // reset resp
        self->respNeed = 0;
        self->respHave = 0;
        self->respMode = RESP_NONE;
      }
    }
  }
}

void ComProtMaster::handlePresenceResult(uint8_t polledId, bool present) {
  if (present) {
    auto it = findSlave(polledId);
    if (it == slaves.end()) {
      slaves.push_back({polledId, 0xFF, millis()});
    } else {
      it->lastSeenMs = millis();
    }
  }
}

void ComProtMaster::handleTypeResult(uint8_t id, uint8_t type) {
  auto it = findSlave(id);
  if (it == slaves.end()) {
    slaves.push_back({id, type & 0x3F, millis()});
  } else {
    it->type = (type & 0x3F);
    it->lastSeenMs = millis();
  }
}

std::vector<SlaveInfo>::iterator ComProtMaster::findSlave(uint8_t id) {
  for (auto it = slaves.begin(); it != slaves.end(); ++it) if (it->id == id) return it;
  return slaves.end();
}

void ComProtMaster::checkTimeouts() {
  unsigned long now = millis();
  for (auto it = slaves.begin(); it != slaves.end();) {
    if (now - it->lastSeenMs > heartbeatTimeout) it = slaves.erase(it);
    else ++it;
  }
}

std::vector<SlaveInfo> ComProtMaster::getConnectedSlaves() { return slaves; }

std::vector<SlaveInfo> ComProtMaster::getSlavesByType(uint8_t type) {
  std::vector<SlaveInfo> r; r.reserve(slaves.size());
  for (auto &s: slaves) if (s.type==type) r.push_back(s);
  return r;
}

bool ComProtMaster::isSlaveConnected(uint8_t id) { return findSlave(id) != slaves.end(); }

bool ComProtMaster::sendCommandToSlaveType(uint8_t slaveType, uint8_t cmd) {
  buildFrameAndKick(CMD_TO_TYPE, slaveType & 0x3F, cmd & 0x0F, RESP_NONE);
  return true;
}

bool ComProtMaster::sendCommandToSlaveId(uint8_t slaveId, uint8_t cmd) {
  buildFrameAndKick(CMD_TO_ID, slaveId & 0x3F, cmd & 0x0F, RESP_NONE);
  return true;
}

bool ComProtMaster::sendCommandToSlaveType(uint8_t slaveType, uint8_t command, uint8_t* data, uint16_t dataLen) {
  if (dataLen > 0) return false; // payload not supported on this wire (4-bit only)
  return sendCommandToSlaveType(slaveType, command & 0x0F);
}

bool ComProtMaster::sendCommandToSlaveId(uint8_t slaveId, uint8_t command, uint8_t* data, uint16_t dataLen) {
  if (dataLen > 0) return false;
  return sendCommandToSlaveId(slaveId, command & 0x0F);
}

// ===================== Slave =====================
ComProtSlave* ComProtSlave::self = nullptr;

ComProtSlave::ComProtSlave(uint8_t _slaveId, uint8_t _slaveType, uint8_t dataPin, uint8_t clkPin,
                           uint8_t _masterId, unsigned long /*hb*/, uint16_t cell_us)
: ComProtBase({clkPin, dataPin}, cell_us),
  slaveId(_slaveId), slaveType(_slaveType), masterId(_masterId),
  DATA(dataPin), CLK(clkPin)
{
  for (auto &h : handlers) h = nullptr;
}

ComProtSlave::~ComProtSlave() {
  if (self == this) self = nullptr;
  detachInterrupt(digitalPinToInterrupt(CLK));
}

void ComProtSlave::begin() {
  pinMode(CLK, INPUT);                   // clock jen čteme
  pinMode(DATA, OUTPUT_OPEN_DRAIN);      // DATA jako open-drain
  fast_data_release(DATA);               // nikdy neřídit mimo reply okno

  rxState = RX_WAIT_SYNC;
  syncWindow = 0;
  recvIdx = 0;
  postCount = 0;
  replyCount = 0;
  expectPresenceReply = false;
  expectTypeReply = false;
  typeReplyLen = 0;
  typeReplyIdx = 0;
  pendingCmdValid = false;
  pendingCmd4 = 0;

  self = this;
  attachInterrupt(digitalPinToInterrupt(CLK), onClkRiseISR, RISING);
}

void ComProtSlave::update() {
  // předej případný user command mimo ISR
  if (pendingCmdValid) {
    noInterrupts();
    uint8_t cmd = pendingCmd4;
    pendingCmdValid = false;
    interrupts();
    if (handlers[cmd]) handlers[cmd](cmd, masterId);
  }
}

void ComProtSlave::setCommandHandler(uint8_t cmdType, CommandHandler h) {
  if (cmdType < 16) handlers[cmdType] = h;
}

void ComProtSlave::removeCommandHandler(uint8_t cmdType) {
  if (cmdType < 16) handlers[cmdType] = nullptr;
}

uint8_t IRAM_ATTR ComProtSlave::decode16_from_32cells(const volatile uint8_t *cells32, uint16_t &bits16_out) {
  uint16_t v = 0;
  for (int i=0;i<32; i+=2) {
    uint8_t a = cells32[i];
    uint8_t b = cells32[i+1];
    if (a!=b) return 1;
    v = (uint16_t)((v << 1) | (a ? 1 : 0));
  }
  uint16_t data12 = (v >> 4);
  uint8_t  crc    = (uint8_t)(v & 0x0F);
  if (crc4(data12) != crc) return 2;
  bits16_out = v;
  return 0;
}

void ComProtSlave::handleDecoded(uint8_t mtype, uint8_t A6, uint8_t cmd4) {
  last_mtype = mtype; last_A6 = A6; last_cmd4 = cmd4;

  // default clear
  expectPresenceReply = false;
  expectTypeReply = false;

  if (mtype == POLL_ID && A6 == (slaveId & 0x3F)) {
    expectPresenceReply = true; // YES=00
    return;
  }

  if (mtype == CMD_TO_TYPE && A6 == (slaveType & 0x3F)) {
    // handler zavolej až mimo ISR
    pendingCmd4 = cmd4 & 0x0F;
    pendingCmdValid = true;
  } else if (mtype == CMD_TO_ID && A6 == (slaveId & 0x3F)) {
    if (cmd4 == CMD_WHOAREYOU) {
      // prepare TYPE reply: 6 bits -> 12 cells
      uint8_t t = (slaveType & 0x3F);
      for (int i=5, w=0; i>=0; --i) {
        uint8_t b = (t >> i) & 1;
        typeReplyCells[w++] = b;
        typeReplyCells[w++] = b;
      }
      typeReplyLen = 12;
      typeReplyIdx = 0;
      expectTypeReply = true;
    } else {
      pendingCmd4 = cmd4 & 0x0F;
      pendingCmdValid = true;
    }
  }
}

void IRAM_ATTR ComProtSlave::onClkRiseISR() {
  if (!self) return;

  int d = fast_data_read(self->DATA);
  self->syncWindow = ((self->syncWindow << 1) | (d & 1)) & SYNC_MASK;

  switch (self->rxState) {
    case RX_WAIT_SYNC:
      if (self->syncWindow == SYNC_WORD) {
        self->rxState = RX_PAYLOAD;
        self->recvIdx = 0;
        self->postCount = 0;
        self->replyCount = 0;
      }
      break;

    case RX_PAYLOAD:
      if (self->recvIdx < 32) {
        self->recvCells[self->recvIdx++] = (uint8_t)d;
        if (self->recvIdx >= 32) {
          uint16_t bits16 = 0;
          if (!decode16_from_32cells(self->recvCells, bits16)) {
            uint16_t bits12 = (bits16 >> 4);
            uint8_t mtype = (bits12 >> 10) & 0x03;
            uint8_t A6    = (bits12 >>  4) & 0x3F;
            uint8_t cmd4  =  bits12        & 0x0F;
            self->handleDecoded(mtype, A6, cmd4);
            self->rxState = RX_POST_GUARD;
            self->postCount = 0;
            self->replyCount = 0;
          } else {
            self->rxState = RX_WAIT_SYNC;
          }
        }
      }
      break;

    case RX_POST_GUARD:
      // count two guard cells "11"
      self->postCount++;
      if (self->postCount >= 2) {
        self->postCount = 0;
        self->replyCount = 0;

        // start reply window: preset FIRST cell value BEFORE its rising edge
        if (self->expectTypeReply && self->typeReplyLen == 12) {
          uint8_t c0 = self->typeReplyCells[self->typeReplyIdx++];
          if (c0==0) fast_data_low(self->DATA); else fast_data_release(self->DATA);
          self->rxState = RX_RESP_WINDOW;
        } else if (self->expectPresenceReply) {
          // PRESENCE YES="00" -> hold low across both reply cells
          fast_data_low(self->DATA);
          self->rxState = RX_RESP_WINDOW;
        } else {
          // NO -> released
          fast_data_release(self->DATA);
          self->rxState = RX_RESP_WINDOW;
        }
      }
      break;

    case RX_RESP_WINDOW:
      if (self->expectTypeReply && self->typeReplyLen == 12) {
        // Stream remaining 11 cells, always preparing NEXT cell level at rising edge
        if (self->typeReplyIdx < self->typeReplyLen) {
          uint8_t c = self->typeReplyCells[self->typeReplyIdx++];
          if (c==0) fast_data_low(self->DATA); else fast_data_release(self->DATA);
        } else {
          // finished all 12 cells
          fast_data_release(self->DATA);
          self->expectTypeReply = false;
          self->typeReplyLen = 0;
          self->typeReplyIdx = 0;
          self->rxState = RX_WAIT_SYNC;
        }
      } else if (self->expectPresenceReply) {
        // Keep LOW for exactly 2 reply cells, then release.
        self->replyCount++;
        if (self->replyCount >= 2) {
          fast_data_release(self->DATA);
          self->expectPresenceReply = false;
          self->replyCount = 0;
          self->rxState = RX_WAIT_SYNC;
        } else {
          // keep holding low for the second cell too
          fast_data_low(self->DATA);
        }
      } else {
        // NO reply ("11"): stay released for 2 cells, then finish
        self->replyCount++;
        if (self->replyCount >= 2) {
          fast_data_release(self->DATA);
          self->replyCount = 0;
          self->rxState = RX_WAIT_SYNC;
        } else {
          fast_data_release(self->DATA);
        }
      }
      break;
  }
}
