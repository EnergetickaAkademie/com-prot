#include "com-prot.h"
using namespace StarWire;

extern "C" {
  #include "ets_sys.h"
  #include "os_type.h"
  #include "osapi.h"
  #include "user_interface.h"
}

// Timer1: CPU / 16 ticks per second
// cpu (MHz) = 80 or 160; ticksPerUs = cpu/16 (5 or 10)
static inline uint32_t ticks_per_us() { return (uint32_t)(system_get_cpu_freq() / 16); }
static inline uint32_t cell_ticks_us(uint16_t us) {
  uint32_t tpu = ticks_per_us();
  uint32_t ticks = (uint32_t)us * (tpu ? tpu : 5);
  if (ticks < 2) ticks = 2; // guard against zero / too fast
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
  pinMode(CLK, OUTPUT);
  digitalWrite(CLK, LOW);
  data_release(DATA); // idle HIGH via pull-up
  self = this;
  startTicker();
}

void ComProtMaster::startTicker() {
  if (ticking) return;
  ticking = true;
  timer1_isr_init();
  timer1_attachInterrupt(onTickISR);
  // TIM_DIV16 => tick = 1/(CPU/16) us; we want one ISR per HALF cell => rising edges every cell
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  uint16_t halfCellUs = (cellUs >= 2) ? (cellUs / 2) : 1;
  timer1_write(cell_ticks_us(halfCellUs));
}

void ComProtMaster::stopTicker() {
  if (!ticking) return;
  timer1_disable();
  ticking = false;
}

// Build: SYNC(15) + payload(24 cells) + GUARD("11")
void ComProtMaster::buildFrameAndKick(uint8_t mtype, uint8_t A6, uint8_t cmd4) {
  uint16_t bits12 = pack12(mtype, A6, cmd4);
  uint8_t payload[24];
  encode12_to_24cells(bits12, payload);

  uint16_t i=0;
  for (uint8_t k=0;k<SYNC_LEN;++k) txCells[i++] = SYNC_PATTERN[k];
  for (uint8_t k=0;k<24;++k)       txCells[i++] = payload[k];
  // GUARD "11"
  txCells[i++] = 1;
  txCells[i++] = 1;

  txLen = i;
  cellIdx = 0;
  respSampleIdx = 0;
  inRespWindow = false;

  if (mtype == POLL_ID) currentlyPollingId = (A6 & 0x3F);
  else                  currentlyPollingId = 0xFF;
}

void ComProtMaster::update() {
  // schedule next poll if idle
  if (!inRespWindow && (cellIdx >= txLen)) {
    // round-robin POLL
    uint8_t id = scanNextId;
    buildFrameAndKick(POLL_ID, id, 0);
    scanNextId = (scanNextId + 1) & 0x3F;
  }

  // consume presence event from ISR
  if (presenceEvtPending) {
    noInterrupts();
    uint8_t id   = presenceEvtId;
    bool    yes  = presenceEvtYes;
    presenceEvtPending = false;
    interrupts();

    if (currentlyPollingId != 0xFF && id == currentlyPollingId) {
      handlePresenceResult(id, yes);
    }
  }

  checkTimeouts();
}

// ISR: toggle CLK each half-cell; act on rising edges (one per cell)
void IRAM_ATTR ComProtMaster::onTickISR() {
  if (!self) return;
  static bool clkHigh = false;
  clkHigh = !clkHigh;
  digitalWrite(self->CLK, clkHigh ? HIGH : LOW);
  if (!clkHigh) return; // only on rising

  if (self->cellIdx < self->txLen) {
    uint8_t b = self->txCells[self->cellIdx++];
    if (b) { data_release(self->DATA); } else { data_drive0(self->DATA); }
    if (self->cellIdx >= self->txLen) {
      // finished payload+guard -> release for response window
      data_release(self->DATA);
      self->inRespWindow = true;
      self->respSampleIdx = 0;
    }
  } else if (self->inRespWindow) {
    if (self->respSampleIdx < 2) {
      self->respSample[self->respSampleIdx++] = data_read(self->DATA) ? 1 : 0;
      if (self->respSampleIdx >= 2) {
        self->inRespWindow = false;
        bool present = (self->respSample[0]==0 && self->respSample[1]==0);
        // Defer vector updates to main loop
        self->presenceEvtId = (self->currentlyPollingId == 0xFF) ? 0 : self->currentlyPollingId;
        self->presenceEvtYes = present ? 1 : 0;
        self->presenceEvtPending = true;
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
  } else {
    // nothing; timeout will prune
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
  buildFrameAndKick(CMD_TO_TYPE, slaveType & 0x3F, cmd & 0x0F);
  return true;
}

bool ComProtMaster::sendCommandToSlaveId(uint8_t slaveId, uint8_t cmd) {
  buildFrameAndKick(CMD_TO_ID, slaveId & 0x3F, cmd & 0x0F);
  return true;
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
  pinMode(DATA, INPUT); // release
  pinMode(CLK, INPUT);
  rxState = RX_WAIT_SYNC;
  syncWindow = 0;
  recvIdx = 0;
  postCount = 0;
  expectReplyWindow = false;
  mustReplyYes = false;

  self = this;
  attachInterrupt(digitalPinToInterrupt(CLK), onClkRiseISR, RISING);
}

void ComProtSlave::update() {
  // no periodic work; replies handled inside ISR in RESP window
}

void ComProtSlave::setCommandHandler(uint8_t cmdType, CommandHandler h) {
  if (cmdType < 16) handlers[cmdType] = h;
}

void ComProtSlave::removeCommandHandler(uint8_t cmdType) {
  if (cmdType < 16) handlers[cmdType] = nullptr;
}

bool ComProtSlave::matchSYNC(uint16_t w) {
  for (int i=0;i<15;++i) {
    uint8_t bit = (w >> (14 - i)) & 1;
    if (bit != SYNC_PATTERN[i]) return false;
  }
  return true;
}

uint8_t ComProtSlave::decode12_from_24cells(const volatile uint8_t *cells24, uint16_t &bits12_out) {
  uint16_t v = 0;
  for (int i=0; i<24; i+=2) {
    uint8_t a = cells24[i];
    uint8_t b = cells24[i+1];
    if (a!=b) return 1; // invalid symbol
    v = (uint16_t)((v << 1) | (a ? 1 : 0));
  }
  bits12_out = v;
  return 0;
}

void ComProtSlave::handleDecoded(uint8_t mtype, uint8_t A6, uint8_t cmd4) {
  last_mtype = mtype; last_A6 = A6; last_cmd4 = cmd4;

  // Presence (POLL_ID)
  if (mtype == POLL_ID && A6 == (slaveId & 0x3F)) {
    expectReplyWindow = true;
    mustReplyYes = true;
    return;
  }

  // Commands
  if (mtype == CMD_TO_TYPE && A6 == (slaveType & 0x3F)) {
    if (handlers[cmd4]) handlers[cmd4](cmd4, masterId);
  } else if (mtype == CMD_TO_ID && A6 == (slaveId & 0x3F)) {
    if (handlers[cmd4]) handlers[cmd4](cmd4, masterId);
  }
}

void IRAM_ATTR ComProtSlave::onClkRiseISR() {
  if (!self) return;

  // sample DATA and update SYNC window (keep 15 bits)
  int d = data_read(self->DATA) ? 1 : 0;
  self->syncWindow = ((self->syncWindow << 1) | (d & 1)) & 0x7FFF;

  switch (self->rxState) {
    case RX_WAIT_SYNC:
      if (matchSYNC(self->syncWindow)) {
        self->rxState = RX_PAYLOAD;
        self->recvIdx = 0;
        self->postCount = 0;
      }
      break;

    case RX_PAYLOAD:
      if (self->recvIdx < 24) {
        self->recvCells[self->recvIdx++] = (uint8_t)d;
        if (self->recvIdx >= 24) {
          uint16_t bits12 = 0;
          if (!decode12_from_24cells(self->recvCells, bits12)) {
            uint8_t mtype = (bits12 >> 10) & 0x03;
            uint8_t A6    = (bits12 >>  4) & 0x3F;
            uint8_t cmd4  =  bits12        & 0x0F;
            self->handleDecoded(mtype, A6, cmd4);
            self->rxState = RX_POST_GUARD;
            self->postCount = 0;
          } else {
            // invalid symbol => resync
            self->rxState = RX_WAIT_SYNC;
          }
        }
      }
      break;

    case RX_POST_GUARD:
      // 2 guard cells "11"
      self->postCount++;
      if (self->postCount >= 2) {
        // begin reply window (2 cells)
        if (self->expectReplyWindow && self->mustReplyYes) {
          data_drive0(self->DATA); // YES="00" -> hold low for both cells
        } else {
          data_release(self->DATA); // NO="11"
        }
        self->rxState = RX_RESP_WINDOW;
        self->postCount = 0;
      }
      break;

    case RX_RESP_WINDOW:
      self->postCount++;
      if (self->postCount >= 2) {
        // end of reply window
        data_release(self->DATA);
        self->expectReplyWindow = false;
        self->mustReplyYes = false;
        self->postCount = 0;
        self->rxState = RX_WAIT_SYNC;
      }
      break;
  }
}
