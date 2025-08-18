#include "com-prot.h"
#include <Ticker.h>
using namespace StarWire;

// --------------------- Master ---------------------
static Ticker _starwire_ticker_master; // single instance guard
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
  _starwire_ticker_master.attach_us(cellUs, onTickISR);
}

void ComProtMaster::stopTicker() {
  if (!ticking) return;
  _starwire_ticker_master.detach();
  ticking = false;
}

// Build: SYNC(15) + payload(24 cells) + GUARD(2x '1') ; resp read handled in ISR
void ComProtMaster::buildFrameAndKick(uint8_t mtype, uint8_t A6, uint8_t cmd4) {
  uint16_t bits12 = pack12(mtype, A6, cmd4);
  uint8_t payload[24];
  encode12_to_24cells(bits12, payload);

  uint16_t i=0;
  for (uint8_t k=0;k<SYNC_LEN;++k) txCells[i++] = SYNC_PATTERN[k];
  for (uint8_t k=0;k<24;++k)       txCells[i++] = payload[k];
  // GUARD (idle 2 cells)
  txCells[i++] = 1;
  txCells[i++] = 1;

  txLen = i;
  cellIdx = 0;
  respSampleIdx = 0;
  inRespWindow = false;
}

// Non-blocking: schedule one POLL per update + do timeouts
void ComProtMaster::update() {
  // Schedule next poll if nothing currently streaming (guarded by checking cellIdx>=txLen and not in resp)
  static bool pendingKick = false;
  static uint8_t pending_mtype=0, pending_A6=0, pending_cmd4=0;
  if (!inRespWindow && (cellIdx >= txLen)) {
    // Prefer queued commands? (simple round-robin: scan IDs)
    pending_mtype = POLL_ID;
    pending_A6 = scanNextId;
    pending_cmd4 = 0;

    buildFrameAndKick(pending_mtype, pending_A6, pending_cmd4);

    // advance for next time
    scanNextId = (scanNextId + 1) & 0x3F;
  }

  checkTimeouts();
}

// ISR: one cell per tick
void IRAM_ATTR ComProtMaster::onTickISR() {
  if (!self) return;
  // Toggle clock rising edge: we keep clock HIGH during cell center, LOW otherwise (simple)
  static bool clk = false;
  clk = !clk;
  digitalWrite(self->CLK, clk ? HIGH : LOW);

  if (!clk) return; // operate on rising edge only

  if (self->cellIdx < self->txLen) {
    // send this cell
    uint8_t b = self->txCells[self->cellIdx++];
    if (b) { data_release(self->DATA); } else { data_drive0(self->DATA); }
    if (self->cellIdx >= self->txLen) {
      // finished payload+guard -> release line for response
      data_release(self->DATA);
      self->inRespWindow = true;
      self->respSampleIdx = 0;
    }
  } else if (self->inRespWindow) {
    // read two cells response
    if (self->respSampleIdx < 2) {
      self->respSample[self->respSampleIdx++] = data_read(self->DATA) ? 1 : 0;
      if (self->respSampleIdx >= 2) {
        self->inRespWindow = false;
        // decode presence: YES="00" => present
        bool present = (self->respSample[0]==0 && self->respSample[1]==0);
        // We know what we just polled: it's the last scheduled POLL_ID
        // Simplify: last polled id is (cellIdx==txLen) and we used scanNextId-1
        uint8_t polledId = (self->scanNextId + 63) & 0x3F;
        self->handlePresenceResult(polledId, present);
      }
    }
  }
}

void ComProtMaster::handlePresenceResult(uint8_t polledId, bool present) {
  if (present) {
    auto it = findSlave(polledId);
    if (it == slaves.end()) {
      // Type unknown yet; we will learn it on first command or provide a separate "who are you" command if needed.
      slaves.push_back({polledId, 0xFF, millis()});
    } else {
      it->lastSeenMs = millis();
    }
  } else {
    // do nothing; timeout will remove
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
  std::vector<SlaveInfo> r; for (auto &s: slaves) if (s.type==type) r.push_back(s); return r;
}

bool ComProtMaster::isSlaveConnected(uint8_t id) { return findSlave(id) != slaves.end(); }

bool ComProtMaster::sendCommandToSlaveType(uint8_t slaveType, uint8_t cmd) {
  // Fire-and-forget broadcast by type (no response window use)
  buildFrameAndKick(CMD_TO_TYPE, slaveType & 0x3F, cmd & 0x0F);
  return true;
}

bool ComProtMaster::sendCommandToSlaveId(uint8_t slaveId, uint8_t cmd) {
  buildFrameAndKick(CMD_TO_ID, slaveId & 0x3F, cmd & 0x0F);
  return true;
}

// --------------------- Slave ---------------------
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
  self = this;
  attachInterrupt(digitalPinToInterrupt(CLK), onClkRiseISR, RISING);
}

void ComProtSlave::update() {
  // nothing periodic (reply is driven inside ISR only in the 2-cell window)
}

void ComProtSlave::setCommandHandler(uint8_t cmdType, CommandHandler h) {
  if (cmdType < 16) handlers[cmdType] = h;
}

void ComProtSlave::removeCommandHandler(uint8_t cmdType) {
  if (cmdType < 16) handlers[cmdType] = nullptr;
}

bool ComProtSlave::matchSYNC(uint16_t w) {
  // compare last 15 bits of w with SYNC_PATTERN
  for (int i=0;i<15;++i) {
    uint8_t bit = (w >> (14 - i)) & 1;
    if (bit != SYNC_PATTERN[i]) return false;
  }
  return true;
}

uint8_t ComProtSlave::decode12_from_24cells(const uint8_t *cells24, uint16_t &bits12_out) {
  uint16_t v = 0;
  for (int i=0, j=0; i<24; i+=2, ++j) {
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

  // Presence reply on POLL_ID match
  if (mtype == POLL_ID && A6 == (slaveId & 0x3F)) {
    expectReplyWindow = true;
    mustReplyYes = true;
    return;
  }

  if (mtype == CMD_TO_TYPE && A6 == (slaveType & 0x3F)) {
    if (handlers[cmd4]) handlers[cmd4](cmd4, masterId);
  } else if (mtype == CMD_TO_ID && A6 == (slaveId & 0x3F)) {
    if (handlers[cmd4]) handlers[cmd4](cmd4, masterId);
  }
}

void IRAM_ATTR ComProtSlave::onClkRiseISR() {
  if (!self) return;

  // sample DATA
  int d = data_read(self->DATA) ? 1 : 0;

  // shift into a 15-bit window
  self->syncWindow = ((self->syncWindow << 1) | (d & 1)) & 0x7FFF;

  if (!self->collecting) {
    // look for SYNC
    if (matchSYNC(self->syncWindow)) {
      self->collecting = 1;
      self->recvIdx = 0;
    }
    return;
  }

  // collecting payload 24 cells
  if (self->recvIdx < 24) {
    self->recvCells[self->recvIdx++] = (uint8_t)d;
    if (self->recvIdx >= 24) {
      // decode 12 bits
      uint16_t bits12 = 0;
      if (!decode12_from_24cells(self->recvCells, bits12)) {
        uint8_t mtype = (bits12 >> 10) & 0x03;
        uint8_t A6    = (bits12 >>  4) & 0x3F;
        uint8_t cmd4  =  bits12        & 0x0F;
        self->handleDecoded(mtype, A6, cmd4);
        if (self->collecting) self->collecting = 0;
      } else {
        self->collecting = 0; // invalid -> resync
      }
    }
    return;
  }

  // After payload there is a 2-cell guard "11" then a 2-cell reply window.
  // We simply count two more rising edges (guard), then if expectReplyWindow drive reply for next 2 cells.
  static uint8_t postCount = 0;
  postCount++;
  if (postCount == 2) {
    // start of reply window
    if (self->expectReplyWindow && self->mustReplyYes) {
      // YES = "00": pull low for next 2 cells
      data_drive0(self->DATA);
    } else {
      // NO = "11": do nothing (release)
      data_release(self->DATA);
    }
  } else if (postCount == 4) {
    // end of reply window
    data_release(self->DATA);
    self->expectReplyWindow = false;
    self->mustReplyYes = false;
    postCount = 0;
  }
}
