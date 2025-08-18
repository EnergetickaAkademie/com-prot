#include "com-prot.h"

// Static instance pointers for callbacks
ComProtMaster* ComProtMaster::instance = nullptr;
ComProtSlave* ComProtSlave::instance = nullptr;

// Protocol framing
// 0xAA 0x55 LEN BODY...
// BODY: SRC DST TYPE PAYLOAD... CRC16(2 bytes, big-endian)
static constexpr uint8_t PRE1 = 0xAA;
static constexpr uint8_t PRE2 = 0x55;

// ============================================================================
// ComProtBase Implementation (UART, CRC, parser)
// ============================================================================

ComProtBase::ComProtBase() : debugHandler(nullptr),
    lastReceiveTime(0), totalReceiveCalls(0), sumIntervals(0), maxInterval(0) {}

ComProtBase::~ComProtBase() {}

void ComProtBase::begin() {
    Serial.begin(COMPROT_BAUD);
    randomSeed(micros());
}

// CRC16-CCITT (0x1021, init 0xFFFF, no xorout)
uint16_t ComProtBase::crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; ++b) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

size_t ComProtBase::buildFrame(uint8_t* out, uint8_t dstId, uint8_t srcId, uint8_t msgType,
                      const uint8_t* payload, uint8_t payloadLen) const {
    // BODY = SRC DST TYPE PAY ... CRC_H CRC_L
    uint8_t bodyLen = 3 + payloadLen + 2;
    uint8_t idx = 0;
    out[idx++] = PRE1;
    out[idx++] = PRE2;
    out[idx++] = bodyLen;
    out[idx++] = srcId;
    out[idx++] = dstId;
    out[idx++] = msgType;
    for (uint8_t i = 0; i < payloadLen; ++i) out[idx++] = payload[i];
    // compute CRC over SRC..payload
    uint16_t crc = crc16_ccitt(&out[3], 3 + payloadLen);
    out[idx++] = (crc >> 8) & 0xFF;
    out[idx++] = crc & 0xFF;
    return idx;
}

void ComProtBase::writeRaw(const uint8_t* data, size_t len) const {
    Serial.write(data, len);
    Serial.flush();
}

void ComProtBase::receive(unsigned long time) {
    unsigned long start = micros();
    do {
        // stats timing
        unsigned long now = micros();
        if (lastReceiveTime > 0) {
            unsigned long dt = now - lastReceiveTime;
            sumIntervals += dt;
            totalReceiveCalls++;
            if (dt > maxInterval) maxInterval = dt;
        }
        lastReceiveTime = now;

        while (Serial.available()) {
            uint8_t byteIn = (uint8_t)Serial.read();
            switch (rxState) {
                case WAIT_PREAMBLE1:
                    if (byteIn == PRE1) rxState = WAIT_PREAMBLE2;
                    break;
                case WAIT_PREAMBLE2:
                    if (byteIn == PRE2) rxState = READ_LEN; else rxState = WAIT_PREAMBLE1;
                    break;
                case READ_LEN:
                    rxLen = byteIn;
                    if (rxLen == 0 || rxLen > MAX_FRAME_SIZE) { rxState = WAIT_PREAMBLE1; break; }
                    rxIndex = 0;
                    rxState = READ_BODY;
                    break;
                case READ_BODY:
                    rxBuf[rxIndex++] = byteIn;
                    if (rxIndex >= rxLen) {
                        // verify CRC
                        if (rxLen < 5) { // minimum SRC DST TYPE CRC(2)
                            rxState = WAIT_PREAMBLE1;
                            break;
                        }
                        uint16_t recvCrc = ((uint16_t)rxBuf[rxLen-2] << 8) | rxBuf[rxLen-1];
                        uint16_t calcCrc = crc16_ccitt(rxBuf, rxLen-2);
                        if (recvCrc == calcCrc) {
                            dispatchFrame(rxBuf, rxLen);
                        } else {
                            onCrcError();
                        }
                        rxState = WAIT_PREAMBLE1;
                    }
                    break;
            }
        }

    // 5s stats print (optional)
#if COMPROT_ENABLE_STATS
    static unsigned long lastPrint = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastPrint >= 5000) {
        calculateAndPrintStats();
        lastPrint = nowMs;
    }
#endif
    } while (time > 0 && (micros() - start) < time);
}

void ComProtBase::dispatchFrame(const uint8_t* body, uint8_t bodyLen) {
    if (bodyLen < 5) return;
    uint8_t src = body[0];
    uint8_t dst = body[1];
    uint8_t type = body[2];
    const uint8_t* payload = &body[3];
    uint16_t payloadLen = bodyLen - 5; // exclude SRC,DST,TYPE and CRC(2)

    if (debugHandler) {
        debugHandler((uint8_t*)payload, payloadLen, src, type);
    }

    // deliver only if for us or broadcast (dst 0)
    if (dst == 0 || dst == getSelfId()) {
        handleMessage(type, (uint8_t*)payload, payloadLen, src, dst);
    }
}

void ComProtBase::sendFrame(uint8_t dstId, uint8_t msgType, const uint8_t* payload, uint8_t payloadLen) {
    uint8_t frame[140];
    uint8_t src = getSelfId();
    uint8_t bodyTmp[128];
    // Build in a single buffer using helper, which writes preamble too
    size_t len = buildFrame(frame, dstId, src, msgType, payload, payloadLen);
    writeRaw(frame, len);
}

void ComProtBase::calculateAndPrintStats() {
#if COMPROT_ENABLE_STATS
    if (totalReceiveCalls == 0) return;
    unsigned long average = sumIntervals / totalReceiveCalls;
    Serial.print("Receive Stats - Count: ");
    Serial.print(totalReceiveCalls);
    Serial.print(", Avg: ");
    Serial.print(average);
    Serial.print("μs, Max: ");
    Serial.print(maxInterval);
    Serial.println("μs");
    totalReceiveCalls = 0;
    sumIntervals = 0;
    maxInterval = 0;
#endif
}

void ComProtBase::setDebugReceiveHandler(DebugReceiveHandler handler) { debugHandler = handler; }
void ComProtBase::removeDebugReceiveHandler() { debugHandler = nullptr; }

// ============================================================================
// ComProtMaster Implementation (UART protocol)
// ============================================================================

ComProtMaster::ComProtMaster(uint8_t masterId, unsigned long heartbeatTimeout)
    : ComProtBase(), masterId(masterId), heartbeatTimeout(heartbeatTimeout) {
    slaves.reserve(20);
    instance = this;
}

ComProtMaster::~ComProtMaster() { instance = nullptr; }

void ComProtMaster::begin() { ComProtBase::begin(); }

void ComProtMaster::update() {
    // Poll UART quickly
    receive(1000); // 1ms window
    // Check timeouts each 500ms
    static unsigned long lastTimeoutCheck = 0;
    unsigned long now = millis();
    if (now - lastTimeoutCheck > 500) { checkSlaveTimeouts(); lastTimeoutCheck = now; }
}

void ComProtMaster::onCrcError() {
    // broadcast retry signal so slaves random-backoff and retry
    sendFrame(0, COM_PROT_RETRY, nullptr, 0);
}

void ComProtMaster::handleMessage(uint8_t messageType, uint8_t* payload, uint16_t length,
                                  uint8_t srcId, uint8_t dstId) {
    switch (messageType) {
        case COM_PROT_HEARTBEAT:
            if (length >= 1) {
                uint8_t slaveType = payload[0];
                addOrUpdateSlave(srcId, slaveType);
            }
            break;
        case COM_PROT_COMMAND:
            // master could process responses here if needed
            break;
        default:
            break;
    }
}

std::vector<SlaveInfo>::iterator ComProtMaster::findSlave(uint8_t id) {
    for (auto it = slaves.begin(); it != slaves.end(); ++it) if (it->id == id) return it;
    return slaves.end();
}

void ComProtMaster::addOrUpdateSlave(uint8_t id, uint8_t type) {
    auto it = findSlave(id);
    if (it != slaves.end()) { it->lastHeartbeat = millis(); it->type = type; }
    else { slaves.emplace_back(id, type); }
}

void ComProtMaster::checkSlaveTimeouts() {
    unsigned long now = millis();
    for (auto it = slaves.begin(); it != slaves.end();) {
        if (now - it->lastHeartbeat > heartbeatTimeout) it = slaves.erase(it);
        else ++it;
    }
}

std::vector<SlaveInfo> ComProtMaster::getConnectedSlaves() { return slaves; }

std::vector<SlaveInfo> ComProtMaster::getSlavesByType(uint8_t type) {
    std::vector<SlaveInfo> res; for (const auto& s : slaves) if (s.type == type) res.push_back(s); return res;
}

bool ComProtMaster::isSlaveConnected(uint8_t id) { return findSlave(id) != slaves.end(); }

bool ComProtMaster::sendCommandToSlaveType(uint8_t slaveType, uint8_t command,
                                           uint8_t* data, uint16_t dataLen) {
    // Broadcast command to type; payload = [type, command, data...]
    uint8_t buf[64];
    if (dataLen + 2 > sizeof(buf)) return false;
    buf[0] = slaveType;
    buf[1] = command;
    if (data && dataLen) memcpy(&buf[2], data, dataLen);
    sendFrame(0 /*broadcast*/, COM_PROT_COMMAND, buf, (uint8_t)(dataLen + 2));
    return true;
}

bool ComProtMaster::sendCommandToSlaveId(uint8_t slaveId, uint8_t command,
                                         uint8_t* data, uint16_t dataLen) {
    uint8_t buf[64];
    if (dataLen + 1 > sizeof(buf)) return false;
    buf[0] = command;
    if (data && dataLen) memcpy(&buf[1], data, dataLen);
    sendFrame(slaveId, COM_PROT_COMMAND, buf, (uint8_t)(dataLen + 1));
    return true;
}

void ComProtMaster::setHeartbeatTimeout(unsigned long timeout) { heartbeatTimeout = timeout; }
uint8_t ComProtMaster::getMasterId() const { return masterId; }
size_t ComProtMaster::getSlaveCount() const { return slaves.size(); }

// ============================================================================
// ComProtSlave Implementation
// ============================================================================

ComProtSlave::ComProtSlave(uint8_t slaveId, uint8_t slaveType, uint8_t masterId, unsigned long heartbeatInterval)
    : ComProtBase(), slaveId(slaveId), slaveType(slaveType), masterId(masterId), heartbeatInterval(heartbeatInterval), lastHeartbeat(0) { instance = this; }

ComProtSlave::~ComProtSlave() {
    instance = nullptr;
}

void ComProtSlave::begin() { ComProtBase::begin(); }

void ComProtSlave::update() {
    // Poll UART non-blocking window
    receive(1000);
    // If a resend is scheduled due to retry, send after time passed
    if (resendScheduled && micros() >= resendAtMicros) {
        writeRaw(lastFrame, lastFrameLen);
        resendScheduled = false;
        lastFrameLen = 0;
    }
    // Heartbeat
    unsigned long now = millis();
    if (now - lastHeartbeat > heartbeatInterval) { sendHeartbeat(); lastHeartbeat = now; }
}

void ComProtSlave::handleMessage(uint8_t messageType, uint8_t* payload, uint16_t length,
                                 uint8_t srcId, uint8_t dstId) {
    if (messageType == COM_PROT_RETRY) {
        // schedule resend with random backoff 0..10ms
        if (lastFrameLen > 0 && !resendScheduled) {
            uint16_t backoff = (uint16_t)random(0, 10000); // microseconds
            resendAtMicros = micros() + backoff;
            resendScheduled = true;
        }
        return;
    }
    if (messageType == COM_PROT_COMMAND) {
        if (length == 0) return;
        // Unicast: payload[0]=command, rest=data
        // Broadcast-to-type: payload[0]=type, payload[1]=command, rest=data
        if (dstId == slaveId) {
            uint8_t cmd = payload[0];
            uint8_t* data = (length > 1) ? &payload[1] : nullptr;
            uint16_t dataLen = (length > 1) ? length - 1 : 0;
            for (const auto& h : commandHandlers) if (h.first == cmd) { h.second(data, dataLen, srcId); break; }
        } else if (dstId == 0 && length >= 2 && payload[0] == slaveType) {
            uint8_t cmd = payload[1];
            uint8_t* data = (length > 2) ? &payload[2] : nullptr;
            uint16_t dataLen = (length > 2) ? length - 2 : 0;
            for (const auto& h : commandHandlers) if (h.first == cmd) { h.second(data, dataLen, srcId); break; }
        }
    }
}

void ComProtSlave::sendHeartbeat() {
    uint8_t hb[1] = { slaveType };
    uint8_t frame[140];
    size_t flen = buildFrame(frame, masterId, slaveId, COM_PROT_HEARTBEAT, hb, 1);
    writeRaw(frame, flen);
    if (flen <= MAX_RAW_FRAME) { memcpy(lastFrame, frame, flen); lastFrameLen = flen; }
}

void ComProtSlave::setHeartbeatInterval(unsigned long interval) {
    heartbeatInterval = interval;
}

void ComProtSlave::setCommandHandler(uint8_t commandType, CommandHandler handler) {
    // Remove existing handler for this command type
    removeCommandHandler(commandType);
    
    // Add new handler
    commandHandlers.emplace_back(commandType, handler);
}

void ComProtSlave::removeCommandHandler(uint8_t commandType) {
    commandHandlers.erase(
        std::remove_if(commandHandlers.begin(), commandHandlers.end(),
            [commandType](const std::pair<uint8_t, CommandHandler>& handler) {
                return handler.first == commandType;
            }),
        commandHandlers.end()
    );
}

bool ComProtSlave::sendResponse(uint8_t commandType, uint8_t* data, uint16_t dataLen) {
    // Build payload: [commandType, data...]
    uint8_t buf[64];
    if (dataLen + 1 > sizeof(buf)) return false;
    buf[0] = commandType;
    if (data && dataLen) memcpy(&buf[1], data, dataLen);
    // Build full frame to allow retry resend
    uint8_t frame[140];
    size_t flen = buildFrame(frame, masterId, slaveId, COM_PROT_COMMAND, buf, (uint8_t)(dataLen + 1));
    writeRaw(frame, flen);
    // keep copy for possible retry
    if (flen <= MAX_RAW_FRAME) { memcpy(lastFrame, frame, flen); lastFrameLen = flen; }
    return true;
}

uint8_t ComProtSlave::getSlaveId() const {
    return slaveId;
}

uint8_t ComProtSlave::getSlaveType() const {
    return slaveType;
}

uint8_t ComProtSlave::getMasterId() const {
    return masterId;
}
