#include "com-prot.h"

// Static instance pointers for callbacks
ComProtMaster* ComProtMaster::instance = nullptr;
ComProtSlave* ComProtSlave::instance = nullptr;

// ============================================================================
// ComProtBase Implementation
// ============================================================================

ComProtBase::ComProtBase() : bus(nullptr), debugHandler(nullptr), 
    lastReceiveTime(0), totalReceiveCalls(0), sumIntervals(0), maxInterval(0) {
}

ComProtBase::~ComProtBase() {
    if (bus) {
        delete bus;
        bus = nullptr;
    }
}

void ComProtBase::initializeBus(uint8_t deviceId) {
    bus = new PJON<ThroughSerialAsync>(deviceId);
}

void ComProtBase::begin() {
    if (bus) {
        Serial.begin(9600);  // Initialize UART at 9600 baud
        bus->strategy.set_serial(&Serial);
        bus->set_acknowledge(false);
        bus->set_crc_32(true);
        bus->set_packet_auto_deletion(true);
        bus->begin();
    }
}

void ComProtBase::receive(unsigned long time) {
    unsigned long currentTime = micros();
    
    // Track timing for statistics
    if (lastReceiveTime > 0) {
        unsigned long interval = currentTime - lastReceiveTime;
        sumIntervals += interval;
        totalReceiveCalls++;
        
        if (interval > maxInterval) {
            maxInterval = interval;
        }
    }
    lastReceiveTime = currentTime;
    
    // Call PJON receive with or without time parameter
    if (time > 0) bus->receive(time);
    else          bus->receive();
    bus->update(); // Ensure bus is updated after receiving
    
}

void ComProtBase::calculateAndPrintStats() {
    if (totalReceiveCalls == 0) {
        return;
    }
    
    // Calculate average
    unsigned long average = sumIntervals / totalReceiveCalls;
    
    // Print statistics
    Serial.print("Receive Stats - Count: ");
    Serial.print(totalReceiveCalls);
    Serial.print(", Avg: ");
    Serial.print(average);
    Serial.print("μs, Max: ");
    Serial.print(maxInterval);
    Serial.println("μs");
    
    // Reset statistics for next interval
    totalReceiveCalls = 0;
    sumIntervals = 0;
    maxInterval = 0;
}

void ComProtBase::setDebugReceiveHandler(DebugReceiveHandler handler) {
    debugHandler = handler;
}

void ComProtBase::removeDebugReceiveHandler() {
    debugHandler = nullptr;
}

// ============================================================================
// ComProtMaster Implementation
// ============================================================================

ComProtMaster::ComProtMaster(uint8_t masterId, unsigned long heartbeatTimeout) 
    : ComProtBase(), masterId(masterId), heartbeatTimeout(heartbeatTimeout) {
    initializeBus(masterId);
    slaves.reserve(20); // Reserve space for performance
    instance = this;
}

ComProtMaster::~ComProtMaster() {
    instance = nullptr;
}

void ComProtMaster::begin() {
    ComProtBase::begin(); // Call base class begin
    if (bus) {
        bus->set_receiver(staticReceiver);
    }
}

void ComProtMaster::update() {
    bus->update();
    
    // Receive with short timeout
    receive(); // 100 microseconds timeout
    
    // Check for slave timeouts periodically (every 500ms)
    static unsigned long lastTimeoutCheck = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastTimeoutCheck > 500) {
        checkSlaveTimeouts();
        lastTimeoutCheck = currentTime;
    }
}

void ComProtMaster::staticReceiver(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
    if (instance) {
        instance->handleMessage(payload, length, packet_info);
    }
}

void ComProtMaster::handleMessage(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
    if (length < 1) return;
    
    uint8_t messageType = payload[0];
    
    // Call debug handler if set
    if (debugHandler) {
        debugHandler(payload, length, packet_info.tx.id, messageType);
    }
    
    switch (messageType) {
        case COM_PROT_HEARTBEAT:
            if (length >= 3) { // messageType + id + type
                uint8_t slaveType = payload[2];
                
                // Use the actual sender ID from packet info
                addOrUpdateSlave(packet_info.tx.id, slaveType);
            }
            break;
            
        default:
            // Handle other message types if needed
            break;
    }
}

std::vector<SlaveInfo>::iterator ComProtMaster::findSlave(uint8_t id) {
    for (auto it = slaves.begin(); it != slaves.end(); ++it) {
        if (it->id == id) {
            return it;
        }
    }
    return slaves.end();
}

void ComProtMaster::addOrUpdateSlave(uint8_t id, uint8_t type) {
    auto it = findSlave(id);
    
    if (it != slaves.end()) {
        // Update existing slave
        it->lastHeartbeat = millis();
        it->type = type;
    } else {
        // Add new slave
        slaves.emplace_back(id, type);
    }
}

void ComProtMaster::checkSlaveTimeouts() {
    unsigned long currentTime = millis();
    
    for (auto it = slaves.begin(); it != slaves.end();) {
        if (currentTime - it->lastHeartbeat > heartbeatTimeout) {
            it = slaves.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<SlaveInfo> ComProtMaster::getConnectedSlaves() {
    return slaves;
}

std::vector<SlaveInfo> ComProtMaster::getSlavesByType(uint8_t type) {
    std::vector<SlaveInfo> result;
    for (const auto& slave : slaves) {
        if (slave.type == type) {
            result.push_back(slave);
        }
    }
    return result;
}

bool ComProtMaster::isSlaveConnected(uint8_t id) {
    return findSlave(id) != slaves.end();
}
bool ComProtMaster::sendCommandToSlaveType(uint8_t slaveType, uint8_t command,
                                           uint8_t* data, uint16_t dataLen) {
  // small stack buffer to avoid heap fragmentation
  uint8_t msg[32];
  uint16_t len = 3 + dataLen;
  if (len > sizeof(msg)) return false;

  msg[0] = COM_PROT_COMMAND;
  msg[1] = slaveType;     // target type for broadcast
  msg[2] = command;
  if (data && dataLen) memcpy(&msg[3], data, dataLen);

  // Try to send with retry
  uint16_t pid = bus->send(PJON_BROADCAST, msg, len);
  if (pid == PJON_FAIL) {
    // buffer full; caller can retry later
    return false;
  }
  return true;
}


bool ComProtMaster::sendCommandToSlaveId(uint8_t slaveId, uint8_t command,
                                         uint8_t* data, uint16_t dataLen) {
  uint8_t msg[32];
  uint16_t len = 3 + dataLen;
  if (len > sizeof(msg)) return false;

  msg[0] = COM_PROT_COMMAND;
  msg[1] = 0;             // 0 ⇒ unicast
  msg[2] = command;
  if (data && dataLen) memcpy(&msg[3], data, dataLen);

  uint16_t pid = bus->send(slaveId, msg, len);
  return pid != PJON_FAIL;
}

void ComProtMaster::setHeartbeatTimeout(unsigned long timeout) {
    heartbeatTimeout = timeout;
}

uint8_t ComProtMaster::getMasterId() const {
    return masterId;
}

size_t ComProtMaster::getSlaveCount() const {
    return slaves.size();
}

// ============================================================================
// ComProtSlave Implementation
// ============================================================================

ComProtSlave::ComProtSlave(uint8_t slaveId, uint8_t slaveType, uint8_t masterId, unsigned long heartbeatInterval)
    : ComProtBase(), slaveId(slaveId), slaveType(slaveType), masterId(masterId), heartbeatInterval(heartbeatInterval), lastHeartbeat(0) {
    initializeBus(slaveId);
    instance = this;
}

ComProtSlave::~ComProtSlave() {
    instance = nullptr;
}

void ComProtSlave::begin() {
    ComProtBase::begin(); // Call base class begin
    if (bus) {
        bus->set_receiver(staticReceiver);
    }
}

void ComProtSlave::update() {
    bus->update();
    
    // Receive with short timeout to avoid blocking
    receive(); // 100 microseconds timeout
    
    // Send heartbeat if interval has passed
    unsigned long currentTime = millis();
    if (currentTime - lastHeartbeat > heartbeatInterval) {
        sendHeartbeat();
        lastHeartbeat = currentTime;
    }
}

void ComProtSlave::staticReceiver(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
    if (instance) {
        instance->handleMessage(payload, length, packet_info);
    }
}

void ComProtSlave::handleMessage(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
    if (length < 1) return;
    
    uint8_t messageType = payload[0];
    
    // Call debug handler if set
    if (debugHandler) {
        debugHandler(payload, length, packet_info.tx.id, messageType);
    }
    
    if (messageType == COM_PROT_COMMAND && length >= 3) {
        uint8_t targetType = payload[1];  // 0 for unicast, specific type for broadcast
        uint8_t command = payload[2];
        
        // Check if this message is for us:
        // - If targetType is 0, it's a unicast message (sent directly to us)
        // - If targetType matches our slaveType, it's a broadcast for our type
        if (targetType == 0 || targetType == slaveType) {
            // Find and execute command handler
            for (const auto& handler : commandHandlers) {
                if (handler.first == command) {
                    // Extract data portion (everything after messageType, targetType, and command)
                    uint8_t* data = (length > 3) ? &payload[3] : nullptr;
                    uint16_t dataLen = (length > 3) ? length - 3 : 0;
                    
                    handler.second(data, dataLen, packet_info.tx.id);
                    break;
                }
            }
        }
    }
}

void ComProtSlave::sendHeartbeat() {
    uint8_t heartbeat[3] = {COM_PROT_HEARTBEAT, slaveId, slaveType};
    bus->send(masterId, heartbeat, sizeof(heartbeat));
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
    // Prepare response message: messageType + command + data
    uint8_t messageLen = 2 + dataLen;
    uint8_t* message = new uint8_t[messageLen];
    
    message[0] = COM_PROT_COMMAND;
    message[1] = commandType;
    
    if (data && dataLen > 0) {
        memcpy(&message[2], data, dataLen);
    }
    
    bus->send(masterId, message, messageLen);
    
    delete[] message;
    
    return true; // Always return true since ACK is disabled
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
