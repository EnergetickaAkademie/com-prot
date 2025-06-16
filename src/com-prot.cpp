#include "com-prot.h"

// Static instance pointers for callbacks
ComProtMaster* ComProtMaster::instance = nullptr;
ComProtSlave* ComProtSlave::instance = nullptr;

// ============================================================================
// ComProtMaster Implementation
// ============================================================================

ComProtMaster::ComProtMaster(uint8_t masterId, uint8_t pin, unsigned long heartbeatTimeout) 
    : masterId(masterId), pin(pin), heartbeatTimeout(heartbeatTimeout) {
    bus = new PJON<SoftwareBitBang>(masterId);
    slaves.reserve(20); // Reserve space for performance
    instance = this;
}

ComProtMaster::~ComProtMaster() {
    delete bus;
    instance = nullptr;
}

void ComProtMaster::begin() {
    bus->strategy.set_pin(pin);
    bus->set_receiver(staticReceiver);
    bus->set_acknowledge(false);
    bus->set_crc_32(true);
    bus->set_packet_auto_deletion(true);
    bus->begin();
}

void ComProtMaster::update() {
    bus->update();
    bus->receive();
    checkSlaveTimeouts();
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

bool ComProtMaster::sendCommandToSlaveType(uint8_t slaveType, uint8_t command, uint8_t* data, uint16_t dataLen) {
    // Check if we have any slaves of this type
    bool hasSlaves = false;
    for (const auto& slave : slaves) {
        if (slave.type == slaveType) {
            hasSlaves = true;
            break;
        }
    }
    
    if (!hasSlaves) {
        return false;
    }
    
    // Prepare broadcast message: messageType + slaveType + command + data
    uint8_t messageLen = 3 + dataLen;
    uint8_t* message = new uint8_t[messageLen];
    
    message[0] = COM_PROT_COMMAND;
    message[1] = slaveType; // Target slave type for broadcast
    message[2] = command;
    
    if (data && dataLen > 0) {
        memcpy(&message[3], data, dataLen);
    }
    
    // Use PJON broadcast to send to all devices
    uint16_t result = bus->send_packet(PJON_BROADCAST, message, messageLen);
    
    delete[] message;
    
    return result == PJON_ACK;
}

bool ComProtMaster::sendCommandToSlaveId(uint8_t slaveId, uint8_t command, uint8_t* data, uint16_t dataLen) {
    if (!isSlaveConnected(slaveId)) {
        return false;
    }
    
    // Prepare unicast message: messageType + 0 (no slave type filter) + command + data
    uint8_t messageLen = 3 + dataLen;
    uint8_t* message = new uint8_t[messageLen];
    
    message[0] = COM_PROT_COMMAND;
    message[1] = 0; // 0 means unicast (no type filtering)
    message[2] = command;
    
    if (data && dataLen > 0) {
        memcpy(&message[3], data, dataLen);
    }
    
    uint16_t result = bus->send(slaveId, message, messageLen);
    
    delete[] message;
    
    return result == PJON_ACK;
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
// Debug Handler Implementations for Master
// ============================================================================

void ComProtMaster::setDebugReceiveHandler(DebugReceiveHandler handler) {
    debugHandler = handler;
}

void ComProtMaster::removeDebugReceiveHandler() {
    debugHandler = nullptr;
}

// ============================================================================
// ComProtSlave Implementation
// ============================================================================

ComProtSlave::ComProtSlave(uint8_t slaveId, uint8_t slaveType, uint8_t pin, uint8_t masterId, unsigned long heartbeatInterval)
    : slaveId(slaveId), slaveType(slaveType), pin(pin), masterId(masterId), heartbeatInterval(heartbeatInterval), lastHeartbeat(0) {
    bus = new PJON<SoftwareBitBang>(slaveId);
    instance = this;
}

ComProtSlave::~ComProtSlave() {
    delete bus;
    instance = nullptr;
}

void ComProtSlave::begin() {
    bus->strategy.set_pin(pin);
    bus->set_receiver(staticReceiver);
    bus->set_acknowledge(false);
    bus->set_crc_32(true);
    bus->set_packet_auto_deletion(true);
    bus->begin();
}

void ComProtSlave::update() {
    bus->update();
    bus->receive();
    
    // Send heartbeat if interval has passed
    if (millis() - lastHeartbeat > heartbeatInterval) {
        sendHeartbeat();
        lastHeartbeat = millis();
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
    
    uint16_t result = bus->send(masterId, message, messageLen);
    
    delete[] message;
    
    return result == PJON_ACK;
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

// ============================================================================
// Debug Handler Implementations for Slave  
// ============================================================================

void ComProtSlave::setDebugReceiveHandler(DebugReceiveHandler handler) {
    debugHandler = handler;
}

void ComProtSlave::removeDebugReceiveHandler() {
    debugHandler = nullptr;
}
