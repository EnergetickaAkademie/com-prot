#ifndef COM_PROT_H
#define COM_PROT_H

#include <Arduino.h>
#include <vector>
#include <functional>

#define PJON_INCLUDE_SWBB
#include <PJONSoftwareBitBang.h>

// Protocol message types
#define COM_PROT_HEARTBEAT 0x03
#define COM_PROT_COMMAND   0x04

// Slave information structure
struct SlaveInfo {
    uint8_t id;
    uint8_t type;
    unsigned long lastHeartbeat;
    
    SlaveInfo(uint8_t _id, uint8_t _type) : id(_id), type(_type), lastHeartbeat(millis()) {}
};

// Command handler function type
typedef std::function<void(uint8_t* payload, uint16_t length, uint8_t senderId)> CommandHandler;

// Debug receive function type - called on every received message
typedef std::function<void(uint8_t* payload, uint16_t length, uint8_t senderId, uint8_t messageType)> DebugReceiveHandler;

// Master class for managing slave communication
class ComProtMaster {
private:
    PJON<SoftwareBitBang>* bus;
    std::vector<SlaveInfo> slaves;
    uint8_t masterId;
    uint8_t pin;
    unsigned long heartbeatTimeout;
    DebugReceiveHandler debugHandler;
    
    // Internal methods
    std::vector<SlaveInfo>::iterator findSlave(uint8_t id);
    static void staticReceiver(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info);
    void handleMessage(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info);
    
    // Static instance for callback
    static ComProtMaster* instance;

public:
    ComProtMaster(uint8_t masterId, uint8_t pin, unsigned long heartbeatTimeout = 3100);
    ~ComProtMaster();
    
    // Initialization
    void begin();
    void update();
    
    // Slave management
    void addOrUpdateSlave(uint8_t id, uint8_t type);
    void checkSlaveTimeouts();
    std::vector<SlaveInfo> getConnectedSlaves();
    std::vector<SlaveInfo> getSlavesByType(uint8_t type);
    bool isSlaveConnected(uint8_t id);
    
    // Command sending
    bool sendCommandToSlaveType(uint8_t slaveType, uint8_t command, uint8_t* data = nullptr, uint16_t dataLen = 0);
    bool sendCommandToSlaveId(uint8_t slaveId, uint8_t command, uint8_t* data = nullptr, uint16_t dataLen = 0);
    
    // Configuration
    void setHeartbeatTimeout(unsigned long timeout);
    uint8_t getMasterId() const;
    size_t getSlaveCount() const;
    
    // Debug functionality
    void setDebugReceiveHandler(DebugReceiveHandler handler);
    void removeDebugReceiveHandler();
};

// Slave class for responding to master commands
class ComProtSlave {
private:
    PJON<SoftwareBitBang>* bus;
    uint8_t slaveId;
    uint8_t slaveType;
    uint8_t pin;
    uint8_t masterId;
    unsigned long heartbeatInterval;
    unsigned long lastHeartbeat;
    DebugReceiveHandler debugHandler;
    
    // Command handlers
    std::vector<std::pair<uint8_t, CommandHandler>> commandHandlers;
    
    // Internal methods
    static void staticReceiver(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info);
    void handleMessage(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info);
    
    // Static instance for callback
    static ComProtSlave* instance;

public:
    ComProtSlave(uint8_t slaveId, uint8_t slaveType, uint8_t pin, uint8_t masterId = 1, unsigned long heartbeatInterval = 1000);
    ~ComProtSlave();
    
    // Initialization
    void begin();
    void update();
    
    // Heartbeat management
    void sendHeartbeat();
    void setHeartbeatInterval(unsigned long interval);
    
    // Command handling
    void setCommandHandler(uint8_t commandType, CommandHandler handler);
    void removeCommandHandler(uint8_t commandType);
    
    // Communication
    bool sendResponse(uint8_t commandType, uint8_t* data = nullptr, uint16_t dataLen = 0);
    
    // Configuration
    uint8_t getSlaveId() const;
    uint8_t getSlaveType() const;
    uint8_t getMasterId() const;
    
    // Debug functionality
    void setDebugReceiveHandler(DebugReceiveHandler handler);
    void removeDebugReceiveHandler();
};

#endif // COM_PROT_H
