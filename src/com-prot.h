#ifndef COM_PROT_H
#define COM_PROT_H

#include <Arduino.h>
#include <vector>
#include <functional>

// Default UART baud rate for Com-Prot (can be overridden via build flags)
#ifndef COMPROT_BAUD
#define COMPROT_BAUD 9600
#endif

// Enable periodic RX timing stats printing (disabled by default to avoid
// interfering with UART communication and to save memory).
#ifndef COMPROT_ENABLE_STATS
#define COMPROT_ENABLE_STATS 0
#endif
// Protocol message types
#define COM_PROT_HEARTBEAT 0x03
#define COM_PROT_COMMAND   0x04
#define COM_PROT_RETRY     0x05

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

// Abstract base class for common communication functionality
class ComProtBase {
protected:
    DebugReceiveHandler debugHandler;
    
    // Timing statistics
    unsigned long lastReceiveTime;
    unsigned long totalReceiveCalls;
    unsigned long sumIntervals;
    unsigned long maxInterval;
    
    // RX parser state
    enum RxState : uint8_t { WAIT_PREAMBLE1, WAIT_PREAMBLE2, READ_LEN, READ_BODY };
    RxState rxState = WAIT_PREAMBLE1;
    uint8_t rxLen = 0;       // length of body (src+dst+type+payload+crc)
    uint8_t rxIndex = 0;     // index in body buffer
    static const uint8_t MAX_FRAME_SIZE = 120; // total body size cap
    uint8_t rxBuf[MAX_FRAME_SIZE];
    
    // Frame building helper (returns total bytes written to out)
    size_t buildFrame(uint8_t* out, uint8_t dstId, uint8_t srcId, uint8_t msgType,
                      const uint8_t* payload, uint8_t payloadLen) const;
    
    // Write raw frame to UART
    void writeRaw(const uint8_t* data, size_t len) const;
    
    // CRC16-CCITT
    static uint16_t crc16_ccitt(const uint8_t* data, size_t len);
    
    // Dispatch a fully parsed frame
    void dispatchFrame(const uint8_t* body, uint8_t bodyLen);
    
    // Hook for CRC errors (master overrides to send retry)
    virtual void onCrcError() {}
    
    // Pure virtual methods to be implemented by derived classes
    virtual void handleMessage(uint8_t messageType, uint8_t* payload, uint16_t length,
                               uint8_t srcId, uint8_t dstId) = 0;
    virtual uint8_t getSelfId() const = 0;
    
    // Statistics calculation
    void calculateAndPrintStats();

public:
    ComProtBase();
    virtual ~ComProtBase();
    
    // Common methods
    void begin();
    void receive(unsigned long time = 0);
    
    // Frame sending helper
    void sendFrame(uint8_t dstId, uint8_t msgType, const uint8_t* payload = nullptr, uint8_t payloadLen = 0);
    
    // Debug functionality
    void setDebugReceiveHandler(DebugReceiveHandler handler);
    void removeDebugReceiveHandler();
};

// Master class for managing slave communication
class ComProtMaster : public ComProtBase {
private:
    std::vector<SlaveInfo> slaves;
    uint8_t masterId;
    unsigned long heartbeatTimeout;
    
    // Internal methods
    std::vector<SlaveInfo>::iterator findSlave(uint8_t id);
    void handleMessage(uint8_t messageType, uint8_t* payload, uint16_t length,
                       uint8_t srcId, uint8_t dstId) override;
    uint8_t getSelfId() const override { return masterId; }
    void onCrcError() override; // send retry broadcast
    
    // Static instance for callback
    static ComProtMaster* instance;

public:
    ComProtMaster(uint8_t masterId, unsigned long heartbeatTimeout = 3100);
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
};

// Slave class for responding to master commands
class ComProtSlave : public ComProtBase {
private:
    uint8_t slaveId;
    uint8_t slaveType;
    uint8_t masterId;
    unsigned long heartbeatInterval;
    unsigned long lastHeartbeat;
    
    // Retry resend support
    bool resendScheduled = false;
    unsigned long resendAtMicros = 0;
    static const uint16_t MAX_RAW_FRAME = 140;
    uint8_t lastFrame[MAX_RAW_FRAME];
    uint16_t lastFrameLen = 0;
    
    // Command handlers
    std::vector<std::pair<uint8_t, CommandHandler>> commandHandlers;
    
    // Internal methods
    void handleMessage(uint8_t messageType, uint8_t* payload, uint16_t length,
                       uint8_t srcId, uint8_t dstId) override;
    uint8_t getSelfId() const override { return slaveId; }
    
    // Static instance for callback
    static ComProtSlave* instance;

public:
    ComProtSlave(uint8_t slaveId, uint8_t slaveType, uint8_t masterId = 1, unsigned long heartbeatInterval = 1000);
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
};

#endif // COM_PROT_H
