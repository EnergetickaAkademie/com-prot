# Com-Prot Library

A PJON-based communication protocol library for ESP8266 master-slave networks.

## Features

- **Master-Slave Architecture**: Simple and reliable communication pattern
- **Type-based Broadcasting**: Send commands to all slaves of a specific type using PJON broadcast
- **Individual Addressing**: Send commands to specific slaves by ID
- **Automatic Slave Discovery**: Masters automatically detect connecting/disconnecting slaves
- **Flexible Command Handling**: Slaves can register custom handlers for different command types
- **Debug Receive Handler**: Optional debug handler called on every received message
- **Heartbeat System**: Automatic connection monitoring and timeout detection
- **Built on PJON**: Leverages robust PJON library for reliable communication

## Installation

### PlatformIO

Add to your `platformio.ini`:

```ini
lib_deps = 
    https://github.com/EnergetickaAkademie/com-prot.git
```

### Arduino IDE

1. Download this repository as ZIP
2. In Arduino IDE: Sketch → Include Library → Add .ZIP Library
3. Select the downloaded ZIP file

## Basic Usage

### Master Example

```cpp
#include <com-prot.h>

ComProtMaster master(1, D1); // Master ID 1, pin D1

// Debug handler - called for every received message
void debugHandler(uint8_t* payload, uint16_t length, uint8_t senderId, uint8_t messageType) {
    Serial.printf("[DEBUG] RX from %d: type=0x%02X, len=%d\n", senderId, messageType, length);
}

void setup() {
    Serial.begin(115200);
    
    // Set debug receive handler (optional)
    master.setDebugReceiveHandler(debugHandler);
    
    master.begin();
}

void loop() {
    master.update();
    
    // Send command 0x10 to all slaves of type 2 (uses PJON broadcast)
    master.sendCommandToSlaveType(2, 0x10);
    
    // Send command 0x20 to specific slave ID 5 (unicast)
    master.sendCommandToSlaveId(5, 0x20);
    
    // Get all connected slaves
    auto slaves = master.getConnectedSlaves();
    Serial.printf("Connected slaves: %d\n", slaves.size());
    
    delay(1000);
}
```

### Slave Example

```cpp
#include <com-prot.h>

ComProtSlave slave(10, 2, D1); // Slave ID 10, Type 2, pin D1

void handleLedCommand(uint8_t* data, uint16_t length, uint8_t senderId) {
    if (length > 0) {
        bool ledState = data[0];
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        Serial.printf("LED turned %s by master %d\n", ledState ? "ON" : "OFF", senderId);
    }
}

void handleTemperatureRequest(uint8_t* data, uint16_t length, uint8_t senderId) {
    // Read temperature sensor
    float temperature = 25.5; // Mock value
    
    // Send response back to master
    uint8_t response[4];
    memcpy(response, &temperature, sizeof(temperature));
    slave.sendResponse(0x21, response, sizeof(response));
}

// Debug handler - called for every received message
void debugHandler(uint8_t* payload, uint16_t length, uint8_t senderId, uint8_t messageType) {
    Serial.printf("[DEBUG] RX from %d: type=0x%02X, len=%d\n", senderId, messageType, length);
}

void setup() {
    Serial.begin(115200);
    
    // Register command handlers
    slave.setCommandHandler(0x10, handleLedCommand);
    slave.setCommandHandler(0x20, handleTemperatureRequest);
    
    // Set debug receive handler (optional)
    slave.setDebugReceiveHandler(debugHandler);
    
    slave.begin();
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    slave.update();
    delay(10);
}
```

## API Reference

### ComProtMaster

#### Constructor
```cpp
ComProtMaster(uint8_t masterId, uint8_t pin, unsigned long heartbeatTimeout = 3100);
```

#### Methods

- `void begin()` - Initialize the master
- `void update()` - Call in loop() to handle communication
- `bool sendCommandToSlaveType(uint8_t slaveType, uint8_t command, uint8_t* data = nullptr, uint16_t dataLen = 0)` - Send command to all slaves of specific type
- `bool sendCommandToSlaveId(uint8_t slaveId, uint8_t command, uint8_t* data = nullptr, uint16_t dataLen = 0)` - Send command to specific slave
- `std::vector<SlaveInfo> getConnectedSlaves()` - Get all connected slaves
- `std::vector<SlaveInfo> getSlavesByType(uint8_t type)` - Get slaves of specific type
- `bool isSlaveConnected(uint8_t id)` - Check if slave is connected
- `void setHeartbeatTimeout(unsigned long timeout)` - Set heartbeat timeout
- `size_t getSlaveCount()` - Get number of connected slaves
- `void setDebugReceiveHandler(DebugReceiveHandler handler)` - Set debug receive handler
- `void removeDebugReceiveHandler()` - Remove debug receive handler

### ComProtSlave

#### Constructor
```cpp
ComProtSlave(uint8_t slaveId, uint8_t slaveType, uint8_t pin, uint8_t masterId = 1, unsigned long heartbeatInterval = 1000);
```

#### Methods

- `void begin()` - Initialize the slave
- `void update()` - Call in loop() to handle communication
- `void setCommandHandler(uint8_t commandType, CommandHandler handler)` - Register command handler
- `void removeCommandHandler(uint8_t commandType)` - Remove command handler
- `bool sendResponse(uint8_t commandType, uint8_t* data = nullptr, uint16_t dataLen = 0)` - Send response to master
- `void setHeartbeatInterval(unsigned long interval)` - Set heartbeat interval
- `void setDebugReceiveHandler(DebugReceiveHandler handler)` - Set debug receive handler
- `void removeDebugReceiveHandler()` - Remove debug receive handler

#### Command Handler Function
```cpp
typedef std::function<void(uint8_t* payload, uint16_t length, uint8_t senderId)> CommandHandler;
```

#### Debug Receive Handler Function
```cpp
typedef std::function<void(uint8_t* payload, uint16_t length, uint8_t senderId, uint8_t messageType)> DebugReceiveHandler;
```

## Hardware Setup

Connect all devices to the same PJON bus:

```
Master (ID: 1)     Slave 1 (ID: 10)    Slave 2 (ID: 11)
       |                  |                    |
     Pin D1             Pin D1               Pin D1
       |                  |                    |
       +------------------+--------------------+  (PJON Bus)
       |                  |                    |
      GND                GND                  GND
       |                  |                    |
       +------------------+--------------------+  (Common Ground)
```

## Protocol Details

### Message Types
- `COM_PROT_HEARTBEAT` (0x03): Slave heartbeat messages
- `COM_PROT_COMMAND` (0x04): Command messages

### Message Format

**Heartbeat**: `[0x03, slave_id, slave_type]`

**Command to Type (Broadcast)**: `[0x04, slave_type, command, data...]`

**Command to ID (Unicast)**: `[0x04, 0x00, command, data...]`

### Debug Handler

The debug receive handler is called for every received message, allowing you to:
- Monitor all network traffic
- Debug communication issues  
- Log message statistics
- Implement custom message filtering

Example debug handler:
```cpp
void debugHandler(uint8_t* payload, uint16_t length, uint8_t senderId, uint8_t messageType) {
    Serial.printf("[DEBUG] From %d: Type=0x%02X, Len=%d\n", senderId, messageType, length);
    
    // Print payload data
    for (uint16_t i = 0; i < length && i < 8; i++) {
        Serial.printf("0x%02X ", payload[i]);
    }
    Serial.println();
}
```

## Integration with Existing Projects

This library is designed to be a drop-in replacement for the existing PJON implementation in your OneWireHost and OneWireSlave projects. Simply replace the manual PJON handling with the library classes.

## License

MIT License - see LICENSE file for details.
