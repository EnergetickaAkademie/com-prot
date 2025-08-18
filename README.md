# Com-Prot Library

A PJON-based communication protocol library for ESP8266 master-slave networks using UART communication.

## Features

- **UART Communication**: Uses PJON's ThroughSerial strategy for reliable communication at 9600 baud
- **Master-Slave Architecture**: Simple and reliable communication pattern
- **Type-based Broadcasting**: Send commands to all slaves of a specific type using PJON broadcast
- **Individual Addressing**: Send commands to specific slaves by ID
- **Automatic Slave Discovery**: Masters automatically detect connecting/disconnecting slaves
- **Flexible Command Handling**: Slaves can register custom handlers for different command types
- **Debug Receive Handler**: Optional debug handler called on every received message
- **Heartbeat System**: Automatic connection monitoring and timeout detection
- **Built on PJON**: Leverages robust PJON library for reliable communication

## Hardware Requirements

- ESP8266 devices connected via UART
- All devices must share common ground
- UART TX/RX lines connected in a bus configuration
- 9600 baud communication speed

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

ComProtMaster master(1); // Master ID 1 (no pin needed for UART)

// Debug handler - called for every received message
void debugHandler(uint8_t* payload, uint16_t length, uint8_t senderId, uint8_t messageType) {
    Serial1.printf("[DEBUG] RX from %d: type=0x%02X, len=%d\n", senderId, messageType, length);
}

void setup() {
    Serial1.begin(115200); // Use Serial1 for debug, Serial is used for PJON
    
    // Set debug receive handler (optional)
    master.setDebugReceiveHandler(debugHandler);
    
    master.begin(); // This initializes Serial for PJON at 9600 baud
}

void loop() {
    master.update();
    
    // Send command 0x10 to all slaves of type 2 (uses PJON broadcast)
    master.sendCommandToSlaveType(2, 0x10);
    
    // Send command 0x20 to specific slave ID 5 (unicast)
    master.sendCommandToSlaveId(5, 0x20);
    
    // Get all connected slaves
    auto slaves = master.getConnectedSlaves();
    Serial1.printf("Connected slaves: %d\n", slaves.size());
    
    delay(1000);
}
```

### Slave Example

```cpp
#include <com-prot.h>

ComProtSlave slave(10, 2); // Slave ID 10, Type 2 (no pin needed for UART)

void handleLedCommand(uint8_t* data, uint16_t length, uint8_t senderId) {
    if (length > 0) {
        bool ledState = data[0];
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        Serial1.printf("LED turned %s by master %d\n", ledState ? "ON" : "OFF", senderId);
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
    Serial1.printf("[DEBUG] RX from %d: type=0x%02X, len=%d\n", senderId, messageType, length);
}

void setup() {
    Serial1.begin(115200); // Use Serial1 for debug, Serial is used for PJON
    
    // Register command handlers
    slave.setCommandHandler(0x10, handleLedCommand);
    slave.setCommandHandler(0x20, handleTemperatureRequest);
    
    // Set debug receive handler (optional)
    slave.setDebugReceiveHandler(debugHandler);
    
    slave.begin(); // This initializes Serial for PJON at 9600 baud
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
ComProtMaster(uint8_t masterId, unsigned long heartbeatTimeout = 3100);
```
- `masterId`: Unique ID for this master (1-254)
- `heartbeatTimeout`: Timeout in milliseconds for slave heartbeats

#### Methods

- `void begin()` - Initialize the master and UART at 9600 baud
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
ComProtSlave(uint8_t slaveId, uint8_t slaveType, uint8_t masterId = 1, unsigned long heartbeatInterval = 1000);
```
- `slaveId`: Unique ID for this slave (1-254)
- `slaveType`: Type identifier for this slave
- `masterId`: ID of the master to communicate with
- `heartbeatInterval`: Interval in milliseconds for sending heartbeats

#### Methods

- `void begin()` - Initialize the slave and UART at 9600 baud
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

Connect all devices via UART in a bus configuration:

```
Master (ID: 1)     Slave 1 (ID: 10)    Slave 2 (ID: 11)
   RX  TX             RX  TX             RX  TX  
    |  |               |  |               |  |
    |  +---------------+--+---------------+--+  TX Line (9600 baud)
    |                  |                  |
    +------------------+------------------+     RX Line (9600 baud)
    |                  |                  |
   GND                GND                GND
    |                  |                  |
    +------------------+------------------+     Common Ground
```

**Important Notes:**
- All devices must share a common ground
- UART operates at 9600 baud
- TX lines are connected together
- RX lines are connected together
- Serial (UART0) is used for PJON communication
- Use Serial1 for debugging output

## Communication Protocol

### Message Types
- `COM_PROT_HEARTBEAT` (0x03): Slave heartbeat messages
- `COM_PROT_COMMAND` (0x04): Command messages

### Message Format

**Heartbeat**: `[0x03, slave_id, slave_type]`

**Command to Type (Broadcast)**: `[0x04, slave_type, command, data...]`

**Command to ID (Unicast)**: `[0x04, 0x00, command, data...]`

### UART Communication Details

- **Baud Rate**: 9600 (configured automatically)
- **Data Bits**: 8
- **Parity**: None  
- **Stop Bits**: 1
- **Flow Control**: None
- **Strategy**: PJON ThroughSerial
- **Acknowledgments**: Disabled for better performance
- **CRC**: 32-bit CRC enabled for data integrity

## Migration from SoftwareBitBang

If you're migrating from the previous SoftwareBitBang implementation:

1. **Remove pin parameters** from constructors:
   ```cpp
   // Old:
   ComProtMaster master(1, D1);
   ComProtSlave slave(10, 2, D1);
   
   // New:
   ComProtMaster master(1);
   ComProtSlave slave(10, 2);
   ```

2. **Update debug output** to use Serial1:
   ```cpp
   // Old:
   Serial.println("Debug message");
   
   // New:
   Serial1.println("Debug message");
   ```

3. **Hardware wiring**: Change from single-wire to UART TX/RX bus

## Debugging

The library provides a debug receive handler that's called for every received message:

```cpp
void debugHandler(uint8_t* payload, uint16_t length, uint8_t senderId, uint8_t messageType) {
    Serial1.printf("[DEBUG] From %d: Type=0x%02X, Len=%d\n", senderId, messageType, length);
    
    // Print payload data (limit output for performance)
    if (messageType != 0x03 && length > 1) { // Skip heartbeats
        Serial1.print("Data: ");
        for (uint16_t i = 0; i < length && i < 8; i++) {
            Serial1.printf("0x%02X ", payload[i]);
        }
        Serial1.println();
    }
}

// Register the handler
master.setDebugReceiveHandler(debugHandler);
slave.setDebugReceiveHandler(debugHandler);
```

## Integration with Existing Projects

This library is designed to be a drop-in replacement for manual PJON handling. The main changes:

1. Remove manual PJON bus configuration
2. Replace pin-based communication with UART
3. Update debug output to use Serial1
4. Update hardware wiring for UART bus

## License

MIT License - see LICENSE file for details.
