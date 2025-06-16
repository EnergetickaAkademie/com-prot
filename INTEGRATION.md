# Integration Guide

This guide shows how to integrate the Com-Prot library into your existing OneWireHost and OneWireSlave projects.

## Updating OneWireHost Project

### 1. Update platformio.ini

Add the library dependency:

```ini
lib_deps = 
    https://github.com/gioblu/PJON.git
    https://github.com/EnergetickaAkademie/ota.git
    https://github.com/EnergetickaAkademie/com-prot.git
    ayushsharma82/WebSerial@^1.4.0
    ottowinter/ESPAsyncWebServer-esphome@^3.0.0
```

### 2. Replace main.cpp

Replace your current `OneWireHost/src/main.cpp` with:

```cpp
#include <Arduino.h>
#include <com-prot.h>
#include <ota.h>
#include "secrets.h"

// Create master instance
ComProtMaster master(1, D1); // Master ID 1, pin D1

void setup() {
    Serial.begin(115200);
    Serial.println("PJON Slave Discovery Master");
    
    // Connect to WiFi and setup OTA
    connectWifi(SECRET_SSID, SECRET_PASSWORD);
    setupOTA(-1, "PjonMaster");
    setupWebSerial("PjonMaster");
    
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Initialize the master
    master.begin();
    
    Serial.println("PJON Master initialized with Com-Prot library");
}

void loop() {
    // Handle OTA updates
    handleOTA();
    
    // Update master (handles incoming messages and timeouts)
    master.update();
    
    // Print slave list every second
    static unsigned long lastListPrint = 0;
    if (millis() - lastListPrint > 1000) {
        auto slaves = master.getConnectedSlaves();
        WebSerial.printf("Active slaves (%d): ", slaves.size());
        
        if (slaves.empty()) {
            WebSerial.println("None");
        } else {
            for (const auto& slave : slaves) {
                WebSerial.printf("ID: %d, Type: %d ", slave.id, slave.type);
            }
            WebSerial.println();
        }
        WebSerial.flush();
        
        lastListPrint = millis();
    }
    
    // Example: Send commands to different slave types
    static unsigned long lastCommand = 0;
    if (millis() - lastCommand > 5000) {
        // Send LED toggle to type 1 slaves
        if (master.getSlavesByType(1).size() > 0) {
            uint8_t ledState = (millis() / 5000) % 2;
            master.sendCommandToSlaveType(1, 0x10, &ledState, 1);
        }
        
        // Send temperature request to type 2 slaves
        if (master.getSlavesByType(2).size() > 0) {
            master.sendCommandToSlaveType(2, 0x20);
        }
        
        lastCommand = millis();
    }
}
```

## Updating OneWireSlave Project

### 1. Update platformio.ini

The `[env]` section should include the library:

```ini
[env]
platform = espressif8266
board = d1_mini
framework = arduino
monitor_speed = 115200
lib_deps = 
    https://github.com/gioblu/PJON.git
    https://github.com/EnergetickaAkademie/ota.git
    https://github.com/EnergetickaAkademie/com-prot.git
    ayushsharma82/WebSerial@^1.4.0
    ottowinter/ESPAsyncWebServer-esphome@^3.0.0
```

### 2. Replace main.cpp

Replace your current `OneWireSlave/src/main.cpp` with:

```cpp
#include <Arduino.h>
#include <com-prot.h>
#include <ota.h>
#include "secrets.h"

// Configuration from build flags (set in platformio.ini)
#ifndef SLAVE_ID
#define SLAVE_ID 10  // Default fallback
#endif

#ifndef SLAVE_TYPE
#define SLAVE_TYPE 1 // Default fallback
#endif

#ifndef DEVICE_NAME
#define DEVICE_NAME "PjonSlave"  // Default fallback
#endif

// Create slave instance
ComProtSlave slave(SLAVE_ID, SLAVE_TYPE, D1); // Use build flags for ID and type

// LED control
constexpr uint8_t pin_led{LED_BUILTIN};

// Command handlers
void handleLedCommand(uint8_t* data, uint16_t length, uint8_t senderId) {
    if (length > 0) {
        bool ledState = data[0];
        digitalWrite(pin_led, ledState ? LOW : HIGH); // LED is inverted
        
        Serial.printf("LED turned %s by master %d\n", ledState ? "ON" : "OFF", senderId);
        WebSerial.printf("LED command: %s\n", ledState ? "ON" : "OFF");
        WebSerial.flush();
    }
}

void handleTemperatureRequest(uint8_t* data, uint16_t length, uint8_t senderId) {
    // Simulate temperature reading
    float temperature = 20.0 + (millis() % 10000) / 1000.0;
    
    // Send response back to master
    uint8_t response[4];
    memcpy(response, &temperature, sizeof(temperature));
    slave.sendResponse(0x21, response, sizeof(response));
    
    Serial.printf("Temperature sent: %.2f°C\n", temperature);
    WebSerial.printf("Temperature: %.2f°C\n", temperature);
    WebSerial.flush();
}

void setup() {
    Serial.begin(115200);
    Serial.println("PJON Temperature Sensor Slave");
    Serial.println("Booting");
    
    pinMode(pin_led, OUTPUT);
    
    // Connect to WiFi and setup OTA
    connectWifi(SECRET_SSID, SECRET_PASSWORD);
    setupOTA(-1, DEVICE_NAME);
    setupWebSerial(DEVICE_NAME);

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Register command handlers
    slave.setCommandHandler(0x10, handleLedCommand);        // LED control
    slave.setCommandHandler(0x20, handleTemperatureRequest); // Temperature request
    
    // Initialize the slave
    slave.begin();
    
    Serial.printf("PJON Slave initialized with Com-Prot library - ID: %d, Type: %d\n", SLAVE_ID, SLAVE_TYPE);
    
    WebSerial.printf("Slave ID: %d, Type: %d\n", SLAVE_ID, SLAVE_TYPE);
    WebSerial.flush();
}

void loop() {
    // Handle OTA updates
    handleOTA();
    
    // Update slave (handles incoming messages and heartbeats)
    slave.update();
    
    // Small delay for stability
    delay(10);
}
```

## Keeping Existing Scripts

Your existing `manage_slaves.py` script will continue to work without changes since:

1. The platformio.ini structure remains the same
2. The build flags (SLAVE_ID, SLAVE_TYPE, DEVICE_NAME) are still used
3. OTA functionality is preserved

## Benefits of Migration

### Before (Manual PJON)
- ~200 lines of boilerplate code per project
- Manual slave tracking and timeout handling
- Complex message parsing logic
- No command abstraction

### After (Com-Prot Library)
- ~50 lines of clean application code
- Automatic slave management
- Simple command handler registration
- Type-safe API with clear method names

## Testing the Migration

1. **Flash Master**: Use the updated OneWireHost code
2. **Flash Slaves**: Use existing `manage_slaves.py` script with updated slave code
3. **Verify Communication**: Check WebSerial output shows slaves connecting
4. **Test Commands**: Observe slaves responding to LED and temperature commands

## Custom Command Examples

### Adding New Commands

**Master side:**
```cpp
// Send custom sensor calibration command to type 3 slaves
uint8_t calibrationData[] = {0x01, 0x02, 0x03, 0x04};
master.sendCommandToSlaveType(3, 0x40, calibrationData, sizeof(calibrationData));
```

**Slave side:**
```cpp
void handleCalibration(uint8_t* data, uint16_t length, uint8_t senderId) {
    // Process calibration data
    Serial.println("Calibration command received");
    // ... implementation
}

// In setup():
slave.setCommandHandler(0x40, handleCalibration);
```

## Troubleshooting

### Compilation Errors
- Ensure all library dependencies are correctly listed in platformio.ini
- Check that #include paths are correct

### Communication Issues
- Verify pin configurations match between master and slaves
- Check that slave IDs are unique
- Monitor serial output for PJON errors

### Missing Features
- The library maintains full compatibility with your existing OTA and WebSerial setup
- All manage_slaves.py functionality is preserved
