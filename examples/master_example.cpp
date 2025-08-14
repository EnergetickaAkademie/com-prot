/*
 * Master Example using Com-Prot Library with UART ThroughSerialAsync
 * 
 * This example shows how to use the Com-Prot library with PJON's
 * ThroughSerialAsync strategy for reliable UART communication at 9600 baud.
 */

#include <Arduino.h>
#include <com-prot.h>
#include <ota.h>
#include "secrets.h"

// Create master instance (no pin needed for UART)
ComProtMaster master(1); // Master ID 1

// Debug receive handler - called for every received message
void debugReceiveHandler(uint8_t* payload, uint16_t length, uint8_t senderId, uint8_t messageType) {
    // Use Serial1 for debug output since Serial is used for PJON communication
    Serial1.printf("[DEBUG] Received from slave %d, type: 0x%02X, length: %d\n", senderId, messageType, length);
    
    // Log to WebSerial as well
    WebSerial.printf("[DEBUG] RX: Slave=%d, Type=0x%02X, Len=%d\n", senderId, messageType, length);
    WebSerial.flush();
    
    // Optionally print payload data for non-heartbeat messages
    if (messageType != 0x03 && length > 1) { // Skip heartbeat messages for cleaner output
        Serial1.print("[DEBUG] Payload: ");
        for (uint16_t i = 0; i < length && i < 16; i++) { // Limit to first 16 bytes
            Serial1.printf("0x%02X ", payload[i]);
        }
        if (length > 16) Serial1.print("...");
        Serial1.println();
    }
}

void setup() {
    // Use Serial1 for debug output (TX only on GPIO2)
    Serial1.begin(115200);
    Serial1.println("Com-Prot Master Example - UART ThroughSerialAsync");
    
    // Connect to WiFi and setup OTA
    connectWifi(SECRET_SSID, SECRET_PASSWORD);
    setupOTA(-1, "ComProtMaster");
    
    // Setup WebSerial debugging
    setupWebSerial("ComProtMaster");
    
    Serial1.println("Ready");
    Serial1.print("IP address: ");
    Serial1.println(WiFi.localIP());
    
    // Set debug receive handler
    master.setDebugReceiveHandler(debugReceiveHandler);
    
    // Initialize the master (this will setup Serial for PJON at 9600 baud)
    master.begin();
    
    Serial1.println("Com-Prot Master initialized with UART at 9600 baud");
    WebSerial.println("UART ThroughSerialAsync initialized at 9600 baud");
    WebSerial.println("Debug receive handler enabled");
    WebSerial.flush();
}

void loop() {
    // Handle OTA updates
    handleOTA();
    
    // Update master (handles incoming messages and timeouts)
    master.update();
    
    // Example: Send commands to slaves every 5 seconds
    static unsigned long lastCommand = 0;
    if (millis() - lastCommand > 5000) {
        
        // Get all connected slaves
        auto allSlaves = master.getConnectedSlaves();
        Serial1.printf("Connected slaves: %d\n", allSlaves.size());
        
        for (const auto& slave : allSlaves) {
            Serial1.printf("Slave ID: %d, Type: %d\n", slave.id, slave.type);
        }
        
        // Example commands:
        
        // 1. Send LED toggle command (0x10) to all slaves of type 1 using broadcast
        if (master.getSlavesByType(1).size() > 0) {
            uint8_t ledState = (millis() / 5000) % 2; // Toggle every 5 seconds
            master.sendCommandToSlaveType(1, 0x10, &ledState, 1);
            Serial1.printf("Sent LED broadcast command (%d) to type 1 slaves\n", ledState);
            WebSerial.printf("LED broadcast to type 1: %s\n", ledState ? "ON" : "OFF");
        }
        
        // 2. Send temperature request (0x20) to all slaves of type 2 using broadcast
        if (master.getSlavesByType(2).size() > 0) {
            master.sendCommandToSlaveType(2, 0x20);
            Serial1.println("Sent temperature request broadcast to type 2 slaves");
            WebSerial.println("Temperature request broadcast to type 2");
        }
        
        // 3. Send custom command to specific slave ID 10 (unicast)
        if (master.isSlaveConnected(10)) {
            uint8_t customData[] = {0xAA, 0xBB, 0xCC};
            master.sendCommandToSlaveId(10, 0x30, customData, sizeof(customData));
            Serial1.println("Sent custom unicast command to slave 10");
            WebSerial.println("Custom command sent to slave 10");
        }
        
        lastCommand = millis();
    }
    
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
}
