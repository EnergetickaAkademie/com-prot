/*
 * Slave Example using Com-Prot Library
 * 
 * This example shows how to replace the existing OneWireSlave implementation
 * with the Com-Prot library for simplified slave functionality.
 */

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
#define DEVICE_NAME "ComProtSlave"  // Default fallback
#endif

// Create slave instance
ComProtSlave slave(SLAVE_ID, SLAVE_TYPE, D1); // Use build flags for ID and type

// LED control
constexpr uint8_t pin_led{LED_BUILTIN};
bool ledBlinking = false;

// Command handlers
void handleLedCommand(uint8_t* data, uint16_t length, uint8_t senderId) {
    if (length > 0) {
        bool ledState = data[0];
        digitalWrite(pin_led, ledState ? LOW : HIGH); // LED is inverted on ESP8266
        
        Serial.printf("LED turned %s by master %d\n", ledState ? "ON" : "OFF", senderId);
        
        WebSerial.printf("LED command received: %s\n", ledState ? "ON" : "OFF");
        WebSerial.flush();
    }
}

void handleTemperatureRequest(uint8_t* data, uint16_t length, uint8_t senderId) {
    // Simulate temperature reading
    float temperature = 20.0 + (millis() % 10000) / 1000.0; // 20-30°C range
    
    Serial.printf("Temperature request from master %d, sending: %.2f°C\n", senderId, temperature);
    
    // Send response back to master
    uint8_t response[4];
    memcpy(response, &temperature, sizeof(temperature));
    slave.sendResponse(0x21, response, sizeof(response)); // Response command type 0x21
    
    WebSerial.printf("Temperature sent: %.2f°C\n", temperature);
    WebSerial.flush();
}

void handleCustomCommand(uint8_t* data, uint16_t length, uint8_t senderId) {
    Serial.printf("Custom command from master %d, data length: %d\n", senderId, length);
    
    if (length > 0) {
        Serial.print("Data: ");
        for (uint16_t i = 0; i < length; i++) {
            Serial.printf("0x%02X ", data[i]);
        }
        Serial.println();
        
        // Start LED blinking for 5 seconds
        ledBlinking = true;
        
        WebSerial.printf("Custom command received with %d bytes\n", length);
        WebSerial.flush();
    }
}

void blinkLed() {
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    
    if (ledBlinking && millis() - lastBlink > 200) {
        blinkState = !blinkState;
        digitalWrite(pin_led, blinkState ? LOW : HIGH); // LED is inverted
        lastBlink = millis();
        
        // Stop blinking after 5 seconds
        static unsigned long blinkStart = 0;
        if (blinkStart == 0) blinkStart = millis();
        if (millis() - blinkStart > 5000) {
            ledBlinking = false;
            blinkStart = 0;
            digitalWrite(pin_led, HIGH); // Turn off LED
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Com-Prot Slave Example");
    Serial.flush();
    Serial.println("Booting");
    
    pinMode(pin_led, OUTPUT);
    digitalWrite(pin_led, HIGH); // Turn off LED initially (inverted)
    
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
    slave.setCommandHandler(0x30, handleCustomCommand);      // Custom command
    
    // Initialize the slave
    slave.begin();
    
    Serial.printf("Com-Prot Slave initialized - ID: %d, Type: %d\n", SLAVE_ID, SLAVE_TYPE);
    
    WebSerial.printf("Slave ID: %d, Type: %d\n", SLAVE_ID, SLAVE_TYPE);
    WebSerial.println("Command handlers registered:");
    WebSerial.println("  0x10 - LED Control");
    WebSerial.println("  0x20 - Temperature Request");
    WebSerial.println("  0x30 - Custom Command");
    WebSerial.flush();
    
    Serial.println("Setup complete - sending heartbeats to master");
}

void loop() {
    // Handle OTA updates
    handleOTA();
    
    // Update slave (handles incoming messages and heartbeats)
    slave.update();
    
    // Handle LED blinking if active
    blinkLed();
    
    // Status indication
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 10000) { // Every 10 seconds
        WebSerial.println("Slave alive and listening");
        WebSerial.flush();
        lastStatus = millis();
    }
    
    // Small delay for stability
    delay(10);
}
