# UART Migration Summary

## Overview

The com-prot library has been updated to use PJON's ThroughSerial strategy instead of SoftwareBitBang for more reliable UART communication at 9600 baud.

## Key Changes

### 1. Library Changes (com-prot)

#### Header File (com-prot.h)
- Changed from `#define PJON_INCLUDE_SWBB` to `#define PJON_INCLUDE_TS`
- Changed from `#include <PJONSoftwareBitBang.h>` to `#include <PJON.h>`
- Updated `PJON<SoftwareBitBang>` to `PJON<ThroughSerial>`
- Removed pin parameter from constructors
- Updated constructor signatures:
  - `ComProtMaster(uint8_t masterId, unsigned long heartbeatTimeout = 3100)`
  - `ComProtSlave(uint8_t slaveId, uint8_t slaveType, uint8_t masterId = 1, unsigned long heartbeatInterval = 1000)`

#### Implementation File (com-prot.cpp)
- Updated bus initialization to use Serial UART at 9600 baud
- Changed from `bus->strategy.set_pin(pin)` to `bus->strategy.set_serial(&Serial)`
- Added `Serial.begin(9600)` in the begin() method

### 2. Retranslation Station Changes

- Removed `PJON_PIN` definition
- Updated master constructor: `ComProtMaster master(MASTER_ID)`
- Commented out UART command processing (Serial now used for PJON)
- Modified slave info sending (Serial now used for PJON)
- Updated setup to note that Serial is used for PJON communication

### 3. OneWireSlave Changes

- Updated slave constructor: `ComProtSlave slave(SLAVE_ID, SLAVE_TYPE)`
- Changed debug output from Serial to Serial1 throughout the code
- Updated debug macros to use Serial1
- Modified setup to use Serial1 for debug output

### 4. Examples Updated

Both master and slave examples updated to:
- Remove pin parameters from constructors
- Use Serial1 for debug output
- Document UART communication at 9600 baud
- Include migration notes in comments

## Hardware Requirements

### Old Setup (SoftwareBitBang)
- Single wire bus on specified pin (e.g., D1)
- Common ground

### New Setup (UART ThroughSerial)
- UART TX/RX bus configuration
- All TX lines connected together
- All RX lines connected together  
- Common ground
- 9600 baud communication

## Communication Details

- **Baud Rate**: 9600 (automatically configured)
- **UART**: Uses hardware Serial (UART0) 
- **Debug Output**: Must use Serial1 (TX-only on GPIO2)
- **Strategy**: PJON ThroughSerial
- **Acknowledgments**: Disabled for better performance
- **CRC**: 32-bit CRC enabled

## Migration Steps

1. **Update Hardware**: Change wiring from single-wire to UART bus
2. **Update Code**: Remove pin parameters from com-prot constructors
3. **Update Debug**: Change Serial debug output to Serial1
4. **Test Communication**: Verify 9600 baud UART communication works

## Benefits

- More reliable communication
- Higher data throughput potential
- Standard UART protocol
- Better error detection with CRC32
- Asynchronous operation for better timing tolerance

## Limitations

- Requires UART bus wiring (more complex than single wire)
- Serial port no longer available for other uses
- Debug output limited to Serial1 (TX-only)
