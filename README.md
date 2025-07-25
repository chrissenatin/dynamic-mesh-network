# Dynamic Mesh Network

## Overview

This project implements a dynamic mesh network for communication between nodes
using LoRa technology.

## Components

- **Gateway**: Receives messages from mesh nodes and uploads data to server
- **Messages**: Handles message formatting and communication
- **SpresenseSerial**: Spresense board implementation with MainCore and SubCore1

## Dependencies

### Required Libraries

1. **ArduinoJson** (v6 or v7)
   - Used for JSON parsing and serialization
   - Install via Arduino Library Manager or download from:
     https://github.com/bblanchon/ArduinoJson
   - Installation:
     - Arduino IDE: Tools > Manage Libraries > Search "ArduinoJson" > Install
     - PlatformIO: Add `lib_deps = bblanchon/ArduinoJson@^7.0.0` to
       platformio.ini

2. **RadioHead Library**
   - Provides RH_RF95 driver for LoRa communication
   - Download from: http://www.airspayce.com/mikem/arduino/RadioHead/

3. **LoRaMesh Library** (for Spresense nodes)
   - Custom mesh networking protocol
   - Located in the `/LoRaMesh` directory

## Gateway Configuration

The Gateway has been modified to support both legacy protocol and LoRaMesh
protocol messages from Spresense nodes.

### Key Features:

- Automatic protocol detection (legacy vs LoRaMesh)
- JSON payload parsing
- Acknowledgment sending
- Data upload to server at 152.42.241.248:8000

### Gateway Address

The gateway uses address `0xFE` in the LoRaMesh network.

