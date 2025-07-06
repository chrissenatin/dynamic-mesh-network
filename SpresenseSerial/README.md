# Spresense Multicore LoRa Mesh Network

This project implements a dynamite explosion detection system using Spresense's multicore architecture and LoRa mesh networking.

## Architecture

- **MainCore**: Handles audio recording, FFT analysis, AI inference, and GNSS positioning
- **SubCore1**: Manages LoRa mesh communication via SPI

## Features

1. **Audio Detection**: Continuously monitors audio input for explosion signatures
2. **AI Inference**: Uses DNNRT to classify sounds as explosion/non-explosion
3. **GPS Positioning**: Obtains real-time coordinates from built-in GNSS
4. **Mesh Networking**: Broadcasts detection events through LoRa mesh network
5. **Multicore Processing**: Separates time-critical audio processing from network communication

## Hardware Requirements

- Spresense main board
- Spresense extension board (for audio input)
- LoRa module connected to SPI pins
- SD card with model.nnb file
- Microphone

## Pin Configuration

LoRa module connections (SubCore1):
- CS: Pin 7
- Reset: Pin 6
- IRQ: Pin 1

## Setup Instructions

1. Install the required libraries:
   - Spresense Arduino Library
   - LoRaMesh library

2. Prepare the SD card:
   - Copy `model.nnb` (trained neural network model) to SD card root
   - Copy DSP binaries to `/mnt/sd0/BIN/` directory

3. Upload sketches:
   - Select "MainCore" in Arduino IDE and upload `MainCore.ino`
   - Select "SubCore 1" in Arduino IDE and upload `SubCore1.ino`

## Communication Flow

1. MainCore continuously monitors audio and updates GPS position
2. When explosion is detected (>90% confidence):
   - MainCore sends detection data to SubCore1 via MP library
   - Detection data includes: node ID, latitude, longitude
3. SubCore1 receives the data and sends it to the gateway (0xFE) via LoRa mesh
4. If gateway is unreachable, SubCore1 falls back to broadcast mode
5. The mesh network automatically handles routing and relaying
6. Gateway uploads the detection data to server and sends acknowledgment

## Message Format

Detection messages are sent as JSON:
```json
{
  "id": 145,
  "latitude": 14.648696,
  "longitude": 121.068517
}
```

## Configuration

- Node ID: Set in `nodeConfig.nodeId` (MainCore.ino)
- LoRa frequency: 915MHz (adjust for your region)
- Mesh address: Set in `myAddress` (SubCore1.ino)
- Gateway address: 0xFE (standard gateway address)
- Detection threshold: 90% confidence
- Cooldown period: 10 seconds between detections

## Debugging

- MainCore logs to Serial (USB)
- SubCore logs via MPLog (viewable in Serial Monitor when MainCore selected)
- Enable `MP.EnableConsole()` in SubCore for direct console access

## Notes

- GNSS may take 30-60 seconds to acquire initial fix
- Hardcoded coordinates are used as fallback if GNSS is not ready
- The mesh network automatically handles message relaying