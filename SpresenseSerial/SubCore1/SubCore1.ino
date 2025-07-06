/*
 *  SubCore1.ino - LoRa mesh communication handler
 *  Copyright 2022 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <SPI.h>
#include <LoRaMesh.h>

// Message IDs for MP communication
#define MSG_ID_EXPLOSION_DETECTED 1
#define MSG_ID_SUBCORE_READY 2

// LoRa pins
const int csPin = 7;
const int resetPin = 6;
const int irqPin = 1;

// LoRaMesh instance
LoRaMesh mesh;

// This node's address in the mesh
uint8_t myAddress = 0x01;

// Gateway address
const uint8_t gatewayAddress = 0xFE;

// Detection data structure (must match MainCore)
struct DetectionData {
  uint8_t nodeId;
  float latitude;
  float longitude;
};

void setup() {
  // Initialize MP library
  MP.begin();
  
  // Optional: Enable console for debugging
  // MP.EnableConsole();
  
  MPLog("SubCore1 started\n");
  
  // Configure LoRa pins
  mesh.setPins(csPin, resetPin, irqPin);
  
  // Initialize LoRaMesh at 915MHz
  if (!mesh.begin(915E6, myAddress)) {
    MPLog("Starting LoRa Mesh failed!\n");
    while (1);
  }
  
  MPLog("LoRa Mesh initialized successfully\n");
  
  // Send ready message to MainCore
  int ret = MP.Send(MSG_ID_SUBCORE_READY, 0);
  if (ret < 0) {
    MPLog("Failed to send ready message: %d\n", ret);
  } else {
    MPLog("Ready message sent to MainCore\n");
  }
}

void loop() {
  // Check for messages from MainCore
  int8_t msgid;
  uint32_t msgdata;
  int ret = MP.Recv(&msgid, &msgdata);
  
  if (ret >= 0) {
    MPLog("Received message from MainCore: msgid=%d\n", msgid);
    
    if (msgid == MSG_ID_EXPLOSION_DETECTED && msgdata != 0) {
      // Convert physical address back to virtual address
      DetectionData* detection = (DetectionData*)msgdata;
      
      MPLog("Explosion detected!\n");
      MPLog("Node ID: %d\n", detection->nodeId);
      MPLog("Latitude: %f\n", detection->latitude);
      MPLog("Longitude: %f\n", detection->longitude);
      
      // Format message as JSON
      char message[128];
      snprintf(message, sizeof(message), 
        "{\"id\":%d,\"latitude\":%.6f,\"longitude\":%.6f}", 
        detection->nodeId, detection->latitude, detection->longitude);
      
      // Send to gateway via LoRa mesh
      if (mesh.sendToWait(gatewayAddress, (uint8_t*)message, strlen(message))) {
        MPLog("Message sent to gateway (0x%02X): %s\n", gatewayAddress, message);
      } else {
        MPLog("Failed to send message to gateway\n");
        // Fallback: try broadcast if gateway is unreachable
        if (mesh.sendToWait(LORAMESH_BROADCAST_ADDRESS, (uint8_t*)message, strlen(message))) {
          MPLog("Fallback: Message broadcast successfully\n");
        } else {
          MPLog("Fallback broadcast also failed\n");
        }
      }
    }
  }
  
  // Process LoRa mesh network
  mesh.process();
  
  // Check for incoming LoRa messages
  uint8_t buf[LORAMESH_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t source, dest, id;
  
  if (mesh.recvFromAck(buf, &len, &source, &dest, &id)) {
    MPLog("=== LoRa Message Received ===\n");
    MPLog("From: 0x%02X\n", source);
    MPLog("To: 0x%02X\n", dest);
    MPLog("ID: %d\n", id);
    MPLog("Message: ");
    
    // Null-terminate the message for printing
    buf[len] = '\0';
    MPLog("%s\n", buf);
    MPLog("============================\n");
    
    // The mesh library automatically handles relaying if needed
    
    // Check if this is an acknowledgment from gateway
    if (source == gatewayAddress) {
      MPLog("Received acknowledgment from gateway\n");
    }
  }
  
  // Small delay to prevent tight loop
  delay(10);
}