//If you use Dragino IoT Mesh Firmware, uncomment below lines.
//For product: LG01. 
#define BAUDRATE 115200

#include <Console.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Process.h>
#include <ArduinoJson.h>

// Singleton instance of the radio driver
RH_RF95 rf95;

int led = A2;
float frequency = 915.0;

// Gateway address (0xFE is typically used for gateways in LoRaMesh)
#define buoyID 0xFE

// LoRaMesh protocol constants
#define LORAMESH_BROADCAST_ADDRESS 0xFF
#define LORAMESH_MAX_HOPS 10
#define LORAMESH_MAX_MESSAGE_LEN 251

// Message Types
#define MESSAGE_TYPE_DATA 0x00
#define MESSAGE_TYPE_ROUTE_REQUEST 0x01
#define MESSAGE_TYPE_ROUTE_REPLY 0x02
#define MESSAGE_TYPE_ROUTE_FAILURE 0x03
#define MESSAGE_TYPE_ACK 0x04


#define SHORE_LAT 14.648696
#define SHORE_LONG 121.068517


void parseMessage (char *message, uint8_t *type, uint8_t *reply, uint8_t *source_id, uint8_t *destination_id, char *payload){
  // get header from the message and extract info
  uint16_t header = (message[1] << 8) | message[0] << 0;
  *type = (header & 32768) >> 15;
  *reply = (header & 16384) >> 14;
  *source_id = (header & 16256) >> 7;
  *destination_id = (header & 127);

  // get payload from message
  // message+2 gives string from 3rd byte skipping header
  // strlen(message+2)+1 to include null terminator
  memmove(payload, message+2, strlen(message+2)+1);
}
void parsePayload (char *payload, double *latitude, double *longitude, uint8_t *origin_id){
  // Extract payload contents
  StaticJsonDocument<256> payload_json;
  deserializeJson(payload_json, payload);
  
  *latitude = payload_json["latitude"];   // Will return 0 if none
  *longitude = payload_json["longitude"]; // Will return 0 if none
  *origin_id = payload_json["id"];
}

void parseLoRaMeshMessage(uint8_t *message, uint8_t len, uint8_t *source, uint8_t *destination, uint8_t *messageType, char *payload) {
  // LoRaMesh message format:
  // Byte 0: Destination
  // Byte 1: Source
  // Byte 2: Message ID
  // Byte 3: Message Type
  // Byte 4: Hop Count
  // Byte 5: Visited Count
  // Byte 6-N: Visited Nodes (variable length based on visited count)
  // Byte N+1: Next Hop
  // Byte N+2: Data Length
  // Byte N+3 onwards: Data
  
  if (len < 8) return; // Minimum header size
  
  *destination = message[0];
  *source = message[1];
  *messageType = message[3];
  uint8_t visitedCount = message[5];
  
  // Skip to data section
  uint8_t dataOffset = 6 + visitedCount + 2; // header + visited nodes + next hop + data length
  
  if (dataOffset >= len) return;
  
  uint8_t dataLen = message[dataOffset - 1];
  
  if (dataOffset + dataLen > len) return;
  
  // Copy the data payload
  if (dataLen > 0) {
    memcpy(payload, &message[dataOffset], dataLen);
    payload[dataLen] = '\0';
  } else {
    payload[0] = '\0';
  }
}

void sendLoRaMeshAck(uint8_t destination, uint8_t messageId) {
  // Send acknowledgment in LoRaMesh format using MESSAGE_TYPE_ACK
  uint8_t ackMessage[20];
  uint8_t pos = 0;
  
  // Build LoRaMesh header
  ackMessage[pos++] = destination;      // Destination
  ackMessage[pos++] = buoyID;          // Source (gateway)
  ackMessage[pos++] = messageId;       // Use original message ID for ACK
  ackMessage[pos++] = MESSAGE_TYPE_ACK; // Message Type ACK
  ackMessage[pos++] = 0;               // Hop Count
  ackMessage[pos++] = 0;               // Visited Count (no visited nodes)
  ackMessage[pos++] = destination;     // Next Hop (direct)
  ackMessage[pos++] = 0;               // Data Length (ACK has no payload)
  
  rf95.send(ackMessage, pos);
  rf95.waitPacketSent();
}
void getBroadcastReply (char *message, uint8_t destination_id, double latitude, double longitude) {
  // create 16 bit header
  // 0 type = broadcast
  // 1 reply = yes
  // 7 bits source ID
  // 7 bits destination ID

  // set reply to 1
  uint16_t header = 16384;

  // set source ID
  header = header | (buoyID << 7);

  // set destination ID
  header = header | destination_id;

  // convert to bitstring
  memcpy(message, &header, 2);
  message[2] = '\0';

  // create and concatenate payload to message
  char payload[48];
  createPayload(payload, 48, latitude, longitude);

  strcat(message, payload);
}

void createPayload(char *payload, size_t buffer_size, double latitude, double longitude){
  // create payload as Json
  StaticJsonDocument<128> payload_json;

  payload_json["latitude"] = latitude;
  payload_json["longitude"] = longitude;
  payload_json["id"] = buoyID;

  // move JSON to the payload buffer
  serializeJson(payload_json, payload, buffer_size);
}
void getDynamiteAcknowledge (char *message, uint8_t destination_id, uint8_t origin_id) {
  // create 16 bit header - 00(srcID)(destID)
  // 1 type = dynamite
  // 1 reply = yes
  // 7 bits source ID = buoyID
  // 7 bits destination ID = destination_id

  // 1 type 1 reply, set own ID as source and destination ID as given
  uint16_t header = 49152 | (buoyID << 8) | (destination_id);

  // convert to bitstring
  memcpy(message, &header, 2);
  message[2] = '\0';

  // create and concatenate payload to message
  char payload[11];
  // create the JSON file
  StaticJsonDocument<128> payload_json;

  payload_json["id"] = origin_id;

  // move JSON to the payload buffer
  serializeJson(payload_json, payload, 11);

  strcat(message, payload);
}


void setup() 
{
  pinMode(led, OUTPUT);     
  Bridge.begin(BAUDRATE);
  Console.begin();
  while (!Console) ; // Wait for console port to be available
  Console.println("Start Sketch");
  if (!rf95.init())
    Console.println("init failed");
  // Setup ISM frequency
  rf95.setFrequency(frequency);
  // Setup Power,dBm
  rf95.setTxPower(13);
  
  // Setup Spreading Factor (6 ~ 12)
  rf95.setSpreadingFactor(7);
  
  // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
  rf95.setSignalBandwidth(125000);
  
  // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) 
  rf95.setCodingRate4(5);
  
  Console.print("Listening on frequency: ");
  Console.println(frequency);
}

void uploadData(double latitude, double longitude, uint8_t source_id) {//Upload Data to ThingSpeak
  // form the string for the API header parameter:

  Console.println("Call Linux Command to Send Data");
  Process p;    // Create a process and call it "p", this process will execute a Linux curl command
  
  p.begin("curl");
  
  p.addParameter("-X");
  p.addParameter("POST");
  p.addParameter("152.42.241.248:8000/occurrences");
  
  p.addParameter("-H");
  p.addParameter("accept: application/json");

  p.addParameter("-H");
  p.addParameter("Content-Type: application/json");

  p.addParameter("-d");
  p.addParameter("{\"buoy_id\": " + String(source_id)+ ", \"latitude\": " + String(latitude, 24) + ", \"longitude\": " + String(longitude, 24) + "}");
  
  p.run();    // Run the process and wait for its termination

  Console.print("Feedback from Linux: ");
  // If there's output from Linux,
  // send it out the Console:
  while (p.available()>0) 
  {
    char c = p.read();
    Console.write(c);
  }
  Console.println("");
  Console.println("Call Finished");
  Console.println("####################################");
  Console.println("");
}


void checkLora() {

  uint8_t message[RH_RF95_MAX_MESSAGE_LEN]; // Buffer to hold the incoming message
  uint8_t len = sizeof(message);


  if (rf95.recv(message, &len)) {
    // Successfully received a message
    message[len] = '\0'; 

    // Check if this is a LoRaMesh message
    bool isLoRaMesh = false;
    if (len >= 8) {
      // Check if message type byte (position 3) is valid LoRaMesh type (0x00-0x04)
      uint8_t msgType = message[3];
      if (msgType <= MESSAGE_TYPE_ACK && message[5] <= LORAMESH_MAX_HOPS) {
        isLoRaMesh = true;
      }
    }

    if (isLoRaMesh) {
      // Handle LoRaMesh protocol message
      uint8_t source_id, destination_id, messageType;
      uint8_t messageId = message[2]; // Extract message ID for ACK
      char payload[256];
      parseLoRaMeshMessage(message, len, &source_id, &destination_id, &messageType, payload);
      
      Console.print("LoRaMesh ");
      switch(messageType) {
        case MESSAGE_TYPE_DATA:
          Console.print("DATA");
          break;
        case MESSAGE_TYPE_ROUTE_REQUEST:
          Console.print("ROUTE_REQUEST");
          break;
        case MESSAGE_TYPE_ROUTE_REPLY:
          Console.print("ROUTE_REPLY");
          break;
        case MESSAGE_TYPE_ROUTE_FAILURE:
          Console.print("ROUTE_FAILURE");
          break;
        case MESSAGE_TYPE_ACK:
          Console.print("ACK");
          break;
        default:
          Console.print("UNKNOWN");
      }
      Console.print(" from node 0x");
      Console.print(source_id, HEX);
      Console.print(" to 0x");
      Console.println(destination_id, HEX);
      
      // Check if message is for gateway
      if (destination_id == buoyID || destination_id == LORAMESH_BROADCAST_ADDRESS) {
        switch(messageType) {
          case MESSAGE_TYPE_DATA:
            Console.print("Payload: ");
            Console.println(payload);
            
            // Parse JSON payload
            double lat, lon;
            uint8_t origin_id;
            parsePayload(payload, &lat, &lon, &origin_id);
            
            // Send acknowledgment
            sendLoRaMeshAck(source_id, messageId);
            Console.print("Sent LoRaMesh ACK to node 0x");
            Console.println(source_id, HEX);
            
            // Upload data if we have valid coordinates
            if (lat != 0 && lon != 0) {
              uploadData(lat, lon, origin_id);
            }
            break;
            
          case MESSAGE_TYPE_ROUTE_REQUEST:
            // Gateway doesn't need to respond to route requests as it's always reachable
            Console.println("Ignoring ROUTE_REQUEST at gateway");
            break;
            
          case MESSAGE_TYPE_ACK:
            Console.println("Received ACK");
            break;
            
          default:
            Console.println("Unhandled message type");
            break;
        }
      }
      return;
    }

    // Original protocol handling
    //parse message to get message type
    uint8_t type, reply, source_id, destination_id;
    char payload[100];
    parseMessage((char*)message, &type, &reply, &source_id, &destination_id, payload);

    //For demonstrating the relay function
    //you can set ignoreNodeID to the ID of the node you want the gateway to ignore
    //----------------------
    uint8_t ignoreNodeID = 140;
    if(source_id == ignoreNodeID){
      return;
    }
    //----------------------


    //check if message is meant for node(if destination ID is broadcast or matches own ID )
    if(destination_id == source_id || destination_id == buoyID){
      switch (type) {
      
      //Message received is broadcast message. Send shore coordinates
      case 0: // binary 00
        Serial.println("Type: 00");
        // Handle case 00
        

        //send coordinates as a reply
        char reply[RH_RF95_MAX_MESSAGE_LEN];            
        getBroadcastReply (reply, source_id, SHORE_LAT, SHORE_LONG);

        rf95.send(reply, strlen(reply)); // strlen(data));
        rf95.waitPacketSent();
        
        break;

      case 1: // binary 01
        Serial.println("Type: 01 -> Ignore replies if not requesting coordinates");
        
        break;


      case 2: // binary 10
        Serial.println("Type: 10 -> Node was chosen");

        //Send acknowledgement to source node
        char ack[RH_RF95_MAX_MESSAGE_LEN]; 
        getDynamiteAcknowledge (ack, source_id, buoyID);

        rf95.send((uint8_t *)ack, strlen(ack));
        rf95.waitPacketSent();

        Serial.print("Sent acknowledgement to node ");
        Serial.println(source_id);

      
        //Upload Data
        //Parse payload to get latitude, longitude, and ID of buoy that sent the message
        double lat, lon;
        uint8_t origin_id, destination_id;
        parsePayload(payload, &lat, &lon, &origin_id);
        uploadData(lat, lon, origin_id);

        break;

      case 3: // binary 11
        Serial.println("Type: 11 -> this shouldnt happen");
        break;

      default:
        Serial.print("Unknown type: ");
        Serial.println(type);
        break;
    }
    }

    
  }
}



void loop()
{

  checkLora();
  
}
