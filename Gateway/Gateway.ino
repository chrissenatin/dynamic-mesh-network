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

void parseLoRaMeshMessage(uint8_t *message, uint8_t len, uint8_t *source, uint8_t *destination, char *payload) {
  // LoRaMesh message format:
  // Byte 0: Destination
  // Byte 1: Source
  // Byte 2: Message ID
  // Byte 3: Message Type (0x00 for data)
  // Byte 4: Hop Count
  // Byte 5: Visited Count
  // Byte 6-N: Visited Nodes (variable length based on visited count)
  // Byte N+1: Next Hop
  // Byte N+2: Data Length
  // Byte N+3 onwards: Data
  
  if (len < 8) return; // Minimum header size
  
  *destination = message[0];
  *source = message[1];
  uint8_t messageType = message[3];
  uint8_t visitedCount = message[5];
  
  // Skip to data section
  uint8_t dataOffset = 6 + visitedCount + 2; // header + visited nodes + next hop + data length
  
  if (dataOffset >= len) return;
  
  uint8_t dataLen = message[dataOffset - 1];
  
  if (dataOffset + dataLen > len) return;
  
  // Copy the data payload
  memcpy(payload, &message[dataOffset], dataLen);
  payload[dataLen] = '\0';
}

void sendLoRaMeshAck(uint8_t destination) {
  // Send a simple acknowledgment in LoRaMesh format
  uint8_t ackMessage[20];
  
  // Build LoRaMesh header
  ackMessage[0] = destination;  // Destination
  ackMessage[1] = buoyID;       // Source (gateway)
  ackMessage[2] = random(255);  // Message ID
  ackMessage[3] = 0x00;         // Message Type (data)
  ackMessage[4] = 0;            // Hop Count
  ackMessage[5] = 0;            // Visited Count
  ackMessage[6] = destination;  // Next Hop (direct)
  ackMessage[7] = 4;            // Data Length
  
  // Simple ACK payload
  memcpy(&ackMessage[8], "ACK\0", 4);
  
  rf95.send(ackMessage, 12);
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
      // Check if message type byte (position 3) is valid LoRaMesh type (0x00-0x03)
      uint8_t msgType = message[3];
      if (msgType <= 0x03 && message[5] <= LORAMESH_MAX_HOPS) {
        isLoRaMesh = true;
      }
    }

    if (isLoRaMesh) {
      // Handle LoRaMesh protocol message
      uint8_t source_id, destination_id;
      char payload[256];
      parseLoRaMeshMessage(message, len, &source_id, &destination_id, payload);
      
      Console.print("LoRaMesh message from node ");
      Console.print(source_id);
      Console.print(" to ");
      Console.println(destination_id);
      Console.print("Payload: ");
      Console.println(payload);
      
      // Check if message is for gateway
      if (destination_id == buoyID || destination_id == LORAMESH_BROADCAST_ADDRESS) {
        // Parse JSON payload
        double lat, lon;
        uint8_t origin_id;
        parsePayload(payload, &lat, &lon, &origin_id);
        
        // Send acknowledgment
        sendLoRaMeshAck(source_id);
        Console.print("Sent LoRaMesh ACK to node ");
        Console.println(source_id);
        
        // Upload data
        uploadData(lat, lon, origin_id);
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
