//If you use Dragino IoT Mesh Firmware, uncomment below lines.
//For product: LG01. 
#define BAUDRATE 115200

#include <Console.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Process.h>

// Singleton instance of the radio driver
RH_RF95 rf95;

int led = A2;
float frequency = 915.0;


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
  JsonDocument payload_json;
  deserializeJson(payload_json, payload);
  
  *latitude = payload_json["latitude"];   // Will return 0 if none
  *longitude = payload_json["longitude"]; // Will return 0 if none
  *origin_id = payload_json["id"];
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
  p.addParameter("{\"buoy_id\": " + String(source_id)+ ", \"latitude\": " + String(latitude, 24) + ", \"longitude\": " String(longitude, 24) + "}");
  
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

  char message[RH_RF95_MAX_MESSAGE_LEN]; // Buffer to hold the incoming message
  uint8_t len = sizeof(message);


  if (rf95.recv(message, &len)) {
    // Successfully received a message
    message[len] = '\0'; 

    //parse message to get message type
    uint8_t type, reply, source_id, destination_id;
    char payload[100];
    parseMessage(message, &type, &reply, &source_id, &destination_id, payload);

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
    String response = (char *)message;
    Serial.println("Received response: " + response);
    }

    
  } else {
    String response = (char *)message;
    Serial.println("Received response: " + response);
    Serial.println("Receive Failed");
  }
}



void loop()
{

  checkLora();
  
}
