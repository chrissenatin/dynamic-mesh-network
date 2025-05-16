#include <ArduinoJson.h>
#include <math.h>

#include <RH_RF95.h>
#include <SoftwareSerial.h>
#include <RHMesh.h>

// LoRa communication settings
#define RF95_CS 10
#define RF95_INT 2
#define RF95_RST 9

#define RF95_FREQ 915.0

#define SHORE_LAT 14.648696
#define SHORE_LONG 121.068517

RH_RF95 rf95(RF95_CS, RF95_INT);

uint8_t data[RH_RF95_MAX_MESSAGE_LEN] = "Hello World!";

uint8_t buoyID = 145; 

// Serial communication with Spresense
const byte rxPin = 4;
const byte txPin = 5;

SoftwareSerial SPRESENSE (rxPin, txPin);

double haversine(double lat1, double lon1){
  // Adapted from: https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
  // Set lat2 and lon2 to be the shore
  double lat2 = SHORE_LAT;
  double lon2 = SHORE_LONG;
  
  // Get distance between latitudes and longitudes
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) *  M_PI / 180.0;

  // convert to radians
  lat1 = (lat1) * M_PI / 180.0;
  lat2 = (lat2) * M_PI / 180.0;

  // apply formulae
  double a = pow(sin(dLat / 2), 2) +  pow(sin(dLon / 2), 2) *  cos(lat1) * cos(lat2);
  double rad = 6371.0;
  double c = 2 * asin(sqrt(a));
  return rad * c;
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

void getBroadcastMessage (char *message, double latitude, double longitude) {
  // create 16 bit header - 00(srcID)(destID)
  // 0 type = broadcast
  // 0 reply = no
  // 6 bits source ID = buoyID
  // 6 bits destination ID = same as source since broadcast, can possibly set unique ID
  // 2 bit empty

  // 0 type 0 reply, set own ID as source and destination ID since broadcast
  uint16_t header = (buoyID << 8) | (buoyID << 2);

  // convert to bitstring
  memcpy(message, &header, 2);
  message[2] = '\0';

  // empty payload since we are simply asking for coordinates
}

void getBroadcastReply (char *message, uint8_t destination_id, double latitude, double longitude) {
  // create 16 bit header
  // 0 type = broadcast
  // 1 reply = yes
  // 6 bits source ID
  // 6 bits destination ID
  // 2 bit empty

  // set reply to 1
  uint16_t header = 16384;

  // set source ID
  header = header | (buoyID << 8);

  // set destination ID
  header = header | (destination_id << 2);

  // convert to bitstring
  memcpy(message, &header, 2);
  message[2] = '\0';

  // create and concatenate payload to message
  char payload[48];
  createPayload(payload, 48, latitude, longitude);

  strcat(message, payload);
}

void getDynamiteMessage (char *message, uint8_t destination_id, double latitude, double longitude) {
  // create 16 bit header - 00(srcID)(destID)
  // 1 type = dynamite
  // 0 reply = no
  // 6 bits source ID = buoyID
  // 6 bits destination ID = destination_id
  // 2 bit empty

  // 1 type 0 reply, set own ID as source and destination ID as given
  uint16_t header = 32768 | (buoyID << 8) | (destination_id << 2);

  // convert to bitstring
  memcpy(message, &header, 2);
  message[2] = '\0';

  // create and concatenate payload to message
  char payload[48];
  createPayload(payload, 48, latitude, longitude);

  strcat(message, payload);
}

void getDynamiteAcknowledge (char *message, uint8_t destination_id, uint8_t origin_id) {
  // create 16 bit header - 00(srcID)(destID)
  // 1 type = dynamite
  // 1 reply = yes
  // 6 bits source ID = buoyID
  // 6 bits destination ID = destination_id
  // 2 bit empty

  // 1 type 1 reply, set own ID as source and destination ID as given
  uint16_t header = 49152 | (2 << 8) | (destination_id << 2);

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

void parseMessage (char *message, uint8_t *type, uint8_t *reply, uint8_t *source_id, uint8_t *destination_id, char *payload){
  // get header from the message and extract info
  uint16_t header = (message[1] << 8) | message[0] << 0;
  *type = (header & 32768) >> 15;
  *reply = (header & 16384) >> 14;
  *source_id = (header & 16128) >> 8;
  *destination_id = (header & 252) >> 2;

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

void checkSpresense(){
  SPRESENSE.listen();
  int messageLength = SPRESENSE.available();
  char message[RH_RF95_MAX_MESSAGE_LEN];
  for(int ctr = 0; ctr < messageLength; ctr++){
    // Read data from Spresense
    message[ctr] = SPRESENSE.read();
    // Print received data to Arduino serial monitor for debugging
    Serial.print(message[ctr]);
  }
  Serial.println();
  message[messageLength] = '\0';
  
  // get latitude and longitude from Spresense data
  double latitude, longitude;
  uint8_t id;
  parsePayload(message, &latitude, &longitude,&id);


  handshake(latitude, longitude);
}

void handshake(double latitude, double longitude){
  
    //ASSUMPTION: All spresense data received in this function is a bomb message

    //create broadcast message
    char message[10];
    getBroadcastMessage(message, latitude, longitude);
    
    //send broadcast message
    rf95.send(message, strlen(message)); // strlen(data);
    rf95.waitPacketSent();

      // --- Block and listen for 1 second ---
  
      double closestLat = 0.0, closestLon = 0.0;
      uint8_t closestNodeId = 0;
      double minDistance = 1e9; // arbitrarily large number

      unsigned long start = millis();
      while (millis() - start < 1000) {
        if (rf95.available()) {
          uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
          uint8_t len = sizeof(buf);
          if (rf95.recv(buf, &len)) {

            char *msg = (char *)buf;

            // Parse message
            uint8_t type, reply, source_id, destination_id;
            char payload[100];
            parseMessage(msg, &type, &reply, &source_id, &destination_id, payload);

            // Parse payload
            double lat, lon;
            uint8_t origin_id;
            parsePayload(payload, &lat, &lon, &origin_id);

            // Calculate distance from shore
            double dist = haversine(lat, lon);
            if (dist < minDistance) {
              minDistance = dist;
              closestLat = lat;
              closestLon = lon;
              closestNodeId = origin_id;
            }
          }
        }
      }

      

      // Send message to chosen node
      // 5. Send a message to the closest node
      if (minDistance < 1e9) {
        char finalMessage[100];
        getDynamiteMessage(finalMessage, closestNodeId, closestLat, closestLon);

        rf95.send((uint8_t *)finalMessage, strlen(finalMessage));
        rf95.waitPacketSent();

        Serial.print("Sent dynamite warning to node ");
        Serial.println(closestNodeId);
      } else {
        Serial.println("No valid responses received.");
      }
}

void getCoordinates(double *latitude, double *longitude){

  SPRESENSE.write("coord request");
  delay(100);

  SPRESENSE.listen();
  int messageLength = SPRESENSE.available();
  char message[RH_RF95_MAX_MESSAGE_LEN];
  for(int ctr = 0; ctr < messageLength; ctr++){
    // Read data from Spresense
    message[ctr] = SPRESENSE.read();
    // Print received data to Arduino serial monitor for debugging
    Serial.print(message[ctr]);
  }
  Serial.println();
  message[messageLength] = '\0';

   double lat, lon;
  uint8_t origin_id;
  parsePayload(payload, &lat, &lon, &origin_id);


  //todo: get current coordinates from spresense
  // 14.648696 121.068517 DCS
  *latitude = 14.648696;
  *longitude = 121.068517;
}

void checkLora() {

  char message[RH_RF95_MAX_MESSAGE_LEN]; // Buffer to hold the incoming message
  uint8_t len = sizeof(message);
  if (rf95.recv(message, &len)) {
    // Successfully received a message
    message[len] = '\0'; // Null-terminate if you want to treat it as a string

    uint8_t type, reply, source_id, destination_id;
    char payload[100];
    parseMessage(message, &type, &reply, &source_id, &destination_id, payload);

    switch (type) {
      case 0: // binary 00
        Serial.println("Type: 00");
        // Handle case 00
        
        double latitude, longitude;
        getCoordinates(&latitude, &longitude);

        //send coordinates as a reply
        char reply[RH_RF95_MAX_MESSAGE_LEN];            //destination is source of sender
        getBroadcastReply (reply, source_id, latitude, longitude);

        rf95.send(reply, strlen(reply)); // strlen(data));
        rf95.waitPacketSent();
        
        break;

      case 1: // binary 01
        Serial.println("Type: 01 -> Ignore replies if not requesting coordinates");
        
        break;

      case 2: // binary 10
        Serial.println("Type: 10 -> Node was chosen to be next sender");
         // Parse payload to get latitude and longitude
         // Spresense message is same format as other messages, except without id
        double lat, lon;
        uint8_t origin_id;
        parsePayload(payload, &lat, &lon, &origin_id);

        handshake(lat,lon);

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
  } else {
    Serial.println("Receive failed");
  }


}


void setup() {
  // 14.648696 121.068517 DCS
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(RF95_RST, OUTPUT);
  digitalWrite(RF95_RST, HIGH);
  delay(10);
  digitalWrite(RF95_RST, LOW);
  delay(10);
  digitalWrite(RF95_RST, HIGH);
  delay(10);
  rf95.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);
  SPRESENSE.begin(9600);

  
  
  // put your setup code here, to run once:
  // Test code
  // Serial.begin(9600);
  // char message[RH_RF95_MAX_MESSAGE_LEN];
  // getDynamiteMessage(message, 1, 4.3189, 121.12345);
  // Serial.println(message);
  
  // uint8_t type, reply, source, destination;
  // char payload[50];
  // parseMessage(message, &type, &reply, &source, &destination, payload);
  // Serial.println(type);
  // Serial.println(reply);
  // Serial.println(source);
  // Serial.println(destination);
  // Serial.println(payload);

  // double latitude, longitude;
  // uint8_t origin_id;
  // parsePayload (payload, &latitude, &longitude, &origin_id);
  // Serial.println(latitude,5);
  // Serial.println(longitude,5);
  // Serial.println(origin_id);

  // char message2[RH_RF95_MAX_MESSAGE_LEN];
  // getDynamiteAcknowledge(message2, 1, 1);
  // Serial.println(message2);
  
  // uint8_t type2, reply2, source2, destination2;
  // char payload2[50];
  // parseMessage(message2, &type2, &reply2, &source2, &destination2, payload2);
  // Serial.println(type2);
  // Serial.println(reply2);
  // Serial.println(source2);
  // Serial.println(destination2);
  // Serial.println(payload2);

  // double latitude2, longitude2;
  // uint8_t origin_id2;
  // parsePayload (payload2, &latitude2, &longitude2, &origin_id2);
  // Serial.println(latitude2,6);
  // Serial.println(longitude2,6);
  // Serial.println(origin_id2);
}

void loop() {
  checkSpresense();

  checkLora();


}
