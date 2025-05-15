#include <ArduinoJson.h>
#include <math.h>

uint8_t node_id = 2;

double haversine(double lat1, double lon1){
  // Adapted from: https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
  // Set lat2 and lon2 to be the shore
  double lat2 = 14.648621;
  double lon2 = 121.068574;
  
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
  payload_json["id"] = node_id;

  // move JSON to the payload buffer
  serializeJson(payload_json, payload, buffer_size);
}

void getBroadcastMessage (char *message, double latitude, double longitude) {
  // create 16 bit header - 00(srcID)(destID)
  // 0 type = broadcast
  // 0 reply = no
  // 6 bits source ID = node_id
  // 6 bits destination ID = same as source since broadcast, can possibly set unique ID
  // 2 bit empty

  // 0 type 0 reply, set own ID as source and destination ID since broadcast
  uint16_t header = (node_id << 8) | (node_id << 2);

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
  header = header | (node_id << 8);

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
  // 6 bits source ID = node_id
  // 6 bits destination ID = destination_id
  // 2 bit empty

  // 1 type 0 reply, set own ID as source and destination ID as given
  uint16_t header = 32768 | (node_id << 8) | (destination_id << 2);

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
  // 6 bits source ID = node_id
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

  if (SPRESENSE.available()) {
    // Read data from Spresense
    char incomingByte = SPRESENSE.read();
    // Print received data to Arduino serial monitor for debugging
    Serial.print("Received: ");
    Serial.println(incomingByte);


    // get latitude and longitude from Spresense data
    uint8_t latitude;
    uint8_t longitude;

    //todo: parsing. spresense messages are going to be different from lora messages
    parseSpresenseMessage(latitude, longitude);

    //ASSUMPTION: All spresense data received in this function is a bomb message

    //create broadcast message
    char message[10];
    getBroadcastMessage(message, latitude, longitude);
    
    //send broadcast message
    rf95.send(message, 12); // strlen(data));
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
}


void checkLora() {

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; // Buffer to hold the incoming message
  uint8_t len = sizeof(buf);
  if (rf95.recv(buf, &len)) {
    // Successfully received a response
    buf[len] = '\0'; // Null-terminate if you want to treat it as a string
    String msg = (char*)buf;

    uint8_t type, reply, source_id, destination_id;
    char payload[100];
    parseMessage(msg, &type, &reply, &source_id, &destination_id, payload);

    switch (type) {
      case 0: // binary 00
        Serial.println("Type: 00");
        // Handle case 00
        break;

      case 1: // binary 01
        Serial.println("Type: 01");
        // Handle case 01
        break;

      case 2: // binary 10
        Serial.println("Type: 10");
        // Handle case 10
        break;

      case 3: // binary 11
        Serial.println("Type: 11");
        // Handle case 11
        break;

      default:
        Serial.print("Unknown type: ");
        Serial.println(type);
        break;
    }


    Serial.println("Received response: " + response);
  } else {
    Serial.println("Receive failed");
  }


}

void loop() {

  ///checks Spresense for any bomb messages
  checkSpresense();

  ///checks lora module for messages from other nodes
  checkLora();

}
void setup() {
  // put your setup code here, to run once:
  // Test code
  // Serial.begin(9600);
  // char message[100];
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

  // char message2[100];
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


}
