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

  payload_json["id"] = node_id;

  // move JSON to the payload buffer
  serializeJson(payload_json, payload, 11);

  strcat(message, payload);
}

void parseMessage (char *message, int *type, int *reply, int *source_id, int *destination_id, char *payload){
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

void parsePayload (char *payload, double *latitude, double *longitude, int *origin_id){
  // Extract payload contents
  JsonDocument payload_json;
  deserializeJson(payload_json, payload);
  
  *latitude = payload_json["latitude"];
  *longitude = payload_json["longitude"];
  *origin_id = payload_json["id"];
}

void setup() {
  // put your setup code here, to run once:
  // Test code
  // Serial.begin(9600);
  // char message[100];
  // getDynamiteMessage(message, 1, 4.3189, 121.12345);
  // Serial.println(message);
  
  // int type, reply, source, destination;
  // char payload[50];
  // parseMessage(message, &type, &reply, &source, &destination, payload);
  // Serial.println(type);
  // Serial.println(reply);
  // Serial.println(source);
  // Serial.println(destination);
  // Serial.println(payload);

  // double latitude, longitude;
  // int origin_id;
  // parsePayload (payload, &latitude, &longitude, &origin_id);
  // Serial.println(latitude);
  // Serial.println(longitude);
  // Serial.println(origin_id);
}

void loop() {
  // put your main code here, to run repeatedly:

}
