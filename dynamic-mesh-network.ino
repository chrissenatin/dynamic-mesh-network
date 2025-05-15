#include <cstring>
#include <cstdint>

uint8_t node_id = 2

void createPayload(char *payload, double latitude, double longitude){
  // create payload as json string
  payload[0] = '\0';
  strcat(payload, "{");
  sprintf(payload+strlen(payload), "latitude:%f", latitude);
  strcat(payload, ",");
  sprintf(payload+strlen(payload), "longitude:%f", longitude);
  strcat(payload, "}");
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
  char payload[42];
  createPayload(payload, latitude, longitude);

  strcat(message, payload);
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
