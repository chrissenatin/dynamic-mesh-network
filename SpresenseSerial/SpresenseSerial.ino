// char msg[6] = "Hello"; //String data

// void setup() {
//   Serial.begin(115200);
// }
// void loop() {
//   Serial.write(msg,6); //Write the serial data
//   Serial.println();
//   delay(1000);
// }
#include <SoftwareSerial.h>

const byte rxPin = 2;
const byte txPin = 3;
static byte ID = 1;

// Set up a new SoftwareSerial object   
SoftwareSerial UNO (rxPin, txPin);

void setup()
{
    Serial.begin(9600);
    UNO.begin(9600);
}

void checkArduino(){
    UNO.listen();
    int messageLength = UNO.available();
    char message[ARDUINO_MAX_MESSAGE_LEN];
    for(int ctr = 0; ctr < messageLength; ctr++){
    // Read data from Spresense
    message[ctr] = UNO.read();
    // Print received data to Arduino serial monitor for debugging
    Serial.print(message[ctr]);
  }
  Serial.println();
  message[messageLength] = '\0';
  if(message == "coord request"){
    UNO.write("{\"id\":145,\"latitude\":14.648696,\"longitude\":121.068517}");
    Serial.print("{\"id\":145,\"latitude\":14.648696,\"longitude\":121.068517}");

  } else {
    Serial.println("message from arduino not processed. message: ");
    Serial.println(message);
  }
}
void loop()
{

    checkArduino();
    // 14.648696 121.068517 DCS
    

    delay(1000);
}
