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

void loop()
{
    // 14.648696 121.068517 DCS
    UNO.write("{\"latitude\":14.648696,\"longitude\":121.068517}");

    Serial.print("{\"latitude\":14.648696,\"longitude\":121.068517}");

    delay(1000);
}
