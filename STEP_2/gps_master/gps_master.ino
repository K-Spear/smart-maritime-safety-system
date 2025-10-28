#include <SoftwareSerial.h>
SoftwareSerial GPS(2,3);   # b5 gps 용 블루투스

void setup() {
  GPS.begin(9600);
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){
    GPS.write(Serial.read());
  }
  if(GPS.available()){
    Serial.write(GPS.read());
  }
}
