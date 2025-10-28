const int AOUTpin=A0;
const int AOUTpin1=A1;
const int AOUTpin2=A2;
const int AOUTpin3=A3;
const int AOUTpin4=A4;
const int AOUTpin5=A5;
const int ledPin=13;

#include <SoftwareSerial.h>
SoftwareSerial btSerial(7,8); // rx 8 tx 7

int value;
int value1;
int value2;
int value3;
int value4;
int value5;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  btSerial.begin(9600);
  
  }

void loop()
{ value= analogRead(AOUTpin);
  value1= analogRead(AOUTpin1);
  value2= analogRead(AOUTpin2);
  value3= analogRead(AOUTpin3);
  value4= analogRead(AOUTpin4);
  value5= analogRead(AOUTpin5);

  
if ((value+value1+value2+value3+value4+value5)/6 > 200){
  digitalWrite(ledPin, HIGH);
  btSerial.println("danger");
  btSerial.println((value+value1+value2+value3+value4+value5)/6);
  
  delay(1000);
  
  }else {
    digitalWrite(ledPin, LOW);
    }
}
