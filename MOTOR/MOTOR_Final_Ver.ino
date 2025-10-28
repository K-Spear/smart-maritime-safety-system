#include <SoftwareSerial.h>

SoftwareSerial SerialReceive(10,11); // RX, TX


// 모터 아두이노 Slave

//Motor Pins

int C_FAN1 = 5; // 쿨링팬1 제어
int C_FAN2 = 6; // 쿨링팬2 제어
int INA = 12; // 팬모터 정방향
int INB = 13; // 팬모터 역방향

char data;

void setup() {
  SerialReceive.begin(9600);
  Serial.begin(9600);
  SerialReceive.listen();
  pinMode(C_FAN1, OUTPUT); //Motor 1
  pinMode(C_FAN2, OUTPUT); //Motor 2
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

}

void loop() {
  if(SerialReceive.available())
  {
    data = SerialReceive.read();
    Serial.println(data);
  }
  if (data == 'X')
  {
    digitalWrite(C_FAN1,LOW);
    digitalWrite(C_FAN2,LOW);

    digitalWrite(INA,HIGH);
    digitalWrite(INB,HIGH);
  }
  else if (data == 'O')
  {
    digitalWrite(C_FAN1,HIGH);
    digitalWrite(C_FAN2,HIGH);

    digitalWrite(INA,LOW);
    digitalWrite(INB,HIGH);
  }
  

}
