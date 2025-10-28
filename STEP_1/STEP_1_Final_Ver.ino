#include <SoftwareSerial.h>
SoftwareSerial SerialSend(11,10); // RX TX

// STEP 1 음주측정 Master 아두이노

int Alcohol_sensor = A0; // 알코올 센서핀
int MQ_LED = 8; // 알코올 센서 LED핀
int Tact = 12; // 택트스위치핀
int SW_LED = 13; // 스위치 LED핀
int State = 0; // 스위치 변수
int Val = 0; // 알코올 센서 값 저장변수
int sum = 0; // 알코올 센서 값 합계
int ave = 0; // 알코올 센서 값 평균값

//
const int readMax = 100; // 평균값을 구하기 위한 측정 횟수
int readings[readMax]; // 측정값 저장 배열변수
int readIndex = 0; // 측정된 값이 저장될 위치
//

//
const long interval = 100; // 0.1 초
unsigned long previousMillis = 0; 
//

void setup() {
  Serial.begin(9600); // 시리얼 통신속도 9600
  pinMode(Tact, INPUT); 
  pinMode(Alcohol_sensor, INPUT);
  pinMode(SW_LED, OUTPUT);
  pinMode(MQ_LED,OUTPUT);
  SerialSend.begin(9600); 
  
// 센서값을 100번 측정하여 저장함 
  for (int i = 0; i < readMax; i++){
    readings[i] = analogRead(Alcohol_sensor)/10;
    sum = sum + readings[i];
  }
//

}

void loop() {
  State = digitalRead(Tact); // 스위치의 상태
  delay(100);
  Serial.print("State : ");
  Serial.println(State);
  unsigned long currentMillis = millis(); 

  if (State == 1) // 스위치 ON
  {
    
    digitalWrite(SW_LED,HIGH); // 스위치 LED 점등
    
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      Val = analogRead(Alcohol_sensor)/10;
    }         // 0.1초 간격으로 알코올 센서 값을 측정
    
    // 100개의 센서 측정값을 0.1초 간격으로 업데이트 하여 평균값을 계산
    sum =  sum - readings[readIndex];
    readings[readIndex] = Val;
    sum = sum + readings[readIndex];
    readIndex = readIndex + 1;

    if (readIndex >= readMax)
    {
      readIndex = 0;
    }
    ave = sum / readMax;

    if ( ave >= 20) // 측정된 100개(0.1초간 100번)의 센서값 평균이 20이 넘을 경우 아래의 명령을 실행 
    {
      Serial.print("Average : ");
      Serial.println(ave);
      digitalWrite(MQ_LED, HIGH); // 알코올 LED 점등
      SerialSend.write('X'); // 모터 아두이노(Slave)에 X 전송
      State = 0;
    }
    else
    {
      Serial.print("Average : ");
      Serial.println(ave);
      digitalWrite(MQ_LED, LOW); // 알코올 LED 소등
      SerialSend.write('O'); // 모터 아두이노(Slave)에 O 전송
      State = 0;
    }
  }
  else // 스위치 OFF
  {
    digitalWrite(SW_LED, LOW); // 스위치 LED 소등
  }

}
