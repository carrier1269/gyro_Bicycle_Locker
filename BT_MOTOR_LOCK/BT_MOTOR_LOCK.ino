#include "stepmotor.h"
#include "motorbluetooth.h"

void setup() {
  //시리얼 모니터로 확인가능
  Serial.begin(115200);
  SerialBT.begin("ESP32test.ver2"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  motorPin();
}

void loop() {
  BLE();
}


  
