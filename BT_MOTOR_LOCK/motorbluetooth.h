#ifndef _MOTORBLUETOOTH_H_
#define _MOTORBLUETOOTH_H_

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

char bt;

BluetoothSerial SerialBT;

void BLE()
{
  if(SerialBT.available()){
    bt =SerialBT.read();

    Serial.println(bt);

    motor_go();
    motor_back();
  }
}

void motor_go()
{
    char drill = bt;
    if (drill == 'a')
    {
        myStepper.step(steps);
    }
    delay(20);
}
void motor_back()
{
    char backdrill = bt;
    if (backdrill == 'b')
    {
        myStepper.step(-steps);
    }
    delay(20);
}

#endif
