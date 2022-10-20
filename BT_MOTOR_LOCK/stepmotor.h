#ifndef _STEPMOTOR_H_
#define _STEPMOTOR_H_

#include <Stepper.h>

#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

const int steps = 2048;

Stepper myStepper(steps, IN4, IN2, IN3, IN1);  // �Ƶ��̳� ���ܸ��ʹ� 200�� 1ȸ���ε� => ����3�� ���� �Ȱ���

void motor_go();
void motor_back();

void motorPin()
{
  myStepper.setSpeed(14);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

#endif
