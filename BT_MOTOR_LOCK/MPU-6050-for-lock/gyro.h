#ifndef _GYRO_H_
#define _GYRO_H_

int buzzer = 23;
int ch  = 2;
double return_angle_X();
double return_angle_Y();
double return_angle_Z();
double angle_X = return_angle_X();
double angle_Y = return_angle_Y();
double angle_Z = return_angle_Z();

void wire()
{
  Serial.begin(9600);                                // 시리얼 통신 속도 조정

  Wire.begin();                                        // i2c통신 활성화
  Wire.setClock(400000);                               // i2c통신속도 설정 함수, 속도는 100000,400000밖에 없음

  Wire.beginTransmission(0x68);                        // 슬레이브역할을 맡고있는 6050의 레지스터인 0x68(주소값)과 통신을 시작한다는 의미
  Wire.write(0x6b);                                   
  Wire.write(0x0);
  Wire.endTransmission(true);

  ledcSetup(ch, 5000, 8);   // ch: PWM channel (0~15), freq: PWM 주파수, resolution: PWM  해상도
  ledcAttachPin(buzzer, ch);   // gpio : GPIO 핀 번호,  ch : PWM channel
  delay(5000);
}

void wirebegin()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3b);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)0x68,(uint8_t)14,true);

  int16_t AcXH =Wire.read();
  int16_t AcXL =Wire.read();
  int16_t AcYH =Wire.read();
  int16_t AcYL =Wire.read();
  int16_t AcZH =Wire.read();
  int16_t AcZL =Wire.read();
  int16_t TepH =Wire.read();
  int16_t TepL =Wire.read();
  int16_t GyXH =Wire.read();
  int16_t GyXL =Wire.read();
  int16_t GyYH =Wire.read();
  int16_t GyYL =Wire.read();
  int16_t GyZH =Wire.read();
  int16_t GyZL =Wire.read();   //센서가 읽어서 레지스터에 저장된 값들을 읽어와 각각 변수에 저장하는 문장들

  int16_t AcX = AcXH <<8 |AcXL;
  int16_t AcY = AcYH <<8 |AcYL;
  int16_t AcZ = AcZH <<8 |AcZL;
  int16_t GyX = GyXH <<8 |GyXL;
  int16_t GyY = GyYH <<8 |GyYL;
  int16_t GyZ = GyZH <<8 |GyZL;

  //MPU-6050보정하는 부분 시작
  static int32_t AcXSum =0, AcYSum =0, AcZSum=0; // 가속도센서의 각 축값들 1000번씩 더해서 저장할 변수선언. static활용하여 루프문 빠져나가도 변수값 유지
  static int32_t GyXSum =0, GyYSum =0, GyZSum=0; // 자이로센서                          "
  static double AcXOff =0, AcYOff =0, AcZOff=0;  
  static double GyXOff =0, GyYOff =0, GyZOff=0;  // 가속도, 자이로 센서의 평균값들 저장해 놓는 변수선언. 평균때문에 소수점 나올수 있어서 double로 선언

  static int cnt_sample =1000;  // 센서들의 평군값을 구하기 위해 센서값을 읽어올 횟수를 저장할 변수 선언 1000번 더해서 1000으로 초기화
  if(cnt_sample>0)
  {
    AcXSum += AcX; AcYSum +=AcY; AcZSum +=AcZ;
    GyXSum += GyX; GyYSum +=GyY; GyZSum +=GyZ; // 각축 센서들의 값을 계속 더해준다
    cnt_sample --;
    if(cnt_sample ==0)                        
    {
      AcXOff = AcXSum/1000.0;
      AcYOff = AcYSum/1000.0;
      AcZOff = AcZSum/1000.0;
      GyXOff = GyXSum/1000.0;
      GyYOff = GyYSum/1000.0;
      GyZOff = GyZSum/1000.0;  // 1000번 더한 값들을 1000으로 나누어 평균값을 Off변수들에 저장하기
    }
    delay(1);  
    return; // 각축의 평균값들을 구하지 못하면 67번째줄 실행X
  }

  double AcXD = AcX - AcXOff;  
  double AcYD = AcY - AcYOff;
  double AcZD = AcZ - AcZOff + 16384;

  double GyXD = GyX - GyXOff;
  double GyYD = GyY - GyYOff;
  double GyZD = GyZ - GyZOff;  // 각 센서 측정값에서 센서 오차값을 뺌으로서 센서의 오차를 0으로 만들어줌
  //MPU-6050보정하는 부분 끝
  
  //주기시간 계산하는 부분 시작
  static unsigned long t_prev =0;  // 바로 이전에 센서를 읽은 시간을 저장
  unsigned long t_now = micros();  // micros()함수를 호출해 센서값을 측정한 현재시간을 변수에 저장
  double dt =(t_now - t_prev)/1000000.0;  // 현재값에서 이전값을 빼고 1000000.0으로 나누어 초단위로 변환후 dt에 저장
  t_prev = t_now; // 현재시간이 이전시간으로 되고 두줄위에있는 센서값에서 현재시간을 다시 추출
  //주기시간: 0.500ms정도 나옴
  //주기시간 계산하는 부분 끝


  //자이로센서가 읽은 값들로 각도표시하는 부분 시작
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;  // ->따로 설명
  double GyXR = GyXD / GYROXYZ_TO_DEGREES_PER_SEC;
  double GyYR = GyYD / GYROXYZ_TO_DEGREES_PER_SEC;
  double GyZR = GyZD / GYROXYZ_TO_DEGREES_PER_SEC;  // 보정 자이로 값을 넣어 131로 나눈다음 각축의 변수에 저장한다.

  static double gyAngleX = 0.0, gyAngleY = 0.0, gyAngleZ =0.0; // 각도를 저장할 변수, static으로 넣어서 루프문 빠져나가도 값이 유지
  gyAngleX += GyXR*dt; 
  gyAngleY += GyYR*dt;
  gyAngleZ += GyZR*dt; //현재의 각도는 = 이전각도 + (각속도 x 시간)
  //자이로센서가 읽은 값들로 각도표시하는 부분 끝
  

  //가속도센서가 읽은 값들로 각도표시하는 부분 시작
  const float RADIANS_TO_DEGREES = 180/3.14159;  
  double AcYZD = sqrt(pow(AcY,2) + pow(AcZ,2));
  double AcXZD = sqrt(pow(AcX,2) + pow(AcZ,2));
  double acAngleY = atan(-AcXD/AcYZD)*RADIANS_TO_DEGREES;
  double acAngleX = atan(AcYD/AcXZD)*RADIANS_TO_DEGREES;
  double acAngleZ = 0;
  //가속도센서가 읽은 값들로 각도표시하는 부분 끝

  //상보필터(상호보완필터) 시작
  const double ALPHA = 0.96;  // 가속도 센서와 자이로센서의 값을 얼마나 반영할지의 비율. 자유롭게 조정가능
  static double cmAngleX = 0.0,cmAngleY = 0.0, cmAngleZ =0.0; //상보필터를 적용한 값들을 저장할 변수 선언
  cmAngleX = ALPHA*(cmAngleX+GyXR*dt)+(1.0-ALPHA)*acAngleX; 
  cmAngleY = ALPHA*(cmAngleY+GyYR*dt)+(1.0-ALPHA)*acAngleY; //상호보완필터의 핵심...문장... 이름은 거창하지만 코드는 한줄이누
  cmAngleZ = gyAngleZ; // z값은 자이로센서만 사용해서 오차가 발생할 수밖에 없을듯... 가속도센서로는 못구함 구하려면 지자기센서 들어가있는 MPU9250을 써야해
  
  
  if((angle_X - cmAngleX) > 20 || (angle_X - cmAngleX) < -20)
  {
    Serial.print("도난");
    ledcWrite(2, 100);
  }
  if((angle_Y - cmAngleY) > 20 && (angle_Y - cmAngleX) < -20)
  {
    Serial.print("도난");
    ledcWrite(2, 100);
  }
   if((angle_Z - cmAngleZ) > 20 && (angle_Z - cmAngleX) < -20)
  {
    Serial.print("도난");
    ledcWrite(2, 100);
  }
  
  Serial.printf(" | X = %6.1f" , cmAngleX);
  Serial.printf(" | X = %6.1f" , angle_X);
  Serial.printf(" | Y = %6.1f" , cmAngleY);
  Serial.printf(" | X = %6.1f" , angle_Y);
  Serial.printf(" | Z = %6.1f" , cmAngleZ);
  Serial.printf(" | X = %6.1f" , angle_Z);//상보필터로 나온 각도값 시리얼모니터에 출력하는 문장
  Serial.println();
}


double return_angle_X()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3b);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)0x68,(uint8_t)14,true);

  int16_t AcXH =Wire.read();
  int16_t AcXL =Wire.read();
  int16_t AcYH =Wire.read();
  int16_t AcYL =Wire.read();
  int16_t AcZH =Wire.read();
  int16_t AcZL =Wire.read();
  int16_t TepH =Wire.read();
  int16_t TepL =Wire.read();
  int16_t GyXH =Wire.read();
  int16_t GyXL =Wire.read();
  int16_t GyYH =Wire.read();
  int16_t GyYL =Wire.read();
  int16_t GyZH =Wire.read();
  int16_t GyZL =Wire.read();   //센서가 읽어서 레지스터에 저장된 값들을 읽어와 각각 변수에 저장하는 문장들

  int16_t AcX = AcXH <<8 |AcXL;
  int16_t AcY = AcYH <<8 |AcYL;
  int16_t AcZ = AcZH <<8 |AcZL;
  int16_t GyX = GyXH <<8 |GyXL;
  int16_t GyY = GyYH <<8 |GyYL;
  int16_t GyZ = GyZH <<8 |GyZL;

  //MPU-6050보정하는 부분 시작
  static int32_t AcXSum =0, AcYSum =0, AcZSum=0; // 가속도센서의 각 축값들 1000번씩 더해서 저장할 변수선언. static활용하여 루프문 빠져나가도 변수값 유지
  static int32_t GyXSum =0, GyYSum =0, GyZSum=0; // 자이로센서                          "
  static double AcXOff =0, AcYOff =0, AcZOff=0;  
  static double GyXOff =0, GyYOff =0, GyZOff=0;  // 가속도, 자이로 센서의 평균값들 저장해 놓는 변수선언. 평균때문에 소수점 나올수 있어서 double로 선언

  static int cnt_sample =1000;  // 센서들의 평군값을 구하기 위해 센서값을 읽어올 횟수를 저장할 변수 선언 1000번 더해서 1000으로 초기화
  if(cnt_sample>0)
  {
    AcXSum += AcX; AcYSum +=AcY; AcZSum +=AcZ;
    GyXSum += GyX; GyYSum +=GyY; GyZSum +=GyZ; // 각축 센서들의 값을 계속 더해준다
    cnt_sample --;
    if(cnt_sample ==0)                        
    {
      AcXOff = AcXSum/1000.0;
      AcYOff = AcYSum/1000.0;
      AcZOff = AcZSum/1000.0;
      GyXOff = GyXSum/1000.0;
      GyYOff = GyYSum/1000.0;
      GyZOff = GyZSum/1000.0;  // 1000번 더한 값들을 1000으로 나누어 평균값을 Off변수들에 저장하기
    }
    delay(1);  
    //return; // 각축의 평균값들을 구하지 못하면 67번째줄 실행X
  }

  double AcXD = AcX - AcXOff;  
  double AcYD = AcY - AcYOff;
  double AcZD = AcZ - AcZOff + 16384;

  double GyXD = GyX - GyXOff;
  double GyYD = GyY - GyYOff;
  double GyZD = GyZ - GyZOff;  // 각 센서 측정값에서 센서 오차값을 뺌으로서 센서의 오차를 0으로 만들어줌
  //MPU-6050보정하는 부분 끝
  
  //주기시간 계산하는 부분 시작
  static unsigned long t_prev =0;  // 바로 이전에 센서를 읽은 시간을 저장
  unsigned long t_now = micros();  // micros()함수를 호출해 센서값을 측정한 현재시간을 변수에 저장
  double dt =(t_now - t_prev)/1000000.0;  // 현재값에서 이전값을 빼고 1000000.0으로 나누어 초단위로 변환후 dt에 저장
  t_prev = t_now; // 현재시간이 이전시간으로 되고 두줄위에있는 센서값에서 현재시간을 다시 추출
  //주기시간: 0.500ms정도 나옴
  //주기시간 계산하는 부분 끝


  //자이로센서가 읽은 값들로 각도표시하는 부분 시작
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;  // ->따로 설명
  double GyXR = GyXD / GYROXYZ_TO_DEGREES_PER_SEC;
  double GyYR = GyYD / GYROXYZ_TO_DEGREES_PER_SEC;
  double GyZR = GyZD / GYROXYZ_TO_DEGREES_PER_SEC;  // 보정 자이로 값을 넣어 131로 나눈다음 각축의 변수에 저장한다.

  static double gyAngleX = 0.0, gyAngleY = 0.0, gyAngleZ =0.0; // 각도를 저장할 변수, static으로 넣어서 루프문 빠져나가도 값이 유지
  gyAngleX += GyXR*dt; 
  gyAngleY += GyYR*dt;
  gyAngleZ += GyZR*dt; //현재의 각도는 = 이전각도 + (각속도 x 시간)
  //자이로센서가 읽은 값들로 각도표시하는 부분 끝
  

  //가속도센서가 읽은 값들로 각도표시하는 부분 시작
  const float RADIANS_TO_DEGREES = 180/3.14159;  
  double AcYZD = sqrt(pow(AcY,2) + pow(AcZ,2));
  double AcXZD = sqrt(pow(AcX,2) + pow(AcZ,2));
  double acAngleY = atan(-AcXD/AcYZD)*RADIANS_TO_DEGREES;
  double acAngleX = atan(AcYD/AcXZD)*RADIANS_TO_DEGREES;
  double acAngleZ = 0;
  //가속도센서가 읽은 값들로 각도표시하는 부분 끝

  //상보필터(상호보완필터) 시작
  const double ALPHA = 0.96;  // 가속도 센서와 자이로센서의 값을 얼마나 반영할지의 비율. 자유롭게 조정가능
  static double cmAngleX = 0.0,cmAngleY = 0.0, cmAngleZ =0.0; //상보필터를 적용한 값들을 저장할 변수 선언
  cmAngleX = ALPHA*(cmAngleX+GyXR*dt)+(1.0-ALPHA)*acAngleX; 
  cmAngleY = ALPHA*(cmAngleY+GyYR*dt)+(1.0-ALPHA)*acAngleY; //상호보완필터의 핵심...문장... 이름은 거창하지만 코드는 한줄이누
  cmAngleZ = gyAngleZ; // z값은 자이로센서만 사용해서 오차가 발생할 수밖에 없을듯... 가속도센서로는 못구함 구하려면 지자기센서 들어가있는 MPU9250을 써야해

  Serial.printf(" | X = %6.1f" , cmAngleX);
  return cmAngleX;
}


double return_angle_Y()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3b);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)0x68,(uint8_t)14,true);

  int16_t AcXH =Wire.read();
  int16_t AcXL =Wire.read();
  int16_t AcYH =Wire.read();
  int16_t AcYL =Wire.read();
  int16_t AcZH =Wire.read();
  int16_t AcZL =Wire.read();
  int16_t TepH =Wire.read();
  int16_t TepL =Wire.read();
  int16_t GyXH =Wire.read();
  int16_t GyXL =Wire.read();
  int16_t GyYH =Wire.read();
  int16_t GyYL =Wire.read();
  int16_t GyZH =Wire.read();
  int16_t GyZL =Wire.read();   //센서가 읽어서 레지스터에 저장된 값들을 읽어와 각각 변수에 저장하는 문장들

  int16_t AcX = AcXH <<8 |AcXL;
  int16_t AcY = AcYH <<8 |AcYL;
  int16_t AcZ = AcZH <<8 |AcZL;
  int16_t GyX = GyXH <<8 |GyXL;
  int16_t GyY = GyYH <<8 |GyYL;
  int16_t GyZ = GyZH <<8 |GyZL;

  //MPU-6050보정하는 부분 시작
  static int32_t AcXSum =0, AcYSum =0, AcZSum=0; // 가속도센서의 각 축값들 1000번씩 더해서 저장할 변수선언. static활용하여 루프문 빠져나가도 변수값 유지
  static int32_t GyXSum =0, GyYSum =0, GyZSum=0; // 자이로센서                          "
  static double AcXOff =0, AcYOff =0, AcZOff=0;  
  static double GyXOff =0, GyYOff =0, GyZOff=0;  // 가속도, 자이로 센서의 평균값들 저장해 놓는 변수선언. 평균때문에 소수점 나올수 있어서 double로 선언

  static int cnt_sample =1000;  // 센서들의 평군값을 구하기 위해 센서값을 읽어올 횟수를 저장할 변수 선언 1000번 더해서 1000으로 초기화
  if(cnt_sample>0)
  {
    AcXSum += AcX; AcYSum +=AcY; AcZSum +=AcZ;
    GyXSum += GyX; GyYSum +=GyY; GyZSum +=GyZ; // 각축 센서들의 값을 계속 더해준다
    cnt_sample --;
    if(cnt_sample ==0)                        
    {
      AcXOff = AcXSum/1000.0;
      AcYOff = AcYSum/1000.0;
      AcZOff = AcZSum/1000.0;
      GyXOff = GyXSum/1000.0;
      GyYOff = GyYSum/1000.0;
      GyZOff = GyZSum/1000.0;  // 1000번 더한 값들을 1000으로 나누어 평균값을 Off변수들에 저장하기
    }
    delay(1);  
    //return; // 각축의 평균값들을 구하지 못하면 67번째줄 실행X
  }

  double AcXD = AcX - AcXOff;  
  double AcYD = AcY - AcYOff;
  double AcZD = AcZ - AcZOff + 16384;

  double GyXD = GyX - GyXOff;
  double GyYD = GyY - GyYOff;
  double GyZD = GyZ - GyZOff;  // 각 센서 측정값에서 센서 오차값을 뺌으로서 센서의 오차를 0으로 만들어줌
  //MPU-6050보정하는 부분 끝
  
  //주기시간 계산하는 부분 시작
  static unsigned long t_prev =0;  // 바로 이전에 센서를 읽은 시간을 저장
  unsigned long t_now = micros();  // micros()함수를 호출해 센서값을 측정한 현재시간을 변수에 저장
  double dt =(t_now - t_prev)/1000000.0;  // 현재값에서 이전값을 빼고 1000000.0으로 나누어 초단위로 변환후 dt에 저장
  t_prev = t_now; // 현재시간이 이전시간으로 되고 두줄위에있는 센서값에서 현재시간을 다시 추출
  //주기시간: 0.500ms정도 나옴
  //주기시간 계산하는 부분 끝


  //자이로센서가 읽은 값들로 각도표시하는 부분 시작
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;  // ->따로 설명
  double GyXR = GyXD / GYROXYZ_TO_DEGREES_PER_SEC;
  double GyYR = GyYD / GYROXYZ_TO_DEGREES_PER_SEC;
  double GyZR = GyZD / GYROXYZ_TO_DEGREES_PER_SEC;  // 보정 자이로 값을 넣어 131로 나눈다음 각축의 변수에 저장한다.

  static double gyAngleX = 0.0, gyAngleY = 0.0, gyAngleZ =0.0; // 각도를 저장할 변수, static으로 넣어서 루프문 빠져나가도 값이 유지
  gyAngleX += GyXR*dt; 
  gyAngleY += GyYR*dt;
  gyAngleZ += GyZR*dt; //현재의 각도는 = 이전각도 + (각속도 x 시간)
  //자이로센서가 읽은 값들로 각도표시하는 부분 끝
  

  //가속도센서가 읽은 값들로 각도표시하는 부분 시작
  const float RADIANS_TO_DEGREES = 180/3.14159;  
  double AcYZD = sqrt(pow(AcY,2) + pow(AcZ,2));
  double AcXZD = sqrt(pow(AcX,2) + pow(AcZ,2));
  double acAngleY = atan(-AcXD/AcYZD)*RADIANS_TO_DEGREES;
  double acAngleX = atan(AcYD/AcXZD)*RADIANS_TO_DEGREES;
  double acAngleZ = 0;
  //가속도센서가 읽은 값들로 각도표시하는 부분 끝

  //상보필터(상호보완필터) 시작
  const double ALPHA = 0.96;  // 가속도 센서와 자이로센서의 값을 얼마나 반영할지의 비율. 자유롭게 조정가능
  static double cmAngleX = 0.0,cmAngleY = 0.0, cmAngleZ =0.0; //상보필터를 적용한 값들을 저장할 변수 선언
  cmAngleX = ALPHA*(cmAngleX+GyXR*dt)+(1.0-ALPHA)*acAngleX; 
  cmAngleY = ALPHA*(cmAngleY+GyYR*dt)+(1.0-ALPHA)*acAngleY; //상호보완필터의 핵심...문장... 이름은 거창하지만 코드는 한줄이누
  cmAngleZ = gyAngleZ; // z값은 자이로센서만 사용해서 오차가 발생할 수밖에 없을듯... 가속도센서로는 못구함 구하려면 지자기센서 들어가있는 MPU9250을 써야해

  Serial.printf(" | X = %6.1f" , cmAngleX);
  return cmAngleY;
}

double return_angle_Z()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3b);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)0x68,(uint8_t)14,true);

  int16_t AcXH =Wire.read();
  int16_t AcXL =Wire.read();
  int16_t AcYH =Wire.read();
  int16_t AcYL =Wire.read();
  int16_t AcZH =Wire.read();
  int16_t AcZL =Wire.read();
  int16_t TepH =Wire.read();
  int16_t TepL =Wire.read();
  int16_t GyXH =Wire.read();
  int16_t GyXL =Wire.read();
  int16_t GyYH =Wire.read();
  int16_t GyYL =Wire.read();
  int16_t GyZH =Wire.read();
  int16_t GyZL =Wire.read();   //센서가 읽어서 레지스터에 저장된 값들을 읽어와 각각 변수에 저장하는 문장들

  int16_t AcX = AcXH <<8 |AcXL;
  int16_t AcY = AcYH <<8 |AcYL;
  int16_t AcZ = AcZH <<8 |AcZL;
  int16_t GyX = GyXH <<8 |GyXL;
  int16_t GyY = GyYH <<8 |GyYL;
  int16_t GyZ = GyZH <<8 |GyZL;

  //MPU-6050보정하는 부분 시작
  static int32_t AcXSum =0, AcYSum =0, AcZSum=0; // 가속도센서의 각 축값들 1000번씩 더해서 저장할 변수선언. static활용하여 루프문 빠져나가도 변수값 유지
  static int32_t GyXSum =0, GyYSum =0, GyZSum=0; // 자이로센서                          "
  static double AcXOff =0, AcYOff =0, AcZOff=0;  
  static double GyXOff =0, GyYOff =0, GyZOff=0;  // 가속도, 자이로 센서의 평균값들 저장해 놓는 변수선언. 평균때문에 소수점 나올수 있어서 double로 선언

  static int cnt_sample =1000;  // 센서들의 평군값을 구하기 위해 센서값을 읽어올 횟수를 저장할 변수 선언 1000번 더해서 1000으로 초기화
  if(cnt_sample>0)
  {
    AcXSum += AcX; AcYSum +=AcY; AcZSum +=AcZ;
    GyXSum += GyX; GyYSum +=GyY; GyZSum +=GyZ; // 각축 센서들의 값을 계속 더해준다
    cnt_sample --;
    if(cnt_sample ==0)                        
    {
      AcXOff = AcXSum/1000.0;
      AcYOff = AcYSum/1000.0;
      AcZOff = AcZSum/1000.0;
      GyXOff = GyXSum/1000.0;
      GyYOff = GyYSum/1000.0;
      GyZOff = GyZSum/1000.0;  // 1000번 더한 값들을 1000으로 나누어 평균값을 Off변수들에 저장하기
    }
    delay(1);  
    //return; // 각축의 평균값들을 구하지 못하면 67번째줄 실행X
  }

  double AcXD = AcX - AcXOff;  
  double AcYD = AcY - AcYOff;
  double AcZD = AcZ - AcZOff + 16384;

  double GyXD = GyX - GyXOff;
  double GyYD = GyY - GyYOff;
  double GyZD = GyZ - GyZOff;  // 각 센서 측정값에서 센서 오차값을 뺌으로서 센서의 오차를 0으로 만들어줌
  //MPU-6050보정하는 부분 끝
  
  //주기시간 계산하는 부분 시작
  static unsigned long t_prev =0;  // 바로 이전에 센서를 읽은 시간을 저장
  unsigned long t_now = micros();  // micros()함수를 호출해 센서값을 측정한 현재시간을 변수에 저장
  double dt =(t_now - t_prev)/1000000.0;  // 현재값에서 이전값을 빼고 1000000.0으로 나누어 초단위로 변환후 dt에 저장
  t_prev = t_now; // 현재시간이 이전시간으로 되고 두줄위에있는 센서값에서 현재시간을 다시 추출
  //주기시간: 0.500ms정도 나옴
  //주기시간 계산하는 부분 끝


  //자이로센서가 읽은 값들로 각도표시하는 부분 시작
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;  // ->따로 설명
  double GyXR = GyXD / GYROXYZ_TO_DEGREES_PER_SEC;
  double GyYR = GyYD / GYROXYZ_TO_DEGREES_PER_SEC;
  double GyZR = GyZD / GYROXYZ_TO_DEGREES_PER_SEC;  // 보정 자이로 값을 넣어 131로 나눈다음 각축의 변수에 저장한다.

  static double gyAngleX = 0.0, gyAngleY = 0.0, gyAngleZ =0.0; // 각도를 저장할 변수, static으로 넣어서 루프문 빠져나가도 값이 유지
  gyAngleX += GyXR*dt; 
  gyAngleY += GyYR*dt;
  gyAngleZ += GyZR*dt; //현재의 각도는 = 이전각도 + (각속도 x 시간)
  //자이로센서가 읽은 값들로 각도표시하는 부분 끝
  

  //가속도센서가 읽은 값들로 각도표시하는 부분 시작
  const float RADIANS_TO_DEGREES = 180/3.14159;  
  double AcYZD = sqrt(pow(AcY,2) + pow(AcZ,2));
  double AcXZD = sqrt(pow(AcX,2) + pow(AcZ,2));
  double acAngleY = atan(-AcXD/AcYZD)*RADIANS_TO_DEGREES;
  double acAngleX = atan(AcYD/AcXZD)*RADIANS_TO_DEGREES;
  double acAngleZ = 0;
  //가속도센서가 읽은 값들로 각도표시하는 부분 끝

  //상보필터(상호보완필터) 시작
  const double ALPHA = 0.96;  // 가속도 센서와 자이로센서의 값을 얼마나 반영할지의 비율. 자유롭게 조정가능
  static double cmAngleX = 0.0,cmAngleY = 0.0, cmAngleZ =0.0; //상보필터를 적용한 값들을 저장할 변수 선언
  cmAngleX = ALPHA*(cmAngleX+GyXR*dt)+(1.0-ALPHA)*acAngleX; 
  cmAngleY = ALPHA*(cmAngleY+GyYR*dt)+(1.0-ALPHA)*acAngleY; //상호보완필터의 핵심...문장... 이름은 거창하지만 코드는 한줄이누
  cmAngleZ = gyAngleZ; // z값은 자이로센서만 사용해서 오차가 발생할 수밖에 없을듯... 가속도센서로는 못구함 구하려면 지자기센서 들어가있는 MPU9250을 써야해

  Serial.printf(" | X = %6.1f" , cmAngleX);
  return cmAngleZ;
}

#endif
