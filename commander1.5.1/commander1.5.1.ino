#include <Wire.h>
#include <IRremote.h>

#include "commander1.5.1.h"
#include "config.h"
#include "global.h"

extern IRsend irsend;

void setup()
{
  Serial.begin(SERIAL_SPEED);
  Serial.setTimeout (10);
  Wire.begin();

//настраиваем прерывание timer1 для управления шаговиком башни
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = Global::valueTowerTIM;
  TIMSK1 |= (1 << TOIE1); 
  interrupts();

  pinMode(PINMOTOR_RIGHT_ENABLE, OUTPUT);   
  pinMode(PINMOTOR_RIGHT_BACK, OUTPUT);  
  pinMode(PINMOTOR_RIGHT_FORWARD, OUTPUT);   
  digitalWrite(PINMOTOR_RIGHT_ENABLE, LOW);
  
  pinMode(PINMOTOR_LEFT_ENABLE, OUTPUT);   
  pinMode(PINMOTOR_LEFT_FORWARD, OUTPUT);  
  pinMode(PINMOTOR_LEFT_BACK, OUTPUT);   
  digitalWrite(PINMOTOR_LEFT_ENABLE, LOW);
  
  pinMode(PINSTEPPER_DIR, OUTPUT);
  digitalWrite (PINSTEPPER_DIR , LOW);
  pinMode(PINSTEPPER_STEP, OUTPUT);
  digitalWrite (PINSTEPPER_STEP , LOW);

  Global::servoAngle.full = SERVANGLE_MID;
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_MODE1 , SLEEP_ON); // сон для выставления частоты
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_PRE_SCALE , PCA9685_SCALE); // частота
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_MODE1 , SLEEP_OFF); // пробуждение
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_ON_L , 0x00); // и 4 регистра
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_ON_H , 0x00);
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_OFF_L , Global::servoAngle.half.byteL);
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_OFF_H , Global::servoAngle.half.byteH);
  
  delay (500);
}

void loop()
{
while (1)
{
  if (Serial.available()) { 
    
    Global::error = readSerialToIncomingBuffer();
    if (!Global::error) {
      transformIncomingToCommandBuffer();
      commandProcessing();
    }
  }
  controlTower ();
  controlGun ();
}
}
