#include <Arduino.h>
#include <Wire.h>
#include <IRremote.h>


#include "config.h"
#include "constants.h"
#include "global.h"
#include "commander1.5.1.h"

IRsend irsend;

bool readSerialToIncomingBuffer(uint8_t* arrayBuffer, int serialCommandLength)
{
  int i = 0;
  while(1) {
    if (Serial.available()) {
      arrayBuffer[i] = Serial.read();
      if (arrayBuffer[i] == '#') {
        if (i == serialCommandLength - 1) {
          return false;
        }
        else {
          return true;
        }
      }
      ++i;
      if (i == serialCommandLength) {
        return true;
      }
    }
  }
}

void transformIncomingToCommandBuffer(uint8_t* arrayFrom, uint8_t* arrayTo, int serialCommandLength, int commandLength)
{
  if (serialCommandLength != commandLength) {
    //here need to make old commands transformation
  }
  else {
    for (int i = 0; i < commandLength-1; ++i) {
      arrayFrom[i] = arrayTo[i];
    }
  }
}

void commandProcessing(uint8_t* arrayCommand, int commandLength)
{
  switch(arrayCommand[COMMAND_TYPE]) {
    case 'T':
      brakeAndStopTower();
      stopMotorLeft();
      stopMotorRight();
      delay(500);
      Serial.println(Constants::response_TYPE);
      cleanCommandBuffer(arrayCommand, commandLength);
      break;
    case 'W':
      brakeAndStopTower();
      stopMotorLeft();
      stopMotorRight();
      Global::IDtank = arrayCommand[MOTORSPEED_OR_ID];
      delay(500);
      Serial.println(Constants::response_OK);
      cleanCommandBuffer(arrayCommand, commandLength);
      break;
    case 'C':
      Global::speedMotorUnion.full = arrayCommand[MOTORSPEED_OR_ID];
      Global::motorGunTowerUnion.full = arrayCommand[MOTOR_GUN_TOWER];
      controlTower();
      controlMotor();
      break;
    case 'F':
      brakeAndStopTower();
      shoot();
      break;
    case 'X':
      brakeAndStopTower();
      motionFromHit();
      cleanCommandBuffer(arrayCommand, commandLength);
      break;
    default:
      brakeAndStopTower();
      cleanCommandBuffer(arrayCommand, commandLength);
      break;
  }
}

void controlGun()
{
  if (Global::motorGunTowerUnion.bitField.moveGunDown) {
    moveGunDown();
  }
  else if (Global::motorGunTowerUnion.bitField.moveGunUp) {
    moveGunUp();
  }
}

void controlTower()
{
  if (Global::motorGunTowerUnion.bitField.moveTowerRight) {
    needTurnTowerRight();
  }
  else if (Global::motorGunTowerUnion.bitField.moveTowerLeft) {
    needTurnTowerLeft();
  }
  else {
    brakeAndStopTower();
  }
}

void controlMotor()
{
  int motorCurrentLeftSpeed = map(Global::speedMotorUnion.half.byteLeftH, 1, 15, 30, 80);
  int motorCurrentRightSpeed = map(Global::speedMotorUnion.half.byteRightL, 1, 15, 30, 80);
  
  setMotorLeftSpeed(motorCurrentLeftSpeed);
  setMotorRightSpeed(motorCurrentRightSpeed);

  if (Global::motorGunTowerUnion.bitField.moveMotorLeftForward) {
    moveMotorLeftForward();
  }
  else if (Global::motorGunTowerUnion.bitField.moveMotorLeftBack) {
    moveMotorLeftBack(); 
  }
  else {
    stopMotorLeft();
  }

  if (Global::motorGunTowerUnion.bitField.moveMotorRightForward) {
    moveMotorRightForward();
  }
  else if (Global::motorGunTowerUnion.bitField.moveMotorRightBack) {
    moveMotorRightBack();
  }
  else {
    stopMotorRight();
  }
}

void shoot()
{
  irsend.sendSony(Global::IDtank, 12);
  motionOnRecoil();
  cleanSerialBuffer();
}

void motionFromHit()
{
    stopMotorLeft();
    stopMotorRight();
    delay(500);

    moveMotorRightBack();
    moveMotorLeftBack();
    setMotorLeftSpeed(30);
    setMotorRightSpeed(30);
    delay(3000);

    stopMotorLeft();
    stopMotorRight();
    delay(1500);

    cleanSerialBuffer();
}

//544 мкс    1520 мкс    2400 мкс          20000мкс 4.882
//112        311         491               4096
//servo_min = 0x70 , но по факту 0x79
//servo_mid = 0x137
//servo_max = 0x1EB

void moveGunUp()
{
  if (Global::servoAngle.full > SERVANGLE_TOP && millis()- Global::timePointOfLastServoMove > Constants::lagBetweenMoveGun) {
    Global::servoAngle.full -= 5;
    writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_OFF_L , Global::servoAngle.half.byteL);
    writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_OFF_H , Global::servoAngle.half.byteH);
    Global::timePointOfLastServoMove = millis();
  }
}

void moveGunDown()
{
  if (Global::servoAngle.full < SERVANGLE_BOTTOM && millis()- Global::timePointOfLastServoMove > Constants::lagBetweenMoveGun) {
    Global::servoAngle.full += 5;
    writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_OFF_L , Global::servoAngle.half.byteL);
    writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_OFF_H , Global::servoAngle.half.byteH);
    Global::timePointOfLastServoMove = millis();
  }
}

void moveMotorLeftForward()
{
  PORTB |= B00000001; //digitalWrite(PINMOTOR_LEFT_FORWARD, HIGH);
  PORTD &= ~B10000000; //digitalWrite(PINMOTOR_LEFT_BACK, LOW);
  PORTB |= B00000010; //digitalWrite(PINMOTOR_LEFT_ENABLE, HIGH);
}

void moveMotorLeftBack()
{
  PORTB &= ~B00000001; //digitalWrite(PINMOTOR_LEFT_FORWARD, LOW);
  PORTD |= B10000000; //digitalWrite(PINMOTOR_LEFT_BACK, HIGH);  
  PORTB |= B00000010; //digitalWrite(PINMOTOR_LEFT_ENABLE, HIGH);
}

void moveMotorRightForward()
{
  PORTB &= ~B00001000; //digitalWrite(PINMOTOR_RIGHT_BACK, LOW);
  PORTD |= B00010000; //digitalWrite(PINMOTOR_RIGHT_FORWARD, HIGH);
  PORTD |= B00000100; //digitalWrite(PINMOTOR_RIGHT_ENABLE, HIGH);
}

void moveMotorRightBack()
{
  PORTB |= B00001000; //digitalWrite(PINMOTOR_RIGHT_BACK, HIGH);
  PORTD &= ~B00010000; //digitalWrite(PINMOTOR_RIGHT_FORWARD, LOW);
  PORTD |= B00000100; //digitalWrite(PINMOTOR_RIGHT_ENABLE, HIGH);
}

void stopMotorLeft()
{
  PORTB &= ~B00000010; //digitalWrite(PINMOTOR_LEFT_ENABLE, LOW);
}

void stopMotorRight()
{
  PORTD &= ~B00000100; //digitalWrite(PINMOTOR_RIGHT_ENABLE, LOW);
}

void setMotorLeftSpeed(int speedMot)
{
  analogWrite(PINMOTOR_LEFT_PWM, speedMot);
}

void setMotorRightSpeed(int speedMot)
{
  analogWrite(PINMOTOR_RIGHT_PWM, speedMot);
}

void cleanCommandBuffer(uint8_t* arrayCom, int commandLength)
{
  for (int i = 0; i < commandLength; ++i) {
    arrayCom[i] = 0;
  }
}

void cleanSerialBuffer()
{
  while (Serial.available() != 0) {
    Serial.read();
  }
}

void motionOnRecoil()
{
  if ( !(digitalRead(PINMOTOR_RIGHT_ENABLE)) && !(digitalRead(PINMOTOR_LEFT_ENABLE)) ) {
        moveMotorRightBack();
        moveMotorLeftBack();
    setMotorLeftSpeed(60);
    setMotorRightSpeed(60);
    delay(50);
    stopMotorLeft();
    stopMotorRight();
    delay(10);
        moveMotorRightForward();
        moveMotorLeftForward();
    setMotorLeftSpeed(60);
    setMotorRightSpeed(60);
    delay(50);
    stopMotorLeft();
    stopMotorRight();
  }
}

void needTurnTowerRight()
{
  if ( !(Global::counterOfTowerSteps > Constants::extremePositionOfTower)) {  
    PORTB |= B00100000; //digitalWrite (PINSTEPPER_DIR , HIGH);
    Global::isCurrentTowerDirectionRight = true; //можно переписать на чтение значения бита
    TCCR1B |= (1 << CS10);
    if (Global::valueTowerTIM < 53000) {
      Global::valueTowerTIM += 50;
    }
    delay (5);
  }
  else {
    brakeAndStopTower();
  }
}

void needTurnTowerLeft()
{
  if ( !(Global::counterOfTowerSteps < -(Constants::extremePositionOfTower))) {
    PORTB &= ~B00100000; //digitalWrite (PINSTEPPER_DIR , LOW);
    Global::isCurrentTowerDirectionRight = false; //можно переписать на чтение значения бита
    TCCR1B |= (1 << CS10);
    if (Global::valueTowerTIM < 53000) {
      Global::valueTowerTIM += 50;
    }
    delay (5);
  }
  else {
    brakeAndStopTower();
  }
}

void brakeAndStopTower()
{
  TCCR1B &= ~(1 << CS10);
  Global::valueTowerTIM = 46000;
  PORTB &= ~B00010000; //digitalWrite(PINSTEPPER_STEP, LOW);
}

void writeToI2c(int address, int reg, int byte)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
	Wire.write(byte);
	Wire.endTransmission();
}

//прерывание для вращения башней
ISR(TIMER1_OVF_vect)
{
  TCNT1 = Global::valueTowerTIM;
  digitalWrite(PINSTEPPER_STEP, digitalRead(PINSTEPPER_STEP) ^ 1);
  if (Global::isCurrentTowerDirectionRight) {
    ++Global::counterOfTowerSteps;
  }
  else {
    --Global::counterOfTowerSteps;
  }
}
