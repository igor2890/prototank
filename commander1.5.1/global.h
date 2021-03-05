#ifndef GLOBAL_H
#define GLOBAL_H

namespace Global
{
extern uint8_t incomingBuffer[10];
extern uint8_t commandBuffer[10];
extern bool error;

extern int IDtank;

//Tower movement
extern int counterOfTowerSteps;
extern float valueTowerTIM;

extern unsigned long timePointOfLastServoMove;
extern bool isCurrentTowerDirectionRight;

extern union servo_angle_uniontype
{
  struct
  {
    uint8_t byteH;
    uint8_t byteL;
  } half;
  int full;
} servoAngle;

extern union speed_motor_uniontype
{
  struct
  {
    uint8_t byteLeftH : 4;
    uint8_t byteRightL : 4;
  } half;
  uint8_t full;
} speedMotorUnion;

extern union motor_gun_tower_uniontype
{
  struct
  {
    uint8_t moveMotorLeftBack : 1;
    uint8_t moveMotorLeftForward : 1;
    uint8_t moveMotorRightBack : 1;
    uint8_t moveMotorRightForward : 1;
    uint8_t moveTowerLeft : 1;
    uint8_t moveTowerRight : 1;
    uint8_t moveGunUp : 1;
    uint8_t moveGunDown : 1;
  } bitField;
  uint8_t full;
} motorGunTowerUnion;

}

#endif
