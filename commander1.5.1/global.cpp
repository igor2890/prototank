#include <Arduino.h>

#include "config.h"
#include "global.h"

namespace Global
{
uint8_t incomingBuffer[20] = {0};
uint8_t commandBuffer[LENGTH_COMMAND] = {0};

int IDtank = ID_TANK;

//Tower movement
int counterOfTowerSteps = 0;
float valueTowerTIM = 45000; //from 43000 to 53000 is good

unsigned long timePointOfLastServoMove = 0;
bool isCurrentTowerDirectionRight = false;

servo_angle_uniontype servoAngle;

speed_motor_uniontype speedMotorUnion;

motor_gun_tower_uniontype motorGunTowerUnion;
}
