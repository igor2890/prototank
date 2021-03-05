#include <Arduino.h>

#include "config.h"
#include "global.h"

namespace Global
{
uint8_t incomingBuffer[10] = {0};
uint8_t commandBuffer[10] = {0};
bool error = false;

int IDtank = ID_TANK;

//Tower movement
int counterOfTowerSteps = 0;
float valueTowerTIM = 45000; //от 43 до 53 тыс оптимально

unsigned long timePointOfLastServoMove = 0;
bool isCurrentTowerDirectionRight = false;

servo_angle_uniontype servoAngle;

speed_motor_uniontype speedMotorUnion;

motor_gun_tower_uniontype motorGunTowerUnion;
}
