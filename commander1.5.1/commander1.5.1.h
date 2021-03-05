#ifndef COMMAND_H
#define COMMAND_H

bool readSerialToIncomingBuffer();

void transformIncomingToCommandBuffer();

void commandProcessing();

void controlGun ();

void controlTower ();

void controlMotor ();

void shoot ();

void motionFromHit ();

void moveGunUp ();

void moveGunDown ();

void moveMotorLeftForward ();

void moveMotorLeftBack ();

void moveMotorRightForward ();

void moveMotorRightBack ();

void stopMotorLeft ();

void stopMotorRight ();

void setMotorLeftSpeed (int speed);

void setMotorRightSpeed (int speed);

void cleanCommandBuffer ();

void cleanSerialBuffer ();

void motionOnRecoil (int i);

void needTurnTowerRight ();

void needTurnTowerLeft ();

void brakeAndStopTower ();

void writeToI2c (int address , int reg , int byte);

ISR(TIMER1_OVF_vect);

#endif