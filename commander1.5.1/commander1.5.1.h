#ifndef COMMAND_H
#define COMMAND_H

bool readSerialToIncomingBuffer(uint8_t* arrayBuffer, int serialCommandLength);

void transformIncomingToCommandBuffer(uint8_t* arrayFrom, uint8_t* arrayTo, int serialCommandLength, int commandLength);

void commandProcessing(uint8_t* arrayCommand, int commandLength);

void controlGun();

void controlTower();

void controlMotor();

void shoot();

void motionFromHit();

void moveGunUp();

void moveGunDown();

void moveMotorLeftForward();

void moveMotorLeftBack();

void moveMotorRightForward();

void moveMotorRightBack();

void stopMotorLeft();

void stopMotorRight();

void setMotorLeftSpeed(int speedMot);

void setMotorRightSpeed(int speedMot);

void cleanCommandBuffer(uint8_t* arrayCom, int commandLength);

void cleanSerialBuffer();

void motionOnRecoil();

void needTurnTowerRight();

void needTurnTowerLeft();

void brakeAndStopTower();

void writeToI2c(int address, int reg, int byte);

ISR(TIMER1_OVF_vect);

#endif
