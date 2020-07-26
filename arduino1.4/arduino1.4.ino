#include <Servo.h>
#include <IRremote.h>

#define PINMOTOR_RIGHT_ENABLE 2 //едем не едем правая dvig1A0
#define PINMOTOR_RIGHT_BACK 11 //правая назад dvig1D7
#define PINMOTOR_RIGHT_FORWARD 4 //правая вперед  dvig1D8
#define PINMOTOR_RIGHT_PWM 5 //pwm правая dvig1D5

#define PINMOTOR_LEFT_ENABLE 9 //едем не едем левая dvig2A1
#define PINMOTOR_LEFT_FORWARD 8 //левая вперед dvig2D9
#define PINMOTOR_LEFT_BACK 7 //левая назад dvig2D4
#define PINMOTOR_LEFT_PWM 6 //pwm левая dvig2D6

#define PINSTEPPER_DIR 13
#define PINSTEPPER_STEP 12

#define SERVANGLE_TOP 10 //10 для 50го, 15 для 51го  
#define SERVANGLE_BOTTOM 65 //65 для 50го, 80 для 51го

//Array element numbers
#define COMMAND_TYPE 0
#define MOTORSPEED_OR_ID 1
#define MOTOR_GUN_TOWER 2


#define ID_TANK 50
#define SERIAL_SPEED 115200

Servo myservo;
IRsend irsend;

uint8_t commandBuffer[10] = {0};
int IDtank = ID_TANK;
//Tower movement
const int lagBetweenStepTower =  1900; //переменная скорости поворота башни в мк на 1 шаг (меньше - скорость выше)
const int startSpeedCorretion = 2000; //переменная настраивает стартовую скорость башни;
const int accelerationValueOfTower = startSpeedCorretion / 40; //переменная задает ускорение (в статичной реализации без синусы);
int antiStartSpeedCorrection = 0;
const int extremePositionOfTower = 1750; //ограничение максимального количества шагов поворота в одну сторону
int counterOfTowerSteps = 0;
int forMakeBiggerLagBetweenStepTower = 0;

int servoAngle = SERVANGLE_TOP; //угол сервы
unsigned long timePointOfLastServoMove;
unsigned long timePointOfLastTowerStep;
const int lagBetweenMoveGun = 50;
const char* response_TYPE = "900001#";
const char* response_OK = "888888#";
bool isCurrentTowerDirectionRight = false;

void setup()
{
  Serial.begin(SERIAL_SPEED);
  Serial.setTimeout (10);
  myservo.attach (10);
  myservo.write (servoAngle);

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

  delay (500);
}

void loop() 
{
while (1)
{
  if (Serial.available()) //корявое тут все, переделать
  { 
    int i = 0;
    while (commandBuffer[i] != '#')
    {
      controlTower ();
      if (Serial.available())
      {
        commandBuffer[i] = Serial.read();
        controlTower ();
        if (commandBuffer[i] == '#') 
          break;
        ++i;
      }
      controlTower ();
    }
    
    switch(commandBuffer[COMMAND_TYPE])
    {
      case 'T':
        delay (500);
        Serial.println (response_TYPE);
        break;
      case 'W':
        IDtank = commandBuffer[MOTORSPEED_OR_ID];
        delay (500);
        Serial.println (response_OK);
        cleanCommandBuffer ();
        break;
      case 'C':
        controlTower ();
        controlMotor ();
        break;
      case 'F':
        shoot ();
        break;
      case 'X':
        myservo.write (SERVANGLE_BOTTOM);
        motionFromHit ();
        cleanCommandBuffer ();
        break;
      default:
        cleanCommandBuffer ();
        break;
    }
  }
  controlTower ();
  controlGun ();
}
}

void controlGun ()
{
  int directionGun = (commandBuffer[MOTOR_GUN_TOWER] & 3);
  switch (directionGun)
  {
    case 1:
      moveGunDown ();
      break;
    case 2:
      moveGunUp ();
      break;
    default:
      break;
  }
}

void controlTower ()
{
  int directionTower = ((commandBuffer[MOTOR_GUN_TOWER] >> 2) & 3);
  switch (directionTower)
  {
    case 1:
      needTurnTowerRight ();
      break;
    case 2:
      needTurnTowerLeft ();
      break;
    default:
      brakeAndStopTower ();
      break;
  }
}

void controlMotor ()
{
  int motorCurrentLeftSpeed = (commandBuffer[MOTORSPEED_OR_ID] >> 4);
  motorCurrentLeftSpeed = map (motorCurrentLeftSpeed, 1, 15, 30, 80);

  int motorCurrentRightSpeed = (commandBuffer[MOTORSPEED_OR_ID] & 15);
  motorCurrentRightSpeed = map (motorCurrentRightSpeed, 1, 15, 30, 80);
  
  setMotorLeftSpeed (motorCurrentLeftSpeed);
  setMotorRightSpeed (motorCurrentRightSpeed);

  int directionLeftMotor = (commandBuffer[MOTOR_GUN_TOWER] >> 6);
  int directionRightMotor = ((commandBuffer[MOTOR_GUN_TOWER] >> 4 ) & 3 );

  switch (directionLeftMotor)
  {
    case 1:
      moveMotorLeftForward ();
      break;
    case 2:
      moveMotorLeftBack ();  
      break;
    default:
      stopMotorLeft ();
      break;
  }

  switch (directionRightMotor)
  {
    case 1:
      moveMotorRightForward ();  
      break;
    case 2:
      moveMotorRightBack ();
      break;
    default:
      stopMotorRight ();
      break;
  }
}

void shoot ()
{
  for (int i = 0; i < 3; i++)
  {
    irsend.sendSony(IDtank, 12);
    motionOnRecoil (i);
    delay(50);
  }
  cleanSerialBuffer ();
}

void motionFromHit ()
{
    stopMotorLeft ();
    stopMotorRight ();
    delay(500);

    moveMotorRightBack ();
    moveMotorLeftBack ();
    setMotorLeftSpeed (30);
    setMotorRightSpeed (30);
    delay (3000);

    stopMotorLeft ();
    stopMotorRight ();
    delay (1500);

    cleanSerialBuffer ();
}

void moveGunUp ()
{
  if (servoAngle > SERVANGLE_TOP && millis()- timePointOfLastServoMove > lagBetweenMoveGun)
  {
    --servoAngle;
    myservo.write (servoAngle);
    timePointOfLastServoMove = millis();
  }
}

void moveGunDown ()
{
  if (servoAngle < SERVANGLE_BOTTOM && millis()- timePointOfLastServoMove > lagBetweenMoveGun)
  {
    ++servoAngle;
    myservo.write (servoAngle);
    timePointOfLastServoMove = millis();
  }
}

void moveMotorLeftForward ()
{
  PORTB |= B00000001; //digitalWrite(PINMOTOR_LEFT_FORWARD, HIGH);
  PORTD &= ~B10000000; //digitalWrite(PINMOTOR_LEFT_BACK, LOW);
  PORTB |= B00000010; //digitalWrite(PINMOTOR_LEFT_ENABLE, HIGH);
}

void moveMotorLeftBack ()
{
  PORTB &= ~B00000001; //digitalWrite(PINMOTOR_LEFT_FORWARD, LOW);
  PORTD |= B10000000; //digitalWrite(PINMOTOR_LEFT_BACK, HIGH);  
  PORTB |= B00000010; //digitalWrite(PINMOTOR_LEFT_ENABLE, HIGH);
}

void moveMotorRightForward ()
{
  PORTB &= ~B00001000; //digitalWrite(PINMOTOR_RIGHT_BACK, LOW);
  PORTD |= B00010000; //digitalWrite(PINMOTOR_RIGHT_FORWARD, HIGH);
  PORTD |= B00000100; //digitalWrite(PINMOTOR_RIGHT_ENABLE, HIGH);
}

void moveMotorRightBack ()
{
  PORTB |= B00001000; //digitalWrite(PINMOTOR_RIGHT_BACK, HIGH);
  PORTD &= ~B00010000; //digitalWrite(PINMOTOR_RIGHT_FORWARD, LOW);
  PORTD |= B00000100; //digitalWrite(PINMOTOR_RIGHT_ENABLE, HIGH);
}

void stopMotorLeft ()
{
  PORTB &= ~B00000010; //digitalWrite(PINMOTOR_LEFT_ENABLE, LOW);
}

void stopMotorRight ()
{
  PORTD &= ~B00000100; //digitalWrite(PINMOTOR_RIGHT_ENABLE, LOW);
}

void setMotorLeftSpeed (int speed)
{
  analogWrite(PINMOTOR_LEFT_PWM, speed);
}

void setMotorRightSpeed (int speed)
{
  analogWrite(PINMOTOR_RIGHT_PWM, speed);
}

void cleanCommandBuffer ()
{
  memset( commandBuffer, 0, sizeof( commandBuffer ) );
}

void cleanSerialBuffer ()
{
  while (Serial.available() != 0)
  {
    Serial.read();
  }
}

void motionOnRecoil (int i)
{
  if ( !(digitalRead(PINMOTOR_RIGHT_ENABLE) ) && !(digitalRead(PINMOTOR_LEFT_ENABLE)) ) {
    switch (i){
      case (1):
        moveMotorRightBack ();
        moveMotorLeftBack ();
        break;
      case (2):
        moveMotorRightForward ();
        moveMotorLeftForward ();
        break;
      default:
        break;
    }
    setMotorLeftSpeed (60);
    setMotorRightSpeed (60);
    delay(50);
    stopMotorLeft ();
    stopMotorRight ();
  }
}

void needTurnTowerRight ()
{
  if ( !(counterOfTowerSteps >= extremePositionOfTower) && !(micros()- timePointOfLastTowerStep <= lagBetweenStepTower + forMakeBiggerLagBetweenStepTower) )
  {  
    PORTB |= B00100000; //digitalWrite (PINSTEPPER_DIR , HIGH);
    isCurrentTowerDirectionRight = true;
    makeStepTower ();
    ++counterOfTowerSteps;
    timePointOfLastTowerStep = micros();
    calculationUpCorrectionOfSpeedTower ();
  }
}

void needTurnTowerLeft ()
{
  if ( !(counterOfTowerSteps <= -(extremePositionOfTower)) && !(micros()- timePointOfLastTowerStep <= lagBetweenStepTower + forMakeBiggerLagBetweenStepTower) )
  {
    PORTB &= ~B00100000; //digitalWrite (PINSTEPPER_DIR , LOW);
    isCurrentTowerDirectionRight = false;
    makeStepTower ();
    --counterOfTowerSteps;
    timePointOfLastTowerStep = micros();
    calculationUpCorrectionOfSpeedTower ();
  }
}

void brakeAndStopTower ()
{
  if ( antiStartSpeedCorrection > 0 )
  {
    switch (isCurrentTowerDirectionRight)
    {
      case true:
        if ( !(counterOfTowerSteps >= extremePositionOfTower) && !(micros()- timePointOfLastTowerStep <= lagBetweenStepTower + forMakeBiggerLagBetweenStepTower) )
        {  
          makeStepTower ();
          ++counterOfTowerSteps;
          timePointOfLastTowerStep = micros();
          calculationDownCorrectionOfSpeedTower ();
        }
        break;
      default:
        if ( !(counterOfTowerSteps <= -(extremePositionOfTower)) && !(micros()- timePointOfLastTowerStep <= lagBetweenStepTower + forMakeBiggerLagBetweenStepTower) )
        {
          makeStepTower ();
          --counterOfTowerSteps;
          timePointOfLastTowerStep = micros();
          calculationDownCorrectionOfSpeedTower ();
        }
        break;
    }
  }
}

void makeStepTower ()
{
  PORTB |= B00010000; //digitalWrite (PINSTEPPER_STEP , HIGH);
  digitalWrite (PINSTEPPER_STEP , LOW);
}

void calculationUpCorrectionOfSpeedTower ()
{
  if (antiStartSpeedCorrection < startSpeedCorretion) 
  {
    antiStartSpeedCorrection += accelerationValueOfTower;
    forMakeBiggerLagBetweenStepTower = startSpeedCorretion - antiStartSpeedCorrection;
  }
}

void calculationDownCorrectionOfSpeedTower ()
{
  antiStartSpeedCorrection -= accelerationValueOfTower;
  forMakeBiggerLagBetweenStepTower = startSpeedCorretion - antiStartSpeedCorrection;
}






