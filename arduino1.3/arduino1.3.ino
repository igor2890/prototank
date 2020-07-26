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
#define COMMAND_TYPE_AND_MOVE_LEFTMOTOR 0
#define LEFT_SPEED_DIGIT_10 1
#define LEFT_SPEED_DIGIT_1_AND_HIT_DETECT 2
#define MOVE_RIGHTMOTOR_AND_ID_100 3
#define RIGHT_SPEED_DIGIT_10_AND_ID_10 4
#define RIGHT_SPEED_DIGIT_1_AND_ID_1 5
#define MOVE_GUN 6
#define MOVE_TOWER 7
#define MAKE_SHOOT 8

#define ID_TANK 50
#define SERIAL_SPEED 115200

Servo myservo;
IRsend irsend;

char commandBuffer[50] = {0};
int IDtank = ID_TANK;
const int stepperSpeed =  1900; //переменная скорости поворота башни в мк на 1 шаг
const int stepperStopStep = 1750; //ограничение максимального количества шагов поворота в одну сторону
int stepperCurrentStep = 0;
int servoAngle = SERVANGLE_TOP; //угол сервы
unsigned long TimeServo;
unsigned long TimeStepper;
unsigned long TimeShoot;
unsigned long TimeDead;
const char* response_TYPE = "900001#";
const char* response_OK = "888888#";

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
  pinMode(PINSTEPPER_STEP, OUTPUT);
  digitalWrite (PINSTEPPER_STEP , LOW);

  delay (500);
}

void loop()
{
  if (Serial.available()) {
    int i = 0;
    while (commandBuffer[i] != '#') {
      controlStepper ();
      if (Serial.available()) {
        commandBuffer[i] = Serial.read();
        controlStepper ();
        if (commandBuffer[i] == '#') 
          break;
        ++i;
        controlStepper ();
      }
    }
  }

  switch(commandBuffer[COMMAND_TYPE_AND_MOVE_LEFTMOTOR]){
    case '9':
      delay (500);
      Serial.println (response_TYPE);
      cleanCommandBuffer ();
      break;
    case '8':
      {
        IDtank = calculateTankID();
        delay (500);
        Serial.println (response_OK);
        cleanCommandBuffer ();
      }
      break;
    case '0':
    case '1':
    case '2':
      controlTower ();
      controlMotor ();
      controlTower ();
      shoot ();
      break;
    case 'x':
      myservo.write (SERVANGLE_BOTTOM);
      motionFromHit ();
      break;
    default:
      cleanCommandBuffer ();
      break;
    }
  controlTower ();
  controlGun ();
}

void controlGun ()
{
  switch (commandBuffer[MOVE_GUN]){
    case '1':
      moveGunUp ();
      break;
    case '2':
      moveGunDown ();
      break;
    default:
      break;
  } 
}

void controlTower ()
{
  switch (commandBuffer[MOVE_TOWER])
  {
    case '1':
      needTurnTowerRight ();
      break;
    case '2':
      needTurnTowerLeft ();
      break;
    default:
      break;
  }
}

void controlMotor ()
{
  int digitSpeedRightDecade = commandBuffer[RIGHT_SPEED_DIGIT_10_AND_ID_10] - '0';        
  int digitSpeedRightOne = commandBuffer[RIGHT_SPEED_DIGIT_1_AND_ID_1] - '0';
  int motorCurrentRightSpeed = (digitSpeedRightDecade*10)+ digitSpeedRightOne;
  motorCurrentRightSpeed = map (motorCurrentRightSpeed, 1, 99, 30, 80);

  int digitSpeedLeftDecade = commandBuffer[LEFT_SPEED_DIGIT_10] - '0';        
  int digitSpeedLeftOne = commandBuffer[LEFT_SPEED_DIGIT_1_AND_HIT_DETECT] - '0';
  int motorCurrentLeftSpeed = (digitSpeedLeftDecade*10)+ digitSpeedLeftOne;
  motorCurrentLeftSpeed = map (motorCurrentLeftSpeed, 1, 99, 30, 80);
  
  setMotorLeftSpeed (motorCurrentLeftSpeed);
  setMotorRightSpeed (motorCurrentRightSpeed);

  switch (commandBuffer[MOVE_RIGHTMOTOR_AND_ID_100]){
    case '1':
      moveMotorRightForward ();  
      break;
    case '2':
      moveMotorRightBack ();
      break;
    default:
      stopMotorRight ();
      break;
  }

  switch (commandBuffer[COMMAND_TYPE_AND_MOVE_LEFTMOTOR]){
    case '1':
      moveMotorLeftForward ();
      break;
    case '2':
      moveMotorLeftBack ();  
      break;
    default:
      stopMotorLeft ();
      break;
  }

}

void shoot ()
{
  if ( commandBuffer[MAKE_SHOOT] == '1'){
    for (int i = 0; i < 3; i++) {
      irsend.sendSony(IDtank, 12);
      motionOnRecoil (i);
      delay(50);
    }
  commandBuffer[MAKE_SHOOT] = '0';
  cleanSerialBuffer ();
  }
}

int calculateTankID ()
{
  int firstDigit = commandBuffer[MOVE_RIGHTMOTOR_AND_ID_100] - '0';
  int secondDigit = commandBuffer[RIGHT_SPEED_DIGIT_10_AND_ID_10] - '0';
  int thirdDigit = commandBuffer[RIGHT_SPEED_DIGIT_1_AND_ID_1] - '0';
  int ID = (firstDigit*100)+(secondDigit*10)+thirdDigit;
  return ID;
}

void motionFromHit ()
{
  if ( commandBuffer[LEFT_SPEED_DIGIT_1_AND_HIT_DETECT] == 'y'){
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
}

void moveGunUp ()
{
  if (servoAngle > SERVANGLE_TOP && millis()- TimeServo > 50)
  {
    --servoAngle;
    myservo.write (servoAngle);
    TimeServo = millis();
  }
}

void moveGunDown ()
{
  if (servoAngle < SERVANGLE_BOTTOM && millis()- TimeServo > 50)
  {
    ++servoAngle;
    myservo.write (servoAngle);
    TimeServo = millis();
  }
}

void makeStepTower ()
{
  digitalWrite (PINSTEPPER_STEP , HIGH);
  digitalWrite (PINSTEPPER_STEP , LOW);
}

void moveMotorRightForward ()
{
  digitalWrite(PINMOTOR_RIGHT_BACK, LOW);
  digitalWrite(PINMOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(PINMOTOR_RIGHT_ENABLE, HIGH);
}

void moveMotorRightBack ()
{
  digitalWrite(PINMOTOR_RIGHT_BACK, HIGH);
  digitalWrite(PINMOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(PINMOTOR_RIGHT_ENABLE, HIGH);
}

void moveMotorLeftForward ()
{
  digitalWrite(PINMOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(PINMOTOR_LEFT_BACK, LOW);
  digitalWrite(PINMOTOR_LEFT_ENABLE, HIGH);
}

void moveMotorLeftBack ()
{
  digitalWrite(PINMOTOR_LEFT_FORWARD, LOW);
  digitalWrite(PINMOTOR_LEFT_BACK, HIGH);  
  digitalWrite(PINMOTOR_LEFT_ENABLE, HIGH);
}

void stopMotorRight ()
{
  digitalWrite(PINMOTOR_RIGHT_ENABLE, LOW);
}

void stopMotorLeft ()
{
  digitalWrite(PINMOTOR_LEFT_ENABLE, LOW);
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
  if ( commandBuffer[COMMAND_TYPE_AND_MOVE_LEFTMOTOR] == '0' && commandBuffer[MOVE_RIGHTMOTOR_AND_ID_100] == '0' ) {
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
  if ( !(stepperCurrentStep >= stepperStopStep) && !(micros()- TimeStepper <= stepperSpeed) )
  {  
    digitalWrite (PINSTEPPER_DIR , HIGH);
    makeStepTower ();
    ++stepperCurrentStep;
    TimeStepper = micros();
  }
}

void needTurnTowerLeft ()
{
  if ( !(stepperCurrentStep <= -(stepperStopStep)) && !(micros()- TimeStepper <= stepperSpeed) )
  {
    digitalWrite (PINSTEPPER_DIR , LOW);
    makeStepTower ();
    --stepperCurrentStep;
    TimeStepper = micros();
  }
}