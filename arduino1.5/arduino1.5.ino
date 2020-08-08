#include <Wire.h>
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

#define PCA9685_ADDRESS 0x40
#define PCA9685_SCALE 0x79 // = 25Mhz / (4096*50) - 1

//Register addreses PCA9685
#define PCA9685_REG_MODE1 0x00
#define PCA9685_REG_MODE2 0x01
#define PCA9685_REG_LED0_ON_L 0x06
#define PCA9685_REG_LED0_ON_H 0x07
#define PCA9685_REG_LED0_OFF_L 0x08
#define PCA9685_REG_LED0_OFF_H 0x09
#define PCA9685_REG_PRE_SCALE 0xFE //Writes to PRE_SCALE register are blocked when SLEEP bit is logic 0 (MODE 1)

//bit masks PCA9685
#define SLEEP_ON 0x11
#define SLEEP_OFF 0x01

#define ID_TANK 50
#define SERIAL_SPEED 115200

IRsend irsend;

uint8_t commandBuffer[10] = {0};
int IDtank = ID_TANK;
//Tower movement
const int extremePositionOfTower = 3500; //ограничение максимального количества шагов поворота в одну сторону 1750 * 2
int counterOfTowerSteps = 0;
float valueTowerTIM = 45000; //от 43 до 53

int servoAngle = SERVANGLE_TOP; //угол сервы
unsigned long timePointOfLastServoMove;
unsigned long timePointOfLastChangeSpeedTower;
const int lagBetweenMoveGun = 50;
const char* response_TYPE = "900001#";
const char* response_OK = "888888#";
bool isCurrentTowerDirectionRight = false;

void setup()
{
  Serial.begin(SERIAL_SPEED);
  Serial.setTimeout (10);
  Wire.begin();

//настраиваем прерывание timer1 для управления шаговиком башни
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = valueTowerTIM;
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

  //на старте прописать частоту в 3 отправки (сон, частота, пробуждение)
  //и стартовые значения ствола в 4 отправки (4 регистра)
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_MODE1 , SLEEP_ON); // сон
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_PRE_SCALE , SCALE); // частота
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_MODE1 , SLEEP_OFF); // пробуждение
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_ON_L , 0x00); // и 4 регистра
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_ON_H , 0x00);
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_OFF_L , 0x37);
  writeToI2c (PCA9685_ADDRESS , PCA9685_REG_LED0_OFF_H , 0x01);

  delay (500);


}

void loop() 
{
while (1)
{
  if (Serial.available())
  { 
    int i = 0;
    while (1)
    {
      if (Serial.available())
      {
        commandBuffer[i] = Serial.read();
        if (commandBuffer[i] == '#') 
          break;
        ++i;
      }
    }
    
    switch(commandBuffer[COMMAND_TYPE])
    {
      case 'T':
        brakeAndStopTower ();
        stopMotorLeft ();
        stopMotorRight ();
        delay (500);
        Serial.println (response_TYPE);
        cleanCommandBuffer ();
        break;
      case 'W':
        brakeAndStopTower ();
        stopMotorLeft ();
        stopMotorRight ();
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
        brakeAndStopTower ();
        shoot ();
        break;
      case 'X':
        brakeAndStopTower ();
        motionFromHit ();
        cleanCommandBuffer ();
        break;
      default:
        brakeAndStopTower ();
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

//544 мкс    1520 мкс    2400 мкс          20000мкс 4.882
//112        311         491                          4096
//servo_min = 0x70 , но по факту 0x79
//servo_mid = 0x137
//servo_max = 0x1EB

void moveGunUp () // переписать на i2c
{
  if (servoAngle > SERVANGLE_TOP && millis()- timePointOfLastServoMove > lagBetweenMoveGun)
  {
    --servoAngle;
    writeToI2c (PCA9685_ADDRESS , int reg , int byte);//недоделано
    writeToI2c (PCA9685_ADDRESS , int reg , int byte);
    timePointOfLastServoMove = millis();
  }
}

void moveGunDown () //переписать на i2c
{
  if (servoAngle < SERVANGLE_BOTTOM && millis()- timePointOfLastServoMove > lagBetweenMoveGun)
  {
    ++servoAngle;
    writeToI2c (PCA9685_ADDRESS , int reg , int byte);//недоделано
    writeToI2c (PCA9685_ADDRESS , int reg , int byte);
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
  if ( !(counterOfTowerSteps > extremePositionOfTower))
  {  
    PORTB |= B00100000; //digitalWrite (PINSTEPPER_DIR , HIGH);
    isCurrentTowerDirectionRight = true; //можно переписать на чтение значения бита
    TCCR1B |= (1 << CS10);
    if (valueTowerTIM < 53000)
    {
      valueTowerTIM += 50;
    }
    delay (5);
  }
  else
  {
    brakeAndStopTower ();
  }
}

void needTurnTowerLeft ()
{
  if ( !(counterOfTowerSteps < -(extremePositionOfTower)))
  {
    PORTB &= ~B00100000; //digitalWrite (PINSTEPPER_DIR , LOW);
    isCurrentTowerDirectionRight = false; //можно переписать на чтение значения бита
    TCCR1B |= (1 << CS10);
    if (valueTowerTIM < 53000)
    {
      valueTowerTIM += 50;
    }
    delay (5);
  }
  else
  {
    brakeAndStopTower ();
  }
}

void brakeAndStopTower ()
{
  TCCR1B &= ~(1 << CS10);
  valueTowerTIM = 46000;
  PORTB &= ~B00010000; //digitalWrite(PINSTEPPER_STEP, LOW);
}

void writeToI2c (int address , int reg , int byte)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
	Wire.write(byte);
	Wire.endTransmission();
}

//прерывание для вращения башней
ISR(TIMER1_OVF_vect)
{
  TCNT1 = valueTowerTIM;
  digitalWrite(PINSTEPPER_STEP, digitalRead(PINSTEPPER_STEP) ^ 1);
  if (isCurrentTowerDirectionRight)
  {
    ++counterOfTowerSteps;
  }
  else
  {
    --counterOfTowerSteps;
  }
}

//здесь переименовать все функции управления digitalWrite всеми пинами