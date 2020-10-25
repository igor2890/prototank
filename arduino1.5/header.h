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

#define SERVANGLE_TOP 0x79 //10 для 50го, 15 для 51го
#define SERVANGLE_MID 0x137
#define SERVANGLE_BOTTOM 0x1EB //65 для 50го, 80 для 51го

//Array element numbers
#define COMMAND_TYPE 0
#define MOTORSPEED_OR_ID 1
#define MOTOR_GUN_TOWER 2

#define PCA9685_ADDRESS 0x40
#define PCA9685_SCALE 0x79 // = 25Mhz / (4096*50) - 1

//Register addresses PCA9685
#define PCA9685_REG_MODE1 0x00
#define PCA9685_REG_MODE2 0x01
#define PCA9685_REG_LED0_ON_L 0x06
#define PCA9685_REG_LED0_ON_H 0x07
#define PCA9685_REG_LED0_OFF_L 0x08
#define PCA9685_REG_LED0_OFF_H 0x09
#define PCA9685_REG_PRE_SCALE 0xFE

//bit masks PCA9685
#define SLEEP_ON 0x11
#define SLEEP_OFF 0x01

#define ID_TANK 50
#define SERIAL_SPEED 115200

union servo_angle_uniontype
{
  struct
  {
    uint8_t byteH;
    uint8_t byteL;
  } half;
  int full;
} servoAngle;

union speed_motor_uniontype
{
  struct
  {
    uint8_t byteLeftH : 4;
    uint8_t byteRightL : 4;
  } half;
  uint8_t full;
} speedMotorUnion;

union motor_gun_tower_uniontype
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