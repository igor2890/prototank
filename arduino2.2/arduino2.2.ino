#include <Wire.h>
#include <IRremote.h>

#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_I2C_ADDRESS 0x68

#define RECV_PIN 11
#define VOLT_PIN A7

IRrecv irrecv(RECV_PIN);
decode_results results;

const char* response = "900002#";
char commandBuffer[15] = {0};

bool hasHandshake = false;
unsigned long long int startTimeSend = 0;
unsigned long long int actualTimeSend = 0;
const int freqSend = 40;
unsigned long long int startTimeVolt = 0;
unsigned long long int actualTimeVolt = 0;
int freqVolt = 0;
int battVoltValue = 0;
int shooterTankID = 0;

float angle_x_global = 0;
float angle_y_global = 0;
float angle_x_glob_start_corr = 0;
float angle_y_glob_start_corr = 0;


//_________________________________
//part of accel

typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};

unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}

float    base_x_accel;
float    base_y_accel;
float    base_z_accel;
float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;


int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;
  int error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));
  uint8_t swap;
  
  #define SWAP(x,y) swap = x; x = y; y = swap
  SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
  SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
  SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
  SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
  SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
  SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
  SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

  return error;
}

void calibrate_sensors() {
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;
 
  read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
  
  for (int i = 0; i < num_readings; i++) {
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
}

//accel part
//____________________________

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial.setTimeout (10);
  
  int error;
  uint8_t c;
  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  calibrate_sensors();  
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
  
  irrecv.enableIRIn();
  GetVoltValue();
  freqVolt = 10000;
  delay (2000);

  for (int ii = 0 ; ii  < 500 ; ++ii)
    {
    Take_Angle();
    delay (5);
    }
  angle_x_glob_start_corr = angle_x_global;
  angle_y_glob_start_corr = angle_y_global;
  delay (1000);  
}

void loop()
{
  GetCommand();   //Слушаем команды от малинки
  GetVoltValue(); //Раз в 10 секунд обновляем значение напряжения батареи
  CatchHit();     //Слушаем попадания в танк
  SendTele();     //Раз в 40 милисекунд (25 раз в секунду)
                  //отправляем телеметрию + напряжение батареи + id попавшего в нас танка
}

void GetCommand(){
 if ( Serial.available( ) ){  
   Serial.readBytesUntil( '$', commandBuffer, 10 );
   delay (200);
   int ee = commandBuffer[0] - '0';
   
   switch(ee){
     case 9:
      Serial.println ( response );
      delay (1000);
      hasHandshake = true;
      memset( commandBuffer, 0, sizeof( commandBuffer ) );
     break;
     default:
      hasHandshake = false;
     break;
   }
 }
}

void GetVoltValue (){
    if ((actualTimeVolt = millis()) - startTimeVolt >= freqVolt ){
      battVoltValue = (analogRead(VOLT_PIN) * 10) / 43;
      startTimeVolt = millis();
    }
}

void CatchHit(){
  if (irrecv.decode(&results)) {
    shooterTankID = results.value;
    irrecv.resume();
  }
}

void SendTele(){
  if (hasHandshake){
    if ((actualTimeSend = millis()) - startTimeSend >= freqSend ){
      Take_Angle();
      angle_x_global -= angle_x_glob_start_corr;
      angle_y_global -= angle_y_glob_start_corr;
       Serial.print(angle_x_global, 2);
       Serial.print(F(":"));
       Serial.print(angle_y_global, 2);
       Serial.print(F(":"));      
       Serial.print (battVoltValue , DEC);
       Serial.print (":");
       Serial.println (shooterTankID , DEC);
      startTimeSend = millis();
      shooterTankID = 0;
    }
  }
}

void Take_Angle ()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);
  unsigned long t_now = millis();
  float FS_SEL = 131;
  float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
  float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
  float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;
  float RADIANS_TO_DEGREES = 180/3.14159;
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_z = 0;

  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();
  
  float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
  
  float alpha = 0.96;
  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
  
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
  
  angle_x_global = angle_x;
  angle_y_global = angle_y;
}

int MPU6050_read (int start, uint8_t *buffer, int size)
{
  int i, n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);
  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);
  return (0);
}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-20);
  n = Wire.write(pData, size);
  if (n != size)
    return (-21);
  error = Wire.endTransmission(true);
  if (error != 0)
    return (error);
  return (0);
}

int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;
  error = MPU6050_write(reg, &data, 1);
  return (error);
}
