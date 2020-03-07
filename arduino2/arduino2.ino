#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9250.h>
#include <IRremote.h>

#define RECV_PIN 11
#define VOLT_PIN A7

IRrecv irrecv(RECV_PIN);
decode_results results;

const char* response = "900002#";
char commandBuffer[15] = {0};
MPU9250 accelgyro;
I2Cdev   I2C_M;
bool hasHandshake = false;
unsigned long long int startTimeSend = 0;
unsigned long long int actualTimeSend = 0;
const int freqSend = 40;
unsigned long long int startTimeVolt = 0;
unsigned long long int actualTimeVolt = 0;
int freqVolt = 0;
int battVoltValue = 0;
int shooterTankID = 0;

int accelX_old = 0;
int accelY_old = 0;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial.setTimeout (10);
  irrecv.enableIRIn();
  accelgyro.initialize();
  GetVoltValue();
  freqVolt = 10000;
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
      Serial.print ( GetTelemetryString( ) );
      Serial.print (battVoltValue , DEC);
      Serial.print (":");
      Serial.println (shooterTankID , DEC);
      startTimeSend = millis();
      shooterTankID = 0;
    }
  }
}

const char* GetTelemetryString( ) {
  static char telemetryString[ 121 ];
  memset( telemetryString, 0, sizeof( telemetryString ) );

  static int16_t accelX,   accelY,   accelZ;
  static int16_t gyroX,    gyroY,    gyroZ;
  static int16_t compassX, compassY, compassZ;
  
  accelgyro.getMotion9( &accelX,   &accelY,   &accelZ, 
                        &gyroX,    &gyroY,    &gyroZ, 
                        &compassX, &compassY, &compassZ ); 
     // short option

           accelX = (accelX + accelX_old) / 2;
           accelY = (accelY + accelY_old) / 2;
     
  sprintf( telemetryString, 
           "%d:%d:%d:",
           accelX,accelY,0 );
           accelX_old = accelX;
           accelY_old = accelY;
           
     //full option
//sprintf( telemetryString, 
//         "%d:%d:%d:%d:%d:%d",
//         accelX,accelY,accelZ,
//         gyroX, gyroY, gyroZ );
    
  return  telemetryString; 
}
