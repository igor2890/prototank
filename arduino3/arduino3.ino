#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9250.h>

const char* handshake_response = "900002#";
MPU9250 accelgyro;
I2Cdev   I2C_M;

void Handshake( const char* response ) {
  bool hasHandshake = false;
  while ( !hasHandshake )  
  {
    static char commandBuffer[8];
    if ( Serial.available( ) ) {  
      memset( commandBuffer, 0, sizeof( commandBuffer ) );
      Serial.readBytesUntil( '$', commandBuffer, 7 );   
      if ( commandBuffer[0] != '9' ) {
        continue;
      }        
      delay(100);
      Serial.println( response );
      hasHandshake = true;
    }
  }
};

const char* GetTelemetryString( ) {
  static char telemetryString[ 30 ];
  memset( telemetryString, 0, sizeof( telemetryString ) );

  static int16_t accelX,   accelY,   accelZ;
  static int16_t gyroX,    gyroY,    gyroZ;
  static int16_t compassX, compassY, compassZ;
  
  accelgyro.getMotion9( &accelX,   &accelY,   &accelZ, 
                        &gyroX,    &gyroY,    &gyroZ, 
                        &compassX, &compassY, &compassZ );
  
  sprintf( telemetryString, 
           "%d:%d:%d:%d:%d:%d#",
           accelX,accelY,accelZ,
           gyroX, gyroY, gyroZ );
    
  return  telemetryString; 
}




void setup()
{
  Wire.begin();
  Serial.begin(9600);
  accelgyro.initialize();
  delay (1000);  
}

void loop()
{
  static bool hasHandshake = false;
  if( !hasHandshake ) {
    Handshake( handshake_response ); 
    hasHandshake = true;
    delay( 100 );
  }
  
  Serial.println( GetTelemetryString( ) );
  delay(100);
}
