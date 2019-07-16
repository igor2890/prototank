#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9250.h>
// По умолчанию адрес устройства на шине I2C - 0x68
MPU9250 accelgyro;
I2Cdev   I2C_M;
char b[10] = {0}; //создаем массив с нулями
int i = 1;
uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;
float heading;
float tiltheading;
float Axyz[3];
float Gxyz[3];
#define sample_num_mdate  5000

void setup()
{
    //подключаемся к шине I2C (I2Cdev не может сделать это самостоятельно)
    Wire.begin();
    Serial.begin(9600);
    accelgyro.initialize();
    delay(1000);
}
void loop()
{
  while (i == 1)  //цикл ожидания опроса от малинки для установления "личности" (ард.управленец или ард.телеметрия)
  {
    if (Serial.available()) {              //если что то пришло
      Serial.readBytesUntil('$', b, 7 );    //записываем полученное в массив
      int ii = b[0] - '0';                  // переводим первый символ в число
      switch(ii){                  //если первый символ
       case 9:                    //равен 9,
        delay (300);
  Serial.print ("900003#");  //то отвечаем малинке, что мы ард.телеметрия
        Serial.write('\n');
        i = 0;                    //и выключаем цикл while
        break;
       default:                  //если первый символ не равен 9
         break;                  //то цикл while повторяется, управление двигателями не начинается
      }
    b[0] = '0';                  //на всякий пожарный перезаписываем первый символ символом 0
    delay (100);
    }
   delay (10);
  }
  
    getAccel_Data();             // Получение значений Акселерометра
    getGyro_Data();              // Получение значений Гироскопа
    Serial.print(Axyz[0]);
    Serial.print(":");
    Serial.print(Axyz[1]);
    Serial.print(":");
    Serial.print(Axyz[2]);
    Serial.print(":");
    Serial.print(Gxyz[0]);
    Serial.print(":");
    Serial.print(Gxyz[1]);
    Serial.print(":");
    Serial.print(Gxyz[2]);
    Serial.println("#");
    delay(50);
}
void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}
void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}
