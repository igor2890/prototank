#include <AccelStepper.h>
#include <Servo.h>
#include <IRremote.h>

#define dvig1A0 2 //едем не едем правая
#define dvig1D7 11 //правая назад
#define dvig1D8 4 //правая вперед
#define dvig1D5 5 //pwm правая

#define dvig2A1 9 //едем не едем левая
#define dvig2D9 8 //левая вперед
#define dvig2D4 7 //левая назад
#define dvig2D6 6 //pwm левая

#define topStopL A2 //концевики поворота башни
#define topStopR A3

Servo myservo;
AccelStepper stepper (1, 12, 13); //(1, pinStep, pinDirection);
IRsend irsend;

char commandBuffer[15] = {0}; //создаем массив с нулями
int DVIGRonoff = 0;
int DVIGLonoff = 0;
int stepcmd = 0;  //рабочая переменная для поворота башни
int servocmd = 0; //рабочая переменная для подъема ствола
int IDtank = 0;     //переменная ID танка, который назначает сервер
int stepspeed = 150; //переменная скорости поворота башни
int servoAngle = 10; //угол сервы
unsigned long Time;
const char* response_TYPE = "900001#";
const char* response_OK = "888888#";

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout (10);
  myservo.attach (10);
  myservo.write (servoAngle);

  pinMode(dvig1A0, OUTPUT);   
  pinMode(dvig1D7, OUTPUT);  
  pinMode(dvig1D8, OUTPUT);   

  digitalWrite(dvig1A0, LOW);
  
  pinMode(dvig2A1, OUTPUT);   
  pinMode(dvig2D9, OUTPUT);  
  pinMode(dvig2D4, OUTPUT);   

  digitalWrite(dvig2A1, LOW);
  
  pinMode(topStopL, INPUT);
  pinMode(topStopR, INPUT);

stepper.setMaxSpeed(3000);
   stepper.setSpeed(stepspeed);
  delay (500);
}

void loop()
{
  if (Serial.available()) {        //если что-то пришло в буфер - читаем
  Serial.readBytesUntil('$', commandBuffer, 11 ); //читаем в массив b до тех пор, пока не увидим символ $ либо не считаем 9 символов
     int ee = commandBuffer[0] - '0';                  // переводим первый символ в число
     switch(ee){                  //если первый символ
       case 9:
        delay (500);
        Serial.println (response_TYPE);  //то отвечаем малинке, что мы ард.управленец
        memset( commandBuffer, 0, sizeof( commandBuffer ) );
        break;
       case 8:
       {
        int aa = commandBuffer[3] - '0';
        int bb = commandBuffer[4] - '0';
        int cc = commandBuffer[5] - '0';
        IDtank = (aa*100)+(bb*10)+cc;
        delay (500);
        Serial.println (response_OK);  //и отвечаем малинке, что мы получили id
        memset( commandBuffer, 0, sizeof( commandBuffer ) );
       }
        break;
       default:
        servocmd = commandBuffer[6] - '0';
        stepcmd = commandBuffer[7] - '0';
        shoot ();
        controlDC ();
        break;
     }
   }
   controlStepper ();
   controlServo ();
} 

void controlServo ()
{
  switch (servocmd){
    case 1:               //поднимаем ствол
      if (servoAngle > 10 && millis()- Time > 50){
        --servoAngle;
        myservo.write (servoAngle);
        Time = millis();
      }
      break;
    case 2:               // опускаем ствол
      if (servoAngle < 65 && millis()- Time > 50){
        ++servoAngle;
        myservo.write (servoAngle);
        Time = millis();
      }
      break;
    default:
      break;
  } 
}

void controlStepper ()
{
  switch (stepcmd){
    case 1:               //поворачиваем башню направо (отрицательное значение скорости)
      if (stepspeed > 0){
        stepspeed = -stepspeed;
        stepper.setSpeed(stepspeed);
      }
      if (analogRead(topStopR) < 300){ //проверяем нажатие стоп-кнопки
      //Serial.println("R");
      stepper.runSpeed();
      }
      break;
    case 2:               //поворачиваем башню налево (положительное значение скорости)
      if (stepspeed < 0){
        stepspeed = -stepspeed;
        stepper.setSpeed(stepspeed);
      }
      if (analogRead(topStopL) < 300){ //проверяем нажатие стоп-кнопки
      //Serial.println("L");
      stepper.runSpeed();
      }
      break;
    default:
      break;
  }
}

void controlDC ()
{
  DVIGRonoff = commandBuffer[3] - '0';
  int dvigR0 = commandBuffer[4] - '0';        
  int dvigR1 = commandBuffer[5] - '0';
  int dvigRight = (dvigR0*10)+ dvigR1;
  dvigRight = map (dvigRight, 1, 99, 30, 150);

  DVIGLonoff = commandBuffer[0] - '0';
  int dvigL0 = commandBuffer[1] - '0';        
  int dvigL1 = commandBuffer[2] - '0';
  int dvigLeft = (dvigL0*10)+ dvigL1;
  dvigLeft = map (dvigLeft, 1, 99, 30, 150);
  
  switch (DVIGRonoff){         // правая
    case 0:
      digitalWrite(dvig1A0, LOW);     //не крутим
      break;
    case 1:
      digitalWrite(dvig1A0, HIGH);     // крутим правую
      digitalWrite(dvig1D7, LOW);      // крутим правую вперед
      digitalWrite(dvig1D8, HIGH);  
      break;
    case 2:
      digitalWrite(dvig1A0, HIGH);    // крутим правую
      digitalWrite(dvig1D7, HIGH);    // крутим правую назад
      digitalWrite(dvig1D8, LOW);
      break;
    default:
      digitalWrite(dvig1A0, LOW);     //не крутим
      break;
  }

  switch (DVIGLonoff){        // левая
    case 0:
      digitalWrite(dvig2A1, LOW);     //не крутим
      break;
    case 1:
      digitalWrite(dvig2A1, HIGH);    // крутим левую
      digitalWrite(dvig2D9, HIGH);    // крутим левую вперед
      digitalWrite(dvig2D4, LOW); 
      break;
    case 2:
      digitalWrite(dvig2A1, HIGH);     // крутим левую
      digitalWrite(dvig2D9, LOW);      // крутим левую назад
      digitalWrite(dvig2D4, HIGH);  
      break;
    default:
      digitalWrite(dvig2A1, LOW);     //не крутим
      break;
  }
     analogWrite(dvig1D5, dvigRight);
     analogWrite(dvig2D6, dvigLeft);
}

void shoot ()
{
  if ( commandBuffer[8] == '1'){
    for (int i = 0; i < 3; i++) {
      irsend.sendSony(IDtank, 12);
      if ( DVIGLonoff == 0 && DVIGRonoff == 0 ) {
        switch (i){
          case (1):
            digitalWrite(dvig1D7, HIGH);    // крутим правую назад
            digitalWrite(dvig1D8, LOW);
            digitalWrite(dvig2D9, LOW);      // крутим левую назад
            digitalWrite(dvig2D4, HIGH);
            break;
          case (2):
            digitalWrite(dvig1D7, LOW);
            digitalWrite(dvig1D8, HIGH);
            digitalWrite(dvig2D9, HIGH);
            digitalWrite(dvig2D4, LOW);
            break;
          default:
            break;
        }
        digitalWrite(dvig1A0, HIGH);
        digitalWrite(dvig2A1, HIGH);
        analogWrite(dvig1D5, 60);
        analogWrite(dvig2D6, 60);
        delay(50);
        digitalWrite(dvig1A0, LOW);
        digitalWrite(dvig2A1, LOW);
      }
      delay(50);
    }
  commandBuffer[8] == '0';
  }
}
