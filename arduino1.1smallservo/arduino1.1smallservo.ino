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

#define stepperDirPin 13
#define stepperStepPin 12

#define STARTSERVANGLE 15 //10 для 50го
#define LOWSERVANGLE 80 //65 для 50го

Servo myservo;
IRsend irsend;

char commandBuffer[15] = {0}; //создаем массив с нулями
int DVIGRonoff = 0;
int DVIGLonoff = 0;
int stepcmd = 0;  //рабочая переменная для поворота башни
int servocmd = 0; //рабочая переменная для подъема ствола
int IDtank = 0;     //переменная ID танка, который назначает сервер
const int stepperSpeed =  1900; //переменная скорости поворота башни в мк на 1 шаг
const int stepperStopStep = 1750; //ограничение максимального количества шагов поворота в одну сторону
int stepperCurrentStep = 0;
int servoAngle = STARTSERVANGLE; //угол сервы
unsigned long TimeServo;
unsigned long TimeStepper;
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
  
  pinMode(stepperDirPin, OUTPUT);
  pinMode(stepperStepPin, OUTPUT);
  digitalWrite (stepperStepPin , LOW);

  delay (500);
}

void loop()
{
  if (Serial.available()) {        //если что-то пришло в буфер - читаем
  Serial.readBytesUntil('$', commandBuffer, 11 ); //читаем в массив b до тех пор, пока не увидим символ $ либо не считаем 9 символов
     switch(commandBuffer[0]){                  //если первый символ
       case '9':
        delay (500);
        Serial.println (response_TYPE);  //то отвечаем малинке, что мы ард.управленец
        memset( commandBuffer, 0, sizeof( commandBuffer ) );
        break;
       case '8':
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
      case 'x':
        deaddance ();
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
      if (servoAngle > STARTSERVANGLE && millis()- TimeServo > 50){
        --servoAngle;
        myservo.write (servoAngle);
        TimeServo = millis();
      }
      break;
    case 2:               // опускаем ствол
      if (servoAngle < LOWSERVANGLE && millis()- TimeServo > 50){
        ++servoAngle;
        myservo.write (servoAngle);
        TimeServo = millis();
      }
      break;
    default:
      break;
  } 
}

int controlStepper ()
{
  switch (stepcmd){
    case 1:               //поворачиваем башню направо (отрицательное значение скорости)
      digitalWrite (stepperDirPin , HIGH);
      if ( (stepperCurrentStep >= stepperStopStep) || (micros()- TimeStepper <= stepperSpeed) )
        return 0;
      digitalWrite (stepperStepPin , HIGH);
      digitalWrite (stepperStepPin , LOW);
      ++stepperCurrentStep;
      TimeStepper = micros();
      break;
    case 2:               //поворачиваем башню налево (положительное значение скорости)
      digitalWrite (stepperDirPin , LOW);
      if ( (stepperCurrentStep <= -(stepperStopStep)) || (micros()- TimeStepper <= stepperSpeed) )
        return 0;
      digitalWrite (stepperStepPin , HIGH);
      digitalWrite (stepperStepPin , LOW);
      --stepperCurrentStep;
      TimeStepper = micros();
      break;
    default:
      return 0;
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

void deaddance ()
{
  if ( commandBuffer[2] == 'y'){
      digitalWrite(dvig1A0, LOW);
      digitalWrite(dvig2A1, LOW);
      myservo.write (LOWSERVANGLE);
      delay(500);
      digitalWrite(dvig1D7, HIGH);    // крутим правую назад
      digitalWrite(dvig1D8, LOW);
      digitalWrite(dvig2D9, LOW);      // крутим левую назад
      digitalWrite(dvig2D4, HIGH);
      digitalWrite(dvig1A0, HIGH);
      digitalWrite(dvig2A1, HIGH);
      analogWrite(dvig1D5, 30);
      analogWrite(dvig2D6, 30);
      delay (3000);
      digitalWrite(dvig1A0, LOW);
      digitalWrite(dvig2A1, LOW);
      delay (1500);
      }
    memset( commandBuffer, 0, sizeof( commandBuffer ) );
}
