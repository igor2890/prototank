#include <AccelStepper.h>


#define dvig1A0 2 //едем не едем правая
#define dvig1D7 3 //правая назад
#define dvig1D8 4 //правая вперед
#define dvig1D5 5 //pwm правая

#define dvig2A1 9 //едем не едем левая
#define dvig2D9 8 //левая вперед
#define dvig2D4 7 //левая назад
#define dvig2D6 6 //pwm левая


AccelStepper stepper (1, 13, 12);

char A;
char b[15] = {0}; //создаем массив с нулями
int stepcmd = 0;  //рабочая переменная для поворота башни

int DVIGRonoff = 0;
int dvigRight = 0;
int dvigR0 = 0;
int dvigR1 = 0;

int DVIGLonoff = 0;
int dvigLeft = 0;
int dvigL0 = 0;
int dvigL1 = 0;

int i = 1;        //рабочая переменная для циклов
int IDtank = 0;     //переменная ID танка, который назначает сервер
int stepspeed = 2000; //переменная скорости поворота башни

void setup()
{
  Serial.begin(9600);
  Serial.setTimeout (10);
  
  pinMode(dvig1A0, OUTPUT);   
  pinMode(dvig1D7, OUTPUT);  
  pinMode(dvig1D8, OUTPUT);   

  digitalWrite(dvig1A0, LOW);
  
  pinMode(dvig2A1, OUTPUT);   
  pinMode(dvig2D9, OUTPUT);  
  pinMode(dvig2D4, OUTPUT);   

  digitalWrite(dvig2A1, LOW);

stepper.setMaxSpeed(3000);
   stepper.setSpeed(stepspeed);
  
}

void loop()
{

while (i == 1)  //цикл ожидания опроса от малинки для установления "личности" (ард.управленец или ард.телеметрия)
  {
    if (Serial.available()) {              //если что то пришло
      Serial.readBytesUntil('$', b, 10 );    //записываем полученное в массив
      int ii = b[0] - '0';                  // переводим первый символ в число
      switch(ii){                  //если первый символ
       case 9:                    //равен 9,
        delay (500);
        Serial.print ("900001#");  //то отвечаем малинке, что мы ард.управленец
        Serial.write('\n');
        i = 2;                    //и выключаем цикл while
        break;
       default:                  //если первый символ не равен 9
        break;                  //то цикл while повторяется, управление двигателями не начинается
      }
    b[0] = '0';                  //на всякий пожарный перезаписываем первый символ символом 0
    delay (10);
    }
   delay (10);
  }
  delay (100);
while (i == 2)  //цикл ожидания команды от малинки для записи id танка
  {
    if (Serial.available()) {              //если что то пришло
      Serial.readBytesUntil('$', b, 10 );    //записываем полученное в массив
      int ee = b[0] - '0';                  // переводим первый символ в число
      //Serial.print (ii);
      //Serial.write('\n');

      switch(ee){                  //если первый символ
       case 8:                    //равен 8,
        {
        int aa = b[3] - '0';
        int bb = b[4] - '0';
        int cc = b[5] - '0';
        IDtank = (aa*100)+(bb*10)+cc; //пока не знаю как использовать id танка в выстреле - пусть полежит здесь (может нужно в другой массив залить)
        delay (500);
        Serial.print ("888888#");  //и отвечаем малинке, что мы получили id
        Serial.write('\n');
        i = 0;                    //и выключаем цикл while
        }
        break;
       default:                  //если первый символ не равен 8
        break;                  //то цикл while повторяется, управление двигателями не начинается
      }
    b[0] = '0';                  //на всякий пожарный перезаписываем первый и четвертый символ символом 0
    b[3] = '0';
    delay (10);
    }
   delay (10);
  }
  delay (100);
  
while (i == 0)
{  
  if (Serial.available())         //если что-то пришло в буфер - читаем
  {
  Serial.readBytesUntil('$', b, 10 ); //читаем в массив b до тех пор, пока не увидим символ $ либо не считаем 9 символов
  DVIGRonoff = b[3] - '0';
  dvigR0 = b[4] - '0';        
  dvigR1 = b[5] - '0';
  dvigRight = (dvigR0*10)+ dvigR1;
  dvigRight = map (dvigRight, 1, 99, 30, 150);

  DVIGLonoff = b[0] - '0';
  dvigL0 = b[1] - '0';        
  dvigL1 = b[2] - '0';
  dvigLeft = (dvigL0*10)+ dvigL1;
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
     stepcmd = b[7] - '0'; 
  } 
      
  switch (stepcmd){
    case 1:               //поворачиваем направо (отрицательное значение скорости)
      if (stepspeed > 0){
        stepspeed = -stepspeed;
        stepper.setSpeed(stepspeed);
      }
      stepper.runSpeed();
      break;
    case 2:               //поворачиваем налево (положительное значение скорости)
      if (stepspeed < 0){
        stepspeed = -stepspeed;
        stepper.setSpeed(stepspeed);
      }
      stepper.runSpeed();
      break;
    default:
      break;
  }
}
} 
