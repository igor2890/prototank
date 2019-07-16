#define dvig1A0 2 //едем не едем правая
#define dvig1D7 3 //правая назад
#define dvig1D8 4 //правая вперед
#define dvig1D5 5 //pwm правая

#define dvig2A1 9 //едем не едем левая
#define dvig2D9 8 //левая вперед
#define dvig2D4 7 //левая назад
#define dvig2D6 6 //pwm левая

char b[10] = {0}; //создаем массив с нулями

int DVIGRonoff = 0;
int dvigRight = 0;
int dvigR0 = 0;
int dvigR1 = 0;

int DVIGLonoff = 0;
int dvigLeft = 0;
int dvigL0 = 0;
int dvigL1 = 0;

int i = 1;
int IDtank = 0;

void setup()
{
  Serial.begin(9600);
  
  pinMode(dvig1A0, OUTPUT);   
  pinMode(dvig1D7, OUTPUT);  
  pinMode(dvig1D8, OUTPUT);   

  digitalWrite(dvig1A0, LOW);
  
  pinMode(dvig2A1, OUTPUT);   
  pinMode(dvig2D9, OUTPUT);  
  pinMode(dvig2D4, OUTPUT);   

  digitalWrite(dvig2A1, LOW);
}

void loop()
{

while (i == 1)  //цикл ожидания опроса от малинки для установления "личности" (ард.управленец или ард.телеметрия)
  {
    if (Serial.available()) {              //если что то пришло
      Serial.readBytesUntil('$', b, 7 );    //записываем полученное в массив
      int ii = b[0] - '0';                  // переводим первый символ в число
      //Serial.write (b);
      //Serial.write('\n');
      //Serial.print (ii);
      //Serial.write('\n');
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

while (i == 2)  //цикл ожидания команды от малинки для записи id танка
  {
    if (Serial.available()) {              //если что то пришло
      Serial.readBytesUntil('$', b, 7 );    //записываем полученное в массив
      int ii = b[0] - '0';                  // переводим первый символ в число
      //Serial.write (b);
      //Serial.write('\n');
      //Serial.print (ii);
      //Serial.write('\n');

      switch(ii){                  //если первый символ
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
  
  while(Serial.available())         //если что-то пришло в буфер - читаем
  {
  Serial.readBytesUntil('$', b, 9 ); //читаем в массив b до тех пор, пока не увидим символ $ либо не считаем 9 символов
Serial.write (b);
Serial.write('\n');

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

if (DVIGRonoff == 0)         // правая
{
  digitalWrite(dvig1A0, LOW);     //не крутим
}
else if (DVIGRonoff == 1)
{
  digitalWrite(dvig1A0, HIGH);     // крутим правую
  digitalWrite(dvig1D7, LOW);      // крутим правую вперед
  digitalWrite(dvig1D8, HIGH);  
}
else if (DVIGRonoff == 2)
{
  digitalWrite(dvig1A0, HIGH);    // крутим правую
  digitalWrite(dvig1D7, HIGH);    // крутим правую назад
  digitalWrite(dvig1D8, LOW);
}
else
{
  digitalWrite(dvig1A0, LOW);     //не крутим
}

if (DVIGLonoff == 0)        // левая
{
  digitalWrite(dvig2A1, LOW);     //не крутим

}
else if (DVIGLonoff == 1)
{
  digitalWrite(dvig2A1, HIGH);    // крутим левую
  digitalWrite(dvig2D9, HIGH);    // крутим левую вперед
  digitalWrite(dvig2D4, LOW); 
}
else if (DVIGLonoff == 2)
{
  digitalWrite(dvig2A1, HIGH);     // крутим левую
  digitalWrite(dvig2D9, LOW);      // крутим левую назад
  digitalWrite(dvig2D4, HIGH);  
}
else
{
  digitalWrite(dvig2A1, LOW);     //не крутим
}
   
     analogWrite(dvig1D5, dvigRight);
     analogWrite(dvig2D6, dvigLeft);   

delay (10);
  } 
      delay (10);
} 
