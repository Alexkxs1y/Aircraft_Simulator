#include <SoftwareSerial.h>
#include "StepperSFC.h"//Библиотека для работы с шаговыми двигателями

//=================================== Настройка SoftwareSerial =========================================//
SoftwareSerial mySerial(4, 5); // RX, TX
float data[2]; 


//==================================== Настройка двигателей ============================================//
#define STOP 0.0f
#define COM_Speed 115200 //скорость обмена данными по компорту (бод)
#define MaxDegreeSpeed 1600//Максимально допустимая угловая скорость град/сек её необходимо считать исходя из времени выполнения кода
#define MinStepPulse 3 //минмальная длина импульса шага микросекунд 
#define StepsInFullRotation2 400//10000//20000 //кол-во шагов в 1 полном обороте 
#define StepsInFullRotation1 400//10000//20000 //кол-во шагов в 1 полном обороте 
//Ноги управления ШД Y  
#define Y1_step    13 //шаг
#define Y1_dir     12 //направление
#define Y1_enable  11 //управление питанием драйвера
#define Y2_step    10 //шаг
#define Y2_dir     9 //направление
#define Y2_enable  8 //управление питанием драйвера
StepperSFC motor_Y1(Y1_enable, Y1_dir, Y1_step, MinStepPulse, StepsInFullRotation1);
StepperSFC motor_Y2(Y2_enable, Y2_dir, Y2_step, MinStepPulse, StepsInFullRotation2);

float R = 0.09; //Радиус вращающегося блока в метрах
float t_curr = 0;
float deg2rad = 2.0 * 3.1415 / 360.0;
//================================= Текущие номинальные данные =======================================//
float t; //сек
float V0 = 600; // град/сек, начальная номинальная скорость
float a0 = 0; // град/с^2, начальное номинальное ускорение
float k = 0; // коэффициент при линейном ускорение от времени

//=================================== Полученные значения ============================================//
float a_f = 0; // м/с^2, измеренное акселерометром начальное ускорения
float V_f = V0 * deg2rad * R; // м/с, полученная с помощью акселерометра скорость
float x_f = 0; // м, полученное с помощью акселерометра перемещение

//=================================== Используемые значения ============================================//
float a_i = a0 * deg2rad * R; // м/с^2, корректированое с помощью ОС ускорение движения
float V_i = V0 * deg2rad * R; // м/с, корректированое с помощью ОС скорость движения

//=================================== Коэффициент наведения ============================================//
float K = 5; //*****************************************************************************************************************************************************

void setup() {
  Serial.begin(COM_Speed);
  mySerial.begin(57600); 
  delay(3000); // Чтобы все пролагало, чтобы акселерометр откалибровался, чтобы фильтр Калмана откалибровался, чтобы сервера подключились.          
  //Serial.println(F("successful connection"));
  Serial.println("Время на ардуине, номинальная координата, измеренная координата, a_f, ax, ay, az, row, pitch, yaw, время на акселерометре");
  motorsOn(); 
  motorsSpeed(V0);
  t = millis();
  t_curr = t;
}

void loop() {
  /*Serial.print(t_curr); Serial.print(" "); Serial.print(x_t() * deg2rad * R); Serial.print(" "); Serial.print(x_f); Serial.print(" "); Serial.print(a_f); Serial.print(" ");
  for(int i = 0; i < 7; i ++){
        Serial.print(data[i]);
        Serial.print(" ");
  }
  Serial.println();*/
  //t_curr = millis();  
  motor_Y1.runSpeed();
  motor_Y2.runSpeed();
  //delay(1);
  //float dt = millis() - t_curr;
  /*if (mySerial.available()){
    readSoftSerial();
  }*/
  //factParams(dt);
  //iParams(dt);
  //motorsSpeed(V_i); //с обратной связью
  //motorsSpeed(V_t());
}


void motorsOn(){
  motor_Y1.setMaxDegreeSpeed(MaxDegreeSpeed);
  motor_Y2.setMaxDegreeSpeed(MaxDegreeSpeed);
  motor_Y1.turnOn();
  motor_Y2.turnOn();  
}

void motorsSpeed(float V){
  motor_Y1.setDegreeSpeed(V);
  motor_Y2.setDegreeSpeed(V);
}


float a_t(){
  return 0; // a0 //a0 + k*(millis() - t)
}

float V_t(){
  return V0; // V0 + a0 * (millis() - t) //V0 + a0*(millis() - t) + k * (millis() - t) * (millis() - t) * 0.5
}

float x_t(){
  return V0 * (millis() - t); // V0 * (millis() - t) + a0 * (millis() - t) * (millis() - t) * 0.5 // V0 * (millis() - t) + a0 * (millis() - t) * (millis() - t) * 0.5 + k * (millis() - t) * (millis() - t) * (millis() - t) * 0.5 * 0.333333333
}



void readSoftSerial(){
  String receivedData = "";
  while (mySerial.available()) {
    char incomingChar = mySerial.read();
    if (incomingChar == '<') {
      receivedData = ""; // Start a new message
      } else if (incomingChar == '>') {
      // End of message, print the received data
      parseReceivedData(receivedData);
      receivedData = ""; // Reset after processing
    } else {
      // Append the character to the received data
      receivedData += incomingChar;
    }
  }
}

void parseReceivedData(String receivedString) {
  int index = 0;
  char charBuf[receivedString.length() + 1];
  receivedString.toCharArray(charBuf, receivedString.length() + 1);
  char* token = strtok(charBuf, ",");
  
  while (token != NULL && index < 2) {
    data[index] = atof(token);
    token = strtok(NULL, ",");
    index++;
  }
}

void iParams(float dt){
  V_i += a_i * dt;
  a_i = K * (x_t() * deg2rad * R -  x_f);
}

void factParams(float dt){
  x_f += V_f * dt;
  V_f += a_f * dt;
  if (V_f > MaxDegreeSpeed){ //*************************************************************************************************
    V_f = MaxDegreeSpeed;
  }
}
