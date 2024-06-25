#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP8266WiFi.h>


//================================ Настройки простого Калмана для ускорений ====================================//
float err_measure = 10;  // примерный шум измерений, типо среднеквадратичное отклонение ошибки
float q = 0.01;   // скорость изменения значений 0.001-1, варьировать самому
//==============================================================================================================//


//=========================================== Переменные для MPU6050 ===========================================//
MPU6050 mpu;
float data[2];

//То же для подгона Калмана
/*int16_t X, Y, Z;
int16_t GX, GY, GZ;*/
float t = 0;
//===============================================================================================================//


//=============================================== Настройка WiFi ================================================//
const char* ssid = "ESP01_AP"; // Same SSID as the server
const char* password = "password"; // Same password as the server
const char* serverAddress = "192.168.4.1"; // IP address of the server in the local network
const int serverPort = 8888;
WiFiClient client;
//===============================================================================================================//


void setup() {
  delay(100);
  Serial.begin(115200);
  Wire.begin();
  startMPU(mpu);
  connectToWiFi();
  ////connectToServer();
  delay(1000);
  t = millis();
}

void loop() {
  //По нему будем подгонять Коэффициенты для Калмана.
  /*mpu.getMotion6(&X, &Y, &Z, &GX, &GY, &GZ);
  Serial.print(float(X)* 2.0 * 9.81 / 32768.0  ); Serial.print(' ');
  Serial.print(float(Y)* 2.0 * 9.81 / 32768.0 ); Serial.print(' ');
  Serial.print(float(Z)* 2.0 * 9.81 / 32768.0 ); Serial.print(' ');*/

  readMPU(mpu, data);  
  sendToServer();
}





//===========================================================================================================//
//============================================== Для MPU6050 ================================================//
//===========================================================================================================//

//======================================= Чтение и фильтрация с MPU =========================================//
void readMPU(MPU6050 &mpu, float* data){
  static int16_t ax;
  //static VectorFloat gravity;
  ax = mpu.getAccelerationX();
  float ax_new = float(ax)* 2.0 * 9.81 / 32768.0;
  //simpleKalman(ax_new);
  
  data[0] = millis(); 
  data[1] = ax_new;

  //Serial.print(data[0]);
  //Serial.print(' ');
  //Serial.print(float(ax)* 2.0 * 9.81 / 32768.0);
  //Serial.print(' ');
  //Serial.print(data[1]);
  
  
  //Serial.println();
  
}


//========================================== Простой Калман ==================================================//
void simpleKalman(float &_newAx) {
  float kalman_gain, current_estimate;
  static float err_estimate = err_measure;
  static float last_estimate;
  kalman_gain = (float)err_estimate / (err_estimate + err_measure);
  current_estimate = last_estimate + (float)kalman_gain * (_newAx - last_estimate);
  err_estimate =  (1.0 - kalman_gain) * err_estimate + fabs(last_estimate - current_estimate) * q;
  last_estimate = current_estimate;
  _newAx = current_estimate;
}

/*void simpleKalman(float &_newAx, float &_newAy, float &_newAz) {
  float kalman_gain[3], current_estimate[3];
  float news[3] = {_newAx, _newAy, _newAz};
  static float err_estimate[3] = {err_measure, err_measure, err_measure};
  static float last_estimate[3];
  for(int i = 0; i < 3; i++){
    kalman_gain[i] = (float)err_estimate[i] / (err_estimate[i] + err_measure);
    current_estimate[i] = last_estimate[i] + (float)kalman_gain[i] * (news[i] - last_estimate[i]);
    err_estimate[i] =  (1.0 - kalman_gain[i]) * err_estimate[i] + fabs(last_estimate[i] - current_estimate[i]) * q;
    last_estimate[i] = current_estimate[i];
  }
  _newAx = current_estimate[0];
  _newAy = current_estimate[1];
  _newAz = current_estimate[2];
}*/


//===================================== Инициализация и запуск MPU ===========================================//
void startMPU(MPU6050 &mpu){
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  
  calibrate(mpu);
}


//========================================== Калибровка MPU =================================================//
void calibrate(MPU6050 &mpu){
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.CalibrateAccel(15);
}



//===========================================================================================================//
//========================================= Для передачи по WiFi ============================================//
//===========================================================================================================//

//========================================= Подключение к WiFi ==============================================//
void connectToWiFi() {
  // Connect to the local Wi-Fi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
}


//======================================== Подключение к серверу ============================================//
void connectToServer() {
  // Connect to TCP/IP server
  Serial.print("Connecting to server: ");
  Serial.println(serverAddress);
  if (client.connect(serverAddress, serverPort)) {
    Serial.println("Connected to server");
  } else {
    Serial.println("Connection failed. Retrying...");
    connectToServer(); // Retry connection
  }
}

//============================ Проверка WiFi подключение и переподключение ==================================//
void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    connectToWiFi();
  }
}


//======================================== Передача массива данных ============================================//
void sendToServer(){
  reconnectWiFi();
  if (client.connect(serverAddress, serverPort)) {
    Serial.println("Connected to server");
    
    // Send telemetry data to the server
    client.write((uint8_t*)data, sizeof(data));

    client.stop();
    Serial.println("Data sent to server");

  } else {
    Serial.println("Failed to connect to server");
  }
}
