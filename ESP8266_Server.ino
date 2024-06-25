#include <ESP8266WiFi.h>

float k_err_err = 0.037;
float exp_x = 0;
float avg_x = 0;
float kal_x = 0;
float V0 = 0.139;
float data_old[2];
bool flag = false;


//================================ Настройки простого Калмана для ускорений ====================================//
float err_measure = 10;  // примерный шум измерений, типо среднеквадратичное отклонение ошибки
float q = 0.7;   // скорость изменения значений 0.001-1, варьировать самому

float k =0.85;
const int averageNum = 60;

//=============================================== Настройка WiFi ================================================//
const char* ssid = "ESP01_AP"; // Choose your desired SSID for the local network
const char* password = "password"; // Choose your desired password for the local network
const int serverPort = 8888;

WiFiServer server(serverPort);
WiFiClient client;

//============================================ Данные акселерометра =============================================//
float data[2];
bool dataAvailable = false;

void setup() {
  //Serial.begin(57600);
  Serial.begin(115200);
  delay(100);

  createLocal();
  server.begin();
  Serial.println("Server started");
}

void loop() {
  readData();
  //Serial.print("zzzzzzzzzzzzzzz");
  /*if(dataAvailable){
    sendToArduino();
    dataAvailable = false;
  }*/
}


//===========================================================================================================//
//========================================= Для передачи по WiFi ============================================//
//===========================================================================================================//

//============================================ Поднятие сети ================================================//
void createLocal(){
  //Serial.print("Creating access point: ");
  //Serial.println(ssid);
  WiFi.softAP(ssid, password);
  
  IPAddress myIP = WiFi.softAPIP();
  //Serial.print("AP IP address: ");
  //Serial.println(myIP);
}

//============================================ Чтение данных ================================================//
void readData(){
  client = server.available();
  if (client && !dataAvailable) {
    //Serial.print("zzzzzzzzzz");
    data_old[0] = data[0];
    data_old[1] = data[1];
    size_t bytesReceived = client.readBytes((char*)data, sizeof(data));
    if(bytesReceived == sizeof(data)){
      if(abs(data[1]) > 0.5) flag = true;
      //dataAvailable = true;
      Serial.print(data[0]);
      Serial.print(' ');
      Serial.print(data[1]);
 
      float ax = data[1];
      
      Serial.print(' ');
      /*
      Serial.print(runAverage(ax));
      Serial.print(' ');
      Serial.print(expRunningAverage(ax));
      Serial.print(' ');*/
      simpleKalman(ax);
      Serial.print(ax);
      
      Serial.println();
    }
    client.stop();
    delay(1);
  }
}


//===========================================================================================================//
//===================================== Передача по Software Serial =========================================//
//===========================================================================================================//

void sendToArduino() {
  String dataString = "<";
  for (int i = 0; i < 2; i++) {
    dataString += String(data[i], 2); // Use 6 decimal places for precision
    if (i < 1) {
      dataString += ","; // Add comma between the elements
    }
  }
  dataString += ">";

  Serial.println(dataString);
}

void simpleKalman(float &_newAx) {
  float kalman_gain, current_estimate;
  static float err_estimate = err_measure;
  static float last_estimate;
  if(flag){
    kal_x += V0 * (data[0] - data_old[0]) * 0.001;
    V0 += last_estimate * (data[0] - data_old[0]) * 0.001; 
    Serial.print(kal_x); Serial.print(' ');
  }
  kalman_gain = (float)err_estimate / (err_estimate + err_measure);
  current_estimate = last_estimate + (float)kalman_gain * (_newAx - last_estimate);
  err_estimate =  (1.0 - kalman_gain) * err_estimate + fabs(last_estimate - current_estimate) * q;
  last_estimate = current_estimate;
  if(flag){
    _newAx = current_estimate - 0.0004 * kal_x + 0.0093;
  }else{
    _newAx = current_estimate;
  }
}

float expRunningAverage(float newVal){
  static float filVal = 0;
  if(flag){
    exp_x += V0 * (data[0] - data_old[0])*0.001 + filVal * (data[0] - data_old[0]) * (data[0] - data_old[0]) * 0.000001 * 0.5; 
  }
  if(flag){
    filVal += k*(newVal-filVal) - 0.0004 * exp_x + 0.0093;
  } else {
    filVal += k*(newVal-filVal);
  }
  return filVal;
}

float runAverage(float newVal){
  static int t = 0;
  static float vals[averageNum];
  static float average = 0;
  if(flag){
    avg_x += V0 * (data[0] - data_old[0]) * 0.001 + (float)average/averageNum * (data[0] - data_old[0]) * (data[0] - data_old[0]) * 0.000001 * 0.5; 
  }
  if(++t >= averageNum) t = 0;
  average -= vals[t];
  average += newVal;
  vals[t] = newVal;
  if (flag) return (float)average/averageNum - 0.0004 * avg_x + 0.0093;
  return (float)average/averageNum;
}
