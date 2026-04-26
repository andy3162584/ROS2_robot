#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <esp_now.h>

// 填入你上次查到的接收端 MAC 地址
uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0xA0, 0xFB, 0xC4};
esp_now_peer_info_t peerInfo;


// --- 馬達腳位定義 ---
const int left_L_PWM = 27;  
const int left_R_PWM = 14;  
const int right_L_PWM = 12; 
const int right_R_PWM = 13;  

// --- 編碼器腳位 ---
#define ENC_LF_A 16
#define ENC_LF_B 17
#define ENC_LB_A 32
#define ENC_LB_B 33
#define ENC_RF_A 35
#define ENC_RF_B 34
#define ENC_RB_A 26
#define ENC_RB_B 25

// 定義四個編碼器物件
ESP32Encoder encLF;
ESP32Encoder encLB;
ESP32Encoder encRF;
ESP32Encoder encRB;

// --- 全域變數 ---
TFT_eSPI tft = TFT_eSPI();
Adafruit_MPU6050 mpu;
volatile long posLF=0, posLB=0, posRF=0, posRB=0;

// 校正參數
float ax_bias = 0, ay_bias = 0, az_bias = 0;
float az_scale = 1.0; 
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

int motorSpeedL = 0; 
int motorSpeedR = 0;
unsigned long lastCmdTime = 0; 
unsigned long lastUpdate = 0;

typedef struct struct_message {
    uint32_t msg_id;   // 加入訊息序號，方便檢查有沒有掉封包
    long lf, lb, rf, rb;
    float ax, ay, az;
    float gx, gy, gz;
} struct_message;

struct_message sensorData;
uint32_t msgCounter = 0; // 用來計數

// --- 編碼器中斷 ---
//void IRAM_ATTR isrLF() { (digitalRead(ENC_LF_B)) ? posLF++ : posLF--; }
//void IRAM_ATTR isrLB() { (digitalRead(ENC_LB_B)) ? posLB++ : posLB--; }
//void IRAM_ATTR isrRF() { (digitalRead(ENC_RF_B)) ? posRF++ : posRF--; }
//void IRAM_ATTR isrRB() { (digitalRead(ENC_RB_B)) ? posRB++ : posRB--; }

// --- 馬達驅動函數 ---
void setMotorPWM(int p_left, int p_right) {
  int L = map(p_left, -100, 100, -255, 255);
  int R = map(p_right, -100, 100, -255, 255);

  if (L >= 0) {
    analogWrite(left_L_PWM, 0);
    analogWrite(left_R_PWM, abs(L));
  } else {
    analogWrite(left_L_PWM, abs(L)); // 修正：負值應取絕對值給 PWM
    analogWrite(left_R_PWM, 0);
  }

  if (R >= 0) {
    analogWrite(right_L_PWM, abs(R));
    analogWrite(right_R_PWM, 0);
  } else {
    analogWrite(right_L_PWM, 0); 
    analogWrite(right_R_PWM, abs(R));
  }
}

void setup() {
  Serial.begin(115200, SERIAL_8N1, 3, 1); 
  
  tft.init(); 
  tft.setTextSize(2);
  tft.setRotation(3); 
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("IMU Calibrating...");
  tft.println("DO NOT MOVE ROBOT!");

  Wire.begin(21, 22);
  if (mpu.begin()) {
    // 設定感測器範圍
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    float sumAX=0, sumAY=0, sumAZ=0, sumGX=0, sumGY=0, sumGZ=0;
    const int samples = 500; 
    
    for(int i=0; i<samples; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      sumAX += a.acceleration.x;
      sumAY += a.acceleration.y;
      sumAZ += a.acceleration.z;
      sumGX += g.gyro.x;
      sumGY += g.gyro.y;
      sumGZ += g.gyro.z;
      delay(3);
    }
    
    ax_bias = sumAX / samples;
    ay_bias = sumAY / samples;
    // 關鍵：修正 12.93 問題，強制將重力基準設為 9.806
    az_scale = 9.80665 / (sumAZ / samples); 
    
    gx_bias = sumGX / samples;
    gy_bias = sumGY / samples;
    gz_bias = sumGZ / samples;
    
    tft.println("IMU Ready!");
    tft.printf("Z-Scale: %.2f\n", az_scale);
  }

  pinMode(left_L_PWM, OUTPUT); pinMode(left_R_PWM, OUTPUT);
  pinMode(right_L_PWM, OUTPUT); pinMode(right_R_PWM, OUTPUT);
  
  pinMode(ENC_LF_A, INPUT_PULLUP); pinMode(ENC_LF_B, INPUT_PULLUP);
  pinMode(ENC_LB_A, INPUT_PULLUP); pinMode(ENC_LB_B, INPUT_PULLUP);
  pinMode(ENC_RF_A, INPUT); pinMode(ENC_RF_B, INPUT); // 34, 35 沒有內建上拉
  pinMode(ENC_RB_A, INPUT_PULLUP); pinMode(ENC_RB_B, INPUT_PULLUP);
  
  // 2. 設定腳位與 PCNT 頻道
  encLF.attachFullQuad(ENC_LF_A, ENC_LF_B);
  encLB.attachFullQuad(ENC_LB_A, ENC_LB_B);
  encRF.attachFullQuad(ENC_RF_A, ENC_RF_B);
  encRB.attachFullQuad(ENC_RB_A, ENC_RB_B);

  // 3. 設定硬體濾波器 (關鍵！)
  // 數值範圍 0-1023，通常設定為 250-1000 之間，數值越大濾波效果越強
  encLF.setFilter(500);
  encLB.setFilter(500);
  encRF.setFilter(500);
  encRB.setFilter(500);

  // 4. 計數清零
  encLF.clearCount();
  encLB.clearCount();
  encRF.clearCount();
  encRB.clearCount();

/*
  attachInterrupt(digitalPinToInterrupt(ENC_LF_A), isrLF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LB_A), isrLB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RF_A), isrRF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RB_A), isrRB, RISING);
*/

  WiFi.mode(WIFI_STA); 
  // 強制設定頻道（這點很重要，若沒設有時會跳到不同頻道）
  // 在某些版本中，ESP-NOW 會跟隨最後一次 Wi-Fi 連線的頻道

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 初始化失敗");
    return;
  }

  // 註冊配對
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  // 0 代表使用當前 Wi-Fi 頻道
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("配對失敗");
  }
}

void loop() {
  unsigned long now = millis();

  // 1. 接收樹莓派指令
  static String inputBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      int comma = inputBuffer.indexOf(',');
      if (comma != -1) {
        motorSpeedL = inputBuffer.substring(0, comma).toInt();
        motorSpeedR = inputBuffer.substring(comma + 1).toInt();
        lastCmdTime = now;
      }
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  // 2. 數據上報 (20Hz)
  if (now - lastUpdate >= 50) {
    lastUpdate = now;

    if (now - lastCmdTime > 1000) { motorSpeedL = 0; motorSpeedR = 0; }
    setMotorPWM(motorSpeedL, motorSpeedR);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 校正數據
    float cal_ax = a.acceleration.x - ax_bias;
    float cal_ay = a.acceleration.y - ay_bias;
    float cal_az = a.acceleration.z * az_scale; // 比例縮放

    float cal_gx = g.gyro.x - gx_bias;
    float cal_gy = g.gyro.y - gy_bias;
    float cal_gz = g.gyro.z - gz_bias;

    long posLF = (long)encLF.getCount();
    long posLB = (long)encLB.getCount();
    long posRF = (long)encRF.getCount();
    long posRB = (long)encRB.getCount();

    // 靜止消噪
    if (abs(cal_gz) < 0.02) cal_gz = 0;

    // 格式：LF,LB,RF,RB,AccX,AccY,AccZ,GyroX,GyroY,GyroZ
    // 輸出與 Raspberry Pi 的 motor_bridge 解析邏輯對齊
    Serial.printf("%ld,%ld,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                  posLF, posLB, -posRF, -posRB,
                  cal_ax, cal_ay, cal_az, 
                  cal_gx, cal_gy, cal_gz);

    // 螢幕顯示除錯資訊
    tft.setCursor(0, 0);
    tft.printf("L/R: %d / %d   \n", motorSpeedL, motorSpeedR);
    tft.printf("AZ: %.2f (9.8?) \n", cal_az);
    tft.printf("GZ: %.3f       \n", cal_gz);
    tft.printf("RF/RB:%7ld/%7ld\n", posRF, posRB);
    tft.printf("LF/LB:%7ld/%7ld\n", posLF, posLB);

    // 填入要發送的數據
    sensorData.lf = posLF;
    sensorData.lb = posLB;
    sensorData.rf = -posRF; // 注意這裡與 Serial 輸出一致，加了負號
    sensorData.rb = -posRB;
    sensorData.ax = cal_ax;
    sensorData.ay = cal_ay;
    sensorData.az = cal_az;
    sensorData.gx = cal_gx;
    sensorData.gy = cal_gy;
    sensorData.gz = cal_gz;

    sensorData.msg_id = msgCounter++; // 序號累加

    // 執行發送
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData));
    
    // 建議在開發階段印出狀態，正式跑的時候可以關掉以節省時間
    if (result != ESP_OK) {
      // Serial.println("發送失敗"); 
    }
  }
}
