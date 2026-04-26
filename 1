/*
 * 项目名称：基于ESP32与华为云的智能骑行导航与监护系统
 * 功能：一周课表导航、摔倒检测邮件报警、骑行日报、华为云IoT互联
 * 作者：你的助手
 */

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "time.h"
#include <ESP32MailClient.h> // 用于发送邮件
#include <vector>

// ================== 1. 硬件引脚定义 (根据你的实际接线修改这里) ==================
#define GPS_RX_PIN 16       // GPS模块 TX 接 ESP32 GPIO 16
#define GPS_TX_PIN 17       // GPS模块 RX 接 ESP32 GPIO 17
#define OLED_SDA_PIN 21     // OLED SDA 接 ESP32 GPIO 21
#define OLED_SCL_PIN 22     // OLED SCL 接 ESP32 GPIO 22
#define BUZZER_PIN 2        // 蜂鸣器 接 ESP32 GPIO 2 (可选)

// ================== 2. 网络与云服务配置 ==================
const char* WIFI_SSID = "你的WiFi名称";
const char* WIFI_PASS = "你的WiFi密码";

// 华为云 IoTDA 配置
const char* HUAWEI_MQTT_SERVER = "你的项目ID.iot-mqtts.cn-north-4.myhuaweicloud.com";
const int HUAWEI_MQTT_PORT = 1883; // 如果使用SSL加密通常是8883
const char* MQTT_CLIENT_ID = "你的设备ID_0_0_2024010100";
const char* MQTT_USERNAME = "你的设备ID 你的产品ID";
const char* MQTT_PASSWORD = "你的设备密钥";
String DEVICE_ID = "你的设备ID";

// 邮箱配置 (用于摔倒报警)
#define SMTP_HOST "smtp.qq.com"       // 发件人邮箱SMTP服务器
#define SMTP_PORT 465                 // SSL端口
#define EMAIL_ACCOUNT "你的发件人邮箱@qq.com"
#define EMAIL_PASSWORD "你的SMTP授权码" // 注意：是授权码，不是登录密码
const char* EMERGENCY_CONTACT_EMAIL = "紧急联系人邮箱@example.com";

// NTP 时间服务器
const char* NTP_SERVER = "ntp.aliyun.com";

// ================== 3. 对象与全局变量 ==================
WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial NeoSerial(1);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
SMTPSession smtp;

// --- 数据结构：课表 ---
struct ScheduleItem {
  int dayOfWeek;  // 1=周一 ... 7=周日
  int hour;
  int minute;
  double lat;
  double lng;
  String name;
  bool executed;
};
std::vector<ScheduleItem> weeklySchedule;

// --- 导航状态 ---
bool isNavigating = false;
double navTargetLat, navTargetLng;
String navTargetName = "";

// --- 骑行统计 ---
float dailyDistance = 0.0;
unsigned long rideStartTime = 0;
unsigned long totalRideTimeSec = 0;
bool isMoving = false;
unsigned long lastStatsUpdate = 0;

// --- 摔倒检测状态 ---
bool fallDetected = false;
unsigned long fallTime = 0;

// ================== 4. 核心功能函数声明 ==================
void connectToWiFi();
void connectToMQTT();
void checkSchedule();
void updateNavigation();
void checkFallDetection();
void updateRideStats();
void sendEmailAlert();
void drawOLED();
float getDistance(double lat1, double lon1, double lat2, double lon2);
float getBearing(double lat1, double lon1, double lat2, double lon2);

// ================== 5. Setup 初始化 ==================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== 系统启动 ===");

  // 1. 初始化硬件
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  NeoSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // OLED 初始化
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED 初始化失败");
    while(1);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("System Boot...");
  display.display();

  // MPU6050 初始化
  if (!mpu.begin()) {
    Serial.println("MPU6050 初始化失败");
    while(1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 就绪");

  // 2. 连接网络
  connectToWiFi();
  configTime(8 * 3600, 0, NTP_SERVER); // 设置时区
  
  // 3. 连接华为云
  client.setServer(HUAWEI_MQTT_SERVER, HUAWEI_MQTT_PORT);
  client.setCallback(mqttCallback);
  connectToMQTT();

  display.println("Ready!");
  display.display();
  Serial.println("=== 系统就绪 ===");
}

// ================== 6. Loop 主循环 ==================
void loop() {
  // 维持 MQTT 连接
  if (!client.connected()) connectToMQTT();
  client.loop();

  // 读取 GPS 数据
  while (NeoSerial.available() > 0) {
    gps.encode(NeoSerial.read());
  }

  // --- 功能模块轮询 ---
  checkSchedule();          // 1. 检查课表
  updateRideStats();        // 2. 统计骑行数据
  checkFallDetection();     // 3. 摔倒检测
  updateNavigation();       // 4. 导航逻辑
  
  // 屏幕显示 (限制刷新率以防闪烁)
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 200) {
    drawOLED();
    lastDisplay = millis();
  }

  delay(20);
}

// ================== 7. 功能实现细节 ==================

// --- 网络连接 ---
void connectToWiFi() {
  display.print("WiFi Conn...");
  display.display();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

void connectToMQTT() {
  if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
    Serial.println("MQTT Connected");
    // 订阅华为云命令主题
    String subTopic = "$oc/devices/" + DEVICE_ID + "/sys/commands/#";
    client.subscribe(subTopic.c_str());
  } else {
    Serial.print("MQTT Failed, rc=");
    Serial.println(client.state());
  }
}

// --- 课表逻辑 ---
void checkSchedule() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) return;

  int currentDay = timeinfo.tm_wday == 0 ? 7 : timeinfo.tm_wday; // 1-7
  int h = timeinfo.tm_hour;
  int m = timeinfo.tm_min;

  // 每天0点重置执行状态
  if (h == 0 && m == 0) {
    for(auto &item : weeklySchedule) item.executed = false;
  }

  if (!isNavigating) { // 只有在非导航状态下才检查新课表
    for (auto &item : weeklySchedule) {
      if (item.dayOfWeek == currentDay && item.hour == h && item.minute == m && !item.executed) {
        isNavigating = true;
        navTargetLat = item.lat;
        navTargetLng = item.lng;
        navTargetName = item.name;
        item.executed = true;
        Serial.println(">>> 触发课表导航: " + item.name);
        tone(BUZZER_PIN, 1000, 200); // 提示音
      }
    }
  }
}

// --- 导航计算 ---
void updateNavigation() {
  if (!isNavigating || !gps.location.isValid()) return;

  double currentLat = gps.location.lat();
  double currentLng = gps.location.lng();
  
  float dist = getDistance(currentLat, currentLng, navTargetLat, navTargetLng);
  float bearing = getBearing(currentLat, currentLng, navTargetLat, navTargetLng);
  
  // 简单航向计算 (实际应结合MPU6050 Yaw角)
  float heading = gps.course.deg(); 
  float turnAngle = bearing - heading;
  while (turnAngle < -180) turnAngle += 360;
  while (turnAngle > 180) turnAngle -= 360;

  // 到达判定
  if (dist < 20) {
    Serial.println("到达目的地: " + navTargetName);
    isNavigating = false;
    tone(BUZZER_PIN, 2000, 1000); // 到达提示音
  }
}

// --- 摔倒检测 ---
void checkFallDetection() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 计算合加速度
  float accMag = sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
  
  // 简易摔倒算法：撞击(G值>2.5) + 姿态改变(倾斜角>60度)
  float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  
  if (!fallDetected && accMag > 2.5 && abs(pitch) > 60) {
    fallDetected = true;
    fallTime = millis();
    Serial.println("⚠️ 检测到摔倒！准备报警...");
    tone(BUZZER_PIN, 1500, 500);
  }

  // 报警触发 (检测到后5秒发送，给用户取消机会可扩展)
  if (fallDetected && millis() - fallTime > 5000) {
    sendEmailAlert();
    // 上传报警到华为云
    String topic = "$oc/devices/" + DEVICE_ID + "/sys/properties/report";
    String json = "{\"SOS\":\"FALL_DETECTED\", \"Lat\":" + String(gps.location.lat()) + ", \"Lng\":" + String(gps.location.lng()) + "}";
    client.publish(topic.c_str(), json.c_str());
    fallDetected = false; // 重置
  }
}

// --- 邮件发送 ---
void sendEmailAlert() {
  Serial.println("📧 正在发送邮件报警...");
  String mapLink = "https://uri.amap.com/marker?position=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  String msg = "紧急报警！检测到用户摔倒。\n位置链接: " + mapLink;
  
  smtp.begin(SMTP_HOST, SMTP_PORT, EMAIL_ACCOUNT, EMAIL_PASSWORD);
  if(smtp.sendEmail(EMAIL_ACCOUNT, EMERGENCY_CONTACT_EMAIL, "🚨 摔倒报警", msg)) {
    Serial.println("邮件发送成功");
  } else {
    Serial.println("邮件发送失败");
  }
}

// --- 骑行统计 ---
void updateRideStats() {
  if (!gps.speed.isValid()) return;
  float speed = gps.speed.kmph();
  
  if (speed > 1.0) { // 移动中
    if (!isMoving) {
      isMoving = true;
      rideStartTime = millis();
    }
    dailyDistance += (speed / 3600.0); // 粗略累加
  } else {
    if (isMoving) {
      isMoving = false;
      totalRideTimeSec += (millis() - rideStartTime) / 1000;
    }
  }
  
  // 每天0点上传日报
  struct tm timeinfo;
  if(getLocalTime(&timeinfo) && timeinfo.tm_hour == 0 && timeinfo.tm_min == 0) {
     // 上传逻辑略，参考课表上传
  }
}

// --- OLED 显示 ---
void drawOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  // 状态栏
  if (isNavigating) {
    display.print("NAV: "); display.println(navTargetName);
    display.print("Dist: "); display.print(getDistance(gps.location.lat(), gps.location.lng(), navTargetLat, navTargetLng), 0); display.println("m");
  } else {
    display.println("Mode: Normal");
  }

  // 数据栏
  display.print("Spd: "); display.print(gps.speed.kmph(), 1); display.println(" km/h");
  display.print("Dist: "); display.print(dailyDistance, 2); display.println(" km");
  
  if (fallDetected) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.println("!!! FALL !!!");
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }
  
  display.display();
}

// --- MQTT 回调 (接收课表) ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  
  // 解析 JSON 命令
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) return;

  // 假设云端下发格式: {"command": "SET_SCHEDULE", "data": [{"day":1, "hour":8, ...}]}
  if (doc["command"] == "SET_SCHEDULE") {
    weeklySchedule.clear();
    JsonArray items = doc["data"];
    for (JsonObject item : items) {
      ScheduleItem s;
      s.dayOfWeek = item["day"];
      s.hour = item["hour"];
      s.minute = item["minute"];
      s.lat = item["lat"];
      s.lng = item["lng"];
      s.name = item["name"].as<String>();
      s.executed = false;
      weeklySchedule.push_back(s);
    }
    Serial.println("✅ 课表已更新");
  }
}

// --- 数学工具 ---
float getDistance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371000; 
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  double a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

float getBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = (lon2 - lon1) * PI / 180.0;
  float y = sin(dLon) * cos(lat2 * PI / 180.0);
  float x = cos(lat1 * PI / 180.0) * sin(lat2 * PI / 180.0) - sin(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * cos(dLon);
  float brng = atan2(y, x);
  return (brng * 180.0 / PI + 360) / 180.0;
}