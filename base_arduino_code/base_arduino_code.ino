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
// #include <ESP_Mail_Client.h> // 用于发送邮件 (需要安装ESP_Mail_Client库)
#include <vector>

// ================== 1. 硬件引脚定义 (根据你的实际接线修改这里) ==================
#define GPS_RX_PIN 16   // GPS模块 TX 接 ESP32 GPIO 16
#define GPS_TX_PIN 17   // GPS模块 RX 接 ESP32 GPIO 17
#define OLED_SDA_PIN 21 // OLED SDA 接 ESP32 GPIO 21
#define OLED_SCL_PIN 22 // OLED SCL 接 ESP32 GPIO 22
#define BUZZER_PIN 2    // 蜂鸣器 接 ESP32 GPIO 2 (可选)

// ================== 2. 网络与云服务配置 ==================
const char *WIFI_SSID = "iPhone(71)";
const char *WIFI_PASS = "12345679";

// 华为云 IoTDA 配置
const char *HUAWEI_MQTT_SERVER = "1fc5f68721.st1.iotda-device.cn-east-3.myhuaweicloud.com";
const int HUAWEI_MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "69ae7ce618855b39c5010ef5_myArduino_0_0_2026031607";
const char *MQTT_USERNAME = "69ae7ce618855b39c5010ef5_myArduino";
const char *MQTT_PASSWORD = "874f2bccd039c02e18e18aff8fdb1f06bd9f9c7d9c5906e6439a4227c532b0e6";
String DEVICE_ID = "69ae7ce618855b39c5010ef5_myArduino";

// 邮箱配置 (用于摔倒报警 - 已禁用)
// #define SMTP_HOST "smtp.qq.com"
// #define SMTP_PORT 465
// #define EMAIL_ACCOUNT "你的发件人邮箱@qq.com"
// #define EMAIL_PASSWORD "你的SMTP授权码"
// const char *EMERGENCY_CONTACT_EMAIL = "紧急联系人邮箱@example.com";

// NTP 时间服务器
const char *NTP_SERVER = "ntp.aliyun.com";

// ================== 3. 对象与全局变量 ==================
WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial NeoSerial(1);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
// SMTPSession smtp; // 邮件功能 (需要安装ESP_Mail_Client库)

// --- 数据结构：课表 ---
struct ScheduleItem
{
  int dayOfWeek; // 1=周一 ... 7=周日
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

// --- 小程序导航状态 ---
String navInstruction = "";      // 导航指令文本
int navDistance = 0;            // 距转向点距离（米）
String navDestination = "";     // 目的地方名称
double navLatitude = 0;         // 目的地方纬度
double navLongitude = 0;        // 目的地方经度
String navStatus = "off";       // 导航状态：off/navigating/arrive
bool navFromApplet = false;     // 是否来自小程序的导航

// --- 骑行统计 ---
float dailyDistance = 0.0;
unsigned long rideStartTime = 0;
unsigned long totalRideTimeSec = 0;
bool isMoving = false;
unsigned long lastStatsUpdate = 0;

// --- 摔倒检测状态 ---
bool fallDetected = false;
unsigned long fallTime = 0;
bool fallAlertSent = false;
unsigned long fallStartTime = 0;      // 首次满足疑似条件的时刻
bool fallSuspected = false;          // 是否处于疑似状态

// 摔倒检测参数
const float ACC_THRESHOLD = 3.0;      // 合加速度阈值 (g)
const float PITCH_THRESHOLD = 55.0;   // 俯仰角阈值 (度)
const unsigned long FALL_CONFIRM_TIME = 2000; // 持续2秒才确认
const float GYRO_THRESHOLD = 150.0;   // 陀螺仪阈值 (度/秒)

// --- 上报状态 ---
unsigned long lastReportTime = 0;
const unsigned long REPORT_INTERVAL = 5000; // 5秒上报一次

// ================== 4. 核心功能函数声明 ==================
void connectToWiFi();
void connectToMQTT();
void checkSchedule();
void updateNavigation();
void checkFallDetection();
void updateRideStats();
void sendEmailAlert();
void drawOLED();
void reportProperties();
void reportFallEvent();
void beepBuzzer(unsigned long duration_ms);
float getDistance(double lat1, double lon1, double lat2, double lon2);
float getBearing(double lat1, double lon1, double lat2, double lon2);

// ================== 5. Setup 初始化 ==================
void setup()
{
  Serial.begin(115200);
  Serial.println("\n\n=== 系统启动 ===");

  // 1. 初始化硬件
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  NeoSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // OLED 初始化
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED 初始化失败");
    while (1)
      ;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("System Boot...");
  display.display();

  // MPU6050 初始化
  if (!mpu.begin())
  {
    Serial.println("MPU6050 初始化失败");
    while (1)
      ;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 就绪");

  // 初始化有源蜂鸣器
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

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
void loop()
{
  // 维持 MQTT 连接
  if (!client.connected())
    connectToMQTT();
  client.loop();

  // 读取 GPS 数据
  while (NeoSerial.available() > 0)
  {
    gps.encode(NeoSerial.read());
  }

  // --- 功能模块轮询 ---
  checkSchedule();      // 1. 检查课表
  updateRideStats();    // 2. 统计骑行数据
  checkFallDetection(); // 3. 摔倒检测
  updateNavigation();   // 4. 导航逻辑

  // --- 定时上报属性数据 (每5秒) ---
  if (millis() - lastReportTime >= REPORT_INTERVAL)
  {
    reportProperties();
    lastReportTime = millis();
  }

  // 屏幕显示 (限制刷新率以防闪烁)
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 200)
  {
    drawOLED();
    lastDisplay = millis();
  }

  delay(20);
}

// ================== 7. 功能实现细节 ==================

// --- 网络连接 ---
void connectToWiFi()
{
  Serial.print("WiFi Connecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  Serial.print("DNS Test: ");
  Serial.println(WiFi.dnsIP().toString());
}

void connectToMQTT()
{
  if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD))
  {
    Serial.println("MQTT Connected");
    // 订阅华为云命令主题
    String subTopic = "$oc/devices/" + DEVICE_ID + "/sys/commands/#";
    client.subscribe(subTopic.c_str());
    // 订阅小程序导航指令主题
    String navTopic = "$oc/devices/" + DEVICE_ID + "/sys/property/down";
    client.subscribe(navTopic.c_str());
    Serial.println("已订阅: " + navTopic);
  }
  else
  {
    Serial.print("MQTT Failed, rc=");
    Serial.println(client.state());
  }
}

// --- 课表逻辑 ---
void checkSchedule()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
    return;

  int currentDay = timeinfo.tm_wday == 0 ? 7 : timeinfo.tm_wday; // 1-7
  int h = timeinfo.tm_hour;
  int m = timeinfo.tm_min;

  // 每天0点重置执行状态
  if (h == 0 && m == 0)
  {
    for (auto &item : weeklySchedule)
      item.executed = false;
  }

  if (!isNavigating)
  { // 只有在非导航状态下才检查新课表
    for (auto &item : weeklySchedule)
    {
      if (item.dayOfWeek == currentDay && item.hour == h && item.minute == m && !item.executed)
      {
        isNavigating = true;
        navTargetLat = item.lat;
        navTargetLng = item.lng;
        navTargetName = item.name;
        item.executed = true;
        Serial.println(">>> 触发课表导航: " + item.name);
        beepBuzzer(200); // 提示音
      }
    }
  }
}

// --- 导航计算 ---
void updateNavigation()
{
  if (!isNavigating || !gps.location.isValid())
    return;

  double currentLat = gps.location.lat();
  double currentLng = gps.location.lng();

  float dist = getDistance(currentLat, currentLng, navTargetLat, navTargetLng);
  float bearing = getBearing(currentLat, currentLng, navTargetLat, navTargetLng);

  // 简单航向计算 (实际应结合MPU6050 Yaw角)
  float heading = gps.course.deg();
  float turnAngle = bearing - heading;
  while (turnAngle < -180)
    turnAngle += 360;
  while (turnAngle > 180)
    turnAngle -= 360;

  // 到达判定
  if (dist < 20)
  {
    Serial.println("到达目的地: " + navTargetName);
    isNavigating = false;
    beepBuzzer(1000); // 到达提示音
  }
}

// --- 摔倒检测 ---
void checkFallDetection()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 计算合加速度
  float accMag = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);

  // 计算俯仰角
  float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

  // 计算陀螺仪合值
  float gyroMag = sqrt(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z) * 180 / PI;

  // === 阶段1: 疑似检测 ===
  // 条件：加速度超标 + 姿态倾斜
  if (!fallDetected && !fallSuspected && accMag > ACC_THRESHOLD && abs(pitch) > PITCH_THRESHOLD)
  {
    fallSuspected = true;
    fallStartTime = millis();
    Serial.println("[摔倒检测] 疑似摔倒，开始计时...");
  }

  // === 阶段2: 确认检测 ===
  // 疑似状态持续超过2秒，且陀螺仪检测到明显旋转
  if (!fallDetected && fallSuspected)
  {
    unsigned long elapsed = millis() - fallStartTime;

    if (elapsed >= FALL_CONFIRM_TIME)
    {
      // 确认条件：持续2秒 + 陀螺仪阈值
      if (gyroMag > GYRO_THRESHOLD)
      {
        fallDetected = true;
        fallTime = millis();
        fallAlertSent = false;
        Serial.println("⚠️ 摔倒确认！准备报警...");
        beepBuzzer(500);
      }
      else
      {
        // 2秒后陀螺仪不达标，取消疑似
        fallSuspected = false;
        Serial.println("[摔倒检测] 陀螺仪未达标，解除疑似");
      }
    }
  }

  // === 阶段3: 报警触发 (确认后5秒发送) ===
  if (fallDetected && millis() - fallTime > 5000 && !fallAlertSent)
  {
    reportFallEvent(); // 发送跌倒事件到华为云
    fallAlertSent = true;
  }

  // === 阶段4: 30秒后自动重置 ===
  if (fallDetected && millis() - fallTime > 30000)
  {
    fallDetected = false;
    fallSuspected = false;
    fallAlertSent = false;
  }

  // === 取消疑似: 如果条件不再满足且未确认 ===
  if (fallSuspected && !fallDetected)
  {
    if (accMag <= ACC_THRESHOLD || abs(pitch) <= PITCH_THRESHOLD)
    {
      fallSuspected = false;
      Serial.println("[摔倒检测] 条件不满足，解除疑似");
    }
  }
}

// --- 邮件发送 (已禁用，如需启用请安装ESP_Mail_Client库) ---
// void sendEmailAlert()
// {
//   Serial.println("📧 正在发送邮件报警...");
//   String mapLink = "https://uri.amap.com/marker?position=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
//   String msg = "紧急报警！检测到用户摔倒。\n位置链接: " + mapLink;

//   smtp.begin(SMTP_HOST, SMTP_PORT, EMAIL_ACCOUNT, EMAIL_PASSWORD);
//   if (smtp.sendEmail(EMAIL_ACCOUNT, EMERGENCY_CONTACT_EMAIL, "🚨 摔倒报警", msg))
//   {
//     Serial.println("邮件发送成功");
//   }
//   else
//   {
//     Serial.println("邮件发送失败");
//   }
// }

// --- 骑行统计 ---
void updateRideStats()
{
  if (!gps.speed.isValid())
    return;
  float speed = gps.speed.kmph();

  if (speed > 1.0)
  { // 移动中
    if (!isMoving)
    {
      isMoving = true;
      rideStartTime = millis();
    }
    dailyDistance += (speed / 3600.0); // 粗略累加
  }
  else
  {
    if (isMoving)
    {
      isMoving = false;
      totalRideTimeSec += (millis() - rideStartTime) / 1000;
    }
  }

  // 每天0点上传日报
  struct tm timeinfo;
  if (getLocalTime(&timeinfo) && timeinfo.tm_hour == 0 && timeinfo.tm_min == 0)
  {
    // 上传逻辑略，参考课表上传
  }
}

// --- 属性上报 (每5秒) ---
void reportProperties()
{
  if (!client.connected())
    return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  double lat = gps.location.isValid() ? gps.location.lat() : 0;
  double lng = gps.location.isValid() ? gps.location.lng() : 0;
  double speed = gps.speed.isValid() ? gps.speed.kmph() : 0;
  double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0;

  float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float roll = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;

  // 计算Yaw（积分陀螺仪Z轴）
  static float yaw = 0;
  static unsigned long lastYawTime = 0;
  unsigned long now = millis();
  if (lastYawTime > 0) {
    float dt = (now - lastYawTime) / 1000.0;
    yaw += g.gyro.z * dt;
  }
  lastYawTime = now;

  String topic = "$oc/devices/" + DEVICE_ID + "/sys/properties/report";
  String json = "{\"services\":[{\"service_id\":\"Arduino\",\"properties\":{";
  json += "\"latitude\":" + String(lat, 6) + ",";
  json += "\"longitude\":" + String(lng, 6) + ",";
  json += "\"speed\":" + String(speed, 2) + ",";
  json += "\"altitude\":" + String(alt, 1) + ",";
  json += "\"accelerationX\":" + String(a.acceleration.x, 3) + ",";
  json += "\"accelerationY\":" + String(a.acceleration.y, 3) + ",";
  json += "\"accelerationZ\":" + String(a.acceleration.z, 3) + ",";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"yaw\":" + String(yaw, 2) + ",";
  json += "\"timestamp\":" + String(now / 1000);
  json += "}}]}";

  client.publish(topic.c_str(), json.c_str());
  Serial.println("[上报] 属性数据已发送:");
  Serial.println(json);
}

// --- 跌倒事件上报 ---
void reportFallEvent()
{
  if (!client.connected())
    return;

  String topic = "$oc/devices/" + DEVICE_ID + "/sys/events";
  String json = "{\"services\":[{\"service_id\":\"Arduino\",\"properties\":{";
  json += "\"event\":\"fall_detected\",";
  json += "\"latitude\":" + String(gps.location.isValid() ? gps.location.lat() : 0, 6) + ",";
  json += "\"longitude\":" + String(gps.location.isValid() ? gps.location.lng() : 0, 6) + ",";
  json += "\"timestamp\":" + String(millis() / 1000);
  json += "}}]}";

  client.publish(topic.c_str(), json.c_str());
  Serial.println("[上报] 跌倒事件已发送");
}

// --- OLED 显示 ---
void drawOLED()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  // 如果小程序导航开启，优先显示导航信息
  if (navFromApplet && navStatus.equals("navigating"))
  {
    // 第1行: 目的地
    display.print("目的: ");
    display.println(navDestination);

    // 第2行: 距离
    display.setCursor(0, 10);
    display.print("前方 ");
    display.print(navDistance);
    display.println("m");

    // 第3行: 导航指令
    display.setCursor(0, 20);
    if (navInstruction.indexOf("左转") >= 0) {
      display.print("<- ");
    } else if (navInstruction.indexOf("右转") >= 0) {
      display.print("-> ");
    } else if (navInstruction.indexOf("掉头") >= 0) {
      display.print("U ");
    } else if (navInstruction.indexOf("直行") >= 0) {
      display.print("^ ");
    }
    display.println(navInstruction);

    display.display();
    return;
  }

  // 如果导航到达，显示到达信息
  if (navFromApplet && navStatus.equals("arrive"))
  {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println("  已到达目的地  ");
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, 40);
    display.println(navDestination);
    display.display();
    return;
  }

  // 正常显示模式（原逻辑）
  // 第1行: GPS状态
  if (gps.location.isValid())
  {
    display.print("GPS: Fix OK");
  }
  else
  {
    display.print("GPS: No Fix");
  }

  // 第2行: 经纬度
  if (gps.location.isValid())
  {
    display.setCursor(0, 8);
    display.print(gps.location.lat(), 5);
    display.print(",");
    display.print(gps.location.lng(), 5);
  }

  // 第3行: 速度
  display.setCursor(0, 16);
  display.print("Spd: ");
  display.print(gps.speed.kmph(), 1);
  display.print(" km/h");

  // 第4行: MPU数据
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  display.setCursor(0, 24);
  display.print("MPU Pitch:");
  display.print(pitch, 0);
  display.print((char)247);

  // 第5行: 跌倒状态
  display.setCursor(0, 32);
  if (fallDetected)
  {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.print("!!! FALL DETECTED !!!");
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }
  else
  {
    display.print("Status: Normal");
  }

  // 第6行: 导航状态
  display.setCursor(0, 40);
  if (isNavigating)
  {
    display.print("NAV:");
    display.print(navTargetName);
  }
  else
  {
    display.print("Mode: Normal");
  }

  display.display();
}

// --- MQTT 回调 (接收课表和导航指令) ---
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String msg = "";
  for (int i = 0; i < length; i++)
    msg += (char)payload[i];

  Serial.println("[MQTT收到] " + msg);

  // 解析 JSON 命令
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error)
    return;

  // 解析小程序导航指令
  if (doc.containsKey("navInstruction"))
  {
    navFromApplet = true;
    navInstruction = doc["navInstruction"].as<String>();
    navDistance = doc["navDistance"].as<int>();
    navDestination = doc["navDestination"].as<String>();
    navLatitude = doc["navLatitude"].as<double>();
    navLongitude = doc["navLongitude"].as<double>();
    navStatus = doc["navStatus"].as<String>();

    Serial.println(">>> 收到导航指令: " + navInstruction);
    Serial.println("    距离: " + String(navDistance) + "米");
    Serial.println("    目的地: " + navDestination);
    Serial.println("    状态: " + navStatus);

    // 蜂鸣器提示
    beepBuzzer(100);
  }
  // 解析课表命令
  else if (doc["command"] == "SET_SCHEDULE")
  {
    weeklySchedule.clear();
    JsonArray items = doc["data"];
    for (JsonObject item : items)
    {
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
float getDistance(double lat1, double lon1, double lat2, double lon2)
{
  double R = 6371000;
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

float getBearing(double lat1, double lon1, double lat2, double lon2)
{
  float dLon = (lon2 - lon1) * PI / 180.0;
  float y = sin(dLon) * cos(lat2 * PI / 180.0);
  float x = cos(lat1 * PI / 180.0) * sin(lat2 * PI / 180.0) - sin(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * cos(dLon);
  float brng = atan2(y, x);
  return (brng * 180.0 / PI + 360) / 180.0;
}

// --- 有源蜂鸣器控制 ---
void beepBuzzer(unsigned long duration_ms)
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration_ms);
  digitalWrite(BUZZER_PIN, LOW);
}