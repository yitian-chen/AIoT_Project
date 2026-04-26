#include <TinyGPS++.h>
#include <HardwareSerial.h>

// 定义引脚：TX接到16(RX2)，RX接到17(TX2)
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

// 创建对象
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // 使用串口2

void setup() {
  // 电脑串口监视器，请在监视器窗口右下角选择 115200
  Serial.begin(115200);
  
  // GPS 模块串口通信
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

  Serial.println(F("========================================"));
  Serial.println(F("   AIoT 码表项目 - GPS 详细参数测试"));
  Serial.println(F("   正在等待 GPS 信号（请确保天线在室外）"));
  Serial.println(F("========================================"));
}

void loop() {
  // 将 GPS 串口传来的原始数据喂给 TinyGPS++ 解析
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      displayFullInfo();
    }
  }

  // 如果 5 秒后还没收到数据，检查一下接线
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("警告：未检测到 GPS 模块，请检查 16/17 引脚接线是否正确！"));
    delay(2000);
  }
}

void displayFullInfo() {
  Serial.println(F("--- 当前实时数据 ---"));

  // 1. 经纬度
  Serial.print(F("位置: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("无效(尚未搜到卫星)"));
  }

  // 2. 日期与时间
  Serial.print(F("  |  日期: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  Serial.print(F("  时间: "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0")); Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0")); Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0")); Serial.print(gps.time.second());
  }

  // 3. 速度与海拔
  Serial.println();
  Serial.print(F("速度: "));
  Serial.print(gps.speed.kmph());
  Serial.print(F(" km/h"));

  Serial.print(F("  |  海拔: "));
  Serial.print(gps.altitude.meters());
  Serial.print(F(" m"));

  // 4. 信号质量参数
  Serial.print(F("  |  卫星数: "));
  Serial.print(gps.satellites.value());

  Serial.print(F("  |  精度(HDOP): "));
  Serial.println(gps.hdop.value());

  Serial.println(F("----------------------------------------"));
  delay(1000); // 每一秒打印一次
}