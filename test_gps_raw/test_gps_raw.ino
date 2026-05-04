#include <HardwareSerial.h>

// 定义引脚：TX接到16(RX2)，RX接到17(TX2)
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

HardwareSerial gpsSerial(2); // 使用串口2

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

  Serial.println(F("========================================"));
  Serial.println(F("   GPS 原始数据测试"));
  Serial.println(F("   正在等待 GPS 信号..."));
  Serial.println(F("========================================"));
}

void loop() {
  // 将GPS串口的原始数据直接输出到电脑串口
  while (gpsSerial.available() > 0) {
    Serial.write(gpsSerial.read());
  }

  // 5秒后检查是否收到数据
  if (millis() > 5000 && gpsSerial.available() == 0) {
    Serial.println(F("警告：未检测到 GPS 模块，请检查 16/17 引脚接线！"));
    delay(2000);
  }
}
