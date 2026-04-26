/*
 * MPU6050 传感器读取测试（ESP32 + GY-521）
 * 连接：VCC -> 3.3V, GND -> GND, SDA -> GPIO21, SCL -> GPIO22
 * 串口波特率：115200
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial); // 等待串口打开（可选）

  // 初始化 I2C 总线（ESP32 默认 SDA=21, SCL=22）
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("❌ 未检测到 MPU6050！请检查接线和供电。");
    while (1) {
      delay(100);
    }
  }

  // 设置传感器量程（可选）
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);     // ±8g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);          // ±500°/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);       // 低通滤波 21Hz

  Serial.println("✅ MPU6050 初始化成功，开始输出数据...");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 打印加速度（单位：m/s²）
  Serial.print("加速度 X: "); Serial.print(a.acceleration.x);
  Serial.print("  Y: "); Serial.print(a.acceleration.y);
  Serial.print("  Z: "); Serial.print(a.acceleration.z);
  Serial.print("  (m/s²)  |  ");

  // 打印角速度（单位：rad/s）
  Serial.print("角速度 X: "); Serial.print(g.gyro.x);
  Serial.print("  Y: "); Serial.print(g.gyro.y);
  Serial.print("  Z: "); Serial.print(g.gyro.z);
  Serial.println("  (rad/s)");

  delay(500); // 每 0.5 秒输出一次
}