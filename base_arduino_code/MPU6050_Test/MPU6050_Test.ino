// MPU6050 单独测试代码
// 功能：I2C扫描 + 原始数据读取

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // ESP32 SDA=21, SCL=22

  Serial.println("\n=== I2C 扫描 ===");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C 设备地址: 0x");
      Serial.println(address, HEX);
    }
  }
  Serial.println("扫描完成\n");

  Serial.println("=== MPU6050 测试 ===");
  if (!mpu.begin()) { // 自动检测地址 (0x68或0x69)
    Serial.println("MPU6050 未找到!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 连接成功!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("\n原始数据 (每500ms打印一次):");
  Serial.println("accX\taccY\taccZ\t\tgyroX\tgyroY\tgyroZ\t\ttemp");
  Serial.println("---------------------------------------------------------------------------");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print(a.acceleration.x); Serial.print("\t");
  Serial.print(a.acceleration.y); Serial.print("\t");
  Serial.print(a.acceleration.z); Serial.print("\t\t");
  Serial.print(g.gyro.x); Serial.print("\t");
  Serial.print(g.gyro.y); Serial.print("\t");
  Serial.print(g.gyro.z); Serial.print("\t\t");
  Serial.println(temp.temperature);

  delay(500);
}
