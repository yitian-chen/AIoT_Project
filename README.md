# 基于ESP32与华为云的智能骑行导航与监护系统

## 项目概述

本系统是一款面向骑行通勤的大学生的智能设备，实现课表导航、摔倒检测与报警、骑行数据统计、防盗模式等功能，通过华为云IoT平台实现ESP32与微信小程序的双向通信。

## 硬件架构

| 模块 | 型号 | 功能 |
|------|------|------|
| 主控 | ESP32 | WiFi连接、数据处理 |
| GPS | NEO-6M | 定位 |
| 加速度计 | MPU6050 | 摔倒检测、姿态角计算 |
| 显示 | SSD1306 OLED 128x64 | 显示导航指令、传感器数据 |
| 蜂鸣器 | 有源蜂鸣器 | 报警提示 |

## 功能模块

### 1. 智能课表导航
- ESP32 订阅华为云主题 `$oc/devices/{device_id}/sys/commands/#`
- 小程序通过云函数下发课表 JSON，ESP32 存储并定时触发导航
- 到达距离小于20米时提醒

### 2. 小程序实时导航
- 小程序通过华为云事件下发导航指令到 `$oc/devices/{device_id}/sys/events/down`
- JSON 格式：
```json
{
  "event_type": "NavigationCommand",
  "paras": {
    "navInstruction": "\\u5de6\\u8f6c",
    "navDistance": 200,
    "navStatus": "navigating"
  }
}
```
- ESP32 将中文Unicode解码并转换为英文显示（LEFT/RIGHT/AHEAD/UTURN）
- OLED 优先显示导航指令

### 3. 摔倒检测
- 使用 MPU6050 检测加速度和陀螺仪数据
- 检测条件：合加速度 > 3g + 俯仰角 > 55度，持续2秒 + 陀螺仪异常
- 摔倒确认后发送事件到华为云 `$oc/devices/{device_id}/sys/events`
- OLED 显示警告信息

### 4. 数据上报
- 每5秒上报一次属性到华为云：
  - 位置（纬度、经度）
  - 速度、高度
  - 加速度XYZ
  - 姿态角（pitch、roll、yaw）
  - 时间戳

### 5. OLED 显示
- 导航模式：显示导航指令英文
- 正常模式：显示 GPS状态、经纬度、速度、MPU数据、摔倒状态

## 华为云配置

- MQTT Broker: `1fc5f68721.st1.iotda-device.cn-east-3.myhuaweicloud.com`
- 端口: 1883
- 设备ID: `69ae7ce618855b39c5010ef5_myArduino`

## 文件结构

```
base_arduino_code/
├── base_arduino_code.ino    # 主程序

```

## 使用说明

### 烧录固件
1. 使用 Arduino IDE 打开 `base_arduino_code.ino`
2. 选择 ESP32 开发板
3. 上传

### 配置WiFi
在代码中修改：
```cpp
const char *WIFI_SSID = "你的WiFi名称";
const char *WIFI_PASS = "你的WiFi密码";
```