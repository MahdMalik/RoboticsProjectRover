/*
 * @Author: ELEGOO
 * @Date: 2025-01-14 11:59:09
 * @LastEditTime: 2026-01-14 16:07:48
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Hardware Device Drivers Implementation / 智能机器人小车 V4.0 - 硬件设备驱动实现
 */
// Only enable NEC protocol to save flash and RAM / 仅启用 NEC 协议以节省 Flash 和 RAM
#define DECODE_NEC
#include <IRremote.hpp>
#include <Arduino.h>
#include "Utils.h"
#include <MPU6050.h>
#include "Wire.h"

static IMU accelgyro;

/*================ MPU6050 Driver / MPU6050 驱动 =================*/

bool IMU::init(void)
{
  Wire.begin();
  uint8_t chip_id = 0x00;
  uint8_t cout = 0;
  do
  {
    chip_id = accelgyro.getDeviceID();
    delay(10);
    cout += 1;
    if (cout > 10)
    {
      return true;
    }
  } while (chip_id == 0X00 || chip_id == 0XFF); //Ensure that the slave device is online（Wait forcibly to get the ID） / 确保从设备在线（强制等待获取 ID）
  accelgyro.initialize();
  return false;
}

void IMU::calibration(void)
{
  unsigned short times = 100; //Sampling times / 采样次数
  for (int i = 0; i < times; i++)
  {
    gz = accelgyro.getRotationZ();
    gzo += gz;
  }
  gzo /= times; //Calculate gyroscope offset / 计算陀螺仪偏移
}

void IMU::getEulerAngles(float *Yaw)
{
  unsigned long now = millis();           //Record the current time(ms) / 记录当前时间(ms)
  dt = (now - lastTime) / 1000.0;         //Caculate the derivative time(s) / 计算微分时间(s)
  lastTime = now;                         //Record the last sampling time(ms) / 记录最后一次采样时间(ms)
  gz = accelgyro.getRotationZ();          //Read the raw values of the six axes / 读取六轴原始值
  float gyroz = -(gz - gzo) / 131.0 * dt; //z-axis angular velocity / Z 轴角速度
  if (fabs(gyroz) < 0.05)                 //Clear instant zero drift signal / 清除瞬时零漂信号
  {
    gyroz = 0.00;
  }
  agz += gyroz; //z-axis angular velocity integral / Z 轴角速度积分
  *Yaw = agz;
}