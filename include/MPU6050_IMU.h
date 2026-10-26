#pragma once
#include "Utils.h"

/**
 * @brief MPU6050 Sensor Class / MPU6050 传感器类
 */
class MPU6050_IMU {
public:
    /**
     * @brief Initialize MPU6050 sensor / 初始化 MPU6050 传感器
     * @return true if initialization failed, false if successful / 初始化失败返回 true，成功返回 false
     */
    bool init();

    /**
     * @brief Calibrate MPU6050 gyro / 校准 MPU6050 陀螺仪
     */
    void calibration();

    /**
     * @brief Get current Euler angles (Yaw) / 获取当前欧拉角 (Yaw)
     * @param Yaw Pointer to store the calculated Yaw angle / 用于存储计算出的偏航角的指针
     */
    void getEulerAngles(float *Yaw);

public:
    int16_t gz;
    unsigned long now, lastTime = 0;
    float dt;      // Derivative time / 微分时间
    float agz = 0; // Angle variable / 角度变量
    long gzo = 0;  // Gyro offset / 陀螺仪偏移
};

