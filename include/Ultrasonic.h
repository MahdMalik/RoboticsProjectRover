#pragma once
#include "Utils.h"

/**
 * @brief Ultrasonic Sensor Class / 超声波传感器类
 */
class Ultrasonic {
public:
    /**
     * @brief Initialize the ultrasonic sensor / 初始化超声波传感器
     */
    void init();

    /**
     * @brief Measure distance to obstacle / 测量障碍物距离
     * @param distance Pointer to store distance (cm) / 用于存储距离（厘米）的指针
     */
    void getDistance(uint16_t* distance) const;

private:
    static constexpr uint8_t TRIG_PIN = 13;  ///< Trigger pin / 触发引脚
    static constexpr uint8_t ECHO_PIN = 12;  ///< Echo pin / 回声引脚
};
