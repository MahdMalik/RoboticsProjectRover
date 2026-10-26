#pragma once
#include "utils.h"

/**
 * @brief ITR20001 Line Tracking Sensor Class / ITR20001 循迹传感器类
 */
class LineTracker {
public:
    /**
     * @brief Initialize the line tracking sensors / 初始化循迹传感器
     */
    void init();

    /**
     * @brief Get left sensor analog value / 获取左侧传感器模拟值
     * @return Analog reading (0-1023) / 模拟读数 (0-1023)
     */
    int getAnalogueL() const;

    /**
     * @brief Get middle sensor analog value / 获取中间传感器模拟值
     * @return Analog reading (0-1023) / 模拟读数 (0-1023)
     */
    int getAnalogueM() const;

    /**
     * @brief Get right sensor analog value / 获取右侧传感器模拟值
     * @return Analog reading (0-1023) / 模拟读数 (0-1023)
     */
    int getAnalogueR() const;

private:
    // Pin configuration / 引脚配置
    static constexpr uint8_t PIN_ITR_L = A2;  ///< Left sensor pin / 左侧传感器引脚
    static constexpr uint8_t PIN_ITR_M = A1;  ///< Middle sensor pin / 中间传感器引脚
    static constexpr uint8_t PIN_ITR_R = A0;  ///< Right sensor pin / 右侧传感器引脚
};