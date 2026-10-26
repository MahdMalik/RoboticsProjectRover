#pragma once
#include "Utils.h"

/**
 * @brief Motor Control Class / 电机控制类
 */
class Motor {
public:
    /**
     * @brief Initialize the motor control module / 初始化电机控制模块
     */
    void init();

    /**
     * @brief Control both motors / 控制两个电机
     * @param directionRight Motor Right direction / 右电机方向
     * @param speedRight Motor Right speed (0-255) / 右电机速度 (0-255)
     * @param directionLeft Motor Left direction / 左电机方向
     * @param speedLeft Motor Left speed (0-255) / 左电机速度 (0-255)
     */
    void control(bool directionRight, uint8_t speedRight,
                bool directionLeft, uint8_t speedLeft);

private:
    // Pin definitions / 引脚定义
    static constexpr uint8_t PIN_PWM_RIGHT = 5;    ///< Motor Right PWM / 右电机 PWM
    static constexpr uint8_t PIN_PWM_LEFT = 6;    ///< Motor Left PWM / 左电机 PWM
    static constexpr uint8_t PIN_LEFT_IN = 8;   ///< Motor Left direction / 左电机方向
    static constexpr uint8_t PIN_RIGHT_IN = 7;   ///< Motor Right direction / 右电机方向
    static constexpr uint8_t PIN_STBY = 3;    ///< Standby pin / 待机引脚

};

