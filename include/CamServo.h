/*
 * @Author: ELEGOO
 * @Date: 2025-01-14 11:59:09
 * @LastEditTime: 2026-01-14 16:07:48
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 Device Driver Set / 智能机器人小车 V4.0 设备驱动集
 */
#pragma once

#include <FastLED.h>
#include <Servo.h>

/**
 * @brief Servo Control Class / 舵机控制类
 */
class DeviceDriverSet_Servo {
public:
    /**
     * @brief Initialize servo motors / 初始化舵机
     * @param positionAngle Initial angle (0-180) / 初始角度 (0-180)
     */
    void init(unsigned int positionAngle);

    /**
     * @brief Control servo position with delay / 控制舵机位置（带延迟）
     * @param positionAngle Target angle (0-180) / 目标角度 (0-180)
     * @param delayTime Delay in ms / 延迟（毫秒）
     */
    void control(unsigned int positionAngle,uint16_t delayTime);

    /**
     * @brief Control multiple servos / 控制多个舵机
     * @param servo Servo selection (0=Z axis, 1=Y axis) / 舵机选择 (0=Z 轴, 1=Y 轴)
     * @param positionAngle Target angle (0-180) / 目标角度 (0-180)
     */
    void controlMultiple(uint8_t servo, unsigned int positionAngle);

    /**
     * @brief Detach all servos to save power / 分离所有舵机以节省电力
     */
    void detachServos();

    /**
     * @brief Set Z-axis servo angle (Non-blocking) / 设置 Z 轴舵机角度（非阻塞）
     * @param positionAngle Target angle (0-180) / 目标角度 (0-180)
     */
    void setZAngle(unsigned int positionAngle);

    /**
     * @brief Get current servo angle / 获取当前舵机角度
     * @return Current angle / 当前角度
     */
    int getCurrentAngle();

    /**
     * @brief Update servo state machine / 更新舵机状态机
     */
    void update();

private:
    static constexpr uint8_t PIN_SERVO_Z = 10;  ///< Z-axis servo pin / Z 轴舵机引脚
    static constexpr uint8_t PIN_SERVO_Y = 11;  ///< Y-axis servo pin / Y 轴舵机引脚
    Servo servoZ_;                              ///< Z-axis servo object / Z 轴舵机对象
    Servo servoY_;                              ///< Y-axis servo object / Y 轴舵机对象

    // State machine for non-blocking control / 非阻塞控制状态机
    enum ServoState {
        IDLE,
        START_Z,
        WAITING_Z,
        START_Y,
        WAITING_Y,
        FINISH
    };
    
    ServoState currentState_ = IDLE;
    NonBlockingTimer timer_;
    uint8_t targetServo_ = 0;
    unsigned int targetAngle_ = 0;
};
