/*
 * @Author: ELEGOO
 * @Date: 2025-01-14 11:59:09
 * @LastEditTime: 2026-01-14 16:07:48
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Hardware Device Drivers Implementation / 智能机器人小车 V4.0 - 硬件设备驱动实现
 */
#include "DeviceDriverSet.h"
// Only enable NEC protocol to save flash and RAM / 仅启用 NEC 协议以节省 Flash 和 RAM
#define DECODE_NEC
#include <IRremote.hpp>
#include <Arduino.h>
#include "Utils.h"
#include <MPU6050.h>
#include "Wire.h"

/*================ Servo Motor Control / 舵机控制 =================*/

/**
 * @brief Initialize the servo motors / 初始化舵机
 * @param positionAngle Initial position angle (0-180 degrees) / 初始位置角度（0-180 度）
 */
void CamServo::init(unsigned int positionAngle)
{
    // Initialize servos / 初始化舵机
    servoZ_.attach(PIN_SERVO_Z, 500, 2400); // Z-axis servo with custom pulse width range / 具有自定义脉冲宽度范围的 Z 轴舵机
    servoZ_.write(positionAngle);
    Utils::delayWithWatchdog(400); // Wait for servo to reach position / 等待舵机到达位置
    
    detachServos(); // Detach to prevent jitter and save power / 分离以防止抖动并节省电力
}

/**
 * @brief Control the Z-axis servo position / 控制 Z 轴舵机位置
 * @param positionAngle Target position angle (0-180 degrees) / 目标位置角度（0-180 度）
 * @param delayTime Time to wait for the servo to reach position (milliseconds) / 等待舵机到达位置的时间（毫秒）
 */
void CamServo::control(unsigned int positionAngle,uint16_t delayTime)
{
    servoZ_.attach(PIN_SERVO_Z);
    servoZ_.write(positionAngle);
    Utils::delayWithWatchdog(delayTime); // Use blocking delay with watchdog / 使用带看门狗的阻塞延迟
    servoZ_.detach(); // Detach to prevent jitter and save power / 分离以防止抖动并节省电力
}

/**
 * @brief Set Z-axis servo angle without delay / 无延迟设置 Z 轴舵机角度
 * @param positionAngle Target angle position (0-180) / 目标角度位置 (0-180)
 */
void CamServo::setZAngle(unsigned int positionAngle)
{
    servoZ_.attach(PIN_SERVO_Z);
    servoZ_.write(positionAngle);
}

/**
 * @brief Get the current angle of the Z-axis servo / 获取 Z 轴舵机的当前角度
 * @return Current servo angle (0-180 degrees) / 当前舵机角度（0-180 度）
 */
int CamServo::getCurrentAngle(){
    return servoZ_.read();
}

/**
 * @brief Control multiple servos with a simplified angle system / 使用简化的角度系统控制多个舵机
 * @param servo Servo selection (1=Z-axis, 2=Y-axis, 3=both) / 舵机选择 (1=Z 轴, 2=Y 轴, 3=两者)
 * @param positionAngle Simplified angle value (multiplied by 10 internally) / 简化角度值（内部乘以 10）
 */
void CamServo::controlMultiple(uint8_t servo, unsigned int positionAngle)
{
    targetServo_ = servo;
    targetAngle_ = positionAngle;
    currentState_ = START_Z; // Start the sequence / 开始序列
}

/**
 * @brief Update servo state machine / 更新舵机状态机
 */
void CamServo::update()
{
    switch (currentState_) {
        case IDLE:
            break;
            
        case START_Z:
            if (targetServo_ == 1 || targetServo_ == 3) {
                unsigned int angle = constrain(targetAngle_, 1, 17);
                servoZ_.attach(PIN_SERVO_Z);
                servoZ_.write(10 * angle);
                timer_.start(500);
                currentState_ = WAITING_Z;
            } else {
                currentState_ = START_Y; // Skip Z / 跳过 Z
            }
            break;
            
        case WAITING_Z:
            if (timer_.isExpired()) {
                currentState_ = START_Y;
            }
            break;
            
        case START_Y:
            if (targetServo_ == 2 || targetServo_ == 3) {
                unsigned int angle = constrain(targetAngle_, 3, 11);
                servoY_.attach(PIN_SERVO_Y);
                servoY_.write(10 * angle);
                timer_.start(500);
                currentState_ = WAITING_Y;
            } else {
                currentState_ = FINISH; // Skip Y / 跳过 Y
            }
            break;
            
        case WAITING_Y:
            if (timer_.isExpired()) {
                currentState_ = FINISH;
            }
            break;
            
        case FINISH:
            detachServos();
            currentState_ = IDLE;
            break;
    }
}

/**
 * @brief Detach all servos to prevent jitter and save power / 分离所有舵机以防止抖动并节省电力
 */
void CamServo::detachServos()
{
    servoZ_.detach();
    servoY_.detach();
}