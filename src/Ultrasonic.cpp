#include "Ultrasonic.h"

/*================ Ultrasonic Distance Sensor / 超声波距离传感器 =================*/
/**
 * @brief Initialize the ultrasonic distance sensor / 初始化超声波距离传感器
 */
void Ultrasonic::init()
{
    pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization / 超声波模块初始化
    pinMode(TRIG_PIN, OUTPUT);
}

/**
 * @brief Measure distance using the ultrasonic sensor / 使用超声波传感器测量距离
 * @param distance Pointer to store the measured distance in centimeters / 用于存储测量距离（厘米）的指针
 */
void Ultrasonic::getDistance(uint16_t *distance) const
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure the width of the echo pulse in microseconds / 测量回声脉冲的宽度（微秒）
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 15000);
        
    // Calculate distance in centimeters / 计算距离（厘米）
    *distance = duration / 58;  // Conversion formula: cm = microseconds / 58 / 转换公式：cm = 微秒 / 58

    // Handle timeout condition / 处理超时情况
    if (duration == 0) {
        *distance = 150;  // Return maximum valid distance on timeout / 超时时返回最大有效距离
    }    

    // Limit to valid range / 限制在有效范围内
    if(*distance > 150) {        // Maximum valid distance: 150cm / 最大有效距离：150cm
        *distance = 150;
    }
    else if(*distance < 2) {     // Minimum valid distance: 2cm / 最小有效距离：2cm
        *distance = 0;
    }
}