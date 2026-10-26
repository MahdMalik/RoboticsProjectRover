#pragma once
#include "utils.h"

/**
 * @brief Button Detection Class / 按键检测类
 */
class ButtonDetection {
public:
    /**
     * @brief Initialize the button detection module / 初始化按键检测模块
     */
    void init();

    /**
     * @brief Get the current button state / 获取当前按键状态
     * @param get_keyValue Pointer to store the button value / 用于存储按键值的指针
     */
    void getKeyValue(uint8_t *get_keyValue);
    
public:
    static constexpr uint8_t PIN_KEY = 2;         ///< Button input pin / 按键输入引脚
    static constexpr uint8_t KEY_VALUE_MAX = 3;   ///< Maximum button value / 按键状态最大值
    static uint8_t keyValue_;                     ///< Current button value / 当前按键值
};
