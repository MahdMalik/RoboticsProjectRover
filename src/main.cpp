/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-01-9 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Main Program 
 * @version 2.1.2  
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet.h"
#include <Arduino.h>

// ========== Serial Communication Constants / 串行通信常量 ==========
#define MAX_MSG_LENGTH 128   // Maximum message length / 最大消息长度
#define START_CHAR '{'       // JSON start identifier / JSON 开始标识符
#define END_CHAR '}'         // JSON end identifier / JSON 结束标识符
#define TIMEOUT_MS 1000      // Receive timeout in milliseconds / 接收超时（毫秒）
#define BAUD_RATE 115200

// ========== Global Objects / 全局对象 ==========
extern ApplicationFunctionSet applicationFunctionSet;



/* Arduino initialization / Arduino 初始化 */
void setup() {
    // Initialize application components / 初始化应用程序组件
    applicationFunctionSet.init(BAUD_RATE);
    
    // Enable watchdog timer for system stability / 启用看门狗定时器以保持系统稳定性
    wdt_enable(WDTO_4S);
    
    // Initialize serial communication / 初始化串行通信
    
    
    // Print startup message / 打印启动消息
    //Serial.println(F("Smart Robot Car V4.0 - System Ready"));
}

int timer = 0;

/* Main program loop / 主程序循环 */
void loop() {
    // Reset watchdog timer to prevent system reset / 重置看门狗定时器以防止系统复位
    wdt_reset();
    
    // Update sensor data and process inputs / 更新传感器数据并处理输入
    applicationFunctionSet.updateSensorData();

    applicationFunctionSet.checkSerial();
    
    // Update timers and execute current state / 更新定时器并执行当前状态
    applicationFunctionSet.updateTimer();
    applicationFunctionSet.checkState();

    timer ++;

    if(timer == 1000)
    {
        //Serial.println("UPdate!");
    }
}