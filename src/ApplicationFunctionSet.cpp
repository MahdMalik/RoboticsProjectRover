/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-06-08 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Application Function Set Implementation / 智能机器人小车 V4.0 - 应用功能集合实现
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet.h"
#include "DeviceDriverSet.h"
#include "RgbControl.h"
#include "MotionControl.h"

// ========== Hardware Device Objects / 硬件设备对象 ==========
DeviceDriverSet_MPU6050 AppMPU6050;
DeviceDriverSet_RGBLED AppRGB_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_Ultrasonic AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;

// ========== Control Objects / 控制对象 ==========
RgbControl rgbControl(AppRGB_LED);
MotionControl motionControl(AppMotor, AppMPU6050, AppULTRASONIC, AppServo, AppITR20001);

// ========== Global Instance / 全局实例 ==========
ApplicationFunctionSet applicationFunctionSet;

// ========== Initialization / 初始化 ==========

void ApplicationFunctionSet::init() {
    // Initialize hardware components / 初始化硬件组件
    AppVoltage.init();
    AppMotor.init();
    AppServo.init(90);
    AppKey.init();
    AppRGB_LED.init(20);
    AppIRrecv.init();
    AppULTRASONIC.init();
    AppITR20001.init();

    // Initialize MPU6050 sensor / 初始化 MPU6050 传感器
    if (AppMPU6050.init()) {
        Serial.println("MPU6050 initialization failed!");
    } else {
        AppMPU6050.calibration();
    }
    
    // Initialize motor state / 初始化电机状态
    motorState.directionRight = true;      // Forward / 前进
    motorState.speedRight = 0;             // Stop / 停止
    motorState.directionLeft = true;      // Forward / 前进
    motorState.speedLeft = 0;             // Stop / 停止
    motorState.isInitialized = true;   // Initialized / 已初始化

    // Set initial state / 设置初始状态
    carDirection = CarDirection::Stop;
    currentState = State::Standby;
    isIrRobotControl = false;
}

// ========== Core State Management / 核心状态管理 ==========

void ApplicationFunctionSet::checkState() {
    // State transition handling / 状态转换处理
    if (currentState != lastState) {
        if (currentState == State::Standby) {
            AppServo.control(90, 200); // Reset servo to 90 degrees when entering Standby / 进入待机模式时将舵机重置为 90 度
        }
        lastState = currentState;
    }

    static unsigned long lastRgbUpdate = 0;
    if (millis() - lastRgbUpdate > 100) { // Update LEDs every 100ms / 每 100ms 更新一次 LED
        // Update RGB LED based on state and voltage / 根据状态和电压更新 RGB LED
        rgbControl.update(voltageDetectionStatus, currentState);
        lastRgbUpdate = millis();
    }
    
    // Update servo state machine / 更新舵机状态机
    AppServo.update();

    // Execute current state function / 执行当前状态功能
    switch (currentState) {
        case State::Standby:
            executeStandbyMode();
            break;
        default:
            break;
    }
}

void ApplicationFunctionSet::updateSensorData() {
    static uint32_t lastUpdate = 0;
    static uint8_t lowVoltageCounter = 0;
    
    const uint32_t UPDATE_INTERVAL_MS = 10;
    const uint8_t LOW_VOLTAGE_THRESHOLD = 50;

    uint32_t currentTime = millis();
    if (currentTime - lastUpdate < UPDATE_INTERVAL_MS) {
        return;
    }
    lastUpdate = currentTime;

    // Update battery voltage status / 更新电池电压状态
    noInterrupts();
    uint16_t voltage = AppVoltage.getVoltage();
    interrupts();

    if (voltage < VOLTAGE_THRESHOLD) {
        if (lowVoltageCounter < 255) {
            lowVoltageCounter++;
        }
        if (lowVoltageCounter >= LOW_VOLTAGE_THRESHOLD) {
            voltageDetectionStatus = true;
        }
    } else {
        lowVoltageCounter = 0;
        voltageDetectionStatus = false;
    }

    // Update line tracking sensor data / 更新循迹传感器数据
    trackingDataRight = AppITR20001.getAnalogueR();
    trackingDataMiddle = AppITR20001.getAnalogueM();
    trackingDataLeft = AppITR20001.getAnalogueL();

    // Check if robot is on ground / 检查机器人是否在地面上
    bool isAboveGround = (trackingDataRight > TRACKING_DETECTION_THRESHOLD) &&
                        (trackingDataMiddle > TRACKING_DETECTION_THRESHOLD) &&
                        (trackingDataLeft > TRACKING_DETECTION_THRESHOLD);

    // Reset target yaw when returning to ground / 返回地面时重置目标偏航角
    if (!isOnGround && !isAboveGround) {
        motionControl.UpdateTargetYAW();
    }

    isOnGround = !isAboveGround;
}

void ApplicationFunctionSet::executeStandbyMode() {
    motionControl.SmartRobotCarMotionControl(CarDirection::Stop, 0);
}

// ========== Servo Control / 舵机控制 ==========

void ApplicationFunctionSet::controlServo(uint8_t servoCommand) {
    if (servoCommand < 1 || servoCommand > 5) {
        return;
    }

    static uint8_t zAxisAngle = 9;  // Z axis angle / Z 轴角度
    static uint8_t yAxisAngle = 9;  // Y axis angle / Y 轴角度
    
    const uint8_t Y_AXIS_MIN = 3;
    const uint8_t Y_AXIS_MAX = 11;
    const uint8_t Z_AXIS_MIN = 1;
    const uint8_t Z_AXIS_MAX = 17;
    const uint8_t CENTER_POSITION = 9;

    switch (servoCommand) {
        case 1: // Y axis down / Y 轴向下
            yAxisAngle = constrain(yAxisAngle - 1, Y_AXIS_MIN, Y_AXIS_MAX);
            AppServo.controlMultiple(2, yAxisAngle);
            break;
        case 2: // Y axis up / Y 轴向上
            yAxisAngle = constrain(yAxisAngle + 1, Y_AXIS_MIN, Y_AXIS_MAX);
            AppServo.controlMultiple(2, yAxisAngle);
            break;
        case 3: // Z axis right / Z 轴向右
            zAxisAngle = constrain(zAxisAngle + 1, Z_AXIS_MIN, Z_AXIS_MAX);
            AppServo.controlMultiple(1, zAxisAngle);
            break;
        case 4: // Z axis left / Z 轴向左
            zAxisAngle = constrain(zAxisAngle - 1, Z_AXIS_MIN, Z_AXIS_MAX);
            AppServo.controlMultiple(1, zAxisAngle);
            break;
        case 5: // Reset to center / 重置到中心
            yAxisAngle = CENTER_POSITION;
            zAxisAngle = CENTER_POSITION;
            AppServo.controlMultiple(2, CENTER_POSITION);
            AppServo.controlMultiple(1, CENTER_POSITION);
            break;
    }
}

// ========== Motor and Movement Control / 电机和移动控制 ==========

void ApplicationFunctionSet::processMotorControl(uint8_t motorSelection, uint8_t motorSpeed, uint8_t motorDirection) {
    
    currentState = State::DoNothing;

    // Stop all motors if direction is 0 / 如果方向为 0，停止所有电机
    if (motorDirection == 0) {
        motorState.speedRight = 0;
        motorState.speedLeft = 0;
        AppMotor.control(motorState.directionRight, motorState.speedRight, 
                        motorState.directionLeft, motorState.speedLeft);
        sendCommandResponse(true);
        return;
    }

    // Determine new direction (true = forward, false = backward) / 确定新方向（true = 前进，false = 后退）
    bool newDirection = (motorDirection & 0x01) != 0;

    // Update motor state based on selection / 根据选择更新电机状态
    switch (motorSelection) {
        case 0: // Both motors / 两个电机
            motorState.directionRight = newDirection;
            motorState.speedRight = motorSpeed;
            motorState.directionLeft = newDirection;
            motorState.speedLeft = motorSpeed;
            break;
            
        case 1: // Left motor only / 仅左电机
            motorState.directionLeft = newDirection;
            motorState.speedLeft = motorSpeed;
            break;
            
        case 2: // Right motor only / 仅右电机
            motorState.directionRight = newDirection;
            motorState.speedRight = motorSpeed;
            break;
            
        default:
            sendCommandResponse(true);
            return;
    }

    // Apply the current motor state to hardware / 将当前电机状态应用到硬件
    AppMotor.control(motorState.directionRight, motorState.speedRight, 
                    motorState.directionLeft, motorState.speedLeft);
    sendCommandResponse(true);
}

void ApplicationFunctionSet::processMotorSpeedControl(uint8_t leftSpeed, uint8_t rightSpeed) {
    // Update motor state / 更新电机状态
    motorState.speedRight = rightSpeed;
    motorState.speedLeft = leftSpeed;
    
    if (leftSpeed == 0 && rightSpeed == 0) {
        // Both motors stopped - use existing directions / 两个电机停止 - 使用现有方向
        AppMotor.control(motorState.directionRight, motorState.speedRight, 
                        motorState.directionLeft, motorState.speedLeft);
    } else {
        // Motors running - assume forward direction for both / 电机运行 - 假设两个都为前进方向
        motorState.directionRight = true;
        motorState.directionLeft = true;
        AppMotor.control(motorState.directionRight, motorState.speedRight, 
                        motorState.directionLeft, motorState.speedLeft);
    }
}

void ApplicationFunctionSet::processCarControl(uint8_t direction, uint8_t speed) {
    switch (direction) {
        case 1: motionControl.SmartRobotCarMotionControl(Left, speed); break;
        case 2: motionControl.SmartRobotCarMotionControl(Right, speed); break;
        case 3: motionControl.SmartRobotCarMotionControl(Forward, speed); break;
        case 4: motionControl.SmartRobotCarMotionControl(Backward, speed); break;
        default: break;
    }
}

// ========== Command Handlers / 命令处理器 ==========

void ApplicationFunctionSet::handleCarControlCommand(uint8_t direction, uint8_t speed, unsigned long duration) {
    currentState = State::DoNothing;
    
    processCarControl(direction, speed);
    
    if (duration != 2) {
        // Timed command / 定时命令
        startTimer(duration, this, &ApplicationFunctionSet::carControlCallback);
    } else {
        sendCommandResponse(true);
    }
}

void ApplicationFunctionSet::handleLightingCommand(uint8_t sequence, uint8_t red, uint8_t green, uint8_t blue, unsigned long duration) {
    currentState = State::DoNothing;
    rgbControl.executeLightingCommand(sequence, red, green, blue);
    
    if (duration != 0) {
        // Timed command / 定时命令
        startTimer(duration, this, &ApplicationFunctionSet::lightingCallback);
    } else {
        sendCommandResponse(true);
    }
}

// ========== Sensor Status Functions / 传感器状态功能 ==========

void ApplicationFunctionSet::getUltrasonicStatus(uint8_t sensorId) {
    uint16_t distance;
    AppULTRASONIC.getDistance(&distance);
    bool obstacleDetected = isWithinRange(distance, 0, OBSTACLE_DETECTION_THRESHOLD);
    
    if (sensorId == 1) {
        // Return obstacle detection status / 返回障碍物检测状态
        sendResponseData(obstacleDetected ? "true" : "false");
    } else if (sensorId == 2) {
        // Return distance data / 返回距离数据
        char distanceStr[10];
        sprintf(distanceStr, "%d", distance);
        sendResponseData(distanceStr);
    }
}

void ApplicationFunctionSet::getTrackingSensorStatus(uint8_t sensorId) {
    char valueStr[5];
    volatile int sensorValue;
    
    switch (sensorId) {
        case 0: sensorValue = trackingDataLeft; break;
        case 1: sensorValue = trackingDataMiddle; break;
        case 2: sensorValue = trackingDataRight; break;
        default: sensorValue = 0; break;
    }
    
    sprintf(valueStr, "%d", sensorValue);
    sendResponseData(valueStr);
}

// ========== Utility Functions / 实用功能 ==========

void ApplicationFunctionSet::clearAllFunctions(State newState) {
    // Reset motor state / 重置电机状态
    motorState.directionRight = true;
    motorState.speedRight = 0;
    motorState.directionLeft = true;
    motorState.speedLeft = 0;
    
    // Stop motors and clear LEDs / 停止电机并清除 LED
    motionControl.SmartRobotCarMotionControl(CarDirection::Stop, 0);
    AppRGB_LED.clear();
    carDirection = CarDirection::Stop;
    currentState = newState;
    isIrRobotControl = false;
}

bool ApplicationFunctionSet::isWithinRange(long value, long start, long end) {
    return (value >= start && value <= end);
}

// ========== Communication Functions / 通信功能 ==========

void ApplicationFunctionSet::sendCommandResponse(bool includeSerialNumber) {
    if (includeSerialNumber && commandSerialNumber) {
        Serial.write("{");
        Serial.write(commandSerialNumber);
        Serial.write("_ok}");
    } 
}

void ApplicationFunctionSet::sendResponseData(const char* data) {
    Serial.write("{");
    Serial.write(commandSerialNumber);
    Serial.write("_");
    Serial.write(data);
    Serial.write("}");
}

// ========== Timer Functions / 定时器功能 ==========

void ApplicationFunctionSet::startTimer(unsigned long durationMs, ApplicationFunctionSet* instance, 
                                       void (ApplicationFunctionSet::*callback)()) {
    timerStart = millis();
    timerDuration = durationMs;
    timerInstance = instance;
    timerCallback = callback;
    isTimerActive = true;
}

void ApplicationFunctionSet::updateTimer() {
    if (isTimerActive && (millis() - timerStart >= timerDuration)) {
        isTimerActive = false;
        if (timerInstance && timerCallback) {
            (timerInstance->*timerCallback)();
        }
    }
}

// ========== Callback Functions / 回调功能 ==========

void ApplicationFunctionSet::carControlCallback() {
    motionControl.SmartRobotCarMotionControl(CarDirection::Stop, 0);
    sendCommandResponse(true);
}

void ApplicationFunctionSet::lightingCallback() {
    AppRGB_LED.clear();
    sendCommandResponse(true);
}
