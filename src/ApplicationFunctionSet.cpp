/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-06-08 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Application Function Set Implementation / 智能机器人小车 V4.0 - 应用功能集合实现
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet.h"
#include "RgbControl.h"
#include "ButtonDetection.h"
#include "VoltageDetection.h"
#include "MotionControl.h"
#include <SerialComm.h>

// ========== Hardware Device Objects / 硬件设备对象 ==========
IMU AppMPU6050;
RGB_LED AppRGB_LED;
ButtonDetection AppKey;
LineTracker AppITR20001;
VoltageDetection AppVoltage;
Motor AppMotor;
Ultrasonic AppULTRASONIC;
CamServo AppServo;

// ========== Control Objects / 控制对象 ==========
RgbControl rgbControl(AppRGB_LED);
MotionControl motionControl(AppMotor, AppMPU6050, AppULTRASONIC, AppServo, AppITR20001);

SerialComm serial;

// ========== Global Instance / 全局实例 ==========
ApplicationFunctionSet applicationFunctionSet;

// ========== Initialization / 初始化 ==========

void ApplicationFunctionSet::init(unsigned long baudRate) {
    // Initialize hardware components / 初始化硬件组件
    AppVoltage.init();
    AppMotor.init();
    AppServo.init(90);
    AppKey.init();
    AppRGB_LED.init(20);
    AppULTRASONIC.init();
    AppITR20001.init();

    serial.init(baudRate);

    // Initialize MPU6050 sensor / 初始化 MPU6050 传感器
    if (AppMPU6050.init()) {
        //Serial.println("MPU6050 initialization failed!");
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
// ---- settings, tune on the floor ----
// drive and turn motor speed
const uint8_t DRIVE_SPEED = 140;
const uint8_t TURN_SPEED = 150;
// run time in milliseconds per cm driven
const unsigned long MS_PER_CM = 40;
// run time in milliseconds per degree turned
const unsigned long MS_PER_DEGREE = 14;
const unsigned long MAX_RUN_MS = 5000;
const unsigned long LOAD_TIMEOUT_MS = 200;
const uint8_t MAX_STEPS = 8;
const float TURN_COAST_DEG = 2.0;

const uint8_t FAIL_BAD_BATCH = 1;
const uint8_t FAIL_LOAD_TIMEOUT = 2;
const uint8_t FAIL_SAFETY_STOP = 3;

// ---- gyro hooks ----
// Stubs for now, so every turn is timed. Fill in later with AppMPU6050.getEulerAngles().
static bool gyroTried = false;
static bool gyroReady = false;
static bool turnTracking = false;
static float turnAccumDeg = 0;

static void gyroInit()
{
    gyroTried = true;
    gyroReady = false;
}

static void gyroTick()
{
}

// ---- plan queue and motion ----
struct PlanStep
{
    uint8_t command;
    uint8_t amount;
};

static PlanStep plan[MAX_STEPS];
static uint8_t planCount = 0;
static uint8_t planNext = 0;
static uint8_t stepsDone = 0;
static bool planActive = false;

static bool loading = false;
static uint8_t loadExpected = 0;
static uint8_t loadGot = 0;
static unsigned long loadDeadlineMs = 0;

static bool motionActive = false;
static bool motionIsPlanStep = false;
static bool motionBounded = false;
static bool motionIsTurn = false;
static uint8_t motionCommand = 0;
static uint8_t motionAmount = 0;
static unsigned long motionEndMs = 0;

static bool partialOpen = false;
static unsigned long partialSinceMs = 0;

static void stopMotors()
{
    motionControl.SmartRobotCarMotionControl(CarDirection::Stop, 0);
    motionActive = false;
    turnTracking = false;
}

// drops the running plan and any motion, reports it if a plan was running
static void preempt()
{
    if (planActive)
    {
        serial.SendData(SentPacketType::BATCH_CANCELLED, stepsDone);
    }
    planActive = false;
    planCount = 0;
    planNext = 0;
    stepsDone = 0;
    stopMotors();
}

static bool amountIsValid(uint8_t command, uint8_t amount, bool allowZero)
{
    if (amount == 0)
    {
        return allowZero;
    }
    if (command == (uint8_t) ReceivedPacketType::TURN_LEFT || command == (uint8_t) ReceivedPacketType::TURN_RIGHT)
    {
        return amount <= 180;
    }
    return true;
}

static bool isMoveCommand(uint8_t command)
{
    return command == (uint8_t) ReceivedPacketType::MOVE_FORWARD
        || command == (uint8_t) ReceivedPacketType::MOVE_BACKWARD
        || command == (uint8_t) ReceivedPacketType::TURN_LEFT
        || command == (uint8_t) ReceivedPacketType::TURN_RIGHT;
}

static void beginMove(uint8_t command, uint8_t amount, bool isPlanStep)
{
    CarDirection direction = CarDirection::Forward;
    uint8_t speed = DRIVE_SPEED;
    unsigned long msPerUnit = MS_PER_CM;
    bool isTurn = false;

    if (command == (uint8_t) ReceivedPacketType::MOVE_BACKWARD)
    {
        direction = CarDirection::Backward;
    }
    else if (command == (uint8_t) ReceivedPacketType::TURN_LEFT)
    {
        direction = CarDirection::Left;
        speed = TURN_SPEED;
        msPerUnit = MS_PER_DEGREE;
        isTurn = true;
    }
    else if (command == (uint8_t) ReceivedPacketType::TURN_RIGHT)
    {
        direction = CarDirection::Right;
        speed = TURN_SPEED;
        msPerUnit = MS_PER_DEGREE;
        isTurn = true;
    }

    motionControl.SmartRobotCarMotionControl(direction, speed);
    motionActive = true;
    motionIsPlanStep = isPlanStep;
    motionIsTurn = isTurn;
    motionCommand = command;
    motionAmount = amount;

    turnTracking = false;
    if (isTurn && gyroReady)
    {
        turnAccumDeg = 0;
        turnTracking = true;
    }

    if (amount == 0)
    {
        // run until STOP, safety stop after MAX_RUN_MS
        motionBounded = false;
        motionEndMs = millis() + MAX_RUN_MS;
        serial.SendData(SentPacketType::ACTION_SUCCESS, command);
    }
    else
    {
        motionBounded = true;
        unsigned long runMs = (unsigned long) amount * msPerUnit;
        if (turnTracking)
        {
            // gyro should end the turn first, the time is only a cap
            runMs = runMs * 3 / 2;
        }
        motionEndMs = millis() + runMs;
    }
}

static void startNextStep()
{
    PlanStep step = plan[planNext];
    planNext++;
    beginMove(step.command, step.amount, true);
}

static void finishMotion(bool ok)
{
    bool wasTurn = motionIsTurn;
    bool wasTracking = turnTracking;
    stopMotors();

    if (wasTurn && wasTracking)
    {
        float measured = turnAccumDeg;
        if (measured > 255)
        {
            measured = 255;
        }
        serial.SendData(SentPacketType::TURN_MEASURED, (uint8_t) measured);
    }

    if (motionIsPlanStep)
    {
        stepsDone++;
        serial.SendData(SentPacketType::ACTION_SUCCESS, stepsDone);
        if (planNext < planCount)
        {
            startNextStep();
        }
        else
        {
            serial.SendData(SentPacketType::BATCH_DONE, planCount);
            planActive = false;
        }
    }
    else if (ok)
    {
        serial.SendData(SentPacketType::ACTION_SUCCESS, motionCommand);
    }
    else
    {
        serial.SendData(SentPacketType::ACTION_FAILED, FAIL_SAFETY_STOP);
    }
}

// reads one 2 byte packet, drops a lone byte that waits longer than the load timeout
static bool readPacket(uint8_t &type, uint8_t &param)
{
    int available = Serial.available();
    if (available >= 2)
    {
        type = (uint8_t) Serial.read();
        param = (uint8_t) Serial.read();
        partialOpen = false;
        return true;
    }
    if (available == 1)
    {
        if (!partialOpen)
        {
            partialOpen = true;
            partialSinceMs = millis();
        }
        else if (millis() - partialSinceMs >= LOAD_TIMEOUT_MS)
        {
            Serial.read();
            partialOpen = false;
        }
    }
    else
    {
        partialOpen = false;
    }
    return false;
}

static void abortLoading(uint8_t reason)
{
    loading = false;
    planCount = 0;
    serial.SendData(SentPacketType::ACTION_FAILED, reason);
}

static void handleLoadPacket(uint8_t type, uint8_t param)
{
    if (type == (uint8_t) ReceivedPacketType::HANDSHAKE)
    {
        serial.SendData(SentPacketType::HANDSHAKE_RESPONSE, param);
        return;
    }
    if (type == (uint8_t) ReceivedPacketType::STOP)
    {
        loading = false;
        planCount = 0;
        serial.SendData(SentPacketType::ACTION_SUCCESS, (uint8_t) ReceivedPacketType::STOP);
        return;
    }
    if (!isMoveCommand(type) || !amountIsValid(type, param, false))
    {
        abortLoading(FAIL_BAD_BATCH);
        return;
    }

    plan[loadGot].command = type;
    plan[loadGot].amount = param;
    loadGot++;
    loadDeadlineMs = millis() + LOAD_TIMEOUT_MS;

    if (loadGot == loadExpected)
    {
        loading = false;
        planCount = loadExpected;
        planNext = 0;
        stepsDone = 0;
        planActive = true;
        serial.SendData(SentPacketType::BATCH_ACCEPTED, planCount);
        startNextStep();
    }
}

void ApplicationFunctionSet::checkSerial()
{
    // gyro setup runs once, at boot while the car is still (still need to test this later guys...)
    if (!gyroTried)
    {
        gyroInit();
    }

    // a partial batch that stalls is trashed and triggers a load timeout failure
    if (loading && (long) (millis() - loadDeadlineMs) >= 0)
    {
        abortLoading(FAIL_LOAD_TIMEOUT);
    }

    // end the running move
    if (motionActive)
    {
        gyroTick();
        bool done = false;
        if (turnTracking && motionBounded && turnAccumDeg >= (float) motionAmount - TURN_COAST_DEG)
        {
            done = true;
        }
        else if ((long) (millis() - motionEndMs) >= 0)
        {
            done = true;
        }
        if (done)
        {
            finishMotion(motionBounded);
        }
    }

    uint8_t type = 0;
    uint8_t param = 0;
    if (!readPacket(type, param))
    {
        return;
    }

    if (loading && type != (uint8_t) ReceivedPacketType::BATCH_BEGIN)
    {
        handleLoadPacket(type, param);
        return;
    }

    switch ((ReceivedPacketType) type)
    {
        case ReceivedPacketType::HANDSHAKE:
        {
            serial.SendData(SentPacketType::HANDSHAKE_RESPONSE, param);
            break;
        }
        case ReceivedPacketType::MOVE_FORWARD:
        case ReceivedPacketType::MOVE_BACKWARD:
        case ReceivedPacketType::TURN_LEFT:
        case ReceivedPacketType::TURN_RIGHT:
        {
            if (!amountIsValid(type, param, true))
            {
                serial.SendData(SentPacketType::ACTION_FAILED, FAIL_BAD_BATCH);
                break;
            }
            currentState = State::DoNothing;
            loading = false;
            preempt();
            beginMove(type, param, false);
            break;
        }
        case ReceivedPacketType::BATCH_BEGIN:
        {
            currentState = State::DoNothing;
            loading = false;
            preempt();
            if (param < 1 || param > MAX_STEPS)
            {
                serial.SendData(SentPacketType::ACTION_FAILED, FAIL_BAD_BATCH);
                break;
            }
            loading = true;
            loadExpected = param;
            loadGot = 0;
            loadDeadlineMs = millis() + LOAD_TIMEOUT_MS;
            break;
        }
        case ReceivedPacketType::STOP:
        {
            currentState = State::DoNothing;
            loading = false;
            preempt();
            serial.SendData(SentPacketType::ACTION_SUCCESS, (uint8_t) ReceivedPacketType::STOP);
            break;
        }
        default:
        {
            break;
        }
    }
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
