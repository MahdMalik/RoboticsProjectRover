/*================ DC Motor Control / 直流电机控制 =================*/

/**
 * @brief Initialize the motor control module / 初始化电机控制模块
 */
void Motor::init()
{
    pinMode(PIN_PWM_RIGHT, OUTPUT);
    pinMode(PIN_PWM_LEFT, OUTPUT);
    pinMode(PIN_RIGHT_IN, OUTPUT);
    pinMode(PIN_LEFT_IN, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);
}


void Motor::control(bool directionRight, uint8_t speedRight,
                                    bool directionLeft, uint8_t speedLeft)
{
    digitalWrite(PIN_STBY, HIGH); // Enable motor driver / 启用电机驱动器
    
    // Motor Right / 右电机
    digitalWrite(PIN_RIGHT_IN, directionRight ? HIGH : LOW); 
    analogWrite(PIN_PWM_RIGHT, speedRight);
    
    // Motor Left / 左电机
    digitalWrite(PIN_LEFT_IN, directionLeft ? HIGH : LOW); 
    analogWrite(PIN_PWM_LEFT, speedLeft);

    // If both motors have zero speed, enter standby mode to save power / 如果两个电机速度都为零，则进入待机模式以节省电力
    if (speedRight == 0 && speedLeft == 0) {
        digitalWrite(PIN_STBY, LOW); 
    }
}