/*================ Battery Voltage Detection / 电池电压检测 =================*/

/**
 * @brief Initialize the voltage detection module / 初始化电压检测模块
 */
void VoltageDetection::init()
{
    pinMode(PIN_VOLTAGE, INPUT);
}

/**
 * @brief Get the current battery voltage / 获取当前电池电压
 * @return Battery voltage in volts / 电池电压（伏特）
 */
float VoltageDetection::getVoltage() const
{
    float voltage = analogRead(PIN_VOLTAGE) * 0.0375;
    voltage += voltage * 0.08; // Add 8% compensation / 增加 8% 补偿
    return voltage;
}

