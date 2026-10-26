/*================ ITR20001 Line Tracking Sensor / ITR20001 循迹传感器 =================*/

/**
 * @brief Initialize the ITR20001 line tracking sensors / 初始化 ITR20001 循迹传感器
 */
void DeviceDriverSet_ITR20001::init()
{
    pinMode(PIN_ITR_L, INPUT);
    pinMode(PIN_ITR_M, INPUT);
    pinMode(PIN_ITR_R, INPUT);
}

/**
 * @brief Get the analog value from the left line tracking sensor / 获取左侧循迹传感器的模拟值
 * @return Analog value (0-1023) where higher values indicate lighter surface / 模拟值 (0-1023)，值越高表示表面越亮
 */
int DeviceDriverSet_ITR20001::getAnalogueL() const
{
    return analogRead(PIN_ITR_L);
}

/**
 * @brief Get the analog value from the middle line tracking sensor / 获取中间循迹传感器的模拟值
 * @return Analog value (0-1023) where higher values indicate lighter surface / 模拟值 (0-1023)，值越高表示表面越亮
 */
int DeviceDriverSet_ITR20001::getAnalogueM() const
{
    return analogRead(PIN_ITR_M);
}

/**
 * @brief Get the analog value from the right line tracking sensor / 获取右侧循迹传感器的模拟值
 * @return Analog value (0-1023) where higher values indicate lighter surface / 模拟值 (0-1023)，值越高表示表面越亮
 */
int DeviceDriverSet_ITR20001::getAnalogueR() const
{
    return analogRead(PIN_ITR_R);
}
