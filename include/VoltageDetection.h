/**
 * @brief Voltage Detection Class / 电压检测类
 */
class DeviceDriverSet_Voltage {
public:
    /**
     * @brief Initialize the voltage detection module / 初始化电压检测模块
     */
    void init();

    /**
     * @brief Get current battery voltage / 获取当前电池电压
     * @return Voltage in volts / 电压（伏特）
     */
    float getVoltage() const;

private:
    static constexpr uint8_t PIN_VOLTAGE = A3;  ///< Voltage detection pin / 电压检测引脚
};
