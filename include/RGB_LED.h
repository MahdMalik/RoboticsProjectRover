/**
 * @brief RGB LED Control Class / RGB LED 控制类
 */
class DeviceDriverSet_RGBLED {
public:
    /**
     * @brief Initialize the RGB LED module / 初始化 RGB LED 模块
     * @param brightness Initial brightness level (0-255) / 初始亮度级别 (0-255)
     */
    void init(uint8_t brightness);

    /**
     * @brief Set RGB LED color using FastLED CRGB object / 使用 FastLED CRGB 对象设置 RGB LED 颜色
     * @param Traversal_Number LED index to control / 要控制的 LED 索引
     * @param colour Color to set / 设置的颜色
     */
    void SetRBGLED(uint8_t Traversal_Number, CRGB colour);

    /**
     * @brief Set RGB LED color using individual RGB values / 使用单独的 RGB 值设置 RGB LED 颜色
     * @param ledIndex LED index to control / 要控制的 LED 索引
     * @param r Red component (0-255) / 红色分量 (0-255)
     * @param g Green component (0-255) / 绿色分量 (0-255)
     * @param b Blue component (0-255) / 蓝色分量 (0-255)
     */
    void setColor(uint8_t ledIndex, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Clear all LED colors (turn off) / 清除所有 LED 颜色（关闭）
     */
    void clear();

private:
    static constexpr uint8_t PIN_RGBLED = 4;  ///< RGB LED control pin / RGB LED 控制引脚
    static constexpr uint8_t NUM_LEDS = 1;    ///< Number of RGB LEDs / RGB LED 数量
    CRGB leds_[NUM_LEDS];                     ///< LED array / LED 数组
};

