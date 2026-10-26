/*================ RGB LED Driver / RGB LED 驱动 =================*/

/**
 * @brief Initialize the RGB LED module / 初始化 RGB LED 模块
 * @param brightness Initial brightness level (0-255) / 初始亮度级别 (0-255)
 */
void DeviceDriverSet_RGBLED::init(uint8_t brightness)
{
    FastLED.addLeds<NEOPIXEL, PIN_RGBLED>(leds_, NUM_LEDS);
    FastLED.setBrightness(brightness);
}

/**
 * @brief Set multiple RGB LEDs to the same color / 将多个 RGB LED 设置为相同颜色
 * @param Traversal_Number Number of LEDs to set (from index 0) / 要设置的 LED 数量（从索引 0 开始）
 * @param colour The color to set for all specified LEDs / 为所有指定 LED 设置的颜色
 */
void DeviceDriverSet_RGBLED::SetRBGLED(uint8_t Traversal_Number, CRGB colour)
{
  if (NUM_LEDS < Traversal_Number)
  {
    Traversal_Number = NUM_LEDS;
  }
  for (int Number = 0; Number < Traversal_Number; Number++)
  {
    leds_[Number] = colour;
    FastLED.show();
  }
}

/**
 * @brief Set a specific LED to a custom RGB color / 将特定 LED 设置为自定义 RGB 颜色
 * @param ledIndex Index of the LED to set (if >= NUM_LEDS, sets all LEDs) / 要设置的 LED 索引（如果 >= NUM_LEDS，则设置所有 LED）
 * @param r Red component (0-255) / 红色分量 (0-255)
 * @param g Green component (0-255) / 绿色分量 (0-255)
 * @param b Blue component (0-255) / 蓝色分量 (0-255)
 */
void DeviceDriverSet_RGBLED::setColor(uint8_t ledIndex, uint8_t r, uint8_t g, uint8_t b)
{
    if (ledIndex >= NUM_LEDS)
    {
        FastLED.showColor(CRGB(r, g, b)); // Set all LEDs to the same color / 将所有 LED 设置为相同颜色
    }
    else
    {
        leds_[ledIndex] = CRGB(r, g, b); // Set only the specified LED / 仅设置指定的 LED
    }
    FastLED.show();
}

/**
 * @brief Clear all RGB LEDs (turn them off) / 清除所有 RGB LED（关闭它们）
 */
void DeviceDriverSet_RGBLED::clear()
{
    FastLED.clear(true);
}
