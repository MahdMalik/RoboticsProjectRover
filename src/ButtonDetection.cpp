
/*================ Key/Button Detection / 按键检测 =================*/
// Initialize static member / 初始化静态成员

/**
 * @brief Static variable to store the current key value / 存储当前按键值的静态变量
 */
uint8_t DeviceDriverSet_Key::keyValue_ = 0;

/**
 * @brief Interrupt handler for button press detection / 按键按下检测的中断处理程序
 * Implements debouncing and increments the key value when a valid press is detected / 实现去抖动并在检测到有效按下时递增按键值
 */
static void attachPinChangeInterrupt_GetKeyValue(void)
{
    static uint32_t lastDebounceTime = 0;
    const uint32_t debounceDelay = 500; // milliseconds / 毫秒

    // Read the state of the button / 读取按键状态
    int buttonState = digitalRead(DeviceDriverSet_Key::PIN_KEY);

    // Check if the button is pressed (taking into account the pull-up mode) / 检查按键是否被按下（考虑到上拉模式）
    if (buttonState == LOW) {
        uint32_t currentTime = millis();

        // Check if the current time is at least debounceDelay ms greater than the last debounce time / 检查当前时间是否比上次去抖动时间至少大 debounceDelay 毫秒
        if ((currentTime - lastDebounceTime) > debounceDelay) {
            // Update the last debounce time / 更新上次去抖动时间
            lastDebounceTime = currentTime;

            // Increment key value and wrap around if necessary / 递增按键值并在必要时回绕
            DeviceDriverSet_Key::keyValue_++;
            if (DeviceDriverSet_Key::keyValue_ > DeviceDriverSet_Key::KEY_VALUE_MAX) {
                DeviceDriverSet_Key::keyValue_ = 0;
            }
        }
    }
}

/**
 * @brief Initialize the key/button detection module / 初始化按键检测模块
 */
void DeviceDriverSet_Key::init()
{
    pinMode(PIN_KEY, INPUT_PULLUP);
    attachInterrupt(0, attachPinChangeInterrupt_GetKeyValue, FALLING); // Using interrupt 0 / 使用中断 0
}

/**
 * @brief Get the current key/button value / 获取当前按键值
 * @param get_keyValue Pointer to store the current key value / 用于存储当前按键值的指针
 */
void DeviceDriverSet_Key::getKeyValue(uint8_t *get_keyValue)
{
  *get_keyValue = keyValue_;
}