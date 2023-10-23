#include "common_inc.h"
#include "configurations.h"
#include "HelloWord/hw_keyboard.h"


/* Component Definitions -----------------------------------------------------*/
KeyboardConfig_t config;
HWKeyboard keyboard(&hspi1);


/* Main Entry ----------------------------------------------------------------*/

static bool sCapsFlag = false;
static bool sRandFlag = true;

void Main() {
    EEPROM eeprom;
    eeprom.Pull(0, config);
    if (config.configStatus != CONFIG_OK) {
        // Use default settings
        config = KeyboardConfig_t{
                .configStatus = CONFIG_OK,
                .serialNum=123,
                .KeyBoardUpFlag={},
                .DynamicUpFlag={},
                .keyMap={}
        };
        memset(config.keyMap, -1, 128);
        eeprom.Push(0, config);
    }

    // Keyboard Report Start
    HAL_TIM_Base_Start_IT(&htim4);


    while (true) {
        /*---- This is a demo RGB effect ----*/
        static uint8_t LedStatus = 0;
        switch (LedStatus) {
            case 0: {
                static float brightness = 0.1, LedCache = 0;
                static bool brightnessFlag = true;
                if (brightnessFlag) {
                    brightness = brightness + 0.007f;
                } else {
                    brightness = brightness - 0.007f;
                }
                if (brightness > 1) brightnessFlag = false;
                else if (brightness < -0.1) brightnessFlag = true;
                if (brightness < 0) {
                    LedCache = 0;
                } else {
                    LedCache = brightness;
                }
                for (uint8_t i = 0; i < HWKeyboard::LED_NUMBER; i++) {
                    keyboard.SetRgbBufferByID(i, HWKeyboard::Color_t{13, 235, 255}, LedCache);
                }
            }
                break;
            case 1: {

            }
                break;
            default:
                break;
        }
        if (sCapsFlag) {
            keyboard.SetRgbBufferByID(82, HWKeyboard::Color_t{(uint8_t) 237, (uint8_t) 28, 36}, 0.9);
            keyboard.SetRgbBufferByID(83, HWKeyboard::Color_t{(uint8_t) 237, (uint8_t) 28, 36}, 0.9);
            keyboard.SetRgbBufferByID(84, HWKeyboard::Color_t{(uint8_t) 237, (uint8_t) 28, 36}, 0.9);
        }
        /*-----------------------------------*/

        // Send RGB buffers to LEDs
        keyboard.SyncLights();
    }
}

/* Event Callbacks -----------------------------------------------------------*/
extern "C" void OnTimerCallback() // 1000Hz callback
{
    static uint32_t count = 0;
    count ++;
    if (count % 2000 == 0) {
        sRandFlag = true;
    }
    keyboard.ScanKeyStates();  // Around 40us use 4MHz SPI clk
    keyboard.ApplyDebounceFilter(100);
    keyboard.Remap(keyboard.FnPressed() ? 2 : 1);  // When Fn pressed use layer-2

    if (keyboard.KeyPressed(HWKeyboard::LEFT_CTRL) &&
        keyboard.KeyPressed(HWKeyboard::A)) {
        // do something...

        // or trigger some keys...
        keyboard.Press(HWKeyboard::DELETE);
    }

    // Report HID key states
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,
                               keyboard.GetHidReportBuffer(1),
                               HWKeyboard::KEY_REPORT_SIZE);
}


extern "C"
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    keyboard.isRgbTxBusy = false;
}

extern "C"
void HID_RxCpltCallback(const uint8_t *_data) {
    sCapsFlag = _data[1] & 0x02;
}