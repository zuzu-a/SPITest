#include "stm32f7xx_hal.h"
#include "spitest.h"
#include "main.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
    }
}
