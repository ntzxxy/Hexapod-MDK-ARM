#include "key.h"

void Key::Init() {
    
}

bool Key::ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    
    return (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET);
}

uint8_t Key::Scan() {
    if (ReadPin(KEY1_GPIO_Port, KEY1_Pin)) 
			return 1;
    if (ReadPin(KEY2_GPIO_Port, KEY2_Pin)) 
			return 2;
    if (ReadPin(KEY3_GPIO_Port, KEY3_Pin)) 
			return 3;
    if (ReadPin(KEY4_GPIO_Port, KEY4_Pin)) 
			return 4;
    return 0;
}