#ifndef KEY_H
#define KEY_H

#include "main.h"


class Key {
public:
    // 初始化（如果使用了CubeMX生成的GPIO初始化，这里可以留空）
    void Init();
    
    // 扫描所有按键，返回键值 (0:无, 1:Key1, 2:Key2, ...)
    uint8_t Scan();
    
private:
    // 读取单个引脚状态
    bool ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
};

#endif