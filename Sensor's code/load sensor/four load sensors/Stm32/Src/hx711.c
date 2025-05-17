#include "hx711.h"


void HX711_Init(HX711* hx) {
    HAL_GPIO_WritePin(hx->SCK_Port, hx->SCK_Pin, GPIO_PIN_RESET);
}

int32_t HX711_ReadRaw(HX711* hx) {
    int32_t count = 0;
    while (HAL_GPIO_ReadPin(hx->DT_Port, hx->DT_Pin));
    for (int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(hx->SCK_Port, hx->SCK_Pin, GPIO_PIN_SET);
        delay_us(1);
        count = count << 1;
        HAL_GPIO_WritePin(hx->SCK_Port, hx->SCK_Pin, GPIO_PIN_RESET);
        delay_us(1);
        if (HAL_GPIO_ReadPin(hx->DT_Port, hx->DT_Pin)) {
            count++;
        }
    }
    // 25th pulse for gain (128)
    HAL_GPIO_WritePin(hx->SCK_Port, hx->SCK_Pin, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(hx->SCK_Port, hx->SCK_Pin, GPIO_PIN_RESET);
    delay_us(1);

    // Convert 24-bit 2's complement to signed int
    if (count & 0x800000)
        count |= ~0xFFFFFF;

    return count;
}

float HX711_ReadWeight(HX711* hx) {
    int32_t raw = HX711_ReadRaw(hx);
    return (raw - hx->offset) / hx->scale;
}
