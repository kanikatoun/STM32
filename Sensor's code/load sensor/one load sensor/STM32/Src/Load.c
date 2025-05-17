///*
// * Load_HX711.c
// *
// *  Created on: Feb 10, 2025
// *      Author: HOKKEY
// */
//#include "Load.h"
////#include "tim.h"  // Ensure this includes the timer used for microDelay
//
//// Private Variables
//static TIM_HandleTypeDef *htim_hx711;
//static HX711_Config sensorConfigs[NUM_LOAD_SENSORS];
//static uint32_t tare_values[NUM_LOAD_SENSORS] = {0};
//static float coefficients[NUM_LOAD_SENSORS] = {0.0f};
//
//// Private Function Prototypes
//static void microDelay(uint16_t delay);
//
//// Function Definitions
//
///**
//  * @brief Initializes the HX711 module for multiple sensors.
//  * @param htim: Timer handle used for microsecond delays.
//  * @param configs: Array of HX711 configurations for each sensor.
//  */
//void HX711_Init(TIM_HandleTypeDef *htim, HX711_Config *configs)
//{
//    htim_hx711 = htim;
//    HAL_TIM_Base_Start(htim_hx711);
//
//    // Copy configurations for each sensor
//    for (uint8_t i = 0; i < NUM_LOAD_SENSORS; i++)
//    {
//        sensorConfigs[i] = configs[i];
//
//        // Initialize SCK pin for each sensor
//        HAL_GPIO_WritePin(sensorConfigs[i].SCK_Port, sensorConfigs[i].SCK_Pin, GPIO_PIN_SET);
//        microDelay(1);
//        HAL_GPIO_WritePin(sensorConfigs[i].SCK_Port, sensorConfigs[i].SCK_Pin, GPIO_PIN_RESET);
//        microDelay(1);
//    }
//}
//
///**
//  * @brief Reads raw data from the HX711 for a specific sensor.
//  * @param sensorIndex: Index of the sensor (0 to NUM_LOAD_SENSORS - 1).
//  * @return Raw 24-bit data from the HX711.
//  */
//int32_t HX711_GetRawData(uint8_t sensorIndex)
//{
//    if (sensorIndex >= NUM_LOAD_SENSORS) return 0; // Invalid index
//
//    uint32_t data = 0;
//    uint32_t startTime = HAL_GetTick();
//
//    // Wait for DT pin to go low
//    while (HAL_GPIO_ReadPin(sensorConfigs[sensorIndex].DT_Port, sensorConfigs[sensorIndex].DT_Pin) == GPIO_PIN_SET)
//    {
//        if (HAL_GetTick() - startTime > 200)
//            return 0; // Timeout
//    }
//
//    // Read 24 bits of data
//    for (int8_t len = 0; len < 24; len++)
//    {
//        HAL_GPIO_WritePin(sensorConfigs[sensorIndex].SCK_Port, sensorConfigs[sensorIndex].SCK_Pin, GPIO_PIN_SET);
//        microDelay(1);
//        data = data << 1;
//        HAL_GPIO_WritePin(sensorConfigs[sensorIndex].SCK_Port, sensorConfigs[sensorIndex].SCK_Pin, GPIO_PIN_RESET);
//        microDelay(1);
//        if (HAL_GPIO_ReadPin(sensorConfigs[sensorIndex].DT_Port, sensorConfigs[sensorIndex].DT_Pin) == GPIO_PIN_SET)
//            data++;
//    }
//
//    // Toggle SCK to complete the read cycle
//    HAL_GPIO_WritePin(sensorConfigs[sensorIndex].SCK_Port, sensorConfigs[sensorIndex].SCK_Pin, GPIO_PIN_SET);
//    microDelay(1);
//    HAL_GPIO_WritePin(sensorConfigs[sensorIndex].SCK_Port, sensorConfigs[sensorIndex].SCK_Pin, GPIO_PIN_RESET);
//    microDelay(1);
//
//    // Convert to signed 24-bit value
//    data = data ^ 0x800000;
//    return (int32_t)data;
//}
//
///**
//  * @brief Calculates the weight based on raw HX711 data for a specific sensor.
//  * @param sensorIndex: Index of the sensor (0 to NUM_LOAD_SENSORS - 1).
//  * @return Weight in kilograms.
//  */
////float HX711_Weigh(uint8_t sensorIndex)
////{
////    if (sensorIndex >= NUM_LOAD_SENSORS) return 0.0f; // Invalid index
////
////    int32_t total = 0;
////    float kilograms;
////
////    // Collect HX711 data samples
////    for (uint16_t i = 0; i < HX711_SAMPLES; i++)
////    {
////        total += HX711_GetRawData(sensorIndex);
////    }
////    int32_t average = total / HX711_SAMPLES;
////
////    // Convert raw data to weight
////    float milligrams = (average - tare_values[sensorIndex]) * coefficients[sensorIndex];
////    kilograms = milligrams / 1000000.0f;  // Convert milligrams to kilograms
////
////    return kilograms;
////}
//
///**
//  * @brief Sets the tare value (zero offset) for a specific sensor.
//  * @param sensorIndex: Index of the sensor (0 to NUM_LOAD_SENSORS - 1).
//  */
//void HX711_Tare(uint8_t sensorIndex)
//{
//    if (sensorIndex >= NUM_LOAD_SENSORS) return; // Invalid index
//
//    int32_t total = 0;
//
//    for (uint16_t i = 0; i < HX711_SAMPLES; i++)
//    {
//        total += HX711_GetRawData(sensorIndex);
//    }
//    tare_values[sensorIndex] = total / HX711_SAMPLES;
//}
//
///**
//  * @brief Sets the calibration coefficient for a specific sensor.
//  * @param sensorIndex: Index of the sensor (0 to NUM_LOAD_SENSORS - 1).
//  * @param knownWeight: Known weight in milligrams.
//  * @param knownRawValue: Raw HX711 value corresponding to the known weight.
//  */
//void HX711_SetCoefficient(uint8_t sensorIndex, float knownWeight, float knownRawValue)
//{
//    if (sensorIndex >= NUM_LOAD_SENSORS) return; // Invalid index
//
//    coefficients[sensorIndex] = knownWeight / knownRawValue;
//}
//
///**
//  * @brief Microsecond delay using a timer.
//  * @param delay: Delay in microseconds.
//  */
//static void microDelay(uint16_t delay)
//{
//    __HAL_TIM_SET_COUNTER(htim_hx711, 0);
//    while (__HAL_TIM_GET_COUNTER(htim_hx711) < delay);
//}
//
