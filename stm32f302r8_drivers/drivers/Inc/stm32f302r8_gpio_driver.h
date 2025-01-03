/*
 * GPIO Driver API Header File for stm32f302r8
 */

#ifndef INC_STM32F302R8_GPIO_DRIVER_H_
#define INC_STM32F302R8_GPIO_DRIVER_H_

#include "stm32f302r8.h"

// Configuration struct for GPIO Pin
typedef struct {
    uint8_t GPIO_PinNumber;         // Find Pin Numbers @GPIO_PIN_NUMBERS
    uint8_t GPIO_PinMode;           // Find modes @GPIO_PIN_MODES
    uint8_t GPIO_PinSpeed;          // Find pin speeds @GPIO_PIN_SPEEDS
    uint8_t GPIO_PinPuPdControl;    // Find PUPD Control @GPIO_PUPD_CONTROL
    uint8_t GPIO_PinOPType;         // Find Output Types @GPIO_PIN_OUTPUT_TYPES
    uint8_t GPIO_PinAltFunMode;     // Find Pin Alternative Function Modes @GPIO_ALT_FUNC_MODES
} GPIO_PinConfig_t;

// Handle Structure for a GPIO Pin
typedef struct {
    // Pointer to hold base address of GPIO
    GPIO_RegDef_t *pGPIOxBaseAddr; // Holds the base address of the GPIO port to which pin belongs
    GPIO_PinConfig_t *GPIO_PinConfig; // Holds GPIO pin configuration settings
} GPIO_Handle_t;

// @GPIO_PIN_NUMBERS : Pin Numbers
#define GPIO_PIN_NUM_0                  0
#define GPIO_PIN_NUM_1                  1
#define GPIO_PIN_NUM_2                  2
#define GPIO_PIN_NUM_3                  3
#define GPIO_PIN_NUM_4                  4
#define GPIO_PIN_NUM_5                  5
#define GPIO_PIN_NUM_6                  6
#define GPIO_PIN_NUM_7                  7
#define GPIO_PIN_NUM_8                  8
#define GPIO_PIN_NUM_9                  9
#define GPIO_PIN_NUM_10                 10
#define GPIO_PIN_NUM_11                 11
#define GPIO_PIN_NUM_12                 12
#define GPIO_PIN_NUM_13                 13
#define GPIO_PIN_NUM_14                 14
#define GPIO_PIN_NUM_15                 15

// @GPIO_PIN_MODES : GPIO Pin Modes for GPIOx_MODER
#define GPIO_MODE_IN                    0
#define GPIO_MODE_OUT                   1
#define GPIO_MODE_ALTFN                 2
#define GPIO_MODE_ANALOG                3
#define GPIO_MODE_IT_FT                 4 // Falling Edge Trigger
#define GPIO_MODE_IT_RT                 5 // Rising Edge Trigger
#define GPIO_MODE_IT_RFT                6 // Rising/Falling Edge Trigger

// @GPIO_PIN_OUTPUT_TYPES : GPIO Pin Output Types (Push Or Pull) for GPIOx_OTYPER
#define GPIO_OUT_TYPE_PUSHPULL          0
#define GPIO_OUT_TYPE_OPENDRAIN         1

// @GPIO_PIN_SPEEDS : GPIO Output Speed for GPIOx_OSPEEDR
#define GPIO_OUT_SPEED_LOW              0
#define GPIO_OUT_SPEED_MEDIUM           1
#define GPIO_OUT_SPEED_HIGH             3

// @GPIO_PUPD_CONTROL : GPIO Pullup/PullDOwn Configuration
#define GPIO_PUPD_NONE                  0
#define GPIO_PIN_PULLUP                 1
#define GPIO_PIN_PULLDOWN               2

/*********** APIs Supported by this driver ***********/

// Init and Deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOxBaseAddr);

// Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOxBaseAddr, uint8_t ENorDI);

// Data Read/Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOxBaseAddr, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOxBaseAddr);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOxBaseAddr,  uint8_t PinNumber, uint8_t outputData);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOxBaseAddr, uint16_t outputData);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOxBaseAddr,  uint8_t PinNumber);

// IRQ Config and ISR Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F302R8_GPIO_DRIVER_H_ */
