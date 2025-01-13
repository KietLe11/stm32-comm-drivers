/*
 * GPIO Driver API Source File for stm32f302r8
 */

#include "stm32f302r8_gpio_driver.h"

/*
 * GPIO_PeriClockControl
 *
 * Description              : Enables or disables peripheral clock for given GPIO Port
 *
 * @param pGPIOxBaseAddr 	: Base address of the gpio peripheral
 * @param ENorDI 			: Enable or disable macros
 *
 * @return 					: None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOxBaseAddr, uint8_t ENorDI) {
	if (ENorDI == ENABLE) {
         if (pGPIOxBaseAddr == GPIOA) {
             GPIOA_PCLK_EN();
         } else if (pGPIOxBaseAddr == GPIOB) {
             GPIOB_PCLK_EN();
         } else if (pGPIOxBaseAddr == GPIOC) {
             GPIOC_PCLK_EN();
         } else if (pGPIOxBaseAddr == GPIOD) {
             GPIOD_PCLK_EN();
         } else if (pGPIOxBaseAddr == GPIOF) {
             GPIOF_PCLK_EN();
         }
    } else {
         if (pGPIOxBaseAddr == GPIOA) {
             GPIOA_PCLK_DI();
         } else if (pGPIOxBaseAddr == GPIOB) {
             GPIOB_PCLK_DI();
         } else if (pGPIOxBaseAddr == GPIOC) {
             GPIOC_PCLK_DI();
         } else if (pGPIOxBaseAddr == GPIOD) {
             GPIOD_PCLK_DI();
         } else if (pGPIOxBaseAddr == GPIOF) {
             GPIOF_PCLK_DI();
         }
    }
}

/*
 * GPIO_Init
 *
 * Description              : Initialize GPIO Port
 *
 * @param pGPIOHandle       : Pointer to GPIOHandle struct
 *
 * @return                  : None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

    uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber;
    uint8_t pPinMode = pGPIOHandle->GPIO_PinConfig->GPIO_PinMode;
    volatile uint32_t* gpioModeRegister = &(pGPIOHandle->pGPIOxBaseAddr->MODER);

    // Configure Mode of GPIO Pin
    if (pPinMode <= GPIO_MODE_ANALOG) { // Non-interrupt Mode
        *gpioModeRegister &= ~(3 << (2 * pinNumber)); // Clear
        *gpioModeRegister |= pPinMode << (2 * pinNumber); // Set
    } else { // Interrupt Mode
        // TODO: Interrupt Mode
    }

    // Configure Speed
    uint8_t pinSpeed = pGPIOHandle->GPIO_PinConfig->GPIO_PinSpeed;
    volatile uint32_t* pinSpeedRegister = &(pGPIOHandle->pGPIOxBaseAddr->OSPEEDR);
    *pinSpeedRegister &= ~(3 << (2 * pinNumber)); // Clear
    *pinSpeedRegister |= pinSpeed << (2 * pinNumber); // Set

    //Configure PullUp/Pulldown PUPD Configuration
    uint8_t pinPUPD = pGPIOHandle->GPIO_PinConfig->GPIO_PinPuPdControl;
    volatile uint32_t* pupdRegister = &(pGPIOHandle->pGPIOxBaseAddr->PUPDR);
    *pupdRegister &= ~(3 << (2 * pinNumber)); // Clear
    *pupdRegister |= pinPUPD << (2 * pinNumber); // Set

    // Configure Output type OPType
    uint8_t pinOPType = pGPIOHandle->GPIO_PinConfig->GPIO_PinOPType;
    volatile uint32_t* outTypeRegister = &(pGPIOHandle->pGPIOxBaseAddr->OTYPER);
    *outTypeRegister &= 1 << pinNumber; // Clear
    *outTypeRegister |= pinOPType << pinNumber; // Set

    // Configure Alternate Functionality
    if (pPinMode == GPIO_MODE_ALTFN) {
        // Configure Alternative Function Registers
        uint8_t pinAltFunMode = pGPIOHandle->GPIO_PinConfig->GPIO_PinAltFunMode;
        uint8_t AFR_select = pinNumber/8;
        uint8_t AFR_shiftAmount = pinNumber % 8;

        pGPIOHandle->pGPIOxBaseAddr->AFR[AFR_select] &= 0xF << (4 * AFR_shiftAmount); // Clear
        pGPIOHandle->pGPIOxBaseAddr->AFR[AFR_select] |= pinAltFunMode << (4 * AFR_shiftAmount); // Set
    }
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOxBaseAddr) {
    if (pGPIOxBaseAddr == GPIOA) {
        GPIOA_REG_RESET();
    } else if (pGPIOxBaseAddr == GPIOB) {
        GPIOB_REG_RESET();
    } else if (pGPIOxBaseAddr == GPIOC) {
        GPIOC_REG_RESET();
    } else if (pGPIOxBaseAddr == GPIOD) {
        GPIOD_REG_RESET();
    } else if (pGPIOxBaseAddr == GPIOF) {
        GPIOF_REG_RESET();
    }
}

// Data Read/Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOxBaseAddr, uint8_t PinNumber) {
    uint8_t value;
    value = (uint8_t)((pGPIOxBaseAddr->IDR >> PinNumber) & 0x00000001);

    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOxBaseAddr) {
    uint16_t value;
    value = (uint16_t)(pGPIOxBaseAddr->IDR);

    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOxBaseAddr,  uint8_t PinNumber, uint8_t outputData);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOxBaseAddr, uint16_t outputData);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOxBaseAddr,  uint8_t PinNumber);

// IRQ Config and ISR Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);
