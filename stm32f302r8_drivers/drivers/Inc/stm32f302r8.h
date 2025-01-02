
#ifndef INC_STM32F302R8_H_
#define INC_STM32F302R8_H_

#include<stdint.h>

#define __vo volatile

/*
 * Base addresses of Flash and SRAM Memories
 */

#define FLASH_BASEADDR					0x08000000U // Base address of flash memory found in datasheet/RM
#define SRAM_BASEADDR					0x20000000U // Base address of SRAM
#define ROM								0x1FFFF800U // Base address of System Memory (ROM)
#define SRAM 							SRAM1_BASEADDR // Our main ram will be SRAM

/*
 * AHBx and APBx Bus Peripheral Base Addresses
 * Can found in "Section 5 Memory mapping" of the data sheet stm32f302r8.pdf
 */
#define PERIPH_BASE                     0x40000000U
#define APB1PERIPH_BASE                 PERIPH_BASE
#define APB2PERIPH_BASE                 0x40010000U
#define AHB1PERIPH_BASE                 0x40020000U
#define AHB2PERIPH_BASE                 0x48000000U
#define AHB3PERIPH_BASE                 0x50000000U

/*
 * Base Addresses of Peripherals on AHB1 Bus
 */
#define DMA1_BASEADDR                   (AHB1PERIPH_BASE + 0x0000)
#define RCC_BASEADDR                    (AHB1PERIPH_BASE + 0x1000)
#define FLASH_IF_BASEADDR               (AHB1PERIPH_BASE + 0x2000)
#define CRC_BASEADDR                    (AHB1PERIPH_BASE + 0x3000)
#define TSC_BASEADDR                    (AHB1PERIPH_BASE + 0x4000)

/*
 * Base Addresses of Peripherals on AHB2 Bus
 */
#define GPIOA_BASEADDR                  (AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR                  (AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR                  (AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR                  (AHB2PERIPH_BASE + 0x0C00)
#define GPIOF_BASEADDR                  (AHB2PERIPH_BASE + 0x1400)

/*
 * Base Addresses of Peripherals on AHB3 Bus
 */
#define ADC1_BASEADDR                   AHB3PERIPH_BASE

/*
 * Base addresses of Peripherals on APB1 Bus
 */
#define I2C1_BASEADDR                   (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR                   (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR                   (APB1PERIPH_BASE + 0x7800)

#define SPI2_BASEADDR                   (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR                   (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR                 (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR                 (APB1PERIPH_BASE + 0x4800)

/*
 * Base Addresses of Peripherals on APB2 Bus
 */
#define EXTI_BASEADDR                   (APB2PERIPH_BASE + 0x0400)
#define USART1_BASEADDR                 (APB2PERIPH_BASE + 0x3800)
#define SYSCFG_BASEADDR                 APB2PERIPH_BASE

/**********************Peripheral Register Definition Structures***********************/

typedef struct {
    __vo uint32_t MODER;     // GPIO port mode register,                 Offset=0x00
    __vo uint32_t OTYPER;    // GPIO port output type register,          Offset=0x04
    __vo uint32_t OSPEEDR;   // GPIO port output speed register,         Offset=0x08
    __vo uint32_t PUPDR;     // GPIO port pull-up/pull-down register,    Offset=0x0C
    __vo uint32_t IDR;       // GPIO port input data register,           Offset=0x10
    __vo uint32_t ODR;       // GPIO port output data register,          Offset=0x14
    __vo uint32_t BSRR;      // GPIO port bit set/reset register,        Offset=0x18
    __vo uint32_t LCKR;      // GPIO port configuration lock register,   Offset=0x1C

    /*
     * GPIO alternate function low/high register
     * AFR[0] : low,    Offset=0x20
     * AFR[1] : high,   Offset=0x24
     */
    __vo uint32_t AFR[2];
} GPIO_RegDef_t;

typedef struct {
    __vo uint32_t CR;           // Clock control register,                  Offset=0x00
    __vo uint32_t CFGR;         // Clock configuration register,            Offset=0x04
    __vo uint32_t CIR;          // Clock interrupt register,                Offset=0x08
    __vo uint32_t APB2RSTR;     // APB2 peripheral reset register,          Offset=0x0C
    __vo uint32_t APB1RSTR;     // APB1 peripheral reset register,          Offset=0x10
    __vo uint32_t AHBENR;       // AHB peripheral clock enable register,    Offset=0x14
    __vo uint32_t APB2ENR;      // APB2 peripheral clock enable register,   Offset=0x18
    __vo uint32_t APB1ENR;      // APB1 peripheral clock enable register,   Offset=0x1C
    __vo uint32_t BDCR;         // RTC domain control register,             Offset=0x20
    __vo uint32_t CSR;          // Control/status register,                 Offset=0x24
    __vo uint32_t AHBRSTR;      // AHB peripheral reset register,           Offset=0x28
    __vo uint32_t CFGR2;        // Clock configuration register 2,          Offset=0x2C
    __vo uint32_t CFGR3;        // Clock configuration register 3,          Offset=0x30
} RCC_RegDef_t;


// GPIO Peripheral Definitions with GPIO_RegDef_t type
#define GPIOA                           ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                           ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                           ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                           ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOF                           ((GPIO_RegDef_t*) GPIOF_BASEADDR)

// RCC Peripheral Definition with RCC_RegDef_t type
#define RCC                             ((RCC_RegDef_t*) RCC_BASEADDR)

// Clock Enable Macros for GPIO Peripherals
#define GPIOA_PCLK_EN()                 (RCC->AHBENR |= (1<<17))
#define GPIOB_PCLK_EN()                 (RCC->AHBENR |= (1<<18))
#define GPIOC_PCLK_EN()                 (RCC->AHBENR |= (1<<19))
#define GPIOD_PCLK_EN()                 (RCC->AHBENR |= (1<<20))
#define GPIOF_PCLK_EN()                 (RCC->AHBENR |= (1<<22))

// Clock Enable Macros for I2C Peripherals
#define I2C1_PCLK_EN()                  (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()                  (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()                  (RCC->APB1ENR |= (1<<30))

// Clock Enable Macros for SPI Peripherals
#define SPI2_PCLK_EN()                  (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()                  (RCC->APB1ENR |= (1<<15))

// Clock Enable Macros for USART Peripherals
#define USART1_PCLK_EN()                (RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN()                (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()                (RCC->APB1ENR |= (1<<18))

// Clock Enable Macros for SYSCFG Peripheral
#define SYSCFG_PCLK_EN()                (RCC->APB2ENR |= (1<<0))

// Clock Disable Macros for GPIO Peripherals
#define GPIOA_PCLK_DI()                 (RCC->AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI()                 (RCC->AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI()                 (RCC->AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI()                 (RCC->AHBENR &= ~(1<<20))
#define GPIOF_PCLK_DI()                 (RCC->AHBENR &= ~(1<<22))

// Clock Disable Macros for I2C Peripherals
#define I2C1_PCLK_DI()                  (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()                  (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()                  (RCC->APB1ENR &= ~(1<<30))

// Clock Disable Macros for SPI Peripherals
#define SPI2_PCLK_DI()                  (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()                  (RCC->APB1ENR &= ~(1<<15))

// Clock Disable Macros for USART Peripherals
#define USART1_PCLK_DI()                (RCC->APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI()                (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()                (RCC->APB1ENR &= ~(1<<18))

// Clock Enable Macros for SYSCFG Peripheral
#define SYSCFG_PCLK_DI()                (RCC->APB2ENR &= ~(1<<0))

#endif /* INC_STM32F302R8_H_ */
