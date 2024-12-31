
#ifndef INC_STM32F302R8_H_
#define INC_STM32F302R8_H_

/*
 *  Base addresses of Flash and SRAM Memories
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

#endif /* INC_STM32F302R8_H_ */
