/*
 * stm32f407xx.h
 *
 *  Created on: Apr 10, 2024
 *      Author: harsh
 */

#include <stdint.h>

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


//*************************** MCU specific register ARM
//interrupt set enable register (ARM)
#define NVIC_ISERx                                          (uint32_t *)(0xE000E100U)

//interrupt clear enable register   (ARM)
#define NVIC_ICERx                                          (uint32_t *)(0xE000E180U)

//interrupt priority register (ARM)
#define NVIC_IPRx                                           (uint32_t *)(0xE000E400U)

#define NoChangeInPriority                                  -5


//MEMORY BASE ADDRESSES
#define FLASH_BASEADDRESS												(0x08000000U)	 /*flash memory starting address*/
#define SRAM1_BASAEADDRESS												(0x20000000U)  /*sram starting address*/
#define SRAM2_BASEADDRESS												(0x2001C000U)  /*sram2 starting address*/
#define ROM_BASEADDRESS													(0x1FFF0000U)	/*system memory is called rom*/
#define SRAM 															(SRAM1_BASAEADDRESS)

//PRIPHERAL BUS ADDRESSES
#define	APB1PERIPH_BASE												(0x40000000U)
#define APB2PERIPH_BASE          									(0x40010000U)
#define AHB1PERIPH_BASE   												(0x40020000U)
#define AHB2PERIPH_BASE         										(0x50000000U)

// RCC addresses

#define RCC_BASEADDR                                        (0x40023800U)
                                    

//BASE ADDRESSES OF PERIPHERALS HANGING ON AHB1 BUS
#define GPIOA_BASEADDR													(0x40020000U)
#define GPIOB_BASEADDR  												(0x40020400U)
#define GPIOC_BASEADDR  												(0x40020800U)
#define GPIOD_BASEADDR  												(0x40020C00U)
#define GPIOE_BASEADDR  												(0x40021000U)
#define GPIOF_BASEADDR  												(0x40021400U)
#define GPIOG_BASEADDR  												(0x40021800U)
#define GPIOH_BASEADDR  												(0x40021C00U)
#define GPIOI_BASEADDR  												(0x40022000U)


//BASE ADDRESSES OF PERIPHERALS HANGING ON APB1 BUS
#define I2C1_BASEADDR 												(0x40005400U)
#define I2C2_BASEADDR 												(0x40005800U)
#define I2C3_BASEADDR 												(0x40005C00U)
#define SPI2_BASEADDR 												(0x40003800U)
#define SPI3_BASEADDR 												(0x40003C00U)
#define USART2_BASEADDR 											(0x40004400U)
#define USART3_BASEADDR 											(0x40004800U)
#define UART4_BASEADDR 												(0x40004C00U)
#define UART5_BASEADDR 												(0x40005000U)

//BASE ADDRESSES OF PERIPHERALS HANGING ON APB2 BUS
#define SPI1_BASEADDR     											(0x40013000U)
#define USART1_BASEADDR   											(0x40011000U)
#define USART6_BASEADDR   											(0x40011400U)
#define EXTI_BASEADDR     											(0x40013C00U)
#define SYSCFG_BASEADDR   											(0x40013800U)


// peripheral register definition

//gpio clock control register structure.
typedef struct
{
   volatile uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
   volatile uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
   volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
   volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
   volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
   volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
   volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
   volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
   volatile uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_RegDef_t;

#define GPIO_MODER_PIN0             0
#define GPIO_MODER_PIN1             2
#define GPIO_MODER_PIN2             4
#define GPIO_MODER_PIN3             6
#define GPIO_MODER_PIN4             8
#define GPIO_MODER_PIN5             10
#define GPIO_MODER_PIN6             12
#define GPIO_MODER_PIN7             14
#define GPIO_MODER_PIN8             16
#define GPIO_MODER_PIN9             18
#define GPIO_MODER_PIN10            20
#define GPIO_MODER_PIN11            22
#define GPIO_MODER_PIN12            24
#define GPIO_MODER_PIN13            26
#define GPIO_MODER_PIN14            28
#define GPIO_MODER_PIN15            30

#define GPIO_AF_PIN_0                0
#define GPIO_AF_PIN_1                4
#define GPIO_AF_PIN_2                8
#define GPIO_AF_PIN_3                12
#define GPIO_AF_PIN_4                16
#define GPIO_AF_PIN_5                20
#define GPIO_AF_PIN_6                24
#define GPIO_AF_PIN_7                28
#define GPIO_AF_PIN_8                0
#define GPIO_AF_PIN_9                4
#define GPIO_AF_PIN_10               8
#define GPIO_AF_PIN_11               12
#define GPIO_AF_PIN_12               16
#define GPIO_AF_PIN_13               20
#define GPIO_AF_PIN_14               24
#define GPIO_AF_PIN_15               28

//rcc clock control register structure.

typedef struct
{
   volatile uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
   volatile uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
   volatile uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
   volatile uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
   volatile uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
   volatile uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
   volatile uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
   volatile    uint32_t RESERVED0;     /*!< Reserved, 0x1C                                                                    */
   volatile uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
   volatile uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
   volatile    uint32_t RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
   volatile uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
   volatile uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
   volatile uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
   volatile    uint32_t RESERVED2;     /*!< Reserved, 0x3C                                                                    */
   volatile uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
   volatile uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
   volatile    uint32_t RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
   volatile uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
   volatile uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
   volatile uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
   volatile    uint32_t RESERVED4;     /*!< Reserved, 0x5C                                                                    */
   volatile uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
   volatile uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
   volatile    uint32_t RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
   volatile uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
   volatile uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
   volatile    uint32_t RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
   volatile uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
   volatile uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_RegDef_t;

//bitpositions for AHB1ENR 
#define AHB1ENR_GPIOAEN                0
#define AHB1ENR_GPIOBEN                1
#define AHB1ENR_GPIOCEN                2
#define AHB1ENR_GPIODEN                3
#define AHB1ENR_GPIOEEN                4
#define AHB1ENR_GPIOFEN                5
#define AHB1ENR_GPIOGEN                6
#define AHB1ENR_GPIOHEN                7
#define AHB1ENR_GPIOIEN                8
#define AHB1ENR_CRCEN                  12
#define AHB1ENR_BKPSRAMEN              18
#define AHB1ENR_CCMDAT                 20
#define AHB1ENR_DMA1EN                 21
#define AHB1ENR_DMA2EN                 22
#define AHB1ENR_ETHMACEN               25
#define AHB1ENR_ETHMACTXEN             26
#define AHB1ENR_ETHMACRXEN             27
#define AHB1ENR_ETHMACPTPEN            28
#define AHB1ENR_OTGHSEN                29
#define AHB1ENR_OTGHSULPIEN            30
// EXTI Register structure 
typedef struct {
   volatile uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
   volatile uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
   volatile uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
   volatile uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
   volatile uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
   volatile uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_RegDef_t;

//SYSCFG register structure

typedef struct {
   volatile uint32_t MEMRMP;       /*!< SYSCFG memory re map register,                      Address offset: 0x00      */
   volatile uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
   volatile uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
   volatile    uint32_t RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
   volatile uint32_t CMPCR; 
} SYSCFG_RegDef_t;


//spi reg definition structure
typedef struct
{
  volatile uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  volatile uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  volatile uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  volatile uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  volatile uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  volatile uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  volatile uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  volatile uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  volatile uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_RegDef_t;

// GPIO pointers

#define GPIOA                                                           ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB                                                           ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC                                                           ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD                                                           ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE                                                           ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF                                                           ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG                                                           ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH                                                           ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI                                                           ((GPIO_RegDef_t *)GPIOI_BASEADDR)

// RCC pointer

#define RCC                                                             ((RCC_RegDef_t *)RCC_BASEADDR)

// EXTI pointer 
#define EXTI                                                            ((EXTI_RegDef_t *)EXTI_BASEADDR)


//SYSCFG pointer 
#define SYSCFG                                                          ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

// SPI pointer

#define SPI1															((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2															((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3															((SPI_RegDef_t *)SPI3_BASEADDR)



// I2C POINTERS
#define I2C1															((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2															((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3															((I2C_RegDef_t *)I2C3_BASEADDR)


// THIS MACROS REPRESENT ON WHICH LINE OF NVIC , IRQ WILL BE DELIVERD
#define IRQ_NO_EXTI_0               6
#define IRQ_NO_EXTI_1               7
#define IRQ_NO_EXTI_2               8
#define IRQ_NO_EXTI_3               9
#define IRQ_NO_EXTI_4               10
#define IRQ_NO_EXTI_5_TO_9          23
#define IRQ_NO_EXTI_10_TO_15        40

// SPI INTERRUPT MACTOS
#define IRQ_NO_SPI1_GLOBAL       35
#define IRQ_NO_SPI2_GLOBAL       36
#define IRQ_NO_SPI3_GLOBAL       51




#define GPIOA_PCLK_EN() 	(RCC->AHB1ENR |= 1<<0)
#define GPIOB_PCLK_EN() 	(RCC->AHB1ENR |= 1<<1)
#define GPIOC_PCLK_EN() 	(RCC->AHB1ENR |= 1<<2)
#define GPIOD_PCLK_EN() 	(RCC->AHB1ENR |= 1<<3)
#define GPIOE_PCLK_EN() 	(RCC->AHB1ENR |= 1<<4)
#define GPIOF_PCLK_EN() 	(RCC->AHB1ENR |= 1<<5)
#define GPIOG_PCLK_EN() 	(RCC->AHB1ENR |= 1<<6)
#define GPIOH_PCLK_EN() 	(RCC->AHB1ENR |= 1<<7)
#define GPIOI_PCLK_EN() 	(RCC->AHB1ENR |= 1<<8)


#define GPIOA_REG_RST()     do{ (RCC->AHB1RSTR |= 1 << 0 ); ( RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RST()     do{ (RCC->AHB1RSTR |= 1 << 1 ); ( RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RST()     do{ (RCC->AHB1RSTR |= 1 << 2 ); ( RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RST()     do{ (RCC->AHB1RSTR |= 1 << 3 ); ( RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RST()     do{ (RCC->AHB1RSTR |= 1 << 4 ); ( RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RST()     do{ (RCC->AHB1RSTR |= 1 << 5 ); ( RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RST()     do{ (RCC->AHB1RSTR |= 1 << 6 ); ( RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RST()     do{ (RCC->AHB1RSTR |= 1 << 7 ); ( RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RST()     do{ (RCC->AHB1RSTR |= 1 << 8 ); ( RCC->AHB1RSTR &= ~(1 << 8)); } while(0)

// CLOCK ENABLE MACROS FOR I2Cx PERIPHERALS
#define I2C1_PCLK_EN()     (RCC->APB1ENR |= 1 << 21)
#define I2C2_PCLK_EN()     (RCC->APB1ENR |= 1 << 22)
#define I2C3_PCLK_EN()     (RCC->APB1ENR |= 1 << 23)


// CLOCK ENABLE MACROS FOR SPIx PERIPHERALS
#define SPI1_PCLK_EN()     (RCC->APB2ENR |= 1 << 12)
#define SPI2_PCLK_EN()     (RCC->APB1ENR |= 1 << 14)
#define SPI3_PCLK_EN()     (RCC->APB1ENR |= 1 << 15)


// SPI RESET MACROS FOR SPIx PERIPHERALS
#define SPI1_REG_RST()     RCC->APB2RSTR |= 1 << 12; RCC->APB2RSTR &= ~(1 << 12);
#define SPI2_REG_RST()     RCC->APB1RSTR |= 1 << 14; RCC->APB1RSTR &= ~(1 << 14);
#define SPI3_REG_RST()     RCC->APB1RSTR |= 1 << 15; RCC->APB1RSTR &= ~(1 << 15);



//CLOCK ENABLE MACROS FOR USARTx PERIPHERALS
#define USART1_PCLK_EN()   (RCC->APB2ENR |= 1 << 4)
#define USART2_PCLK_EN()   (RCC->APB1ENR |= 1 << 17)
#define USART3_PCLK_EN()   (RCC->APB1ENR |= 1 << 18)
#define USART6_PCLK_EN()   (RCC->APB2ENR |= 1 << 5)


//CLOCK ENABLE MACROS FOR UARTx PERIPHERALS
#define UART4_PCLK_EN()   (RCC->APB1ENR |= 1 << 19)
#define UART5_PCLK_EN()   (RCC->APB1ENR |= 1 << 20)

//CLOCK ENABLE MACROS FOR SYSCFG PERIPHERALS
#define SYSCFG_PCLK_EN()   (RCC->APB2ENR |= 1 << 14)


//clock DISABLE macros for GPIOx peripherals
#define GPIOA_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<8))


// CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS
#define I2C1_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 23))


// CLOCK DISABLE MACROS FOR SPIx PERIPHERALS
#define SPI1_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 15))


//CLOCK DISABLE MACROS FOR USARTx PERIPHERALS
#define USART1_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 5))


//CLOCK DISABLE MACROS FOR UARTx PERIPHERALS
#define UART4_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 20))

//CLOCK DISABLE MACROS FOR SYSCFG PERIPHERALS
#define SYSCFG_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 14))


// some generic macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define SUCCESS      	1
#define FAIL         	0
#define __IO			volatile



#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#endif /* INC_STM32F407XX_H_ */
