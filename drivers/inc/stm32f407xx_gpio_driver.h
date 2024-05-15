/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Apr 17, 2024
 *      Author: harsh
 */
#include <stdint.h>
#include <stm32f407xx.h>

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


typedef struct {
    uint8_t gpioPinNumber;					// possible values @GPIO_PIN_NUM
    uint8_t gpioPinMode;					// possible values @GPIO_MODE
    uint8_t gpioPinSpeed;					// possible values @GPIO_SPEED
    uint8_t gpioPuPdControl;				// possible values @GPIO_PUPD
    uint8_t gpioPinOpType;					// possible values @GPIO_TYPE
    uint8_t gpioPinAltFuncMode;
} GPIO_PinConfig_t;


typedef struct{
    GPIO_RegDef_t * pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

//****************** MACROS FOR GPIO PIN NUMBER @GPIO_PIN_NUM
#define GPIO_PIN_NUM_0			0
#define GPIO_PIN_NUM_1			1
#define GPIO_PIN_NUM_2			2
#define GPIO_PIN_NUM_3			3
#define GPIO_PIN_NUM_4			4
#define GPIO_PIN_NUM_5			5
#define GPIO_PIN_NUM_6			6
#define GPIO_PIN_NUM_7			7
#define GPIO_PIN_NUM_8			8
#define GPIO_PIN_NUM_9			9
#define GPIO_PIN_NUM_10			10
#define GPIO_PIN_NUM_11			11
#define GPIO_PIN_NUM_12			12
#define GPIO_PIN_NUM_13			13
#define GPIO_PIN_NUM_14			14
#define GPIO_PIN_NUM_15			15

//****************** MACROS FOR GPIO PIN NUMBER
//****************** MACROS FOR MODE OF THE GPIO PIN @GPIO_MODE
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4   //GPIO trigger interrupt on falling edge
#define GPIO_MODE_IT_RT			5	//GPIO trigger interrupt on rising edge
#define GPIO_MODE_IT_FRT		6	//GPIO trigger interrupt on both edges
//****************** MACROS FOR MODE OF THE GPIO PIN


//****************** MACROS FOR GPIO PIN TYPE @GPIO_TYPE
#define GPIO_TYPE_PP			0 	// GPIO type push pull
#define GPIO_TYPE_OD			1	// GPIO type open drain.
//*******************MACROS FOR GPIO PIN TYPE


//****************** MACROS FOR GPIO PIN SPEED @GPIO_SPEED
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VHIGH		3
//****************** MACROS FOR GPIO PIN SPEED


//****************** MACROS FOR GPIO PIN PULL UP AND PULL DOWN CONFIGURATION @GPOI_PUPD
#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2
//****************** MACROS FOR GPIO PIN PULL UP AND PULL DOWN CONFIGURATION

/************************************************************
 *
 * @fn				GPIO_PCLKControl
 *
 * @brief 			Enables or disables the clock of the GPIO Port.
 *
 * @param[0]		pGPIO - base address of the port
 * @param[1]		EnorDi - ENABLE or DISABLE macros.
 *
 * @return			None.
 *
 * @note
 ************************************************************/

void GPIO_PCLKControl(GPIO_RegDef_t * pGPIO , uint8_t EnorDi);


/************************************************************
 *
 * @fn				GPIO_Init
 *
 * @brief 			Initializes the GPIO we want to use
 *
 * @param[0]		gpioHandle - address to GPIO_Handle_t struct ( stm32f407xx_gpio_driver.h)
 *
 * @return			None.
 *
 * @note			None.
 ************************************************************/

void GPIO_Init(GPIO_Handle_t * gpioHandle);

/************************************************************
 *
 * @fn				GPIO_Deinit
 *
 * @brief 			deinitializes the GPIO we want to use
 *
 * @param[0]		pGPIO - base address of the port
 *
 * @return			None.
 *
 * @note			None.
 ************************************************************/
void GPIO_Deinit(GPIO_RegDef_t * pGPIO);



/************************************************************
 *
 * @fn				GPIO_ReadFromInputPin
 *
 * @brief 			Read from the input pin
 *
 * @param[0]		pGPIO - base address of the port
 * @param[1]		pinNumber - number of the pin of which you want to read the value.
 *
 * @return			0 or 1.
 *
 * @note
 ************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t * pGPIO , uint8_t pinNumber);


/************************************************************
 *
 * @fn				GPIO_ReadFromInputPort
 *
 * @brief 			Read from the whole port
 *
 * @param[0]		pGPIO - base address of the port (16 pins)
 *
 *
 * @return			16 bit unsigned integer.
 *
 * @note
 ************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t * pGPIO);



/************************************************************
 *
 * @fn				GPIO_WriteToOutputPin
 *
 * @brief 			write 0 or 1 to an output pin.
 *
 * @param[0]		pGPIO - base address of the port
 * @param[1]		pinNumber - pin number at which you want to write.
 * @param[2]		StorRst - SET or RESET macros ( stm32f407xx.h)
 * @return			None.
 *
 * @note
 ************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t * pGPIO , uint8_t pinNumber, uint8_t StorRst);

/************************************************************
 *
 * @fn				GPIO_WriteToOutputPort
 *
 * @brief 			write value to the whole port (16 pin)
 *
 * @param[0]		pGPIO - base address of the port
 * @param[1]		data - data you want to write.
 *
 * @return			None.
 *
 * @note
 ************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t * pGPIO , uint16_t data);


/************************************************************
 *
 * @fn				GPIO_ToggleOutputPin
 *
 * @brief 			toggle the pin.
 *
 * @param[0]		pGPIO - base address of the port
 * @param[1]		pinNumber - number of the pin you want to toggle.
 *
 * @return			None.
 *
 * @note
 ************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t * pGPIO , uint8_t pinNumber);

/************************************************************
 *
 * @fn				GPIO_IRQConfig
 *
 * @brief 			Enables or disables the clock of the GPIO Port.
 *
 * @param[0]		IRQNumber - interrupt number at which the EXTI will deliver the interrupt to nvic
 * @param[1]		EnorDi - ENABLE or DISABLE macros ( stm32f407xx_gpio_driver.h)
 *
 * @return			None.
 *
 * @note
 ************************************************************/

void GPIO_IRQConfig(uint8_t IRQNumber  , uint8_t EnorDi);


/************************************************************
 *
 * @fn				GPIO_IRQPriorityConfig
 *
 * @brief 			set the priority
 *
 * @param[0]		IRQNumber - interrupt number at which the EXTI will deliver the interrupt to nvic
 * @param[1]		IRQPriority - priority you want to set to the irq number.
 *
 * @return			None.
 *
 * @note
 ************************************************************/
void GPIO_IRQPriorityConfig( uint8_t IRQNumber, uint8_t IRQPriority);

/************************************************************
 *
 * @fn				GPIO_IRQHandling
 *
 * @brief 			Enables or disables the clock of the GPIO Port.
 *
 * @param[0]		pinNumber - pin number from which interrupt has come.

 *
 * @return			None.
 *
 * @note
 ************************************************************/
void GPIO_IRQHandling(uint8_t pinNumber);
#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
