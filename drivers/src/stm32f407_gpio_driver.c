/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Apr 17, 2024
 *      Author: harsh
 */


#include <stm32f407xx_gpio_driver.h>

//GPIO init and deinit
void GPIO_Init(GPIO_Handle_t * gpioHandle){
	uint32_t temp = 0;
	uint32_t pin = gpioHandle->GPIO_PinConfig.gpioPinNumber ;
	uint8_t port = ((uint32_t)gpioHandle->pGPIOx - (uint32_t)GPIOA) / 0x400;

	GPIO_PCLKControl( gpioHandle->pGPIOx , ENABLE);
	
	gpioHandle->pGPIOx->PUPDR &= ~(0x3 << 2*pin);
	temp = gpioHandle->GPIO_PinConfig.gpioPuPdControl << 2*pin;
	gpioHandle->pGPIOx->PUPDR |= temp;

	if( gpioHandle->GPIO_PinConfig.gpioPinMode <= GPIO_MODE_ANALOG){
		// 1 configure the output mode of the pin
		gpioHandle->pGPIOx->MODER &= ~(0x3 << 2*(pin));
		temp = gpioHandle->GPIO_PinConfig.gpioPinMode << 2*pin;
		gpioHandle->pGPIOx->MODER |= temp;

		// 2 configure the output speed
		gpioHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2*pin);
		temp = gpioHandle->GPIO_PinConfig.gpioPinSpeed << 2*pin;
		gpioHandle->pGPIOx->OSPEEDR |= temp;

		// 4 configure the output type
		if( gpioHandle->GPIO_PinConfig.gpioPinAltFuncMode == GPIO_MODE_OUT){
			gpioHandle->pGPIOx->OTYPER &= ~(0x3 << pin);
			temp = gpioHandle->GPIO_PinConfig.gpioPinOpType << pin;
			gpioHandle->pGPIOx->OTYPER |= temp;
		}

		//5 configure the alt functionality of the pin.
		if(gpioHandle->GPIO_PinConfig.gpioPinMode == GPIO_MODE_ALTFN ){
			gpioHandle->pGPIOx->AFR[pin/8] &= ~(0xF << 4*(pin%8));
			temp = gpioHandle->GPIO_PinConfig.gpioPinAltFuncMode << 4*(pin%8);
			gpioHandle->pGPIOx->AFR[pin/8] |= temp;
		}

	}else{


		//enable the clock for system configuration peri
		SYSCFG_PCLK_EN();
		
		//set the pin in input mode
		gpioHandle->pGPIOx->MODER &= ~(0x3 << 2*(pin));
		
		//trigger selection
		if ( gpioHandle->GPIO_PinConfig.gpioPinMode == GPIO_MODE_IT_RT ){
			EXTI->RTSR |= 1 << pin;
			EXTI->FTSR &= ~(1 << pin);
		}
		if ( gpioHandle->GPIO_PinConfig.gpioPinMode == GPIO_MODE_IT_FT ){
			EXTI->FTSR |= 1 << pin;
			EXTI->RTSR &= ~(1 << pin);
		}
		if(gpioHandle->GPIO_PinConfig.gpioPinMode == GPIO_MODE_IT_FRT){
			EXTI->FTSR |= 1 << pin;
			EXTI->RTSR |= 1 << pin;

		}

		//configure the gpio port for the exti
		temp = pin/4;

		SYSCFG->EXTICR[temp] &= ~(0xF << (pin%4)*4);
		SYSCFG->EXTICR[temp] |= (port << (pin%4)*4);
		
		// enable interrupt delivery to processor
		EXTI->IMR |= 1 << pin;


	}

}

void GPIO_Deinit(GPIO_RegDef_t * pGPIO){
	uint32_t temp = 1 <<  ((uint32_t) pGPIO - (uint32_t) GPIOA )  % 0x400 ;
	RCC->AHB1RSTR |= temp;
	RCC->AHB1RSTR &= ~temp;
}

//GPIO peripheral clock control
void GPIO_PCLKControl(GPIO_RegDef_t * pGPIO , uint8_t EnorDi){

	if( EnorDi == ENABLE){
		switch ((uint32_t)pGPIO) {
			case (uint32_t)GPIOA:
				GPIOA_PCLK_EN();
				break;
			case (uint32_t)GPIOB:
				GPIOB_PCLK_EN();
				break;
			case (uint32_t)GPIOC:
				GPIOC_PCLK_EN();
				break;
			case (uint32_t)GPIOD:
				GPIOD_PCLK_EN();
				break;
			case (uint32_t)GPIOE:
				GPIOE_PCLK_EN();
				break;
			case (uint32_t)GPIOF:
				GPIOF_PCLK_EN();
				break;
			case (uint32_t)GPIOG:
				GPIOG_PCLK_EN();
				break;
			case (uint32_t)GPIOH:
				GPIOH_PCLK_EN();
				break;
			case (uint32_t)GPIOI:
				GPIOI_PCLK_EN();
				break;

		}
	}else if ( EnorDi == DISABLE){

		switch ((uint32_t)pGPIO) {
			case (uint32_t)GPIOA:
				GPIOA_PCLK_DI();
				break;
			case (uint32_t)GPIOB:
				GPIOB_PCLK_DI();
				break;
			case (uint32_t)GPIOC:
				GPIOC_PCLK_DI();
				break;
			case (uint32_t)GPIOD:
				GPIOD_PCLK_DI();
				break;
			case (uint32_t)GPIOE:
				GPIOE_PCLK_DI();
				break;
			case (uint32_t)GPIOF:
				GPIOF_PCLK_DI();
				break;
			case (uint32_t)GPIOG:
				GPIOG_PCLK_DI();
				break;
			case (uint32_t)GPIOH:
				GPIOH_PCLK_DI();
				break;
			case (uint32_t)GPIOI:
				GPIOI_PCLK_DI();
				break;

		}
	}


}

//GPIO read write from input and output
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t * pGPIO , uint8_t pinNumber){
	uint32_t inputData = pGPIO->IDR;
	if( inputData & 1 << pinNumber ){
		return 1;
	}
	return 0;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t * pGPIO){
	return (uint16_t) pGPIO->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t * pGPIO , uint8_t pinNumber, uint8_t StorRst){
	if( StorRst){
		pGPIO->ODR |= 1 << pinNumber;
	}else{
		pGPIO->ODR &= ~(1 << pinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t * pGPIO , uint16_t data){
	pGPIO->ODR = data;
}

//GPIO toggle pin
void GPIO_ToggleOutputPin(GPIO_RegDef_t * pGPIO , uint8_t pinNumber){
	pGPIO->ODR ^= (1 << pinNumber);
}


//GPIO IRQ config processor side
void GPIO_IRQConfig(uint8_t IRQNumber  , uint8_t EnorDi){

	if( EnorDi == ENABLE){
		// enable interrupt by ARM registers  
		*((uint32_t *)(NVIC_ISERx + (IRQNumber/32)*4)) |= 1 << IRQNumber%32; 

	}else{
		//disable interrupt by arm register
		*((uint32_t *)(NVIC_ICERx + (IRQNumber/32)*4)) |= 1 << IRQNumber%32; 
	}


}

// priority config processor side
void GPIO_IRQPriorityConfig( uint8_t IRQNumber, uint8_t IRQPriority){

	if ( IRQPriority <= NoChangeInPriority)
		return;
	
	*((uint32_t *)(NVIC_IPRx + (IRQNumber/4)*4)) &=  ~(0xFF << (IRQNumber%4)*8);
	*((uint32_t *)(NVIC_IPRx + (IRQNumber/4)*4)) |=  IRQPriority << (IRQNumber%4)*8;
		
}

void GPIO_IRQHandling(uint8_t pinNumber);
