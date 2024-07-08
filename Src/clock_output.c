
#include "stm32f407xx.h"

#define RCC_CR_HSEON 	16
#define RCC_CR_HSERDY 	17
#define RCC_CFGR_MCO1 	21


#define RCC_CFGR_MCO1PRE 	24

#define	READ_RCC_CR_FLAG(flag) (RCC->CR &= 1<<flag )
// see hse clock output on pa8 pin of stm32
int main(){
	//enable hse clock
	RCC->CR  |= 1 << RCC_CR_HSEON;


	//WAIT TILL HSE STABILIZES
	while( !READ_RCC_CR_FLAG(RCC_CR_HSERDY));


	//SET THE SYSTEM CLOCK AS THE HSE
	RCC->CFGR |= 1<<0;


	//configure the mco1 as the hse;
	RCC->CFGR &= ~(0X3 << RCC_CFGR_MCO1);
	RCC->CFGR |= (0X2 << RCC_CFGR_MCO1);

			//SET THE PRE SCALER
	RCC->CFGR &= (0X8 << RCC_CFGR_MCO1PRE);
	RCC->CFGR |= 0X6 << RCC_CFGR_MCO1PRE;

	//enable CLOCK FOR GPIOA
		RCC->AHB1ENR |= 1<< AHB1ENR_GPIOAEN;


	//SET THE ALTERNATE FUNCTIONALITY MODE.
	GPIOA->MODER |= 0X2 << GPIO_MODER_PIN8;

	//SET THE ALTERNATE FUNCTIONALITY 
	GPIOA->AFR[1] &= 0XF << GPIO_AF_PIN_8;


	while(1);
	return 0;
}




