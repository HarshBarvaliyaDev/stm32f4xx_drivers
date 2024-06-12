#include <stm32f407xx_gpio_driver.h>




volatile int control = 1;
int main(){

    GPIO_Handle_t LED;
    LED.pGPIOx = GPIOA;
    LED.GPIO_PinConfig.gpioPinNumber = GPIO_PIN_NUM_6;
    LED.GPIO_PinConfig.gpioPinMode = GPIO_MODE_OUT;
    LED.GPIO_PinConfig.gpioPinOpType = GPIO_TYPE_PP;
    LED.GPIO_PinConfig.gpioPinSpeed = GPIO_SPEED_MEDIUM;
    LED.GPIO_PinConfig.gpioPuPdControl = GPIO_NO_PUPD;

    GPIO_Handle_t SW_FAST;
    SW_FAST.pGPIOx = GPIOE;
    SW_FAST.GPIO_PinConfig.gpioPinMode = GPIO_MODE_IT_FT;
    SW_FAST.GPIO_PinConfig.gpioPinNumber = GPIO_PIN_NUM_4;
    SW_FAST.GPIO_PinConfig.gpioPinSpeed = GPIO_SPEED_MEDIUM;
    SW_FAST.GPIO_PinConfig.gpioPuPdControl = GPIO_PU;

    GPIO_Handle_t SW_SLOW;
    SW_SLOW.pGPIOx = GPIOE;
    SW_SLOW.GPIO_PinConfig.gpioPinMode = GPIO_MODE_IT_FT;
    SW_SLOW.GPIO_PinConfig.gpioPinNumber = GPIO_PIN_NUM_3;
    SW_SLOW.GPIO_PinConfig.gpioPinSpeed = GPIO_SPEED_MEDIUM;
    SW_SLOW.GPIO_PinConfig.gpioPuPdControl = GPIO_PU;

    GPIO_Init(&SW_FAST);
    GPIO_Init(&SW_SLOW);
    GPIO_Init(&LED);

    GPIO_IRQConfig(9 , ENABLE);
    GPIO_IRQConfig(10 , ENABLE);

    while(1){

        while(control){
            GPIO_ToggleOutputPin( LED.pGPIOx , LED.GPIO_PinConfig.gpioPinNumber);
            delay(500);
        }
        while(!control){
            GPIO_ToggleOutputPin( LED.pGPIOx , LED.GPIO_PinConfig.gpioPinNumber);
            delay(100);
        }
    }

}


void EXTI3_IRQHandler(void){ // slow switch     SW_SLOW
    //make the global variable "control" 1
	delay(100); //this delay is for switch debouncing
    control = 1;
    EXTI->PR |= 1 << GPIO_PIN_NUM_3;
}

void EXTI4_IRQHandler(void){ // fast switch     SW_FAST
    //make the global variable "control" 0
	delay(100); //this delay is for switch debouncing
    control = 0;
    EXTI->PR |= 1 << GPIO_PIN_NUM_4;
}
