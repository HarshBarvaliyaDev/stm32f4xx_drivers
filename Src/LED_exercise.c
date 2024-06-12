
#include <stm32f407xx_gpio_driver.h>


int main(){
    
    GPIO_Handle_t LED;
    LED.pGPIOx = GPIOA;
    LED.GPIO_PinConfig.gpioPinNumber = GPIO_PIN_NUM_6;
    LED.GPIO_PinConfig.gpioPinMode = GPIO_MODE_OUT;
    LED.GPIO_PinConfig.gpioPinOpType = GPIO_TYPE_OD;
    LED.GPIO_PinConfig.gpioPinSpeed = GPIO_SPEED_HIGH;
    LED.GPIO_PinConfig.gpioPuPdControl = GPIO_PU;

    // GPIO_PCLKControl(LED.pGPIOx , ENABLE);
    GPIOA_PCLK_EN();

    GPIO_Init(&LED);
    while(1){
        GPIO_ToggleOutputPin( LED.pGPIOx , LED.GPIO_PinConfig.gpioPinNumber);
        delay(100);
    }

    return 0;
}



