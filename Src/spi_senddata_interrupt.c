


#include <stm32f407xx_gpio_driver.h>
#include <stm32f407xx_spi_driver.h>




SPI_Handle_t SPI_GLOBAL_STRUCT;


void main(){
    char data_to_send[] = "hello world!";

    
    SPI_GLOBAL_STRUCT.pSPI                          = SPI1;
    // SPI_GLOBAL_STRUCT.SPI_TxBuffer                  = data_to_send;
    // SPI_GLOBAL_STRUCT.SPI_TxLength                  = sizeof(data_to_send);
    SPI_GLOBAL_STRUCT.SPI_config.SPI_DeviceMode     = SPI_DEVICEMODE_MASTER;
    SPI_GLOBAL_STRUCT.SPI_config.SPI_BusConfig      = SPI_BUSCONFIG_FD;
    SPI_GLOBAL_STRUCT.SPI_config.SPI_DFF            = SPI_DFF_8;
    SPI_GLOBAL_STRUCT.SPI_config.SPI_CPHA           = SPI_CPHA_FIRST;
    SPI_GLOBAL_STRUCT.SPI_config.SPI_CPOL           = AT_IDLE_0;
    SPI_GLOBAL_STRUCT.SPI_config.SPI_SSM            = SOFTWARE_SLAVE_DI;
    SPI_GLOBAL_STRUCT.SPI_config.SPI_SclkSpeed      = SPI_SCLK_SPEED_DIV2;
    SPI_GLOBAL_STRUCT.SPI_config.SPI_MSB_LSB        = SPI_MSB_FIRST;
    SPI_GLOBAL_STRUCT.SPI_config.SPI_IRQtype        = SPI_NOIRQ;


    GPIO_Handle_t LED;
    LED.pGPIOx = GPIOA;
    LED.GPIO_PinConfig.gpioPinNumber = GPIO_PIN_NUM_6;
    LED.GPIO_PinConfig.gpioPinMode = GPIO_MODE_OUT;
    LED.GPIO_PinConfig.gpioPinOpType = GPIO_TYPE_OD;
    LED.GPIO_PinConfig.gpioPinSpeed = GPIO_SPEED_HIGH;
    LED.GPIO_PinConfig.gpioPuPdControl = GPIO_PU;

    // GPIO_PCLKControl(LED.pGPIOx , ENABLE);
    GPIOA_PCLK_EN();
    SPI_sendData_byInterrupt( &SPI_GLOBAL_STRUCT , sizeof(data_to_send) , data_to_send);

    GPIO_Init(&LED);
    while(1){
        GPIO_ToggleOutputPin( LED.pGPIOx , LED.GPIO_PinConfig.gpioPinNumber);
        delay(100);
    }

}

void SPI1_IRQHandler(void){
    SPI_IRQHandling(&SPI_GLOBAL_STRUCT);
}