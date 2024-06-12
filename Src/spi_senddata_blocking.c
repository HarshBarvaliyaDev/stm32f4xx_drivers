
// pb12 	spi2_nss
// pb13		spi2_sck
// pb14		spi2_miso
// pb15		spi2_mosi

#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>
#include <stm32f407xx_spi_driver.h>

int main(){

	

	// initialize gpio handle
	GPIO_Handle_t spi_pins ;
	spi_pins.pGPIOx = GPIOB;
	spi_pins.GPIO_PinConfig.gpioPinAltFuncMode = 5;
	spi_pins.GPIO_PinConfig.gpioPinMode = GPIO_MODE_ALTFN;
	spi_pins.GPIO_PinConfig.gpioPinOpType = GPIO_TYPE_PP;
	spi_pins.GPIO_PinConfig.gpioPuPdControl = GPIO_NO_PUPD;

	// nss 
	spi_pins.GPIO_PinConfig.gpioPinNumber = GPIO_PIN_NUM_12;
	GPIO_Init(&spi_pins);

	//sck
	spi_pins.GPIO_PinConfig.gpioPinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&spi_pins);

	//miso
	spi_pins.GPIO_PinConfig.gpioPinNumber = GPIO_PIN_NUM_14;
	GPIO_Init(&spi_pins);

	// mosi
	spi_pins.GPIO_PinConfig.gpioPinNumber = GPIO_PIN_NUM_15;
	GPIO_Init(&spi_pins);
	

	SPI_Handle_t spiHandle;
	spiHandle.pSPI = SPI2;
	spiHandle.SPI_config.SPI_BusConfig = SPI_BUSCONFIG_FD;
	spiHandle.SPI_config.SPI_CPHA = SPI_CPHA_FIRST;
	spiHandle.SPI_config.SPI_CPOL = AT_IDLE_0;
	spiHandle.SPI_config.SPI_DeviceMode = SPI_DEVICEMODE_MASTER;
	spiHandle.SPI_config.SPI_DFF = SPI_DFF_8;
	spiHandle.SPI_config.SPI_IRQtype = SPI_NOIRQ;
	spiHandle.SPI_config.SPI_MSB_LSB = SPI_MSB_FIRST;
	spiHandle.SPI_config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	spiHandle.SPI_config.SPI_SSM = SOFTWARE_SLAVE_EN;
	SPI_init(&spiHandle);

	char data[] = "hello world!"; 

	SPI_PeripheralControl(SPI2 , ENABLE);
	SPI_sendData_blocking(spiHandle.pSPI , data , sizeof( data));
	return 0;
}