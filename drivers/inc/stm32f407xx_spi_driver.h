/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: May 15, 2024
 *      Author: harsh
 */
#include <stdint.h>
#include <stm32f407xx.h>
#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

typedef struct {



} SPI_Config_t;


typedef struct {
    SPI_Config_t  SPI_config;
    SPI_RegDef_t * pSPI;
}SPI_Handle_t;

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
