
#include <stm32f407xx_spi_driver.h>


void SPI_PCLKClockControl ( SPI_RegDef_t * pSPI , uint8_t EnOrDi) {

    if ( EnOrDi == ENABLE){
        switch ((uint32_t) pSPI)
        {
        case SPI1_BASEADDR:
                SPI1_PCLK_EN();
            break;
        case SPI2_BASEADDR:
                SPI2_PCLK_EN();
            break;
        case SPI3_BASEADDR:
                SPI3_PCLK_EN();        
            break;
        }
    }else if (EnOrDi == DISABLE){
        switch ((uint32_t) pSPI)
        {
        case SPI1_BASEADDR:
                SPI1_PCLK_DI();
            break;
        case SPI2_BASEADDR:
                SPI2_PCLK_DI();
            break;
        case SPI3_BASEADDR:
                SPI3_PCLK_DI();        
            break;
        }

    }
}

void SPI_deinit ( SPI_RegDef_t* pSPI) {

    switch ((uint32_t) pSPI)
    {
        case SPI1_BASEADDR:
            SPI1_REG_RST();
            break;
        case SPI2_BASEADDR:
            SPI2_REG_RST();
            break;
        case SPI3_BASEADDR:
            SPI3_REG_RST();
            break;
    }
}

void SPI_IRQConfig(uint8_t IRQNumber  , uint8_t EnorDi){
    if (EnorDi == ENABLE){
        *(uint32_t *)(NVIC_ISERx + (IRQNumber/32) * 4 ) |= 1 << (IRQNumber/32);
    }
    else if (EnorDi == DISABLE){
        *(uint32_t *)(NVIC_ICERx + (IRQNumber/32) * 4 ) |= 1 << (IRQNumber/32);
    }
}

void SPI_IRQPriorityConfig( uint8_t IRQNumber, uint8_t IRQPriority){
    
    *(uint32_t *)(NVIC_IPRx + IRQNumber/4) &= ~(0xff << (IRQNumber % 4)*8);
    *(uint32_t *)(NVIC_IPRx + IRQNumber/4) |= ( IRQPriority << (IRQNumber % 4)*8);
}



void SPI_init ( SPI_Handle_t* spi_handle) {

    uint32_t tempRegCR1 = 0;

    //configure master mode
    tempRegCR1 |= spi_handle->SPI_config.SPI_DeviceMode << SPI_CR1_MSTR;

    //configure bus config
    switch (spi_handle->SPI_config.SPI_BusConfig )
    {
        case SPI_BUSCONFIG_FD:
            // no need to do anything
            break;
        case SPI_BUSCONFIG_HD:
            // set bidi mode bit
            tempRegCR1 |= 1 << SPI_CR1_BIDIMODE;
            break;
        case SPI_BUSCONFIG_SIMPLEX_RX:
            // clear bidimode and set rx only bit
            tempRegCR1 |= 1 << SPI_CR1_RXONLY;
            break;
    }

    //configure DFF
    tempRegCR1 |= spi_handle->SPI_config.SPI_DFF << SPI_CR1_DFF;

    //configure cpha
    tempRegCR1 |= spi_handle->SPI_config.SPI_CPHA << SPI_CR1_CPHA;

    //configure cpol
    tempRegCR1 |= spi_handle->SPI_config.SPI_CPOL << SPI_CR1_CPOL;

    //configure ssm
    tempRegCR1 |= spi_handle->SPI_config.SPI_SSM << SPI_CR1_SSM;
    // configure ssi bit
    if ( spi_handle->SPI_config.SPI_SSM == SOFTWARE_SLAVE_EN){
        if(spi_handle->SPI_config.SPI_DeviceMode ==  SPI_DEVICEMODE_MASTER){
            tempRegCR1 |= 1<< SPI_CR1_SSI;
        }
        // no need to set for slave as default value in tempReg is 0;
    }

    //configure sclkspeed
    tempRegCR1 |= spi_handle->SPI_config.SPI_SclkSpeed << SPI_CR1_BR;

    //configure msb or lsb first
    tempRegCR1 |= spi_handle->SPI_config.SPI_MSB_LSB << SPI_CR1_LSBFIRST;


    spi_handle->pSPI->CR1 = tempRegCR1;

}

