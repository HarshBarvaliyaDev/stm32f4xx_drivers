
#include <stm32f407xx_spi_driver.h>

static void spi_rx_interrupt_handler(SPI_Handle_t * spi_handle_ptr);
static void spi_tx_interrupt_handler(SPI_Handle_t * spi_handle_ptr);
static void spi_error_interrupt_handler(SPI_Handle_t * spi_handle_ptr);


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




void SPI_init ( SPI_Handle_t* spi_handle_ptr) {

    //enable the peripheral clock
    SPI_PCLKClockControl(spi_handle_ptr->pSPI , ENABLE);

    uint32_t tempRegCR1 = 0;

    //configure master mode
    tempRegCR1 |= spi_handle_ptr->SPI_config.SPI_DeviceMode << SPI_CR1_MSTR;

    //configure bus config
    switch (spi_handle_ptr->SPI_config.SPI_BusConfig )
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
    tempRegCR1 |= spi_handle_ptr->SPI_config.SPI_DFF << SPI_CR1_DFF;

    //configure cpha
    tempRegCR1 |= spi_handle_ptr->SPI_config.SPI_CPHA << SPI_CR1_CPHA;

    //configure cpol
    tempRegCR1 |= spi_handle_ptr->SPI_config.SPI_CPOL << SPI_CR1_CPOL;

    //configure ssm
    tempRegCR1 |= spi_handle_ptr->SPI_config.SPI_SSM << SPI_CR1_SSM;
    // configure ssi bit
    if ( spi_handle_ptr->SPI_config.SPI_SSM == SOFTWARE_SLAVE_EN){
        if(spi_handle_ptr->SPI_config.SPI_DeviceMode ==  SPI_DEVICEMODE_MASTER){
            tempRegCR1 |= 1<< SPI_CR1_SSI;
        }
        // no need to set for slave as default value in tempReg is 0;
    }

    //configure sclkspeed
    tempRegCR1 |= spi_handle_ptr->SPI_config.SPI_SclkSpeed << SPI_CR1_BR;

    //configure msb or lsb first
    tempRegCR1 |= spi_handle_ptr->SPI_config.SPI_MSB_LSB << SPI_CR1_LSBFIRST;


    spi_handle_ptr->pSPI->CR1 = tempRegCR1;

    // switch (spi_handle_ptr->SPI_config.SPI_IRQtype) // TXEIE and RXNEIE is set in SPI_sendData_byInterrupt and SPI_receiveData_byInterrupt so no need to set here
    // {
    // case SPI_TX_IRQ:
    //     spi_handle_ptr->pSPI->CR2 |= (1<<SPI_CR2_TXEIE) ;
    //     break;
    // case SPI_RX_IRQ:
    //     spi_handle_ptr->pSPI->CR2 |= (1<<SPI_CR2_RXNEIE) ;
    //     break;
    // case SPI_TXRX_IRQ:
    //     spi_handle_ptr->pSPI->CR2 |= (1<<SPI_CR2_TXEIE) | (1<<SPI_CR2_RXNEIE);
    //     break;
    // case SPI_NOIRQ:
    //     // default no interrupt will be generated so no need to do anything;
    //     break;

    // }


}

void SPI_PeripheralControl(SPI_RegDef_t* pSPI, uint8_t EnOrDi){

    if ( EnOrDi == ENABLE){

        pSPI->CR1 |= 1 << SPI_CR1_SPE;

    }else {

        pSPI->CR1 &= ~(1 << SPI_CR1_SPE);
        
    }
}

uint8_t SPI_sendData_blocking ( SPI_RegDef_t* pSPI , uint8_t * TxBuffer , uint32_t len){
    
    if ( !READ_DFF_SPI_CR1 (pSPI)){

        while( len > 0){ // when data fram format is of 8 bit
            
            while( !READ_TXE_SPI_SR(pSPI) ); // wait for the txe flag
            pSPI->DR = *TxBuffer;
            TxBuffer += 1;
            len--;
        }

        return SUCCESS;
    }

    if ( READ_DFF_SPI_CR1(pSPI)){

        while( len > 0 ){ // when data fram format is of 16 bit
            while( !READ_TXE_SPI_SR(pSPI) ); // wait for the txe flag
            pSPI->DR = *((uint16_t *)TxBuffer);
            TxBuffer = ((uint16_t *)TxBuffer) + 1;
            len--;
            len--;  
        }
        return SUCCESS;

    }

    return FAIL;
}


uint8_t SPI_receiveData_blocking ( SPI_RegDef_t* pSPI , uint8_t * RxBuffer , uint32_t len){
    
    if ( !READ_DFF_SPI_CR1 (pSPI)){

        while( len > 0){ // when data fram format is of 8 bit
            
            while( !READ_RXNE_SPI_SR(pSPI) ); // wait for the RXNE flag
            *RxBuffer = pSPI->DR ;
            RxBuffer++;
            len--;
        }

        return SUCCESS;
    }

    if ( READ_DFF_SPI_CR1(pSPI)){

        while( len > 1 ){ // when data fram format is of 16 bit
            while( !READ_RXNE_SPI_SR(pSPI) ); // wait for the RXNE flag
            *((uint16_t *)RxBuffer) = pSPI->DR ;
            RxBuffer++;
            RxBuffer++;
            len--;
            len--;
        }

        return SUCCESS;

    }

    return FAIL;
}


void SPI_IRQConfig(uint8_t IRQNumber  , uint8_t EnorDi){
    if (EnorDi == ENABLE){
        *(NVIC_ISERx + (IRQNumber/32)  ) |= 1 << (IRQNumber/32);

        
    }
    else if (EnorDi == DISABLE){
        *(NVIC_ICERx + (IRQNumber/32) ) |= 1 << (IRQNumber/32);

        
    }
}

void SPI_IRQPriorityConfig( uint8_t IRQNumber, uint8_t IRQPriority){
    
    *(uint32_t *)(NVIC_IPRx + IRQNumber/4) &= ~(0xff << (IRQNumber % 4)*8);
    *(uint32_t *)(NVIC_IPRx + IRQNumber/4) |= ( IRQPriority << (IRQNumber % 4)*8);
}



uint8_t SPI_sendData_byInterrupt( SPI_Handle_t * spi_handle_ptr , uint32_t dataLenght, uint8_t * dataBuffer){
    // spi_handle_ptr is pointer to global SPI_Handle_t stucture. 
    if ( spi_handle_ptr -> SPI_TxState != SPI_BUSY_IN_TX){ // checking if the structure(peripheral) is busy or free.

        spi_handle_ptr->SPI_TxBuffer = dataBuffer; // storing the data starting address to global structure

        spi_handle_ptr->SPI_TxLength = dataLenght; //// storing the data length to global structure

        spi_handle_ptr->SPI_TxState = SPI_BUSY_IN_TX;   // setting the state to busy in transmission.

        spi_handle_ptr->pSPI->CR2 |= (1<<SPI_CR2_TXEIE) ; // enabling interrupt in control register 2 in spi peripheral
    }

    return spi_handle_ptr->SPI_TxState;
}

uint8_t SPI_receiveData_byInterrupt( SPI_Handle_t * spi_handle_ptr , uint32_t dataLenght, uint8_t * dataBuffer){

        // spi_handle_ptr is pointer to global SPI_Handle_t stucture. 
    if ( spi_handle_ptr -> SPI_RxState != SPI_BUSY_IN_RX){ // checking if the structure(peripheral) is busy or free.

        spi_handle_ptr->SPI_RxBuffer = dataBuffer; // storing the data starting address to global structure

        spi_handle_ptr->SPI_RxLength = dataLenght; //// storing the data length to global structure

        spi_handle_ptr->SPI_RxState = SPI_BUSY_IN_RX;   // setting the state to busy in receiving.

        spi_handle_ptr->pSPI->CR2 |= (1<<SPI_CR2_RXNEIE) ; // enabling interrupt in control register 2 in spi peripheral
    }

    return spi_handle_ptr->SPI_RxState;
}

void SPI_IRQHandling(SPI_Handle_t * spi_handle_ptr){
    if( READ_RXNE_SPI_SR(spi_handle_ptr->pSPI)){

        spi_rx_interrupt_handler(spi_handle_ptr);

    }
    if( READ_TXE_SPI_SR(spi_handle_ptr->pSPI)){

        spi_tx_interrupt_handler(spi_handle_ptr);

    }
    
}

static void spi_rx_interrupt_handler(SPI_Handle_t * spi_handle_ptr){
        if( !READ_DFF_SPI_CR1(spi_handle_ptr->pSPI) ){ // when data frame format is of 8 bit
        *(spi_handle_ptr->SPI_RxBuffer) = spi_handle_ptr->pSPI->DR;

        spi_handle_ptr->SPI_RxBuffer += 1;

        spi_handle_ptr->SPI_RxLength -= 1;


    }
    else if( READ_DFF_SPI_CR1(spi_handle_ptr->pSPI) ){  // when data frame format is of 16 bit

        if ( spi_handle_ptr->SPI_RxLength > 0){ 

            *(uint16_t *)(spi_handle_ptr->SPI_TxBuffer) = spi_handle_ptr->pSPI->DR ;

            spi_handle_ptr->SPI_RxBuffer += 2;

            spi_handle_ptr->SPI_RxLength -= 2;
        }
        // else if ( spi_handle_ptr->SPI_TxLength == 1){    // case when data length is in odd number is handled in transmission side. , so we dont need this in receiver side
        //     spi_handle_ptr->pSPI->DR = 0 | *(spi_handle_ptr->SPI_TxBuffer);

        //     spi_handle_ptr->SPI_TxBuffer += 2;

        //     spi_handle_ptr->SPI_TxLength -= 1; // now data length becomes 0 , so we have to set the state to ready


        // }
    }

    if ( spi_handle_ptr->SPI_TxLength <= 0){
        spi_handle_ptr->SPI_RxState = SPI_READY;
        spi_handle_ptr->pSPI->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    }
}

static void spi_tx_interrupt_handler(SPI_Handle_t * spi_handle_ptr){
    if( !READ_DFF_SPI_CR1(spi_handle_ptr->pSPI) ){ // when data frame format is of 8 bit
        spi_handle_ptr->pSPI->DR = *(spi_handle_ptr->SPI_TxBuffer);

        spi_handle_ptr->SPI_TxBuffer += 1;

        spi_handle_ptr->SPI_TxLength -= 1;


    }
    else if( READ_DFF_SPI_CR1(spi_handle_ptr->pSPI) ){  // when data frame format is of 16 bit

        if ( spi_handle_ptr->SPI_TxLength >= 2){

            spi_handle_ptr->pSPI->DR = *(uint16_t *)(spi_handle_ptr->SPI_TxBuffer);

            spi_handle_ptr->SPI_TxBuffer += 2;

            spi_handle_ptr->SPI_TxLength -= 2;
        }
        else if ( spi_handle_ptr->SPI_TxLength == 1){
            spi_handle_ptr->pSPI->DR = 0 | *(spi_handle_ptr->SPI_TxBuffer);

            spi_handle_ptr->SPI_TxBuffer += 2;

            spi_handle_ptr->SPI_TxLength -= 1; // now data length becomes 0 , so we have to set the state to ready


        }
    }

    if ( !spi_handle_ptr->SPI_TxLength ){
        spi_handle_ptr->SPI_TxState = SPI_READY;
        spi_handle_ptr->pSPI->CR2 &= ~(1 << SPI_CR2_TXEIE);
    }
}

static void spi_error_interrupt_handler(SPI_Handle_t * spi_handle_ptr){
    
}