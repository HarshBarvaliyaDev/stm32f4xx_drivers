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

// spi configureation specifc macros

#define LSB                 1
#define MSB                 0


typedef struct {

    uint8_t SPI_DeviceMode;         // @SPI_DeviceMode
    uint8_t SPI_BusConfig;          // @SPI_BusConfig
    uint8_t SPI_DFF;                // @SPI_DFF
    uint8_t SPI_CPHA;               // possibe value is 0 or 1
    uint8_t SPI_CPOL;               // possibe value is 0 or 1
    uint8_t SPI_SSM;                // @SPI_SSM
    uint8_t SPI_SclkSpeed;          // @SPI_SclkSpeed
    uint8_t SPI_MSB_LSB ;            // @SPI_MSB_LSB
    uint8_t SPI_IRQtype;
} SPI_Config_t;

// @SPI_DeviceMode
#define SPI_DEVICEMODE_SLAVE            0
#define SPI_DEVICEMODE_MASTER           1


// @SPI_BusConfig
#define SPI_BUSCONFIG_FD                        0
#define SPI_BUSCONFIG_HD                        1
#define SPI_BUSCONFIG_SIMPLEX_RX                2

// @SPI_DFF
#define SPI_DFF_8               0
#define SPI_DFF_16              1

// @SPI_CPHA
#define SPI_CPHA_FIRST          0 // first clock transition is data capture edge
#define SPI_CPHA_SECOND          1 // first clock transition is data capture edge

//@ SPI_CPOL
#define AT_IDLE_0               0 // CLOCK VALUE AT IDLE WILL BE 0
#define AT_IDLE_1               1 // CLOCK VALUE AT IDLE WILL BE 1


// @SPI_SSM
#define SOFTWARE_SLAVE_EN       1
#define SOFTWARE_SLAVE_DI       0

// @SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2        0    
#define SPI_SCLK_SPEED_DIV4        1          
#define SPI_SCLK_SPEED_DIV8        2          
#define SPI_SCLK_SPEED_DIV16       3          
#define SPI_SCLK_SPEED_DIV32       4          
#define SPI_SCLK_SPEED_DIV64       5          
#define SPI_SCLK_SPEED_DIV128      6          
#define SPI_SCLK_SPEED_DIV256      7           

// @SPI_MSB_LSB
#define SPI_LSB_FIRST               1
#define SPI_MSB_FIRST               0

// @SPI_IRQtype @irq_type
#define SPI_TX_IRQ              0  // interrupt processor on completion of transmission 
#define SPI_RX_IRQ              1   // interrupt processor on completion of receive.
#define SPI_TXRX_IRQ            2   // interrupt processor on completion of both of above
#define SPI_NOIRQ               3


//spi bit macros for SPI_CR1 register BIT POSITIONS
#define SPI_CR1_CPHA            0
#define SPI_CR1_CPOL            1
#define SPI_CR1_MSTR            2
#define SPI_CR1_BR              3
#define SPI_CR1_SPE             6
#define SPI_CR1_LSBFIRST        7
#define SPI_CR1_SSI             8
#define SPI_CR1_SSM             9
#define SPI_CR1_RXONLY          10
#define SPI_CR1_DFF             11
#define SPI_CR1_CRCNEXT         12
#define SPI_CR1_CRCEN           13
#define SPI_CR1_BIDIOE          14
#define SPI_CR1_BIDIMODE        15

//spi bit macros for SPI_CR2 register BIT POSITIONS
#define SPI_CR2_RXDMAEN         0
#define SPI_CR2_TXDMAEN         1
#define SPI_CR2_SSOE            2
#define SPI_CR2_FRF             4
#define SPI_CR2_ERRIE           5
#define SPI_CR2_RXNEIE          6
#define SPI_CR2_TXEIE           7

//spi bit macros for SPI_SR register BIT POSITIONS

#define SPI_SR_RXNE             0 //receiver not empty
#define SPI_SR_TXE              1 //transmitter empty
#define SPI_SR_CHSIDE           2
#define SPI_SR_UDR              3
#define SPI_SR_CRCERR           4
#define SPI_SR_MODF             5
#define SPI_SR_OVR              6
#define SPI_SR_BSY              7
#define SPI_SR_FRE              8





typedef struct {
    SPI_Config_t  SPI_config;
    SPI_RegDef_t * pSPI;
}SPI_Handle_t;


//macro for reading dff bit in cr1 register
#define READ_DFF_SPI_CR1( peripheral_baseAddr) (peripheral_baseAddr->CR1 &= 1<<SPI_CR1_DFF)


// macros for read txe and rxne flag in spi status register 
#define READ_TXE_SPI_SR( peripheral_baseAddr) (peripheral_baseAddr->SR &= 1<<SPI_SR_TXE)
#define READ_RXNE_SPI_SR( peripheral_baseAddr) (peripheral_baseAddr->SR &= 1<<SPI_SR_RXNE)



/************************************************************
 *
 * @fn				SPI_PCLKClockControl
 *
 * @brief 			initialize spi peripheral
 *
 * @param[0]		pSPI - SPI_Handle_t structure pointer which contains all the info we need to set
 * @param[1]		EnOrDi - Enable or Disable 
 *
 * @return			nothing.
 *
 * @note
 ************************************************************/
void SPI_PCLKClockControl ( SPI_RegDef_t * pSPI , uint8_t EnOrDi) ;


/************************************************************
 *
 * @fn				SPI_init
 *
 * @brief 			initialize spi peripheral
 *
 * @param[0]		spi_handle_ptr - SPI_Handle_t structure pointer which contains all the info we need to set
 *
 * @return			nothing.
 *
 * @note
 ************************************************************/
void SPI_init ( SPI_Handle_t* spi_handle_ptr) ;

/************************************************************
 *
 * @fn				SPI_deinit
 *
 * @brief 			de-initialize spi peripheral
 *
 * @param[0]		pSPI - SPI_RegDef_t - base address of the peripheral we want to reset
 *
 * @return			nothing.
 *
 * @note
 ************************************************************/
void SPI_deinit ( SPI_RegDef_t* pSPI) ;

/************************************************************
 *
 * @fn				SPI_sendData_blocking
 *
 * @brief 			send data over spi peripheral
 *
 * @param[0]		pSPI - SPI_RegDef_t - base address of the peripheral we want to reset
 * @param[1]        TxBuffer - 8 bit we want to send
 * @param[2]        len        - length of the data
 * @return			0 or 1 , depending on the success of the transmission
 *
 * @note            this function can transmit arrays also , but we have to provide length in bytes
 ************************************************************/
uint8_t SPI_sendData_blocking ( SPI_RegDef_t* pSPI , uint8_t * TxBuffer , uint32_t len);

/************************************************************
 *
 * @fn				SPI_receiveData
 *
 * @brief 			receive data over spi peripheral
 *
 * @param[0]		pSPI - SPI_RegDef_t - base address of the peripheral we want to reseive
 * @param[1]        RxBuffer - 8 bit we want to receive .
 * @param[2]        len        - length of the data that we are going to receive
 * 
 * @return			returns the 8 bit that we received.
 *
 * @note            this api will block the execution as it uses while loop
 ************************************************************/
uint8_t SPI_receiveData ( SPI_RegDef_t* pSPI , uint8_t * RxBuffer , uint32_t len);


/************************************************************
 *
 * @fn				SPI_IRQPriorityConfig
 *
 * @brief 			to set the priority of the spi interrupt
 *
 * @param[0]		IRQNumber - interrupt number we want to set priority of
 * @param[1]        IRQPriority - new priority .
 * 
 * @return			nothing.
 *
 * @note
 ************************************************************/
void SPI_IRQPriorityConfig( uint8_t IRQNumber, uint8_t IRQPriority);





/************************************************************
 *
 * @fn				SPI_IRQConfig
 *
 * @brief 			to enable or disable the interrupt of the spi irq.
 *
 * @param[0]		IRQNumber - interrupt number we want to enable interrupt of .
 * @param[1]        EnOrDi - Enable or disable macro.
 * @param[2]        pSPI    - base address of the spi peripheral used
 * @return			nothing.
 *
 * @note
 ************************************************************/
void SPI_IRQConfig(uint8_t IRQNumber  , uint8_t EnorDi, SPI_RegDef_t* pSPI, uint8_t irq_type);



void SPI_IRQHandling(uint8_t pinNumber);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
