/*
 * stm32f407_i2c_driver.h
 *
 *  Created on: Jun 18, 2024
 *      Author: harsh
 */
#include <stdint.h>
#include <stm32f407xx.h>

#ifndef INC_STM32F407_I2C_DRIVER_H_
#define INC_STM32F407_I2C_DRIVER_H_



/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  		7
#define I2C_CR1_START 				8
#define I2C_CR1_STOP  				9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	  0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			  10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 				3
#define I2C_SR1_STOPF 				4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 			14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 			4
#define I2C_SR2_DUALF 				7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


typedef struct
{
  __IO uint32_t CR1;   /*!< I2C Control register 1,     Address offset: 0x00 */
  __IO uint32_t CR2;   /*!< I2C Control register 2,     Address offset: 0x04 */
  __IO uint32_t OAR1;  /*!< I2C Own address register 1, Address offset: 0x08 */
  __IO uint32_t OAR2;  /*!< I2C Own address register 2, Address offset: 0x0C */
  __IO uint32_t DR;    /*!< I2C Data register,          Address offset: 0x10 */
  __IO uint32_t SR1;   /*!< I2C Status register 1,      Address offset: 0x14 */
  __IO uint32_t SR2;   /*!< I2C Status register 2,      Address offset: 0x18 */
  __IO uint32_t CCR;   /*!< I2C Clock control register, Address offset: 0x1C */
  __IO uint32_t TRISE; /*!< I2C TRISE register,         Address offset: 0x20 */
} I2C_RegDef_t;

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

}I2C_Config_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	  100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000


/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE        1
#define I2C_ACK_DISABLE       0


/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2        0
#define I2C_FM_DUTY_16_9     1


typedef struct
{
  I2C_RegDef_t    *pI2Cx;
  I2C_Config_t    I2C_Config;
  uint8_t         *pTxBuffer; /* !< To store the app. Tx buffer address > */
  uint8_t         *pRxBuffer; /* !< To store the app. Rx buffer address > */
  uint32_t        TxLen;     /* !< To store Tx len > */
  uint32_t        RxLen;     /* !< To store Rx len > */
  uint8_t         TxRxState;  /* !< To store Communication state > */
  uint8_t         DevAddr;    /* !< To store slave/device address > */
  uint32_t        RxSize;    /* !< To store Rx size  > */
  uint8_t         Sr;         /* !< To store repeated start value  > */
} I2C_Handle_t;


/*
 * I2C application states @TxRxState
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2


#define READ_I2C_FLAG_TXE(PI2CX)   		( (PI2CX-> SR1) & 1 << I2C_SR1_TXE)
#define READ_I2C_FLAG_RXNE(PI2CX)   	( (PI2CX-> SR1) & 1 << I2C_SR1_RXNE)
#define READ_I2C_FLAG_SB(PI2CX)			  ( (PI2CX-> SR1) & 1 << I2C_SR1_SB)
#define READ_I2C_FLAG_OVR(PI2CX)  		( (PI2CX-> SR1) & 1 << I2C_SR1_OVR)
#define READ_I2C_FLAG_AF(PI2CX)   		( (PI2CX-> SR1) & 1 << I2C_SR1_AF)
#define READ_I2C_FLAG_ARLO(PI2CX) 		( (PI2CX-> SR1) & 1 << I2C_SR1_ARLO)
#define READ_I2C_FLAG_BERR(PI2CX) 		( (PI2CX-> SR1) & 1 << I2C_SR1_BERR)
#define READ_I2C_FLAG_STOPF(PI2CX) 		( (PI2CX-> SR1) & 1 << I2C_SR1_STOPF)
#define READ_I2C_FLAG_ADD10(PI2CX) 		( (PI2CX-> SR1) & 1 << I2C_SR1_ADD10)
#define READ_I2C_FLAG_BTF(PI2CX)  		( (PI2CX-> SR1) & 1 << I2C_SR1_BTF)
#define READ_I2C_FLAG_ADDR(PI2CX) 		( (PI2CX-> SR1) & 1 << I2C_SR1_ADDR)
#define READ_I2C_FLAG_TIMEOUT(PI2CX) 	( (PI2CX-> SR1) & 1 << I2C_SR1_TIMEOUT)


#endif /* INC_STM32F407_I2C_DRIVER_H_ */
