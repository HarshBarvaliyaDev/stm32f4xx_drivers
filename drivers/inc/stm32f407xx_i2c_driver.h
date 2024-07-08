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
#define I2C_CR1_PE 0
#define I2C_CR1_NOSTRETCH 7
#define I2C_CR1_START 8
#define I2C_CR1_STOP 9
#define I2C_CR1_ACK 10
#define I2C_CR1_SWRST 15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ 0
#define I2C_CR2_ITERREN 8
#define I2C_CR2_ITEVTEN 9
#define I2C_CR2_ITBUFEN 10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0 0
#define I2C_OAR1_ADD71 1
#define I2C_OAR1_ADD98 8
#define I2C_OAR1_ADDMODE 15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 0
#define I2C_SR1_ADDR 1
#define I2C_SR1_BTF 2
#define I2C_SR1_ADD10 3
#define I2C_SR1_STOPF 4
#define I2C_SR1_RXNE 6
#define I2C_SR1_TXE 7
#define I2C_SR1_BERR 8
#define I2C_SR1_ARLO 9
#define I2C_SR1_AF 10
#define I2C_SR1_OVR 11
#define I2C_SR1_TIMEOUT 14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL 0
#define I2C_SR2_BUSY 1
#define I2C_SR2_TRA 2
#define I2C_SR2_GENCALL 4
#define I2C_SR2_DUALF 7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 0
#define I2C_CCR_DUTY 14
#define I2C_CCR_FS 15

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
  uint8_t I2C_DeviceAddress;
  uint8_t I2C_AckControl;
  uint8_t I2C_FMDutyCycle;

} I2C_Config_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM2K 200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1

typedef struct
{
  I2C_RegDef_t *pI2Cx;
  I2C_Config_t I2C_Config;
  uint8_t *pTxBuffer; /* !< To store the app. Tx buffer address > */
  uint8_t *pRxBuffer; /* !< To store the app. Rx buffer address > */
  uint32_t TxLen;     /* !< To store Tx len > */
  uint32_t RxLen;     /* !< To store Rx len > */
  uint8_t TxRxState;  /* !< To store Communication state > */
  uint8_t DevAddr;    /* !< To store slave/device address > */
  uint32_t RxSize;    /* !< To store Rx size  > */
  uint8_t Sr;         /* !< To store repeated start value  > */
} I2C_Handle_t;

/*
 * I2C application states @TxRxState
 */
#define I2C_READY 0
#define I2C_BUSY_IN_RX 1
#define I2C_BUSY_IN_TX 2

#define READ_I2C_FLAG_TXE(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_TXE)
#define READ_I2C_FLAG_RXNE(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_RXNE)
#define READ_I2C_FLAG_SB(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_SB)
#define READ_I2C_FLAG_OVR(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_OVR)
#define READ_I2C_FLAG_AF(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_AF)
#define READ_I2C_FLAG_ARLO(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_ARLO)
#define READ_I2C_FLAG_BERR(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_BERR)
#define READ_I2C_FLAG_STOPF(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_STOPF)
#define READ_I2C_FLAG_ADD10(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_ADD10)
#define READ_I2C_FLAG_BTF(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_BTF)
#define READ_I2C_FLAG_ADDR(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_ADDR)
#define READ_I2C_FLAG_TIMEOUT(PI2CX) ((PI2CX->SR1) & 1 << I2C_SR1_TIMEOUT)

/*
 * Peripheral Clock setup
 */
/************************************************************
 *
 * @fn				I2C_PeriClockControl
 *
 * @brief 			enable or disable clock for i2c
 *
 * @param[0]		pI2Cx - I2C_RegDef_t structure pointer.
 * @param[1]		EnorDi - ENABLE/DISABLE macros.
 *
 * @return			nothing
 *
 * @note
 ************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
/************************************************************
 *
 * @fn				I2C_Init
 *
 * @brief 			to initialize i2c peripheral.
 *
 * @param[0]		pI2CHandle - I2C_Handle_t structure pointer.
 *
 * @return			nothing
 *
 * @note
 ************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle);

/************************************************************
 *
 * @fn				I2C_DeInit
 *
 * @brief 			to deinitialize i2c peripheral.
 *
 * @param[0]		pI2Cx - I2C_RegDef_t structure pointer.
 *
 * @return			nothing
 *
 * @note
 ************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
/************************************************************
 *
 * @fn				I2C_MasterSendData
 *
 * @brief 			send data , this is a blocking function
 *
 * @param[0]		pI2CHandle - I2C_Handle_t structure pointer.
 * @param[1]		pTxbuffer - starting address of the data to be sent.
 * @param[2]		Len - length of the data in byte.
 * @param[3]		SlaveAddr - slave device address at which we want to send data..
 * @param[4]		Sr - ---
 *
 * @return			nothing
 *
 * @note
 ************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/************************************************************
 *
 * @fn				I2C_MasterReceiveData
 *
 * @brief 			receive data , this is blocking function.
 *
 * @param[0]		pI2CHandle - I2C_Handle_t structure pointer.
 * @param[1]		pRxBuffer - starting address of the data to be receive.
 * @param[2]		Len - length of the data in byte.
 * @param[3]		SlaveAddr - slave device address of which we want to read..
 * @param[4]		Sr - ---
 *
 * @return			nothing
 *
 * @note
 ************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/************************************************************
 *
 * @fn				I2C_MasterSendData
 *
 * @brief 			send data , this is a non blocking function
 *
 * @param[0]		pI2CHandle - I2C_Handle_t structure pointer.
 * @param[1]		pTxbuffer - starting address of the data to be sent.
 * @param[2]		Len - length of the data in byte.
 * @param[3]		SlaveAddr - slave device address at which we want to send data..
 * @param[4]		Sr - ---
 *
 * @return			nothing
 *
 * @note
 ************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/************************************************************
 *
 * @fn				I2C_MasterReceiveData
 *
 * @brief 			receive data , this is non blocking function.
 *
 * @param[0]		pI2CHandle - I2C_Handle_t structure pointer.
 * @param[1]		pRxBuffer - starting address of the data to be receive.
 * @param[2]		Len - length of the data in byte.
 * @param[3]		SlaveAddr - slave device address of which we want to read..
 * @param[4]		Sr - ---
 *
 * @return			nothing
 *
 * @note
 ************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/************************************************************
 *
 * @fn				I2C_CloseReceiveData
 *
 * @brief 			close receiving the data.
 *
 * @param[0]		pI2CHandle - I2C_Handle_t structure pointer.
 *
 * @return			nothing
 *
 * @note
 ************************************************************/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

/************************************************************
 *
 * @fn				I2C_CloseSendData
 *
 * @brief 			close sending the data.
 *
 * @param[0]		pI2CHandle - I2C_Handle_t structure pointer.
 *
 * @return			nothing
 *
 * @note
 ************************************************************/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F407_I2C_DRIVER_H_ */
