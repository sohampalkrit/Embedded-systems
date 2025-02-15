/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jul 15, 2024
 *      Author: hp
 */
#include<stdint.h>
#include<stdio.h>
#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_
#include "stm32f407xx.h"

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_Sclk_speed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct{
	SPI_RegDef_t      *pSPIx;
	SPI_Config_t      SPIConfig;


}SPI_Handle_t;
/*
 * SPI Device Mode
 */
#define SPI_DEVICE_MODE_MASTER         1
#define SPI_DEVICE_MODE_SLAVE          0

/*
 * @SPI_BUSConfig
 */
#define SPI_BUS_CONFIG_FD              1
#define SPI_BUS_CONFIG_HD              2
#define SPI_BUS_CONFIG_TXONLY          3
#define SPI_BUS_CONFIG_RXONLY          4

/*
 * @SPI_ClkSpeed
 */
#define SPI_CLK_SPEED_DIV2             0
#define SPI_CLK_SPEED_DIV4             1
#define SPI_CLK_SPEED_DIV8             2
#define SPI_CLK_SPEED_DIV16            3
#define SPI_CLK_SPEED_DIV32            4
#define SPI_CLK_SPEED_DIV64            5
#define SPI_CLK_SPEED_DIV128           6
#define SPI_CLK_SPEED_DIV256           7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS                  0
#define SPI_DFF_16BITS                 1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH                 1
#define SPI_CPOL_LOW                 0

/*
 * @SPI_CPOL
 */

#define SPI_CPOH_HIGH                 1
#define SPI_CPOH_LOW                  0

/*
 * @SPI_SSM
 */

#define SPI_SSM_EN	                 1
#define SPI_SSM_DI                   0

#define SPI_TXE_FLAG                 (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG                (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG                (1 << SPI_SR_BSY)


//clock set-up
void SPI_PeriClockControl(SPI_RegDef_t *SPIx,uint32_t EnorD);

//init and dinit
void SPI_Init(SPI_Handle_t * pSPIHandle);
void SPI_DInit(SPI_RegDef_t *pSPIx);

// send and receive data

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);

//IRQ Config and IRQ Handling
void SPI_IRQITConfig(uint8_t IRQNumber,uint8_t EnorD);
void SPI_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t * pHandle);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
