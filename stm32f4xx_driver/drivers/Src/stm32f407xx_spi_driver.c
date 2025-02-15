/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 15, 2024
 *      Author: hp
 */

#include"stm32f407xx_spi_driver.h"
#include<stdint.h>
uint8_t GET_FLAG_STATUS(SPI_RegDef_t *pSPIx,uint32_t FlagName);
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint32_t EnorD){
	if(EnorD==ENABLE){
			if(pSPIx==SPI1){
				SPI1_PCLK_EN();
			}else if(pSPIx==SPI2){
				SPI2_PCLK_EN();
			}else if(pSPIx==SPI3){
				SPI3_PCLK_EN();
			}

		}else{
			        if(pSPIx==SPI1){
						SPI1_PCLK_DI();
					}else if(pSPIx==SPI2){
						SPI2_PCLK_DI();
					}else if(pSPIx==SPI3){
						SPI3_PCLK_DI();
					}
		}


}

void SPI_Init(SPI_Handle_t * pSPIHandle){
	//SPI CR1 register
	uint32_t tempreg=0;
	//config device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode <<SPI_CR1_MSTR;
	//config Bus
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD){
		//BIDI cleared
		tempreg &=~(1<<SPI_CR1_BIDIMO);
	}
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD){
			//BIDI set
			tempreg |=(1<<SPI_CR1_BIDIMO);
		}
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_TXONLY){
			//BIDI cleared
			tempreg &~(1<<SPI_CR1_BIDIMO);
			//TXONLY set
			tempreg|=(1<<SPI_CR1_RXONLY);
		}
	//config SPI_Sclk_speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_Sclk_speed<<SPI_CR1_BR);
	//config DFF
	tempreg |=(pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF);
	//config CPOl
	tempreg |=(pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL);
	//config CPHA
	tempreg |=(pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA);

	pSPIHandle->pSPIx->SPI_CR1=tempreg;


}
void SPI_DInit(SPI_RegDef_t *pSPIx){
    if(pSPIx==SPI1){
		SPI1_REG_RESET();
	}else if(pSPIx==SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx==SPI3){
		SPI3_REG_RESET();
	}

}

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len){

	//checking for the length to be zero if not then send data and reduce length to zero
    //basically wait for txe to set
	while(Len>0){

		while(GET_FLAG_STATUS(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);
	}
	//DFF data frame for SPISend to be set (16 bits) or reset(8 bits)
	if(pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF)){
		//16 bits
		pSPIx->SPI_DR = *((uint16_t)pTxBuffer);
		Len--;
		Len--;
		(uint16_t)pTxBuffer++;
	}else{
		pSPIx->SPI_DR = *(pTxBuffer);
		Len--;
		pTxBuffer++;
	}


}
uint8_t GET_FLAG_STATUS(SPI_RegDef_t *pSPIx,uint32_t FlagName){
	if(pSPIx->SPI_SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;

}


