/*
 * stm32f4xx.h
 *
 *  Created on: Jun 1, 2024
 *      Author: hp
 */
#include<stdint.h>
#include<stdio.h>

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#define __vo volatile


/************************************PROCESSOR SPECIFIC HEADER FILE **************************************************/
/*
 * ARM CORTEX M4 Processor
 */
// NVIC ISER Registers
#define NVIC_ISER0           ((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1           ((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2           ((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3           ((__vo uint32_t *)0xE000E10C)

// NVIC ICER Registers
#define NVIC_ICER0           ((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1           ((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2           ((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3           ((__vo uint32_t *)0xE000E18C)
#define NVIC_PR_BASEADDR     ((__vo uint32_t *)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED   4

//defining memory base addresses
#define FLASH_BASEADDR       0x80000000U
#define SRAM1_BASEADDR       0x20000000U
#define SRAM2_BASEADDR       0x20001C00U
#define ROM_BASEADDR         0x1FFF0000U
#define SRAM                 SRAM1_BASEADDR

//defining base addresses of different bus domains

#define PERIPH_BASEADDR          0x40000000U
#define APB1PERIPH_BASEADDR      PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR      0x40010000U
#define AHB1PERIPH_BASEADDR      0x40020000U
#define AHB2PERIPH_BASEADDR      0x50000000U

//Peripherals on AHB1 bus

#define GPIOA_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR       (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR         (AHB1PERIPH_BASEADDR + 0x3800)

//defining base addresses of peripherals on APB1 bus

#define I2C1_BASEADDR        (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR        (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR        (APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR        (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR        (APB1PERIPH_BASEADDR + 0x3C00)
#define USRAT2_BASEADDR      (APB1PERIPH_BASEADDR + 0x4400)
#define USRAT3_BASEADDR      (APB1PERIPH_BASEADDR + 0x4800)
#define URAT4_BASEADDR      (APB1PERIPH_BASEADDR + 0x4C00)
#define URAT5_BASEADDR      (APB1PERIPH_BASEADDR + 0x5000)

//defining base addresses of peripherals on APB2 bus

#define SPI1_BASEADDR        (APB2PERIPH_BASEADDR + 0x13000)
#define USART1_BASEADDR      (APB2PERIPH_BASEADDR + 0x11000)
#define USART6_BASEADDR      (APB2PERIPH_BASEADDR + 0x14000)
#define EXTI_BASEADDR        (APB2PERIPH_BASEADDR + 0x13C00)
#define SYSCFG_BASEADDR      0x40013800U

#define GPIO_BASEADDR_TO_CODE(x)  ( (x==GPIOA) ? 0 :\
		                           (x==GPIOB) ? 1 :\
		                           (x==GPIOC) ? 2 :\
		                           (x==GPIOD) ? 3 :\
		                           (x==GPIOE) ? 4 :\
		                           (x==GPIOF) ? 5 :\
		                           (x==GPIOG) ? 6 :\
		                           (x==GPIOH) ? 7 :\
		                           (x==GPIOI) ? 8 :0)
#define EXTI0               6
#define EXTI1               7
#define EXTI2               8
#define EXtI3               9
#define EXTI4               10
#define EXTI9_5             23
#define EXTI15_10           40


//structuring peripheral registers

typedef struct {
	__vo uint32_t MODER;  //GPIO port mode register
	__vo uint32_t OTYPER; //GPIO port output type register
	__vo uint32_t OSPEEDR;//GPIO port output speed register
	__vo uint32_t PUPDR;  //GPIO port pull-up/pull-down register
	__vo uint32_t IDR;    //GPIO port input data register
	__vo uint32_t ODR;    //GPIO port output data register
	__vo uint32_t BSSR;   //GPIO port bit set/reset register
	__vo uint32_t LCKR;   //GPIO port configuration lock register
	__vo uint32_t AFR[2]; //

}GPIO_regDef_t;


//RCC peripheral registers

typedef struct{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_PLLCFGR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_ABH1RSTR;
	__vo uint32_t RCC_ABH2RSTR;
	__vo uint32_t RCC_ABH3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t RCC_APB1RSTR;
	__vo uint32_t RCC_APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t RCC_AHB1ENR;
	__vo uint32_t RCC_AHB2ENR;
	__vo uint32_t RCC_AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t RCC_AHB1LPENR;
	__vo uint32_t RCC_AHB2LPENR;
	__vo uint32_t RCC_AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t RCC_APB1LPENR;
	__vo uint32_t RCC_APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t RCC_SSCGR;
    __vo uint32_t RCC_PLLI2SCFGR;
	__vo uint32_t RCC_PLLSAICFGR;
	__vo uint32_t RCC_DCKCFGR;
	__vo uint32_t RCC_CKGATENR;
	__vo uint32_t RCC_DCKCFGR2;

}RCC_RegDef_t;
//structure for EXTI interupt control
typedef struct{
	__vo uint32_t IMR ;
	__vo uint32_t EMR ;
	__vo uint32_t RTSR ;
	__vo uint32_t FTSR ;
	__vo uint32_t SWIER ;
	__vo uint32_t PR ;

}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRPM ;
	__vo uint32_t PMC ;
	__vo uint32_t EXRICR[4] ;
	uint32_t Reserved1[2];
	__vo uint32_t CMPCR ;
	uint32_t Reserved2[2];
	__vo uint32_t CFGR;


}SYSCFG_RegDef_t;

//SPI structure defination
typedef struct{
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;

}SPI_RegDef_t;

#define SPI1     ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2     ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3     ((SPI_RegDef_t*)SPI3_BASEADDR)

#define GPIOA            ((GPIO_regDef_t*)GPIOA_BASEADDR)
#define GPIOB            ((GPIO_regDef_t*)GPIOB_BASEADDR)
#define GPIOC            ((GPIO_regDef_t*)GPIOC_BASEADDR)
#define GPIOD            ((GPIO_regDef_t*)GPIOD_BASEADDR)
#define GPIOE            ((GPIO_regDef_t*)GPIOE_BASEADDR)
#define GPIOF            ((GPIO_regDef_t*)GPIOF_BASEADDR)
#define GPIOG            ((GPIO_regDef_t*)GPIOG_BASEADDR)
#define GPIOH            ((GPIO_regDef_t*)GPIOH_BASEADDR)
#define GPIOI            ((GPIO_regDef_t*)GPIOI_BASEADDR)

#define RCC              ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI             ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG           ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

//some clock enable registers

//GPIOs clock enable macros AHB1 bus

#define GPIOA_PCLK_EN()   (RCC->RCC_AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()   (RCC->RCC_AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()   (RCC->RCC_AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()   (RCC->RCC_AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()   (RCC->RCC_AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()   (RCC->RCC_AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()   (RCC->RCC_AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()   (RCC->RCC_AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN()   (RCC->RCC_AHB1ENR |=(1<<8))

//I2C clock enable macros APB1

#define I2C1_PCLK_EN()     (RCC->RCC_APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()     (RCC->RCC_APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()     (RCC->RCC_APB1ENR |=(1<<23))

//SPIx clock enable macros
#define SPI1_PCLK_EN()     (RCC->RCC_APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()     (RCC->RCC_APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()     (RCC->RCC_APB1ENR |=(1<<15))

//clock enable USART and UART

#define USART1_PCLK_EN()   (RCC->RCC_APB2ENR |=(1<<4))
#define USART2_PCLK_EN()   (RCC->RCC_APB1ENR |=(1<<17))
#define USART3_PCLK_EN()   (RCC->RCC_APB1ENR |=(1<<18))
#define USART4_PCLK_EN()   (RCC->RCC_APB1ENR |=(1<<19))
#define USART5_PCLK_EN()   (RCC->RCC_APB1ENR |=(1<<20))
#define USART6_PCLK_EN()   (RCC->RCC_APB2ENR |=(1<<5))

//clock enable for SYSCFG peripheral
#define SYSCFG_PCLK_EN()   (RCC->RCC_APB2ENR |=(1<<14))

//disable macros for all peripherals

#define GPIOA_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<7))
#define GPIOI_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<8))

//I2C clock disable macros APB1

#define I2C1_PCLK_DI()     (RCC->RCC_APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI()     (RCC->RCC_APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI()     (RCC->RCC_APB1ENR &=~(1<<23))

//SPIx clock disable macros
#define SPI1_PCLK_DI()     (RCC->RCC_APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI()     (RCC->RCC_APB1ENR &=~(1<<14))
#define SPI3_PCLK_DI()     (RCC->RCC_APB1ENR &=~(1<<15))

//RCC Reset clock enable
#define GPIOA_REG_RESET()   do{(RCC->RCC_ABH1RSTR |= (1<<0)); (RCC->RCC_ABH1RSTR &=~(1<<0));}while(0)
#define GPIOB_REG_RESET()   do{(RCC->RCC_ABH1RSTR |= (1<<1)); (RCC->RCC_ABH1RSTR &=~(1<<1));}while(0)
#define GPIOC_REG_RESET()   do{(RCC->RCC_ABH1RSTR |= (1<<2)); (RCC->RCC_ABH1RSTR &=~(1<<2));}while(0)
#define GPIOD_REG_RESET()   do{(RCC->RCC_ABH1RSTR |= (1<<3)); (RCC->RCC_ABH1RSTR &=~(1<<3));}while(0)
#define GPIOE_REG_RESET()   do{(RCC->RCC_ABH1RSTR |= (1<<4)); (RCC->RCC_ABH1RSTR &=~(1<<4));}while(0)
#define GPIOF_REG_RESET()   do{(RCC->RCC_ABH1RSTR |= (1<<5)); (RCC->RCC_ABH1RSTR &=~(1<<5));}while(0)
#define GPIOG_REG_RESET()   do{(RCC->RCC_ABH1RSTR |= (1<<6)); (RCC->RCC_ABH1RSTR &=~(1<<6));}while(0)
#define GPIOH_REG_RESET()   do{(RCC->RCC_ABH1RSTR |= (1<<7)); (RCC->RCC_ABH1RSTR &=~(1<<7));}while(0)
#define GPIOI_REG_RESET()   do{(RCC->RCC_ABH1RSTR |= (1<<8)); (RCC->RCC_ABH1RSTR &=~(1<<8));}while(0)

// RCC Reset clock enable for SPI peripherals on STM32F407
#define SPI1_REG_RESET()   do{(RCC->RCC_APB2RSTR |= (1 << 12)); (RCC->RCC_APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()   do{(RCC->RCC_APB1RSTR |= (1 << 14)); (RCC->RCC_APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()   do{(RCC->RCC_APB1RSTR |= (1 << 15)); (RCC->RCC_APB1RSTR &= ~(1 << 15));}while(0)


//clock disable USART and UART

#define USART1_PCLK_DI()   (RCC->RCC_APB2ENR &=~(1<<4))
#define USART2_PCLK_DI()   (RCC->RCC_APB1ENR &=~(1<<17))
#define USART3_PCLK_DI()   (RCC->RCC_APB1ENR &=~(1<<18))
#define USART4_PCLK_DI()   (RCC->RCC_APB1ENR &=~(1<<19))
#define USART5_PCLK_DI()   (RCC->RCC_APB1ENR &=~(1<<20))
#define USART6_PCLK_DI()   (RCC->RCC_APB2ENR &=~(1<<5))

//clock disable for SYSCFG peripheral
#define SYSCFG_PCLK_DI()   (RCC->RCC_APB2ENR &=~(1<<14))

//some generic macros
#define ENABLE         1
#define DISABLE        0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESER RESET
#define FLAG_RESET     RESET
#define FLAG_SET       SET
/*
 * Bit fields for SPI
 */
#define SPI_CR1_CPHA   0
#define SPI_CR1_CPOL   1
#define SPI_CR1_MSTR   2
#define SPI_CR1_BR     3
#define SPI_CR1_SPE    6
#define SPI_CR1_LSB1   7
#define SPI_CR1_SSI    8
#define SPI_CR1_SSM    9
#define SPI_CR1_RXONLY 10
#define SPI_CR1_DFF    11
#define SPI_CR1_CRCNXT 12
#define SPI_CR1_CRCEN  13
#define SPI_CR1_BIDIOE 14
#define SPI_CR1_BIDIMO 15

#define SPI_CR2_RXDMAEN 0
#define SPI_CR2_TXDMAEN 1
#define SPI_CR2_SSOE    2
#define SPI_CR2_RES     3
#define SPI_CR2_FRF     4
#define SPI_CR2_ERRIE   5
#define SPI_CR2_RXNEIE  6
#define SPI_CR2_TXEIE   7

#define SPI_SR_RXNE     0
#define SPI_SR_TXE      1
#define SPI_SR_CHSIDE   2
#define SPI_SR_UDR      3
#define SPI_SR_CRCERR   4
#define SPI_SR_MODF     5
#define SPI_SR_OVR      6
#define SPI_SR_BSY      7
#define SPI_SR_FRE      8




#include"stm32f407xx_gpio_driver.h"
#include"stm32f407xx_spi_driver.h"


#endif /* INC_STM32F4XX_H_ */
