/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Jun 23, 2024
 *      Author: hp
 */
#include<stdint.h>
#include<stdio.h>
#ifndef INC_STM32F407_GPIO_DRIVER_C_
#define INC_STM32F407_GPIO_DRIVER_C_
#include "stm32f407xx.h"
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct{
	GPIO_regDef_t *pGPIOx;
	GPIO_PinConfig_t pGPIO_PinConfig;

}GPIO_Handle_t;

//GPIO PIN NUMBERS
#define GPIO_PIN_NO0   0
#define GPIO_PIN_NO1   1
#define GPIO_PIN_NO2   2
#define GPIO_PIN_NO3   3
#define GPIO_PIN_NO4   4
#define GPIO_PIN_NO5   5
#define GPIO_PIN_NO6   6
#define GPIO_PIN_NO7   7
#define GPIO_PIN_NO8   8
#define GPIO_PIN_NO9   9
#define GPIO_PIN_NO10  10
#define GPIO_PIN_NO11  11
#define GPIO_PIN_NO12  12
#define GPIO_PIN_NO13  13
#define GPIO_PIN_NO14  14
#define GPIO_PIN_NO15  15


//PORT MODE REGISTE
#define GPIO_MODE_IN     0
#define GPIO_MODE_OUT    1
#define GPIO_MODE_ALTFUN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4
#define GPIO_MODE_IT_RT  5
#define GPIO_MODE_IT_RFT 6
//OUTPUT TYPE REGISTER
#define GPIO_OP_TYPE_PP  0
#define GPIO_OP_TYPE_OD  1

//OUTPUT SPPED REGISTER
#define GPIO_OP_SP_LOWSP    0
#define GPIO_OP_SP_MIDSP    1
#define GPIO_OP_SP_HIGHSP   2
#define GPIO_OP_SP_VHIGHSP  3

//PULL UP PULL DOWN REGISTER
#define GPIO_PUPD_NONE      0
#define GPIO_PUPD_PULLUP    1
#define GPIO_PUPD_PULLD     2
#define GPIO_PUPD_RES       3

//PORT INPUT MODE REGISTER
#define GPIO_IN


//clock set-up
void GPIO_PeriClockControl(GPIO_regDef_t *pGPIOx,uint32_t EnorD);

//init and dinit
void GPIO_Init(GPIO_Handle_t * pGPIOHandle);
void GPIO_DInit(GPIO_regDef_t *pGPIOx);

//data read and write

uint8_t GPIO_ReadFromInputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_regDef_t *pGPIOx);
void GPIO_ReadToOutputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_ReadToOutputPort(GPIO_regDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutPutPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber);

//IRQ configuration and handling
void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnorD);
void GPIO_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407_GPIO_DRIVER_C_ */
