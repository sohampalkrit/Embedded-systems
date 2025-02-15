#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include<stdio.h>
#include<stdint.h>
void delay(void){
	for(uint32_t i=0;i<500000;i++);
}
int main(void){
	GPIO_Handle_t gpioled , GPIOBtn;
	gpioled.pGPIOx=GPIOD;
	gpioled.pGPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO12;
	gpioled.pGPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	gpioled.pGPIO_PinConfig.GPIO_PinSpeed=GPIO_OP_SP_HIGHSP;
	gpioled.pGPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	gpioled.pGPIO_PinConfig.GPIO_PuPdControl=GPIO_PUPD_NONE;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&gpioled);

	GPIOBtn.pGPIOx=GPIOA;
	GPIOBtn.pGPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO0;
	GPIOBtn.pGPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOBtn.pGPIO_PinConfig.GPIO_PinSpeed=GPIO_OP_SP_HIGHSP;
	GPIOBtn.pGPIO_PinConfig.GPIO_PuPdControl=GPIO_PUPD_NONE;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GPIOBtn);
	while(1){
	if((GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO0))==1){

			GPIO_ToggleOutPutPin(GPIOD,GPIO_PIN_NO12);

		}
	}
	return 0;
}
