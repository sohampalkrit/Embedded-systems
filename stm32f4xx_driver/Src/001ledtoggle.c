#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include<stdio.h>
#include<stdint.h>
void delay(void){
	for(uint32_t i=0;i<500000;i++);
}
int main(void){
	GPIO_Handle_t gpioled;
	gpioled.pGPIOx=GPIOD;
	gpioled.pGPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO12;
	gpioled.pGPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	gpioled.pGPIO_PinConfig.GPIO_PinSpeed=GPIO_OP_SP_HIGHSP;
	gpioled.pGPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	gpioled.pGPIO_PinConfig.GPIO_PuPdControl=GPIO_PUPD_NONE;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&gpioled);
	while(1){
		GPIO_ToggleOutPutPin(GPIOD,GPIO_PIN_NO12);
		delay();
	}
	return 0;
}
