#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include<stdio.h>
#include<stdint.h>
#include<string.h>
void delay(void){
	for(uint32_t i=0;i<500000;i++);
}
int main(void){
	GPIO_Handle_t gpioled , GPIOBtn;
	memset(&gpioled,0,sizeof(gpioled));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));
	gpioled.pGPIOx=GPIOD;
	gpioled.pGPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO12;
	gpioled.pGPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	gpioled.pGPIO_PinConfig.GPIO_PinSpeed=GPIO_OP_SP_HIGHSP;
	gpioled.pGPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	gpioled.pGPIO_PinConfig.GPIO_PuPdControl=GPIO_PUPD_NONE;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&gpioled);

	GPIOBtn.pGPIOx=GPIOD;
	GPIOBtn.pGPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO5;
	GPIOBtn.pGPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	GPIOBtn.pGPIO_PinConfig.GPIO_PinSpeed=GPIO_OP_SP_HIGHSP;
	GPIOBtn.pGPIO_PinConfig.GPIO_PuPdControl=GPIO_PUPD_PULLUP;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GPIOBtn);

	GPIO_IRQPriority(EXTI9_5,15);
	GPIO_IRQITConfig(EXTI9_5,ENABLE);


	return 0;
}
void EXTI9_5_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO5);
	GPIO_ToggleOutPutPin(GPIOD,GPIO_PIN_NO12);
}
