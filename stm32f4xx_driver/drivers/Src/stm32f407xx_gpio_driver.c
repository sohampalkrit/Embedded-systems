/*
 * stm32f407_gpio.c
 *
 *  Created on: Jun 23, 2024
 *      Author: hp
 */


#include"stm32f407xx_gpio_driver.h"
#include<stdint.h>

//clock set-up

void GPIO_PeriClockControl(GPIO_regDef_t *pGPIOx,uint32_t EnorD){
	if(EnorD==ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx==GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx==GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx==GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx==GPIOI){
			GPIOI_PCLK_EN();
		}

	}else{
		        if(pGPIOx==GPIOA){
					GPIOA_PCLK_DI();
				}else if(pGPIOx==GPIOB){
					GPIOB_PCLK_DI();
				}else if(pGPIOx==GPIOC){
					GPIOC_PCLK_DI();
				}else if(pGPIOx==GPIOD){
					GPIOD_PCLK_DI();
				}else if(pGPIOx==GPIOE){
					GPIOE_PCLK_DI();
				}else if(pGPIOx==GPIOF){
					GPIOF_PCLK_DI();
				}else if(pGPIOx==GPIOG){
					GPIOG_PCLK_DI();
				}else if(pGPIOx==GPIOH){
					GPIOH_PCLK_DI();
				}else if(pGPIOx==GPIOI){
					GPIOI_PCLK_DI();
				}
		//
	}

}

//init and dinit

void GPIO_Init(GPIO_Handle_t * pGPIOHandle){
	//1.configure the pin of GPIO mode
	uint32_t temp=0;
	if(pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp=pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->MODER &=~(0x03 << pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |=temp;

	}else{
		//IT controlled registers
		if(pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){
			//configure the FTSR
			EXTI->FTSR |=(1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
			//clear the RTSR bit
			EXTI->RTSR &=~(1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){
			//configure the RTSR=
			EXTI->RTSR |=(1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
			//clear the FTSR bit
			EXTI->FTSR &=~(1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT){
			//configure both FTSR and RTSR
			EXTI->RTSR |=(1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |=(1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);

		}
		//configure SYSCFG_EXTICR
		uint8_t temp1=(pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber/4);
		uint8_t temp2=(pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber%	4);
		uint8_t portcode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXRICR[temp1]=portcode << (temp2*4);



		//enable exti using IMR
		EXTI->EMR |=(1<< pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);

	}
	temp=0;
	//2. configure the speed
	temp=pGPIOHandle->pGPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OSPEEDR &=~(0x03 <<pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |=temp;

	temp=0;
	//3.configure the pupd registre
	temp=pGPIOHandle->pGPIO_PinConfig.GPIO_PuPdControl<<(2 * pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->PUPDR &=~(0x03 <<pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |=temp;
	temp=0;

	//4.configure the optype
	temp=pGPIOHandle->pGPIO_PinConfig.GPIO_PinOPType<<(pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &=~(0x01<<pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |=temp;
	temp=0;


	//5. configure the alt function mode
	if(pGPIOHandle->pGPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFUN){
		uint8_t temp1,temp2;
		temp1=pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber /8;
		temp2=pGPIOHandle->pGPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &=~(0x0F <<(4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |=(pGPIOHandle->pGPIO_PinConfig.GPIO_PinAltFunMode <<(4*temp2));
	}else{

	}
	temp=0;


}
void GPIO_DInit(GPIO_regDef_t *pGPIOx){
    if(pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx==GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx==GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx==GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx==GPIOI){
		GPIOI_REG_RESET();
	}

}

//data read and write

uint8_t GPIO_ReadFromInputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}
uint16_t GPIO_ReadFromInputPort(GPIO_regDef_t *pGPIOx){
	uint16_t value;
	value=(uint16_t)(pGPIOx->IDR);
	return value;

}
void GPIO_ReadToOutputPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber,uint8_t value){
	if(value==GPIO_PIN_SET){
		//write 1 to the baseaddr
		pGPIOx->ODR |=(1<<PinNumber);


	}else{
		//write 0
		pGPIOx->ODR &=~(1<<PinNumber);
	}

}
void GPIO_ReadToOutputPort(GPIO_regDef_t *pGPIOx,uint16_t Value){

	pGPIOx->IDR=Value;
}
void GPIO_ToggleOutPutPin(GPIO_regDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^=(1<<PinNumber);

}

//IRQ configuration and handling
void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnorD){
	if(EnorD==ENABLE){
		if(IRQNumber <= 31){
			//enable ISER0 register
			*NVIC_ISER0 |=(1<<IRQNumber);

		}else if(IRQNumber>31 && IRQNumber<=64){
			//enable ISER1 register
			*NVIC_ISER1 |=(1<<(IRQNumber%32));
		}else if(IRQNumber>64 && IRQNumber<=96){
			//enable ISER2 register
			*NVIC_ISER2 |=(1<<(IRQNumber%64));

		}
	}else{
		        if(IRQNumber <= 31){
					//enable ICER0 register
		        	*NVIC_ICER0 |=(1<<(IRQNumber));

				}else if(IRQNumber>31 && IRQNumber<=64){
					//enable ICER1 register
					*NVIC_ICER1 |=(1<<(IRQNumber%32));

				}else if(IRQNumber>64 && IRQNumber<=96){
					//enable ICER2 register
					*NVIC_ICER2 |=(1<<(IRQNumber%64));

				}
	}

}
void GPIO_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority){
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section =IRQNumber%4;
	uint8_t Shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + (iprx)) |= (IRQPriority<<Shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR &(1<<PinNumber)){
		EXTI->PR |=(1<<PinNumber);
	}

}
