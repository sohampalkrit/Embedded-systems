/*
 * main.h
 *
 *  Created on: May 11, 2024
 *      Author: hp
 */

#ifndef MAIN_H_
#define MAIN_H_
#include<stdint.h>

typedef struct{
	uint32_t gpioaen:1;
	uint32_t gpioben:1;
	uint32_t gpiocen:1;
	uint32_t gpioden:1;
	uint32_t gpioeen:1;
	uint32_t gpiofen:1;
	uint32_t gpiogen:1;
	uint32_t gpiohen:1;
	uint32_t gpioien:1;
	uint32_t reserved1:3;
	uint32_t crcen  :1;
	uint32_t reserved2:5;
	uint32_t bksramen:1;
	uint32_t resvered3:1;
	uint32_t ccmdataramen:1;
	uint32_t dma1en:1;
	uint32_t dma2en:1;
	uint32_t resvered4:2;
	uint32_t ethmacen:1;
	uint32_t ethmactxen:1;
	uint32_t ethmacrxen:1;
	uint32_t ethmacptpen:1;
	uint32_t otghsen:1;
	uint32_t otghsulpien:1;
	uint32_t resvered5:1;




}RCC_AHB1ENR_t;
typedef struct{
	uint32_t moder0 :2;
	uint32_t moder1 :2;
	uint32_t moder2 :2;
	uint32_t moder3 :2;
	uint32_t moder4 :2;
	uint32_t moder5 :2;
	uint32_t moder6 :2;
	uint32_t moder7 :2;
	uint32_t moder8 :2;
	uint32_t moder9 :2;
	uint32_t moder10:2;
	uint32_t moder11:2;
	uint32_t moder12:2;
	uint32_t moder13:2;
	uint32_t moder14:2;
	uint32_t moder15:2;


}GPIOx_MODE_t;

typedef struct{
	    uint32_t ot0 :1;
		uint32_t ot1 :1;
		uint32_t ot2 :1;
		uint32_t ot3 :1;
		uint32_t ot4 :1;
		uint32_t ot5 :1;
		uint32_t ot6 :1;
		uint32_t ot7 :1;
		uint32_t ot8 :1;
		uint32_t ot9 :1;
		uint32_t ot10:1;
		uint32_t ot11:1;
		uint32_t ot12:1;
		uint32_t ot13:1;
		uint32_t ot14:1;
		uint32_t ot15:1;
		uint32_t reserved:16;

}GPIOx_ORD_t;



#endif /* MAIN_H_ */
