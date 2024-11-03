/*
 * stm32f407xx_spi_controller.c
 *
 *  Created on: Oct 27, 2024
 *      Author: Usuario
 */

#include "stm32f407xx_spi_driver.h"

typedef struct{
	SPI_RegDef_t spi_driver;
	uint8_t data_send[MAX_DATA_SEND];
	uint8_t total_data_send;
	uint8_t data_recv[MAX_DATA_RECV];
	uint8_t total_data_recv;
}SPI_Controller;

SPI_Controller SPI_Controller[TOTAL_STM32F407_SPI];

typedef enum {
	SPI_controller_st_idle = 0,
	SPI_controller_st_send,
	SPI_controller_st_send_w,
	SPI_controller_st_recv,
	SPI_controller_st_recv_w,
	SPI_controller_st_recv_ACK_NACK,
} SPI_Controller_States;

void SPI_Controller_load_data_send(uint8_t* data, uint8_t total_data)
{
	memcpy(&SPI_Controller[0].data_send[0], &data[0], total_data);
	SPI_Controller[0].total_data_send = total_data;
	SPI_Controller[0].state = SPI_controller_st_send;
}

void SPI_Controller_tick()
{

	switch(SPI_Controller[0].state) {
	case SPI_controller_st_send:
		if()
		SPI_Send_Data(0, SPI_Controller[0].data_send, SPI_Controller[0].total_data_send);
		break;


	}


}
