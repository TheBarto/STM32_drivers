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
};
