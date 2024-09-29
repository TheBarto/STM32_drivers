#include "stm32f407xx_spi_driver.h"

#define TOTAL_STM32F407_SPI (uint8_t) 6
static SPI_RegDef_t* SPIs[TOTAL_STM32F407_SPI];

void SPI_initialization_module()
{
	SPIs[0] = SPI1_BASEADDR;
	SPIs[1] = SPI2_BASEADDR;
	SPIs[2] = SPI3_BASEADDR;
	SPIs[3] = SPI4_BASEADDR;
	SPIs[4] = SPI5_BASEADDR;
	SPIs[5] = SPI6_BASEADDR;
}

void SPI_Initialization(uint8_t SPI_peripheral, uint8_t communication_mode,
                        uint8_t mode, uint8_t CPOL, uint8_t CPHA,
                        uint8_t DFF, uint8_t BR_prescaler, uint8_t SSM,
                        uint8_t SSI, uint8_t SSOE)
{
	/* This way we get the peripheral clean */
	SPI_Reset(SPI_peripheral);

	uint16_t aux_setter = 0;
	aux_setter |= (CPHA);
	aux_setter |= (CPOL << 1);
	aux_setter |= (mode << 2);
	aux_setter |= (BR_prescaler << 3);
	aux_setter |= (1 << 6); //SPE bit. To enable the peripheral
	//aux_setter |= (SSI << 8);
	aux_setter |= (SSM << 9);
	aux_setter |= (SSM << 11);

	switch(communication_mode)
	{
	case SPI_HALF_DUPLEX_MODE:
		aux_setter |= (1 << 15);
		/* BIDIOE bit must be used to deceide which send and receive */
		break;
	case SPI_SIMPLEX_CONECTION_MODE:
		/* With this we just disable the output. We're just receiving data. */
		aux_setter |= (1 << 10);
		break;
	}

	SPIs[SPI_peripheral]->CR1 = aux_setter;

	/* We're just assuming that it wont be multimaster capability */
	/*if(mode == SPI_MODE_MASTER && SSM == SPI_SSM_DISABLE)
		SPIs[SPI_peripheral]->CR2 |= (SPI_SSOE_ENABLE << 2);*/
	return;
}


void SPI_Reset(uint8_t SPI_peripheral)
{
	uint8_t pos = 0;
	switch(SPI_peripheral)
	{
	case SPI1:
		RCC->APB2RSTR |= (1 << 12);
		RCC->APB2RSTR &= ~(1 << 12);
		return;
		break;
	case SPI2:
		pos = 14;
		break;
	case SPI3:
		pos = 15;
		break;
	}
	RCC->APB1RSTR |= (1 << pos);
	RCC->APB1RSTR &= ~(1 << pos);
	return;
}

void SPI_clock_enable_disabled(uint8_t SPI_peripheral, bool enable_disable)
{
	uint8_t pos = 0;
	switch(SPI_peripheral)
	{
	case SPI1:
		(enable_disable) ? (RCC->APB2ENR |= (1 << 12)) : (RCC->APB2ENR &= ~(1 << 12));
		return;
		break;
	case SPI2:
		pos = 14;
		break;
	case SPI3:
		pos = 15;
		break;
	}
	(enable_disable) ? (RCC->APB1ENR |= (1 << pos)) : (RCC->APB1ENR &= ~(1 << pos));
	return;
}

/* This function is a blocking function. We have to wait until it finish the data send */
/* The way of transmiting/receiving data is describe in the reference manual of the microcontroller. You only has to read it and understanding what are they saying.
It's not complicated. */
void SPI_Send_Data(uint8_t SPI_peripheral, uint8_t* data, uint8_t data_len, uint8_t *data_recv)
{
	/* First, in master mode, we have to enable SPI NSS port (case of none multimaster).
	 * If we use hardware mode, it's mandatory to set SSOE bit. In software mode, SSI must
	 * be set. */
	/* IMPORTANT: THIS MUST BE BEFORE THE SPI ACTIVATION, OTHERWISE AN ERROR WILL HAPPEND */
	if(SPIs[SPI_peripheral]->CR1 & 0x200) {
		SPIs[SPI_peripheral]->CR1 |= 0x100;
	} else {
		SPIs[SPI_peripheral]->CR2 |= 0x04;
	}
	
	/* Second, enable the SPI peripheral. WHEN THIS BIT IS ENABLE, COMUNICATION WILL START, AND NO CONFIGURATION CHANGES WILL BE ACCEPTED. */
	SPIs[SPI_peripheral]->CR1 |= (1 << 6);

	/* I'm assuming that DFF is set to 8 bits */
	for(uint8_t i = 0; i < data_len; i++) {
		//Third, we need to check that the TXE flag is empty-> 1
		while(!(SPIs[SPI_peripheral]->SR & 0x0002));

		//Second, we load the data into the TX buffer, to start the sending process process
		SPIs[SPI_peripheral]->DR = *data_recv ;

		//Wait until the bit RXNE is set to 1. This indicate that all the data are transfer.
		while(!(SPIs[SPI_peripheral]->SR & 0x0001));

		//Read the data from the SPI_DR register.
		*(data_recv+i) = SPIs[SPI_peripheral]->DR;
	}

	/* Last, wait until TX flag is empty and BSY flag will be 0. TX flag indicates that TX buffer is empty and BSY indicates that SPI is not busy anymore. */
	while((!(SPIs[SPI_peripheral]->SR & 0x0002)) && (SPIs[SPI_peripheral]->SR & 0x40));

	//Deactivate the SPI peripheral
	SPIs[SPI_peripheral]->CR1 &= ~(1 << 6);
}






