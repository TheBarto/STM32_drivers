#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx.h"

/* SPI -> Serial Peripheral Interface */
typedef struct{
  volatile uint32_t CR1;     /* SPI Control Register 1 */
  volatile uint32_t CR2;     /* SPI Control Register 2 */
  volatile uint32_t SR;      /* SPI Status Register */
  volatile uint32_t DR;      /* SPI Data Register */
  volatile uint32_t CRCPR;   /* SPI CRC Polynomial Register */
  volatile uint32_t RXCRCR;  /* SPI RX CRC Register */
  volatile uint32_t TXCRCR;  /* SPI TX CRC Register */
  volatile uint32_t I2SCFGR; /* SPI I2S Configuration Register */
  volatile uint32_t I2SPR;   /* SPI I2S Prescaler Register */
}SPI_RegDef_t;

#define SPI1_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40013000)
#define SPI2_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40003800)
#define SPI3_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40003C00)
#define SPI4_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40013400)
#define SPI5_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40015000)
#define SPI6_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40015400)

#define TOTAL_STM32F407_SPI ((uint8_t) 6)

static SPI_RegDef_t* SPIs[TOTAL_STM32F407_SPI];

void SPI_initialization_module()
{
	SPIs[SPI1_PERIPHERAL] = SPI1_BASEADDR;
	SPIs[SPI2_PERIPHERAL] = SPI2_BASEADDR;
	SPIs[SPI3_PERIPHERAL] = SPI3_BASEADDR;
	SPIs[SPI4_PERIPHERAL] = SPI4_BASEADDR;
	SPIs[SPI5_PERIPHERAL] = SPI5_BASEADDR;
	SPIs[SPI6_PERIPHERAL] = SPI6_BASEADDR;
}

void SPI_Initialization(uint8_t SPI_peripheral, uint8_t communication_mode,
                        uint8_t mode, uint8_t CPOL, uint8_t CPHA,
                        uint8_t DFF, uint8_t BR_prescaler, uint8_t SSM)
{
	/* This way we get the peripheral clean */
	SPI_Reset(SPI_peripheral);

	uint16_t aux_setter = 0;
	aux_setter |= (CPHA * SPI_CR1_MASK_CPHA);
	aux_setter |= (CPOL * SPI_CR1_MASK_CPOL);
	aux_setter |= (mode * SPI_CR1_MASK_MASTER);
	aux_setter |= (BR_prescaler * SPI_CR1_MASK_BAUD_RATE_CLK);
	aux_setter |= (SSM * SPI_CR1_MASK_SSM);
	aux_setter |= (DFF * SPI_CR1_MASK_DFF);

	/* In Full-Duplex mode, the BIDIOE bit must remain as 0 */
	switch(communication_mode)
	{
	case SPI_HALF_DUPLEX_MODE:
		aux_setter |= SPI_CR1_MASK_BIDIMODE;
		/* BIDIOE bit must be used to deceide which send and receive */
		break;
	case SPI_SIMPLE_CONECTION_MODE:
		/* With this we just disable the output. We're just receiving data. */
		aux_setter |= SPI_CR1_MASK_RX_ONLY;
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
	uint32_t pos = 0;
	switch(SPI_peripheral)
	{
	case SPI1_PERIPHERAL:
		RCC->APB2RSTR |= RCC_APB2_EN_RESET_SPI1;
		RCC->APB2RSTR &= ~RCC_APB2_EN_RESET_SPI1;
		return;
		break;
	case SPI2_PERIPHERAL:
		pos = RCC_APB1_EN_RESET_SPI2;
		break;
	case SPI3_PERIPHERAL:
		pos = RCC_APB1_EN_RESET_SPI3;
		break;
	}
	RCC->APB1RSTR |= pos;
	RCC->APB1RSTR &= ~pos;
	return;
}

void SPI_clock_enable_disabled(uint8_t SPI_peripheral, uint8_t enable_disable)
{
	uint32_t pos = 0;
	switch(SPI_peripheral)
	{
	case SPI1_PERIPHERAL:
		(enable_disable) ? (RCC->APB2ENR |= RCC_APB2_EN_RESET_SPI1) : (RCC->APB2ENR &= ~RCC_APB2_EN_RESET_SPI1);
		return;
		break;
	case SPI2_PERIPHERAL:
		pos = RCC_APB1_EN_RESET_SPI2;
		break;
	case SPI3_PERIPHERAL:
		pos = RCC_APB1_EN_RESET_SPI3;
		break;
	}
	(enable_disable) ? (RCC->APB1ENR |= pos) : (RCC->APB1ENR &= ~pos);
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
	if(SPIs[SPI_peripheral]->CR1 & SPI_CR1_MASK_SSM) {
		SPIs[SPI_peripheral]->CR1 |= SPI_CR1_MASK_SSI;
	} else {
		SPIs[SPI_peripheral]->CR2 |= SPI_CR2_MASK_SSOE;
	}
	
	/* Second, enable the SPI peripheral. WHEN THIS BIT IS ENABLE, COMUNICATION WILL START, AND NO CONFIGURATION CHANGES WILL BE ACCEPTED. */
	SPIs[SPI_peripheral]->CR1 |= SPI_CR1_MASK_SPE;

	/* I'm assuming that DFF is set to 8 bits */
	for(uint8_t i = 0; i < data_len; i++) {
		//Third, we need to check that the TXE flag is empty-> 1
		while(!(SPIs[SPI_peripheral]->SR & SPI_SR_TXE));

		//Second, we load the data into the TX buffer, to start the sending process process
		SPIs[SPI_peripheral]->DR = *data_recv ;

		//Wait until the bit RXNE is set to 1. This indicate that all the data are transfer.
		//while(!(SPIs[SPI_peripheral]->SR & SPI_SR_RXNE));

		//Read the data from the SPI_DR register.
		//*(data_recv+i) = SPIs[SPI_peripheral]->DR;
	}

	/* Last, wait until TX flag is empty and BSY flag will be 0. TX flag indicates that TX buffer is empty and BSY indicates that SPI is not busy anymore. */
	while((!(SPIs[SPI_peripheral]->SR & SPI_SR_TXE)) && (SPIs[SPI_peripheral]->SR & SPI_SR_BSY));

	//Deactivate the SPI peripheral
	SPIs[SPI_peripheral]->CR1 &= ~SPI_CR1_MASK_SPE;
}
