#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx.h"

#define VERSION2

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


#if !defined(BOARDLESS_VERSION)
#define SPI1_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40013000)
#define SPI2_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40003800)
#define SPI3_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40003C00)
#define SPI4_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40013400)
#define SPI5_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40015000)
#define SPI6_BASEADDR  (SPI_RegDef_t *) ((uint32_t) 0x40015400)
#else
/* Simular las dir. base de configuracion de los registros*/
uint8_t SPI_Reg_Mem[216];
uint8_t RCC_mem_struct[136]; /* 34 registros * 4bytes */

#define SPI1_BASEADDR  (SPI_RegDef_t *) ((uint32_t* ) &SPI_Reg_Mem[0])
#define SPI2_BASEADDR  (SPI_RegDef_t *) ((uint32_t* ) &SPI_Reg_Mem[36])
#define SPI3_BASEADDR  (SPI_RegDef_t *) ((uint32_t* ) &SPI_Reg_Mem[72])
#define SPI4_BASEADDR  (SPI_RegDef_t *) ((uint32_t* ) &SPI_Reg_Mem[108])
#define SPI5_BASEADDR  (SPI_RegDef_t *) ((uint32_t* ) &SPI_Reg_Mem[144])
#define SPI6_BASEADDR  (SPI_RegDef_t *) ((uint32_t* ) &SPI_Reg_Mem[180])
#endif

/* Mark as the receptor of the data the SPI 3.*/
uint8_t recp = 2;

#define TOTAL_STM32F407_SPI ((uint8_t) 6)

static void SPI_IRQ_configure(uint8_t SPI_peripheral, uint8_t IRQ_priority)
{
	/* Get the IRQ number for the peripheral */
	uint8_t SPI_IRQ_number = 0;
	switch(SPI_peripheral) {
	case SPI1_PERIPHERAL:
		SPI_IRQ_number = 35;
		break;
	case SPI2_PERIPHERAL:
		SPI_IRQ_number = 36;
		break;
	case SPI3_PERIPHERAL:
		SPI_IRQ_number = 51;
		break;
	case SPI4_PERIPHERAL:
		SPI_IRQ_number = 84;
		break;
	case SPI5_PERIPHERAL:
		SPI_IRQ_number = 85;
		break;
	case SPI6_PERIPHERAL:
		SPI_IRQ_number = 86;
		break;
	}

	/* With the IRQ peripheral number, configure first the IRQ Priority
	 * and then activate the IRQ exception.
	 *
	 * The IRQ priority must be at the four first bit position of the
	 * 8 bits IRQ priority sub-set. THIS CAN BE DIFFERENT IN OTHER PROCESSORS.
	 */
	NVIC->NVIC_IPR[(SPI_IRQ_number/4)] |= ((IRQ_priority*0x10)*((SPI_IRQ_number%4)<<8));
	NVIC->NVIC_ISER[(SPI_IRQ_number/31)] |= (SPI_IRQ_number%31);

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

#if defined(VERSION1)
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
                        uint8_t DFF, uint8_t BR_prescaler, uint8_t SSM,
                        uint8_t interrupt_enable, uint8_t IRQ_priority)
{
	/* Before doing nothing it mandatory to activate the SPI clock */
	SPI_clock_enable_disabled(SPI_peripheral, true);

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
		/* BIDIOE bit must be used to decide which send and receive */
		break;
	case SPI_SIMPLE_CONECTION_RX_MODE:
		/* With this we just disable the output. We're just receiving data. */
		aux_setter |= SPI_CR1_MASK_RX_ONLY;
		break;
	}

#if defined(VERSION1)
	SPIs[SPI_peripheral]->CR1 = aux_setter;

	if(interrupt_enable) {
		 SPI_IRQ_configure(SPI_peripheral, IRQ_priority);
		 SPIs[SPI_peripheral]->CR2 |= interrupt_enable;
	}
#else
	SPIs[SPI_peripheral].spi_driver->CR1 = aux_setter;

	if(interrupt_enable) {
		 SPI_IRQ_configure(SPI_peripheral, IRQ_priority);
		 SPIs[SPI_peripheral].spi_driver->CR2 |= interrupt_enable;
	}
#endif

	return;
}

void enable_disable_SPI_peripheral(uint8_t SPI_peripheral, bool enable)
{

	/* First of all, activate the NSS exit, by software of hardware. */
	if(enable) {
		(SPIs[SPI_peripheral]->CR1 & SPI_CR1_MASK_SSM) ?
			(SPIs[SPI_peripheral]->CR1 |= SPI_CR1_MASK_SSI) :
			(SPIs[SPI_peripheral]->CR2 |= SPI_CR2_MASK_SSOE);

		/* Second, enable the SPI peripheral. WHEN THIS BIT IS ENABLE,
		 * COMUNICATION WILL START, AND NO CONFIGURATION CHANGES WILL
		 * BE ACCEPTED. */
		SPIs[SPI_peripheral]->CR1 |= SPI_CR1_MASK_SPE;
	} else {
		(SPIs[SPI_peripheral]->CR1 & SPI_CR1_MASK_SSM) ?
			(SPIs[SPI_peripheral]->CR1 |= SPI_CR1_MASK_SSI) :
			(SPIs[SPI_peripheral]->CR2 |= SPI_CR2_MASK_SSOE);

		//Deactivate the SPI peripheral
		SPIs[SPI_peripheral]->CR1 &= ~SPI_CR1_MASK_SPE;
	}
	return;
}

/* This function is a blocking function. We have to wait until it finish the data send */
/* The way of transmiting/receiving data is describe in the reference manual of the microcontroller. You only has to read it and understanding what are they saying.
It's not complicated. */
void SPI_Send_Receive_Data(uint8_t SPI_peripheral, uint8_t* data, uint8_t data_len, uint8_t *data_recv)
{
	/* First, in master mode, we have to enable SPI NSS port (case of none multimaster).
	 * If we use hardware mode, it's mandatory to set SSOE bit. In software mode, SSI must
	 * be set. */
	/* IMPORTANT: THIS MUST BE BEFORE THE SPI ACTIVATION, OTHERWISE AN ERROR WILL HAPPEND */
	enable_disable_SPI_peripheral(SPI_peripheral, true);

	for(uint8_t i = 0; i < data_len; i++) {
		//Third, we need to check that the TXE flag is empty-> 1
		while(!(SPIs[SPI_peripheral]->SR & SPI_SR_TXE));

		//Second, we load the data into the TX buffer, to start the sending process process
		SPIs[SPI_peripheral]->DR = (SPIs[SPI_peripheral]->CR1&SPI_CR1_MASK_DFF) ? ((data[i]*0x100)+data[i+1]) : data[i];

		//Wait until the bit RXNE is set to 1. This indicate that all the data are transfer.
		while(!(SPIs[SPI_peripheral]->SR & SPI_SR_RXNE));

		//Read the data from the SPI_DR register.
		if(SPIs[SPI_peripheral]->CR1&SPI_CR1_MASK_DFF) {
			*(data_recv+i) = SPIs[SPI_peripheral]->DR/0x100;
			*(data_recv+i+1) = SPIs[SPI_peripheral]->DR%0x100;
		} else {
			*(data_recv+i) = SPIs[SPI_peripheral]->DR;
		}
	}

	/* Last, wait until TX flag is empty and BSY flag will be 0. TX flag indicates that TX buffer is empty and BSY indicates that SPI is not busy anymore. */
	while((!(SPIs[SPI_peripheral]->SR & SPI_SR_TXE)) && (SPIs[SPI_peripheral]->SR & SPI_SR_BSY));

	enable_disable_SPI_peripheral(SPI_peripheral, false);
}

void SPI_Send_Data(uint8_t SPI_peripheral, uint8_t* data, uint8_t data_len)
{
	/* First, in master mode, we have to enable SPI NSS port (case of none multimaster).
	 * If we use hardware mode, it's mandatory to set SSOE bit. In software mode, SSI must
	 * be set. */
	/* IMPORTANT: THIS MUST BE BEFORE THE SPI ACTIVATION, OTHERWISE AN ERROR WILL HAPPEND */
	//enable_disable_SPI_peripheral(SPI_peripheral, true);

	/* Second, enable the SPI peripheral. WHEN THIS BIT IS ENABLE, COMUNICATION WILL START, AND NO CONFIGURATION CHANGES WILL BE ACCEPTED. */
	//SPIs[SPI_peripheral]->CR1 |= SPI_CR1_MASK_SPE;

	/* I'm assuming that DFF is set to 8 bits */
	for(uint8_t i = 0; i < data_len; i++) {
		//Third, we need to check that the TXE flag is empty-> 1
		while(!(SPIs[SPI_peripheral]->SR & SPI_SR_TXE));

		//Second, we load the data into the TX buffer, to start the sending process process
		SPIs[SPI_peripheral]->DR = (SPIs[SPI_peripheral]->CR1&SPI_CR1_MASK_DFF) ?
				                   ((data[i]*0x100)+data[i+1]) : data[i];

		/* If the BIDIRECTIONAL MODE is NOT enable, and we can send data => FULL DUPLEX,
		 * then read the value received, and clear the RXNE Flag. COMPLETE IT*/
		if(!(SPIs[SPI_peripheral]->CR1 & SPI_CR1_MASK_BIDIMODE)) {


		}
	}

	/* Last, wait until TX flag is empty and BSY flag will be 0. TX flag indicates that TX buffer is empty and BSY indicates that SPI is not busy anymore. */
	while((!(SPIs[SPI_peripheral]->SR & SPI_SR_TXE)) && (SPIs[SPI_peripheral]->SR & SPI_SR_BSY));

	//Deactivate the SPI peripheral
	//enable_disable_SPI_peripheral(SPI_peripheral, false);
}

void SPI_Receive_Data(uint8_t SPI_peripheral, uint8_t *data_recv)
{
	/* First, in master mode, we have to enable SPI NSS port (case of none multimaster).
	 * If we use hardware mode, it's mandatory to set SSOE bit. In software mode, SSI must
	 * be set. */
	/* IMPORTANT: THIS MUST BE BEFORE THE SPI ACTIVATION, OTHERWISE AN ERROR WILL HAPPEND */
	//enable_disable_SPI_peripheral(SPI_peripheral, true);

	/* Second, enable the SPI peripheral. WHEN THIS BIT IS ENABLE, COMUNICATION WILL START, AND NO CONFIGURATION CHANGES WILL BE ACCEPTED. */
	//SPIs[SPI_peripheral]->CR1 |= SPI_CR1_MASK_SPE;

	//Wait until the bit RXNE is set to 1. This indicate that all the data are transfer.
	while(!(SPIs[SPI_peripheral]->SR & SPI_SR_RXNE));

	//Read the data from the SPI_DR register.
	if(SPIs[SPI_peripheral]->CR1&SPI_CR1_MASK_DFF) {
		*(data_recv) = SPIs[SPI_peripheral]->DR/0x100;
		*(data_recv+1) = SPIs[SPI_peripheral]->DR%0x100;
	} else {
		*(data_recv) = SPIs[SPI_peripheral]->DR;
	}

	/* Last, wait until TX flag is empty and BSY flag will be 0. TX flag indicates that TX buffer is empty and BSY indicates that SPI is not busy anymore. */
	//while((!(SPIs[SPI_peripheral]->SR & SPI_SR_TXE)) && (SPIs[SPI_peripheral]->SR & SPI_SR_BSY));

	//Deactivate the SPI peripheral
	//enable_disable_SPI_peripheral(SPI_peripheral, false);
}

#elif defined(VERSION2)
//For the moment is just for full-duplex mode.

#include <stdio.h>
#include <string.h>

typedef enum {
	SPI_controller_st_non_init = 0,
	SPI_controller_st_idle,
	SPI_controller_st_send,
	SPI_controller_st_send_w,
	SPI_controller_st_recv,
	SPI_controller_st_recv_w,
	SPI_controller_st_deactive_SPI,
	SPI_controller_st_recv_ACK_NACK,
#if defined(BOARDLESS_VERSION)
	SPI_controller_st_pass_data,
#endif
} SPI_Controller_States;

#define MAX_DATA_SEND ((uint8_t) 50)
#define MAX_DATA_RECV ((uint8_t) 50)

typedef struct{
	SPI_RegDef_t* spi_driver;
	uint8_t data_send[MAX_DATA_SEND];
	uint8_t ind_data_send;
	uint8_t total_data_send;
	uint8_t data_recv[MAX_DATA_RECV];
	uint8_t ind_data_recv;
	uint8_t total_data_recv;
	uint8_t state;
}SPI_Controller;

static SPI_Controller SPIs[TOTAL_STM32F407_SPI];

void SPI_initialization_module()
{
	memset(&SPIs[SPI1_PERIPHERAL], 0, sizeof(SPI_Controller));
	SPIs[SPI1_PERIPHERAL].spi_driver = SPI1_BASEADDR;
	memset(&SPIs[SPI2_PERIPHERAL], 0, sizeof(SPI_Controller));
	SPIs[SPI2_PERIPHERAL].spi_driver = SPI2_BASEADDR;
	memset(&SPIs[SPI3_PERIPHERAL], 0, sizeof(SPI_Controller));
	SPIs[SPI3_PERIPHERAL].spi_driver = SPI3_BASEADDR;
	memset(&SPIs[SPI4_PERIPHERAL], 0, sizeof(SPI_Controller));
	SPIs[SPI4_PERIPHERAL].spi_driver = SPI4_BASEADDR;
	memset(&SPIs[SPI5_PERIPHERAL], 0, sizeof(SPI_Controller));
	SPIs[SPI5_PERIPHERAL].spi_driver = SPI5_BASEADDR;
	memset(&SPIs[SPI6_PERIPHERAL], 0, sizeof(SPI_Controller));
	SPIs[SPI6_PERIPHERAL].spi_driver = SPI6_BASEADDR;
}

void SPI_Initialization(uint8_t SPI_peripheral, uint8_t communication_mode,
                        uint8_t mode, uint8_t CPOL, uint8_t CPHA,
                        uint8_t DFF, uint8_t BR_prescaler, uint8_t SSM,
                        uint8_t interrupt_enable, uint8_t IRQ_priority)
{
	/* Before doing nothing it mandatory to activate the SPI clock */
	SPI_clock_enable_disabled(SPI_peripheral, true);

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
		/* BIDIOE bit must be used to decide which send and receive */
		break;
	case SPI_SIMPLE_CONECTION_RX_MODE:
		/* With this we just disable the output. We're just receiving data. */
		aux_setter |= SPI_CR1_MASK_RX_ONLY;
		break;
	}

	SPIs[SPI_peripheral].spi_driver->CR1 = aux_setter;

	if(interrupt_enable) {
		 SPI_IRQ_configure(SPI_peripheral, IRQ_priority);
		 SPIs[SPI_peripheral].spi_driver->CR2 |= interrupt_enable;
	}

	SPIs[SPI_peripheral].state = SPI_controller_st_idle;

	return;
}

void SPI_load_data_send(uint8_t SPI_peripheral, uint8_t* data, uint8_t total_data)
{
	memcpy(&SPIs[SPI_peripheral].data_send[1], &data[0], total_data);
	SPIs[SPI_peripheral].ind_data_send = 0;
	SPIs[SPI_peripheral].total_data_send = (total_data+1); //+1 is for the size
	SPIs[SPI_peripheral].data_send[0] = total_data;
	SPIs[SPI_peripheral].state = SPI_controller_st_send;

	//Before send anything, enable/activate the peripheral
	enable_disable_SPI_peripheral(SPI_peripheral, true);

#if defined(PRINTF_DEBUG)
	printf("Data to send: %s\n", SPIs[SPI_peripheral].data_send);
#endif
	return;
}

void enable_disable_SPI_peripheral(uint8_t SPI_peripheral, bool enable)
{

#if defined(PRINTF_DEBUG)
	printf("enable_disable_SPI_peripheral, to peripheral: %d, enable: %d\n",
	       SPI_peripheral, enable);
#endif

	/* First of all, activate the NSS exit, by software of hardware. */
	if(enable) {
		(SPIs[SPI_peripheral].spi_driver->CR1 & SPI_CR1_MASK_SSM) ?
			(SPIs[SPI_peripheral].spi_driver->CR1 |= SPI_CR1_MASK_SSI) :
			(SPIs[SPI_peripheral].spi_driver->CR2 |= SPI_CR2_MASK_SSOE);

		/* Second, enable the SPI peripheral. WHEN THIS BIT IS ENABLE,
		 * COMUNICATION WILL START, AND NO CONFIGURATION CHANGES WILL
		 * BE ACCEPTED. */
		SPIs[SPI_peripheral].spi_driver->CR1 |= SPI_CR1_MASK_SPE;

#if defined(BOARDLESS_VERSION)
		//Enable the transmition flag
		SPIs[SPI_peripheral].spi_driver->SR |= SPI_SR_TXE;
#endif
	} else {
		//Deactivate the SPI peripheral
		SPIs[SPI_peripheral].spi_driver->CR1 &= ~SPI_CR1_MASK_SPE;

		(SPIs[SPI_peripheral].spi_driver->CR1 & SPI_CR1_MASK_SSM) ?
			(SPIs[SPI_peripheral].spi_driver->CR1 &= ~SPI_CR1_MASK_SSI) :
			(SPIs[SPI_peripheral].spi_driver->CR2 &= ~SPI_CR2_MASK_SSOE);
	}
	return;
}

int8_t SPI_peripheral_able(uint8_t SPI_peripheral)
{
	return ((SPIs[SPI_peripheral].ind_data_send == 0) &&
			(SPIs[SPI_peripheral].state == SPI_controller_st_idle)) ?
			0 : -1;
}

/* Try to make a non-blocking system without exceptions */
void SPI_Controller_tick()
{
	for(uint8_t i = 0; i < TOTAL_STM32F407_SPI; i++) {
		switch(SPIs[i].state) {
		case SPI_controller_st_non_init:
			break;
		case SPI_controller_st_idle:
			if((SPIs[i].ind_data_send < SPIs[i].total_data_send) &&
			   (SPIs[i].spi_driver->SR & SPI_SR_TXE))
				SPIs[i].state = SPI_controller_st_send;
			else if(SPIs[i].spi_driver->SR & SPI_SR_RXNE) //BUG QUE MARCA RXNE SIN ESTARLO. CAE EN BUCLE INFINITO
				SPIs[i].state = SPI_controller_st_recv;
			//Check if we have something to send or recv something
			break;
		case SPI_controller_st_recv:
			if(!(SPIs[i].spi_driver->SR & SPI_SR_RXNE)) {
				SPIs[i].state = SPI_controller_st_idle;
				break;
			}

			//Must check that total_data_recv == 0, first data is the total
			if(SPIs[i].ind_data_recv == 0) {
				SPIs[i].total_data_recv = (SPIs[i].spi_driver->DR%0x100);
			} else {
				//Read the data from the SPI_DR register.
				if(SPIs[i].spi_driver->CR1&SPI_CR1_MASK_DFF) {
					uint16_t aux = SPIs[i].spi_driver->DR;
					SPIs[i].data_recv[SPIs[i].ind_data_recv] = (aux/0x100);
					SPIs[i].data_recv[SPIs[i].ind_data_recv+1] = (aux%0x100);
					SPIs[i].ind_data_recv++;
				} else {
					SPIs[i].data_recv[SPIs[i].ind_data_recv] = SPIs[i].spi_driver->DR;
				}
			}
			SPIs[i].ind_data_recv++;

			if((SPIs[i].ind_data_send > 0) &&
			   (SPIs[i].total_data_send)) {
				SPIs[i].state = SPI_controller_st_send;
			} else if(SPIs[i].ind_data_recv == SPIs[i].total_data_recv) {
				//call a callback, or something to save the data
#if defined(PRINTF_DEBUG)
				printf("Data received: %s\n", SPIs[i].data_recv);
#endif
				SPIs[i].state = SPI_controller_st_idle;
			}
			break;
		case SPI_controller_st_send:
			if((!SPIs[i].total_data_send)) {
				SPIs[i].state = SPI_controller_st_idle;
				break;
			}else if(!(SPIs[i].spi_driver->SR & SPI_SR_TXE)) {
				break;
			}
			SPIs[i].spi_driver->DR = (SPIs[i].spi_driver->CR1&SPI_CR1_MASK_DFF) ?
										((SPIs[i].data_send[SPIs[i].ind_data_send]*0x100)+
										  SPIs[i].data_send[SPIs[i].ind_data_send+1]) :
										  SPIs[i].data_send[SPIs[i].ind_data_send];

			/*******************/
			SPIs[i].ind_data_send++; //BUG, SI CR1_MASK_DFF, SUMAR 2
			if(SPIs[i].ind_data_send < SPIs[i].total_data_send) {
				SPIs[i].state = SPI_controller_st_recv;
			} else if(SPIs[i].ind_data_send == SPIs[i].total_data_send) {
#if defined(PRINTF_DEBUG)
				printf("All data sent, pass to state: SPI_controller_st_deactive_SPI\n");
#endif
				SPIs[i].state = SPI_controller_st_deactive_SPI;
			}
			/*******************/
#if !defined(BOARDLESS_VERSION)
			//SPIs[i].state = SPI_controller_st_send_w;
#else
			SPIs[i].state = SPI_controller_st_pass_data;
		case SPI_controller_st_pass_data:
#if defined(PRINTF_DEBUG)
			printf(">>>> Data sent MASTER TO SLAVE: %c\n", SPIs[i].spi_driver->DR);
			printf("<< Data sent SLAVE TO MASTER: %c\n", SPIs[recp].spi_driver->DR);
#endif
			uint16_t swap = SPIs[i].spi_driver->DR;
			SPIs[i].spi_driver->DR = SPIs[recp].spi_driver->DR;
			SPIs[recp].spi_driver->DR = swap;

			SPIs[recp].spi_driver->SR |= SPI_SR_RXNE;
			SPIs[i].spi_driver->SR |= SPI_SR_RXNE;
			SPIs[i].spi_driver->SR |= SPI_SR_TXE;
#endif
			break;
		case SPI_controller_st_send_w:
			if((SPIs[i].spi_driver->CR1 & SPI_CR1_MASK_BIDIMODE) ||
			   (!(SPIs[i].spi_driver->SR & SPI_SR_RXNE)))
				break;

			uint16_t aux = SPIs[i].spi_driver->DR;
			/* SPI_Receive_Data */
			if(SPIs[i].spi_driver->CR1&SPI_CR1_MASK_DFF) {
				SPIs[i].data_recv[SPIs[i].ind_data_recv] = aux/0x100;
				SPIs[i].data_recv[SPIs[i].ind_data_recv+1] = aux%0x100;
				SPIs[i].ind_data_recv++;
			} else {
				SPIs[i].data_recv[SPIs[i].ind_data_recv] = SPIs[i].spi_driver->DR;
			}
			SPIs[i].ind_data_recv++;

			SPIs[i].ind_data_send++;
			if(SPIs[i].ind_data_send < SPIs[i].total_data_send) {
				SPIs[i].state = SPI_controller_st_send;
			} else {
#if defined(PRINTF_DEBUG)
				printf("All data sent, pass to state: SPI_controller_st_deactive_SPI\n"); 
#endif
				SPIs[i].state = SPI_controller_st_deactive_SPI;
				//Use function to deactivate
				//while((!(SPIs[SPI_peripheral]->SR & SPI_SR_TXE)) && (SPIs[SPI_peripheral]->SR & SPI_SR_BSY));
			}
			//SPIs[i].state = SPI_controller_st_recv_ACK_NACK;
			break;
		case SPI_controller_st_deactive_SPI:
			if((!(SPIs[i].spi_driver->SR & SPI_SR_TXE)) ||
				(SPIs[i].spi_driver->SR & SPI_SR_BSY))
				break;
			SPIs[i].state = SPI_controller_st_idle;
			SPIs[i].ind_data_send = 0;
			SPIs[i].total_data_send = 0;
			SPIs[i].ind_data_recv = 0;
			SPIs[i].total_data_recv = 0;
#if defined(PRINTF_DEBUG)
			printf("From st_deactive_SPI to st_idle\n");
#endif
			enable_disable_SPI_peripheral(i, false);
			break;
		/*case SPI_controller_st_recv_ACK_NACK:
			if((SPIs[i]->CR1 & SPI_CR1_MASK_BIDIMODE) ||
			   (!SPIs[i]->SR & SPI_SR_RXNE))
				break;
			SPI_Receive_Data(i, uint8_t *data_recv);
			if(data_recv == ACK)
				SPIs[i].ind_data_send++;

			if(SPIs[i].ind_data_send < SPIs[i].total_data_send)
				SPIs[i].state = SPI_controller_st_send;
			else
				SPIs[i].state = SPI_controller_st_idle;
			break;*/

		}
	}
}

void SPI_Send_Receive_Data(uint8_t SPI_peripheral, uint8_t* data, uint8_t data_len)
{
	/* First, in master mode, we have to enable SPI NSS port (case of none multimaster).
	 * If we use hardware mode, it's mandatory to set SSOE bit. In software mode, SSI must
	 * be set. */
	/* IMPORTANT: THIS MUST BE BEFORE THE SPI ACTIVATION, OTHERWISE AN ERROR WILL HAPPEND */
	enable_disable_SPI_peripheral(SPI_peripheral, true);

	for(uint8_t i = 0; i < data_len; i++) {
		//Third, we need to check that the TXE flag is empty-> 1
		while(!(SPIs[SPI_peripheral].spi_driver->SR & SPI_SR_TXE));

		//Second, we load the data into the TX buffer, to start the sending process process
		SPIs[SPI_peripheral].spi_driver->DR = (SPIs[SPI_peripheral].spi_driver->CR1&SPI_CR1_MASK_DFF) ? ((data[i]*0x100)+data[i+1]) : data[i];

		//Wait until the bit RXNE is set to 1. This indicate that all the data are transfer.
		while(!(SPIs[SPI_peripheral].spi_driver->SR & SPI_SR_RXNE));

		//Read the data from the SPI_DR register.
		if(SPIs[SPI_peripheral].spi_driver->CR1&SPI_CR1_MASK_DFF) {
			SPIs[SPI_peripheral].data_recv[i] = SPIs[SPI_peripheral].spi_driver->DR/0x100;
			SPIs[SPI_peripheral].data_recv[(i+1)] = SPIs[SPI_peripheral].spi_driver->DR%0x100;
		} else {
			SPIs[SPI_peripheral].data_recv[i] = SPIs[SPI_peripheral].spi_driver->DR;
		}
	}

	/* Last, wait until TX flag is empty and BSY flag will be 0. TX flag indicates that TX buffer is empty and BSY indicates that SPI is not busy anymore. */
	while((!(SPIs[SPI_peripheral].spi_driver->SR & SPI_SR_TXE)) && (SPIs[SPI_peripheral].spi_driver->SR & SPI_SR_BSY));

	enable_disable_SPI_peripheral(SPI_peripheral, false);
}
#endif
