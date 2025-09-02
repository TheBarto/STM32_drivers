#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

/* TODO: ANTES DE CAMBIAR DE ESTADO, COMPROBAR QUE ESTEMOS
 * INICIALIZADOS PARA EVITAR PROBLEMAS
 */ 

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

static uint8_t SPI_default_value = 0xFF;

#define SPI_NON_INIT_ST ((uint8_t) 0x00)
#define SPI_IDLE_ST     ((uint8_t) 0x01)
#define SPI_RXNE_ST     ((uint8_t) 0x02)
#define SPI_TXE_ST      ((uint8_t) 0x04)
#define SPI_TXE_AU_ST   ((uint8_t) 0x08)
#define SPI_MASTER_ST   ((uint8_t) 0x10)

#ifdef BOARDLESS_VERSION
#define SPI_INTERM_ST      ((uint8_t) 0x20)
#define SPI_TICK_STATES \
	(SPI_IDLE_ST | \
	 SPI_TXE_ST  | \
	 SPI_RXNE_ST | \
	 SPI_INTERM_ST )
uint8_t aux_value = 0xFF;
uint8_t aux_i = 0xFF;
#else
#define SPI_TICK_STATES \
	(SPI_IDLE_ST | \
	 SPI_TXE_ST  | \
	 SPI_RXNE_ST )
#endif /* BOARDLESS_VERSION */

#define SPI_SET_TICK_STATE(x, state) \
	do { \
	x &= ~SPI_TICK_STATES; \
	x |= state; \
	} while(0)

#define SPI_SET_IDLE_STATE(x) SPI_SET_TICK_STATE(x, SPI_IDLE_ST)
#define SPI_SET_TXE_STATE(x)  SPI_SET_TICK_STATE(x, SPI_TXE_ST)
#define SPI_SET_RXNE_STATE(x) SPI_SET_TICK_STATE(x, SPI_RXNE_ST)
#ifdef BOARDLESS_VERSION
#define SPI_SET_INTERM_STATE(x) SPI_SET_TICK_STATE(x, SPI_INTERM_ST)
#endif /* BOARDLESS_VERSION */

#ifndef BOARDLESS_VERSION
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
#endif /* BOARDLESS_VERSION */

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
		(enable_disable) ?
			(RCC->APB2ENR |= RCC_APB2_EN_RESET_SPI1) :
			(RCC->APB2ENR &= ~RCC_APB2_EN_RESET_SPI1);
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

//For the moment is just for full-duplex mode.

typedef struct {
	uint16_t total_data;
	uint16_t pos_data;
	uint8_t state;
	uint8_t* data;
	SPI_RegDef_t* spi_reg;
} SPI_Controller_t;

static SPI_Controller_t SPIs[TOTAL_STM32F407_SPI];

void SPI_initialization_module()
{
	memset(&SPIs[SPI1_PERIPHERAL], 0, sizeof(SPI_Controller_t));
	SPIs[SPI1_PERIPHERAL].spi_reg = SPI1_BASEADDR;
	memset(&SPIs[SPI2_PERIPHERAL], 0, sizeof(SPI_Controller_t));
	SPIs[SPI2_PERIPHERAL].spi_reg = SPI2_BASEADDR;
	memset(&SPIs[SPI3_PERIPHERAL], 0, sizeof(SPI_Controller_t));
	SPIs[SPI3_PERIPHERAL].spi_reg = SPI3_BASEADDR;
	memset(&SPIs[SPI4_PERIPHERAL], 0, sizeof(SPI_Controller_t));
	SPIs[SPI4_PERIPHERAL].spi_reg = SPI4_BASEADDR;
	memset(&SPIs[SPI5_PERIPHERAL], 0, sizeof(SPI_Controller_t));
	SPIs[SPI5_PERIPHERAL].spi_reg = SPI5_BASEADDR;
	memset(&SPIs[SPI6_PERIPHERAL], 0, sizeof(SPI_Controller_t));
	SPIs[SPI6_PERIPHERAL].spi_reg = SPI6_BASEADDR;
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

	SPIs[SPI_peripheral].spi_reg->CR1 = aux_setter;
	// WARN: CARGAR AQUI EL MODO MASTER.
	if(mode == SPI_MODE_MASTER) {
		SPIs[SPI_peripheral].spi_reg->CR2 |= SPI_CR2_MASK_SSOE;
		SPIs[SPI_peripheral].state |= SPI_MASTER_ST;
	} else {
		SPIs[SPI_peripheral].state |= SPI_TXE_AU_ST;
	}

	if(interrupt_enable) {
		 SPI_IRQ_configure(SPI_peripheral, IRQ_priority);
		 SPIs[SPI_peripheral].spi_reg->CR2 |= interrupt_enable;
	}

	SPI_SET_IDLE_STATE(SPIs[SPI_peripheral].state);
	/* If we're the slave, activate the peripheral. If
	 * we're the master, wait until we need to send data
	 */
	if(mode != SPI_MODE_MASTER) {
		SPIs[SPI_peripheral].spi_reg->CR1 |= SPI_CR1_MASK_SPE;
	}

#ifdef BOARDLESS_VERSION
	SPIs[SPI_peripheral].spi_reg->SR |= SPI_SR_TXE;
	SPIs[SPI_peripheral].spi_reg->SR |= SPI_SR_RXNE;
#endif /* BOARDLESS_VERSION */
	return;
}

int8_t SPI_continuos_data_send(uint8_t SPI_peripheral, uint8_t* data)
{
	// TODO: COMPROBAR QUE ESTEMOS INICIALIZADOS PRIMERO
	if((SPIs[SPI_peripheral].total_data > 0) ||
	   (SPIs[SPI_peripheral].state & SPI_TXE_AU_ST))
		return -1;

	SPIs[SPI_peripheral].state |= SPI_TXE_AU_ST;
	SPIs[SPI_peripheral].data = data;
	SPIs[SPI_peripheral].total_data = 0;

	if(SPIs[SPI_peripheral].state & SPI_MASTER_ST)
		SPIs[SPI_peripheral].spi_reg->CR1 |= SPI_CR1_MASK_SPE;

	return 0;
}

int8_t SPI_load_data_send(uint8_t SPI_peripheral, uint8_t* data, uint8_t total_data)
{
	// TODO: COMPROBAR QUE ESTEMOS INICIALIZADOS PRIMERO
	if((SPIs[SPI_peripheral].total_data > 0) ||
	   (SPIs[SPI_peripheral].state & SPI_TXE_AU_ST))
		return 1;

	SPIs[SPI_peripheral].data = data;
	SPIs[SPI_peripheral].total_data = total_data;
	// Take off the automatic data send.
	SPIs[SPI_peripheral].state &= ~SPI_TXE_AU_ST;

	//Before send anything, enable/activate the master peripheral
	if(SPIs[SPI_peripheral].state & SPI_MASTER_ST)
		SPIs[SPI_peripheral].spi_reg->CR1 |= SPI_CR1_MASK_SPE;

#if defined(PRINTF_DEBUG)
	printf("Data to send: %s\n", SPIs[SPI_peripheral].data);
#endif
	return 0;
}

/* Try to make a non-blocking system without exceptions */
void SPI_tick()
{
	for(uint8_t i = 0; i < TOTAL_STM32F407_SPI; i++) {
		switch(SPIs[i].state & SPI_TICK_STATES) {
		 case SPI_NON_INIT_ST:
		 	break;
		case SPI_IDLE_ST:
			if((SPIs[i].total_data > 0) ||
			   (SPIs[i].state & SPI_TXE_AU_ST)) {
				SPI_SET_TXE_STATE(SPIs[i].state);
				// if(SPIs[i].state & SPI_TXE_AU_ST)
				// 	SPIs[i].data = &SPI_default_value;
			}
			break;
		case SPI_TXE_ST:
			if(((!SPIs[i].total_data) &&
			   !(SPIs[i].state & SPI_TXE_AU_ST)) ||
			   !(SPIs[i].spi_reg->SR & SPI_SR_TXE)) {
				SPI_SET_IDLE_STATE(SPIs[i].state);
				break;
			}
			SPIs[i].spi_reg->DR = SPIs[i].data[SPIs[i].pos_data];
#ifndef BOARDLESS_VERSION
			SPI_SET_RXNE_STATE(SPIs[i].state);
#else
			SPI_SET_INTERM_STATE(SPIs[i].state);
			break;
		case SPI_INTERM_ST:
			if(aux_value == 0xFF) {
				aux_value = SPIs[i].spi_reg->DR;
				aux_i = i;
			} else {
				SPIs[aux_i].spi_reg->DR = SPIs[i].spi_reg->DR;
				SPIs[i].spi_reg->DR = aux_value;
				SPIs[i].spi_reg->SR|=SPI_SR_RXNE;
				SPIs[aux_i].spi_reg->SR|=SPI_SR_RXNE;
				aux_value = 0xFF;
				aux_i = 0xFF;
			}
			SPI_SET_RXNE_STATE(SPIs[i].state);
#endif /* BOARDLESS_VERSION */
			break;
		case SPI_RXNE_ST:
			if(!(SPIs[i].spi_reg->SR & SPI_SR_RXNE)) {
				break;
			}
#ifdef BOARDLESS_VERSION
			SPIs[i].spi_reg->SR &= ~SPI_SR_RXNE;
#endif /* BOARDLESS_VERSION */
			SPIs[i].data[SPIs[i].pos_data] = SPIs[i].spi_reg->DR;

			if(!SPIs[i].total_data)
				SPIs[i].total_data = SPIs[i].data[SPIs[i].pos_data];

			//WARN: SET RECV BIT TO TRUE
			if(SPIs[i].total_data == SPIs[i].pos_data) {
				SPI_SET_IDLE_STATE(SPIs[i].state);
				SPIs[i].total_data = 0;
				SPIs[i].pos_data = 0;
				(SPIs[i].state & SPI_MASTER_ST) ?
					(SPIs[i].state&=~SPI_TXE_AU_ST) :
					(SPIs[i].state|=SPI_TXE_AU_ST);
				break;
			}
			SPIs[i].pos_data++;
			SPI_SET_TXE_STATE(SPIs[i].state);
		}
	}
}

void SPI_load_data_array(uint8_t SPI_peripheral, uint8_t* array)
{
	SPIs[SPI_peripheral].data = array;
}
