#include "../include/stm32f407xx_iic_driver.h"
#include <string.h>

#define I2C_TOTAL_PERIPHERALS ((uint8_t)3)

typedef struct {
	volatile uint32_t I2C_CR1;
	volatile uint32_t I2C_CR2;
	volatile uint32_t I2C_OAR1;
	volatile uint32_t I2C_OAR2;
	volatile uint32_t I2C_DR;
	volatile uint32_t I2C_SR1;
	volatile uint32_t I2C_SR2;
	volatile uint32_t I2C_CCR;
	volatile uint32_t I2C_TRISE;
	volatile uint32_t I2C_FLTR;
} I2C_RegDef_t;

I2C_RegDef_t *I2C_peripherals[I2C_TOTAL_PERIPHERALS];

typedef enum {
	I2C_send_OK = 0,
	I2C_send_ERR,
	I2C_send_WAIT,
	I2C_recv_OK,
	I2C_recv_ERR,
	I2C_recv_WAIT
} I2C_enums;

#if 0
void I2C_initialization_module()
{
	I2C_peripherals[I2C_PERIPHERAL_1] = (I2C_RegDef_t *)I2C1B_ADDR;
	I2C_peripherals[I2C_PERIPHERAL_2] = (I2C_RegDef_t *)I2C2B_ADDR;
	I2C_peripherals[I2C_PERIPHERAL_3] = (I2C_RegDef_t *)I2C3B_ADDR;

	return;
}

void I2C_initialization(uint8_t  I2C_peripheral,
                        bool     ACK_enable,
                        bool     stretching_enable,
                        uint8_t  address,
                        bool     fast_mode,
                        uint8_t  APB_freq,
                        uint16_t SCLK)
{
	uint16_t aux_config = 0;

	// First thing is to enable the RCC clock for this peripheral
	switch (I2C_peripheral) {
	case I2C_PERIPHERAL_1: RCC->APB1ENR |= RCC_APB1_EN_RESET_I2C1; break;
	case I2C_PERIPHERAL_2: RCC->APB1ENR |= RCC_APB1_EN_RESET_I2C2; break;
	case I2C_PERIPHERAL_3: RCC->APB1ENR |= RCC_APB1_EN_RESET_I2C3; break;
	}
	// Config I2C_CR1 register
	if (ACK_enable)
		aux_config |= I2C_CR1_ACK_BIT;

	if (!stretching_enable)
		aux_config |= I2C_CR1_STRETCH_BIT;
	I2C_peripherals[I2C_peripheral]->I2C_CR1 = aux_config;

	// Config I2C_CR2 register. FREQ value
	I2C_peripherals[I2C_peripheral]->I2C_CR2 |= (APB_freq & I2C_CR2_FREQ_FIELD);

	// Config the OAR1 register - Address of the peripheral
	// *2 to shift one position to the left
	I2C_peripherals[I2C_peripheral]->I2C_OAR1 |= (address * 2);
	I2C_peripherals[I2C_peripheral]->I2C_OAR1 |= I2C_OAR1_ALWAYS_1_BIT;
	// Config the CCR register to generate the clock signal correctly
	if (fast_mode)
		I2C_peripherals[I2C_peripheral]->I2C_CCR |= I2C_CCR_FS_BIT;

	aux_config = (SCLK / 2);
	aux_config = (APB_freq / (SCLK * 2));
	I2C_peripherals[I2C_peripheral]->I2C_CCR |=
	        (aux_config * I2C_CCR_CCR_FIELD);

	// Config the TRISE register.
	aux_config = ((1 / APB_freq) * 1000000000);
	aux_config = (1000 / aux_config) + 1;
	I2C_peripherals[I2C_peripheral]->I2C_TRISE |=
	        (aux_config * I2C_TRISE_TRISE_FIELD);

	// Enable the peripheral
	I2C_peripherals[I2C_peripheral]->I2C_CR1 |= I2C_CR1_ENABLE_BIT;

	return;
}

void I2C_set_master(uint8_t I2C_peripheral, uint8_t address_dst, bool send_recv)
{
	I2C_peripherals[I2C_peripheral]->I2C_CR1 |= I2C_CR1_START_BIT;
	// Wait until the SB flag indicates that the
	while (!(I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_START_BIT))
		;

	// Read the SR1 register to clear SR1
	uint32_t aux = I2C_peripherals[I2C_peripheral]->I2C_SR1;

	// Create the msj to set the address and the mode
	// Fix the direction of the element we want to communicate with
	I2C_peripherals[I2C_peripheral]->I2C_DR |= (address_dst * 2);

	// Wait until receive an ACK and bit ADDR will set to 1
	while (!(I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_ADDR_BIT))
		;

	// clear the SR1 register. To do this read SR1 and SR2
	aux = I2C_peripherals[I2C_peripheral]->I2C_SR1;
	aux = I2C_peripherals[I2C_peripheral]->I2C_SR2;

	// Now we are master and have the slave ready to send
	return;
}

void I2C_send_data(uint8_t  I2C_peripheral,
                   uint8_t *data_send,
                   uint8_t  total_data_send)
{
	// Start sending the total data length
	while (!(I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_TXE_BIT))
		;
	I2C_peripherals[I2C_peripheral]->I2C_DR = total_data_send;

	do {
		// Wait until the TXE is set to 1 - Data register empty
		while (!(I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_TXE_BIT))
			;
		// If slave and receive a NACK from master, no more packets wanted.
		if ((I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_AF_BIT) &&
		    (!(I2C_peripherals[I2C_peripheral]->I2C_SR2 & I2C_SR2_MST_BIT))) {
			break;
		} else if ((I2C_peripherals[I2C_peripheral]->I2C_SR2 &
		            I2C_SR2_MST_BIT) &&
		           (I2C_peripherals[I2C_peripheral]->I2C_SR1 &
		            I2C_SR1_AF_BIT)) {
			// If master mode, and NACK is received, last packet was lost.
			data_send--;
			total_data_send++;
		}

		I2C_peripherals[I2C_peripheral]->I2C_DR = *data_send;
		total_data_send--;
		data_send++;

	} while (total_data_send);

	return;
}

void I2C_receive_data(uint8_t  I2C_peripheral,
                      uint8_t *data_recv,
                      uint8_t  total_data_recv)
{
	uint8_t aux_total_data_recv = 0;
	bool ack_en = (I2C_peripherals[I2C_peripheral]->I2C_CR1 & I2C_CR1_ACK_BIT);
	// Load the total data to receive
	while (!(I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_RXNE_BIT))
		;
	aux_total_data_recv = I2C_peripherals[I2C_peripheral]->I2C_DR;
	total_data_recv     = aux_total_data_recv;

	// Start receiving all the data
	do {
		// Wait until the RXNE is set to 1 - Data received
		while (!(I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_RXNE_BIT))
			;

		// If we are master device
		if ((I2C_peripherals[I2C_peripheral]->I2C_SR2 & I2C_SR2_MST_BIT)) {
			// If 3 elements remains, N-2, set ACK to low
			if (aux_total_data_recv == 3)
				// TODO: RESTORE THIS VALUE
				I2C_peripherals[I2C_peripheral]->I2C_CR1 &= ~I2C_CR1_ACK_BIT;
			else if (aux_total_data_recv == 2)
				I2C_peripherals[I2C_peripheral]->I2C_CR1 |= I2C_CR1_STOP_BIT;
		}
		*data_recv = I2C_peripherals[I2C_peripheral]->I2C_DR;
		aux_total_data_recv--;

	} while (aux_total_data_recv);

	// TODO: CHECK THIS PROPERTLY
	if ((I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_STOPF_BIT) &&
	    (!(I2C_peripherals[I2C_peripheral]->I2C_SR2 & I2C_SR2_MST_BIT))) {
		// Finalization process correctly.
	}

	if (ack_en)
		I2C_peripherals[I2C_peripheral]->I2C_CR1 |= I2C_CR1_ACK_BIT;

	return;
}

void I2C_release_master(uint8_t I2C_peripheral)
{
	// First, check the TXE and BTF bits are set
	while (!((I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_CR1_STRETCH_BIT) ||
	         (I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_STOPF_BIT)))
		;

	// After sending all data generate a stop condition.
	I2C_peripherals[I2C_peripheral]->I2C_CR1 |= I2C_CR1_STOP_BIT;

	return;
}
#endif
//-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- ----

typedef struct {
	I2C_RegDef_t *I2C_regs;
	uint8_t       buff_send[50];
	uint8_t       buff_recv[50];
	uint8_t       total_data_send;
	uint8_t       total_data_recv;
	uint16_t state; //El byte de mayor peso se emplea para almacenar el addrs del destino
} I2C_peripheral;

#define I2C_STATE_ACTIVE      0x01
#define I2C_STATE_MASTER      0x02
#define I2C_STATE_1_DATA_RECV 0x04
#define I2C_STATE_CHANGE_W_R  0x08
#define I2C_STATE_ACK_ENABLE  0x10
#define I2C_STATE_WR_DATA     0x20 //Indica si estamos enviando datos (modo envio)
#define I2C_STATE_SEND_DATA   0x40 // If not true, we're in read data
/*#define I2C_STATE_INITIALIZE  0x80*/

#define I2C_CHECK_STATE_STATUS(x, status) (I2C[x].state & status)

#define I2C_CHECK_STATE_ACTIVE(x) I2C_CHECK_STATE_STATUS(x, I2C_STATE_ACTIVE)
#define I2C_CHECK_STATE_MASTER(x) I2C_CHECK_STATE_STATUS(x, I2C_STATE_MASTER)
#define I2C_CHECK_STATE_1_DATA_RECV(x)                                         \
	I2C_CHECK_STATE_STATUS(x, I2C_STATE_1_DATA_RECV)
#define I2C_CHECK_STATE_CHANGE_W_R(x)                                          \
	I2C_CHECK_STATE_STATUS(x, I2C_STATE_CHANGE_W_R)
#define I2C_CHECK_STATE_ACK_ENABLE(x)                                          \
	I2C_CHECK_STATE_STATUS(x, I2C_STATE_ACK_ENABLE)
/*#define I2C_CHECK_STATE_INITIALIZE(x)                                          \*/
/*	I2C_CHECK_STATE_STATUS(x, I2C_STATE_INITIALIZE)*/

I2C_peripheral I2C[I2C_TOTAL_PERIPHERALS];

void I2C_initialization_module()
{
	I2C_peripherals[I2C_PERIPHERAL_1] = (I2C_RegDef_t *)I2C1B_ADDR;
	I2C_peripherals[I2C_PERIPHERAL_2] = (I2C_RegDef_t *)I2C2B_ADDR;
	I2C_peripherals[I2C_PERIPHERAL_3] = (I2C_RegDef_t *)I2C3B_ADDR;

	return;
}

void I2C_initialization(uint8_t  I2C_periph,
                        bool     ACK_enable,
                        bool     stretching_enable,
                        uint8_t  address,
                        bool     fast_mode,
                        uint8_t  APB_freq,
                        uint16_t SCLK)
{
	uint16_t        aux_config = 0;
	I2C_peripheral *I2C        = &I2C[I2C_periph];
	I2C->state                 = 0;

	// First thing is to enable the RCC clock for this peripheral
	switch (I2C_periph) {
	case I2C_PERIPHERAL_1: RCC->APB1ENR |= RCC_APB1_EN_RESET_I2C1; break;
	case I2C_PERIPHERAL_2: RCC->APB1ENR |= RCC_APB1_EN_RESET_I2C2; break;
	case I2C_PERIPHERAL_3: RCC->APB1ENR |= RCC_APB1_EN_RESET_I2C3; break;
	}
	// Config I2C_CR1 register
	if (ACK_enable) {
		aux_config |= I2C_CR1_ACK_BIT;
		I2C->state |= I2C_STATE_ACK_ENABLE;
	}

	if (!stretching_enable)
		aux_config |= I2C_CR1_STRETCH_BIT;
	I2C->I2C_regs->I2C_CR1 = aux_config;

	// Config I2C_CR2 register. FREQ value
	I2C->I2C_regs->I2C_CR2 |= (APB_freq & I2C_CR2_FREQ_FIELD);

	// Config the OAR1 register - Address of the peripheral
	// *2 to shift one position to the left
	I2C->I2C_regs->I2C_OAR1 |= (address * 2);
	I2C->I2C_regs->I2C_OAR1 |= I2C_OAR1_ALWAYS_1_BIT;
	// Config the CCR register to generate the clock signal correctly
	if (fast_mode)
		I2C->I2C_regs->I2C_CCR |= I2C_CCR_FS_BIT;

	aux_config = (SCLK / 2);
	aux_config = (APB_freq / (SCLK * 2));
	I2C->I2C_regs->I2C_CCR |= (aux_config * I2C_CCR_CCR_FIELD);

	// Config the TRISE register. Search the formaula at the reference manual
	aux_config = ((1 / APB_freq) * 1000000000);
	aux_config = (1000 / aux_config) + 1;
	I2C->I2C_regs->I2C_TRISE |= (aux_config * I2C_TRISE_TRISE_FIELD);

	// Enable the peripheral
	I2C->I2C_regs->I2C_CR1 |= I2C_CR1_ENABLE_BIT;

	return;
}
void I2C_init(uint8_t  I2C_peripheral,
              bool     ACK_enable,
              bool     stretching_enable,
              uint8_t  address,
              bool     fast_mode,
              uint8_t  APB_freq,
              uint16_t SCLK)
{
	memset(&I2C[I2C_peripheral], 0, sizeof(I2C_peripheral));
	I2C_initialization(I2C_peripheral,
	                   ACK_enable,
	                   stretching_enable,
	                   address,
	                   fast_mode,
	                   APB_freq,
	                   SCLK);

	return;
}

// change indicates if we have to change/swap the mode of the I2C
static void I2C_fix_change_working_mode(uint8_t I2C_peripheral, bool change)
{
	uint8_t address_dst = (I2C[I2C_peripheral].state % 0x100);
	if (change) {
		if (I2C[I2C_peripheral].state & I2C_STATE_SEND_DATA) {
			address_dst |= 1;
		} else {
			address_dst &= ~1;
		}
	} else {
		if (I2C[I2C_peripheral].state & I2C_STATE_SEND_DATA) {
			address_dst &= ~1;
		} else {
			address_dst |= 1;
		}
	}
	// Create the msj to set the address and the mode
	// Fix the direction of the element we want to communicate with
	//TODO: GUARDAR EL VALOR DEL ADDRESS EN EL BYTE DE MAYOR PESO.TAMBIEN ALMACENAR SI ENVIAMOS EN EL STATE
	I2C[I2C_peripheral].I2C_regs->I2C_DR |= address_dst;
	// Wait until receive an ACK and bit ADDR will set to 1
	while (!(I2C[I2C_peripheral].I2C_regs->I2C_SR1 & I2C_SR1_BTF_BIT))
		;

	// ERROR. No device with that address
	if (I2C[I2C_peripheral].I2C_regs->I2C_SR1 & I2C_SR1_AF_BIT)
		return;

	// While ADDR is 0, wait. THIS MUST BE 1 AT THIS POINT.
	while (!(I2C[I2C_peripheral].I2C_regs->I2C_SR1 & I2C_SR1_ADDR_BIT))
		;

	// If we only are interested in 1 byte recv
	if (I2C_CHECK_STATE_1_DATA_RECV(I2C_peripheral)) {
		I2C[I2C_peripheral].I2C_regs->I2C_CR1 &= ~I2C_CR1_ACK_BIT;
		I2C[I2C_peripheral].I2C_regs->I2C_CR1 |= I2C_CR1_STOP_BIT;
	}
	// clear the SR1 register. To do this read SR1 and SR2
	uint32_t aux = 0;
	aux          = I2C[I2C_peripheral].I2C_regs->I2C_SR1;
	aux          = I2C[I2C_peripheral].I2C_regs->I2C_SR2;

	// Now we are master and have the slave ready to send
	I2C[I2C_peripheral].state |= I2C_STATE_MASTER;
	return;
}

void I2C_set_master(uint8_t I2C_peripheral,
                    bool    enable_master,
                    uint8_t address_dst,
                    bool    send)
{
	uint32_t aux = 0;
	if (enable_master) {
		I2C[I2C_peripheral].I2C_regs->I2C_CR1 |= I2C_CR1_START_BIT;
		// Wait until the SB flag indicates that the
		while (!(I2C[I2C_peripheral].I2C_regs->I2C_SR1 & I2C_SR1_START_BIT))
			;
		// Read the SR1 register to clear SB bit at SR1
		aux = I2C[I2C_peripheral].I2C_regs->I2C_SR1;
	}

	//Save the address at the MSB del state
	address_dst *= 2;
	I2C[I2C_peripheral].state |= (0x100 * address_dst);
	(send) ? (I2C[I2C_peripheral].state |= I2C_STATE_SEND_DATA) :
	         (I2C[I2C_peripheral].state &= ~I2C_STATE_SEND_DATA);

	//TODO: EN CASO DE ERROR AL FIJAR EL ADDRESS, FINALIZAR.
	I2C_fix_change_working_mode(I2C_peripheral, false);
}

static I2C_enums I2C_send_data(uint8_t I2C_peripheral, uint8_t data_send)
{
	if (!(I2C[I2C_peripheral].I2C_regs->I2C_SR1 & I2C_SR1_TXE_BIT))
		return I2C_send_WAIT;
	I2C[I2C_peripheral].I2C_regs->I2C_DR = data_send;

	while ((!(I2C[I2C_peripheral].I2C_regs->I2C_SR1 & I2C_SR1_TXE_BIT)) ||
	       (!(I2C[I2C_peripheral].I2C_regs->I2C_SR1 & I2C_SR1_BTF_BIT)))
		;

	uint8_t aux = (I2C[I2C_peripheral].I2C_regs->I2C_SR1 & I2C_SR1_AF_BIT);
	return (aux) ? I2C_send_ERR : I2C_send_OK;
}

static I2C_enums I2C_receive_data(uint8_t  I2C_peripheral,
                                  uint8_t *data_recv/*,
                                  uint8_t  total_data_recv*/)
{
	// Start sending the total data length
	if (!(I2C[I2C_peripheral].I2C_regs->I2C_SR1 & I2C_SR1_RXNE_BIT))
		return I2C_recv_WAIT;
	*data_recv = I2C[I2C_peripheral].I2C_regs->I2C_DR;
	return I2C_recv_OK;
}

void I2C_exec()
{
	uint8_t aux = 0;
	for (uint8_t i = 0; i < I2C_TOTAL_PERIPHERALS; i++) {
		if (!(I2C[i].I2C_regs->I2C_CR1 & I2C_CR1_ENABLE_BIT))
			break;
		if (!I2C_CHECK_STATE_ACTIVE(i)) {
			if (I2C[i].I2C_regs->I2C_SR1 & I2C_SR1_ADDR_BIT) {
				I2C[i].state |= I2C_STATE_ACTIVE;
				// clear the SR1 register. To do this read SR1 and SR2
				aux = I2C[i].I2C_regs->I2C_SR1;
				aux = I2C[i].I2C_regs->I2C_SR2;
			}
		}
		if (I2C[i].total_data_send) {
			aux = I2C_send_data(i, I2C[i].buff_send[I2C[i].total_data_send]);
			if (aux == I2C_send_OK)
				I2C[i].total_data_send--;
			if ((aux == I2C_send_ERR) && (I2C_CHECK_STATE_MASTER(i))) {
				//Limpiar bit AF. No enviar mas
				I2C[i].I2C_regs->I2C_SR1 &= ~0x0400;
				I2C[i].total_data_send = 0;
			}

			// Ver como enviar primero el total de datos.
			if (!I2C[i].total_data_send) {
				if (I2C_CHECK_STATE_MASTER(i)) {
					if (I2C_CHECK_STATE_CHANGE_W_R(i)) {
						I2C_fix_change_working_mode(i, true);
					} else {
						//Activamos stop bit
						I2C[i].I2C_regs->I2C_SR1 |= 0x0200;
					}
					// PONER UN WHILE A LA ESPERA?
				} else {
					// Tal vez haya que esperar a que AF = 1
					I2C[i].I2C_regs->I2C_SR1 &= ~0x0400;
				}
			}
		}

		aux = I2C_receive_data(i, &I2C[i].buff_recv[I2C[i].total_data_recv]);
		if (aux == I2C_recv_OK) {
			if (!I2C[i].total_data_recv) {
				I2C[i].total_data_recv = I2C[i].buff_recv[0];
			} else if ((I2C[i].total_data_recv == 2) &&
			           (I2C_CHECK_STATE_MASTER(i))) {
				// VER COMO VOLVERLO A ACTIVAR
				I2C[i].I2C_regs->I2C_SR1 &= ~0x0400; //ACK
				I2C[i].I2C_regs->I2C_SR1 |= 0x0200;  //STOP
			} else {
				I2C[i].total_data_recv--;
			}

			if (!I2C[i].total_data_recv) {
				if ((I2C_CHECK_STATE_MASTER(i)) &&
				    (I2C_CHECK_STATE_CHANGE_W_R(i))) {
					// When we finish the reception, activate ACK if proceed it
					if (I2C_CHECK_STATE_ACK_ENABLE(i))
						I2C[i].I2C_regs->I2C_SR1 |= 0x0400; //ACK
					I2C_fix_change_working_mode(i, true);
				} else {
					while (!(I2C[i].I2C_regs->I2C_SR1 & 0x0010))
						;
					aux = I2C[i].I2C_regs->I2C_SR1;
					I2C[i].I2C_regs->I2C_SR1 |= 0x0200; //STOP
				}
			}
		}
	}
}

void I2C_send_data_dst(uint8_t I2C_periph, uint8_t *data, uint8_t data_len)
{
	memcpy(&I2C[I2C_periph].buff_send[1], data, data_len);
	/* Set the first element as the total data */
	I2C[I2C_periph].buff_send[0]    = data_len;
	I2C[I2C_periph].total_data_send = (data_len + 1);
	return;
}

//
bool I2C_get_data(uint8_t I2C_periph)
{
	//TODO: RETORNAR EL VALOR DEL BIT DE RECEPCION TOTAL
	return I2C[I2C_periph].state;
}

//TODO: FUNCION PARA OBTENER TODA EL ARRAY DE DATOS RECIBIDOS.
