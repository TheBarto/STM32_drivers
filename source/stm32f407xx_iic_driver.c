#include "../include/stm32f407xx_iic_driver.h"

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

void I2C_initialization_module() {

  I2C_peripherals[I2C_PERIPHERAL_1] = (I2C_RegDef_t *)I2C1B_ADDR;
  I2C_peripherals[I2C_PERIPHERAL_2] = (I2C_RegDef_t *)I2C2B_ADDR;
  I2C_peripherals[I2C_PERIPHERAL_3] = (I2C_RegDef_t *)I2C3B_ADDR;

  return;
}

void I2C_initialization(uint8_t I2C_peripheral, bool ACK_enable,
                        bool stretching_enable, uint8_t address, bool fast_mode,
                        uint8_t APB_freq, uint16_t SCLK) {
  uint16_t aux_config = 0;

  // First thing is to enable the RCC clock for this peripheral
  switch (I2C_peripheral) {
  case I2C_PERIPHERAL_1:
    RCC->APB1ENR |= RCC_APB1_EN_RESET_I2C1;
    break;
  case I2C_PERIPHERAL_2:
    RCC->APB1ENR |= RCC_APB1_EN_RESET_I2C2;
    break;
  case I2C_PERIPHERAL_3:
    RCC->APB1ENR |= RCC_APB1_EN_RESET_I2C3;
    break;
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
  I2C_peripherals[I2C_peripheral]->I2C_CCR |= (aux_config * I2C_CCR_CCR_FIELD);

  // Config the TRISE register.
  aux_config = ((1 / APB_freq) * 1000000000);
  aux_config = (1000 / aux_config) + 1;
  I2C_peripherals[I2C_peripheral]->I2C_TRISE |=
      (aux_config * I2C_TRISE_TRISE_FIELD);

  // Enable the periphe
  I2C_peripherals[I2C_peripheral]->I2C_CR1 |= I2C_CR1_ENABLE_BIT;

  return;
}

void I2C_set_master(uint8_t I2C_peripheral, uint8_t address_dst,
                    bool send_recv) {
  I2C_peripherals[I2C_peripheral]->I2C_CR1 |= I2C_CR1_START_BIT;
  // Wait until the SB flag indicates that the
  while (!(I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_START_BIT))
    ;

  // Read the SR1 register to clear SR1
  uint32_t aux = I2C_peripherals[I2C_peripheral]->I2C_SR1;

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

void I2C_send_data(uint8_t I2C_peripheral, uint8_t *data_send,
                   uint8_t total_data_send) {

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
    } else if ((I2C_peripherals[I2C_peripheral]->I2C_SR2 & I2C_SR2_MST_BIT) &&
               (I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_AF_BIT)) {
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

void I2C_receive_data(uint8_t I2C_peripheral, uint8_t *data_recv,
                      uint8_t total_data_recv) {
  uint8_t aux_total_data_recv = 0;

  // Load the total data to receive
  while (!(I2C_peripherals[I2C_peripheral]->I2C_SR1 & 0x40))
    ;
  aux_total_data_recv = I2C_peripherals[I2C_peripheral]->I2C_DR;
  total_data_recv = aux_total_data_recv;

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
  return;
}

void I2C_release_master(uint8_t I2C_peripheral) {

  // First, check the TXE and BTF bits are set
  while (!((I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_CR1_STRETCH_BIT) ||
           (I2C_peripherals[I2C_peripheral]->I2C_SR1 & I2C_SR1_STOPF_BIT)))
    ;

  // After sending all data generate a stop condition.
  I2C_peripherals[I2C_peripheral]->I2C_CR1 |= I2C_CR1_STOP_BIT;

  return;
}
