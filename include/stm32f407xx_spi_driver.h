#ifndef STM32F407XX_SPI_DRIVER_H
#define STM32F407XX_SPI_DRIVER_H

#include <stdint.h>

/* SPI Configuration Macros */

#define SPI_MODE_SLAVE  (uint8_t) 0
#define SPI_MODE_MASTER (uint8_t) 1

#define SPI_CPOL_IDLE_LOW   (uint8_t) 0
#define SPI_CPOL_IDLE_HIGH  (uint8_t) 1

#define SPI_CPHA_FIRST_CLK_TRANSITION_CAPTURE  (uint8_t) 0
#define SPI_CPHA_FIRST_CLK_TRANSITION_SEND     (uint8_t) 1

#define SPI_DATA_FRAME_FORMAT_8    (uint8_t) 0
#define SPI_DATA_FRAME_FORMAT_16   (uint8_t) 1

#define SPI_SCK_BAUD_RATE_PRESCALER_2    (uint8_t) 0
#define SPI_SCK_BAUD_RATE_PRESCALER_4    (uint8_t) 1
#define SPI_SCK_BAUD_RATE_PRESCALER_8    (uint8_t) 2
#define SPI_SCK_BAUD_RATE_PRESCALER_16   (uint8_t) 3
#define SPI_SCK_BAUD_RATE_PRESCALER_32   (uint8_t) 4
#define SPI_SCK_BAUD_RATE_PRESCALER_64   (uint8_t) 5
#define SPI_SCK_BAUD_RATE_PRESCALER_128  (uint8_t) 6
#define SPI_SCK_BAUD_RATE_PRESCALER_256  (uint8_t) 7

#define SPI_SSM_DISABLE  (uint8_t) 0
#define SPI_SSM_ENABLE   (uint8_t) 1

#define SPI_SSI_DISABLE  (uint8_t) 0
#define SPI_SSI_ENABLE   (uint8_t) 1

/* In hardware slave select mode (SSI = 0), IN MASTER, this puts to ground
 * the exit of the NSS pin, allowing the comunication with the slave */
#define SPI_SSOE_DISABLE (uint8_t) 0
#define SPI_SSOE_ENABLE  (uint8_t) 1

#define SPI_RXONLY_FULL_DUPLEX    (uint8_t) 0
#define SPI_RXONLY_OUTPUT_DISABLE (uint8_t) 1

#define SPI_FULL_DUPLEX_MODE      (uint8_t) 0
#define SPI_HALF_DUPLEX_MODE      (uint8_t) 1
#define SPI_SIMPLE_CONECTION_MODE (uint8_t) 2

#define SPI_MSB_FIRST  (uint8_t) 0
#define SPI_LSB_FIRST  (uint8_t) 1

#define SPI1_PERIPHERAL	((uint8_t) 0)
#define SPI2_PERIPHERAL	((uint8_t) 1)
#define SPI3_PERIPHERAL	((uint8_t) 2)
#define SPI4_PERIPHERAL	((uint8_t) 3)
#define SPI5_PERIPHERAL	((uint8_t) 4)
#define SPI6_PERIPHERAL	((uint8_t) 5)

/* SPI mask configuration flags */
/* CR1 flag register */
#define SPI_CR1_MASK_CPHA			((uint16_t) 0x0001)
#define SPI_CR1_MASK_CPOL			((uint16_t) 0x0002)
#define SPI_CR1_MASK_MASTER			((uint16_t) 0x0004)
#define SPI_CR1_MASK_BAUD_RATE_CLK	((uint16_t) 0x0008)
#define SPI_CR1_MASK_SPE			((uint16_t) 0x0040)
#define SPI_CR1_MASK_LSB_FIRST		((uint16_t) 0x0080)
#define SPI_CR1_MASK_SSI			((uint16_t) 0x0100)
#define SPI_CR1_MASK_SSM			((uint16_t) 0x0200)
#define SPI_CR1_MASK_RX_ONLY		((uint16_t) 0x0400)
#define SPI_CR1_MASK_DFF			((uint16_t) 0x0800)
#define SPI_CR1_MASK_CRC_NEXT		((uint16_t) 0x1000)
#define SPI_CR1_MASK_CRC_EN			((uint16_t) 0x2000)
#define SPI_CR1_MASK_BIDIOE			((uint16_t) 0x4000)
#define SPI_CR1_MASK_BIDIMODE		((uint16_t) 0x8000)

/* CR2 flag register */
#define SPI_CR2_MASK_RXDMAEN		((uint16_t) 0x0001)
#define SPI_CR2_MASK_TXDMAEN		((uint16_t) 0x0002)
#define SPI_CR2_MASK_SSOE			((uint16_t) 0x0004)
#define SPI_CR2_MASK_FRF			((uint16_t) 0x0010)
#define SPI_CR2_MASK_ERRIE			((uint16_t) 0x0020)
#define SPI_CR2_MASK_RXNEIE			((uint16_t) 0x0040)
#define SPI_CR2_MASK_TXEIE			((uint16_t) 0x0080)

/* Status register flags */
#define SPI_SR_RXNE					((uint16_t) 0x0001)
#define SPI_SR_TXE					((uint16_t) 0x0002)
#define SPI_SR_CHSIDE				((uint16_t) 0x0004)
#define SPI_SR_UDR					((uint16_t) 0x0008)
#define SPI_SR_CRC_ERR				((uint16_t) 0x0010)
#define SPI_SR_MODF					((uint16_t) 0x0020)
#define SPI_SR_OVR					((uint16_t) 0x0040)
#define SPI_SR_BSY					((uint16_t) 0x0080)
#define SPI_SR_FRE					((uint16_t) 0x0100)

/* Function declarations */
/************************************************************************
 * @fn          - GPIO_Initialization
 *
 * @brief       - This function initializes the SPI selected
 *                peripheral with the desire values.
 *
 * @param[in]   - GPIO_port. Value of the desired port.
 * @param[in]   - GPIO_pin. Value of the desired pin inside the port.
 * @param[in]   - mode. Mode of the GPIO. Could be input or output.
 * @param[in]   - pull_up_down. Mode of the internal resistors.
 * @param[in]   - output_type. Indicates if the output is push-pull (two pull resistors) or drain-open (one resistor).
 * @param[in]   - output_speed. Speed of the output. Must see the data sheet to know the values.
 * @param[in]   - alternate_function. Identifier of the alternate pin functionality, if desired.
 * @param[in]   - IRQ_priority. Priority of the interrupt (in case of use it).
 *
 * @return      -
 */
void SPI_initialization_module();

/************************************************************************
 * @fn          - SPI_Initialization
 *
 * @brief       - This function initializes the SPI selected
 *                peripheral with the desire values.
 *
 * @param[in]   - SPI_peripheral. Value of the desired SPI peripheral.
 * @param[in]   - communication_mode. Used mode for the communication
 *                (full-dupex, half-duplex, simple-comm).
 * @param[in]   - mode. Mode of SPI peripheral configuration (master or slave).
 * @param[in]   - CPOL. Value of CPOL to define the idle state to 0 or 1.
 * @param[in]   - CPHA. Value of CPHA to indicate which clock transaction send or receive the data.
 * @param[in]   - DFF. Data Frame Format of the communication.
 * @param[in]   - BR_prescaler. Prescaler of the SCK/SCLK signal (only use in master mode).
 * @param[in]   - SSM. Software Slave Management (only in master mode).
 *
 * @return      -
 */
void SPI_Initialization(uint8_t SPI_peripheral, uint8_t communication_mode,
                        uint8_t mode, uint8_t CPOL, uint8_t CPHA,
                        uint8_t DFF, uint8_t BR_prescaler, uint8_t SSM);

/************************************************************************
 * @fn          - SPI_Reset
 *
 * @brief       - This function reset the SPI with the initial values.
 *
 * @param[in]   - SPI_peoipheral. Value of the desired peripheral.
 *
 * @return      -
 */
void SPI_Reset(uint8_t SPI_peripheral);

/************************************************************************
 * @fn          - SPI_clock_enable_disable
 *
 * @brief       - This function enables/disables the clock of the SPI.
 *
 * @param[in]   - SPI_peripheral. Value of the desired SPI peripheral.
 * @param[in]   - enable_disable. Flag to indicate if must enable or disable the clock.
 *
 * @return      -
 */
void SPI_clock_enable_disabled(uint8_t SPI_peripheral, uint8_t enable_disable);

/************************************************************************
 * @fn          - SPI_Send_Data
 *
 * @brief       - This function allow send data from the transmitter to the receiver.
 *
 * @param[in]   - SPI_peripheral. Value of the desired SPI peripheral.
 * @param[in]   - data. Pointer to the data to send.
 * @param[in]   - data_len. Length of the data to send.
 * @param[in]   - data_recv. Array to save the received data.
 *
 * @return      -
 */
void SPI_Send_Data(uint8_t SPI_peripheral, uint8_t* data, uint8_t data_len, uint8_t *data_recv);

#endif /* STM32F407XX_SPI_DRIVER_H */
