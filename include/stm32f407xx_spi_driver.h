#ifndef STM32F407XX_SPI_DRIVER_H
#define STM32F407XX_SPI_DRIVER_H

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


#endif /* STM32F407XX_SPI_DRIVER_H */
