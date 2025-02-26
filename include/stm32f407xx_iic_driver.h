#include "stm32f407xx.h"

#define I2C_PERIPHERAL_1 ((uint8_t)0)
#define I2C_PERIPHERAL_2 ((uint8_t)1)
#define I2C_PERIPHERAL_3 ((uint8_t)2)

#define I2C_ACK_ENABLE  1
#define I2C_ACK_DISABLE 0

#define I2C_STRETCH_ENABLE  1
#define I2C_STRETCH_DISABLE 0

#define I2C_STANDARD_MODE 0
#define I2C_FAST_MODE     1

/* Defines for the I2C_CR1 register */
#define I2C_CR1_SWRST_BIT      ((uint32_t)0x00008000)
#define I2C_CR1_POS_BIT        ((uint32_t)0x00000800)
#define I2C_CR1_ACK_BIT        ((uint32_t)0x00000400)
#define I2C_CR1_STOP_BIT       ((uint32_t)0x00000200)
#define I2C_CR1_START_BIT      ((uint32_t)0x00000100)
#define I2C_CR1_STRETCH_BIT    ((uint32_t)0x00000080)
#define I2C_CR1_SMBUS_MODE_BIT ((uint32_t)0x00000002)
#define I2C_CR1_ENABLE_BIT     ((uint32_t)0x00000001)

/* Defines for the I2C_CR2 register */
#define I2C_CR2_FREQ_FIELD ((uint32_t)0x0000003F)

/* Defines for the I2C_OAR1 register */
#define I2C_OAR1_ADD_MODE_BIT ((uint32_t)0x00008000)
#define I2C_OAR1_ALWAYS_1_BIT ((uint32_t)0x00004000)

/* Defines for the I2C_SR1 register */
#define I2C_SR1_START_BIT ((uint32_t)0x00000001)
#define I2C_SR1_ADDR_BIT  ((uint32_t)0x00000002)
#define I2C_SR1_BTF_BIT   ((uint32_t)0x00000004)
#define I2C_SR1_STOPF_BIT ((uint32_t)0x00000010)
#define I2C_SR1_RXNE_BIT  ((uint32_t)0x00000040)
#define I2C_SR1_TXE_BIT   ((uint32_t)0x00000080)
#define I2C_SR1_AF_BIT    ((uint32_t)0x00000400)

/* Defines for the I2C_SR2 register */
#define I2C_SR2_MST_BIT ((uint32_t)0x00000001)

#define I2C_CCR_FS_BIT    ((uint32_t)0x00008000)
#define I2C_CCR_DUTY_BIT  ((uint32_t)0x00004000)
#define I2C_CCR_CCR_FIELD ((uint32_t)0x00000FFF)

#define I2C_TRISE_TRISE_FIELD ((uint32_t)0x0000003F)

/************************************************************************
 * @fn          - I2C_initialization_module
 *
 * @brief       - Initiate the I2C module,
 *                setting the address of the I2C structures.
 *
 ************************************************************************/
void I2C_initialization_module();

/************************************************************************
 * @fn          - I2C_initialization
 *
 * @brief       - Initiate one I2C peripheral to be
 *                use at the communication process.
 *
 * @param[in]   - I2C_peripheral     Indicate the peripheral to be initalized.
 * @param[in]   - ACK_enable         True if want to use the ACKing feature.
 * @param[in]   - stretching_enable  True if want to use stretching feature.
 * @param[in]   - adress             Adress of the device used in the
 *communication.
 * @param[in]   - fast_mode          True if want to use fast communication
 *mode.
 * @param[in]   - APB_freq           Frecuency of the I2C peripheral bus.
 * @param[in]   - SCLK               Desired clock frecuency to be used
 *
 ************************************************************************/
void I2C_initialization(uint8_t  I2C_peripheral,
                        bool     ACK_enable,
                        bool     stretching_enable,
                        uint8_t  address,
                        bool     fast_mode,
                        uint8_t  APB_freq,
                        uint16_t SCLK);

/************************************************************************
 * @fn          - I2C_set_master
 *
 * @brief       - Set a I2C peripheral to master mode to send/receive data
 *                use at the communication process.
 *
 * @param[in]   - I2C_peripheral  Indicate the peripheral to be set as master.
 * @param[in]   - address_dst     Address of the target peripheral.
 * @param[in]   - send_recv       Boolean which indicate if we want send or
 *recive
 *
 ************************************************************************/
void I2C_set_master(uint8_t I2C_peripheral,
                    bool    enable_master,
                    uint8_t address_dst,
                    bool    send_recv);

/************************************************************************
 * @fn          - I2C_send_data
 *
 * @brief       - Send data using a I2C peripheral to a master or slave
 *                device. First the total data will be sent, later the
 *                desired data
 *
 * @param[in]   - I2C_peripheral  Indicate the peripheral to send data.
 * @param[in]   - data_send       Array pointer with the data to be sent.
 * @param[in]   - total_data_send Integer with the number of data to send.
 *
 ************************************************************************/
/*void I2C_send_data(uint8_t  I2C_peripheral,
                   uint8_t *data_send,
                   uint8_t  total_data_send);*/

/************************************************************************
 * @fn          - I2C_send_data
 *
 * @brief       - Send data using a I2C peripheral to a master or slave
 *                device. First the total data will be sent, later the
 *                desired data
 *
 * @param[in]   - I2C_peripheral  Indicate the peripheral to received.
 * @param[in]   - data_send       Array pointer with the data to be sent.
 * @param[in]   - total_data_send Integer with the number of data to send.
 *
 ************************************************************************/
/*void I2C_receive_data(uint8_t  I2C_peripheral,
                      uint8_t *data_recv,
                      uint8_t  total_data_recv);*/

/************************************************************************
 * @fn          - I2C_release_master
 *
 * @brief       - Release a peripheral of being master
 *
 * @param[in]   - I2C_peripheral  Indicate the peripheral to be released.
 *
 ************************************************************************/
void I2C_release_master(uint8_t I2C_peripheral);
