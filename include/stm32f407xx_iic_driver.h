#include "stm32f407xx.h"

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
void I2C_initialization(uint8_t I2C_peripheral, bool ACK_enable,
                        bool stretching_enable, uint8_t address, bool fast_mode,
                        uint8_t APB_freq, uint16_t SCLK);

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
void I2C_set_master(uint8_t I2C_peripheral, uint8_t address_dst,
                    bool send_recv);

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
void I2C_send_data(uint8_t I2C_peripheral, uint8_t *data_send,
                   uint8_t total_data_send);

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
void I2C_receive_data(uint8_t I2C_peripheral, uint8_t *data_recv,
                      uint8_t total_data_recv);

/************************************************************************
 * @fn          - I2C_release_master
 *
 * @brief       - Release a peripheral of being master
 *
 * @param[in]   - I2C_peripheral  Indicate the peripheral to be released.
 *
 ************************************************************************/
void I2C_release_master(uint8_t I2C_peripheral);
