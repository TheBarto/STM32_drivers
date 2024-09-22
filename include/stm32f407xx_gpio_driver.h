#ifndef STM32F407XX_GPIO_DRIVER_H
#define STM32F407XX_GPIO_DRIVER_H

#include "stm32f407xx.h"

#define TOTAL_STM32F407_GPIO ((uint8_t) 8)

void GPIO_initialization_module();

/************************************************************************
 * @fn          - GPIO_Initialization
 *
 * @brief       - This function initializes the GPIO with the desire values. 
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
void GPIO_Initialization(uint8_t GPIO_port, uint8_t GPIO_pin, uint8_t mode,
                         uint8_t pull_up_down, uint8_t output_type,
                         uint8_t output_speed, uint16_t alternate_function,
                         uint8_t IRQ_priority);

/************************************************************************
 * @fn          - GPIO_Reset
 *
 * @brief       - This function reset the GPIO with the initial values. 
 *
 * @param[in]   - GPIO_port. Value of the desired port.
 *
 * @return      -
 */
void GPIO_Reset(uint8_t GPIO_port);

/************************************************************************
 * @fn          - GPIO_clock_enable_disable
 *
 * @brief       - This function enables/disables the clock of the GPIO. 
 *
 * @param[in]   - GPIO_port. Value of the desired port.
 * @param[in]   - enable_disable. Flag to indicate if must enable or disable the clock.
 *
 * @return      -
 */
void GPIO_clock_enable_disable(uint8_t GPIO_port, uint8_t enable_disable);

/************************************************************************
 * @fn          - GPIO_read_pin
 *
 * @brief       - Reads a certain pin of the desired GPIO.
 *
 * @param[in]   - GPIO_port. Value of the desired port.
 * @param[in]   - GPIO_pin. Value of the desired pin inside the port.
 *
 * @return      -
 */
uint8_t GPIO_read_pin(uint8_t GPIO_port, uint8_t GPIO_pin);

/************************************************************************
 * @fn          - GPIO_write_pin
 *
 * @brief       - Writes a certain pin of the desired GPIO.
 *
 * @param[in]   - GPIO_port. Value of the desired port.
 * @param[in]   - GPIO_pin.  Value of the desired pin inside the port.
 * @param[in]   - pin_value. Value to write into the pin.
 *
 * @return      -
 */
void GPIO_write_pin(uint8_t GPIO_port, uint8_t GPIO_pin, uint8_t pin_value);

/************************************************************************
 * @fn          - GPIO_read_port
 *
 * @brief       - Reads a certain port of the desired GPIO.
 *
 * @param[in]   - GPIO_port. Value of the desired port.
 *
 * @return      -
 */
uint16_t GPIO_read_port(uint8_t GPIO_port);

/************************************************************************
 * @fn          - GPIO_write_port
 *
 * @brief       - Writes a certain port of the desired GPIO.
 *
 * @param[in]   - GPIO_port. Value of the desired port.
 * @param[in]   - port_value. Value to write into the port.
 *
 * @return      -
 */
void GPIO_write_port(uint8_t GPIO_port, uint16_t port_value);

/************************************************************************
 * @fn          - GPIO_alternate_funtionality
 *
 * @brief       - Configure the alternate funtionality of a certain GPIO's pin.
 *
 * @param[in]   - gpio. Pointer to the handling struct of the GPIO.
 * @param[in]   - mode. Mode of the GPIO. Could be input or output.
 *
 * @return      -
 */
/*void GPIO_alternate_funtionality(GPIO_RegDef_t* gpio, uint8_t pin, uint8_t alternate_function);*/

/************************************************************************
 * @fn          - GPIO_IRQ_configure
 *
 * @brief       - Configure all the interrupt handling of a GPIO's pin.
 *
 * @param[in]   - gpio. Pointer to the handling struct of the GPIO.
 * @param[in]   - mode. Mode of the GPIO. Could be input or output.
 *
 * @return      -
 */
void GPIO_IRQ_configure(uint8_t GPIO_port, uint8_t GPIO_pin,
                        uint8_t IRQ_priority, uint8_t mode);

void GPIO_IRQ_handling(uint8_t GPIO_pin);

#endif /* STM32F407XX_GPIO_DRIVER_H */
