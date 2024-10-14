<<<<<<< Updated upstream
#ifndef STM32F407XX_GPIO_DRIVER_H
#define STM32F407XX_GPIO_DRIVER_H

#include "stm32f407xx.h"

#define GPIO_PORT_A ((uint8_t) 0)
#define GPIO_PORT_B ((uint8_t) 1)
#define GPIO_PORT_C ((uint8_t) 2)
#define GPIO_PORT_D ((uint8_t) 3)
#define GPIO_PORT_E ((uint8_t) 4)
#define GPIO_PORT_F ((uint8_t) 5)
#define GPIO_PORT_G ((uint8_t) 6)
#define GPIO_PORT_H ((uint8_t) 7)
#define GPIO_PORT_I ((uint8_t) 8)

#define GPIO_PIN_0   ((uint8_t) 0)
#define GPIO_PIN_1   ((uint8_t) 1)
#define GPIO_PIN_2   ((uint8_t) 2)
#define GPIO_PIN_3   ((uint8_t) 3)
#define GPIO_PIN_4   ((uint8_t) 4)
#define GPIO_PIN_5   ((uint8_t) 5)
#define GPIO_PIN_6   ((uint8_t) 6)
#define GPIO_PIN_7   ((uint8_t) 7)
#define GPIO_PIN_8   ((uint8_t) 8)
#define GPIO_PIN_9   ((uint8_t) 9)
#define GPIO_PIN_10  ((uint8_t) 10)
#define GPIO_PIN_11  ((uint8_t) 11)
#define GPIO_PIN_12  ((uint8_t) 12)
#define GPIO_PIN_13  ((uint8_t) 13)
#define GPIO_PIN_14  ((uint8_t) 14)
#define GPIO_PIN_15  ((uint8_t) 15)

#define GPIO_NON_ALTERNATE_FUNCTIONALITY  ((uint16_t) 0xFFFF)

#define GPIO_MODE_INPUT       ((uint8_t) 0)
#define GPIO_MODE_OUTPUT      ((uint8_t) 1)
#define GPIO_MODE_ALTERNATE   ((uint8_t) 2)
#define GPIO_MODE_ANALOG      ((uint8_t) 3)
#define GPIO_MODE_INT_RISING  ((uint8_t) 4)
#define GPIO_MODE_INT_FALLING ((uint8_t) 5)
#define GPIO_MODE_INT_RIS_FAL ((uint8_t) 6)

#define GPIO_NO_PULL_UP_DOWN  ((uint8_t) 0)
#define GPIO_PULL_UP          ((uint8_t) 1)
#define GPIO_PULL_DOWN        ((uint8_t) 2)
#define GPIO_PUPD_RESERVED    ((uint8_t) 3)

/* */
#define GPIO_OUTPUT_PUSH_PULL   ((uint8_t) 0)
#define GPIO_OUTPUT_OPEN_DRAIN  ((uint8_t) 1)

#define GPIO_SPEED_LOW        ((uint8_t) 0)
#define GPIO_SPEED_MEDIUM     ((uint8_t) 1)
#define GPIO_SPEED_HIGH       ((uint8_t) 2)
#define GPIO_SPEED_VERY_HIGH  ((uint8_t) 3)

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
=======
#ifndef STM32F407XX_GPIO_DRIVER_H
#define STM32F407XX_GPIO_DRIVER_H

#include "stm32f407xx.h"

#define GPIO_PORT_A ((uint8_t) 0)
#define GPIO_PORT_B ((uint8_t) 1)
#define GPIO_PORT_C ((uint8_t) 2)
#define GPIO_PORT_D ((uint8_t) 3)
#define GPIO_PORT_E ((uint8_t) 4)
#define GPIO_PORT_F ((uint8_t) 5)
#define GPIO_PORT_G ((uint8_t) 6)
#define GPIO_PORT_H ((uint8_t) 7)
#define GPIO_PORT_I ((uint8_t) 8)

#define GPIO_PIN_0   ((uint8_t) 0)
#define GPIO_PIN_1   ((uint8_t) 1)
#define GPIO_PIN_2   ((uint8_t) 2)
#define GPIO_PIN_3   ((uint8_t) 3)
#define GPIO_PIN_4   ((uint8_t) 4)
#define GPIO_PIN_5   ((uint8_t) 5)
#define GPIO_PIN_6   ((uint8_t) 6)
#define GPIO_PIN_7   ((uint8_t) 7)
#define GPIO_PIN_8   ((uint8_t) 8)
#define GPIO_PIN_9   ((uint8_t) 9)
#define GPIO_PIN_10  ((uint8_t) 10)
#define GPIO_PIN_11  ((uint8_t) 11)
#define GPIO_PIN_12  ((uint8_t) 12)
#define GPIO_PIN_13  ((uint8_t) 13)
#define GPIO_PIN_14  ((uint8_t) 14)
#define GPIO_PIN_15  ((uint8_t) 15)

#define GPIO_NON_ALTERNATE_FUNCTIONALITY	((uint16_t) 0xFFFF)
#define GPIO_NON_IRQ_PRIORITY				((uint8_t) 0x00)

#define GPIO_MODE_INPUT       ((uint8_t) 0)
#define GPIO_MODE_OUTPUT      ((uint8_t) 1)
#define GPIO_MODE_ALTERNATE   ((uint8_t) 2)
#define GPIO_MODE_ANALOG      ((uint8_t) 3)
#define GPIO_MODE_INT_RISING  ((uint8_t) 4)
#define GPIO_MODE_INT_FALLING ((uint8_t) 5)
#define GPIO_MODE_INT_RIS_FAL ((uint8_t) 6)

#define GPIO_NO_PULL_UP_DOWN  ((uint8_t) 0)
#define GPIO_PULL_UP          ((uint8_t) 1)
#define GPIO_PULL_DOWN        ((uint8_t) 2)
#define GPIO_PUPD_RESERVED    ((uint8_t) 3)

/* */
#define GPIO_OUTPUT_PUSH_PULL   ((uint8_t) 0)
#define GPIO_OUTPUT_OPEN_DRAIN  ((uint8_t) 1)

#define GPIO_SPEED_LOW        ((uint8_t) 0)
#define GPIO_SPEED_MEDIUM     ((uint8_t) 1)
#define GPIO_SPEED_HIGH       ((uint8_t) 2)
#define GPIO_SPEED_VERY_HIGH  ((uint8_t) 3)

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

/* Macros defined to make easier the programming of the different elements */
#define GPIO_activate_alternate_functionality(GPIO_Port, GPIO_port_pin, GPIO_alternate_functionality) \
		GPIO_Initialization(GPIO_Port, GPIO_port_pin, GPIO_MODE_ALTERNATE, GPIO_NO_PULL_UP_DOWN, \
							GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_LOW, GPIO_alternate_functionality, \
							GPIO_NON_IRQ_PRIORITY)

#define GPIO_initialization_input_mode(GPIO_Port, GPIO_port_pin, GPIO_Pull_Up_Down) \
		GPIO_Initialization(GPIO_Port, GPIO_port_pin, GPIO_MODE_INPUT, GPIO_Pull_Up_Down, \
							GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_LOW, GPIO_NON_ALTERNATE_FUNCTIONALITY, \
							GPIO_NON_IRQ_PRIORITY)


#endif /* STM32F407XX_GPIO_DRIVER_H */
>>>>>>> Stashed changes
