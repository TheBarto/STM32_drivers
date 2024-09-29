#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <stdio.h>

/* Ejercicio 1: utilizar el periférico SPI para enviar una cadena (Hello world)".
 * SPI-2 en modo maestro. El CLK será el máximo posible y con DFF a 0 y luego a 1

 * Para este ejercicio no emplearemos un esclavo (es raro), pero así probaremos el funcionamiento del maestro.
 * Al no emplear un esclavo, los pines MISO y NSS no se emplearán, con lo que podemos dejarlos fuera. Los que
 * si emplearemos serán los pines MOSI, para enviar, y SCLK.

 * El siguiente paso será buscar los pines físicos para poder emplear los pines MISO y SCLK. Para esto tenemos
 * que ir al datasheet del micro-controlador y ver que pines emplearemos. Para ello iremos a la tabla de:
 * "Alternate function mapping" (mapeo de funcion alterna). En estas tablas buscaremos los diferentes SPI, y
 * veremos que puerto emplear (A, B, C, D, etc) y dentro del puerto que pin.
 * HABRÁ QUE EMPLEAR FUNCIONES ALTERNATIVAS DE LOS PINES DEL PUERTO.


 * Tras mirar la tabla, tenemos que en el puerto B, existen pines GPIO con funcionalidad alternativa = 5 que podemos
 * emplear para el periférico SPI:

 *		PB9 -> NSS
 *		PB10 -> SCLK
 *		PB14 -> MISO
 *		PB15 -> MOSI
 */


int main()
{
	/* Como primer paso, inicializamos estos pines con su funcionalidad alternativa.
	 * PARA ELLO ES NECESARIO EMPLEAR A LIBRERIA DE GPIO, YA QUE HAY QUE EMPLEAR
	 * CONFIGURAR LA FUNCIONALIDAD ALTERNATIVA DE ESTOS PINES.
	 */

	/* Inicializamos el modulo GPIO, y procedemos a configurar los diferentes pines que emplearemos. */
	GPIO_initialization_module();
	/* IMPORTANTE, ACTIVAR EL RELOJ DEL GPIO ANTES DE SU CONFIGURACION, O NO FUNCIONARÁ */
	GPIO_clock_enable_disable(GPIO_PORT_B, 1);
	/* PB-15 como SCLK */
	GPIO_Initialization(GPIO_PORT_B, GPIO_PIN_10, GPIO_MODE_ALTERNATE,
	                    GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL,
	                    GPIO_SPEED_LOW, 5, 0);

	/* PB-10 como MOSI */
	GPIO_Initialization(GPIO_PORT_B, GPIO_PIN_15, GPIO_MODE_ALTERNATE,
	                    GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL,
	                    GPIO_SPEED_LOW, 5, 0);

	/* Ahora debemos inicializar el periferico SPI y cnfigurarlo como deseemos. */
	SPI_initialization_module();
	/* IMPORTANTE, ACTIVAR EL RELOJ DEL SPI ANTES DE SU CONFIGURACION, O NO FUNCIONARÁ */
	SPI_clock_enable_disabled(SPI2_PERIPHERAL, 1);

	/* Initialization of SPI2 */
	SPI_Initialization(SPI2_PERIPHERAL, SPI_FULL_DUPLEX_MODE, SPI_MODE_MASTER,
					   SPI_CPOL_IDLE_LOW, SPI_CPHA_FIRST_CLK_TRANSITION_CAPTURE,
					   SPI_DATA_FRAME_FORMAT_8, SPI_SCK_BAUD_RATE_PRESCALER_2,
					   SPI_SSM_ENABLE);

	char data_send[] = "Hello World";
	char data_recv[20];
	SPI_Send_Data(SPI2_PERIPHERAL, (uint8_t *)data_send, 12, (uint8_t *)data_recv);

	while(1);

	return 0;
}
