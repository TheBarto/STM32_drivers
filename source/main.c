#if 0
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <stdio.h>
#include <string.h>

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
void ejercicio1()
{
	/* Como primer paso, inicializamos estos pines con su funcionalidad alternativa.
	 * PARA ELLO ES NECESARIO EMPLEAR A LIBRERIA DE GPIO, YA QUE HAY QUE EMPLEAR
	 * CONFIGURAR LA FUNCIONALIDAD ALTERNATIVA DE ESTOS PINES.
	 */

	/* Inicializamos el modulo GPIO, y procedemos a configurar los diferentes pines que emplearemos. */
	GPIO_initialization_module();

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


	/* Initialization of SPI2 */
	SPI_Initialization(SPI2_PERIPHERAL, SPI_FULL_DUPLEX_MODE, SPI_MODE_MASTER,
					   SPI_CPOL_IDLE_LOW, SPI_CPHA_FIRST_CLK_TRANSITION_CAPTURE,
					   SPI_DATA_FRAME_FORMAT_8, SPI_SCK_BAUD_RATE_PRESCALER_2,
					   SPI_SSM_ENABLE);

	char data_send[] = "Hello World";
	/* Before sending any data, enable the SPI Peripheral to start the data sending.
	 * With this we will configure the SSI or SSOE bit and after enable the peripheral
	 * (SPE bit).*/
	enable_disable_SPI_peripheral(SPI2_PERIPHERAL, true);
	SPI_Send_Data(SPI2_PERIPHERAL, (uint8_t *)data_send, strlen(data_send));

	while(1);
}

/* In this exercise we'll use two boards. The master board will be the STM32 and the slave will be the Arduino Uno.
 *
 * When the master's button is pressed, master will send a string of data to the slave connected. The data received
 * at the Arduino board will be displayed on the serial port.

 * The configuration will be:
 * - Full duplex communication mode.
 * - STM32 in master mode and Arduino will be in slave mode.
 * - Use DFF = 0
 * - Hardware Slave Management
 * - SCLK speed = 2MHz, fclk = 16MHz

 * Master WILL NOT RECEIVE NOTHING, so if we don't have to configure MISO. ONLY TRANSMIT.

 * NOTE: Slave peripheral does not know how many charaters will receive. Send this value first.
 */
//SECCION_41_MINUTO_28_TENEMOS LA PRUEBA. PROBARLO.
void ejercicio2()
{
	//FALTA INICIALIZAR TODOS LOS PINES GPIO COMO ALTERNOS.
	/* Initiation of GPIO module to load the button configuration */
	GPIO_initialization_module();

	/* PB-10 como SCLK */
	GPIO_activate_alternate_functionality(GPIO_PORT_B, GPIO_PIN_10, 5);

	/* PB-15 como MOSI */
	GPIO_activate_alternate_functionality(GPIO_PORT_B, GPIO_PIN_15, 5);

	/* PB-9 como NSS */
	GPIO_activate_alternate_functionality(GPIO_PORT_B, GPIO_PIN_9, 5);

	/* PB-14 como MISO */
	GPIO_activate_alternate_functionality(GPIO_PORT_B, GPIO_PIN_14, 5);

	/* Ahora debemos inicializar el periferico SPI y cnfigurarlo como deseemos. */
	SPI_initialization_module();

	/* Initialization of SPI2 */
	SPI_Initialization(SPI2_PERIPHERAL, SPI_FULL_DUPLEX_MODE, SPI_MODE_MASTER,
					   SPI_CPOL_IDLE_LOW, SPI_CPHA_FIRST_CLK_TRANSITION_CAPTURE,
					   SPI_DATA_FRAME_FORMAT_8, SPI_SCK_BAUD_RATE_PRESCALER_8,
					   SPI_SSM_DISABLE);

	/* Before sending any data, enable the SPI Peripheral to start the data sending.
	 * With this we will configure the SSI or SSOE bit and after enable the peripheral
	 * (SPE bit).*/
	enable_disable_SPI_peripheral(SPI2_PERIPHERAL, true);

	/* Initiation of pin 10 of GPIO PORT A */
	//GPIO_initialization_input_mode
	GPIO_Initialization(GPIO_PORT_A, GPIO_PIN_0, GPIO_MODE_INPUT,
						GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL,
						GPIO_SPEED_MEDIUM, GPIO_NON_ALTERNATE_FUNCTIONALITY,
	                    0);

	while(1) {
		/* Read the status of the pin 0, and when detect a 1
		 * (VDD voltage in that pin), send the data string.
		 */
		while(!GPIO_read_pin(GPIO_PORT_A, GPIO_PIN_0));

		char data_send[] = "Hello World";
		uint8_t data_len = strlen(data_send);
		//First of all, let's send the total of the information.
		SPI_Send_Data(SPI2_PERIPHERAL, &data_len, 1);

		//Second, send the data
		SPI_Send_Data(SPI2_PERIPHERAL, (uint8_t *)data_send, strlen(data_send));
	}

}

//SECCION43. PROBARLO.
/* In this exercise, we will press the master button to start the transmission of data to the slave.
 * When this happens, the slave will respond to each information sent. In this exercise, master WILL
 * RECEIVE DATA, so it's mandatory to connect the MISO line in both sites.
 *
 * The configuration will be:
 * 	- Full duplex mode.
 * 	- SPI Board will be master and Arduino will be slave.
 *  - DFF = 0
 *	- Hardware Slave Management
 *	- SCLK speed = 2MHz, fclk = 16MHz.
 *
 *	At this exercise, we will send commands to the Arduino Board. This commands will do some actions
 *	at the Arduino board.
 *
 *	The commands are:
 *
 *	- CMD_LED_CTRL	<pin_number> <value>
 *		- <pin_number>: digital pin number of the Arduino Board.
 *		- <value>: 1 = ON, 0 = OFF
 *		Slave action: control a digital pin to turn on/off a LED
 *		Slave returns: None
 *
 *	- CMD_SENSOR_READ <analog pin number>
 *		- <analog pin number>: Analog pin number of the Arduino board (A0 -> A5). 1 byte.
 *		Slave action: slave should read the analog value from the pin. (let's suppose there is a sensor connected).
 *		Slave returns: 1 byte with the value readed.
 *
 *	- CMD_LED_READ <pin_number>
 *		- <pin_number>: digital pin number of the Arduino Board (0 to 9).
 *		Slave action: read the status of the supplied pin number.
 *		Slave returns: 1 byte with the status. 1 = ON, 0 = OFF.
 *
 *	- CMD_PRINT	<len> <message>
 *		- <len>: 1 byte with the len of the message to print.
 *		- <message>: message to print with <len> bytes.
 *		Slave action: prints the received message into the serial port.
 *		Slave returns: nothing.
 *
 *	- CMD_ID_READ
 *		Slave returns: 10 bytes of the slave board id string.
 *
 *	The master must send all the data in two times. The first one must send the command, only the command. Then, the
 *	slave will responde with a ACK(0xF5) or NACK(0xA5) byte value. If we receive a NACK value, we print/display a
 *	error message. If ACK byte is received, then we must send all the arguments to the slave.
 */

uint8_t commands[] = {/*"CMD_LED_CTRL"*/0x50, /*"CMD_SENSOR_READ"*/0x51, /*"CMD_LED_READ"*/0x52, /*"CMD_PRINT"*/0x53, /*"CMD_ID_READ"*/0x54};
void ejercicio3_1()
{

	uint8_t data_send[10];
	uint8_t data_recv[10];
	uint8_t dummy_data = 0xFF;
	uint8_t dummy_read = 0xFF;

	/* Wait until STM button is press. */
	while(!GPIO_read_pin(GPIO_PORT_A, GPIO_PIN_0));

	// Send the first command. Then wait to the ACK/NACK. Then send the argumments.
	SPI_Send_Data(SPI2_PERIPHERAL, &commands[0], 1);

	/* IMPORTANT: after every data send, you must read the data received from the slave.
	 * In full-duplex mode, every time the master send data, the slave also send data
	 * which are receive in the master side. This data, if we don't expect nothing, is
	 * garbage data, but this way we reset the RXNE (receive bit). Every time we sent
	 * data, we should read the DR buffer to reset the bit (this means, if we sent 10
	 * bytes, receive 10 garbage bytes
	 */
	SPI_Receive_Data(SPI2_PERIPHERAL, &dummy_read);

	/* IMPORTANT, now the slave has received the command, process it and has to send
	 * to the master the response. To receive the response, the master has to send
	 * some dummy data, otherwise it won't receive the ACK/NACK from the slave.
	 *
	 * This ACK/NACK response is now at the SHIFT BUFFER at the slave device. To fetch it
	 * the master must send something, activating the exchange data process. Otherwise we
	 * won't get the response, because is the master the one who activate the whole process.
	 */
	SPI_Send_Data(SPI2_PERIPHERAL, &dummy_data, 1);
	/* The receive data is received at the shift buffer of the master, and will passed
	 * into the RX_buffer. So we have to check if the RX bit is correct, and read the response
	 * from the DR register.
	 */
	SPI_Receive_Data(SPI2_PERIPHERAL, data_recv);

	if(*data_recv == 0xA5)
		return;

	data_send[0] = 4; //Pin number 4
	data_send[1] = 1; //Turn on the pin
	SPI_Send_Data(SPI2_PERIPHERAL, data_send, 2);

	while(!GPIO_read_pin(GPIO_PORT_A, GPIO_PIN_0));

	// Send the first command. Then wait to the ACK/NACK. Then send the argumments.
	SPI_Send_Data(SPI2_PERIPHERAL, &commands[1], 1);
	SPI_Receive_Data(SPI2_PERIPHERAL, &dummy_read);

	SPI_Send_Data(SPI2_PERIPHERAL, &dummy_data, 1);
	SPI_Receive_Data(SPI2_PERIPHERAL, data_recv);

	if(*data_recv == 0xA5)
		return;

	data_send[0] = 3; //Pin A3
	SPI_Send_Data(SPI2_PERIPHERAL, data_send, 1);
	SPI_Receive_Data(SPI2_PERIPHERAL, &dummy_read);

	/* With this send dummy data, we receive the correct data.
	 * This step can require some time, due to the conversion
	 * of the ADC at the Arduino Board. Insert a delete is man-
	 * datory to receive correctly the data. */
	//delay();
	SPI_Send_Data(SPI2_PERIPHERAL, &dummy_data, 1);
	SPI_Receive_Data(SPI2_PERIPHERAL, data_recv);

	while(!GPIO_read_pin(GPIO_PORT_A, GPIO_PIN_0));

	// Send the first command. Then wait to the ACK/NACK. Then send the argumments.
	SPI_Send_Data(SPI2_PERIPHERAL, &commands[2], 1);
	SPI_Receive_Data(SPI2_PERIPHERAL, &dummy_read);

	SPI_Send_Data(SPI2_PERIPHERAL, &dummy_data, 1);
	SPI_Receive_Data(SPI2_PERIPHERAL, data_recv);

	if(*data_recv == 0xA5)
		return;

	data_send[0] = 4; //Pin number 4
	SPI_Send_Data(SPI2_PERIPHERAL, data_send, 1);
	SPI_Receive_Data(SPI2_PERIPHERAL, data_recv);

	/* To get the response it's mandatory to send
	 * something and activate the receive mechanism. */
	SPI_Send_Data(SPI2_PERIPHERAL, &dummy_data, 1);
	SPI_Receive_Data(SPI2_PERIPHERAL, data_recv);

}

void ejercicio3()
{

	/* First, let's initiate all the peripherals and configure them as we need. */
	SPI_initialization_module();

	//Configure the SPI_Peripheral.
	SPI_Initialization(SPI2_PERIPHERAL, SPI_FULL_DUPLEX_MODE, SPI_MODE_MASTER,
					   SPI_CPOL_IDLE_LOW, SPI_CPHA_FIRST_CLK_TRANSITION_CAPTURE,
					   SPI_DATA_FRAME_FORMAT_8, SPI_SCK_BAUD_RATE_PRESCALER_8,
					   SPI_SSM_DISABLE);

	/* Initiation of GPIO module to load the button configuration */
	GPIO_initialization_module();

	/* Initiation of pin 0 of GPIO PORT A. The button to send commands. */
	GPIO_initialization_input_mode(GPIO_PORT_A, GPIO_PIN_0, GPIO_NO_PULL_UP_DOWN);

	/* Now configure all the GPIOs port with the alternate functionality. */
	/* PB-15 como SCLK */
	GPIO_activate_alternate_functionality(GPIO_PORT_B, GPIO_PIN_10, 5);

	/* PB-14 como MISO */
	GPIO_activate_alternate_functionality(GPIO_PORT_B, GPIO_PIN_14, 5);

	/* PB-10 como MOSI */
	GPIO_activate_alternate_functionality(GPIO_PORT_B, GPIO_PIN_15, 5);

	/* PB-9 como NSS */
	GPIO_activate_alternate_functionality(GPIO_PORT_B, GPIO_PIN_9, 5);

	enable_disable_SPI_peripheral(SPI2_PERIPHERAL, true);
}


/* This part is used to check the interrupt part of the SPI peripheral. For that purpose, we are
 * going to use the function associated to the interrupt. Just must look which one is and fill it
 * with the proper instructions.
 *
 * Comment: the moment we activate the interrupts, the transmission will produce one. So we have to
 * be ready to send the data immediately, otherwise we can not send the data and lose the opportunity
 * of sending.
 */
void SPI2_IRQHandler()
{

	//SPI_


}

int main()
{
	ejercicio2();

	return 0;
}
#endif
