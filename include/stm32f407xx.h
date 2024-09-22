#ifndef STM32F407_XX_H
#define STM32F407_XX_H

#include <stdint.h>

/* Posibilidad de añadir un prefijo a las macros como DRV_ o
 * algo asi para saber a que capa pertenece la macro */

/* Direcciones base de la FLASH y las SRAM. 
 * Ambos datos pueden obtenerse del datasheet, de la seccion "Memory mapping" */
#define FLASHB_ADDR (uint32_t) 0x08000000  
#define SRAM1B_ADRR (uint32_t) 0x20000000
#define SRAM2B_ADDR (uint32_t) 0x2001C000
/* Tambien es conoce como "system memory" */
#define ROMB_ADDR   (uint32_t) 0x1FFF0000

/* Definimos las direcciones de los distintos dominios de bus */
/* Estas las podemos obtener de la seccion "memory map" del manual de refencia */
#define APB1B_ADDR  (uint32_t) 0x40000000
#define APB1_LENGTH (uint32_t) 0x00008000

#define APB2B_ADDR  (uint32_t) 0x40010000
#define APB2_LENGTH (uint32_t) 0x00006C00

#define AHB1B_ADDR  (uint32_t) 0x40020000
#define AHB1_LENGTH (uint32_t) 0x00060000

#define AHB2B_ADDR  (uint32_t) 0x50000000
#define AHB2_LENGTH (uint32_t) 0x00060C00

#define AHB3B_ADDR  (uint32_t) 0xA0000000
#define AHB3_LENGTH (uint32_t) 0x00001000

/* Para cada elemento que cuelga de los perifericos, anotamos sus direcciones base. */
/* AHB1 - GPIOs */
#define GPIOAB_ADDR   (uint32_t) 0x40020000
#define GPIOBB_ADDR   (uint32_t) 0x40020400
#define GPIOCB_ADDR   (uint32_t) 0x40020800
#define GPIODB_ADDR   (uint32_t) 0x40020C00
#define GPIOEB_ADDR   (uint32_t) 0x40021000
#define GPIOFB_ADDR   (uint32_t) 0x40021400
#define GPIOGB_ADDR   (uint32_t) 0x40021800
#define GPIOHB_ADDR   (uint32_t) 0x40021C00
#define GPIOIB_ADDR   (uint32_t) 0x40022000
#define GPIOJB_ADDR   (uint32_t) 0x40022400
#define GPIOKB_ADDR   (uint32_t) 0x40022800
#define RCCB_ADDR     (uint32_t) 0x40023800
/* APB1 -I2Cx, SPIx, USARTx, UARTx */
#define I2C1B_ADDR    (uint32_t) 0x40005400
#define I2C2B_ADDR    (uint32_t) 0x40005800
#define I2C3B_ADDR    (uint32_t) 0x40005C00
#define SPI2B_ADDR    (uint32_t) 0x40003800
#define SPI3B_ADDR    (uint32_t) 0x40003C00
#define USART2B_ADDR  (uint32_t) 0x40004400
#define USART3B_ADDR  (uint32_t) 0x40004800
#define UART4B_ADDR   (uint32_t) 0x40004C00
#define UART5B_ADDR   (uint32_t) 0x40005000

/* APB2 - SPI1, USART1, USART6, EXTI, SYSCFG */
#define SPIB_ADDR     (uint32_t) 0x40013000
#define USART1B_ADDR  (uint32_t) 0x40011000
#define USART6B_ADDR  (uint32_t) 0x40011400
#define EXTIB_ADDR    (uint32_t) 0x40013C00
#define SYSCFGB_ADDR  (uint32_t) 0x40013800

/* GPIO Registers Base Address */
#define GPIO_PORT_MODE_REG_BADDR                (uint32_t) 0x00000000
#define GPIO_PORT_OUTPUT_TYPE_REG_B_ADDR        (uint32_t) 0x00000004
#define GPIO_PORT_OUTPUT_SPEED_REG_B_ADDR       (uint32_t) 0x00000008
#define GPIO_PORT_PULL_UP_PULL_DOWN_REG_B_ADDR  (uint32_t) 0x0000000C
#define GPIO_PORT_INPUT_DATA_REG_B_ADDR         (uint32_t) 0x00000010
#define GPIO_PORT_OUTPUT_DATA_REG_B_ADDR        (uint32_t) 0x00000014
#define GPIO_PORT_BIT_SET_GET_REG_B_ADDR        (uint32_t) 0x00000018
#define GPIO_PORT_CONFIG_LOCK_REG_B_ADDR        (uint32_t) 0x0000001C
#define GPIO_PORT_ALTERNATE_LOW_REG_B_ADDR      (uint32_t) 0x00000020
#define GPIO_PORT_ALTERNATE_HIGH_REG_B_ADDR     (uint32_t) 0x00000020

/* Serial Peripheral Interface registers base address (SPI) */
#define SPI_CONTROL_REG1_B_ADDR         (uint32_t) 0x00000000
#define SPI_CONTROL_REG2_B_ADDR         (uint32_t) 0x00000004
#define SPI_STATUS_REG_B_ADDR           (uint32_t) 0x00000008
#define SPI_DATA_REG_B_ADDR             (uint32_t) 0x0000000C
#define SPI_CRC_POLYNOMIAL_REG_B_ADDR   (uint32_t) 0x00000010
#define SPI_RX_CRC_REG_B_ADDR           (uint32_t) 0x00000014
#define SPI_TX_CRC_REG_B_ADDR           (uint32_t) 0x00000018
#define SPI_CONFIG_REG_B_ADDR           (uint32_t) 0x0000001C
#define SPI_PRESCALAR_REG_B_ADDR        (uint32_t) 0x00000020

/* Para acceder a todos los registros de un un periférico, en vez de definir todas las direcciones para los diferentes periféricos, podemos generar una estructura que contenga varios elementos/variables (que hagan referencia a los registros del periférico). Luego simplemente cogeremos esa estructura como un puntero, y le pondremos la dirección base del periférico. De esta manera podremos acceder facilmente a los periféricos sin necesidad de estar generando todas las direcciones una a una. 

Es muy importante que las variables que empleemos TENGAN EL MISMO TAMAÑO QUE LOS REGISTROS, YA QUE SINO GENERAREMOS UN DESCUADRE EN LOS ACCESOS Y LAS ASIGNACIONES.
*/

/* Consejo, añadir una breve descripción de la finalidad de cada variable. De este modo se sabrá para que vale cada uno */
/* Aunque NO SON NECESARIOS EN TODOS LOS CAMPOS, al hacer referencia a valores que pueden modificarse en cualquier momento y sin necesidad de hacer nada nosotros, es muy recomendable incluir el modificador volatile en todos los campos; o al menos en los de entrada de valores. */
typedef struct {
  volatile uint32_t MODER;     // GPIO port mode register
  volatile uint32_t OTYPER;    // GPIO port output type register
  volatile uint32_t OSPEEDR;   // GPIO port output speed register
  volatile uint32_t PUPDR;     // GPIO port pull-up/pull-down register
  volatile uint32_t IDR;       // GPIO port input data register
  volatile uint32_t ODR;       // GPIO port output data register
  volatile uint32_t BSRR;      // GPIO port bit set/reset register
  volatile uint32_t LCKR;      // GPIO port configuration lock register
  volatile uint32_t AFR[2];    // GPIO alternate function. 0 -> low / 1 -> high
} GPIO_RegDef_t;

typedef struct {
  volatile uint32_t CR;          // RCC clock control register
  volatile uint32_t PLLCFGR;     // RCC PLL configuration register
  volatile uint32_t CFGR;        // RCC clock configuration register
  volatile uint32_t CIR;         // RCC clock interrupt register
  volatile uint32_t AHB1RSTR;    // RCC AHB1 peripheral reset register
  volatile uint32_t AHB2RSTR;    // RCC AHB2 peripheral reset register
  volatile uint32_t AHB3RSTR;    // RCC AHB3 peripheral reset register
  volatile uint32_t reserved_1;
  volatile uint32_t APB1RSTR;    // RCC APB1 peripheral reset register
  volatile uint32_t APB2RSTR;    // RCC APB2 peripheral reset register
  volatile uint32_t reserved_2;
  volatile uint32_t reserved_3;
  volatile uint32_t AHB1ENR;     // RCC AHB1 peripheral clock enable register
  volatile uint32_t AHB2ENR;     // RCC AHB2 peripheral clock enable register
  volatile uint32_t AHB3ENR;     // RCC AHB3 peripheral clock enable register
  volatile uint32_t reserved_4;
  volatile uint32_t APB1ENR;     // RCC APB1 peripheral clock enable register
  volatile uint32_t APB2ENR;     // RCC APB2 peripheral clock enable register
  volatile uint32_t reserved_5;
  volatile uint32_t reserved_6;
  volatile uint32_t AHB1LPENR;   // RCC AHB1 peripheral clock enable in low power mode register
  volatile uint32_t AHB2LPENR;   // RCC AHB2 peripheral clock enable in low power mode register
  volatile uint32_t AHB3LPENR;   // RCC AHB3 peripheral clock enable in low power mode register
  volatile uint32_t reserved_7;
  volatile uint32_t APB1LPENR;   // RCC APB1 peripheral clock enable in low power mode register
  volatile uint32_t APB2LPENR;   // RCC APB2 peripheral clock enable in low power mode register
  volatile uint32_t reserved_8;
  volatile uint32_t reserved_9;
  volatile uint32_t BDCR;        // RCC Backup domain control register
  volatile uint32_t CSR;         // RCC clock control & status register
  volatile uint32_t reserved_10;
  volatile uint32_t reserved_11;
  volatile uint32_t SSCGR;       // RCC spread spectrum clock generation register
  volatile uint32_t PLLI2SCFGR;  // RCC PLLI2S configuration register
} RCC_RegDef_t;

// EXTI -> EXTernal Interrupt
#define TOTAL_EXTI_CONFIGURATION_REGISTERS ((uint8_t) 4)

typedef struct {
  volatile uint32_t MEMRMP;
  volatile uint32_t PMC;
  volatile uint32_t EXTICR[TOTAL_EXTI_CONFIGURATION_REGISTERS];
  uint32_t reserved[2];
  volatile uint32_t CMPCR;
} SYSCFG_t;

typedef struct {
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_t;

/* Declaramos las macros con los punteros a los diferentes perifericos GPIOs. Al tener la estructura el mismo tamaño que los diferentes registros de un GPIO, y tener esta estructura los mismo tamaños que los registros, cuando modifiquemos un registro del struct, modificaremos directamente la posicion de memoria donde está el registro deseado. */  
#define GPIOA   ((GPIO_RegDef_t *) GPIOAB_ADDR)
#define GPIOB   ((GPIO_RegDef_t *) GPIOBB_ADDR)
#define GPIOC   ((GPIO_RegDef_t *) GPIOCB_ADDR)
#define GPIOD   ((GPIO_RegDef_t *) GPIODB_ADDR)
#define GPIOE   ((GPIO_RegDef_t *) GPIOEB_ADDR)
#define GPIOF   ((GPIO_RegDef_t *) GPIOFB_ADDR)
#define GPIOG   ((GPIO_RegDef_t *) GPIOGB_ADDR)
#define GPIOH   ((GPIO_RegDef_t *) GPIOHB_ADDR)
#define GPIOI   ((GPIO_RegDef_t *) GPIOIB_ADDR)
#define GPIOJ   ((GPIO_RegDef_t *) GPIOJB_ADDR)
#define GPIOK   ((GPIO_RegDef_t *) GPIOKB_ADDR)

#define RCC     ((RCC_RegDef_t *) RCCB_ADDR)

#define SYSCFG  ((SYSCFG_t *) SYSCFGB_ADDR)

#define EXTI    ((EXTI_t *) EXTIB_ADDR)

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

#endif /* STM32F407_XX_H */
