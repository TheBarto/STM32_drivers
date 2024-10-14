#ifndef STM32F407_XX_H
#define STM32F407_XX_H

#include <stdint.h>

typedef uint8_t bool;

#define true	((uint8_t) 1)
#define false	((uint8_t) 0)

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

/* Para acceder a todos los registros de un un periférico, en vez de definir todas las direcciones para los diferentes periféricos, podemos generar una estructura que contenga varios elementos/variables (que hagan referencia a los registros del periférico). Luego simplemente cogeremos esa estructura como un puntero, y le pondremos la dirección base del periférico. De esta manera podremos acceder facilmente a los periféricos sin necesidad de estar generando todas las direcciones una a una. 

Es muy importante que las variables que empleemos TENGAN EL MISMO TAMAÑO QUE LOS REGISTROS, YA QUE SINO GENERAREMOS UN DESCUADRE EN LOS ACCESOS Y LAS ASIGNACIONES.
*/

/* Consejo, añadir una breve descripción de la finalidad de cada variable. De este modo se sabrá para que vale cada uno */
/* Aunque NO SON NECESARIOS EN TODOS LOS CAMPOS, al hacer referencia a valores que pueden modificarse en cualquier momento y sin necesidad de hacer nada nosotros, es muy recomendable incluir el modificador volatile en todos los campos; o al menos en los de entrada de valores. */

/* RCC struct definition and macro declaration */

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

#define RCC_AHB1_EN_RESET_GPIOA	((uint32_t) 0x00000001)
#define RCC_AHB1_EN_RESET_GPIOB	((uint32_t) 0x00000002)
#define RCC_AHB1_EN_RESET_GPIOC	((uint32_t) 0x00000004)
#define RCC_AHB1_EN_RESET_GPIOD	((uint32_t) 0x00000008)
#define RCC_AHB1_EN_RESET_GPIOE	((uint32_t) 0x00000010)
#define RCC_AHB1_EN_RESET_GPIOF	((uint32_t) 0x00000020)
#define RCC_AHB1_EN_RESET_GPIOG	((uint32_t) 0x00000040)
#define RCC_AHB1_EN_RESET_GPIOH	((uint32_t) 0x00000080)
#define RCC_AHB1_EN_RESET_GPIOI	((uint32_t) 0x00000100)

#define RCC_APB1_EN_RESET_SPI2	((uint32_t) 0x00004000)
#define RCC_APB1_EN_RESET_SPI3	((uint32_t) 0x00008000)

#define RCC_APB2_EN_RESET_SPI1	((uint32_t) 0x00001000)

#define RCC     ((RCC_RegDef_t *) RCCB_ADDR)

// EXTI -> EXTernal Interrupt
#define TOTAL_EXTI_CONFIGURATION_REGISTERS ((uint8_t) 4)

typedef struct {
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_t;

#define EXTI    ((EXTI_t *) EXTIB_ADDR)

typedef struct {
  volatile uint32_t MEMRMP;
  volatile uint32_t PMC;
  volatile uint32_t EXTICR[TOTAL_EXTI_CONFIGURATION_REGISTERS];
  uint32_t reserved[2];
  volatile uint32_t CMPCR;
} SYSCFG_t;

#define SYSCFG  ((SYSCFG_t *) SYSCFGB_ADDR)

#endif /* STM32F407_XX_H */
