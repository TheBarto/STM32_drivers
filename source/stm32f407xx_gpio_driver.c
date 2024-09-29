#include "stm32f407xx_gpio_driver.h"
#include "stm32cortexm4.h"
#include <stdio.h>

/* GPIO -> General Purpose Input Output */
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

#define TOTAL_STM32F407_GPIO ((uint8_t) 8)

/* Declaramos las macros con los punteros a los diferentes perifericos GPIOs. Al tener la estructura el mismo tamaño que los diferentes registros de un GPIO, y tener esta estructura los mismo tamaños que los registros, cuando modifiquemos un registro del struct, modificaremos directamente la posicion de memoria donde está el registro deseado. */
/* Static array with the memory address of the different GPIOs employed.
 * With this we can modify the different GPIO element presents in the board */
static GPIO_RegDef_t* GPIOS_ports[TOTAL_STM32F407_GPIO];

void GPIO_initialization_module()
{
  GPIOS_ports[GPIO_PORT_A] = (GPIO_RegDef_t*) GPIOAB_ADDR;
  GPIOS_ports[GPIO_PORT_B] = (GPIO_RegDef_t*) GPIOBB_ADDR;
  GPIOS_ports[GPIO_PORT_C] = (GPIO_RegDef_t*) GPIOCB_ADDR;
  GPIOS_ports[GPIO_PORT_D] = (GPIO_RegDef_t*) GPIODB_ADDR;
  GPIOS_ports[GPIO_PORT_E] = (GPIO_RegDef_t*) GPIOEB_ADDR;
  GPIOS_ports[GPIO_PORT_F] = (GPIO_RegDef_t*) GPIOFB_ADDR;
  GPIOS_ports[GPIO_PORT_G] = (GPIO_RegDef_t*) GPIOGB_ADDR;
  GPIOS_ports[GPIO_PORT_H] = (GPIO_RegDef_t*) GPIOHB_ADDR;

  return;
}

void GPIO_Initialization(uint8_t GPIO_port, uint8_t GPIO_pin, uint8_t mode,
                         uint8_t pull_up_down, uint8_t output_type,
                         uint8_t output_speed, uint16_t alternate_function,
                         uint8_t IRQ_priority)
{
  //First, for each pin, clear the register, and later configure it!
  if(mode <= GPIO_MODE_ANALOG) {
    GPIOS_ports[GPIO_port]->MODER &= ~(0x03 << (GPIO_pin*2));
    GPIOS_ports[GPIO_port]->MODER |= ((mode & 0x03) << (GPIO_pin*2));
  } else {
    /* Configure the GPIO pin with interrupt mode.
     * For that must be in input mode. Input mode is 0x00.
     */
    GPIOS_ports[GPIO_port]->MODER &= ~(0x00 << (GPIO_pin*2));
    GPIO_IRQ_configure(GPIO_port, GPIO_pin, IRQ_priority, mode);
  }
  
  GPIOS_ports[GPIO_port]->PUPDR &= ~(0x03 << (GPIO_pin*2));
  GPIOS_ports[GPIO_port]->PUPDR |= ((pull_up_down & 0x03) << (GPIO_pin*2));
  
  GPIOS_ports[GPIO_port]->OTYPER &= ~(0x01 << GPIO_pin);
  GPIOS_ports[GPIO_port]->OTYPER |= ((output_type & 0x01) << GPIO_pin);
  
  GPIOS_ports[GPIO_port]->OSPEEDR &= ~(0x03 << (GPIO_pin*2));
  GPIOS_ports[GPIO_port]->OSPEEDR |= ((output_speed & 0x03) << (GPIO_pin*2));

  if(alternate_function != GPIO_NON_ALTERNATE_FUNCTIONALITY) {
    GPIOS_ports[GPIO_port]->AFR[(GPIO_pin/GPIO_PIN_8)] &= ~(0x0F << ((GPIO_pin%GPIO_PIN_8)*4));
    GPIOS_ports[GPIO_port]->AFR[(GPIO_pin/GPIO_PIN_8)] |= ((alternate_function & 0x0F) << ((GPIO_pin%GPIO_PIN_8)*4));
  }

  return;
}

void GPIO_Reset(uint8_t GPIO_port)
{
  uint16_t GPIO_pos = 0;
  GPIO_pos |= (1 << GPIO_port);

  RCC->AHB1RSTR |= GPIO_pos;
  RCC->AHB1RSTR &= ~GPIO_pos;

  return;
}

void GPIO_clock_enable_disable(uint8_t GPIO_port, uint8_t enable_disable)
{
  uint16_t GPIO_pos = 0;
  GPIO_pos |= (1 << GPIO_port);
  
  if(enable_disable == 1)
    RCC->AHB1ENR |= GPIO_pos;
  else
    RCC->AHB1ENR &= ~GPIO_pos;

  return;
}

uint8_t GPIO_read_pin(uint8_t GPIO_port, uint8_t GPIO_pin)
{
  return (GPIOS_ports[GPIO_port]->IDR & (1 << GPIO_pin));
}

void GPIO_write_pin(uint8_t GPIO_port, uint8_t GPIO_pin, uint8_t pin_value)
{
  GPIOS_ports[GPIO_port]->ODR = (pin_value << GPIO_pin); 
  return;
}

uint16_t GPIO_read_port(uint8_t GPIO_port)
{
  return GPIOS_ports[GPIO_port]->IDR;
}

void GPIO_write_port(uint8_t GPIO_port, uint16_t port_value)
{
  GPIOS_ports[GPIO_port]->ODR = port_value;
  return;
}

void GPIO_IRQ_configure(uint8_t GPIO_port, uint8_t GPIO_pin,
                        uint8_t IRQ_priority, uint8_t mode)
{
  /* To configure this, it's necessary to modify several registers of interrupts and events.*/
  /* First of all, with the desired pin, we have to configure it to be the EXTI pin. All of
   * this must be seen in the reference manual, in the chapter "Interrupts and events".
   * After decide if the interrupt will trigger with rising or a falling event, and enable it 
   * with the interrupt mask register. */
   
   /* To select the EXTIx port, it's mandatory use the SYSCFG_EXTICRx register. First it's
    * necessary enable it. */
   RCC->APB2LPENR |= (1 << 14);
   
   /* After, select the pin to activate the interrupt */
   SYSCFG->EXTICR[GPIO_pin/4] &= ~(0x0F << (((GPIO_pin%4)*4)));
   SYSCFG->EXTICR[GPIO_pin/4] |= (GPIO_port << (((GPIO_pin%4)*4)));
   
   /* Select the activation's mode, by rising or falling edge */
   EXTI->RTSR &= ~(1 << GPIO_pin);
   EXTI->FTSR &= ~(1 << GPIO_pin);
   if((mode == GPIO_MODE_INT_RISING) ||
      (mode == GPIO_MODE_INT_RIS_FAL))
    EXTI->RTSR |= (1 << GPIO_pin);
   if((mode == GPIO_MODE_INT_FALLING) ||
      (mode == GPIO_MODE_INT_RIS_FAL))
    EXTI->FTSR |= (1 << GPIO_pin);
   
   /* Enable the EXTI mask */
   EXTI->IMR |= (1 << GPIO_pin);
   
   uint8_t IRQ_number = 0;
   //NEED TO OBTAIN THE IRQ NUMBER.
   switch(GPIO_pin) {
    case GPIO_PIN_0:
    case GPIO_PIN_1:
    case GPIO_PIN_2:
    case GPIO_PIN_3:
    case GPIO_PIN_4:
      IRQ_number = (GPIO_pin+6);
      break;
    case GPIO_PIN_5:
    case GPIO_PIN_6:
    case GPIO_PIN_7:
    case GPIO_PIN_8:
    case GPIO_PIN_9:
      IRQ_number = 23;
      break;
    case GPIO_PIN_10:
    case GPIO_PIN_11:
    case GPIO_PIN_12:
    case GPIO_PIN_13:
    case GPIO_PIN_14:
    case GPIO_PIN_15:
      IRQ_number = 40;
      break;
   }
   
  /* Configure the priority of the interrupt - processor side.  */
  NVIC->NVIC_IPR[IRQ_number/4] = ((IRQ_priority * 0x10) << ((IRQ_number%4)*8));
      
  /* Enable the EXTI mask and the processor's interrupt enable */
  NVIC->NVIC_ISER[IRQ_number/31] |= (1 << (IRQ_number%31));
}

/* This function has the mission of cleaning the pending register of the EXTI element. */
void GPIO_IRQ_handling(uint8_t GPIO_pin)
{
  EXTI->PR &= ~(1 << GPIO_pin);
}
