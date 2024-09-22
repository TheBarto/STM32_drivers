#include <stdint.h>

#define SRAM_START  (0x20000000U)
#define SRAM_SIZE   (128U*1024U) //128KB
#define SRAM_END    (SRAM_START + SRAM_SIZE)

/* RECORDAR QUE LA PILA VA DE LA DIRECCION MAS ALTA A
 * LA MAS BAJA. QUE ESTO DEPENDE DEL TIPO DE MICRO. */
#define STACK_START SRAM_END

void __libc_init_array(void);

void Reset_Handler(void);

void NMI_Handler(void)                        __attribute__ ((weak,alias("Default_Handler")));
void HardFault_Handler(void)                  __attribute__ ((weak,alias("Default_Handler")));
void MemManage_Handler(void)                  __attribute__ ((weak,alias("Default_Handler")));
void BusFault_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void UsageFault_Handler(void)                 __attribute__ ((weak,alias("Default_Handler")));
void SVC_Handler(void)                        __attribute__ ((weak,alias("Default_Handler")));
void Debug_Monitor_Handler(void)              __attribute__ ((weak,alias("Default_Handler")));
void PendSV_Handler(void)                     __attribute__ ((weak,alias("Default_Handler")));
void SysTick_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void WWDG_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void PVD_Handler(void)                        __attribute__ ((weak,alias("Default_Handler")));
void TampStamp_Handler(void)                  __attribute__ ((weak,alias("Default_Handler")));
void RTC_WKUP_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void FLASH_Handler(void)                      __attribute__ ((weak,alias("Default_Handler")));
void RCC_Handler(void)                        __attribute__ ((weak,alias("Default_Handler")));
void EXTI0_Handler(void)                      __attribute__ ((weak,alias("Default_Handler")));
void EXTI1_Handler(void)                      __attribute__ ((weak,alias("Default_Handler")));
void EXTI2_Handler(void)                      __attribute__ ((weak,alias("Default_Handler")));
void EXTI3_Handler(void)                      __attribute__ ((weak,alias("Default_Handler")));
void EXTI4_Handler(void)                      __attribute__ ((weak,alias("Default_Handler")));
void DMA1_Stream0_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA1_Stream1_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA1_Stream2_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA1_Stream3_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA1_Stream4_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA1_Stream5_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA1_Stream6_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void ADC_Handler(void)                        __attribute__ ((weak,alias("Default_Handler")));
void CAN1_TX_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void CAN1_RX0_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void CAN1_RX1_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void CAN1_SCE_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void EXTI9_5_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void TIM1_BRK_TIM9_Handler(void)              __attribute__ ((weak,alias("Default_Handler")));
void TIM1_UP_TIM10_Handler(void)              __attribute__ ((weak,alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_Handler(void)         __attribute__ ((weak,alias("Default_Handler")));
void TIM1_CC_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void TIM2_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void TIM3_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void TIM4_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void I2C1_EV_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void I2C1_ER_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void I2C2_EV_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void I2C2_ER_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void SPI1_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void SPI2_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void USART1_Handler(void)                     __attribute__ ((weak,alias("Default_Handler")));
void USART2_Handler(void)                     __attribute__ ((weak,alias("Default_Handler")));
void USART3_Handler(void)                     __attribute__ ((weak,alias("Default_Handler")));
void EXTI15_10_Handler(void)                  __attribute__ ((weak,alias("Default_Handler")));
void RTC_Alarm_Handler(void)                  __attribute__ ((weak,alias("Default_Handler")));
void OTG_FS_WKUP_Handler(void)                __attribute__ ((weak,alias("Default_Handler")));
void TIM8_BRK_TIM12_Handler(void)             __attribute__ ((weak,alias("Default_Handler")));
void TIM8_BRK_TIM13_Handler(void)             __attribute__ ((weak,alias("Default_Handler")));
void TIM8_TRG_COM_TIM14_Handler(void)         __attribute__ ((weak,alias("Default_Handler")));
void TIM8_CC_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void DMA1_Stream7_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void FSMC_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void SDIO_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void TIM5_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void SPI3_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void UART4_Handler(void)                      __attribute__ ((weak,alias("Default_Handler")));
void UART5_Handler(void)                      __attribute__ ((weak,alias("Default_Handler")));
void TIM6_DAC_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void TIM7_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void DMA2_Stream0_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA2_Stream1_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA2_Stream2_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA2_Stream3_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA2_Stream4_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void ETH_Handler(void)                        __attribute__ ((weak,alias("Default_Handler")));
void ETH_WKUP_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void CAN2_TX_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void CAN2_RX0_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void CAN2_RX1_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void CAN2_SCE_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void OTG_FS_Handler(void)                     __attribute__ ((weak,alias("Default_Handler")));
void DMA2_Stream5_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA2_Stream6_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void DMA2_Stream7_Handler(void)               __attribute__ ((weak,alias("Default_Handler")));
void USART6_Handler(void)                     __attribute__ ((weak,alias("Default_Handler")));
void I2C3_EV_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void I2C3_ER_Handler(void)                    __attribute__ ((weak,alias("Default_Handler")));
void OTG_HS_EP1_OUT_Handler(void)             __attribute__ ((weak,alias("Default_Handler")));
void OTG_HS_EP1_IN_Handler(void)              __attribute__ ((weak,alias("Default_Handler")));
void OTG_HS_WKUP_Handler(void)                __attribute__ ((weak,alias("Default_Handler")));
void OTG_HS_Handler(void)                     __attribute__ ((weak,alias("Default_Handler")));
void DCMI_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void CRYP_Handler(void)                       __attribute__ ((weak,alias("Default_Handler")));
void HASH_RNG_Handler(void)                   __attribute__ ((weak,alias("Default_Handler")));
void FPU_Handler(void)                        __attribute__ ((weak,alias("Default_Handler")));

void Default_Handler(void)
{
  while(1);
}

/* Importamos como externs todos los simbolos creados para poder acceder a las diferentes
 * zonas de memoria, y asi poder acceder a los datos para su manipulación. 
 * Como son externs, el compilador no dara ningún error, sino que es tarea del linkador buscar
 * esos simbolos y añadirlos al codigo final, dando este un error si no encuentra ningun elemento
 * con estos nombres/identificadores */
extern uint32_t _stext;
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _la_data;

/* Declaramos la funcion main como un extern y así tener lista la llamada en el Reset_Handler.*/
extern int main(void);

void Reset_Handler()
{
  /* Esta va a ser la primera funcion que se ejecute cuando iniciemos el micro,
   * o cuando lo reiniciemos, así que en esta funcion vamos a poner todos los pasos
   * previos a la llamada a la funcion main, como pueden ser la copia de datos de la
   * FLASH a la seccion .data de la memoria RAM y la inicialización de la zona .bss. */
   //Si empleamos funciones de la libreria standard, aqui se deben inicializar tambien.
   /* Para saber cual seria la primera funcion en ejecutarse, tenemos que buscar en los
    * archivos de la placa de desarrollo o del micro-controlador. En este caso busque
    * 'boot' en los diferentes archivos, y encontre que en el manual de referencia de
    * la placa (no del micro), se encuentra la seccion Boot Configuration, la cual nos
    * indica que siempre obtiene la direccion del vector de reset, siendo esta la que
    * emplea durante el inicio */
   
   /* Copiar los datos de la parte .data a la memoria RAM. Para ello emplearemos los simbolos
    * que hemos declarado en el script de linkado, y para acceder a ellos simplemente utilizamos
    * la palabra clave extern, con el nombre del simbolo */
    
    /* IMPORTANTE, QUEREMOS ACCEDER A LA DIRECCION DE MEMORIA, NO AL CONTENIDO, POR ESO
     * EMPLEAMOS EL SIMBOLO & */
    uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;
    uint8_t* pSrc = (uint8_t *)&_la_data;
    uint8_t* pDst = (uint8_t *)&_sdata;
    
    /* En este punto ya tenemos el tamaño a copiar, y las direcciones inicial y final para
     * realizar la copia; por lo que solo quedar copiar los datos y ya. */
    for(uint32_t i = 0; i < size; i++) 
      *pDst++ = *pSrc++;
    
    /* Tras haber copiado los datos de la FLASH a la RAM, ya solamente queda inicializar a
     * 0 el valor de la zona de memoria .bss y llamar a la funcion main(), y con esto ya
     * tendríamos el fichero .elf listo para cargar en el micro para funcionar correctamente */
    size = &_ebss - &_sbss;
    pDst = (uint8_t *)&_sbss;
    for(uint32_t i = 0; i < size; i++)
      *pDst = 0;
    
    __libc_init_array();
    
    main();
}

/* MUY IMPORTANTE, FORZAMOS A QUE LA TABLA DE VECTORES SE CARGUE EN UNA SECCION EN PARTICULAR,
YA QUE SI LO DEJAMOS POR DEFECTO, AL SER UNA VARIABLE GLOBAL E INICIALIZADA, IRIA A LA SECCIÓN
DE DATOS, Y NO QUEREMOS ESO, YA QUE DEBEMOS PONERLA AL INICIO DE LA MEMORIA */
uint32_t Vector_List[] __attribute__((section(".isr_vector"))) = {
  STACK_START,
  (uint32_t) &Reset_Handler,
  (uint32_t) &NMI_Handler,
  (uint32_t) &HardFault_Handler,
  (uint32_t) &MemManage_Handler,
  (uint32_t) &BusFault_Handler,
  (uint32_t) &UsageFault_Handler,
  (uint32_t) 0x00,
  (uint32_t) 0x00,
  (uint32_t) 0x00,
  (uint32_t) 0x00,
  (uint32_t) &SVC_Handler,
  (uint32_t) &Debug_Monitor_Handler,
  (uint32_t) 0x00,
  (uint32_t) &PendSV_Handler,
  (uint32_t) &SysTick_Handler,
  (uint32_t) &WWDG_Handler,
  (uint32_t) &PVD_Handler,
  (uint32_t) &TampStamp_Handler,
  (uint32_t) &RTC_WKUP_Handler,
  (uint32_t) &FLASH_Handler,
  (uint32_t) &RCC_Handler,
  (uint32_t) &EXTI0_Handler,
  (uint32_t) &EXTI1_Handler,
  (uint32_t) &EXTI2_Handler,
  (uint32_t) &EXTI3_Handler,
  (uint32_t) &EXTI4_Handler,
  (uint32_t) &DMA1_Stream0_Handler,
  (uint32_t) &DMA1_Stream1_Handler,
  (uint32_t) &DMA1_Stream2_Handler,
  (uint32_t) &DMA1_Stream3_Handler,
  (uint32_t) &DMA1_Stream4_Handler,
  (uint32_t) &DMA1_Stream5_Handler,
  (uint32_t) &DMA1_Stream6_Handler,
  (uint32_t) &ADC_Handler,
  (uint32_t) &CAN1_TX_Handler,
  (uint32_t) &CAN1_RX0_Handler,
  (uint32_t) &CAN1_RX1_Handler,
  (uint32_t) &CAN1_SCE_Handler,
  (uint32_t) &EXTI9_5_Handler,
  (uint32_t) &TIM1_BRK_TIM9_Handler,
  (uint32_t) &TIM1_UP_TIM10_Handler,
  (uint32_t) &TIM1_TRG_COM_TIM11_Handler,
  (uint32_t) &TIM1_CC_Handler,
  (uint32_t) &TIM2_Handler,
  (uint32_t) &TIM3_Handler,
  (uint32_t) &TIM4_Handler,
  (uint32_t) &I2C1_EV_Handler,
  (uint32_t) &I2C1_ER_Handler,
  (uint32_t) &I2C2_EV_Handler,
  (uint32_t) &I2C2_ER_Handler,
  (uint32_t) &SPI1_Handler,
  (uint32_t) &SPI2_Handler,
  (uint32_t) &USART1_Handler,
  (uint32_t) &USART2_Handler,
  (uint32_t) &USART3_Handler,
  (uint32_t) &EXTI15_10_Handler,
  (uint32_t) &RTC_Alarm_Handler,
  (uint32_t) &OTG_FS_WKUP_Handler,
  (uint32_t) &TIM8_BRK_TIM12_Handler,
  (uint32_t) &TIM8_BRK_TIM13_Handler,
  (uint32_t) &TIM8_TRG_COM_TIM14_Handler,
  (uint32_t) &TIM8_CC_Handler,
  (uint32_t) &DMA1_Stream7_Handler,
  (uint32_t) &FSMC_Handler,
  (uint32_t) &SDIO_Handler,
  (uint32_t) &TIM5_Handler,
  (uint32_t) &SPI3_Handler,
  (uint32_t) &UART4_Handler,
  (uint32_t) &UART5_Handler,
  (uint32_t) &TIM6_DAC_Handler,
  (uint32_t) &TIM7_Handler,
  (uint32_t) &DMA2_Stream0_Handler,
  (uint32_t) &DMA2_Stream1_Handler,
  (uint32_t) &DMA2_Stream2_Handler,
  (uint32_t) &DMA2_Stream3_Handler,
  (uint32_t) &DMA2_Stream4_Handler,
  (uint32_t) &ETH_Handler,
  (uint32_t) &ETH_WKUP_Handler,
  (uint32_t) &CAN2_TX_Handler,
  (uint32_t) &CAN2_RX0_Handler,
  (uint32_t) &CAN2_RX1_Handler,
  (uint32_t) &CAN2_SCE_Handler,
  (uint32_t) &OTG_FS_Handler,
  (uint32_t) &DMA2_Stream5_Handler,
  (uint32_t) &DMA2_Stream6_Handler,
  (uint32_t) &DMA2_Stream7_Handler,
  (uint32_t) &USART6_Handler,
  (uint32_t) &I2C3_EV_Handler,
  (uint32_t) &I2C3_ER_Handler,
  (uint32_t) &OTG_HS_EP1_OUT_Handler,
  (uint32_t) &OTG_HS_EP1_IN_Handler,
  (uint32_t) &OTG_HS_WKUP_Handler,
  (uint32_t) &OTG_HS_Handler,
  (uint32_t) &DCMI_Handler,
  (uint32_t) &CRYP_Handler,
  (uint32_t) &HASH_RNG_Handler,
  (uint32_t) &FPU_Handler
};
