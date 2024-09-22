#ifndef STM32CORTEXM4_H
#define STM32CORTEXM4_H

/* Nested vectored interrupt controller base address */
#define NVIC_ISER0_BADDR  ((uint32_t) 0xE000E100)
#define NVIC_ISER1_BADDR  ((uint32_t) 0xE000E104)
#define NVIC_ISER2_BADDR  ((uint32_t) 0xE000E108)
#define NVIC_ISER3_BADDR  ((uint32_t) 0xE000E10C)
#define NVIC_ISER4_BADDR  ((uint32_t) 0xE000E110)
#define NVIC_ISER5_BADDR  ((uint32_t) 0xE000E114)
#define NVIC_ISER6_BADDR  ((uint32_t) 0xE000E118)
#define NVIC_ISER7_BADDR  ((uint32_t) 0xE000E11C)

typedef struct {
  uint32_t NVIC_ISER[8];
  uint32_t NVIC_ICER[8];
  uint32_t NVIC_ISPR[8];
  uint32_t NVIC_ICPR[8];
  uint32_t NVIC_IABR[8];
  uint32_t NVIC_IPR[60];
  uint32_t NVIC_STIR;
} NVIC_t;

#define NVIC  ((NVIC_t *) NVIC_ISER0_BADDR)

#endif /* STM32CORTEXM4_H */
