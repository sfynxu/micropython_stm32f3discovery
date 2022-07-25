# 1 "<stdin>"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "<stdin>"
# 29 "<stdin>"
# 1 "../../py/mpconfig.h" 1
# 62 "../../py/mpconfig.h"
# 1 "./mpconfigport.h" 1
# 31 "./mpconfigport.h"
# 1 "boards/STM32F3DISC/mpconfigboard.h" 1
# 32 "./mpconfigport.h" 2
# 1 "./mpconfigboard_common.h" 1
# 30 "./mpconfigboard_common.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h" 1
# 30 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"
# 1 "boards/STM32F3DISC/stm32f3xx_hal_conf.h" 1







# 1 "./boards/stm32f3xx_hal_conf_base.h" 1
# 30 "./boards/stm32f3xx_hal_conf_base.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma.h" 1
# 29 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_def.h" 1
# 30 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_def.h"
# 1 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/stm32f3xx.h" 1
# 136 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/stm32f3xx.h"
# 1 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/stm32f303xc.h" 1
# 66 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/stm32f303xc.h"
typedef enum
{

  NonMaskableInt_IRQn = -14,
  HardFault_IRQn = -13,
  MemoryManagement_IRQn = -12,
  BusFault_IRQn = -11,
  UsageFault_IRQn = -10,
  SVCall_IRQn = -5,
  DebugMonitor_IRQn = -4,
  PendSV_IRQn = -2,
  SysTick_IRQn = -1,

  WWDG_IRQn = 0,
  PVD_IRQn = 1,
  TAMP_STAMP_IRQn = 2,
  RTC_WKUP_IRQn = 3,
  FLASH_IRQn = 4,
  RCC_IRQn = 5,
  EXTI0_IRQn = 6,
  EXTI1_IRQn = 7,
  EXTI2_TSC_IRQn = 8,
  EXTI3_IRQn = 9,
  EXTI4_IRQn = 10,
  DMA1_Channel1_IRQn = 11,
  DMA1_Channel2_IRQn = 12,
  DMA1_Channel3_IRQn = 13,
  DMA1_Channel4_IRQn = 14,
  DMA1_Channel5_IRQn = 15,
  DMA1_Channel6_IRQn = 16,
  DMA1_Channel7_IRQn = 17,
  ADC1_2_IRQn = 18,
  USB_HP_CAN_TX_IRQn = 19,
  USB_LP_CAN_RX0_IRQn = 20,
  CAN_RX1_IRQn = 21,
  CAN_SCE_IRQn = 22,
  EXTI9_5_IRQn = 23,
  TIM1_BRK_TIM15_IRQn = 24,
  TIM1_UP_TIM16_IRQn = 25,
  TIM1_TRG_COM_TIM17_IRQn = 26,
  TIM1_CC_IRQn = 27,
  TIM2_IRQn = 28,
  TIM3_IRQn = 29,
  TIM4_IRQn = 30,
  I2C1_EV_IRQn = 31,
  I2C1_ER_IRQn = 32,
  I2C2_EV_IRQn = 33,
  I2C2_ER_IRQn = 34,
  SPI1_IRQn = 35,
  SPI2_IRQn = 36,
  USART1_IRQn = 37,
  USART2_IRQn = 38,
  USART3_IRQn = 39,
  EXTI15_10_IRQn = 40,
  RTC_Alarm_IRQn = 41,
  USBWakeUp_IRQn = 42,
  TIM8_BRK_IRQn = 43,
  TIM8_UP_IRQn = 44,
  TIM8_TRG_COM_IRQn = 45,
  TIM8_CC_IRQn = 46,
  ADC3_IRQn = 47,
  SPI3_IRQn = 51,
  UART4_IRQn = 52,
  UART5_IRQn = 53,
  TIM6_DAC_IRQn = 54,
  TIM7_IRQn = 55,
  DMA2_Channel1_IRQn = 56,
  DMA2_Channel2_IRQn = 57,
  DMA2_Channel3_IRQn = 58,
  DMA2_Channel4_IRQn = 59,
  DMA2_Channel5_IRQn = 60,
  ADC4_IRQn = 61,
  COMP1_2_3_IRQn = 64,
  COMP4_5_6_IRQn = 65,
  COMP7_IRQn = 66,
  USB_HP_IRQn = 74,
  USB_LP_IRQn = 75,
  USBWakeUp_RMP_IRQn = 76,
  FPU_IRQn = 81,
} IRQn_Type;





# 1 "../../lib/cmsis/inc/core_cm4.h" 1
# 34 "../../lib/cmsis/inc/core_cm4.h"
# 1 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stdint.h" 1 3 4
# 34 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stdint.h" 3 4

# 34 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stdint.h" 3 4
typedef signed char int8_t;


typedef short int int16_t;


typedef long int int32_t;


typedef long long int int64_t;


typedef unsigned char uint8_t;


typedef short unsigned int uint16_t;


typedef long unsigned int uint32_t;


typedef long long unsigned int uint64_t;




typedef signed char int_least8_t;
typedef short int int_least16_t;
typedef long int int_least32_t;
typedef long long int int_least64_t;
typedef unsigned char uint_least8_t;
typedef short unsigned int uint_least16_t;
typedef long unsigned int uint_least32_t;
typedef long long unsigned int uint_least64_t;



typedef int int_fast8_t;
typedef int int_fast16_t;
typedef int int_fast32_t;
typedef long long int int_fast64_t;
typedef unsigned int uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
typedef long long unsigned int uint_fast64_t;




typedef int intptr_t;


typedef unsigned int uintptr_t;




typedef long long int intmax_t;
typedef long long unsigned int uintmax_t;
# 35 "../../lib/cmsis/inc/core_cm4.h" 2
# 63 "../../lib/cmsis/inc/core_cm4.h"
# 1 "../../lib/cmsis/inc/cmsis_version.h" 1
# 64 "../../lib/cmsis/inc/core_cm4.h" 2
# 162 "../../lib/cmsis/inc/core_cm4.h"
# 1 "../../lib/cmsis/inc/cmsis_compiler.h" 1
# 54 "../../lib/cmsis/inc/cmsis_compiler.h"
# 1 "../../lib/cmsis/inc/cmsis_gcc.h" 1
# 29 "../../lib/cmsis/inc/cmsis_gcc.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"
# 71 "../../lib/cmsis/inc/cmsis_gcc.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  
# 74 "../../lib/cmsis/inc/cmsis_gcc.h"
 struct __attribute__((packed)) T_UINT32 { uint32_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT16_WRITE { uint16_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT16_READ { uint16_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT32_WRITE { uint32_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT32_READ { uint32_t v; };
#pragma GCC diagnostic pop
# 129 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}







__attribute__((always_inline)) static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}







__attribute__((always_inline)) static inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 181 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
}
# 205 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static inline uint32_t __get_PSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, psp" : "=r" (result) );
  return(result);
}
# 277 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0" : : "r" (topOfProcStack) : );
}
# 301 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_MSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, msp" : "=r" (result) );
  return(result);
}
# 331 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : );
}
# 382 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) :: "memory");
  return(result);
}
# 412 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 766 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_FPSCR(void)
{
# 782 "../../lib/cmsis/inc/cmsis_gcc.h"
  return(0U);

}







__attribute__((always_inline)) static inline void __set_FPSCR(uint32_t fpscr)
{
# 805 "../../lib/cmsis/inc/cmsis_gcc.h"
  (void)fpscr;

}
# 866 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __ISB(void)
{
  __asm volatile ("isb 0xF":::"memory");
}







__attribute__((always_inline)) static inline void __DSB(void)
{
  __asm volatile ("dsb 0xF":::"memory");
}







__attribute__((always_inline)) static inline void __DMB(void)
{
  __asm volatile ("dmb 0xF":::"memory");
}
# 900 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __REV(uint32_t value)
{

  return __builtin_bswap32(value);






}
# 919 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __REV16(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev16 %0, %1" : "=r" (result) : "r" (value) );
  return result;
}
# 934 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline int16_t __REVSH(int16_t value)
{

  return (int16_t)__builtin_bswap16(value);






}
# 954 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{
  op2 %= 32U;
  if (op2 == 0U)
  {
    return op1;
  }
  return (op1 >> op2) | (op1 << (32U - op2));
}
# 981 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;






  uint32_t s = (4U * 8U) - 1U;

  result = value;
  for (value >>= 1U; value != 0U; value >>= 1U)
  {
    result <<= 1U;
    result |= value & 1U;
    s--;
  }
  result <<= s;

  return result;
}
# 1011 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint8_t __CLZ(uint32_t value)
{
# 1022 "../../lib/cmsis/inc/cmsis_gcc.h"
  if (value == 0U)
  {
    return 32U;
  }
  return __builtin_clz(value);
}
# 1315 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline int32_t __SSAT(int32_t val, uint32_t sat)
{
  if ((sat >= 1U) && (sat <= 32U))
  {
    const int32_t max = (int32_t)((1U << (sat - 1U)) - 1U);
    const int32_t min = -1 - max ;
    if (val > max)
    {
      return max;
    }
    else if (val < min)
    {
      return min;
    }
  }
  return val;
}
# 1340 "../../lib/cmsis/inc/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __USAT(int32_t val, uint32_t sat)
{
  if (sat <= 31U)
  {
    const uint32_t max = ((1U << sat) - 1U);
    if (val > (int32_t)max)
    {
      return max;
    }
    else if (val < 0)
    {
      return 0U;
    }
  }
  return (uint32_t)val;
}
# 2099 "../../lib/cmsis/inc/cmsis_gcc.h"
#pragma GCC diagnostic pop
# 55 "../../lib/cmsis/inc/cmsis_compiler.h" 2
# 163 "../../lib/cmsis/inc/core_cm4.h" 2
# 259 "../../lib/cmsis/inc/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t _reserved0:16;
    uint32_t GE:4;
    uint32_t _reserved1:7;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;
# 298 "../../lib/cmsis/inc/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;
# 316 "../../lib/cmsis/inc/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:1;
    uint32_t ICI_IT_1:6;
    uint32_t GE:4;
    uint32_t _reserved1:4;
    uint32_t T:1;
    uint32_t ICI_IT_2:2;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;
# 371 "../../lib/cmsis/inc/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t FPCA:1;
    uint32_t _reserved0:29;
  } b;
  uint32_t w;
} CONTROL_Type;
# 406 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
  volatile uint32_t ISER[8U];
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];
        uint32_t RESERVED1[24U];
  volatile uint32_t ISPR[8U];
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];
        uint32_t RESERVED4[56U];
  volatile uint8_t IP[240U];
        uint32_t RESERVED5[644U];
  volatile uint32_t STIR;
} NVIC_Type;
# 440 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t SHP[12U];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile const uint32_t PFR[2U];
  volatile const uint32_t DFR;
  volatile const uint32_t ADR;
  volatile const uint32_t MMFR[4U];
  volatile const uint32_t ISAR[5U];
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;
} SCB_Type;
# 719 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const uint32_t ICTR;
  volatile uint32_t ACTLR;
} SCnSCB_Type;
# 759 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 811 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
  volatile union
  {
    volatile uint8_t u8;
    volatile uint16_t u16;
    volatile uint32_t u32;
  } PORT [32U];
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;
        uint32_t RESERVED3[32U];
        uint32_t RESERVED4[43U];
  volatile uint32_t LAR;
  volatile const uint32_t LSR;
        uint32_t RESERVED5[6U];
  volatile const uint32_t PID4;
  volatile const uint32_t PID5;
  volatile const uint32_t PID6;
  volatile const uint32_t PID7;
  volatile const uint32_t PID0;
  volatile const uint32_t PID1;
  volatile const uint32_t PID2;
  volatile const uint32_t PID3;
  volatile const uint32_t CID0;
  volatile const uint32_t CID1;
  volatile const uint32_t CID2;
  volatile const uint32_t CID3;
} ITM_Type;
# 899 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t CYCCNT;
  volatile uint32_t CPICNT;
  volatile uint32_t EXCCNT;
  volatile uint32_t SLEEPCNT;
  volatile uint32_t LSUCNT;
  volatile uint32_t FOLDCNT;
  volatile const uint32_t PCSR;
  volatile uint32_t COMP0;
  volatile uint32_t MASK0;
  volatile uint32_t FUNCTION0;
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;
  volatile uint32_t MASK1;
  volatile uint32_t FUNCTION1;
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;
  volatile uint32_t MASK2;
  volatile uint32_t FUNCTION2;
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;
  volatile uint32_t MASK3;
  volatile uint32_t FUNCTION3;
} DWT_Type;
# 1046 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
  volatile const uint32_t SSPSR;
  volatile uint32_t CSPSR;
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;
        uint32_t RESERVED2[131U];
  volatile const uint32_t FFSR;
  volatile uint32_t FFCR;
  volatile const uint32_t FSCR;
        uint32_t RESERVED3[759U];
  volatile const uint32_t TRIGGER;
  volatile const uint32_t FIFO0;
  volatile const uint32_t ITATBCTR2;
        uint32_t RESERVED4[1U];
  volatile const uint32_t ITATBCTR0;
  volatile const uint32_t FIFO1;
  volatile uint32_t ITCTRL;
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;
  volatile uint32_t CLAIMCLR;
        uint32_t RESERVED7[8U];
  volatile const uint32_t DEVID;
  volatile const uint32_t DEVTYPE;
} TPI_Type;
# 1208 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
  volatile const uint32_t TYPE;
  volatile uint32_t CTRL;
  volatile uint32_t RNR;
  volatile uint32_t RBAR;
  volatile uint32_t RASR;
  volatile uint32_t RBAR_A1;
  volatile uint32_t RASR_A1;
  volatile uint32_t RBAR_A2;
  volatile uint32_t RASR_A2;
  volatile uint32_t RBAR_A3;
  volatile uint32_t RASR_A3;
} MPU_Type;
# 1304 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;
  volatile uint32_t FPCAR;
  volatile uint32_t FPDSCR;
  volatile const uint32_t MVFR0;
  volatile const uint32_t MVFR1;
  volatile const uint32_t MVFR2;
} FPU_Type;
# 1416 "../../lib/cmsis/inc/core_cm4.h"
typedef struct
{
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
} CoreDebug_Type;
# 1648 "../../lib/cmsis/inc/core_cm4.h"
static inline void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);

  reg_value = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));
  reg_value = (reg_value |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U) );
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = reg_value;
}







static inline uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}
# 1679 "../../lib/cmsis/inc/core_cm4.h"
static inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1696 "../../lib/cmsis/inc/core_cm4.h"
static inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1715 "../../lib/cmsis/inc/core_cm4.h"
static inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __DSB();
    __ISB();
  }
}
# 1734 "../../lib/cmsis/inc/core_cm4.h"
static inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1753 "../../lib/cmsis/inc/core_cm4.h"
static inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1768 "../../lib/cmsis/inc/core_cm4.h"
static inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1785 "../../lib/cmsis/inc/core_cm4.h"
static inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1807 "../../lib/cmsis/inc/core_cm4.h"
static inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}
# 1829 "../../lib/cmsis/inc/core_cm4.h"
static inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)] >> (8U - 4U)));
  }
  else
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4U)));
  }
}
# 1854 "../../lib/cmsis/inc/core_cm4.h"
static inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority & (uint32_t)((1UL << (SubPriorityBits )) - 1UL)))
         );
}
# 1881 "../../lib/cmsis/inc/core_cm4.h"
static inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority = (Priority ) & (uint32_t)((1UL << (SubPriorityBits )) - 1UL);
}
# 1904 "../../lib/cmsis/inc/core_cm4.h"
static inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t vectors = (uint32_t )((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  (* (int *) (vectors + ((int32_t)IRQn + 16) * 4)) = vector;
}
# 1919 "../../lib/cmsis/inc/core_cm4.h"
static inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t vectors = (uint32_t )((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  return (uint32_t)(* (int *) (vectors + ((int32_t)IRQn + 16) * 4));
}






__attribute__((__noreturn__)) static inline void __NVIC_SystemReset(void)
{
  __DSB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = (uint32_t)((0x5FAUL << 16U) |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U) );
  __DSB();

  for(;;)
  {
    __asm volatile ("nop");
  }
}
# 1952 "../../lib/cmsis/inc/core_cm4.h"
# 1 "../../lib/cmsis/inc/mpu_armv7.h" 1
# 183 "../../lib/cmsis/inc/mpu_armv7.h"
typedef struct {
  uint32_t RBAR;
  uint32_t RASR;
} ARM_MPU_Region_t;




static inline void ARM_MPU_Enable(uint32_t MPU_Control)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL = MPU_Control | (1UL );

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR |= (1UL << 16U);

  __DSB();
  __ISB();
}



static inline void ARM_MPU_Disable(void)
{
  __DMB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR &= ~(1UL << 16U);

  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL &= ~(1UL );
}




static inline void ARM_MPU_ClrRegion(uint32_t rnr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = 0U;
}





static inline void ARM_MPU_SetRegion(uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}






static inline void ARM_MPU_SetRegionEx(uint32_t rnr, uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}






static inline void ARM_MPU_OrderedMemcpy(volatile uint32_t* dst, const uint32_t* __restrict src, uint32_t len)
{
  uint32_t i;
  for (i = 0U; i < len; ++i)
  {
    dst[i] = src[i];
  }
}





static inline void ARM_MPU_Load(ARM_MPU_Region_t const* table, uint32_t cnt)
{
  const uint32_t rowWordSize = sizeof(ARM_MPU_Region_t)/4U;
  while (cnt > 4U) {
    ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), 4U*rowWordSize);
    table += 4U;
    cnt -= 4U;
  }
  ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), cnt*rowWordSize);
}
# 1953 "../../lib/cmsis/inc/core_cm4.h" 2
# 1973 "../../lib/cmsis/inc/core_cm4.h"
static inline uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = ((FPU_Type *) ((0xE000E000UL) + 0x0F30UL) )->MVFR0;
  if ((mvfr0 & ((0xFUL << 4U) | (0xFUL << 8U))) == 0x020U)
  {
    return 1U;
  }
  else
  {
    return 0U;
  }
}
# 2014 "../../lib/cmsis/inc/core_cm4.h"
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (uint32_t)(ticks - 1UL);
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 4U) - 1UL);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0UL;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) |
                   (1UL << 1U) |
                   (1UL );
  return (0UL);
}
# 2044 "../../lib/cmsis/inc/core_cm4.h"
extern volatile int32_t ITM_RxBuffer;
# 2056 "../../lib/cmsis/inc/core_cm4.h"
static inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL ) != 0UL) )
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __asm volatile ("nop");
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}
# 2077 "../../lib/cmsis/inc/core_cm4.h"
static inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;

  if (ITM_RxBuffer != ((int32_t)0x5AA55AA5U))
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ((int32_t)0x5AA55AA5U);
  }

  return (ch);
}
# 2097 "../../lib/cmsis/inc/core_cm4.h"
static inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ((int32_t)0x5AA55AA5U))
  {
    return (0);
  }
  else
  {
    return (1);
  }
}
# 152 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/stm32f303xc.h" 2
# 1 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/system_stm32f3xx.h" 1
# 58 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/system_stm32f3xx.h"
extern uint32_t SystemCoreClock;
extern const uint8_t AHBPrescTable[16];
extern const uint8_t APBPrescTable[8];
# 87 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/system_stm32f3xx.h"
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
# 153 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/stm32f303xc.h" 2
# 163 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/stm32f303xc.h"
typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IER;
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  uint32_t RESERVED0;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  uint32_t RESERVED1;
  volatile uint32_t TR1;
  volatile uint32_t TR2;
  volatile uint32_t TR3;
  uint32_t RESERVED2;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t SQR4;
  volatile uint32_t DR;
  uint32_t RESERVED3;
  uint32_t RESERVED4;
  volatile uint32_t JSQR;
  uint32_t RESERVED5[4];
  volatile uint32_t OFR1;
  volatile uint32_t OFR2;
  volatile uint32_t OFR3;
  volatile uint32_t OFR4;
  uint32_t RESERVED6[4];
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  uint32_t RESERVED7[4];
  volatile uint32_t AWD2CR;
  volatile uint32_t AWD3CR;
  uint32_t RESERVED8;
  uint32_t RESERVED9;
  volatile uint32_t DIFSEL;
  volatile uint32_t CALFACT;

} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;
  uint32_t RESERVED;
  volatile uint32_t CCR;
  volatile uint32_t CDR;

} ADC_Common_TypeDef;




typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;




typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;




typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;




typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t RESERVED2;
  volatile uint32_t FS1R;
  uint32_t RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t RESERVED4;
  volatile uint32_t FA1R;
  uint32_t RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[28];
} CAN_TypeDef;




typedef struct
{
  volatile uint32_t CSR;
} COMP_TypeDef;

typedef struct
{
  volatile uint32_t CSR;
} COMP_Common_TypeDef;





typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t IDR;
  uint8_t RESERVED0;
  uint16_t RESERVED1;
  volatile uint32_t CR;
  uint32_t RESERVED2;
  volatile uint32_t INIT;
  volatile uint32_t POL;
} CRC_TypeDef;





typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;
  volatile uint32_t SR;
} DAC_TypeDef;





typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;
  volatile uint32_t APB1FZ;
  volatile uint32_t APB2FZ;
}DBGMCU_TypeDef;





typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;





typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
  uint32_t RESERVED1;
  uint32_t RESERVED2;
  volatile uint32_t IMR2;
  volatile uint32_t EMR2;
  volatile uint32_t RTSR2;
  volatile uint32_t FTSR2;
  volatile uint32_t SWIER2;
  volatile uint32_t PR2;
}EXTI_TypeDef;





typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;

} FLASH_TypeDef;




typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;





typedef struct
{
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
  volatile uint32_t BRR;
}GPIO_TypeDef;





typedef struct
{
  volatile uint32_t CSR;
} OPAMP_TypeDef;





typedef struct
{
  volatile uint32_t CFGR1;
  volatile uint32_t RCR;
  volatile uint32_t EXTICR[4];
  volatile uint32_t CFGR2;
} SYSCFG_TypeDef;





typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t OAR1;
  volatile uint32_t OAR2;
  volatile uint32_t TIMINGR;
  volatile uint32_t TIMEOUTR;
  volatile uint32_t ISR;
  volatile uint32_t ICR;
  volatile uint32_t PECR;
  volatile uint32_t RXDR;
  volatile uint32_t TXDR;
}I2C_TypeDef;





typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
  volatile uint32_t WINR;
} IWDG_TypeDef;





typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;




typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  volatile uint32_t AHBRSTR;
  volatile uint32_t CFGR2;
  volatile uint32_t CFGR3;
} RCC_TypeDef;





typedef struct
{
  volatile uint32_t TR;
  volatile uint32_t DR;
  volatile uint32_t CR;
  volatile uint32_t ISR;
  volatile uint32_t PRER;
  volatile uint32_t WUTR;
  uint32_t RESERVED0;
  volatile uint32_t ALRMAR;
  volatile uint32_t ALRMBR;
  volatile uint32_t WPR;
  volatile uint32_t SSR;
  volatile uint32_t SHIFTR;
  volatile uint32_t TSTR;
  volatile uint32_t TSDR;
  volatile uint32_t TSSSR;
  volatile uint32_t CALR;
  volatile uint32_t TAFCR;
  volatile uint32_t ALRMASSR;
  volatile uint32_t ALRMBSSR;
  uint32_t RESERVED7;
  volatile uint32_t BKP0R;
  volatile uint32_t BKP1R;
  volatile uint32_t BKP2R;
  volatile uint32_t BKP3R;
  volatile uint32_t BKP4R;
  volatile uint32_t BKP5R;
  volatile uint32_t BKP6R;
  volatile uint32_t BKP7R;
  volatile uint32_t BKP8R;
  volatile uint32_t BKP9R;
  volatile uint32_t BKP10R;
  volatile uint32_t BKP11R;
  volatile uint32_t BKP12R;
  volatile uint32_t BKP13R;
  volatile uint32_t BKP14R;
  volatile uint32_t BKP15R;
} RTC_TypeDef;






typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
  volatile uint32_t I2SCFGR;
  volatile uint32_t I2SPR;
} SPI_TypeDef;




typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMCR;
  volatile uint32_t DIER;
  volatile uint32_t SR;
  volatile uint32_t EGR;
  volatile uint32_t CCMR1;
  volatile uint32_t CCMR2;
  volatile uint32_t CCER;
  volatile uint32_t CNT;
  volatile uint32_t PSC;
  volatile uint32_t ARR;
  volatile uint32_t RCR;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  volatile uint32_t BDTR;
  volatile uint32_t DCR;
  volatile uint32_t DMAR;
  volatile uint32_t OR;
  volatile uint32_t CCMR3;
  volatile uint32_t CCR5;
  volatile uint32_t CCR6;
} TIM_TypeDef;




typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t IER;
  volatile uint32_t ICR;
  volatile uint32_t ISR;
  volatile uint32_t IOHCR;
  uint32_t RESERVED1;
  volatile uint32_t IOASCR;
  uint32_t RESERVED2;
  volatile uint32_t IOSCR;
  uint32_t RESERVED3;
  volatile uint32_t IOCCR;
  uint32_t RESERVED4;
  volatile uint32_t IOGCSR;
  volatile uint32_t IOGXCR[8];
} TSC_TypeDef;





typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t BRR;
  volatile uint32_t GTPR;
  volatile uint32_t RTOR;
  volatile uint32_t RQR;
  volatile uint32_t ISR;
  volatile uint32_t ICR;
  volatile uint16_t RDR;
  uint16_t RESERVED1;
  volatile uint16_t TDR;
  uint16_t RESERVED2;
} USART_TypeDef;





typedef struct
{
  volatile uint16_t EP0R;
  volatile uint16_t RESERVED0;
  volatile uint16_t EP1R;
  volatile uint16_t RESERVED1;
  volatile uint16_t EP2R;
  volatile uint16_t RESERVED2;
  volatile uint16_t EP3R;
  volatile uint16_t RESERVED3;
  volatile uint16_t EP4R;
  volatile uint16_t RESERVED4;
  volatile uint16_t EP5R;
  volatile uint16_t RESERVED5;
  volatile uint16_t EP6R;
  volatile uint16_t RESERVED6;
  volatile uint16_t EP7R;
  volatile uint16_t RESERVED7[17];
  volatile uint16_t CNTR;
  volatile uint16_t RESERVED8;
  volatile uint16_t ISTR;
  volatile uint16_t RESERVED9;
  volatile uint16_t FNR;
  volatile uint16_t RESERVEDA;
  volatile uint16_t DADDR;
  volatile uint16_t RESERVEDB;
  volatile uint16_t BTABLE;
  volatile uint16_t RESERVEDC;
} USB_TypeDef;




typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;
# 137 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/stm32f3xx.h" 2
# 164 "../../lib/stm32lib/CMSIS/STM32F3xx/Include/stm32f3xx.h"
typedef enum
{
  RESET = 0U,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0U,
  ENABLE = !DISABLE
} FunctionalState;


typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;
# 31 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_def.h" 2



# 1 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h" 1 3 4
# 143 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h" 3 4

# 143 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h" 3 4
typedef int ptrdiff_t;
# 209 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h" 3 4
typedef unsigned int size_t;
# 321 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h" 3 4
typedef unsigned int wchar_t;
# 35 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_def.h" 2







# 41 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_def.h"
typedef enum
{
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;




typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED = 0x01
} HAL_LockTypeDef;
# 30 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma.h" 2
# 48 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma.h"
typedef struct
{
  uint32_t Direction;



  uint32_t PeriphInc;


  uint32_t MemInc;


  uint32_t PeriphDataAlignment;


  uint32_t MemDataAlignment;


  uint32_t Mode;




  uint32_t Priority;

} DMA_InitTypeDef;




typedef enum
{
  HAL_DMA_STATE_RESET = 0x00U,
  HAL_DMA_STATE_READY = 0x01U,
  HAL_DMA_STATE_BUSY = 0x02U,
  HAL_DMA_STATE_TIMEOUT = 0x03
}HAL_DMA_StateTypeDef;




typedef enum
{
  HAL_DMA_FULL_TRANSFER = 0x00U,
  HAL_DMA_HALF_TRANSFER = 0x01
}HAL_DMA_LevelCompleteTypeDef;




typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID = 0x00U,
  HAL_DMA_XFER_HALFCPLT_CB_ID = 0x01U,
  HAL_DMA_XFER_ERROR_CB_ID = 0x02U,
  HAL_DMA_XFER_ABORT_CB_ID = 0x03U,
  HAL_DMA_XFER_ALL_CB_ID = 0x04
}HAL_DMA_CallbackIDTypeDef;




typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef *Instance;

  DMA_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  HAL_DMA_StateTypeDef State;

  void *Parent;

  void (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);

  void (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);

  void (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);

  void (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);

  volatile uint32_t ErrorCode;

  DMA_TypeDef *DmaBaseAddress;

  uint32_t ChannelIndex;
} DMA_HandleTypeDef;
# 355 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma_ex.h" 1
# 356 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma.h" 2
# 366 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma.h"
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit (DMA_HandleTypeDef *hdma);
# 376 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma.h"
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, uint32_t CompleteLevel, uint32_t Timeout);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)( DMA_HandleTypeDef * _hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);
# 392 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dma.h"
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *hdma);
# 31 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc.h" 1
# 33 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h" 1
# 43 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
struct __ADC_HandleTypeDef;
# 63 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
typedef struct
{
  uint32_t ClockPrescaler;






  uint32_t Resolution;

  uint32_t DataAlign;



  uint32_t ScanConvMode;






  uint32_t EOCSelection;

  FunctionalState LowPowerAutoWait;






  FunctionalState ContinuousConvMode;


  uint32_t NbrOfConversion;



  FunctionalState DiscontinuousConvMode;



  uint32_t NbrOfDiscConversion;


  uint32_t ExternalTrigConv;



  uint32_t ExternalTrigConvEdge;


  FunctionalState DMAContinuousRequests;




  uint32_t Overrun;






}ADC_InitTypeDef;
# 139 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
typedef struct
{
  uint32_t Channel;


  uint32_t Rank;


  uint32_t SamplingTime;
# 156 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
  uint32_t SingleDiff;
# 166 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
  uint32_t OffsetNumber;


  uint32_t Offset;



}ADC_ChannelConfTypeDef;
# 190 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
typedef struct
{
  uint32_t InjectedChannel;


  uint32_t InjectedRank;


  uint32_t InjectedSamplingTime;
# 207 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
  uint32_t InjectedSingleDiff;
# 217 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
  uint32_t InjectedOffsetNumber;


  uint32_t InjectedOffset;



  uint32_t InjectedNbrOfConversion;




  FunctionalState InjectedDiscontinuousConvMode;







  FunctionalState AutoInjectedConv;







  FunctionalState QueueInjectedContext;
# 254 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
  uint32_t ExternalTrigInjecConv;




  uint32_t ExternalTrigInjecConvEdge;




}ADC_InjectionConfTypeDef;




typedef struct
{
  uint32_t ContextQueue;



  uint32_t ChannelCount;
}ADC_InjectionConfigTypeDef;






typedef struct
{
  uint32_t WatchdogNumber;



  uint32_t WatchdogMode;


  uint32_t Channel;




  FunctionalState ITMode;

  uint32_t HighThreshold;



  uint32_t LowThreshold;



}ADC_AnalogWDGConfTypeDef;
# 318 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
typedef struct
{
  uint32_t Mode;

  uint32_t DMAAccessMode;





  uint32_t TwoSamplingDelay;



}ADC_MultiModeTypeDef;
# 3852 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(struct __ADC_HandleTypeDef* hadc, uint32_t SingleDiff);
uint32_t HAL_ADCEx_Calibration_GetValue(struct __ADC_HandleTypeDef *hadc, uint32_t SingleDiff);
HAL_StatusTypeDef HAL_ADCEx_Calibration_SetValue(struct __ADC_HandleTypeDef *hadc, uint32_t SingleDiff, uint32_t CalibrationFactor);
# 3865 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(struct __ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(struct __ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion(struct __ADC_HandleTypeDef* hadc, uint32_t Timeout);


HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(struct __ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(struct __ADC_HandleTypeDef* hadc);






HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(struct __ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(struct __ADC_HandleTypeDef *hadc);
uint32_t HAL_ADCEx_MultiModeGetValue(struct __ADC_HandleTypeDef *hadc);







HAL_StatusTypeDef HAL_ADCEx_RegularStop(struct __ADC_HandleTypeDef* hadc);

HAL_StatusTypeDef HAL_ADCEx_RegularStop_IT(struct __ADC_HandleTypeDef* hadc);

HAL_StatusTypeDef HAL_ADCEx_RegularStop_DMA(struct __ADC_HandleTypeDef* hadc);





HAL_StatusTypeDef HAL_ADCEx_RegularMultiModeStop_DMA(struct __ADC_HandleTypeDef *hadc);






uint32_t HAL_ADCEx_InjectedGetValue(struct __ADC_HandleTypeDef* hadc, uint32_t InjectedRank);


void HAL_ADCEx_InjectedConvCpltCallback(struct __ADC_HandleTypeDef* hadc);





void HAL_ADCEx_InjectedQueueOverflowCallback(struct __ADC_HandleTypeDef* hadc);
void HAL_ADCEx_LevelOutOfWindow2Callback(struct __ADC_HandleTypeDef* hadc);
void HAL_ADCEx_LevelOutOfWindow3Callback(struct __ADC_HandleTypeDef* hadc);
# 3929 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc_ex.h"
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(struct __ADC_HandleTypeDef* hadc,ADC_InjectionConfTypeDef* sConfigInjected);





HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(struct __ADC_HandleTypeDef *hadc, ADC_MultiModeTypeDef *multimode);
# 34 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc.h" 2
# 91 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc.h"
typedef struct __ADC_HandleTypeDef
{
  ADC_TypeDef *Instance;

  ADC_InitTypeDef Init;

  DMA_HandleTypeDef *DMA_Handle;

  HAL_LockTypeDef Lock;

  volatile uint32_t State;

  volatile uint32_t ErrorCode;





  ADC_InjectionConfigTypeDef InjectionConfig ;
# 124 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc.h"
}ADC_HandleTypeDef;
# 189 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc.h"
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);
# 207 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc.h"
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout);


HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);


HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);


uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);


void HAL_ADC_IRQHandler(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);
# 237 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc.h"
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig);
# 248 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_adc.h"
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc);
# 32 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_can.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_can.h"
typedef enum
{
  HAL_CAN_STATE_RESET = 0x00U,
  HAL_CAN_STATE_READY = 0x01U,
  HAL_CAN_STATE_LISTENING = 0x02U,
  HAL_CAN_STATE_SLEEP_PENDING = 0x03U,
  HAL_CAN_STATE_SLEEP_ACTIVE = 0x04U,
  HAL_CAN_STATE_ERROR = 0x05U

} HAL_CAN_StateTypeDef;




typedef struct
{
  uint32_t Prescaler;


  uint32_t Mode;


  uint32_t SyncJumpWidth;



  uint32_t TimeSeg1;


  uint32_t TimeSeg2;


  FunctionalState TimeTriggeredMode;


  FunctionalState AutoBusOff;


  FunctionalState AutoWakeUp;


  FunctionalState AutoRetransmission;


  FunctionalState ReceiveFifoLocked;


  FunctionalState TransmitFifoPriority;


} CAN_InitTypeDef;




typedef struct
{
  uint32_t FilterIdHigh;



  uint32_t FilterIdLow;



  uint32_t FilterMaskIdHigh;




  uint32_t FilterMaskIdLow;




  uint32_t FilterFIFOAssignment;


  uint32_t FilterBank;


  uint32_t FilterMode;


  uint32_t FilterScale;


  uint32_t FilterActivation;


  uint32_t SlaveStartFilterBank;




} CAN_FilterTypeDef;




typedef struct
{
  uint32_t StdId;


  uint32_t ExtId;


  uint32_t IDE;


  uint32_t RTR;


  uint32_t DLC;


  FunctionalState TransmitGlobalTime;





} CAN_TxHeaderTypeDef;




typedef struct
{
  uint32_t StdId;


  uint32_t ExtId;


  uint32_t IDE;


  uint32_t RTR;


  uint32_t DLC;


  uint32_t Timestamp;



  uint32_t FilterMatchIndex;


} CAN_RxHeaderTypeDef;




typedef struct __CAN_HandleTypeDef
{
  CAN_TypeDef *Instance;

  CAN_InitTypeDef Init;

  volatile HAL_CAN_StateTypeDef State;

  volatile uint32_t ErrorCode;
# 234 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_can.h"
} CAN_HandleTypeDef;
# 636 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_can.h"
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan);
# 657 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_can.h"
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig);
# 669 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_can.h"
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_RequestSleep(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsSleepActive(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxTimestamp(CAN_HandleTypeDef *hcan, uint32_t TxMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan, uint32_t RxFifo);
# 691 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_can.h"
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan);
# 705 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_can.h"
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
# 728 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_can.h"
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan);
# 33 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_cortex.h" 1
# 48 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_cortex.h"
typedef struct
{
  uint8_t Enable;

  uint8_t Number;

  uint32_t BaseAddress;
  uint8_t Size;

  uint8_t SubRegionDisable;

  uint8_t TypeExtField;

  uint8_t AccessPermission;

  uint8_t DisableExec;

  uint8_t IsShareable;

  uint8_t IsCacheable;

  uint8_t IsBufferable;

}MPU_Region_InitTypeDef;
# 260 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_cortex.h"
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);
# 275 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_cortex.h"
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);

uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);
# 402 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_cortex.h"
void HAL_MPU_Disable(void);
void HAL_MPU_Enable(uint32_t MPU_Control);
# 34 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_crc.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_crc.h"
typedef enum
{
  HAL_CRC_STATE_RESET = 0x00U,
  HAL_CRC_STATE_READY = 0x01U,
  HAL_CRC_STATE_BUSY = 0x02U,
  HAL_CRC_STATE_TIMEOUT = 0x03U,
  HAL_CRC_STATE_ERROR = 0x04U
} HAL_CRC_StateTypeDef;




typedef struct
{
  uint8_t DefaultPolynomialUse;





  uint8_t DefaultInitValueUse;




  uint32_t GeneratingPolynomial;




  uint32_t CRCLength;






  uint32_t InitValue;


  uint32_t InputDataInversionMode;






  uint32_t OutputDataInversionMode;



} CRC_InitTypeDef;




typedef struct
{
  CRC_TypeDef *Instance;

  CRC_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  volatile HAL_CRC_StateTypeDef State;

  uint32_t InputDataFormat;







} CRC_HandleTypeDef;
# 287 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_crc.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_crc_ex.h" 1
# 127 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_crc_ex.h"
HAL_StatusTypeDef HAL_CRCEx_Polynomial_Set(CRC_HandleTypeDef *hcrc, uint32_t Pol, uint32_t PolyLength);
HAL_StatusTypeDef HAL_CRCEx_Input_Data_Reverse(CRC_HandleTypeDef *hcrc, uint32_t InputReverseMode);
HAL_StatusTypeDef HAL_CRCEx_Output_Data_Reverse(CRC_HandleTypeDef *hcrc, uint32_t OutputReverseMode);
# 288 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_crc.h" 2
# 298 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_crc.h"
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);
# 310 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_crc.h"
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
# 320 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_crc.h"
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc);
# 35 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac.h" 1
# 48 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac.h"
typedef enum
{
  HAL_DAC_STATE_RESET = 0x00U,
  HAL_DAC_STATE_READY = 0x01U,
  HAL_DAC_STATE_BUSY = 0x02U,
  HAL_DAC_STATE_TIMEOUT = 0x03U,
  HAL_DAC_STATE_ERROR = 0x04U

}HAL_DAC_StateTypeDef;




typedef struct
{
  uint32_t DAC_Trigger;


  uint32_t DAC_OutputBuffer;




  uint32_t DAC_OutputSwitch;




}DAC_ChannelConfTypeDef;




typedef struct __DAC_HandleTypeDef
{
  DAC_TypeDef *Instance;

  volatile HAL_DAC_StateTypeDef State;

  HAL_LockTypeDef Lock;

  DMA_HandleTypeDef *DMA_Handle1;

  DMA_HandleTypeDef *DMA_Handle2;

  volatile uint32_t ErrorCode;
# 109 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac.h"
}DAC_HandleTypeDef;
# 408 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac_ex.h" 1
# 303 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac_ex.h"
uint32_t HAL_DACEx_DualGetValue(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DACEx_DualSetValue(DAC_HandleTypeDef* hdac, uint32_t Alignment, uint32_t Data1, uint32_t Data2);
HAL_StatusTypeDef HAL_DACEx_TriangleWaveGenerate(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Amplitude);
HAL_StatusTypeDef HAL_DACEx_NoiseWaveGenerate(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Amplitude);





void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_ErrorCallbackCh2(DAC_HandleTypeDef *hdac);
void HAL_DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef *hdac);
# 409 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac.h" 2
# 420 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac.h"
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac);
# 433 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac.h"
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel);
uint32_t HAL_DAC_GetValue(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef* hdac, DAC_ChannelConfTypeDef* sConfig, uint32_t Channel);

void HAL_DAC_IRQHandler(DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);
# 460 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac.h"
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);
# 470 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_dac.h"
HAL_DAC_StateTypeDef HAL_DAC_GetState(DAC_HandleTypeDef* hdac);
uint32_t HAL_DAC_GetError(DAC_HandleTypeDef *hdac);
# 36 "./boards/stm32f3xx_hal_conf_base.h" 2


# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash.h" 1
# 71 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash.h"
typedef enum
{
  FLASH_PROC_NONE = 0U,
  FLASH_PROC_PAGEERASE = 1U,
  FLASH_PROC_MASSERASE = 2U,
  FLASH_PROC_PROGRAMHALFWORD = 3U,
  FLASH_PROC_PROGRAMWORD = 4U,
  FLASH_PROC_PROGRAMDOUBLEWORD = 5U
} FLASH_ProcedureTypeDef;




typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;

  volatile uint32_t DataRemaining;

  volatile uint32_t Address;

  volatile uint64_t Data;

  HAL_LockTypeDef Lock;

  volatile uint32_t ErrorCode;

} FLASH_ProcessTypeDef;
# 304 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash_ex.h" 1
# 137 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash_ex.h"
typedef struct
{
  uint32_t TypeErase;


  uint32_t PageAddress;


  uint32_t NbPages;


} FLASH_EraseInitTypeDef;




typedef struct
{
  uint32_t OptionType;


  uint32_t WRPState;


  uint32_t WRPPage;


  uint8_t RDPLevel;


  uint8_t USERConfig;
# 180 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash_ex.h"
  uint32_t DATAAddress;


  uint8_t DATAData;

} FLASH_OBProgramInitTypeDef;
# 436 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash_ex.h"
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
# 447 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash_ex.h"
HAL_StatusTypeDef HAL_FLASHEx_OBErase(void);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);
uint32_t HAL_FLASHEx_OBGetUserData(uint32_t DATAAdress);
# 305 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash.h" 2
# 315 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash.h"
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);


void HAL_FLASH_IRQHandler(void);

void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);
# 332 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash.h"
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);
# 346 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash.h"
uint32_t HAL_FLASH_GetError(void);
# 360 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_flash.h"
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
# 39 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_gpio.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_gpio.h"
typedef struct
{
  uint32_t Pin;


  uint32_t Mode;


  uint32_t Pull;


  uint32_t Speed;


  uint32_t Alternate;

}GPIO_InitTypeDef;




typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
}GPIO_PinState;
# 242 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_gpio.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_gpio_ex.h" 1
# 243 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_gpio.h" 2
# 255 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_gpio.h"
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
# 267 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_gpio.h"
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
# 40 "./boards/stm32f3xx_hal_conf_base.h" 2


# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h" 1
# 48 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
typedef struct
{
  uint32_t Timing;



  uint32_t OwnAddress1;


  uint32_t AddressingMode;


  uint32_t DualAddressMode;


  uint32_t OwnAddress2;


  uint32_t OwnAddress2Masks;


  uint32_t GeneralCallMode;


  uint32_t NoStretchMode;


} I2C_InitTypeDef;
# 108 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
typedef enum
{
  HAL_I2C_STATE_RESET = 0x00U,
  HAL_I2C_STATE_READY = 0x20U,
  HAL_I2C_STATE_BUSY = 0x24U,
  HAL_I2C_STATE_BUSY_TX = 0x21U,
  HAL_I2C_STATE_BUSY_RX = 0x22U,
  HAL_I2C_STATE_LISTEN = 0x28U,
  HAL_I2C_STATE_BUSY_TX_LISTEN = 0x29U,

  HAL_I2C_STATE_BUSY_RX_LISTEN = 0x2AU,

  HAL_I2C_STATE_ABORT = 0x60U,
  HAL_I2C_STATE_TIMEOUT = 0xA0U,
  HAL_I2C_STATE_ERROR = 0xE0U

} HAL_I2C_StateTypeDef;
# 148 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
typedef enum
{
  HAL_I2C_MODE_NONE = 0x00U,
  HAL_I2C_MODE_MASTER = 0x10U,
  HAL_I2C_MODE_SLAVE = 0x20U,
  HAL_I2C_MODE_MEM = 0x40U

} HAL_I2C_ModeTypeDef;
# 186 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
typedef struct __I2C_HandleTypeDef
{
  I2C_TypeDef *Instance;

  I2C_InitTypeDef Init;

  uint8_t *pBuffPtr;

  uint16_t XferSize;

  volatile uint16_t XferCount;

  volatile uint32_t XferOptions;


  volatile uint32_t PreviousState;

  HAL_StatusTypeDef(*XferISR)(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  volatile HAL_I2C_StateTypeDef State;

  volatile HAL_I2C_ModeTypeDef Mode;

  volatile uint32_t ErrorCode;

  volatile uint32_t AddrEventCount;
# 236 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
} I2C_HandleTypeDef;
# 568 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c_ex.h" 1
# 95 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c_ex.h"
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);
HAL_StatusTypeDef HAL_I2CEx_EnableWakeUp(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2CEx_DisableWakeUp(I2C_HandleTypeDef *hi2c);
void HAL_I2CEx_EnableFastModePlus(uint32_t ConfigFastModePlus);
void HAL_I2CEx_DisableFastModePlus(uint32_t ConfigFastModePlus);
# 569 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h" 2
# 579 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);
# 601 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);


HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);


HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
# 645 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);
# 665 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2c.h"
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);
# 43 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s.h" 1
# 48 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s.h"
typedef struct
{
  uint32_t Mode;


  uint32_t Standard;


  uint32_t DataFormat;


  uint32_t MCLKOutput;


  uint32_t AudioFreq;


  uint32_t CPOL;


  uint32_t ClockSource;

  uint32_t FullDuplexMode;

} I2S_InitTypeDef;




typedef enum
{
  HAL_I2S_STATE_RESET = 0x00U,
  HAL_I2S_STATE_READY = 0x01U,
  HAL_I2S_STATE_BUSY = 0x02U,
  HAL_I2S_STATE_BUSY_TX = 0x03U,
  HAL_I2S_STATE_BUSY_RX = 0x04U,
  HAL_I2S_STATE_BUSY_TX_RX = 0x05U,
  HAL_I2S_STATE_TIMEOUT = 0x06U,
  HAL_I2S_STATE_ERROR = 0x07U
} HAL_I2S_StateTypeDef;




typedef struct __I2S_HandleTypeDef
{
  SPI_TypeDef *Instance;

  I2S_InitTypeDef Init;

  uint16_t *pTxBuffPtr;

  volatile uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint16_t *pRxBuffPtr;

  volatile uint16_t RxXferSize;

  volatile uint16_t RxXferCount;





  void (*IrqHandlerISR)(struct __I2S_HandleTypeDef *hi2s);

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  volatile HAL_LockTypeDef Lock;

  volatile HAL_I2S_StateTypeDef State;

  volatile uint32_t ErrorCode;
# 139 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s.h"
} I2S_HandleTypeDef;
# 413 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s_ex.h" 1
# 129 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s_ex.h"
HAL_StatusTypeDef HAL_I2SEx_TransmitReceive(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData,
                                            uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_I2SEx_TransmitReceive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData,
                                               uint16_t Size);

HAL_StatusTypeDef HAL_I2SEx_TransmitReceive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData,
                                                uint16_t Size);

void HAL_I2SEx_FullDuplex_IRQHandler(I2S_HandleTypeDef *hi2s);
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s);
# 414 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s.h" 2
# 424 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s.h"
HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DeInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);
# 444 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s.h"
HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);


HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
void HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s);


HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s);


void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);
# 474 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_i2s.h"
HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s);
uint32_t HAL_I2S_GetError(I2S_HandleTypeDef *hi2s);
# 44 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_iwdg.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_iwdg.h"
typedef struct
{
  uint32_t Prescaler;


  uint32_t Reload;


  uint32_t Window;


} IWDG_InitTypeDef;




typedef struct
{
  IWDG_TypeDef *Instance;

  IWDG_InitTypeDef Init;
} IWDG_HandleTypeDef;
# 141 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_iwdg.h"
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg);
# 150 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_iwdg.h"
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);
# 45 "./boards/stm32f3xx_hal_conf_base.h" 2

# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h" 1
# 29 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usb.h" 1
# 48 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usb.h"
typedef enum
{
  USB_DEVICE_MODE = 0
} USB_ModeTypeDef;




typedef struct
{
  uint32_t dev_endpoints;



  uint32_t speed;


  uint32_t ep0_mps;

  uint32_t phy_itface;


  uint32_t Sof_enable;

  uint32_t low_power_enable;

  uint32_t lpm_enable;

  uint32_t battery_charging_enable;
} USB_CfgTypeDef;

typedef struct
{
  uint8_t num;


  uint8_t is_in;


  uint8_t is_stall;


  uint8_t type;


  uint8_t data_pid_start;


  uint16_t pmaadress;


  uint16_t pmaaddr0;


  uint16_t pmaaddr1;


  uint8_t doublebuffer;


  uint16_t tx_fifo_num;



  uint32_t maxpacket;


  uint8_t *xfer_buff;

  uint32_t xfer_len;

  uint32_t xfer_count;

} USB_EPTypeDef;
# 192 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usb.h"
HAL_StatusTypeDef USB_CoreInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_SetCurrentMode(USB_TypeDef *USBx, USB_ModeTypeDef mode);
HAL_StatusTypeDef USB_SetDevSpeed(USB_TypeDef *USBx, uint8_t speed);
HAL_StatusTypeDef USB_FlushRxFifo(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_FlushTxFifo(USB_TypeDef *USBx, uint32_t num);
HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_WritePacket(USB_TypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len);
void *USB_ReadPacket(USB_TypeDef *USBx, uint8_t *dest, uint16_t len);
HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_SetDevAddress(USB_TypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(USB_TypeDef *USBx, uint8_t *psetup);
uint32_t USB_ReadInterrupts(USB_TypeDef *USBx);
uint32_t USB_ReadDevAllOutEpInterrupt(USB_TypeDef *USBx);
uint32_t USB_ReadDevOutEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
uint32_t USB_ReadDevAllInEpInterrupt(USB_TypeDef *USBx);
uint32_t USB_ReadDevInEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
void USB_ClearInterrupts(USB_TypeDef *USBx, uint32_t interrupt);

HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_TypeDef *USBx);
void USB_WritePMA(USB_TypeDef *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);
void USB_ReadPMA(USB_TypeDef *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);
# 30 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h" 2
# 49 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h"
typedef enum
{
  HAL_PCD_STATE_RESET = 0x00,
  HAL_PCD_STATE_READY = 0x01,
  HAL_PCD_STATE_ERROR = 0x02,
  HAL_PCD_STATE_BUSY = 0x03,
  HAL_PCD_STATE_TIMEOUT = 0x04
} PCD_StateTypeDef;


typedef enum
{
  LPM_L0 = 0x00,
  LPM_L1 = 0x01,
  LPM_L2 = 0x02,
  LPM_L3 = 0x03,
} PCD_LPM_StateTypeDef;

typedef enum
{
  PCD_LPM_L0_ACTIVE = 0x00,
  PCD_LPM_L1_ACTIVE = 0x01,
} PCD_LPM_MsgTypeDef;

typedef enum
{
  PCD_BCD_ERROR = 0xFF,
  PCD_BCD_CONTACT_DETECTION = 0xFE,
  PCD_BCD_STD_DOWNSTREAM_PORT = 0xFD,
  PCD_BCD_CHARGING_DOWNSTREAM_PORT = 0xFC,
  PCD_BCD_DEDICATED_CHARGING_PORT = 0xFB,
  PCD_BCD_DISCOVERY_COMPLETED = 0x00,

} PCD_BCD_MsgTypeDef;





typedef USB_TypeDef PCD_TypeDef;
typedef USB_CfgTypeDef PCD_InitTypeDef;
typedef USB_EPTypeDef PCD_EPTypeDef;
# 99 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h"
typedef struct

{
  PCD_TypeDef *Instance;
  PCD_InitTypeDef Init;
  volatile uint8_t USB_Address;
  PCD_EPTypeDef IN_ep[8];
  PCD_EPTypeDef OUT_ep[8];
  HAL_LockTypeDef Lock;
  volatile PCD_StateTypeDef State;
  volatile uint32_t ErrorCode;
  uint32_t Setup[12];
  PCD_LPM_StateTypeDef LPM_State;
  uint32_t BESL;

  void *pData;
# 133 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h"
} PCD_HandleTypeDef;






# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd_ex.h" 1
# 52 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd_ex.h"
HAL_StatusTypeDef HAL_PCDEx_PMAConfig(PCD_HandleTypeDef *hpcd,
                                       uint16_t ep_addr,
                                       uint16_t ep_kind,
                                       uint32_t pmaadress);

void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state);

void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);
void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);
# 141 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h" 2
# 219 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h"
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);
# 287 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h"
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd);
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
# 311 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h"
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
uint32_t HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);
# 332 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pcd.h"
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef *hpcd);
# 47 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pwr.h" 1
# 156 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pwr.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pwr_ex.h" 1
# 52 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pwr_ex.h"
typedef struct
{
  uint32_t PVDLevel;


  uint32_t Mode;

}PWR_PVDTypeDef;
# 284 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pwr_ex.h"
void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);
void HAL_PWR_PVD_IRQHandler(void);
void HAL_PWR_PVDCallback(void);
# 157 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pwr.h" 2
# 169 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pwr.h"
void HAL_PWR_DeInit(void);
# 180 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_pwr.h"
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);


void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinx);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);


void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);
# 48 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rcc.h" 1
# 251 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rcc.h"
typedef struct
{
  uint32_t PLLState;


  uint32_t PLLSource;


  uint32_t PLLMUL;







} RCC_PLLInitTypeDef;




typedef struct
{
  uint32_t OscillatorType;


  uint32_t HSEState;



  uint32_t HSEPredivValue;



  uint32_t LSEState;


  uint32_t HSIState;


  uint32_t HSICalibrationValue;


  uint32_t LSIState;


  RCC_PLLInitTypeDef PLL;

} RCC_OscInitTypeDef;




typedef struct
{
  uint32_t ClockType;


  uint32_t SYSCLKSource;


  uint32_t AHBCLKDivider;


  uint32_t APB1CLKDivider;


  uint32_t APB2CLKDivider;

} RCC_ClkInitTypeDef;
# 1678 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rcc.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rcc_ex.h" 1
# 486 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rcc_ex.h"
typedef struct
{
  uint32_t PeriphClockSelection;


  uint32_t RTCClockSelection;


  uint32_t Usart1ClockSelection;


  uint32_t Usart2ClockSelection;


  uint32_t Usart3ClockSelection;


  uint32_t Uart4ClockSelection;


  uint32_t Uart5ClockSelection;


  uint32_t I2c1ClockSelection;


  uint32_t I2c2ClockSelection;


  uint32_t Adc12ClockSelection;


  uint32_t Adc34ClockSelection;


  uint32_t I2sClockSelection;


  uint32_t Tim1ClockSelection;


  uint32_t Tim8ClockSelection;


  uint32_t USBClockSelection;


}RCC_PeriphCLKInitTypeDef;
# 3800 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rcc_ex.h"
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit);
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);
# 1679 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rcc.h" 2
# 1690 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rcc.h"
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);
# 1703 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rcc.h"
void HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void HAL_RCC_EnableCSS(void);

void HAL_RCC_NMI_IRQHandler(void);

void HAL_RCC_CSSCallback(void);
void HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);
# 49 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h"
typedef enum
{
  HAL_RTC_STATE_RESET = 0x00U,
  HAL_RTC_STATE_READY = 0x01U,
  HAL_RTC_STATE_BUSY = 0x02U,
  HAL_RTC_STATE_TIMEOUT = 0x03U,
  HAL_RTC_STATE_ERROR = 0x04U

} HAL_RTCStateTypeDef;




typedef struct
{
  uint32_t HourFormat;


  uint32_t AsynchPrediv;


  uint32_t SynchPrediv;


  uint32_t OutPut;


  uint32_t OutPutPolarity;


  uint32_t OutPutType;

} RTC_InitTypeDef;




typedef struct
{
  uint8_t Hours;



  uint8_t Minutes;


  uint8_t Seconds;


  uint8_t TimeFormat;


  uint32_t SubSeconds;



  uint32_t SecondFraction;





  uint32_t DayLightSaving;


  uint32_t StoreOperation;


} RTC_TimeTypeDef;




typedef struct
{
  uint8_t WeekDay;


  uint8_t Month;


  uint8_t Date;


  uint8_t Year;


} RTC_DateTypeDef;




typedef struct
{
  RTC_TimeTypeDef AlarmTime;

  uint32_t AlarmMask;


  uint32_t AlarmSubSecondMask;


  uint32_t AlarmDateWeekDaySel;


  uint8_t AlarmDateWeekDay;



  uint32_t Alarm;

} RTC_AlarmTypeDef;







typedef struct

{
  RTC_TypeDef *Instance;

  RTC_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  volatile HAL_RTCStateTypeDef State;
# 200 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h"
} RTC_HandleTypeDef;
# 673 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc_ex.h" 1
# 48 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc_ex.h"
typedef struct
{
  uint32_t Tamper;


  uint32_t Trigger;


  uint32_t Filter;


  uint32_t SamplingFrequency;


  uint32_t PrechargeDuration;


  uint32_t TamperPullUp;


  uint32_t TimeStampOnTamperDetection;

} RTC_TamperTypeDef;
# 837 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc_ex.h"
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp_IT(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format);

HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTamper(RTC_HandleTypeDef *hrtc, uint32_t Tamper);
void HAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc);

void HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_Tamper2EventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_Tamper3EventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForTimeStampEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper2Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper3Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
# 864 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc_ex.h"
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
uint32_t HAL_RTCEx_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc);
uint32_t HAL_RTCEx_GetWakeUpTimer(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerIRQHandler(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForWakeUpTimerEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
# 880 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc_ex.h"
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);

HAL_StatusTypeDef HAL_RTCEx_SetSmoothCalib(RTC_HandleTypeDef *hrtc, uint32_t SmoothCalibPeriod, uint32_t SmoothCalibPlusPulses, uint32_t SmoothCalibMinusPulsesValue);
HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(RTC_HandleTypeDef *hrtc, uint32_t ShiftAdd1S, uint32_t ShiftSubFS);
HAL_StatusTypeDef HAL_RTCEx_SetCalibrationOutPut(RTC_HandleTypeDef *hrtc, uint32_t CalibOutput);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCalibrationOutPut(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DisableBypassShadow(RTC_HandleTypeDef *hrtc);
# 899 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc_ex.h"
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForAlarmBEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
# 674 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h" 2
# 685 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h"
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);
# 704 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h"
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
# 716 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h"
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
# 731 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h"
HAL_StatusTypeDef HAL_RTC_WaitForSynchro(RTC_HandleTypeDef *hrtc);
# 740 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h"
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);
# 872 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_rtc.h"
HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef *hrtc);
uint8_t RTC_ByteToBcd2(uint8_t Value);
uint8_t RTC_Bcd2ToByte(uint8_t Value);
# 50 "./boards/stm32f3xx_hal_conf_base.h" 2


# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi.h"
typedef struct
{
  uint32_t Mode;


  uint32_t Direction;


  uint32_t DataSize;


  uint32_t CLKPolarity;


  uint32_t CLKPhase;


  uint32_t NSS;



  uint32_t BaudRatePrescaler;





  uint32_t FirstBit;


  uint32_t TIMode;


  uint32_t CRCCalculation;


  uint32_t CRCPolynomial;


  uint32_t CRCLength;



  uint32_t NSSPMode;





} SPI_InitTypeDef;




typedef enum
{
  HAL_SPI_STATE_RESET = 0x00U,
  HAL_SPI_STATE_READY = 0x01U,
  HAL_SPI_STATE_BUSY = 0x02U,
  HAL_SPI_STATE_BUSY_TX = 0x03U,
  HAL_SPI_STATE_BUSY_RX = 0x04U,
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,
  HAL_SPI_STATE_ERROR = 0x06U,
  HAL_SPI_STATE_ABORT = 0x07U
} HAL_SPI_StateTypeDef;




typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef *Instance;

  SPI_InitTypeDef Init;

  uint8_t *pTxBuffPtr;

  uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint8_t *pRxBuffPtr;

  uint16_t RxXferSize;

  volatile uint16_t RxXferCount;

  uint32_t CRCSize;

  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);

  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  volatile HAL_SPI_StateTypeDef State;

  volatile uint32_t ErrorCode;
# 163 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi.h"
} SPI_HandleTypeDef;
# 759 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi_ex.h" 1
# 52 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi_ex.h"
HAL_StatusTypeDef HAL_SPIEx_FlushRxFifo(SPI_HandleTypeDef *hspi);
# 760 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi.h" 2
# 770 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi.h"
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);
# 788 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi.h"
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi);
# 824 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_spi.h"
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t HAL_SPI_GetError(SPI_HandleTypeDef *hspi);
# 53 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
typedef struct
{
  uint32_t Prescaler;


  uint32_t CounterMode;


  uint32_t Period;



  uint32_t ClockDivision;


  uint32_t RepetitionCounter;
# 71 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
  uint32_t AutoReloadPreload;

} TIM_Base_InitTypeDef;




typedef struct
{
  uint32_t OCMode;


  uint32_t Pulse;


  uint32_t OCPolarity;


  uint32_t OCNPolarity;



  uint32_t OCFastMode;




  uint32_t OCIdleState;



  uint32_t OCNIdleState;


} TIM_OC_InitTypeDef;




typedef struct
{
  uint32_t OCMode;


  uint32_t Pulse;


  uint32_t OCPolarity;


  uint32_t OCNPolarity;



  uint32_t OCIdleState;



  uint32_t OCNIdleState;



  uint32_t ICPolarity;


  uint32_t ICSelection;


  uint32_t ICFilter;

} TIM_OnePulse_InitTypeDef;




typedef struct
{
  uint32_t ICPolarity;


  uint32_t ICSelection;


  uint32_t ICPrescaler;


  uint32_t ICFilter;

} TIM_IC_InitTypeDef;




typedef struct
{
  uint32_t EncoderMode;


  uint32_t IC1Polarity;


  uint32_t IC1Selection;


  uint32_t IC1Prescaler;


  uint32_t IC1Filter;


  uint32_t IC2Polarity;


  uint32_t IC2Selection;


  uint32_t IC2Prescaler;


  uint32_t IC2Filter;

} TIM_Encoder_InitTypeDef;




typedef struct
{
  uint32_t ClockSource;

  uint32_t ClockPolarity;

  uint32_t ClockPrescaler;

  uint32_t ClockFilter;

} TIM_ClockConfigTypeDef;




typedef struct
{
  uint32_t ClearInputState;

  uint32_t ClearInputSource;

  uint32_t ClearInputPolarity;

  uint32_t ClearInputPrescaler;

  uint32_t ClearInputFilter;

} TIM_ClearInputConfigTypeDef;






typedef struct
{
  uint32_t MasterOutputTrigger;


  uint32_t MasterOutputTrigger2;


  uint32_t MasterSlaveMode;






} TIM_MasterConfigTypeDef;




typedef struct
{
  uint32_t SlaveMode;

  uint32_t InputTrigger;

  uint32_t TriggerPolarity;

  uint32_t TriggerPrescaler;

  uint32_t TriggerFilter;


} TIM_SlaveConfigTypeDef;






typedef struct
{
  uint32_t OffStateRunMode;

  uint32_t OffStateIDLEMode;

  uint32_t LockLevel;

  uint32_t DeadTime;

  uint32_t BreakState;

  uint32_t BreakPolarity;

  uint32_t BreakFilter;


  uint32_t Break2State;

  uint32_t Break2Polarity;

  uint32_t Break2Filter;


  uint32_t AutomaticOutput;

} TIM_BreakDeadTimeConfigTypeDef;




typedef enum
{
  HAL_TIM_STATE_RESET = 0x00U,
  HAL_TIM_STATE_READY = 0x01U,
  HAL_TIM_STATE_BUSY = 0x02U,
  HAL_TIM_STATE_TIMEOUT = 0x03U,
  HAL_TIM_STATE_ERROR = 0x04U
} HAL_TIM_StateTypeDef;




typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1 = 0x01U,
  HAL_TIM_ACTIVE_CHANNEL_2 = 0x02U,
  HAL_TIM_ACTIVE_CHANNEL_3 = 0x04U,
  HAL_TIM_ACTIVE_CHANNEL_4 = 0x08U,

  HAL_TIM_ACTIVE_CHANNEL_5 = 0x10U,


  HAL_TIM_ACTIVE_CHANNEL_6 = 0x20U,

  HAL_TIM_ACTIVE_CHANNEL_CLEARED = 0x00U
} HAL_TIM_ActiveChannel;







typedef struct

{
  TIM_TypeDef *Instance;
  TIM_Base_InitTypeDef Init;
  HAL_TIM_ActiveChannel Channel;
  DMA_HandleTypeDef *hdma[7];

  HAL_LockTypeDef Lock;
  volatile HAL_TIM_StateTypeDef State;
# 378 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
} TIM_HandleTypeDef;
# 2090 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h" 1
# 48 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h"
typedef struct
{
  uint32_t IC1Polarity;


  uint32_t IC1Prescaler;


  uint32_t IC1Filter;


  uint32_t Commutation_Delay;

} TIM_HallSensor_InitTypeDef;
# 189 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, TIM_HallSensor_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);


HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);
# 214 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);


HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);


HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 234 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);


HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 253 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
# 268 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent(TIM_HandleTypeDef *htim, uint32_t InputTrigger,
                                              uint32_t CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_IT(TIM_HandleTypeDef *htim, uint32_t InputTrigger,
                                                 uint32_t CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_DMA(TIM_HandleTypeDef *htim, uint32_t InputTrigger,
                                                  uint32_t CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim,
                                                        TIM_MasterConfigTypeDef *sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim,
                                                TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);

HAL_StatusTypeDef HAL_TIMEx_GroupChannel5(TIM_HandleTypeDef *htim, uint32_t Channels);

HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef *htim, uint32_t Remap);
# 291 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h"
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_CommutHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);

void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim);
# 306 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h"
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef *htim);
# 320 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim_ex.h"
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);
# 2091 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h" 2
# 2102 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);
# 2124 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 2146 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 2168 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 2190 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
# 2209 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim, TIM_Encoder_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1,
                                            uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 2232 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
# 2242 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef *sConfig,
                                                 uint32_t OutputChannel, uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef *sClearInputConfig,
                                           uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef *sClockSourceConfig);
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                              uint32_t BurstRequestSrc, uint32_t *BurstBuffer, uint32_t BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_MultiWriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                                   uint32_t BurstRequestSrc, uint32_t *BurstBuffer, uint32_t BurstLength,
                                                   uint32_t DataLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                             uint32_t BurstRequestSrc, uint32_t *BurstBuffer, uint32_t BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_MultiReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                                  uint32_t BurstRequestSrc, uint32_t *BurstBuffer, uint32_t BurstLength,
                                                  uint32_t DataLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);
# 2276 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);
# 2303 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);
# 2322 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_tim.h"
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_ETR_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter);

void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);
# 54 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
typedef struct
{
  uint32_t BaudRate;
# 59 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
  uint32_t WordLength;


  uint32_t StopBits;


  uint32_t Parity;






  uint32_t Mode;


  uint32_t HwFlowCtl;



  uint32_t OverSampling;


  uint32_t OneBitSampling;




} UART_InitTypeDef;




typedef struct
{
  uint32_t AdvFeatureInit;



  uint32_t TxPinLevelInvert;


  uint32_t RxPinLevelInvert;


  uint32_t DataInvert;



  uint32_t Swap;


  uint32_t OverrunDisable;


  uint32_t DMADisableonRxError;


  uint32_t AutoBaudRateEnable;


  uint32_t AutoBaudRateMode;



  uint32_t MSBFirst;

} UART_AdvFeatureInitTypeDef;
# 167 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
typedef uint32_t HAL_UART_StateTypeDef;




typedef enum
{
  UART_CLOCKSOURCE_PCLK1 = 0x00U,
  UART_CLOCKSOURCE_PCLK2 = 0x01U,
  UART_CLOCKSOURCE_HSI = 0x02U,
  UART_CLOCKSOURCE_SYSCLK = 0x04U,
  UART_CLOCKSOURCE_LSE = 0x08U,
  UART_CLOCKSOURCE_UNDEFINED = 0x10U
} UART_ClockSourceTypeDef;




typedef struct __UART_HandleTypeDef
{
  USART_TypeDef *Instance;

  UART_InitTypeDef Init;

  UART_AdvFeatureInitTypeDef AdvancedInit;

  uint8_t *pTxBuffPtr;

  uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint8_t *pRxBuffPtr;

  uint16_t RxXferSize;

  volatile uint16_t RxXferCount;

  uint16_t Mask;

  void (*RxISR)(struct __UART_HandleTypeDef *huart);

  void (*TxISR)(struct __UART_HandleTypeDef *huart);

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  volatile HAL_UART_StateTypeDef gState;



  volatile HAL_UART_StateTypeDef RxState;


  volatile uint32_t ErrorCode;
# 241 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
} UART_HandleTypeDef;
# 1365 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart_ex.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart_ex.h"
typedef struct
{
  uint32_t WakeUpEvent;




  uint16_t AddressLength;


  uint8_t Address;
} UART_WakeUpTypeDef;
# 109 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart_ex.h"
HAL_StatusTypeDef HAL_RS485Ex_Init(UART_HandleTypeDef *huart, uint32_t Polarity, uint32_t AssertionTime,
                                   uint32_t DeassertionTime);
# 120 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart_ex.h"
void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart);
# 132 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart_ex.h"
HAL_StatusTypeDef HAL_UARTEx_StopModeWakeUpSourceConfig(UART_HandleTypeDef *huart, UART_WakeUpTypeDef WakeUpSelection);
HAL_StatusTypeDef HAL_UARTEx_EnableStopMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_DisableStopMode(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_MultiProcessorEx_AddressLength_Set(UART_HandleTypeDef *huart, uint32_t AddressLength);
# 1366 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h" 2
# 1378 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);
# 1402 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);
# 1438 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
void HAL_UART_ReceiverTimeout_Config(UART_HandleTypeDef *huart, uint32_t TimeoutValue);
HAL_StatusTypeDef HAL_UART_EnableReceiverTimeout(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DisableReceiverTimeout(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnableMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_DisableMuteMode(UART_HandleTypeDef *huart);
void HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);
# 1458 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *huart);
# 1476 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_uart.h"
HAL_StatusTypeDef UART_SetConfig(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                              uint32_t Tickstart, uint32_t Timeout);
void UART_AdvFeatureConfig(UART_HandleTypeDef *huart);
# 55 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_usart.h" 1
# 47 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_usart.h"
typedef struct
{
  uint32_t BaudRate;







  uint32_t WordLength;


  uint32_t StopBits;


  uint32_t Parity;






  uint32_t Mode;


  uint32_t CLKPolarity;


  uint32_t CLKPhase;


  uint32_t CLKLastBit;



} USART_InitTypeDef;




typedef enum
{
  HAL_USART_STATE_RESET = 0x00U,
  HAL_USART_STATE_READY = 0x01U,
  HAL_USART_STATE_BUSY = 0x02U,
  HAL_USART_STATE_BUSY_TX = 0x12U,
  HAL_USART_STATE_BUSY_RX = 0x22U,
  HAL_USART_STATE_BUSY_TX_RX = 0x32U,
  HAL_USART_STATE_TIMEOUT = 0x03U,
  HAL_USART_STATE_ERROR = 0x04U
} HAL_USART_StateTypeDef;




typedef enum
{
  USART_CLOCKSOURCE_PCLK1 = 0x00U,
  USART_CLOCKSOURCE_PCLK2 = 0x01U,
  USART_CLOCKSOURCE_HSI = 0x02U,
  USART_CLOCKSOURCE_SYSCLK = 0x04U,
  USART_CLOCKSOURCE_LSE = 0x08U,
  USART_CLOCKSOURCE_UNDEFINED = 0x10U
} USART_ClockSourceTypeDef;




typedef struct __USART_HandleTypeDef
{
  USART_TypeDef *Instance;

  USART_InitTypeDef Init;

  uint8_t *pTxBuffPtr;

  uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint8_t *pRxBuffPtr;

  uint16_t RxXferSize;

  volatile uint16_t RxXferCount;

  uint16_t Mask;

  void (*RxISR)(struct __USART_HandleTypeDef *husart);

  void (*TxISR)(struct __USART_HandleTypeDef *husart);

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  volatile HAL_USART_StateTypeDef State;

  volatile uint32_t ErrorCode;
# 163 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_usart.h"
} USART_HandleTypeDef;
# 695 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_usart.h"
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_usart_ex.h" 1
# 696 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_usart.h" 2
# 707 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_usart.h"
HAL_StatusTypeDef HAL_USART_Init(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DeInit(USART_HandleTypeDef *husart);
void HAL_USART_MspInit(USART_HandleTypeDef *husart);
void HAL_USART_MspDeInit(USART_HandleTypeDef *husart);
# 728 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_usart.h"
HAL_StatusTypeDef HAL_USART_Transmit(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_TransmitReceive(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData,
                                            uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Transmit_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData,
                                               uint16_t Size);
HAL_StatusTypeDef HAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData,
                                                uint16_t Size);
HAL_StatusTypeDef HAL_USART_DMAPause(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAResume(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAStop(USART_HandleTypeDef *husart);

HAL_StatusTypeDef HAL_USART_Abort(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_Abort_IT(USART_HandleTypeDef *husart);

void HAL_USART_IRQHandler(USART_HandleTypeDef *husart);
void HAL_USART_TxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxRxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart);
void HAL_USART_AbortCpltCallback(USART_HandleTypeDef *husart);
# 765 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_usart.h"
HAL_USART_StateTypeDef HAL_USART_GetState(USART_HandleTypeDef *husart);
uint32_t HAL_USART_GetError(USART_HandleTypeDef *husart);
# 56 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_wwdg.h" 1
# 48 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_wwdg.h"
typedef struct
{
  uint32_t Prescaler;


  uint32_t Window;


  uint32_t Counter;


  uint32_t EWIMode ;


} WWDG_InitTypeDef;







typedef struct

{
  WWDG_TypeDef *Instance;

  WWDG_InitTypeDef Init;






} WWDG_HandleTypeDef;
# 259 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_wwdg.h"
HAL_StatusTypeDef HAL_WWDG_Init(WWDG_HandleTypeDef *hwwdg);
void HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg);
# 275 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal_wwdg.h"
HAL_StatusTypeDef HAL_WWDG_Refresh(WWDG_HandleTypeDef *hwwdg);
void HAL_WWDG_IRQHandler(WWDG_HandleTypeDef *hwwdg);
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg);
# 57 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h" 1
# 431 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
typedef struct
{
  uint32_t CommonClock;
# 442 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
  uint32_t Multimode;




  uint32_t MultiDMATransfer;




  uint32_t MultiTwoSamplingDelay;





} LL_ADC_CommonInitTypeDef;
# 480 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
typedef struct
{
  uint32_t Resolution;




  uint32_t DataAlignment;




  uint32_t LowPowerMode;




} LL_ADC_InitTypeDef;
# 518 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
typedef struct
{
  uint32_t TriggerSource;







  uint32_t SequencerLength;




  uint32_t SequencerDiscont;






  uint32_t ContinuousMode;





  uint32_t DMATransfer;




  uint32_t Overrun;





} LL_ADC_REG_InitTypeDef;
# 578 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
typedef struct
{
  uint32_t TriggerSource;







  uint32_t SequencerLength;




  uint32_t SequencerDiscont;






  uint32_t TrigAuto;





} LL_ADC_INJ_InitTypeDef;
# 2492 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_DMA_GetRegAddr(ADC_TypeDef *ADCx, uint32_t Register)
{
  register uint32_t data_reg_addr = 0U;

  if (Register == ((uint32_t)0x00000000U))
  {

    data_reg_addr = (uint32_t)&(ADCx->DR);
  }
  else
  {

    data_reg_addr = (uint32_t)&((((((ADCx) == ((ADC_TypeDef *) ((0x40000000UL + 0x10000000UL) + 0x00000000UL))) || ((ADCx) == ((ADC_TypeDef *) ((0x40000000UL + 0x10000000UL) + 0x00000100UL)))) ? ( (((ADC_Common_TypeDef *) ((0x40000000UL + 0x10000000UL) + 0x00000300UL))) ) : ( (((ADC_Common_TypeDef *) ((0x40000000UL + 0x10000000UL) + 0x00000700UL))) ) ))->CDR);
  }

  return data_reg_addr;
}
# 2548 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t CommonClock)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (16U))))) | (CommonClock))));
}
# 2565 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (16U)))));
}
# 2605 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U))))) | (PathInternal))));
}
# 2627 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U)))));
}
# 2671 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetCalibrationFactor(ADC_TypeDef *ADCx, uint32_t SingleDiff, uint32_t CalibrationFactor)
{
  (((ADCx->CALFACT)) = ((((((ADCx->CALFACT))) & (~(SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U)))))) | (CalibrationFactor << (__CLZ(__RBIT(SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U))))))))))

                                                                                              ;
}
# 2694 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetCalibrationFactor(ADC_TypeDef *ADCx, uint32_t SingleDiff)
{




  return (uint32_t)(((ADCx->CALFACT) & ((SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U)))))) >> (__CLZ(__RBIT(SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U)))))));
}
# 2720 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetResolution(ADC_TypeDef *ADCx, uint32_t Resolution)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (3U))))) | (Resolution))));
}
# 2737 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetResolution(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x3UL << (3U)))));
}
# 2757 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetDataAlignment(ADC_TypeDef *ADCx, uint32_t DataAlignment)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (5U))))) | (DataAlignment))));
}
# 2772 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetDataAlignment(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (5U)))));
}
# 2825 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetLowPowerMode(ADC_TypeDef *ADCx, uint32_t LowPowerMode)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (14U))))) | (LowPowerMode))));
}
# 2873 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetLowPowerMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (14U)))));
}
# 2952 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetOffset(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t Channel, uint32_t OffsetLevel)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2U))));

  (((*preg)) = ((((((*preg))) & (~((0x1UL << (31U)) | (0x1FUL << (26U)) | (0xFFFUL << (0U))))) | ((0x1UL << (31U)) | (Channel & ((0x1FUL << (26U)))) | OffsetLevel))))

                                                                                        ;
}
# 3025 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetOffsetChannel(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2U))));

  return (uint32_t) ((*preg) & ((0x1FUL << (26U))));
}
# 3051 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetOffsetLevel(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2U))));

  return (uint32_t) ((*preg) & ((0xFFFUL << (0U))));
}
# 3084 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetOffsetState(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t OffsetState)
{
  register volatile uint32_t *preg = (volatile uint32_t *)((uint32_t)
                            ((uint32_t)(&ADCx->OFR1) + (Offsety*4U)));

  (((*preg)) = ((((((*preg))) & (~((0x1UL << (31U))))) | (OffsetState))))

                         ;
}
# 3111 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetOffsetState(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2U))));

  return (uint32_t) ((*preg) & ((0x1UL << (31U))));
}
# 3205 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (10U)) | (0xFUL << (6U))))) | (TriggerSource))));
}
# 3284 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetTriggerSource(ADC_TypeDef *ADCx)
{
  register uint32_t TriggerSource = ((ADCx->CFGR) & ((0xFUL << (6U)) | (0x3UL << (10U))));



  register uint32_t ShiftExten = ((TriggerSource & (0x3UL << (10U))) >> (((uint32_t)10U) - 2U));



  return ((TriggerSource
           & ((((((uint32_t)0x00000000U) & (0xFUL << (6U))) << (4U * 0U)) | (((0xFUL << (6U))) << (4U * 1U)) | (((0xFUL << (6U))) << (4U * 2U)) | (((0xFUL << (6U))) << (4U * 3U)) ) >> ShiftExten) & (0xFUL << (6U)))
          | (((((((uint32_t)0x00000000U) & (0x3UL << (10U))) << (4U * 0U)) | ((((0x1UL << (10U)))) << (4U * 1U)) | ((((0x1UL << (10U)))) << (4U * 2U)) | ((((0x1UL << (10U)))) << (4U * 3U)) ) >> ShiftExten) & (0x3UL << (10U)))
         );
}
# 3311 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return (((ADCx->CFGR) & ((0x3UL << (10U)))) == (((uint32_t)0x00000000U) & (0x3UL << (10U))));
}
# 3331 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_SetTriggerEdge(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (10U))))) | (ExternalTriggerEdge))));
}
# 3346 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x3UL << (10U)))));
}
# 3406 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->SQR1)) = ((((((ADCx->SQR1))) & (~((0xFUL << (0U))))) | (SequencerNbRanks))));
}
# 3460 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->SQR1) & ((0xFUL << (0U)))));
}
# 3492 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (16U)) | (0x7UL << (17U))))) | (SeqDiscont))));
}
# 3515 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (16U)) | (0x7UL << (17U)))));
}
# 3610 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{




  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + (((((Rank) & ((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))))))) << 2U))));

  (((*preg)) = ((((((*preg))) & (~(((0x1FUL << (0U))) << (Rank & (((0x1FUL << (0U)))))))) | ((Channel & ((0x1FUL << (26U)))) >> (((uint32_t)26U) - (Rank & (((0x1FUL << (0U))))))))))

                                                                                                                                  ;
}
# 3715 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + (((((Rank) & ((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))))))) << 2U))));

  return (uint32_t) (((*preg) & (((0x1FUL << (0U))) << (Rank & (((0x1FUL << (0U)))))))

                     << (((uint32_t)26U) - (Rank & (((0x1FUL << (0U))))))
                    );
}
# 3744 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_SetContinuousMode(ADC_TypeDef *ADCx, uint32_t Continuous)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (13U))))) | (Continuous))));
}
# 3761 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetContinuousMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (13U)))));
}
# 3801 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_SetDMATransfer(ADC_TypeDef *ADCx, uint32_t DMATransfer)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (0U)) | (0x1UL << (1U))))) | (DMATransfer))));
}
# 3836 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetDMATransfer(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (0U)) | (0x1UL << (1U)))));
}
# 3861 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_SetOverrun(ADC_TypeDef *ADCx, uint32_t Overrun)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (12U))))) | (Overrun))));
}
# 3875 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetOverrun(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (12U)))));
}
# 3967 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0xFUL << (2U)) | (0x3UL << (6U))))) | (TriggerSource))));
}
# 4039 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetTriggerSource(ADC_TypeDef *ADCx)
{
  register uint32_t TriggerSource = ((ADCx->JSQR) & ((0xFUL << (2U)) | (0x3UL << (6U))));



  register uint32_t ShiftJexten = ((TriggerSource & (0x3UL << (6U))) >> (((uint32_t) 6U) - 2U));



  return ((TriggerSource
           & ((((((uint32_t)0x00000000U) & (0xFUL << (2U))) << (4U * 0U)) | (((0xFUL << (2U))) << (4U * 1U)) | (((0xFUL << (2U))) << (4U * 2U)) | (((0xFUL << (2U))) << (4U * 3U)) ) >> ShiftJexten) & (0xFUL << (2U)))
          | (((((((uint32_t)0x00000000U) & (0x3UL << (6U))) << (4U * 0U)) | ((((0x1UL << (6U)))) << (4U * 1U)) | ((((0x1UL << (6U)))) << (4U * 2U)) | ((((0x1UL << (6U)))) << (4U * 3U)) ) >> ShiftJexten) & (0x3UL << (6U)))
         );
}
# 4066 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return (((ADCx->JSQR) & ((0x3UL << (6U)))) == (((uint32_t)0x00000000U) & (0x3UL << (6U))));
}
# 4086 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_SetTriggerEdge(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x3UL << (6U))))) | (ExternalTriggerEdge))));
}
# 4101 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->JSQR) & ((0x3UL << (6U)))));
}
# 4134 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x3UL << (0U))))) | (SequencerNbRanks))));
}
# 4155 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->JSQR) & ((0x3UL << (0U)))));
}
# 4173 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (20U))))) | (SeqDiscont))));
}
# 4188 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (20U)))));
}
# 4259 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{




  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~(((0x1FUL << (26U))) >> (((uint32_t)26U) - (Rank & (((0x1FUL << (0U))))))))) | ((Channel & ((0x1FUL << (26U)))) >> (((uint32_t)26U) - (Rank & (((0x1FUL << (0U))))))))))

                                                                                                                                  ;
}
# 4334 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  return (uint32_t)(((ADCx->JSQR) & (((0x1FUL << (26U))) >> (((uint32_t)26U) - (Rank & (((0x1FUL << (0U))))))))

                    << (((uint32_t)26U) - (Rank & (((0x1FUL << (0U))))))
                   );
}
# 4372 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_SetTrigAuto(ADC_TypeDef *ADCx, uint32_t TrigAuto)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (25U))))) | (TrigAuto))));
}
# 4386 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetTrigAuto(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (25U)))));
}
# 4430 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_SetQueueMode(ADC_TypeDef *ADCx, uint32_t QueueMode)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (21U))))) | (QueueMode))));
}
# 4443 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetQueueMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (21U)))));
}
# 4682 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_ConfigQueueContext(ADC_TypeDef *ADCx,
                                                   uint32_t TriggerSource,
                                                   uint32_t ExternalTriggerEdge,
                                                   uint32_t SequencerNbRanks,
                                                   uint32_t Rank1_Channel,
                                                   uint32_t Rank2_Channel,
                                                   uint32_t Rank3_Channel,
                                                   uint32_t Rank4_Channel)
{






  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0xFUL << (2U)) | (0x3UL << (6U)) | (0x1FUL << (26U)) | (0x1FUL << (20U)) | (0x1FUL << (14U)) | (0x1FUL << (8U)) | (0x3UL << (0U))))) | (TriggerSource | (ExternalTriggerEdge * ((TriggerSource != ((uint32_t)0x00000000U)))) | ((Rank4_Channel & ((0x1FUL << (26U)))) >> (((uint32_t)26U) - ((((uint32_t)0x00000300U) | ((uint32_t)26U)) & (((0x1FUL << (0U))))))) | ((Rank3_Channel & ((0x1FUL << (26U)))) >> (((uint32_t)26U) - ((((uint32_t)0x00000200U) | ((uint32_t)20U)) & (((0x1FUL << (0U))))))) | ((Rank2_Channel & ((0x1FUL << (26U)))) >> (((uint32_t)26U) - ((((uint32_t)0x00000100U) | ((uint32_t)14U)) & (((0x1FUL << (0U))))))) | ((Rank1_Channel & ((0x1FUL << (26U)))) >> (((uint32_t)26U) - ((((uint32_t)0x00000000U) | ((uint32_t) 8U)) & (((0x1FUL << (0U))))))) | SequencerNbRanks))))
# 4712 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
             ;
}
# 4814 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SamplingTime)
{




  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + (((((Channel) & ((((uint32_t)0x00000000U) | ((uint32_t)0x02000000U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x02000000U)))))))) << 2U))));

  (((*preg)) = ((((((*preg))) & (~((0x7UL << (0U)) << (((Channel) & (((uint32_t)0x01F00000U))) >> (__CLZ(__RBIT((((uint32_t)0x01F00000U))))))))) | (SamplingTime << (((Channel) & (((uint32_t)0x01F00000U))) >> (__CLZ(__RBIT((((uint32_t)0x01F00000U))))))))))

                                                                                          ;
}
# 4902 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + (((((Channel) & ((((uint32_t)0x00000000U) | ((uint32_t)0x02000000U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x02000000U)))))))) << 2U))));

  return (uint32_t)(((*preg) & ((0x7UL << (0U)) << (((Channel) & (((uint32_t)0x01F00000U))) >> (__CLZ(__RBIT((((uint32_t)0x01F00000U))))))))

                    >> (((Channel) & (((uint32_t)0x01F00000U))) >> (__CLZ(__RBIT((((uint32_t)0x01F00000U))))))
                   );
}
# 4964 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetChannelSingleDiff(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SingleDiff)
{



  (((ADCx->DIFSEL)) = ((((((ADCx->DIFSEL))) & (~(Channel & (((0x3FFFFUL << (1U))))))) | ((Channel & (((0x3FFFFUL << (1U))))) & ((0x3FFFFUL << (1U)) >> (SingleDiff & ((0x10UL << (0U)) | (0x08UL << (0U)))))))))

                                                                                                                               ;
}
# 5019 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetChannelSingleDiff(ADC_TypeDef *ADCx, uint32_t Channel)
{
  return (uint32_t)(((ADCx->DIFSEL) & ((Channel & (((0x3FFFFUL << (1U))))))));
}
# 5170 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDChannelGroup)
{




  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->CFGR)) + (((((AWDy) & ((((uint32_t)0x00000000U) | ((uint32_t)0x00100000U) | ((uint32_t)0x00200000U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x00100000U) | ((uint32_t)0x00200000U))))))) + ((AWDy & ((0x00001UL << (1U)))) * ((uint32_t)0x00000024U))) << 2U))))
                                                                                                                                         ;

  (((*preg)) = ((((((*preg))) & (~((AWDy & (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | ((0x3FFFFUL << (1U)))))))) | (AWDChannelGroup & AWDy))))

                                    ;
}
# 5306 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDy)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->CFGR)) + (((((AWDy) & ((((uint32_t)0x00000000U) | ((uint32_t)0x00100000U) | ((uint32_t)0x00200000U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x00100000U) | ((uint32_t)0x00200000U))))))) + ((AWDy & ((0x00001UL << (1U)))) * ((uint32_t)0x00000024U))) << 2U))))
                                                                                                                                         ;



  register uint32_t AWD123ChannelGroup = ((*preg) & ((AWDy | (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | ((0x3FFFFUL << (1U)))))));
# 5322 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
  register uint32_t AWD1ChannelSingle = ((AWD123ChannelGroup & (0x1UL << (22U))) >> ((uint32_t)22U));

  register uint32_t AWD1ChannelGroup = ( ( AWD123ChannelGroup
                                          | ((0x00000001UL << ((AWD123ChannelGroup & ((0x1FUL << (26U)))) >> ((uint32_t)26U))) * AWD1ChannelSingle)
                                          | (((0x3FFFFUL << (1U))) * (~AWD1ChannelSingle & ((uint32_t)0x00000001U)))
                                         )
                                        * (((AWD123ChannelGroup & (0x1UL << (24U))) >> ((uint32_t)24U)) | ((AWD123ChannelGroup & (0x1UL << (23U))) >> ((uint32_t)23U)))
                                       );
# 5343 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
  register uint32_t AWD23Enabled = ((((uint32_t)0x00000001U) >> (AWD123ChannelGroup % 3U)) << 6U);

  register uint32_t AWD23ChannelGroup = ((( AWD123ChannelGroup
                                           | ((uint32_t)(__CLZ(__RBIT(AWD123ChannelGroup))) << ((uint32_t)26U))
                                           | (((0x1UL << (22U))) >> ((((uint32_t)0x00000001U) >> (((0x3FFFFUL << (1U))) - AWD123ChannelGroup)) << 5U))
                                           | ((0x1UL << (24U)) | (0x1UL << (23U)))
                                          ) >> AWD23Enabled
                                         ) >> (((AWDy & (0x1UL << (22U))) >> ((uint32_t)22U)) << 5U));

  return (AWD1ChannelGroup | AWD23ChannelGroup);
}
# 5403 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_ConfigAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdHighValue, uint32_t AWDThresholdLowValue)
{





  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + (((((AWDy) & (((((uint32_t)0x00000000U)) | (((uint32_t)0x00100000U)) | (((uint32_t)0x00200000U))))) >> (__CLZ(__RBIT((((((uint32_t)0x00000000U)) | (((uint32_t)0x00100000U)) | (((uint32_t)0x00200000U))))))))) << 2U))));

  (((*preg)) = ((((((*preg))) & (~((0xFFFUL << (16U)) | (0xFFFUL << (0U))))) | ((AWDThresholdHighValue << ((uint32_t)16U)) | AWDThresholdLowValue))))

                                                                                         ;
}
# 5467 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdsHighLow, uint32_t AWDThresholdValue)
{





  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + (((((AWDy) & (((((uint32_t)0x00000000U)) | (((uint32_t)0x00100000U)) | (((uint32_t)0x00200000U))))) >> (__CLZ(__RBIT((((((uint32_t)0x00000000U)) | (((uint32_t)0x00100000U)) | (((uint32_t)0x00200000U))))))))) << 2U))));

  (((*preg)) = ((((((*preg))) & (~(AWDThresholdsHighLow))) | (AWDThresholdValue << (__CLZ(__RBIT(AWDThresholdsHighLow)))))))

                                                                     ;
}
# 5509 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdsHighLow)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + (((((AWDy) & (((((uint32_t)0x00000000U)) | (((uint32_t)0x00100000U)) | (((uint32_t)0x00200000U))))) >> (__CLZ(__RBIT((((((uint32_t)0x00000000U)) | (((uint32_t)0x00100000U)) | (((uint32_t)0x00200000U))))))))) << 2U))));

  return (uint32_t)(((*preg) & ((AWDThresholdsHighLow | (0xFFFUL << (0U)))))

                    >> (__CLZ(__RBIT(AWDThresholdsHighLow)))
                   );
}
# 5554 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetMultimode(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t Multimode)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1FUL << (0U))))) | (Multimode))));
}
# 5578 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetMultimode(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1FUL << (0U)))));
}
# 5629 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiDMATransfer)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (14U)) | (0x1UL << (13U))))) | (MultiDMATransfer))));
}
# 5675 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (14U)) | (0x1UL << (13U)))));
}
# 5715 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_SetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiTwoSamplingDelay)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0xFUL << (8U))))) | (MultiTwoSamplingDelay))));
}
# 5743 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_GetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0xFUL << (8U)))));
}
# 5770 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_EnableInternalRegulator(ADC_TypeDef *ADCx)
{


  ((ADCx->CR) &= ~(((0x2UL << (28U)) | (0x1UL << (28U)))));





  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (28U))))))

                               ;
}
# 5794 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_DisableInternalRegulator(ADC_TypeDef *ADCx)
{
  ((ADCx->CR) &= ~(((0x3UL << (28U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U))))));
}







static inline uint32_t LL_ADC_IsInternalRegulatorEnabled(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR) & (((0x2UL << (28U)) | (0x1UL << (28U))))) == ((0x1UL << (28U))));
}
# 5826 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_Enable(ADC_TypeDef *ADCx)
{



  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (0U))))))

                         ;
}
# 5846 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_Disable(ADC_TypeDef *ADCx)
{



  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (1U))))))

                          ;
}
# 5865 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabled(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR) & ((0x1UL << (0U)))) == ((0x1UL << (0U))));
}







static inline uint32_t LL_ADC_IsDisableOngoing(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR) & ((0x1UL << (1U)))) == ((0x1UL << (1U))));
}
# 5904 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_StartCalibration(ADC_TypeDef *ADCx, uint32_t SingleDiff)
{



  (((ADCx->CR)) = ((((((ADCx->CR))) & (~((0x1UL << (30U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (31U)) | (SingleDiff & ((0x1UL << (30U))))))))

                                                                           ;
}







static inline uint32_t LL_ADC_IsCalibrationOnGoing(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR) & ((0x1UL << (31U)))) == ((0x1UL << (31U))));
}
# 5951 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_StartConversion(ADC_TypeDef *ADCx)
{



  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (2U))))))

                            ;
}
# 5971 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_REG_StopConversion(ADC_TypeDef *ADCx)
{



  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (4U))))))

                          ;
}







static inline uint32_t LL_ADC_REG_IsConversionOngoing(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR) & ((0x1UL << (2U)))) == ((0x1UL << (2U))));
}







static inline uint32_t LL_ADC_REG_IsStopConversionOngoing(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR) & ((0x1UL << (4U)))) == ((0x1UL << (4U))));
}
# 6012 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_ReadConversionData32(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 6027 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint16_t LL_ADC_REG_ReadConversionData12(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 6042 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint16_t LL_ADC_REG_ReadConversionData10(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 6057 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint8_t LL_ADC_REG_ReadConversionData8(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 6072 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint8_t LL_ADC_REG_ReadConversionData6(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 6099 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_ReadMultiConversionData32(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t ConversionData)
{
  return (uint32_t)(((ADCxy_COMMON->CDR) & (ConversionData))

                    >> (__CLZ(__RBIT(ConversionData)))
                   );
}
# 6134 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_StartConversion(ADC_TypeDef *ADCx)
{



  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (3U))))))

                             ;
}
# 6154 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_INJ_StopConversion(ADC_TypeDef *ADCx)
{



  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (5U))))))

                           ;
}







static inline uint32_t LL_ADC_INJ_IsConversionOngoing(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR) & ((0x1UL << (3U)))) == ((0x1UL << (3U))));
}







static inline uint32_t LL_ADC_INJ_IsStopConversionOngoing(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR) & ((0x1UL << (5U)))) == ((0x1UL << (5U))));
}
# 6203 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_ReadConversionData32(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))))))) << 2U))));

  return (uint32_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}
# 6230 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint16_t LL_ADC_INJ_ReadConversionData12(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))))))) << 2U))));

  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}
# 6257 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint16_t LL_ADC_INJ_ReadConversionData10(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))))))) << 2U))));

  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}
# 6284 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint8_t LL_ADC_INJ_ReadConversionData8(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))))))) << 2U))));

  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}
# 6311 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint8_t LL_ADC_INJ_ReadConversionData6(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))) >> (__CLZ(__RBIT(((((uint32_t)0x00000000U) | ((uint32_t)0x00000100U) | ((uint32_t)0x00000200U) | ((uint32_t)0x00000300U)))))))) << 2U))));

  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}
# 6337 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_ADRDY(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (0U)))) == ((0x1UL << (0U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_EOC(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (2U)))) == ((0x1UL << (2U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_EOS(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (3U)))) == ((0x1UL << (3U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_OVR(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (4U)))) == ((0x1UL << (4U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_EOSMP(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (1U)))) == ((0x1UL << (1U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_JEOC(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (5U)))) == ((0x1UL << (5U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_JEOS(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (6U)))) == ((0x1UL << (6U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_JQOVF(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (10U)))) == ((0x1UL << (10U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_AWD1(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (7U)))) == ((0x1UL << (7U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_AWD2(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (8U)))) == ((0x1UL << (8U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_AWD3(ADC_TypeDef *ADCx)
{
  return (((ADCx->ISR) & ((0x1UL << (9U)))) == ((0x1UL << (9U))));
}
# 6461 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_ClearFlag_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (0U))));
}







static inline void LL_ADC_ClearFlag_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (2U))));
}







static inline void LL_ADC_ClearFlag_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (3U))));
}







static inline void LL_ADC_ClearFlag_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (4U))));
}







static inline void LL_ADC_ClearFlag_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (1U))));
}







static inline void LL_ADC_ClearFlag_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (5U))));
}







static inline void LL_ADC_ClearFlag_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (6U))));
}







static inline void LL_ADC_ClearFlag_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (10U))));
}







static inline void LL_ADC_ClearFlag_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (7U))));
}







static inline void LL_ADC_ClearFlag_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (8U))));
}







static inline void LL_ADC_ClearFlag_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (9U))));
}
# 6584 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_ADRDY(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (0U)))) == ((0x1UL << (0U))));
}
# 6596 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_ADRDY(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (16U)))) == ((0x1UL << (16U))));
}
# 6608 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_EOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (18U)))) == ((0x1UL << (18U))));
}
# 6620 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_EOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (18U)))) == ((0x1UL << (18U))));
}
# 6632 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_EOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (3U)))) == ((0x1UL << (3U))));
}
# 6644 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_EOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (19U)))) == ((0x1UL << (19U))));
}
# 6656 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (4U)))) == ((0x1UL << (4U))));
}
# 6668 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (20U)))) == ((0x1UL << (20U))));
}
# 6680 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_EOSMP(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (1U)))) == ((0x1UL << (1U))));
}
# 6692 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_EOSMP(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (17U)))) == ((0x1UL << (17U))));
}
# 6704 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_JEOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (5U)))) == ((0x1UL << (5U))));
}
# 6716 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_JEOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (21U)))) == ((0x1UL << (21U))));
}
# 6728 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (6U)))) == ((0x1UL << (6U))));
}
# 6740 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (22U)))) == ((0x1UL << (22U))));
}
# 6752 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_JQOVF(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (10U)))) == ((0x1UL << (10U))));
}
# 6764 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_JQOVF(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (26U)))) == ((0x1UL << (26U))));
}
# 6776 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (7U)))) == ((0x1UL << (7U))));
}
# 6788 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (23U)))) == ((0x1UL << (23U))));
}
# 6800 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_AWD2(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (8U)))) == ((0x1UL << (8U))));
}
# 6812 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD2(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (24U)))) == ((0x1UL << (24U))));
}
# 6824 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_AWD3(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (9U)))) == ((0x1UL << (9U))));
}
# 6836 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD3(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (25U)))) == ((0x1UL << (25U))));
}
# 6856 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline void LL_ADC_EnableIT_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (0U))));
}







static inline void LL_ADC_EnableIT_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (2U))));
}







static inline void LL_ADC_EnableIT_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (3U))));
}







static inline void LL_ADC_EnableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (4U))));
}







static inline void LL_ADC_EnableIT_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (1U))));
}







static inline void LL_ADC_EnableIT_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (5U))));
}







static inline void LL_ADC_EnableIT_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (6U))));
}







static inline void LL_ADC_EnableIT_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (10U))));
}







static inline void LL_ADC_EnableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (7U))));
}







static inline void LL_ADC_EnableIT_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (8U))));
}







static inline void LL_ADC_EnableIT_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (9U))));
}







static inline void LL_ADC_DisableIT_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (0U))));
}







static inline void LL_ADC_DisableIT_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (2U))));
}







static inline void LL_ADC_DisableIT_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (3U))));
}







static inline void LL_ADC_DisableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (4U))));
}







static inline void LL_ADC_DisableIT_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (1U))));
}







static inline void LL_ADC_DisableIT_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (5U))));
}







static inline void LL_ADC_DisableIT_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (6U))));
}







static inline void LL_ADC_DisableIT_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (10U))));
}







static inline void LL_ADC_DisableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (7U))));
}







static inline void LL_ADC_DisableIT_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (8U))));
}







static inline void LL_ADC_DisableIT_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (9U))));
}
# 7099 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_ADRDY(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (0U)))) == ((0x1UL << (0U))));
}
# 7111 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_EOC(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (2U)))) == ((0x1UL << (2U))));
}
# 7123 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_EOS(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (3U)))) == ((0x1UL << (3U))));
}
# 7135 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_OVR(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (4U)))) == ((0x1UL << (4U))));
}
# 7147 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_EOSMP(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (1U)))) == ((0x1UL << (1U))));
}
# 7159 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_JEOC(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (5U)))) == ((0x1UL << (5U))));
}
# 7171 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_JEOS(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (6U)))) == ((0x1UL << (6U))));
}
# 7183 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_JQOVF(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (10U)))) == ((0x1UL << (10U))));
}
# 7195 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_AWD1(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (7U)))) == ((0x1UL << (7U))));
}
# 7207 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_AWD2(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (8U)))) == ((0x1UL << (8U))));
}
# 7219 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_AWD3(ADC_TypeDef *ADCx)
{
  return (((ADCx->IER) & ((0x1UL << (9U)))) == ((0x1UL << (9U))));
}
# 7234 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_adc.h"
ErrorStatus LL_ADC_CommonDeInit(ADC_Common_TypeDef *ADCxy_COMMON);
ErrorStatus LL_ADC_CommonInit(ADC_Common_TypeDef *ADCxy_COMMON, LL_ADC_CommonInitTypeDef *ADC_CommonInitStruct);
void LL_ADC_CommonStructInit(LL_ADC_CommonInitTypeDef *ADC_CommonInitStruct);



ErrorStatus LL_ADC_DeInit(ADC_TypeDef *ADCx);


ErrorStatus LL_ADC_Init(ADC_TypeDef *ADCx, LL_ADC_InitTypeDef *ADC_InitStruct);
void LL_ADC_StructInit(LL_ADC_InitTypeDef *ADC_InitStruct);


ErrorStatus LL_ADC_REG_Init(ADC_TypeDef *ADCx, LL_ADC_REG_InitTypeDef *ADC_REG_InitStruct);
void LL_ADC_REG_StructInit(LL_ADC_REG_InitTypeDef *ADC_REG_InitStruct);


ErrorStatus LL_ADC_INJ_Init(ADC_TypeDef *ADCx, LL_ADC_INJ_InitTypeDef *ADC_INJ_InitStruct);
void LL_ADC_INJ_StructInit(LL_ADC_INJ_InitTypeDef *ADC_INJ_InitStruct);
# 58 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h" 1
# 202 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline void LL_PWR_EnableSDADC(uint32_t Analogx)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) |= (Analogx));
}
# 218 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline void LL_PWR_DisableSDADC(uint32_t Analogx)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) &= ~(Analogx));
}
# 234 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline uint32_t LL_PWR_IsEnabledSDADC(uint32_t Analogx)
{
  return (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) & (Analogx)) == (Analogx));
}






static inline void LL_PWR_EnableBkUpAccess(void)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) |= ((0x1UL << (8U))));
}






static inline void LL_PWR_DisableBkUpAccess(void)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) &= ~((0x1UL << (8U))));
}






static inline uint32_t LL_PWR_IsEnabledBkUpAccess(void)
{
  return (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) & ((0x1UL << (8U)))) == ((0x1UL << (8U))));
}
# 278 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline void LL_PWR_SetRegulModeDS(uint32_t RegulMode)
{
  (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR)) = ((((((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR))) & (~((0x1UL << (0U))))) | (RegulMode))));
}
# 290 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline uint32_t LL_PWR_GetRegulModeDS(void)
{
  return (uint32_t)(((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) & ((0x1UL << (0U)))));
}
# 306 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline void LL_PWR_SetPowerMode(uint32_t PDMode)
{
  (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR)) = ((((((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR))) & (~(((0x1UL << (1U))| (0x1UL << (0U)))))) | (PDMode))));
}
# 320 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline uint32_t LL_PWR_GetPowerMode(void)
{
  return (uint32_t)(((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) & (((0x1UL << (1U))| (0x1UL << (0U))))));
}
# 340 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline void LL_PWR_SetPVDLevel(uint32_t PVDLevel)
{
  (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR)) = ((((((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR))) & (~((0x7UL << (5U))))) | (PVDLevel))));
}
# 358 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline uint32_t LL_PWR_GetPVDLevel(void)
{
  return (uint32_t)(((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) & ((0x7UL << (5U)))));
}






static inline void LL_PWR_EnablePVD(void)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) |= ((0x1UL << (4U))));
}






static inline void LL_PWR_DisablePVD(void)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) &= ~((0x1UL << (4U))));
}






static inline uint32_t LL_PWR_IsEnabledPVD(void)
{
  return (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) & ((0x1UL << (4U)))) == ((0x1UL << (4U))));
}
# 407 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline void LL_PWR_EnableWakeUpPin(uint32_t WakeUpPin)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CSR) |= (WakeUpPin));
}
# 425 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline void LL_PWR_DisableWakeUpPin(uint32_t WakeUpPin)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CSR) &= ~(WakeUpPin));
}
# 443 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline uint32_t LL_PWR_IsEnabledWakeUpPin(uint32_t WakeUpPin)
{
  return (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CSR) & (WakeUpPin)) == (WakeUpPin));
}
# 462 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline uint32_t LL_PWR_IsActiveFlag_WU(void)
{
  return (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CSR) & ((0x1UL << (0U)))) == ((0x1UL << (0U))));
}






static inline uint32_t LL_PWR_IsActiveFlag_SB(void)
{
  return (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CSR) & ((0x1UL << (1U)))) == ((0x1UL << (1U))));
}







static inline uint32_t LL_PWR_IsActiveFlag_PVDO(void)
{
  return (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CSR) & ((0x1UL << (2U)))) == ((0x1UL << (2U))));
}
# 495 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
static inline uint32_t LL_PWR_IsActiveFlag_VREFINTRDY(void)
{
  return (((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CSR) & ((0x1UL << (3U)))) == ((0x1UL << (3U))));
}






static inline void LL_PWR_ClearFlag_SB(void)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) |= ((0x1UL << (3U))));
}






static inline void LL_PWR_ClearFlag_WU(void)
{
  ((((PWR_TypeDef *) (0x40000000UL + 0x00007000UL))->CR) |= ((0x1UL << (2U))));
}
# 528 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_pwr.h"
ErrorStatus LL_PWR_DeInit(void);
# 59 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h" 1
# 86 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
typedef struct
{
  uint32_t HourFormat;





  uint32_t AsynchPrescaler;





  uint32_t SynchPrescaler;




} LL_RTC_InitTypeDef;




typedef struct
{
  uint32_t TimeFormat;




  uint8_t Hours;





  uint8_t Minutes;




  uint8_t Seconds;



} LL_RTC_TimeTypeDef;




typedef struct
{
  uint8_t WeekDay;




  uint8_t Month;




  uint8_t Day;




  uint8_t Year;



} LL_RTC_DateTypeDef;




typedef struct
{
  LL_RTC_TimeTypeDef AlarmTime;

  uint32_t AlarmMask;






  uint32_t AlarmDateWeekDaySel;






  uint8_t AlarmDateWeekDay;
# 192 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
} LL_RTC_AlarmTypeDef;
# 793 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_SetHourFormat(RTC_TypeDef *RTCx, uint32_t HourFormat)
{
  (((RTCx->CR)) = ((((((RTCx->CR))) & (~((0x1UL << (6U))))) | (HourFormat))));
}
# 806 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_GetHourFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->CR) & ((0x1UL << (6U)))));
}
# 823 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_SetAlarmOutEvent(RTC_TypeDef *RTCx, uint32_t AlarmOutput)
{
  (((RTCx->CR)) = ((((((RTCx->CR))) & (~((0x3UL << (21U))))) | (AlarmOutput))));
}
# 838 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_GetAlarmOutEvent(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->CR) & ((0x3UL << (21U)))));
}
# 855 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_SetAlarmOutputType(RTC_TypeDef *RTCx, uint32_t Output)
{
  (((RTCx->TAFCR)) = ((((((RTCx->TAFCR))) & (~((0x1UL << (18U))))) | (Output))));
}
# 871 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_GetAlarmOutputType(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TAFCR) & ((0x1UL << (18U)))));
}
# 890 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_EnablePushPullMode(RTC_TypeDef *RTCx, uint32_t PinMask)
{
  ((RTCx->TAFCR) |= (PinMask));
}
# 909 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DisablePushPullMode(RTC_TypeDef *RTCx, uint32_t PinMask)
{
  ((RTCx->TAFCR) &= ~(PinMask));
}
# 925 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_SetOutputPin(RTC_TypeDef *RTCx, uint32_t PinMask)
{
  ((RTCx->TAFCR) |= ((PinMask >> 1)));
}
# 941 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ResetOutputPin(RTC_TypeDef *RTCx, uint32_t PinMask)
{
  ((RTCx->TAFCR) &= ~((PinMask >> 1)));
}
# 955 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_EnableInitMode(RTC_TypeDef *RTCx)
{

  ((RTCx->ISR) = (0xFFFFFFFFU));
}







static inline void LL_RTC_DisableInitMode(RTC_TypeDef *RTCx)
{

  ((RTCx->ISR) = ((uint32_t)~(0x1UL << (7U))));
}
# 983 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_SetOutputPolarity(RTC_TypeDef *RTCx, uint32_t Polarity)
{
  (((RTCx->CR)) = ((((((RTCx->CR))) & (~((0x1UL << (20U))))) | (Polarity))));
}
# 996 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_GetOutputPolarity(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->CR) & ((0x1UL << (20U)))));
}
# 1008 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_EnableShadowRegBypass(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (5U))));
}







static inline void LL_RTC_DisableShadowRegBypass(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (5U))));
}







static inline uint32_t LL_RTC_IsShadowRegBypassEnabled(RTC_TypeDef *RTCx)
{
  return (((RTCx->CR) & ((0x1UL << (5U)))) == ((0x1UL << (5U))));
}
# 1043 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_EnableRefClock(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (4U))));
}
# 1056 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DisableRefClock(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (4U))));
}
# 1068 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_SetAsynchPrescaler(RTC_TypeDef *RTCx, uint32_t AsynchPrescaler)
{
  (((RTCx->PRER)) = ((((((RTCx->PRER))) & (~((0x7FUL << (16U))))) | (AsynchPrescaler << (16U)))));
}
# 1080 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_SetSynchPrescaler(RTC_TypeDef *RTCx, uint32_t SynchPrescaler)
{
  (((RTCx->PRER)) = ((((((RTCx->PRER))) & (~((0x7FFFUL << (0U))))) | (SynchPrescaler))));
}







static inline uint32_t LL_RTC_GetAsynchPrescaler(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->PRER) & ((0x7FUL << (16U)))) >> (16U));
}







static inline uint32_t LL_RTC_GetSynchPrescaler(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->PRER) & ((0x7FFFUL << (0U)))));
}







static inline void LL_RTC_EnableWriteProtection(RTC_TypeDef *RTCx)
{
  ((RTCx->WPR) = (((uint8_t)0xFFU)));
}







static inline void LL_RTC_DisableWriteProtection(RTC_TypeDef *RTCx)
{
  ((RTCx->WPR) = (((uint8_t)0xCAU)));
  ((RTCx->WPR) = (((uint8_t)0x53U)));
}
# 1149 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_SetFormat(RTC_TypeDef *RTCx, uint32_t TimeFormat)
{
  (((RTCx->TR)) = ((((((RTCx->TR))) & (~((0x1UL << (22U))))) | (TimeFormat))));
}
# 1166 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TIME_GetFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TR) & ((0x1UL << (22U)))));
}
# 1182 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_SetHour(RTC_TypeDef *RTCx, uint32_t Hours)
{
  (((RTCx->TR)) = ((((((RTCx->TR))) & (~(((0x3UL << (20U)) | (0xFUL << (16U)))))) | ((((Hours & 0xF0U) << ((20U) - 4U)) | ((Hours & 0x0FU) << (16U)))))))
                                                                                              ;
}
# 1201 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TIME_GetHour(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->TR) & (((0x3UL << (20U)) | (0xFUL << (16U)))))) >> (16U));
}
# 1217 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_SetMinute(RTC_TypeDef *RTCx, uint32_t Minutes)
{
  (((RTCx->TR)) = ((((((RTCx->TR))) & (~(((0x7UL << (12U)) | (0xFUL << (8U)))))) | ((((Minutes & 0xF0U) << ((12U) - 4U)) | ((Minutes & 0x0FU) << (8U)))))))
                                                                                                    ;
}
# 1236 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TIME_GetMinute(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TR) & (((0x7UL << (12U)) | (0xFUL << (8U))))) >> (8U));
}
# 1252 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_SetSecond(RTC_TypeDef *RTCx, uint32_t Seconds)
{
  (((RTCx->TR)) = ((((((RTCx->TR))) & (~(((0x7UL << (4U)) | (0xFUL << (0U)))))) | ((((Seconds & 0xF0U) << ((4U) - 4U)) | ((Seconds & 0x0FU) << (0U)))))))
                                                                                                  ;
}
# 1271 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TIME_GetSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TR) & (((0x7UL << (4U)) | (0xFUL << (0U))))) >> (0U));
}
# 1297 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_Config(RTC_TypeDef *RTCx, uint32_t Format12_24, uint32_t Hours, uint32_t Minutes, uint32_t Seconds)
{
  register uint32_t temp = 0U;

  temp = Format12_24 |
         (((Hours & 0xF0U) << ((20U) - 4U)) | ((Hours & 0x0FU) << (16U))) |
         (((Minutes & 0xF0U) << ((12U) - 4U)) | ((Minutes & 0x0FU) << (8U))) |
         (((Seconds & 0xF0U) << ((4U) - 4U)) | ((Seconds & 0x0FU) << (0U)));
  (((RTCx->TR)) = ((((((RTCx->TR))) & (~(((0x1UL << (22U)) | (0x3UL << (20U)) | (0xFUL << (16U)) | (0x7UL << (12U)) | (0xFUL << (8U)) | (0x7UL << (4U)) | (0xFUL << (0U)))))) | (temp))));
}
# 1325 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TIME_Get(RTC_TypeDef *RTCx)
{
  register uint32_t temp = 0U;

  temp = ((RTCx->TR) & (((0x3UL << (20U)) | (0xFUL << (16U)) | (0x7UL << (12U)) | (0xFUL << (8U)) | (0x7UL << (4U)) | (0xFUL << (0U)))));
  return (uint32_t)((((((temp & (0x3UL << (20U))) >> (20U)) << 4U) | ((temp & (0xFUL << (16U))) >> (16U))) << 16U) |
                    (((((temp & (0x7UL << (12U))) >> (12U)) << 4U) | ((temp & (0xFUL << (8U))) >> (8U))) << 8U) |
                    ((((temp & (0x7UL << (4U))) >> (4U)) << 4U) | ((temp & (0xFUL << (0U))) >> (0U))));
}
# 1342 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_EnableDayLightStore(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (18U))));
}
# 1354 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_DisableDayLightStore(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (18U))));
}







static inline uint32_t LL_RTC_TIME_IsDayLightStoreEnabled(RTC_TypeDef *RTCx)
{
  return (((RTCx->CR) & ((0x1UL << (18U)))) == ((0x1UL << (18U))));
}
# 1377 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_DecHour(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (17U))));
}
# 1389 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_IncHour(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (16U))));
}
# 1407 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TIME_GetSubSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->SSR) & ((0xFFFFUL << (0U)))));
}
# 1426 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TIME_Synchronize(RTC_TypeDef *RTCx, uint32_t ShiftSecond, uint32_t Fraction)
{
  ((RTCx->SHIFTR) = (ShiftSecond | Fraction));
}
# 1448 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DATE_SetYear(RTC_TypeDef *RTCx, uint32_t Year)
{
  (((RTCx->DR)) = ((((((RTCx->DR))) & (~(((0xFUL << (20U)) | (0xFUL << (16U)))))) | ((((Year & 0xF0U) << ((20U) - 4U)) | ((Year & 0x0FU) << (16U)))))))
                                                                                            ;
}
# 1464 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_DATE_GetYear(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->DR) & (((0xFUL << (20U)) | (0xFUL << (16U)))))) >> (16U));
}
# 1483 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DATE_SetWeekDay(RTC_TypeDef *RTCx, uint32_t WeekDay)
{
  (((RTCx->DR)) = ((((((RTCx->DR))) & (~((0x7UL << (13U))))) | (WeekDay << (13U)))));
}
# 1503 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_DATE_GetWeekDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->DR) & ((0x7UL << (13U)))) >> (13U));
}
# 1529 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DATE_SetMonth(RTC_TypeDef *RTCx, uint32_t Month)
{
  (((RTCx->DR)) = ((((((RTCx->DR))) & (~(((0x1UL << (12U)) | (0xFUL << (8U)))))) | ((((Month & 0xF0U) << ((12U) - 4U)) | ((Month & 0x0FU) << (8U)))))))
                                                                                              ;
}
# 1557 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_DATE_GetMonth(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->DR) & (((0x1UL << (12U)) | (0xFUL << (8U)))))) >> (8U));
}
# 1571 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DATE_SetDay(RTC_TypeDef *RTCx, uint32_t Day)
{
  (((RTCx->DR)) = ((((((RTCx->DR))) & (~(((0x3UL << (4U)) | (0xFUL << (0U)))))) | ((((Day & 0xF0U) << ((4U) - 4U)) | ((Day & 0x0FU) << (0U)))))))
                                                                                          ;
}
# 1587 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_DATE_GetDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->DR) & (((0x3UL << (4U)) | (0xFUL << (0U)))))) >> (0U));
}
# 1627 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DATE_Config(RTC_TypeDef *RTCx, uint32_t WeekDay, uint32_t Day, uint32_t Month, uint32_t Year)
{
  register uint32_t temp = 0U;

  temp = (WeekDay << (13U)) |
         (((Year & 0xF0U) << ((20U) - 4U)) | ((Year & 0x0FU) << (16U))) |
         (((Month & 0xF0U) << ((12U) - 4U)) | ((Month & 0x0FU) << (8U))) |
         (((Day & 0xF0U) << ((4U) - 4U)) | ((Day & 0x0FU) << (0U)));

  (((RTCx->DR)) = ((((((RTCx->DR))) & (~(((0x7UL << (13U)) | (0x1UL << (12U)) | (0xFUL << (8U)) | (0x3UL << (4U)) | (0xFUL << (0U)) | (0xFUL << (20U)) | (0xFUL << (16U)))))) | (temp))));
}
# 1655 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_DATE_Get(RTC_TypeDef *RTCx)
{
  register uint32_t temp = 0U;

  temp = ((RTCx->DR) & (((0x7UL << (13U)) | (0x1UL << (12U)) | (0xFUL << (8U)) | (0x3UL << (4U)) | (0xFUL << (0U)) | (0xFUL << (20U)) | (0xFUL << (16U)))));
  return (uint32_t)((((temp & (0x7UL << (13U))) >> (13U)) << 24U) |
                    (((((temp & (0x3UL << (4U))) >> (4U)) << 4U) | ((temp & (0xFUL << (0U))) >> (0U))) << 16U) |
                    (((((temp & (0x1UL << (12U))) >> (12U)) << 4U) | ((temp & (0xFUL << (8U))) >> (8U))) << 8U) |
                    ((((temp & (0xFUL << (20U))) >> (20U)) << 4U) | ((temp & (0xFUL << (16U))) >> (16U))));
}
# 1681 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_Enable(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (8U))));
}
# 1693 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_Disable(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (8U))));
}
# 1714 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_SetMask(RTC_TypeDef *RTCx, uint32_t Mask)
{
  (((RTCx->ALRMAR)) = ((((((RTCx->ALRMAR))) & (~((0x1UL << (31U)) | (0x1UL << (23U)) | (0x1UL << (15U)) | (0x1UL << (7U))))) | (Mask))));
}
# 1734 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMA_GetMask(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMAR) & ((0x1UL << (31U)) | (0x1UL << (23U)) | (0x1UL << (15U)) | (0x1UL << (7U)))));
}







static inline void LL_RTC_ALMA_EnableWeekday(RTC_TypeDef *RTCx)
{
  ((RTCx->ALRMAR) |= ((0x1UL << (30U))));
}







static inline void LL_RTC_ALMA_DisableWeekday(RTC_TypeDef *RTCx)
{
  ((RTCx->ALRMAR) &= ~((0x1UL << (30U))));
}
# 1770 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_SetDay(RTC_TypeDef *RTCx, uint32_t Day)
{
  (((RTCx->ALRMAR)) = ((((((RTCx->ALRMAR))) & (~(((0x3UL << (28U)) | (0xFUL << (24U)))))) | ((((Day & 0xF0U) << ((28U) - 4U)) | ((Day & 0x0FU) << (24U)))))))
                                                                                                  ;
}
# 1784 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMA_GetDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->ALRMAR) & (((0x3UL << (28U)) | (0xFUL << (24U)))))) >> (24U));
}
# 1803 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_SetWeekDay(RTC_TypeDef *RTCx, uint32_t WeekDay)
{
  (((RTCx->ALRMAR)) = ((((((RTCx->ALRMAR))) & (~((0xFUL << (24U))))) | (WeekDay << (24U)))));
}
# 1821 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMA_GetWeekDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMAR) & ((0xFUL << (24U)))) >> (24U));
}
# 1835 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_SetTimeFormat(RTC_TypeDef *RTCx, uint32_t TimeFormat)
{
  (((RTCx->ALRMAR)) = ((((((RTCx->ALRMAR))) & (~((0x1UL << (22U))))) | (TimeFormat))));
}
# 1848 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMA_GetTimeFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMAR) & ((0x1UL << (22U)))));
}
# 1862 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_SetHour(RTC_TypeDef *RTCx, uint32_t Hours)
{
  (((RTCx->ALRMAR)) = ((((((RTCx->ALRMAR))) & (~(((0x3UL << (20U)) | (0xFUL << (16U)))))) | ((((Hours & 0xF0U) << ((20U) - 4U)) | ((Hours & 0x0FU) << (16U)))))))
                                                                                                      ;
}
# 1876 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMA_GetHour(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->ALRMAR) & (((0x3UL << (20U)) | (0xFUL << (16U)))))) >> (16U));
}
# 1890 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_SetMinute(RTC_TypeDef *RTCx, uint32_t Minutes)
{
  (((RTCx->ALRMAR)) = ((((((RTCx->ALRMAR))) & (~(((0x7UL << (12U)) | (0xFUL << (8U)))))) | ((((Minutes & 0xF0U) << ((12U) - 4U)) | ((Minutes & 0x0FU) << (8U)))))))
                                                                                                            ;
}
# 1904 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMA_GetMinute(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->ALRMAR) & (((0x7UL << (12U)) | (0xFUL << (8U)))))) >> (8U));
}
# 1918 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_SetSecond(RTC_TypeDef *RTCx, uint32_t Seconds)
{
  (((RTCx->ALRMAR)) = ((((((RTCx->ALRMAR))) & (~(((0x7UL << (4U)) | (0xFUL << (0U)))))) | ((((Seconds & 0xF0U) << ((4U) - 4U)) | ((Seconds & 0x0FU) << (0U)))))))
                                                                                                          ;
}
# 1932 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMA_GetSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->ALRMAR) & (((0x7UL << (4U)) | (0xFUL << (0U)))))) >> (0U));
}
# 1955 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_ConfigTime(RTC_TypeDef *RTCx, uint32_t Format12_24, uint32_t Hours, uint32_t Minutes, uint32_t Seconds)
{
  register uint32_t temp = 0U;

  temp = Format12_24 | (((Hours & 0xF0U) << ((20U) - 4U)) | ((Hours & 0x0FU) << (16U))) |
         (((Minutes & 0xF0U) << ((12U) - 4U)) | ((Minutes & 0x0FU) << (8U))) |
         (((Seconds & 0xF0U) << ((4U) - 4U)) | ((Seconds & 0x0FU) << (0U)));

  (((RTCx->ALRMAR)) = ((((((RTCx->ALRMAR))) & (~((0x1UL << (22U)) | (0x3UL << (20U)) | (0xFUL << (16U)) | (0x7UL << (12U)) | (0xFUL << (8U)) | (0x7UL << (4U)) | (0xFUL << (0U))))) | (temp))));
}
# 1979 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMA_GetTime(RTC_TypeDef *RTCx)
{
  return (uint32_t)((LL_RTC_ALMA_GetHour(RTCx) << 16U) | (LL_RTC_ALMA_GetMinute(RTCx) << 8U) | LL_RTC_ALMA_GetSecond(RTCx));
}
# 1993 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_SetSubSecondMask(RTC_TypeDef *RTCx, uint32_t Mask)
{
  (((RTCx->ALRMASSR)) = ((((((RTCx->ALRMASSR))) & (~((0xFUL << (24U))))) | (Mask << (24U)))));
}







static inline uint32_t LL_RTC_ALMA_GetSubSecondMask(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMASSR) & ((0xFUL << (24U)))) >> (24U));
}
# 2016 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMA_SetSubSecond(RTC_TypeDef *RTCx, uint32_t Subsecond)
{
  (((RTCx->ALRMASSR)) = ((((((RTCx->ALRMASSR))) & (~((0x7FFFUL << (0U))))) | (Subsecond))));
}







static inline uint32_t LL_RTC_ALMA_GetSubSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMASSR) & ((0x7FFFUL << (0U)))));
}
# 2047 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_Enable(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (9U))));
}
# 2059 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_Disable(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (9U))));
}
# 2080 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_SetMask(RTC_TypeDef *RTCx, uint32_t Mask)
{
  (((RTCx->ALRMBR)) = ((((((RTCx->ALRMBR))) & (~((0x1UL << (31U)) | (0x1UL << (23U)) | (0x1UL << (15U)) | (0x1UL << (7U))))) | (Mask))));
}
# 2100 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMB_GetMask(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMBR) & ((0x1UL << (31U)) | (0x1UL << (23U)) | (0x1UL << (15U)) | (0x1UL << (7U)))));
}







static inline void LL_RTC_ALMB_EnableWeekday(RTC_TypeDef *RTCx)
{
  ((RTCx->ALRMBR) |= ((0x1UL << (30U))));
}







static inline void LL_RTC_ALMB_DisableWeekday(RTC_TypeDef *RTCx)
{
  ((RTCx->ALRMBR) &= ~((0x1UL << (30U))));
}
# 2136 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_SetDay(RTC_TypeDef *RTCx, uint32_t Day)
{
  (((RTCx->ALRMBR)) = ((((((RTCx->ALRMBR))) & (~(((0x3UL << (28U)) | (0xFUL << (24U)))))) | ((((Day & 0xF0U) << ((28U) - 4U)) | ((Day & 0x0FU) << (24U)))))))
                                                                                                  ;
}
# 2150 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMB_GetDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->ALRMBR) & (((0x3UL << (28U)) | (0xFUL << (24U)))))) >> (24U));
}
# 2169 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_SetWeekDay(RTC_TypeDef *RTCx, uint32_t WeekDay)
{
  (((RTCx->ALRMBR)) = ((((((RTCx->ALRMBR))) & (~((0xFUL << (24U))))) | (WeekDay << (24U)))));
}
# 2187 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMB_GetWeekDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMBR) & ((0xFUL << (24U)))) >> (24U));
}
# 2201 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_SetTimeFormat(RTC_TypeDef *RTCx, uint32_t TimeFormat)
{
  (((RTCx->ALRMBR)) = ((((((RTCx->ALRMBR))) & (~((0x1UL << (22U))))) | (TimeFormat))));
}
# 2214 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMB_GetTimeFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMBR) & ((0x1UL << (22U)))));
}
# 2228 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_SetHour(RTC_TypeDef *RTCx, uint32_t Hours)
{
  (((RTCx->ALRMBR)) = ((((((RTCx->ALRMBR))) & (~(((0x3UL << (20U)) | (0xFUL << (16U)))))) | ((((Hours & 0xF0U) << ((20U) - 4U)) | ((Hours & 0x0FU) << (16U)))))))
                                                                                                      ;
}
# 2242 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMB_GetHour(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->ALRMBR) & (((0x3UL << (20U)) | (0xFUL << (16U)))))) >> (16U));
}
# 2256 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_SetMinute(RTC_TypeDef *RTCx, uint32_t Minutes)
{
  (((RTCx->ALRMBR)) = ((((((RTCx->ALRMBR))) & (~(((0x7UL << (12U)) | (0xFUL << (8U)))))) | ((((Minutes & 0xF0U) << ((12U) - 4U)) | ((Minutes & 0x0FU) << (8U)))))))
                                                                                                            ;
}
# 2270 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMB_GetMinute(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->ALRMBR) & (((0x7UL << (12U)) | (0xFUL << (8U)))))) >> (8U));
}
# 2284 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_SetSecond(RTC_TypeDef *RTCx, uint32_t Seconds)
{
  (((RTCx->ALRMBR)) = ((((((RTCx->ALRMBR))) & (~(((0x7UL << (4U)) | (0xFUL << (0U)))))) | ((((Seconds & 0xF0U) << ((4U) - 4U)) | ((Seconds & 0x0FU) << (0U)))))))
                                                                                                          ;
}
# 2298 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMB_GetSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)((((RTCx->ALRMBR) & (((0x7UL << (4U)) | (0xFUL << (0U)))))) >> (0U));
}
# 2321 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_ConfigTime(RTC_TypeDef *RTCx, uint32_t Format12_24, uint32_t Hours, uint32_t Minutes, uint32_t Seconds)
{
  register uint32_t temp = 0U;

  temp = Format12_24 | (((Hours & 0xF0U) << ((20U) - 4U)) | ((Hours & 0x0FU) << (16U))) |
         (((Minutes & 0xF0U) << ((12U) - 4U)) | ((Minutes & 0x0FU) << (8U))) |
         (((Seconds & 0xF0U) << ((4U) - 4U)) | ((Seconds & 0x0FU) << (0U)));

  (((RTCx->ALRMBR)) = ((((((RTCx->ALRMBR))) & (~((0x1UL << (22U)) | (0x3UL << (20U)) | (0xFUL << (16U)) | (0x7UL << (12U)) | (0xFUL << (8U)) | (0x7UL << (4U)) | (0xFUL << (0U))))) | (temp))));
}
# 2345 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_ALMB_GetTime(RTC_TypeDef *RTCx)
{
  return (uint32_t)((LL_RTC_ALMB_GetHour(RTCx) << 16U) | (LL_RTC_ALMB_GetMinute(RTCx) << 8U) | LL_RTC_ALMB_GetSecond(RTCx));
}
# 2359 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_SetSubSecondMask(RTC_TypeDef *RTCx, uint32_t Mask)
{
  (((RTCx->ALRMBSSR)) = ((((((RTCx->ALRMBSSR))) & (~((0xFUL << (24U))))) | (Mask << (24U)))));
}







static inline uint32_t LL_RTC_ALMB_GetSubSecondMask(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMBSSR) & ((0xFUL << (24U)))) >> (24U));
}
# 2382 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ALMB_SetSubSecond(RTC_TypeDef *RTCx, uint32_t Subsecond)
{
  (((RTCx->ALRMBSSR)) = ((((((RTCx->ALRMBSSR))) & (~((0x7FFFUL << (0U))))) | (Subsecond))));
}







static inline uint32_t LL_RTC_ALMB_GetSubSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->ALRMBSSR) & ((0x7FFFUL << (0U)))));
}
# 2413 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TS_Enable(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (11U))));
}
# 2425 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TS_Disable(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (11U))));
}
# 2441 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TS_SetActiveEdge(RTC_TypeDef *RTCx, uint32_t Edge)
{
  (((RTCx->CR)) = ((((((RTCx->CR))) & (~((0x1UL << (3U))))) | (Edge))));
}
# 2455 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetActiveEdge(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->CR) & ((0x1UL << (3U)))));
}
# 2468 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetTimeFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSTR) & ((0x1UL << (22U)))));
}
# 2481 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetHour(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSTR) & ((0x3UL << (20U)) | (0xFUL << (16U)))) >> (16U));
}
# 2494 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetMinute(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSTR) & ((0x7UL << (12U)) | (0xFUL << (8U)))) >> (8U));
}
# 2507 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSTR) & ((0x7UL << (4U)) | (0xFUL << (0U)))));
}
# 2525 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetTime(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSTR) & ((0x3UL << (20U)) | (0xFUL << (16U)) | (0x7UL << (12U)) | (0xFUL << (8U)) | (0x7UL << (4U)) | (0xFUL << (0U))))
                                                                                                                 );
}
# 2544 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetWeekDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSDR) & ((0x7UL << (13U)))) >> (13U));
}
# 2569 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetMonth(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSDR) & ((0x1UL << (12U)) | (0xFUL << (8U)))) >> (8U));
}
# 2582 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSDR) & ((0x3UL << (4U)) | (0xFUL << (0U)))));
}
# 2599 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TS_GetDate(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSDR) & ((0x7UL << (13U)) | (0x1UL << (12U)) | (0xFUL << (8U)) | (0x3UL << (4U)) | (0xFUL << (0U)))));
}







static inline uint32_t LL_RTC_TS_GetSubSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TSSSR) & ((0xFFFFUL << (0U)))));
}
# 2622 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TS_EnableOnTamper(RTC_TypeDef *RTCx)
{
  ((RTCx->TAFCR) |= ((0x1UL << (7U))));
}







static inline void LL_RTC_TS_DisableOnTamper(RTC_TypeDef *RTCx)
{
  ((RTCx->TAFCR) &= ~((0x1UL << (7U))));
}
# 2661 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TAMPER_Enable(RTC_TypeDef *RTCx, uint32_t Tamper)
{
  ((RTCx->TAFCR) |= (Tamper));
}
# 2680 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TAMPER_Disable(RTC_TypeDef *RTCx, uint32_t Tamper)
{
  ((RTCx->TAFCR) &= ~(Tamper));
}
# 2692 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TAMPER_DisablePullUp(RTC_TypeDef *RTCx)
{
  ((RTCx->TAFCR) |= ((0x1UL << (15U))));
}







static inline void LL_RTC_TAMPER_EnablePullUp(RTC_TypeDef *RTCx)
{
  ((RTCx->TAFCR) &= ~((0x1UL << (15U))));
}
# 2721 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TAMPER_SetPrecharge(RTC_TypeDef *RTCx, uint32_t Duration)
{
  (((RTCx->TAFCR)) = ((((((RTCx->TAFCR))) & (~((0x3UL << (13U))))) | (Duration))));
}
# 2736 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TAMPER_GetPrecharge(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TAFCR) & ((0x3UL << (13U)))));
}
# 2754 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TAMPER_SetFilterCount(RTC_TypeDef *RTCx, uint32_t FilterCount)
{
  (((RTCx->TAFCR)) = ((((((RTCx->TAFCR))) & (~((0x3UL << (11U))))) | (FilterCount))));
}
# 2769 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TAMPER_GetFilterCount(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TAFCR) & ((0x3UL << (11U)))));
}
# 2791 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TAMPER_SetSamplingFreq(RTC_TypeDef *RTCx, uint32_t SamplingFreq)
{
  (((RTCx->TAFCR)) = ((((((RTCx->TAFCR))) & (~((0x7UL << (8U))))) | (SamplingFreq))));
}
# 2810 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_TAMPER_GetSamplingFreq(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->TAFCR) & ((0x7UL << (8U)))));
}
# 2830 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TAMPER_EnableActiveLevel(RTC_TypeDef *RTCx, uint32_t Tamper)
{
  ((RTCx->TAFCR) |= (Tamper));
}
# 2849 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_TAMPER_DisableActiveLevel(RTC_TypeDef *RTCx, uint32_t Tamper)
{
  ((RTCx->TAFCR) &= ~(Tamper));
}
# 2870 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_WAKEUP_Enable(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (10U))));
}
# 2882 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_WAKEUP_Disable(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (10U))));
}







static inline uint32_t LL_RTC_WAKEUP_IsEnabled(RTC_TypeDef *RTCx)
{
  return (((RTCx->CR) & ((0x1UL << (10U)))) == ((0x1UL << (10U))));
}
# 2913 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_WAKEUP_SetClock(RTC_TypeDef *RTCx, uint32_t WakeupClock)
{
  (((RTCx->CR)) = ((((((RTCx->CR))) & (~((0x7UL << (0U))))) | (WakeupClock))));
}
# 2930 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_WAKEUP_GetClock(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->CR) & ((0x7UL << (0U)))));
}
# 2943 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_WAKEUP_SetAutoReload(RTC_TypeDef *RTCx, uint32_t Value)
{
  (((RTCx->WUTR)) = ((((((RTCx->WUTR))) & (~((0xFFFFUL << (0U))))) | (Value))));
}







static inline uint32_t LL_RTC_WAKEUP_GetAutoReload(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->WUTR) & ((0xFFFFUL << (0U)))));
}
# 3011 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_BAK_SetRegister(RTC_TypeDef *RTCx, uint32_t BackupRegister, uint32_t Data)
{
  register uint32_t tmp = 0U;

  tmp = (uint32_t)(&(RTCx->BKP0R));
  tmp += (BackupRegister * 4U);


  *(volatile uint32_t *)tmp = (uint32_t)Data;
}
# 3063 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_BAK_GetRegister(RTC_TypeDef *RTCx, uint32_t BackupRegister)
{
  register uint32_t tmp = 0U;

  tmp = (uint32_t)(&(RTCx->BKP0R));
  tmp += (BackupRegister * 4U);


  return (*(volatile uint32_t *)tmp);
}
# 3095 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_CAL_SetOutputFreq(RTC_TypeDef *RTCx, uint32_t Frequency)
{
  (((RTCx->CR)) = ((((((RTCx->CR))) & (~((0x1UL << (23U)) | (0x1UL << (19U))))) | (Frequency))));
}
# 3110 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_CAL_GetOutputFreq(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->CR) & ((0x1UL << (23U)) | (0x1UL << (19U)))));
}
# 3126 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_CAL_SetPulse(RTC_TypeDef *RTCx, uint32_t Pulse)
{
  (((RTCx->CALR)) = ((((((RTCx->CALR))) & (~((0x1UL << (15U))))) | (Pulse))));
}







static inline uint32_t LL_RTC_CAL_IsPulseInserted(RTC_TypeDef *RTCx)
{
  return (((RTCx->CALR) & ((0x1UL << (15U)))) == ((0x1UL << (15U))));
}
# 3155 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_CAL_SetPeriod(RTC_TypeDef *RTCx, uint32_t Period)
{
  (((RTCx->CALR)) = ((((((RTCx->CALR))) & (~((0x1UL << (14U)) | (0x1UL << (13U))))) | (Period))));
}
# 3170 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_CAL_GetPeriod(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->CALR) & ((0x1UL << (14U)) | (0x1UL << (13U)))));
}
# 3184 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_CAL_SetMinus(RTC_TypeDef *RTCx, uint32_t CalibMinus)
{
  (((RTCx->CALR)) = ((((((RTCx->CALR))) & (~((0x1FFUL << (0U))))) | (CalibMinus))));
}







static inline uint32_t LL_RTC_CAL_GetMinus(RTC_TypeDef *RTCx)
{
  return (uint32_t)(((RTCx->CALR) & ((0x1FFUL << (0U)))));
}
# 3214 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsActiveFlag_RECALP(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (16U)))) == ((0x1UL << (16U))));
}
# 3226 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsActiveFlag_TAMP3(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (15U)))) == ((0x1UL << (15U))));
}
# 3239 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsActiveFlag_TAMP2(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (14U)))) == ((0x1UL << (14U))));
}
# 3252 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsActiveFlag_TAMP1(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (13U)))) == ((0x1UL << (13U))));
}
# 3264 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsActiveFlag_TSOV(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (12U)))) == ((0x1UL << (12U))));
}







static inline uint32_t LL_RTC_IsActiveFlag_TS(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (11U)))) == ((0x1UL << (11U))));
}
# 3287 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsActiveFlag_WUT(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (10U)))) == ((0x1UL << (10U))));
}
# 3299 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsActiveFlag_ALRB(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (9U)))) == ((0x1UL << (9U))));
}







static inline uint32_t LL_RTC_IsActiveFlag_ALRA(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (8U)))) == ((0x1UL << (8U))));
}
# 3322 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ClearFlag_TAMP3(RTC_TypeDef *RTCx)
{
  ((RTCx->ISR) = ((~(((0x1UL << (15U)) | (0x1UL << (7U))) & 0x0000FFFFU) | (RTCx->ISR & (0x1UL << (7U))))));
}
# 3335 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ClearFlag_TAMP2(RTC_TypeDef *RTCx)
{
  ((RTCx->ISR) = ((~(((0x1UL << (14U)) | (0x1UL << (7U))) & 0x0000FFFFU) | (RTCx->ISR & (0x1UL << (7U))))));
}
# 3348 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ClearFlag_TAMP1(RTC_TypeDef *RTCx)
{
  ((RTCx->ISR) = ((~(((0x1UL << (13U)) | (0x1UL << (7U))) & 0x0000FFFFU) | (RTCx->ISR & (0x1UL << (7U))))));
}
# 3360 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ClearFlag_TSOV(RTC_TypeDef *RTCx)
{
  ((RTCx->ISR) = ((~(((0x1UL << (12U)) | (0x1UL << (7U))) & 0x0000FFFFU) | (RTCx->ISR & (0x1UL << (7U))))));
}







static inline void LL_RTC_ClearFlag_TS(RTC_TypeDef *RTCx)
{
  ((RTCx->ISR) = ((~(((0x1UL << (11U)) | (0x1UL << (7U))) & 0x0000FFFFU) | (RTCx->ISR & (0x1UL << (7U))))));
}
# 3383 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ClearFlag_WUT(RTC_TypeDef *RTCx)
{
  ((RTCx->ISR) = ((~(((0x1UL << (10U)) | (0x1UL << (7U))) & 0x0000FFFFU) | (RTCx->ISR & (0x1UL << (7U))))));
}
# 3395 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_ClearFlag_ALRB(RTC_TypeDef *RTCx)
{
  ((RTCx->ISR) = ((~(((0x1UL << (9U)) | (0x1UL << (7U))) & 0x0000FFFFU) | (RTCx->ISR & (0x1UL << (7U))))));
}







static inline void LL_RTC_ClearFlag_ALRA(RTC_TypeDef *RTCx)
{
  ((RTCx->ISR) = ((~(((0x1UL << (8U)) | (0x1UL << (7U))) & 0x0000FFFFU) | (RTCx->ISR & (0x1UL << (7U))))));
}







static inline uint32_t LL_RTC_IsActiveFlag_INIT(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (6U)))) == ((0x1UL << (6U))));
}







static inline uint32_t LL_RTC_IsActiveFlag_RS(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (5U)))) == ((0x1UL << (5U))));
}







static inline void LL_RTC_ClearFlag_RS(RTC_TypeDef *RTCx)
{
  ((RTCx->ISR) = ((~(((0x1UL << (5U)) | (0x1UL << (7U))) & 0x0000FFFFU) | (RTCx->ISR & (0x1UL << (7U))))));
}







static inline uint32_t LL_RTC_IsActiveFlag_INITS(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (4U)))) == ((0x1UL << (4U))));
}







static inline uint32_t LL_RTC_IsActiveFlag_SHP(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (3U)))) == ((0x1UL << (3U))));
}
# 3473 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsActiveFlag_WUTW(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (2U)))) == ((0x1UL << (2U))));
}
# 3485 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsActiveFlag_ALRBW(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (1U)))) == ((0x1UL << (1U))));
}







static inline uint32_t LL_RTC_IsActiveFlag_ALRAW(RTC_TypeDef *RTCx)
{
  return (((RTCx->ISR) & ((0x1UL << (0U)))) == ((0x1UL << (0U))));
}
# 3516 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_EnableIT_TS(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (15U))));
}
# 3528 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DisableIT_TS(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (15U))));
}
# 3541 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_EnableIT_WUT(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (14U))));
}
# 3553 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DisableIT_WUT(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (14U))));
}
# 3566 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_EnableIT_ALRB(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (13U))));
}
# 3578 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DisableIT_ALRB(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (13U))));
}
# 3590 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_EnableIT_ALRA(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) |= ((0x1UL << (12U))));
}
# 3602 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline void LL_RTC_DisableIT_ALRA(RTC_TypeDef *RTCx)
{
  ((RTCx->CR) &= ~((0x1UL << (12U))));
}







static inline void LL_RTC_EnableIT_TAMP(RTC_TypeDef *RTCx)
{
  ((RTCx->TAFCR) |= ((0x1UL << (2U))));
}







static inline void LL_RTC_DisableIT_TAMP(RTC_TypeDef *RTCx)
{
  ((RTCx->TAFCR) &= ~((0x1UL << (2U))));
}







static inline uint32_t LL_RTC_IsEnabledIT_TS(RTC_TypeDef *RTCx)
{
  return (((RTCx->CR) & ((0x1UL << (15U)))) == ((0x1UL << (15U))));
}
# 3647 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsEnabledIT_WUT(RTC_TypeDef *RTCx)
{
  return (((RTCx->CR) & ((0x1UL << (14U)))) == ((0x1UL << (14U))));
}
# 3659 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
static inline uint32_t LL_RTC_IsEnabledIT_ALRB(RTC_TypeDef *RTCx)
{
  return (((RTCx->CR) & ((0x1UL << (13U)))) == ((0x1UL << (13U))));
}







static inline uint32_t LL_RTC_IsEnabledIT_ALRA(RTC_TypeDef *RTCx)
{
  return (((RTCx->CR) & ((0x1UL << (12U)))) == ((0x1UL << (12U))));
}







static inline uint32_t LL_RTC_IsEnabledIT_TAMP(RTC_TypeDef *RTCx)
{
  return (((RTCx->TAFCR) & ((0x1UL << (2U))))
                                     == ((0x1UL << (2U))));
}
# 3696 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_rtc.h"
ErrorStatus LL_RTC_DeInit(RTC_TypeDef *RTCx);
ErrorStatus LL_RTC_Init(RTC_TypeDef *RTCx, LL_RTC_InitTypeDef *RTC_InitStruct);
void LL_RTC_StructInit(LL_RTC_InitTypeDef *RTC_InitStruct);
ErrorStatus LL_RTC_TIME_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_TimeTypeDef *RTC_TimeStruct);
void LL_RTC_TIME_StructInit(LL_RTC_TimeTypeDef *RTC_TimeStruct);
ErrorStatus LL_RTC_DATE_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_DateTypeDef *RTC_DateStruct);
void LL_RTC_DATE_StructInit(LL_RTC_DateTypeDef *RTC_DateStruct);
ErrorStatus LL_RTC_ALMA_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_AlarmTypeDef *RTC_AlarmStruct);
ErrorStatus LL_RTC_ALMB_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_AlarmTypeDef *RTC_AlarmStruct);
void LL_RTC_ALMA_StructInit(LL_RTC_AlarmTypeDef *RTC_AlarmStruct);
void LL_RTC_ALMB_StructInit(LL_RTC_AlarmTypeDef *RTC_AlarmStruct);
ErrorStatus LL_RTC_EnterInitMode(RTC_TypeDef *RTCx);
ErrorStatus LL_RTC_ExitInitMode(RTC_TypeDef *RTCx);
ErrorStatus LL_RTC_WaitForSynchro(RTC_TypeDef *RTCx);
# 60 "./boards/stm32f3xx_hal_conf_base.h" 2
# 1 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h" 1
# 64 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
typedef struct
{

  uint32_t BaudRate;



  uint32_t DataWidth;




  uint32_t StopBits;




  uint32_t Parity;




  uint32_t TransferDirection;




  uint32_t HardwareFlowControl;




  uint32_t OverSampling;




} LL_USART_InitTypeDef;




typedef struct
{
  uint32_t ClockOutput;






  uint32_t ClockPolarity;





  uint32_t ClockPhase;





  uint32_t LastBitClockPulse;






} LL_USART_ClockInitTypeDef;
# 522 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_Enable(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (0U))));
}
# 536 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_Disable(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (0U))));
}







static inline uint32_t LL_USART_IsEnabled(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}
# 562 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableInStopMode(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (1U))));
}
# 576 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableInStopMode(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (1U))));
}
# 589 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledInStopMode(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}







static inline void LL_USART_EnableDirectionRx(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (2U))));
}







static inline void LL_USART_DisableDirectionRx(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (2U))));
}







static inline void LL_USART_EnableDirectionTx(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (3U))));
}







static inline void LL_USART_DisableDirectionTx(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (3U))));
}
# 651 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetTransferDirection(USART_TypeDef *USARTx, uint32_t TransferDirection)
{
  (((USARTx->CR1)) = ((((((USARTx->CR1))) & (~((0x1UL << (2U)) | (0x1UL << (3U))))) | (TransferDirection))));
}
# 667 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetTransferDirection(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR1) & ((0x1UL << (2U)) | (0x1UL << (3U)))));
}
# 686 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetParity(USART_TypeDef *USARTx, uint32_t Parity)
{
  (((USARTx->CR1)) = ((((((USARTx->CR1))) & (~((0x1UL << (9U)) | (0x1UL << (10U))))) | (Parity))));
}
# 701 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetParity(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR1) & ((0x1UL << (9U)) | (0x1UL << (10U)))));
}
# 715 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetWakeUpMethod(USART_TypeDef *USARTx, uint32_t Method)
{
  (((USARTx->CR1)) = ((((((USARTx->CR1))) & (~((0x1UL << (11U))))) | (Method))));
}
# 728 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetWakeUpMethod(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR1) & ((0x1UL << (11U)))));
}
# 746 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetDataWidth(USART_TypeDef *USARTx, uint32_t DataWidth)
{
  (((USARTx->CR1)) = ((((((USARTx->CR1))) & (~((0x1UL << (12U))))) | (DataWidth))));
}
# 763 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetDataWidth(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR1) & ((0x1UL << (12U)))));
}







static inline void LL_USART_EnableMuteMode(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (13U))));
}







static inline void LL_USART_DisableMuteMode(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (13U))));
}







static inline uint32_t LL_USART_IsEnabledMuteMode(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (13U)))) == ((0x1UL << (13U)))) ? 1UL : 0UL);
}
# 810 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetOverSampling(USART_TypeDef *USARTx, uint32_t OverSampling)
{
  (((USARTx->CR1)) = ((((((USARTx->CR1))) & (~((0x1UL << (15U))))) | (OverSampling))));
}
# 823 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetOverSampling(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR1) & ((0x1UL << (15U)))));
}
# 839 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetLastClkPulseOutput(USART_TypeDef *USARTx, uint32_t LastBitClockPulse)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (8U))))) | (LastBitClockPulse))));
}
# 855 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetLastClkPulseOutput(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (8U)))));
}
# 871 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetClockPhase(USART_TypeDef *USARTx, uint32_t ClockPhase)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (9U))))) | (ClockPhase))));
}
# 886 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetClockPhase(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (9U)))));
}
# 902 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetClockPolarity(USART_TypeDef *USARTx, uint32_t ClockPolarity)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (10U))))) | (ClockPolarity))));
}
# 917 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetClockPolarity(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (10U)))));
}
# 945 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigClock(USART_TypeDef *USARTx, uint32_t Phase, uint32_t Polarity, uint32_t LBCPOutput)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (9U)) | (0x1UL << (10U)) | (0x1UL << (8U))))) | (Phase | Polarity | LBCPOutput))));
}
# 958 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableSCLKOutput(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) |= ((0x1UL << (11U))));
}
# 971 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableSCLKOutput(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) &= ~((0x1UL << (11U))));
}
# 984 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledSCLKOutput(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR2) & ((0x1UL << (11U)))) == ((0x1UL << (11U)))) ? 1UL : 0UL);
}
# 1000 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetStopBitsLength(USART_TypeDef *USARTx, uint32_t StopBits)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x3UL << (12U))))) | (StopBits))));
}
# 1015 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetStopBitsLength(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x3UL << (12U)))));
}
# 1049 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigCharacter(USART_TypeDef *USARTx, uint32_t DataWidth, uint32_t Parity,
                                              uint32_t StopBits)
{
  (((USARTx->CR1)) = ((((((USARTx->CR1))) & (~((0x1UL << (9U)) | (0x1UL << (10U)) | (0x1UL << (12U))))) | (Parity | DataWidth))));
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x3UL << (12U))))) | (StopBits))));
}
# 1065 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetTXRXSwap(USART_TypeDef *USARTx, uint32_t SwapConfig)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (15U))))) | (SwapConfig))));
}
# 1078 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetTXRXSwap(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (15U)))));
}
# 1092 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetRXPinLevel(USART_TypeDef *USARTx, uint32_t PinInvMethod)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (16U))))) | (PinInvMethod))));
}
# 1105 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetRXPinLevel(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (16U)))));
}
# 1119 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetTXPinLevel(USART_TypeDef *USARTx, uint32_t PinInvMethod)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (17U))))) | (PinInvMethod))));
}
# 1132 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetTXPinLevel(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (17U)))));
}
# 1148 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetBinaryDataLogic(USART_TypeDef *USARTx, uint32_t DataLogic)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (18U))))) | (DataLogic))));
}
# 1161 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetBinaryDataLogic(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (18U)))));
}
# 1177 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetTransferBitOrder(USART_TypeDef *USARTx, uint32_t BitOrder)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (19U))))) | (BitOrder))));
}
# 1192 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetTransferBitOrder(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (19U)))));
}
# 1205 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableAutoBaudRate(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) |= ((0x1UL << (20U))));
}
# 1218 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableAutoBaudRate(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) &= ~((0x1UL << (20U))));
}
# 1231 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledAutoBaud(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR2) & ((0x1UL << (20U)))) == ((0x1UL << (20U)))) ? 1UL : 0UL);
}
# 1249 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetAutoBaudRateMode(USART_TypeDef *USARTx, uint32_t AutoBaudRateMode)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x3UL << (21U))))) | (AutoBaudRateMode))));
}
# 1266 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetAutoBaudRateMode(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x3UL << (21U)))));
}







static inline void LL_USART_EnableRxTimeout(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) |= ((0x1UL << (23U))));
}







static inline void LL_USART_DisableRxTimeout(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) &= ~((0x1UL << (23U))));
}







static inline uint32_t LL_USART_IsEnabledRxTimeout(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR2) & ((0x1UL << (23U)))) == ((0x1UL << (23U)))) ? 1UL : 0UL);
}
# 1327 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigNodeAddress(USART_TypeDef *USARTx, uint32_t AddressLen, uint32_t NodeAddress)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0xFFUL << (24U)) | (0x1UL << (4U))))) | ((uint32_t)(AddressLen | (NodeAddress << (24U)))))))
                                                                         ;
}
# 1343 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetNodeAddress(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0xFFUL << (24U)))) >> (24U));
}
# 1356 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetNodeAddressLen(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (4U)))));
}
# 1369 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableRTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (8U))));
}
# 1382 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableRTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (8U))));
}
# 1395 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableCTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (9U))));
}
# 1408 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableCTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (9U))));
}
# 1427 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetHWFlowCtrl(USART_TypeDef *USARTx, uint32_t HardwareFlowControl)
{
  (((USARTx->CR3)) = ((((((USARTx->CR3))) & (~((0x1UL << (8U)) | (0x1UL << (9U))))) | (HardwareFlowControl))));
}
# 1445 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetHWFlowCtrl(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR3) & ((0x1UL << (8U)) | (0x1UL << (9U)))));
}







static inline void LL_USART_EnableOneBitSamp(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (11U))));
}







static inline void LL_USART_DisableOneBitSamp(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (11U))));
}







static inline uint32_t LL_USART_IsEnabledOneBitSamp(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (11U)))) == ((0x1UL << (11U)))) ? 1UL : 0UL);
}







static inline void LL_USART_EnableOverrunDetect(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (12U))));
}







static inline void LL_USART_DisableOverrunDetect(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (12U))));
}







static inline uint32_t LL_USART_IsEnabledOverrunDetect(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (12U)))) != (0x1UL << (12U))) ? 1UL : 0UL);
}
# 1528 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetWKUPType(USART_TypeDef *USARTx, uint32_t Type)
{
  (((USARTx->CR3)) = ((((((USARTx->CR3))) & (~((0x3UL << (20U))))) | (Type))));
}
# 1544 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetWKUPType(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR3) & ((0x3UL << (20U)))));
}
# 1565 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetBaudRate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t OverSampling,
                                          uint32_t BaudRate)
{
  uint32_t usartdiv;
  register uint32_t brrtemp;

  if (OverSampling == (0x1UL << (15U)))
  {
    usartdiv = (uint16_t)(((((PeriphClk)*2U) + ((BaudRate)/2U))/(BaudRate)));
    brrtemp = usartdiv & 0xFFF0U;
    brrtemp |= (uint16_t)((usartdiv & (uint16_t)0x000FU) >> 1U);
    USARTx->BRR = brrtemp;
  }
  else
  {
    USARTx->BRR = (uint16_t)((((PeriphClk) + ((BaudRate)/2U))/(BaudRate)));
  }
}
# 1597 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetBaudRate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t OverSampling)
{
  register uint32_t usartdiv;
  register uint32_t brrresult = 0x0U;

  usartdiv = USARTx->BRR;

  if (usartdiv == 0U)
  {

  }
  else if (OverSampling == (0x1UL << (15U)))
  {
    usartdiv = (uint16_t)((usartdiv & 0xFFF0U) | ((usartdiv & 0x0007U) << 1U)) ;
    if (usartdiv != 0U)
    {
      brrresult = (PeriphClk * 2U) / usartdiv;
    }
  }
  else
  {
    if ((usartdiv & 0xFFFFU) != 0U)
    {
      brrresult = PeriphClk / usartdiv;
    }
  }
  return (brrresult);
}
# 1633 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetRxTimeout(USART_TypeDef *USARTx, uint32_t Timeout)
{
  (((USARTx->RTOR)) = ((((((USARTx->RTOR))) & (~((0xFFFFFFUL << (0U))))) | (Timeout))));
}







static inline uint32_t LL_USART_GetRxTimeout(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->RTOR) & ((0xFFFFFFUL << (0U)))));
}
# 1656 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetBlockLength(USART_TypeDef *USARTx, uint32_t BlockLength)
{
  (((USARTx->RTOR)) = ((((((USARTx->RTOR))) & (~((0xFFUL << (24U))))) | (BlockLength << (24U)))));
}







static inline uint32_t LL_USART_GetBlockLength(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->RTOR) & ((0xFFUL << (24U)))) >> (24U));
}
# 1688 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableIrda(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (1U))));
}
# 1701 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableIrda(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (1U))));
}
# 1714 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledIrda(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}
# 1730 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetIrdaPowerMode(USART_TypeDef *USARTx, uint32_t PowerMode)
{
  (((USARTx->CR3)) = ((((((USARTx->CR3))) & (~((0x1UL << (2U))))) | (PowerMode))));
}
# 1745 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetIrdaPowerMode(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR3) & ((0x1UL << (2U)))));
}
# 1760 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetIrdaPrescaler(USART_TypeDef *USARTx, uint32_t PrescalerValue)
{
  (((USARTx->GTPR)) = ((((((USARTx->GTPR))) & (~((uint16_t)(0xFFUL << (0U))))) | ((uint16_t)PrescalerValue))));
}
# 1774 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetIrdaPrescaler(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->GTPR) & ((0xFFUL << (0U)))));
}
# 1795 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableSmartcardNACK(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (4U))));
}
# 1808 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableSmartcardNACK(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (4U))));
}
# 1821 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledSmartcardNACK(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}
# 1834 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableSmartcard(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (5U))));
}
# 1847 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableSmartcard(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (5U))));
}
# 1860 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledSmartcard(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}
# 1879 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetSmartcardAutoRetryCount(USART_TypeDef *USARTx, uint32_t AutoRetryCount)
{
  (((USARTx->CR3)) = ((((((USARTx->CR3))) & (~((0x7UL << (17U))))) | (AutoRetryCount << (17U)))));
}
# 1892 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetSmartcardAutoRetryCount(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR3) & ((0x7UL << (17U)))) >> (17U));
}
# 1907 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetSmartcardPrescaler(USART_TypeDef *USARTx, uint32_t PrescalerValue)
{
  (((USARTx->GTPR)) = ((((((USARTx->GTPR))) & (~((uint16_t)(0xFFUL << (0U))))) | ((uint16_t)PrescalerValue))));
}
# 1921 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetSmartcardPrescaler(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->GTPR) & ((0xFFUL << (0U)))));
}
# 1936 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetSmartcardGuardTime(USART_TypeDef *USARTx, uint32_t GuardTime)
{
  (((USARTx->GTPR)) = ((((((USARTx->GTPR))) & (~((uint16_t)(0xFFUL << (8U))))) | ((uint16_t)(GuardTime << (8U))))));
}
# 1950 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetSmartcardGuardTime(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->GTPR) & ((0xFFUL << (8U)))) >> (8U));
}
# 1971 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableHalfDuplex(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (3U))));
}
# 1984 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableHalfDuplex(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (3U))));
}
# 1997 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledHalfDuplex(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}
# 2021 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetLINBrkDetectionLen(USART_TypeDef *USARTx, uint32_t LINBDLength)
{
  (((USARTx->CR2)) = ((((((USARTx->CR2))) & (~((0x1UL << (5U))))) | (LINBDLength))));
}
# 2036 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetLINBrkDetectionLen(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR2) & ((0x1UL << (5U)))));
}
# 2049 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableLIN(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) |= ((0x1UL << (14U))));
}
# 2062 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableLIN(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) &= ~((0x1UL << (14U))));
}
# 2075 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledLIN(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR2) & ((0x1UL << (14U)))) == ((0x1UL << (14U)))) ? 1UL : 0UL);
}
# 2097 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetDEDeassertionTime(USART_TypeDef *USARTx, uint32_t Time)
{
  (((USARTx->CR1)) = ((((((USARTx->CR1))) & (~((0x1FUL << (16U))))) | (Time << (16U)))));
}
# 2110 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetDEDeassertionTime(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR1) & ((0x1FUL << (16U)))) >> (16U));
}
# 2124 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetDEAssertionTime(USART_TypeDef *USARTx, uint32_t Time)
{
  (((USARTx->CR1)) = ((((((USARTx->CR1))) & (~((0x1FUL << (21U))))) | (Time << (21U)))));
}
# 2137 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetDEAssertionTime(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR1) & ((0x1FUL << (21U)))) >> (21U));
}
# 2150 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableDEMode(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (14U))));
}
# 2163 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableDEMode(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (14U))));
}
# 2176 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledDEMode(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (14U)))) == ((0x1UL << (14U)))) ? 1UL : 0UL);
}
# 2192 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_SetDESignalPolarity(USART_TypeDef *USARTx, uint32_t Polarity)
{
  (((USARTx->CR3)) = ((((((USARTx->CR3))) & (~((0x1UL << (15U))))) | (Polarity))));
}
# 2207 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_GetDESignalPolarity(USART_TypeDef *USARTx)
{
  return (uint32_t)(((USARTx->CR3) & ((0x1UL << (15U)))));
}
# 2245 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigAsyncMode(USART_TypeDef *USARTx)
{




  ((USARTx->CR2) &= ~(((0x1UL << (14U)) | (0x1UL << (11U)))));
  ((USARTx->CR3) &= ~(((0x1UL << (5U)) | (0x1UL << (1U)) | (0x1UL << (3U)))));
}
# 2282 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigSyncMode(USART_TypeDef *USARTx)
{




  ((USARTx->CR2) &= ~(((0x1UL << (14U)))));
  ((USARTx->CR3) &= ~(((0x1UL << (5U)) | (0x1UL << (1U)) | (0x1UL << (3U)))));

  ((USARTx->CR2) |= ((0x1UL << (11U))));
}
# 2323 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigLINMode(USART_TypeDef *USARTx)
{




  ((USARTx->CR2) &= ~(((0x1UL << (11U)) | (0x3UL << (12U)))));
  ((USARTx->CR3) &= ~(((0x1UL << (1U)) | (0x1UL << (5U)) | (0x1UL << (3U)))));

  ((USARTx->CR2) |= ((0x1UL << (14U))));
}
# 2362 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigHalfDuplexMode(USART_TypeDef *USARTx)
{




  ((USARTx->CR2) &= ~(((0x1UL << (14U)) | (0x1UL << (11U)))));
  ((USARTx->CR3) &= ~(((0x1UL << (5U)) | (0x1UL << (1U)))));

  ((USARTx->CR3) |= ((0x1UL << (3U))));
}
# 2403 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigSmartcardMode(USART_TypeDef *USARTx)
{




  ((USARTx->CR2) &= ~(((0x1UL << (14U)))));
  ((USARTx->CR3) &= ~(((0x1UL << (1U)) | (0x1UL << (3U)))));


  ((USARTx->CR2) |= (((0x1UL << (12U)) | (0x2UL << (12U)) | (0x1UL << (11U)))));

  ((USARTx->CR3) |= ((0x1UL << (5U))));
}
# 2447 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigIrdaMode(USART_TypeDef *USARTx)
{




  ((USARTx->CR2) &= ~(((0x1UL << (14U)) | (0x1UL << (11U)) | (0x3UL << (12U)))));
  ((USARTx->CR3) &= ~(((0x1UL << (5U)) | (0x1UL << (3U)))));

  ((USARTx->CR3) |= ((0x1UL << (1U))));
}
# 2486 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ConfigMultiProcessMode(USART_TypeDef *USARTx)
{




  ((USARTx->CR2) &= ~(((0x1UL << (14U)) | (0x1UL << (11U)))));
  ((USARTx->CR3) &= ~(((0x1UL << (5U)) | (0x1UL << (3U)) | (0x1UL << (1U)))));
}
# 2510 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsActiveFlag_PE(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_FE(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_NE(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_ORE(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_IDLE(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_RXNE(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_TC(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_TXE(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}
# 2600 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsActiveFlag_LBD(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}
# 2613 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsActiveFlag_nCTS(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}
# 2626 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsActiveFlag_CTS(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_RTO(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (11U)))) == ((0x1UL << (11U)))) ? 1UL : 0UL);
}
# 2650 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsActiveFlag_EOB(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (12U)))) == ((0x1UL << (12U)))) ? 1UL : 0UL);
}
# 2663 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsActiveFlag_ABRE(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (14U)))) == ((0x1UL << (14U)))) ? 1UL : 0UL);
}
# 2676 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsActiveFlag_ABR(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (15U)))) == ((0x1UL << (15U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_BUSY(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (16U)))) == ((0x1UL << (16U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_CM(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (17U)))) == ((0x1UL << (17U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_SBK(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (18U)))) == ((0x1UL << (18U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_RWU(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (19U)))) == ((0x1UL << (19U)))) ? 1UL : 0UL);
}
# 2733 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsActiveFlag_WKUP(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (20U)))) == ((0x1UL << (20U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_TEACK(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (21U)))) == ((0x1UL << (21U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsActiveFlag_REACK(USART_TypeDef *USARTx)
{
  return ((((USARTx->ISR) & ((0x1UL << (22U)))) == ((0x1UL << (22U)))) ? 1UL : 0UL);
}







static inline void LL_USART_ClearFlag_PE(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (0U))));
}







static inline void LL_USART_ClearFlag_FE(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (1U))));
}







static inline void LL_USART_ClearFlag_NE(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (2U))));
}







static inline void LL_USART_ClearFlag_ORE(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (3U))));
}







static inline void LL_USART_ClearFlag_IDLE(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (4U))));
}







static inline void LL_USART_ClearFlag_TC(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (6U))));
}
# 2835 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ClearFlag_LBD(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (8U))));
}
# 2848 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ClearFlag_nCTS(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (9U))));
}







static inline void LL_USART_ClearFlag_RTO(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (11U))));
}
# 2872 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ClearFlag_EOB(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (12U))));
}







static inline void LL_USART_ClearFlag_CM(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (17U))));
}
# 2896 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_ClearFlag_WKUP(USART_TypeDef *USARTx)
{
  ((USARTx->ICR) = ((0x1UL << (20U))));
}
# 2915 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableIT_IDLE(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (4U))));
}







static inline void LL_USART_EnableIT_RXNE(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (5U))));
}







static inline void LL_USART_EnableIT_TC(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (6U))));
}







static inline void LL_USART_EnableIT_TXE(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (7U))));
}







static inline void LL_USART_EnableIT_PE(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (8U))));
}







static inline void LL_USART_EnableIT_CM(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (14U))));
}







static inline void LL_USART_EnableIT_RTO(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (26U))));
}
# 2994 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableIT_EOB(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) |= ((0x1UL << (27U))));
}
# 3007 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableIT_LBD(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) |= ((0x1UL << (6U))));
}
# 3022 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableIT_ERROR(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (0U))));
}
# 3035 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableIT_CTS(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (10U))));
}
# 3048 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableIT_WKUP(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (22U))));
}
# 3060 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableIT_IDLE(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (4U))));
}







static inline void LL_USART_DisableIT_RXNE(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (5U))));
}







static inline void LL_USART_DisableIT_TC(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (6U))));
}







static inline void LL_USART_DisableIT_TXE(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (7U))));
}







static inline void LL_USART_DisableIT_PE(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (8U))));
}







static inline void LL_USART_DisableIT_CM(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (14U))));
}







static inline void LL_USART_DisableIT_RTO(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (26U))));
}
# 3139 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableIT_EOB(USART_TypeDef *USARTx)
{
  ((USARTx->CR1) &= ~((0x1UL << (27U))));
}
# 3152 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableIT_LBD(USART_TypeDef *USARTx)
{
  ((USARTx->CR2) &= ~((0x1UL << (6U))));
}
# 3167 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableIT_ERROR(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (0U))));
}
# 3180 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableIT_CTS(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (10U))));
}
# 3193 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_DisableIT_WKUP(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (22U))));
}
# 3205 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledIT_IDLE(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsEnabledIT_RXNE(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1U : 0U);
}







static inline uint32_t LL_USART_IsEnabledIT_TC(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsEnabledIT_TXE(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1U : 0U);
}







static inline uint32_t LL_USART_IsEnabledIT_PE(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsEnabledIT_CM(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (14U)))) == ((0x1UL << (14U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsEnabledIT_RTO(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (26U)))) == ((0x1UL << (26U)))) ? 1UL : 0UL);
}
# 3284 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledIT_EOB(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR1) & ((0x1UL << (27U)))) == ((0x1UL << (27U)))) ? 1UL : 0UL);
}
# 3297 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledIT_LBD(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR2) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







static inline uint32_t LL_USART_IsEnabledIT_ERROR(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}
# 3321 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledIT_CTS(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}
# 3334 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_IsEnabledIT_WKUP(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (22U)))) == ((0x1UL << (22U)))) ? 1UL : 0UL);
}
# 3354 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_EnableDMAReq_RX(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (6U))));
}







static inline void LL_USART_DisableDMAReq_RX(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (6U))));
}







static inline uint32_t LL_USART_IsEnabledDMAReq_RX(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







static inline void LL_USART_EnableDMAReq_TX(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (7U))));
}







static inline void LL_USART_DisableDMAReq_TX(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (7U))));
}







static inline uint32_t LL_USART_IsEnabledDMAReq_TX(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}







static inline void LL_USART_EnableDMADeactOnRxErr(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) |= ((0x1UL << (13U))));
}







static inline void LL_USART_DisableDMADeactOnRxErr(USART_TypeDef *USARTx)
{
  ((USARTx->CR3) &= ~((0x1UL << (13U))));
}







static inline uint32_t LL_USART_IsEnabledDMADeactOnRxErr(USART_TypeDef *USARTx)
{
  return ((((USARTx->CR3) & ((0x1UL << (13U)))) == ((0x1UL << (13U)))) ? 1UL : 0UL);
}
# 3457 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint32_t LL_USART_DMA_GetRegAddr(USART_TypeDef *USARTx, uint32_t Direction)
{
  register uint32_t data_reg_addr;

  if (Direction == 0x00000000U)
  {

    data_reg_addr = (uint32_t) &(USARTx->TDR);
  }
  else
  {

    data_reg_addr = (uint32_t) &(USARTx->RDR);
  }

  return data_reg_addr;
}
# 3489 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline uint8_t LL_USART_ReceiveData8(USART_TypeDef *USARTx)
{
  return (uint8_t)(((USARTx->RDR) & ((0x1FFUL << (0U)))) & 0xFFU);
}







static inline uint16_t LL_USART_ReceiveData9(USART_TypeDef *USARTx)
{
  return (uint16_t)(((USARTx->RDR) & ((0x1FFUL << (0U)))));
}
# 3512 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_TransmitData8(USART_TypeDef *USARTx, uint8_t Value)
{
  USARTx->TDR = Value;
}
# 3524 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_TransmitData9(USART_TypeDef *USARTx, uint16_t Value)
{
  USARTx->TDR = (uint16_t)(Value & 0x1FFUL);
}
# 3545 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_RequestAutoBaudRate(USART_TypeDef *USARTx)
{
  ((USARTx->RQR) |= ((uint16_t)(0x1UL << (0U))));
}







static inline void LL_USART_RequestBreakSending(USART_TypeDef *USARTx)
{
  ((USARTx->RQR) |= ((uint16_t)(0x1UL << (1U))));
}







static inline void LL_USART_RequestEnterMuteMode(USART_TypeDef *USARTx)
{
  ((USARTx->RQR) |= ((uint16_t)(0x1UL << (2U))));
}
# 3580 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_RequestRxDataFlush(USART_TypeDef *USARTx)
{
  ((USARTx->RQR) |= ((uint16_t)(0x1UL << (3U))));
}
# 3593 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
static inline void LL_USART_RequestTxDataFlush(USART_TypeDef *USARTx)
{
  ((USARTx->RQR) |= ((uint16_t)(0x1UL << (4U))));
}
# 3606 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_ll_usart.h"
ErrorStatus LL_USART_DeInit(USART_TypeDef *USARTx);
ErrorStatus LL_USART_Init(USART_TypeDef *USARTx, LL_USART_InitTypeDef *USART_InitStruct);
void LL_USART_StructInit(LL_USART_InitTypeDef *USART_InitStruct);
ErrorStatus LL_USART_ClockInit(USART_TypeDef *USARTx, LL_USART_ClockInitTypeDef *USART_ClockInitStruct);
void LL_USART_ClockStructInit(LL_USART_ClockInitTypeDef *USART_ClockInitStruct);
# 61 "./boards/stm32f3xx_hal_conf_base.h" 2
# 9 "boards/STM32F3DISC/stm32f3xx_hal_conf.h" 2
# 31 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h" 2
# 61 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"
typedef enum
{
  HAL_TICK_FREQ_10HZ = 100U,
  HAL_TICK_FREQ_100HZ = 10U,
  HAL_TICK_FREQ_1KHZ = 1U,
  HAL_TICK_FREQ_DEFAULT = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;
# 878 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);
# 891 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"
extern volatile uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;
# 903 "../../lib/stm32lib/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);
# 31 "./mpconfigboard_common.h" 2
# 33 "./mpconfigport.h" 2
# 167 "./mpconfigport.h"
extern const struct _mp_obj_module_t pyb_module;







extern const struct _mp_obj_module_t stm_module;
# 251 "./mpconfigport.h"
typedef int mp_int_t;
typedef unsigned int mp_uint_t;


typedef long mp_off_t;
# 265 "./mpconfigport.h"
static inline void enable_irq(mp_uint_t state) {
    __set_PRIMASK(state);
}

static inline mp_uint_t disable_irq(void) {
    mp_uint_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}
# 336 "./mpconfigport.h"
# 1 "/usr/include/newlib/alloca.h" 1 3
# 10 "/usr/include/newlib/alloca.h" 3
# 1 "/usr/include/newlib/_ansi.h" 1 3
# 10 "/usr/include/newlib/_ansi.h" 3
# 1 "/usr/include/newlib/newlib.h" 1 3
# 14 "/usr/include/newlib/newlib.h" 3
# 1 "/usr/include/newlib/_newlib_version.h" 1 3
# 15 "/usr/include/newlib/newlib.h" 2 3
# 11 "/usr/include/newlib/_ansi.h" 2 3
# 1 "/usr/include/newlib/sys/config.h" 1 3



# 1 "/usr/include/newlib/machine/ieeefp.h" 1 3
# 5 "/usr/include/newlib/sys/config.h" 2 3
# 1 "/usr/include/newlib/sys/features.h" 1 3
# 6 "/usr/include/newlib/sys/config.h" 2 3
# 12 "/usr/include/newlib/_ansi.h" 2 3
# 11 "/usr/include/newlib/alloca.h" 2 3
# 1 "/usr/include/newlib/sys/reent.h" 1 3
# 13 "/usr/include/newlib/sys/reent.h" 3
# 1 "/usr/include/newlib/_ansi.h" 1 3
# 14 "/usr/include/newlib/sys/reent.h" 2 3
# 1 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h" 1 3 4
# 15 "/usr/include/newlib/sys/reent.h" 2 3
# 1 "/usr/include/newlib/sys/_types.h" 1 3
# 24 "/usr/include/newlib/sys/_types.h" 3
# 1 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h" 1 3 4
# 350 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h" 3 4

# 350 "/usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h" 3 4
typedef unsigned int wint_t;
# 25 "/usr/include/newlib/sys/_types.h" 2 3


# 1 "/usr/include/newlib/machine/_types.h" 1 3






# 1 "/usr/include/newlib/machine/_default_types.h" 1 3
# 41 "/usr/include/newlib/machine/_default_types.h" 3
typedef signed char __int8_t;

typedef unsigned char __uint8_t;
# 55 "/usr/include/newlib/machine/_default_types.h" 3
typedef short int __int16_t;

typedef short unsigned int __uint16_t;
# 77 "/usr/include/newlib/machine/_default_types.h" 3
typedef long int __int32_t;

typedef long unsigned int __uint32_t;
# 103 "/usr/include/newlib/machine/_default_types.h" 3
typedef long long int __int64_t;

typedef long long unsigned int __uint64_t;
# 134 "/usr/include/newlib/machine/_default_types.h" 3
typedef signed char __int_least8_t;

typedef unsigned char __uint_least8_t;
# 160 "/usr/include/newlib/machine/_default_types.h" 3
typedef short int __int_least16_t;

typedef short unsigned int __uint_least16_t;
# 182 "/usr/include/newlib/machine/_default_types.h" 3
typedef long int __int_least32_t;

typedef long unsigned int __uint_least32_t;
# 200 "/usr/include/newlib/machine/_default_types.h" 3
typedef long long int __int_least64_t;

typedef long long unsigned int __uint_least64_t;
# 214 "/usr/include/newlib/machine/_default_types.h" 3
typedef long long int __intmax_t;







typedef long long unsigned int __uintmax_t;







typedef int __intptr_t;

typedef unsigned int __uintptr_t;
# 8 "/usr/include/newlib/machine/_types.h" 2 3
# 28 "/usr/include/newlib/sys/_types.h" 2 3


typedef long __blkcnt_t;



typedef long __blksize_t;



typedef __uint64_t __fsblkcnt_t;



typedef __uint32_t __fsfilcnt_t;



typedef long _off_t;





typedef int __pid_t;



typedef short __dev_t;



typedef unsigned short __uid_t;


typedef unsigned short __gid_t;



typedef __uint32_t __id_t;







typedef unsigned short __ino_t;
# 90 "/usr/include/newlib/sys/_types.h" 3
typedef __uint32_t __mode_t;





__extension__ typedef long long _off64_t;





typedef _off_t __off_t;


typedef _off64_t __loff_t;


typedef long __key_t;







typedef long _fpos_t;
# 131 "/usr/include/newlib/sys/_types.h" 3
typedef unsigned int __size_t;
# 147 "/usr/include/newlib/sys/_types.h" 3
typedef signed int _ssize_t;
# 158 "/usr/include/newlib/sys/_types.h" 3
typedef _ssize_t __ssize_t;



typedef struct
{
  int __count;
  union
  {
    wint_t __wch;
    unsigned char __wchb[4];
  } __value;
} _mbstate_t;




typedef void *_iconv_t;






typedef unsigned long __clock_t;






typedef __int_least64_t __time_t;





typedef unsigned long __clockid_t;


typedef unsigned long __timer_t;


typedef __uint8_t __sa_family_t;



typedef __uint32_t __socklen_t;


typedef int __nl_item;
typedef unsigned short __nlink_t;
typedef long __suseconds_t;
typedef unsigned long __useconds_t;







typedef __builtin_va_list __va_list;
# 16 "/usr/include/newlib/sys/reent.h" 2 3






typedef unsigned long __ULong;
# 34 "/usr/include/newlib/sys/reent.h" 3
# 1 "/usr/include/newlib/sys/lock.h" 1 3
# 11 "/usr/include/newlib/sys/lock.h" 3
typedef int _LOCK_T;
typedef int _LOCK_RECURSIVE_T;
# 35 "/usr/include/newlib/sys/reent.h" 2 3
typedef _LOCK_RECURSIVE_T _flock_t;







struct _reent;

struct __locale_t;






struct _Bigint
{
  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm
{
  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _on_exit_args {
 void * _fnargs[32];
 void * _dso_handle[32];

 __ULong _fntypes;


 __ULong _is_cxa;
};
# 98 "/usr/include/newlib/sys/reent.h" 3
struct _atexit {
 struct _atexit *_next;
 int _ind;

 void (*_fns[32])(void);
        struct _on_exit_args _on_exit_args;
};
# 122 "/usr/include/newlib/sys/reent.h" 3
struct __sbuf {
 unsigned char *_base;
 int _size;
};
# 186 "/usr/include/newlib/sys/reent.h" 3
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;






  void * _cookie;

  int (*_read) (struct _reent *, void *,
        char *, int);
  int (*_write) (struct _reent *, void *,
         const char *,
         int);
  _fpos_t (*_seek) (struct _reent *, void *, _fpos_t, int);
  int (*_close) (struct _reent *, void *);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  _off_t _offset;


  struct _reent *_data;



  _flock_t _lock;

  _mbstate_t _mbstate;
  int _flags2;
};
# 292 "/usr/include/newlib/sys/reent.h" 3
typedef struct __sFILE __FILE;



struct _glue
{
  struct _glue *_next;
  int _niobs;
  __FILE *_iobs;
};
# 324 "/usr/include/newlib/sys/reent.h" 3
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;




};
# 613 "/usr/include/newlib/sys/reent.h" 3
struct _reent
{
  int _errno;




  __FILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];


  int _unspecified_locale_info;
  struct __locale_t *_locale;

  int __sdidinit;

  void (*__cleanup) (struct _reent *);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union
    {
      struct
        {
          unsigned int _unused_rand;
          char * _strtok_last;
          char _asctime_buf[26];
          struct __tm _localtime_buf;
          int _gamma_signgam;
          __extension__ unsigned long long _rand_next;
          struct _rand48 _r48;
          _mbstate_t _mblen_state;
          _mbstate_t _mbtowc_state;
          _mbstate_t _wctomb_state;
          char _l64a_buf[8];
          char _signal_buf[24];
          int _getdate_err;
          _mbstate_t _mbrlen_state;
          _mbstate_t _mbrtowc_state;
          _mbstate_t _mbsrtowcs_state;
          _mbstate_t _wcrtomb_state;
          _mbstate_t _wcsrtombs_state;
   int _h_errno;
        } _reent;



      struct
        {

          unsigned char * _nextf[30];
          unsigned int _nmalloc[30];
        } _unused;
    } _new;



  struct _atexit *_atexit;
  struct _atexit _atexit0;



  void (**(_sig_func))(int);




  struct _glue __sglue;

  __FILE __sf[3];

};
# 819 "/usr/include/newlib/sys/reent.h" 3
extern struct _reent *_impure_ptr ;
extern struct _reent *const _global_impure_ptr ;

void _reclaim_reent (struct _reent *);
# 12 "/usr/include/newlib/alloca.h" 2 3
# 337 "./mpconfigport.h" 2



# 339 "./mpconfigport.h"
uint32_t rng_get(void);
# 63 "../../py/mpconfig.h" 2
# 783 "../../py/mpconfig.h"
typedef float mp_float_t;
# 30 "<stdin>" 2





QCFG(BYTES_IN_LEN, (1))
QCFG(BYTES_IN_HASH, (2))

Q()
Q(*)
Q(_)
Q(/)

Q(>>> )
Q(... )


Q(%#o)
Q(%#x)




Q({:#b})
Q( )
Q(\n)
Q(maximum recursion depth exceeded)
Q(<module>)
Q(<lambda>)
Q(<listcomp>)
Q(<dictcomp>)
Q(<setcomp>)
Q(<genexpr>)
Q(<string>)
Q(<stdin>)
Q(utf-8)


Q(.frozen)
# 104 "<stdin>"
Q(/flash)
Q(/flash/lib)
Q(/sd)
Q(/sd/lib)


Q(/)



Q(VCP)
Q(MSC)
Q(VCP+MSC)
Q(VCP+HID)
Q(VCP+MSC+HID)
# 130 "<stdin>"
Q(A0)
Q(A1)
Q(A10)
Q(A11)
Q(A12)
Q(A13)
Q(A14)
Q(A15)
Q(A2)
Q(A3)
Q(A4)
Q(A5)
Q(A6)
Q(A7)
Q(A8)
Q(A9)
Q(AF10_TIM17)
Q(AF10_TIM2)
Q(AF10_TIM3)
Q(AF10_TIM4)
Q(AF10_TIM8)
Q(AF11_TIM1)
Q(AF11_TIM8)
Q(AF12_TIM1)
Q(AF1_TIM15)
Q(AF1_TIM16)
Q(AF1_TIM17)
Q(AF1_TIM2)
Q(AF2_TIM1)
Q(AF2_TIM15)
Q(AF2_TIM2)
Q(AF2_TIM3)
Q(AF2_TIM4)
Q(AF2_TIM8)
Q(AF3_TIM8)

Q(AF4_I2C1)


Q(AF4_I2C2)

Q(AF4_TIM1)
Q(AF4_TIM16)
Q(AF4_TIM17)
Q(AF4_TIM8)







Q(AF5_SPI1)


Q(AF5_SPI2)

Q(AF5_TIM8)
# 201 "<stdin>"
Q(AF6_SPI2)




Q(AF6_TIM)
Q(AF6_TIM1)
Q(AF6_TIM8)

Q(AF7_CAN1)





Q(AF7_USART2)


Q(AF7_USART3)


Q(AF9_CAN1)

Q(AF9_TIM1)
Q(AF9_TIM15)
Q(AF9_TIM8)
Q(B0)
Q(B1)
Q(B10)
Q(B11)
Q(B12)
Q(B13)
Q(B14)
Q(B15)
Q(B2)
Q(B4)
Q(B5)
Q(B6)
Q(B7)
Q(B8)
Q(B9)
Q(C0)
Q(C1)
Q(C10)
Q(C11)
Q(C12)
Q(C13)
Q(C14)
Q(C15)
Q(C2)
Q(C3)
Q(C4)
Q(C5)
Q(C6)
Q(C7)
Q(C8)
Q(C9)
Q(D0)
Q(D1)
Q(D10)
Q(D11)
Q(D12)
Q(D13)
Q(D14)
Q(D15)
Q(D2)
Q(D3)
Q(D4)
Q(D5)
Q(D6)
Q(D7)
Q(D8)
Q(D9)
Q(E0)
Q(E1)
Q(E10)
Q(E11)
Q(E12)
Q(E13)
Q(E14)
Q(E15)
Q(E2)
Q(E3)
Q(E4)
Q(E5)
Q(E6)
Q(E7)
Q(E8)
Q(E9)
Q(LED_BLUE)
Q(LED_GREEN)
Q(LED_ORANGE)
Q(LED_RED)
Q(PA0)
Q(PA1)
Q(PA10)
Q(PA13)
Q(PA14)
Q(PA15)
Q(PA2)
Q(PA3)
Q(PA4)
Q(PA5)
Q(PA6)
Q(PA7)
Q(PA8)
Q(PA9)
Q(PB0)
Q(PB1)
Q(PB10)
Q(PB11)
Q(PB12)
Q(PB13)
Q(PB14)
Q(PB15)
Q(PB2)
Q(PB4)
Q(PB5)
Q(PB6)
Q(PB7)
Q(PB8)
Q(PB9)
Q(PC0)
Q(PC1)
Q(PC10)
Q(PC11)
Q(PC12)
Q(PC13)
Q(PC14)
Q(PC15)
Q(PC2)
Q(PC3)
Q(PC4)
Q(PC5)
Q(PC6)
Q(PC7)
Q(PC8)
Q(PC9)
Q(PD0)
Q(PD1)
Q(PD10)
Q(PD11)
Q(PD12)
Q(PD13)
Q(PD14)
Q(PD15)
Q(PD2)
Q(PD3)
Q(PD4)
Q(PD5)
Q(PD6)
Q(PD7)
Q(PD8)
Q(PD9)
Q(PE0)
Q(PE1)
Q(PE10)
Q(PE11)
Q(PE12)
Q(PE13)
Q(PE14)
Q(PE15)
Q(PE2)
Q(PE3)
Q(PE4)
Q(PE5)
Q(PE6)
Q(PE7)
Q(PE8)
Q(PE9)
Q(SW)
Q(USB_DM)
Q(USB_DP)

Q(ADC1)
Q(ADC12_COMMON)
Q(ADC2)
Q(ADC3)
Q(ADC34_COMMON)
Q(ADC4)
Q(ADC_AWD2CR)
Q(ADC_AWD3CR)
Q(ADC_CALFACT)
Q(ADC_CFGR)
Q(ADC_CR)
Q(ADC_DIFSEL)
Q(ADC_DR)
Q(ADC_IER)
Q(ADC_ISR)
Q(ADC_JDR1)
Q(ADC_JDR2)
Q(ADC_JDR3)
Q(ADC_JDR4)
Q(ADC_JSQR)
Q(ADC_OFR1)
Q(ADC_OFR2)
Q(ADC_OFR3)
Q(ADC_OFR4)
Q(ADC_SMPR1)
Q(ADC_SMPR2)
Q(ADC_SQR1)
Q(ADC_SQR2)
Q(ADC_SQR3)
Q(ADC_SQR4)
Q(ADC_TR1)
Q(ADC_TR2)
Q(ADC_TR3)
Q(CAN)
Q(COMP)
Q(COMP1)
Q(COMP12_COMMON)
Q(COMP2)
Q(COMP3)
Q(COMP34_COMMON)
Q(COMP4)
Q(COMP5)
Q(COMP56_COMMON)
Q(COMP6)
Q(COMP7)
Q(CRC)
Q(CRC_CR)
Q(CRC_DR)
Q(CRC_IDR)
Q(CRC_INIT)
Q(CRC_POL)
Q(DAC)
Q(DAC1)
Q(DAC_CR)
Q(DAC_DHR12L1)
Q(DAC_DHR12L2)
Q(DAC_DHR12LD)
Q(DAC_DHR12R1)
Q(DAC_DHR12R2)
Q(DAC_DHR12RD)
Q(DAC_DHR8R1)
Q(DAC_DHR8R2)
Q(DAC_DHR8RD)
Q(DAC_DOR1)
Q(DAC_DOR2)
Q(DAC_SR)
Q(DAC_SWTRIGR)
Q(DBGMCU)
Q(DBGMCU_APB1FZ)
Q(DBGMCU_APB2FZ)
Q(DBGMCU_CR)
Q(DBGMCU_IDCODE)
Q(DMA1)
Q(DMA2)
Q(DMA_IFCR)
Q(DMA_ISR)
Q(EXTI)
Q(EXTI_EMR)
Q(EXTI_EMR2)
Q(EXTI_FTSR)
Q(EXTI_FTSR2)
Q(EXTI_IMR)
Q(EXTI_IMR2)
Q(EXTI_PR)
Q(EXTI_PR2)
Q(EXTI_RTSR)
Q(EXTI_RTSR2)
Q(EXTI_SWIER)
Q(EXTI_SWIER2)
Q(FLASH)
Q(FLASH_ACR)
Q(FLASH_AR)
Q(FLASH_CR)
Q(FLASH_KEYR)
Q(FLASH_OBR)
Q(FLASH_OPTKEYR)
Q(FLASH_SR)
Q(FLASH_WRPR)
Q(GPIOA)
Q(GPIOB)
Q(GPIOC)
Q(GPIOD)
Q(GPIOE)
Q(GPIOF)
Q(GPIO_AFR0)
Q(GPIO_AFR1)
Q(GPIO_BRR)
Q(GPIO_BSRR)
Q(GPIO_IDR)
Q(GPIO_LCKR)
Q(GPIO_MODER)
Q(GPIO_ODR)
Q(GPIO_OSPEEDR)
Q(GPIO_OTYPER)
Q(GPIO_PUPDR)
Q(I2C1)
Q(I2C2)
Q(I2C_CR1)
Q(I2C_CR2)
Q(I2C_ICR)
Q(I2C_ISR)
Q(I2C_OAR1)
Q(I2C_OAR2)
Q(I2C_PECR)
Q(I2C_RXDR)
Q(I2C_TIMEOUTR)
Q(I2C_TIMINGR)
Q(I2C_TXDR)
Q(I2S2EXT)
Q(I2S3EXT)
Q(IWDG)
Q(IWDG_KR)
Q(IWDG_PR)
Q(IWDG_RLR)
Q(IWDG_SR)
Q(IWDG_WINR)
Q(OB)
Q(OPAMP)
Q(OPAMP1)
Q(OPAMP2)
Q(OPAMP3)
Q(OPAMP4)
Q(PWR)
Q(PWR_CR)
Q(PWR_CSR)
Q(RCC)
Q(RCC_AHBENR)
Q(RCC_AHBRSTR)
Q(RCC_APB1ENR)
Q(RCC_APB1RSTR)
Q(RCC_APB2ENR)
Q(RCC_APB2RSTR)
Q(RCC_BDCR)
Q(RCC_CFGR)
Q(RCC_CFGR2)
Q(RCC_CFGR3)
Q(RCC_CIR)
Q(RCC_CR)
Q(RCC_CSR)
Q(RTC)
Q(RTC_ALRMAR)
Q(RTC_ALRMASSR)
Q(RTC_ALRMBR)
Q(RTC_ALRMBSSR)
Q(RTC_BKP0R)
Q(RTC_BKP10R)
Q(RTC_BKP11R)
Q(RTC_BKP12R)
Q(RTC_BKP13R)
Q(RTC_BKP14R)
Q(RTC_BKP15R)
Q(RTC_BKP1R)
Q(RTC_BKP2R)
Q(RTC_BKP3R)
Q(RTC_BKP4R)
Q(RTC_BKP5R)
Q(RTC_BKP6R)
Q(RTC_BKP7R)
Q(RTC_BKP8R)
Q(RTC_BKP9R)
Q(RTC_CALR)
Q(RTC_CR)
Q(RTC_DR)
Q(RTC_ISR)
Q(RTC_PRER)
Q(RTC_SHIFTR)
Q(RTC_SSR)
Q(RTC_TAFCR)
Q(RTC_TR)
Q(RTC_TSDR)
Q(RTC_TSSSR)
Q(RTC_TSTR)
Q(RTC_WPR)
Q(RTC_WUTR)
Q(SPI1)
Q(SPI2)
Q(SPI3)
Q(SPI_CR1)
Q(SPI_CR2)
Q(SPI_CRCPR)
Q(SPI_DR)
Q(SPI_I2SCFGR)
Q(SPI_I2SPR)
Q(SPI_RXCRCR)
Q(SPI_SR)
Q(SPI_TXCRCR)
Q(SYSCFG)
Q(SYSCFG_CFGR1)
Q(SYSCFG_CFGR2)
Q(SYSCFG_EXTICR0)
Q(SYSCFG_EXTICR1)
Q(SYSCFG_EXTICR2)
Q(SYSCFG_EXTICR3)
Q(SYSCFG_RCR)
Q(TIM1)
Q(TIM15)
Q(TIM16)
Q(TIM17)
Q(TIM2)
Q(TIM3)
Q(TIM4)
Q(TIM6)
Q(TIM7)
Q(TIM8)
Q(TIM_ARR)
Q(TIM_BDTR)
Q(TIM_CCER)
Q(TIM_CCMR1)
Q(TIM_CCMR2)
Q(TIM_CCMR3)
Q(TIM_CCR1)
Q(TIM_CCR2)
Q(TIM_CCR3)
Q(TIM_CCR4)
Q(TIM_CCR5)
Q(TIM_CCR6)
Q(TIM_CNT)
Q(TIM_CR1)
Q(TIM_CR2)
Q(TIM_DCR)
Q(TIM_DIER)
Q(TIM_DMAR)
Q(TIM_EGR)
Q(TIM_OR)
Q(TIM_PSC)
Q(TIM_RCR)
Q(TIM_SMCR)
Q(TIM_SR)
Q(TSC)
Q(UART4)
Q(UART5)
Q(USART1)
Q(USART2)
Q(USART3)
Q(USART_BRR)
Q(USART_CR1)
Q(USART_CR2)
Q(USART_CR3)
Q(USART_GTPR)
Q(USART_ICR)
Q(USART_ISR)
Q(USART_RDR)
Q(USART_RQR)
Q(USART_RTOR)
Q(USART_TDR)
Q(USB)
Q(WWDG)
Q(WWDG_CFR)
Q(WWDG_CR)
Q(WWDG_SR)

Q(ADC)

Q(ADC)

Q(ADC)

Q(ADC)

Q(ADC1)

Q(ADC12_COMMON)

Q(ADC2)

Q(ADC3)

Q(ADC34_COMMON)

Q(ADC4)

Q(ADCAll)

Q(ADCAll)

Q(ADC_AWD2CR)

Q(ADC_AWD3CR)

Q(ADC_CALFACT)

Q(ADC_CFGR)

Q(ADC_CR)

Q(ADC_DIFSEL)

Q(ADC_DR)

Q(ADC_IER)

Q(ADC_ISR)

Q(ADC_JDR1)

Q(ADC_JDR2)

Q(ADC_JDR3)

Q(ADC_JDR4)

Q(ADC_JSQR)

Q(ADC_OFR1)

Q(ADC_OFR2)

Q(ADC_OFR3)

Q(ADC_OFR4)

Q(ADC_SMPR1)

Q(ADC_SMPR2)

Q(ADC_SQR1)

Q(ADC_SQR2)

Q(ADC_SQR3)

Q(ADC_SQR4)

Q(ADC_TR1)

Q(ADC_TR2)

Q(ADC_TR3)

Q(AF10_TIM17)

Q(AF10_TIM2)

Q(AF10_TIM3)

Q(AF10_TIM4)

Q(AF10_TIM8)

Q(AF11_TIM1)

Q(AF11_TIM8)

Q(AF12_TIM1)

Q(AF1_TIM15)

Q(AF1_TIM16)

Q(AF1_TIM17)

Q(AF1_TIM2)

Q(AF2_TIM1)

Q(AF2_TIM15)

Q(AF2_TIM2)

Q(AF2_TIM3)

Q(AF2_TIM4)

Q(AF2_TIM8)

Q(AF3_TIM8)

Q(AF4_I2C1)

Q(AF4_I2C2)

Q(AF4_TIM1)

Q(AF4_TIM16)

Q(AF4_TIM17)

Q(AF4_TIM8)

Q(AF5_SPI1)

Q(AF5_SPI2)

Q(AF5_TIM8)

Q(AF6_SPI2)

Q(AF6_TIM)

Q(AF6_TIM1)

Q(AF6_TIM8)

Q(AF7_CAN1)

Q(AF7_USART2)

Q(AF7_USART3)

Q(AF9_CAN1)

Q(AF9_TIM1)

Q(AF9_TIM15)

Q(AF9_TIM8)

Q(AF_INET)

Q(AF_INET6)

Q(AF_OD)

Q(AF_PP)

Q(ALT)

Q(ALT)

Q(ALT_OPEN_DRAIN)

Q(ALT_OPEN_DRAIN)

Q(ANALOG)

Q(AP_IF)

Q(ARRAY)

Q(ArithmeticError)

Q(ArithmeticError)

Q(AssertionError)

Q(AssertionError)

Q(AssertionError)

Q(AttributeError)

Q(AttributeError)

Q(BFINT16)

Q(BFINT32)

Q(BFINT8)

Q(BFUINT16)

Q(BFUINT32)

Q(BFUINT8)

Q(BF_LEN)

Q(BF_POS)

Q(BIG_ENDIAN)

Q(BOTH)

Q(BRK_HIGH)

Q(BRK_LOW)

Q(BRK_OFF)

Q(BUS_OFF)

Q(BaseException)

Q(BaseException)

Q(BaseException)

Q(BytesIO)

Q(BytesIO)

Q(CAN)

Q(CAN)

Q(CAN)

Q(CENTER)

Q(CIRCULAR)

Q(COMP)

Q(COMP1)

Q(COMP12_COMMON)

Q(COMP2)

Q(COMP3)

Q(COMP34_COMMON)

Q(COMP4)

Q(COMP5)

Q(COMP56_COMMON)

Q(COMP6)

Q(COMP7)

Q(CONTROLLER)

Q(CONTROLLER)

Q(CORE_TEMP)

Q(CORE_VBAT)

Q(CORE_VREF)

Q(CRC)

Q(CRC_CR)

Q(CRC_DR)

Q(CRC_IDR)

Q(CRC_INIT)

Q(CRC_POL)

Q(CTS)

Q(CTS)

Q(CancelledError)

Q(DAC)

Q(DAC)

Q(DAC)

Q(DAC1)

Q(DAC_CR)

Q(DAC_DHR12L1)

Q(DAC_DHR12L2)

Q(DAC_DHR12LD)

Q(DAC_DHR12R1)

Q(DAC_DHR12R2)

Q(DAC_DHR12RD)

Q(DAC_DHR8R1)

Q(DAC_DHR8R2)

Q(DAC_DHR8RD)

Q(DAC_DOR1)

Q(DAC_DOR2)

Q(DAC_SR)

Q(DAC_SWTRIGR)

Q(DBGMCU)

Q(DBGMCU_APB1FZ)

Q(DBGMCU_APB2FZ)

Q(DBGMCU_CR)

Q(DBGMCU_IDCODE)

Q(DEEPSLEEP_RESET)

Q(DMA1)

Q(DMA2)

Q(DMA_IFCR)

Q(DMA_ISR)

Q(DOWN)

Q(DecompIO)

Q(DecompIO)

Q(EACCES)

Q(EACCES)

Q(EADDRINUSE)

Q(EADDRINUSE)

Q(EAGAIN)

Q(EAGAIN)

Q(EALREADY)

Q(EALREADY)

Q(EBADF)

Q(EBADF)

Q(ECONNABORTED)

Q(ECONNABORTED)

Q(ECONNREFUSED)

Q(ECONNREFUSED)

Q(ECONNRESET)

Q(ECONNRESET)

Q(EEXIST)

Q(EEXIST)

Q(EHOSTUNREACH)

Q(EHOSTUNREACH)

Q(EINPROGRESS)

Q(EINPROGRESS)

Q(EINVAL)

Q(EINVAL)

Q(EIO)

Q(EIO)

Q(EISDIR)

Q(EISDIR)

Q(ENC_A)

Q(ENC_A)

Q(ENC_AB)

Q(ENC_AB)

Q(ENC_B)

Q(ENC_B)

Q(ENOBUFS)

Q(ENOBUFS)

Q(ENODEV)

Q(ENODEV)

Q(ENOENT)

Q(ENOENT)

Q(ENOMEM)

Q(ENOMEM)

Q(ENOTCONN)

Q(ENOTCONN)

Q(EOFError)

Q(EOFError)

Q(EOPNOTSUPP)

Q(EOPNOTSUPP)

Q(EPERM)

Q(EPERM)

Q(ERROR_ACTIVE)

Q(ERROR_PASSIVE)

Q(ERROR_WARNING)

Q(ETIMEDOUT)

Q(ETIMEDOUT)

Q(EVT_FALLING)

Q(EVT_RISING)

Q(EVT_RISING_FALLING)

Q(EXTI)

Q(EXTI_EMR)

Q(EXTI_EMR2)

Q(EXTI_FTSR)

Q(EXTI_FTSR2)

Q(EXTI_IMR)

Q(EXTI_IMR2)

Q(EXTI_PR)

Q(EXTI_PR2)

Q(EXTI_RTSR)

Q(EXTI_RTSR2)

Q(EXTI_SWIER)

Q(EXTI_SWIER2)

Q(Ellipsis)

Q(Ellipsis)

Q(Exception)

Q(Exception)

Q(ExtInt)

Q(ExtInt)

Q(FALLING)

Q(FLASH)

Q(FLASH_ACR)

Q(FLASH_AR)

Q(FLASH_CR)

Q(FLASH_KEYR)

Q(FLASH_OBR)

Q(FLASH_OPTKEYR)

Q(FLASH_SR)

Q(FLASH_WRPR)

Q(FLOAT32)

Q(FLOAT64)

Q(False)

Q(FileIO)

Q(FileIO)

Q(FileIO)

Q(FileIO)

Q(Flash)

Q(Flash)

Q(FrameBuffer)

Q(FrameBuffer)

Q(FrameBuffer1)

Q(GPIOA)

Q(GPIOB)

Q(GPIOC)

Q(GPIOD)

Q(GPIOE)

Q(GPIOF)

Q(GPIO_AFR0)

Q(GPIO_AFR1)

Q(GPIO_BRR)

Q(GPIO_BSRR)

Q(GPIO_IDR)

Q(GPIO_LCKR)

Q(GPIO_MODER)

Q(GPIO_ODR)

Q(GPIO_OSPEEDR)

Q(GPIO_OTYPER)

Q(GPIO_PUPDR)

Q(GS2_HMSB)

Q(GS4_HMSB)

Q(GS8)

Q(GeneratorExit)

Q(GeneratorExit)

Q(HARD_RESET)

Q(HIGH)

Q(I2C)

Q(I2C)

Q(I2C)

Q(I2C)

Q(I2C1)

Q(I2C2)

Q(I2C_CR1)

Q(I2C_CR2)

Q(I2C_ICR)

Q(I2C_ISR)

Q(I2C_OAR1)

Q(I2C_OAR2)

Q(I2C_PECR)

Q(I2C_RXDR)

Q(I2C_TIMEOUTR)

Q(I2C_TIMINGR)

Q(I2C_TXDR)

Q(I2S2EXT)

Q(I2S3EXT)

Q(IC)

Q(IC)

Q(IN)

Q(IN)

Q(INT)

Q(INT16)

Q(INT32)

Q(INT64)

Q(INT8)

Q(IOBase)

Q(IOBase)

Q(IRQ_FALLING)

Q(IRQ_FALLING)

Q(IRQ_RISING)

Q(IRQ_RISING)

Q(IRQ_RISING_FALLING)

Q(IRQ_RX)

Q(IRQ_RXIDLE)

Q(IWDG)

Q(IWDG_KR)

Q(IWDG_PR)

Q(IWDG_RLR)

Q(IWDG_SR)

Q(IWDG_WINR)

Q(ImportError)

Q(ImportError)

Q(IndentationError)

Q(IndentationError)

Q(IndexError)

Q(IndexError)

Q(KeyError)

Q(KeyError)

Q(KeyboardInterrupt)

Q(KeyboardInterrupt)

Q(LIST16)

Q(LIST32)

Q(LITTLE_ENDIAN)

Q(LONG)

Q(LONGLONG)

Q(LOOPBACK)

Q(LOOPBACK)

Q(LOW)

Q(LSB)

Q(LSB)

Q(LookupError)

Q(LookupError)

Q(MASK16)

Q(MASK32)

Q(MASTER)

Q(MASTER)

Q(MONO_HLSB)

Q(MONO_HMSB)

Q(MONO_VLSB)

Q(MSB)

Q(MSB)

Q(MSC)

Q(MVLSB)

Q(MemoryError)

Q(MemoryError)

Q(NATIVE)

Q(NORMAL)

Q(NORMAL)

Q(NORMAL)

Q(NameError)

Q(NameError)

Q(None)

Q(NoneType)

Q(NotImplemented)

Q(NotImplemented)

Q(NotImplementedError)

Q(NotImplementedError)

Q(OB)

Q(OC_ACTIVE)

Q(OC_ACTIVE)

Q(OC_FORCED_ACTIVE)

Q(OC_FORCED_ACTIVE)

Q(OC_FORCED_INACTIVE)

Q(OC_FORCED_INACTIVE)

Q(OC_INACTIVE)

Q(OC_INACTIVE)

Q(OC_TIMING)

Q(OC_TIMING)

Q(OC_TOGGLE)

Q(OC_TOGGLE)

Q(ONE_SHOT)

Q(ONE_SHOT)

Q(OPAMP)

Q(OPAMP1)

Q(OPAMP2)

Q(OPAMP3)

Q(OPAMP4)

Q(OPEN_DRAIN)

Q(OPEN_DRAIN)

Q(OSError)

Q(OSError)

Q(OUT)

Q(OUT)

Q(OUT_OD)

Q(OUT_PP)

Q(OrderedDict)

Q(OrderedDict)

Q(OrderedDict)

Q(OverflowError)

Q(OverflowError)

Q(PERIODIC)

Q(PERIODIC)

Q(PERIPHERAL)

Q(PERIPHERAL)

Q(POLLERR)

Q(POLLHUP)

Q(POLLIN)

Q(POLLOUT)

Q(PTR)

Q(PULL_DOWN)

Q(PULL_DOWN)

Q(PULL_NONE)

Q(PULL_UP)

Q(PULL_UP)

Q(PWM)

Q(PWM)

Q(PWM_INVERTED)

Q(PWM_INVERTED)

Q(PWR)

Q(PWRON_RESET)

Q(PWR_CR)

Q(PWR_CSR)

Q(Pin)

Q(Pin)

Q(Pin)

Q(PinAF)

Q(PinBase)

Q(RCC)

Q(RCC_AHBENR)

Q(RCC_AHBRSTR)

Q(RCC_APB1ENR)

Q(RCC_APB1RSTR)

Q(RCC_APB2ENR)

Q(RCC_APB2RSTR)

Q(RCC_BDCR)

Q(RCC_CFGR)

Q(RCC_CFGR2)

Q(RCC_CFGR3)

Q(RCC_CIR)

Q(RCC_CR)

Q(RCC_CSR)

Q(RGB565)

Q(RISING)

Q(RTC)

Q(RTC)

Q(RTC)

Q(RTC)

Q(RTC_ALRMAR)

Q(RTC_ALRMASSR)

Q(RTC_ALRMBR)

Q(RTC_ALRMBSSR)

Q(RTC_BKP0R)

Q(RTC_BKP10R)

Q(RTC_BKP11R)

Q(RTC_BKP12R)

Q(RTC_BKP13R)

Q(RTC_BKP14R)

Q(RTC_BKP15R)

Q(RTC_BKP1R)

Q(RTC_BKP2R)

Q(RTC_BKP3R)

Q(RTC_BKP4R)

Q(RTC_BKP5R)

Q(RTC_BKP6R)

Q(RTC_BKP7R)

Q(RTC_BKP8R)

Q(RTC_BKP9R)

Q(RTC_CALR)

Q(RTC_CR)

Q(RTC_DR)

Q(RTC_ISR)

Q(RTC_PRER)

Q(RTC_SHIFTR)

Q(RTC_SSR)

Q(RTC_TAFCR)

Q(RTC_TR)

Q(RTC_TSDR)

Q(RTC_TSSSR)

Q(RTC_TSTR)

Q(RTC_WPR)

Q(RTC_WUTR)

Q(RTS)

Q(RTS)

Q(RuntimeError)

Q(RuntimeError)

Q(SHORT)

Q(SILENT)

Q(SILENT)

Q(SILENT_LOOPBACK)

Q(SILENT_LOOPBACK)

Q(SLAVE)

Q(SLAVE)

Q(SOCK_DGRAM)

Q(SOCK_RAW)

Q(SOCK_STREAM)

Q(SOFT_RESET)

Q(SOL_SOCKET)

Q(SO_KEEPALIVE)

Q(SO_RCVTIMEO)

Q(SO_REUSEADDR)

Q(SO_SNDTIMEO)

Q(SPI)

Q(SPI)

Q(SPI)

Q(SPI)

Q(SPI1)

Q(SPI2)

Q(SPI3)

Q(SPI_CR1)

Q(SPI_CR2)

Q(SPI_CRCPR)

Q(SPI_DR)

Q(SPI_I2SCFGR)

Q(SPI_I2SPR)

Q(SPI_RXCRCR)

Q(SPI_SR)

Q(SPI_TXCRCR)

Q(STA_IF)

Q(STOPPED)

Q(SYSCFG)

Q(SYSCFG_CFGR1)

Q(SYSCFG_CFGR2)

Q(SYSCFG_EXTICR0)

Q(SYSCFG_EXTICR1)

Q(SYSCFG_EXTICR2)

Q(SYSCFG_EXTICR3)

Q(SYSCFG_RCR)

Q(Signal)

Q(Signal)

Q(SoftI2C)

Q(SoftI2C)

Q(SoftSPI)

Q(SoftSPI)

Q(StopAsyncIteration)

Q(StopAsyncIteration)

Q(StopAsyncIteration)

Q(StopIteration)

Q(StopIteration)

Q(StringIO)

Q(StringIO)

Q(Switch)

Q(Switch)

Q(SyntaxError)

Q(SyntaxError)

Q(SystemExit)

Q(SystemExit)

Q(TIM1)

Q(TIM15)

Q(TIM16)

Q(TIM17)

Q(TIM2)

Q(TIM3)

Q(TIM4)

Q(TIM6)

Q(TIM7)

Q(TIM8)

Q(TIM_ARR)

Q(TIM_BDTR)

Q(TIM_CCER)

Q(TIM_CCMR1)

Q(TIM_CCMR2)

Q(TIM_CCMR3)

Q(TIM_CCR1)

Q(TIM_CCR2)

Q(TIM_CCR3)

Q(TIM_CCR4)

Q(TIM_CCR5)

Q(TIM_CCR6)

Q(TIM_CNT)

Q(TIM_CR1)

Q(TIM_CR2)

Q(TIM_DCR)

Q(TIM_DIER)

Q(TIM_DMAR)

Q(TIM_EGR)

Q(TIM_OR)

Q(TIM_PSC)

Q(TIM_RCR)

Q(TIM_SMCR)

Q(TIM_SR)

Q(TSC)

Q(Task)

Q(Task)

Q(TaskQueue)

Q(TaskQueue)

Q(TextIOWrapper)

Q(TextIOWrapper)

Q(Timer)

Q(Timer)

Q(Timer)

Q(Timer)

Q(TimerChannel)

Q(True)

Q(TypeError)

Q(TypeError)

Q(UART)

Q(UART)

Q(UART)

Q(UART4)

Q(UART5)

Q(UINT)

Q(UINT16)

Q(UINT32)

Q(UINT64)

Q(UINT8)

Q(ULONG)

Q(ULONGLONG)

Q(UP)

Q(USART1)

Q(USART2)

Q(USART3)

Q(USART_BRR)

Q(USART_CR1)

Q(USART_CR2)

Q(USART_CR3)

Q(USART_GTPR)

Q(USART_ICR)

Q(USART_ISR)

Q(USART_RDR)

Q(USART_RQR)

Q(USART_RTOR)

Q(USART_TDR)

Q(USB)

Q(USB_HID)

Q(USB_HID)

Q(USB_VCP)

Q(USB_VCP)

Q(USHORT)

Q(UnicodeError)

Q(UnicodeError)

Q(VCP)

Q(VCP_plus_HID)

Q(VCP_plus_MSC)

Q(VCP_plus_MSC_plus_HID)

Q(VOID)

Q(VREF)

Q(ValueError)

Q(ValueError)

Q(VfsFat)

Q(VfsFat)

Q(ViperTypeError)

Q(ViperTypeError)

Q(WDT)

Q(WDT)

Q(WDT_RESET)

Q(WWDG)

Q(WWDG_CFR)

Q(WWDG_CR)

Q(WWDG_SR)

Q(ZeroDivisionError)

Q(ZeroDivisionError)

Q(_)

Q(_0x0a_)

Q(__abs__)

Q(__add__)

Q(__aenter__)

Q(__aenter__)

Q(__aexit__)

Q(__aexit__)

Q(__aiter__)

Q(__and__)

Q(__anext__)

Q(__bases__)

Q(__bool__)

Q(__build_class__)

Q(__build_class__)

Q(__call__)

Q(__class__)

Q(__class__)

Q(__class__)

Q(__class__)

Q(__class__)

Q(__class__)

Q(__class__)

Q(__contains__)

Q(__contains__)

Q(__contains__)

Q(__del__)

Q(__del__)

Q(__del__)

Q(__del__)

Q(__del__)

Q(__delattr__)

Q(__delattr__)

Q(__delattr__)

Q(__delattr__)

Q(__delete__)

Q(__delete__)

Q(__delitem__)

Q(__delitem__)

Q(__dict__)

Q(__dict__)

Q(__dir__)

Q(__divmod__)

Q(__enter__)

Q(__enter__)

Q(__enter__)

Q(__enter__)

Q(__enter__)

Q(__enter__)

Q(__eq__)

Q(__eq__)

Q(__exit__)

Q(__exit__)

Q(__exit__)

Q(__exit__)

Q(__exit__)

Q(__exit__)

Q(__file__)

Q(__file__)

Q(__file__)

Q(__floordiv__)

Q(__ge__)

Q(__get__)

Q(__get__)

Q(__getattr__)

Q(__getattr__)

Q(__getattr__)

Q(__getattr__)

Q(__getitem__)

Q(__getitem__)

Q(__getitem__)

Q(__getitem__)

Q(__globals__)

Q(__gt__)

Q(__hash__)

Q(__iadd__)

Q(__import__)

Q(__import__)

Q(__init__)

Q(__init__)

Q(__init__)

Q(__init__)

Q(__init__)

Q(__int__)

Q(__invert__)

Q(__isub__)

Q(__iter__)

Q(__le__)

Q(__len__)

Q(__lshift__)

Q(__lt__)

Q(__main__)

Q(__main__)

Q(__matmul__)

Q(__mod__)

Q(__module__)

Q(__mul__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__name__)

Q(__ne__)

Q(__neg__)

Q(__new__)

Q(__new__)

Q(__new__)

Q(__next__)

Q(__next__)

Q(__next__)

Q(__next__)

Q(__or__)

Q(__path__)

Q(__path__)

Q(__path__)

Q(__pos__)

Q(__pow__)

Q(__qualname__)

Q(__radd__)

Q(__rand__)

Q(__repl_print__)

Q(__repl_print__)

Q(__repr__)

Q(__repr__)

Q(__reversed__)

Q(__rfloordiv__)

Q(__rlshift__)

Q(__rmatmul__)

Q(__rmod__)

Q(__rmul__)

Q(__ror__)

Q(__rpow__)

Q(__rrshift__)

Q(__rshift__)

Q(__rsub__)

Q(__rtruediv__)

Q(__rxor__)

Q(__set__)

Q(__set__)

Q(__setattr__)

Q(__setattr__)

Q(__setattr__)

Q(__setattr__)

Q(__setitem__)

Q(__setitem__)

Q(__str__)

Q(__sub__)

Q(__traceback__)

Q(__truediv__)

Q(__xor__)

Q(_brace_open__colon__hash_b_brace_close_)

Q(_dot__dot__dot__space_)

Q(_dot_frozen)

Q(_gt__gt__gt__space_)

Q(_lt_dictcomp_gt_)

Q(_lt_dictcomp_gt_)

Q(_lt_genexpr_gt_)

Q(_lt_genexpr_gt_)

Q(_lt_lambda_gt_)

Q(_lt_lambda_gt_)

Q(_lt_listcomp_gt_)

Q(_lt_listcomp_gt_)

Q(_lt_module_gt_)

Q(_lt_module_gt_)

Q(_lt_setcomp_gt_)

Q(_lt_setcomp_gt_)

Q(_lt_stdin_gt_)

Q(_lt_stdin_gt_)

Q(_lt_string_gt_)

Q(_machine)

Q(_mpy)

Q(_onewire)

Q(_percent__hash_o)

Q(_percent__hash_x)

Q(_slash_)

Q(_slash_)

Q(_slash_)

Q(_slash_)

Q(_slash_)

Q(_slash_flash)

Q(_slash_flash)

Q(_slash_flash_slash_lib)

Q(_slash_sd)

Q(_slash_sd_slash_lib)

Q(_space_)

Q(_star_)

Q(_star_)

Q(_star_)

Q(_task_queue)

Q(_uasyncio)

Q(_uasyncio)

Q(a2b_base64)

Q(abs)

Q(abs_tol)

Q(accept)

Q(acos)

Q(acosh)

Q(add)

Q(add)

Q(add)

Q(addr)

Q(addr)

Q(addr)

Q(addr)

Q(addr)

Q(addr_size)

Q(addressof)

Q(addrsize)

Q(af)

Q(af)

Q(af_list)

Q(align)

Q(all)

Q(alloc_emergency_exception_buf)

Q(alt)

Q(alt)

Q(and_)

Q(any)

Q(any)

Q(any)

Q(any)

Q(append)

Q(append)

Q(append)

Q(arg)

Q(args)

Q(argv)

Q(array)

Q(array)

Q(asin)

Q(asinh)

Q(asm_thumb)

Q(asr)

Q(atan)

Q(atan2)

Q(atanh)

Q(auto_restart)

Q(b)

Q(b2a_base64)

Q(bank)

Q(baudrate)

Q(baudrate)

Q(baudrate)

Q(baudrate)

Q(baudrate)

Q(baudrate)

Q(baudrate)

Q(baudrate)

Q(bin)

Q(bind)

Q(bits)

Q(bits)

Q(bits)

Q(bits)

Q(bits)

Q(bits)

Q(bitstream)

Q(bl)

Q(blit)

Q(board)

Q(board)

Q(bool)

Q(bool)

Q(bool)

Q(bool)

Q(bool)

Q(bootloader)

Q(bootloader)

Q(bound_method)

Q(brk)

Q(bs1)

Q(bs2)

Q(buffer)

Q(buffering)

Q(buffering)

Q(builtins)

Q(builtins)

Q(bx)

Q(bytearray)

Q(bytearray)

Q(bytearray_at)

Q(bytecode)

Q(byteorder)

Q(bytes)

Q(bytes)

Q(bytes)

Q(bytes_at)

Q(calcsize)

Q(calibration)

Q(callable)

Q(callback)

Q(callback)

Q(callback)

Q(callback)

Q(callback)

Q(callback)

Q(callback)

Q(cancel)

Q(capture)

Q(ceil)

Q(center)

Q(center)

Q(channel)

Q(chdir)

Q(chdir)

Q(chdir)

Q(chdir)

Q(choice)

Q(chr)

Q(classmethod)

Q(classmethod)

Q(clear)

Q(clear)

Q(clear)

Q(clearfilter)

Q(close)

Q(close)

Q(close)

Q(close)

Q(close)

Q(close)

Q(close)

Q(closure)

Q(clz)

Q(cmath)

Q(cmath)

Q(cmp)

Q(code)

Q(collect)

Q(compare)

Q(compare)

Q(compile)

Q(compile)

Q(complex)

Q(complex)

Q(connect)

Q(const)

Q(const)

Q(copy)

Q(copy)

Q(copy)

Q(copy)

Q(copysign)

Q(coro)

Q(cos)

Q(cos)

Q(cosh)

Q(count)

Q(count)

Q(count)

Q(count)

Q(count)

Q(counter)

Q(country)

Q(cpsid)

Q(cpsie)

Q(cpu)

Q(cpu)

Q(crc)

Q(crc32)

Q(crc8)

Q(cur_task)

Q(cur_task)

Q(data)

Q(data)

Q(data)

Q(data)

Q(data)

Q(data)

Q(data)

Q(data)

Q(datetime)

Q(deadtime)

Q(debug)

Q(decode)

Q(decode)

Q(decompress)

Q(deepsleep)

Q(default)

Q(degrees)

Q(deinit)

Q(deinit)

Q(deinit)

Q(deinit)

Q(deinit)

Q(deinit)

Q(deinit)

Q(deinit)

Q(delattr)

Q(delay)

Q(deleter)

Q(deque)

Q(deque)

Q(dht_readinto)

Q(dict)

Q(dict)

Q(dict)

Q(dict_view)

Q(difference)

Q(difference)

Q(difference_update)

Q(digest)

Q(dir)

Q(dir)

Q(disable)

Q(disable)

Q(disable_irq)

Q(disable_irq)

Q(discard)

Q(div)

Q(divmod)

Q(dma)

Q(doc)

Q(done)

Q(dump)

Q(dumps)

Q(dupterm)

Q(e)

Q(e)

Q(elapsed_micros)

Q(elapsed_millis)

Q(enable)

Q(enable)

Q(enable_irq)

Q(enable_irq)

Q(encode)

Q(encoding)

Q(encoding)

Q(end)

Q(endswith)

Q(endswith)

Q(enumerate)

Q(enumerate)

Q(erf)

Q(erfc)

Q(errno)

Q(errno)

Q(errorcode)

Q(eval)

Q(eval)

Q(exec)

Q(exec)

Q(execfile)

Q(exit)

Q(exp)

Q(exp)

Q(expm1)

Q(extend)

Q(extend)

Q(extframe)

Q(extframe)

Q(extframe)

Q(fabs)

Q(factorial)

Q(fault_debug)

Q(feed)

Q(fifo)

Q(fifo)

Q(file)

Q(file)

Q(file)

Q(fill)

Q(fill_rect)

Q(filter)

Q(filter)

Q(find)

Q(find)

Q(firstbit)

Q(firstbit)

Q(firstbit)

Q(firstbit)

Q(flags)

Q(float)

Q(float)

Q(floor)

Q(flow)

Q(flow)

Q(flush)

Q(flush)

Q(fmod)

Q(format)

Q(format)

Q(framebuf)

Q(framebuf)

Q(freq)

Q(freq)

Q(freq)

Q(freq)

Q(freq)

Q(freq)

Q(freq)

Q(freq)

Q(frexp)

Q(from_bytes)

Q(fromkeys)

Q(frozenset)

Q(frozenset)

Q(function)

Q(function)

Q(function)

Q(function)

Q(function)

Q(function)

Q(function)

Q(function)

Q(function)

Q(function)

Q(gamma)

Q(gc)

Q(gc)

Q(gencall)

Q(generator)

Q(generator)

Q(generator)

Q(get)

Q(getaddrinfo)

Q(getattr)

Q(getcwd)

Q(getcwd)

Q(getcwd)

Q(getrandbits)

Q(getter)

Q(getvalue)

Q(globals)

Q(gmtime)

Q(gpio)

Q(group)

Q(handler)

Q(handler)

Q(handler)

Q(hard)

Q(hard)

Q(hard)

Q(hard_reset)

Q(hasattr)

Q(hash)

Q(have_cdc)

Q(heap_lock)

Q(heap_unlock)

Q(heapify)

Q(heappop)

Q(heappush)

Q(help)

Q(hex)

Q(hexlify)

Q(hid)

Q(hid)

Q(hid_keyboard)

Q(hid_mouse)

Q(high)

Q(hline)

Q(id)

Q(id)

Q(id)

Q(id)

Q(id)

Q(idle)

Q(ilistdir)

Q(ilistdir)

Q(ilistdir)

Q(ilistdir)

Q(imag)

Q(implementation)

Q(index)

Q(index)

Q(index)

Q(index)

Q(index)

Q(indices)

Q(inf)

Q(info)

Q(info)

Q(info)

Q(info)

Q(init)

Q(init)

Q(init)

Q(init)

Q(init)

Q(init)

Q(init)

Q(init)

Q(init)

Q(init)

Q(init)

Q(init)

Q(input)

Q(insert)

Q(int)

Q(int)

Q(int)

Q(int)

Q(int)

Q(intersection)

Q(intersection)

Q(intersection_update)

Q(invert)

Q(invert)

Q(ioctl)

Q(ioctl)

Q(ioctl)

Q(ipoll)

Q(irq)

Q(irq)

Q(irq)

Q(irq)

Q(is_ready)

Q(isalpha)

Q(isalpha)

Q(isclose)

Q(isconnected)

Q(isdigit)

Q(isdigit)

Q(isdisjoint)

Q(isdisjoint)

Q(isenabled)

Q(isfinite)

Q(isinf)

Q(isinstance)

Q(islower)

Q(islower)

Q(isnan)

Q(isspace)

Q(isspace)

Q(issubclass)

Q(issubset)

Q(issubset)

Q(issuperset)

Q(issuperset)

Q(isupper)

Q(isupper)

Q(items)

Q(iter)

Q(iterable)

Q(iterator)

Q(iterator)

Q(iterator)

Q(iterator)

Q(iterator)

Q(join)

Q(join)

Q(kbd_intr)

Q(keepends)

Q(key)

Q(key)

Q(keys)

Q(keys)

Q(label)

Q(ldexp)

Q(ldr)

Q(ldrb)

Q(ldrex)

Q(ldrh)

Q(len)

Q(len)

Q(lgamma)

Q(libc_ver)

Q(lightsleep)

Q(line)

Q(line)

Q(list)

Q(list)

Q(list)

Q(listdir)

Q(listen)

Q(little)

Q(little)

Q(little)

Q(little)

Q(load)

Q(loads)

Q(locals)

Q(localtime)

Q(log)

Q(log)

Q(log10)

Q(log10)

Q(log2)

Q(low)

Q(lower)

Q(lower)

Q(lsl)

Q(lsr)

Q(lstrip)

Q(lstrip)

Q(machine)

Q(machine)

Q(main)

Q(makefile)

Q(map)

Q(map)

Q(mapper)

Q(match)

Q(match)

Q(match)

Q(math)

Q(math)

Q(max)

Q(maximum_space_recursion_space_depth_space_exceeded)

Q(maxsize)

Q(mem)

Q(mem16)

Q(mem16)

Q(mem32)

Q(mem32)

Q(mem8)

Q(mem8)

Q(mem_alloc)

Q(mem_free)

Q(mem_info)

Q(mem_read)

Q(mem_write)

Q(memaddr)

Q(memaddr)

Q(memoryview)

Q(memoryview)

Q(micropython)

Q(micropython)

Q(micropython)

Q(micropython)

Q(micros)

Q(millis)

Q(min)

Q(miso)

Q(miso)

Q(miso)

Q(mkdir)

Q(mkdir)

Q(mkdir)

Q(mkfs)

Q(mkfs)

Q(mktime)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(mode)

Q(modf)

Q(modify)

Q(module)

Q(modules)

Q(modules)

Q(mosi)

Q(mosi)

Q(mosi)

Q(mount)

Q(mount)

Q(mount)

Q(mount)

Q(mount)

Q(mov)

Q(mov)

Q(movt)

Q(movw)

Q(movwt)

Q(mrs)

Q(msc)

Q(name)

Q(name)

Q(name)

Q(namedtuple)

Q(names)

Q(nan)

Q(native)

Q(network)

Q(network)

Q(newline)

Q(next)

Q(nodename)

Q(noise)

Q(nop)

Q(nss)

Q(num_filter_banks)

Q(object)

Q(object)

Q(object)

Q(object)

Q(object)

Q(oct)

Q(off)

Q(off)

Q(on)

Q(on)

Q(onewire)

Q(open)

Q(open)

Q(open)

Q(open)

Q(opt)

Q(opt_level)

Q(ord)

Q(pack)

Q(pack_into)

Q(params)

Q(parity)

Q(partition)

Q(partition)

Q(path)

Q(peek)

Q(peektime)

Q(pend_throw)

Q(period)

Q(period)

Q(period)

Q(ph_key)

Q(phase)

Q(phase)

Q(phase)

Q(phase)

Q(phase)

Q(phase)

Q(pi)

Q(pi)

Q(pid)

Q(pin)

Q(pin)

Q(pin)

Q(pixel)

Q(platform)

Q(platform)

Q(polar)

Q(polarity)

Q(polarity)

Q(polarity)

Q(polarity)

Q(polarity)

Q(polarity)

Q(poll)

Q(poll)

Q(poll)

Q(pop)

Q(pop)

Q(pop)

Q(pop)

Q(pop)

Q(pop)

Q(popitem)

Q(popleft)

Q(port)

Q(port)

Q(pow)

Q(pow)

Q(prescaler)

Q(prescaler)

Q(prescaler)

Q(prescaler)

Q(print)

Q(print_exception)

Q(property)

Q(property)

Q(ps1)

Q(ps2)

Q(ptr)

Q(ptr)

Q(ptr16)

Q(ptr16)

Q(ptr32)

Q(ptr32)

Q(ptr8)

Q(ptr8)

Q(pull)

Q(pull)

Q(pull)

Q(pulse_width)

Q(pulse_width)

Q(pulse_width_percent)

Q(pulse_width_percent)

Q(push)

Q(push)

Q(push)

Q(pyb)

Q(pyb)

Q(pyb)

Q(python_compiler)

Q(qstr_info)

Q(r)

Q(r)

Q(radians)

Q(randint)

Q(random)

Q(randrange)

Q(range)

Q(range)

Q(range)

Q(rb)

Q(rbit)

Q(read)

Q(read)

Q(read)

Q(read)

Q(read)

Q(read)

Q(read)

Q(read)

Q(read)

Q(read)

Q(read_buf_len)

Q(read_channel)

Q(read_core_temp)

Q(read_core_vbat)

Q(read_core_vref)

Q(read_timed)

Q(read_timed_multi)

Q(read_u16)

Q(read_vref)

Q(readbit)

Q(readblocks)

Q(readblocks)

Q(readbyte)

Q(readchar)

Q(readfrom)

Q(readfrom_into)

Q(readfrom_mem)

Q(readfrom_mem_into)

Q(readinto)

Q(readinto)

Q(readinto)

Q(readinto)

Q(readinto)

Q(readinto)

Q(readinto)

Q(readinto)

Q(readinto)

Q(readinto)

Q(readinto)

Q(readline)

Q(readline)

Q(readline)

Q(readline)

Q(readline)

Q(readline)

Q(readline)

Q(readlines)

Q(readlines)

Q(readlines)

Q(readonly)

Q(real)

Q(rect)

Q(rect)

Q(recv)

Q(recv)

Q(recv)

Q(recv)

Q(recv)

Q(recv)

Q(recv)

Q(recv)

Q(recv)

Q(recvfrom)

Q(reg)

Q(register)

Q(regs)

Q(rel_tol)

Q(release)

Q(remove)

Q(remove)

Q(remove)

Q(remove)

Q(remove)

Q(remove)

Q(remove)

Q(rename)

Q(rename)

Q(rename)

Q(repl_info)

Q(repl_uart)

Q(replace)

Q(replace)

Q(repr)

Q(reset)

Q(reset)

Q(reset_cause)

Q(restart)

Q(reverse)

Q(reverse)

Q(reversed)

Q(reversed)

Q(rfind)

Q(rfind)

Q(rindex)

Q(rindex)

Q(rmdir)

Q(rmdir)

Q(rmdir)

Q(rng)

Q(rng)

Q(round)

Q(route)

Q(rpartition)

Q(rpartition)

Q(rsplit)

Q(rsplit)

Q(rstrip)

Q(rstrip)

Q(rtr)

Q(rtr)

Q(rxbuf)

Q(rxcallback)

Q(sample_point)

Q(scan)

Q(scan)

Q(schedule)

Q(sck)

Q(sck)

Q(sck)

Q(scl)

Q(scl)

Q(scroll)

Q(sda)

Q(sda)

Q(sdiv)

Q(search)

Q(search)

Q(seed)

Q(seek)

Q(seek)

Q(select)

Q(send)

Q(send)

Q(send)

Q(send)

Q(send)

Q(send)

Q(send)

Q(send)

Q(send)

Q(send)

Q(send)

Q(send_recv)

Q(sendall)

Q(sendbreak)

Q(sendto)

Q(sep)

Q(sep)

Q(separators)

Q(set)

Q(set)

Q(setattr)

Q(setblocking)

Q(setdefault)

Q(setfilter)

Q(setinterrupt)

Q(setsockopt)

Q(setter)

Q(settimeout)

Q(sha256)

Q(sha256)

Q(sin)

Q(sin)

Q(single)

Q(sinh)

Q(sizeof)

Q(sjw)

Q(sleep)

Q(sleep)

Q(sleep_ms)

Q(sleep_us)

Q(slice)

Q(slice)

Q(socket)

Q(socket)

Q(soft_reset)

Q(sort)

Q(sorted)

Q(source_freq)

Q(split)

Q(split)

Q(split)

Q(splitlines)

Q(splitlines)

Q(sqrt)

Q(sqrt)

Q(stack_use)

Q(standby)

Q(start)

Q(start)

Q(start)

Q(start)

Q(start)

Q(startswith)

Q(startswith)

Q(stat)

Q(stat)

Q(stat)

Q(stat)

Q(state)

Q(state)

Q(state)

Q(staticmethod)

Q(staticmethod)

Q(statvfs)

Q(statvfs)

Q(statvfs)

Q(stderr)

Q(stdin)

Q(stdout)

Q(step)

Q(step)

Q(stm)

Q(stm)

Q(stm)

Q(stop)

Q(stop)

Q(stop)

Q(stop)

Q(stop)

Q(str)

Q(str)

Q(str)

Q(str)

Q(str)

Q(strb)

Q(strex)

Q(strh)

Q(strip)

Q(strip)

Q(struct)

Q(struct)

Q(sub)

Q(sub)

Q(sub)

Q(sub)

Q(sum)

Q(super)

Q(super)

Q(super)

Q(swint)

Q(symmetric_difference)

Q(symmetric_difference)

Q(symmetric_difference_update)

Q(sync)

Q(sync)

Q(sync)

Q(sys)

Q(sysname)

Q(tan)

Q(tanh)

Q(tau)

Q(tell)

Q(tell)

Q(text)

Q(threshold)

Q(throw)

Q(throw)

Q(ti)

Q(tick_hz)

Q(tick_hz)

Q(ticks_add)

Q(ticks_cpu)

Q(ticks_diff)

Q(ticks_ms)

Q(ticks_us)

Q(time)

Q(time_ns)

Q(time_pulse_us)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout)

Q(timeout_char)

Q(to_bytes)

Q(triangle)

Q(trigger)

Q(trigger)

Q(trigger)

Q(trigger)

Q(trunc)

Q(tuple)

Q(tuple)

Q(tuple)

Q(type)

Q(type)

Q(uarray)

Q(uarray)

Q(ubinascii)

Q(ubinascii)

Q(ucollections)

Q(ucollections)

Q(uctypes)

Q(uctypes)

Q(uctypes)

Q(udelay)

Q(udiv)

Q(uerrno)

Q(uerrno)

Q(uhashlib)

Q(uhashlib)

Q(uheapq)

Q(uheapq)

Q(uint)

Q(uint)

Q(uint)

Q(uio)

Q(uio)

Q(ujson)

Q(ujson)

Q(umachine)

Q(umachine)

Q(umachine)

Q(umount)

Q(umount)

Q(umount)

Q(uname)

Q(unhexlify)

Q(uniform)

Q(union)

Q(union)

Q(unique_id)

Q(unique_id)

Q(unlink)

Q(unpack)

Q(unpack_from)

Q(unregister)

Q(uos)

Q(uos)

Q(update)

Q(update)

Q(update)

Q(uplatform)

Q(uplatform)

Q(upper)

Q(upper)

Q(urandom)

Q(urandom)

Q(urandom)

Q(ure)

Q(ure)

Q(ure)

Q(usb_mode)

Q(uselect)

Q(uselect)

Q(usocket)

Q(usocket)

Q(ustruct)

Q(ustruct)

Q(usys)

Q(utf_hyphen_8)

Q(utf_hyphen_8)

Q(utime)

Q(utime)

Q(utimeq)

Q(utimeq)

Q(utimeq)

Q(utimeq)

Q(uzlib)

Q(uzlib)

Q(value)

Q(value)

Q(value)

Q(value)

Q(value)

Q(value)

Q(value)

Q(value)

Q(values)

Q(vcmp)

Q(vcvt_f32_s32)

Q(vcvt_s32_f32)

Q(version)

Q(version)

Q(version)

Q(version_info)

Q(vid)

Q(viper)

Q(vldr)

Q(vline)

Q(vmov)

Q(vmrs)

Q(vneg)

Q(vsqrt)

Q(vstr)

Q(wakeup)

Q(wfi)

Q(wfi)

Q(write)

Q(write)

Q(write)

Q(write)

Q(write)

Q(write)

Q(write)

Q(write)

Q(write)

Q(write)

Q(write)

Q(write_readinto)

Q(write_readinto)

Q(write_timed)

Q(writebit)

Q(writeblocks)

Q(writeblocks)

Q(writebyte)

Q(writechar)

Q(writeto)

Q(writeto_mem)

Q(writevto)

Q(zip)

Q(zip)
