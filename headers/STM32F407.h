#define IN_REG(reg) ( reg )
#define OUT_REG(reg,value) ( reg = value )

/* Примечания:
   Соглашения EABI об использовании регистров:
    r0 -r1  Argument / result / scratch registers
    r2 -r3  Argument / scratch registers
    r4 -r8  Variable-registers
    r9      Platform register
    r10-r11 Variable-registers
    r12     The Intra-Procedure-call scratch register
    r13     The Stack Pointer
    r14     The Link Register
    r15     The Program Counter
    s0 -s15 Argument / result / scratch registers
   (d0 -d7 )
   (q0 -q3 )
    s16-s31 Variable-registers
   (d8 -d15)
   (q4 -q7 )
    d16-d31 Scratch registers
   (q8 -q15)

   Стек должен быть выравнен на 8 байт.
*/

#ifndef __STM32F407_H__
#define __STM32F407_H__ 1

#include "base_types.h"

#define BREAKPOINT  __asm ( " bkpt #0" )

#ifdef __cplusplus
extern "C" {
#endif


/* ----------------------------------------------------------------------------
 * Регистры ядра Cortex-M
 * ----------------------------------------------------------------------------
 */

/* Биты регистра APSR */
#define MCU_APSR_BITS                         ( 0xF80F0000U )
#define MCU_APSR_N                            ( 0x80000000U )
#define MCU_APSR_Z                            ( 0x40000000U )
#define MCU_APSR_C                            ( 0x20000000U )
#define MCU_APSR_V                            ( 0x10000000U )
#define MCU_APSR_Q                            ( 0x8000000U ) 
#define MCU_ASPR_GE                           ( 0xF0000U )

/* Биты регистра IPSR */
#define MCU_ISR_BITS                          ( 0x1FFU )
#define MCU_ISR_NUMBER                        ( 0x1FFU )

/* Биты регистра EPSR */
#define MCU_EPSR_BITS                         ( 0x700FC00U ) 
#define MCU_EPSR_ICI_IT                       ( 0x600FC00U )  
#define MCU_EPSR_T                            ( 0x1000000U )

/* Биты регистра PSR */
#define MCU_PSR_BITS                          ( 0xF80FFDFFU ) 
#define MCU_PSR_N                             ( 0x80000000U )
#define MCU_PSR_Z                             ( 0x40000000U )
#define MCU_PSR_C                             ( 0x20000000U )
#define MCU_PSR_V                             ( 0x10000000U )
#define MCU_PSR_Q                             ( 0x8000000U )
#define MCU_PSR_GE                            ( 0xF0000U )
#define MCU_PSR_ICI_IT                        ( 0xFC00U )  
#define MCU_PSR_ISR_NUMBER                    ( 0x1FFU )

/* Биты регистра PRIMASK */
#define MCU_PRIMASK_MASK                      ( 0x1U )
#define MCU_PRIMASK_PRIMASK                   ( 0x1U )

/* Биты регистра FAULTMASK */
#define MCU_FAULTMASK_BITS                    ( 0x1U )
#define MCU_FAULTMASK_FAULTMASK               ( 0x1U )

/* Биты регистра BASEPRI */
#define MCU_BASEPRI_BITS                      ( 0xF0U )
#define MCU_BASEPRI_BASEPRI                   ( 0xF0U )

/* Биты регистра CONTROL */
#define MCU_CONTROL_BITS                      ( 0x7U )
#define MCU_CONTROL_FPCA                      ( 0x4U )
#define MCU_CONTROL_SPSEL                     ( 0x2U )
#define MCU_CONTROL_nPRIV                     ( 0x1U )


/* Базовые адреса регистров микропроцессора */  
/* Includes the Auxiliary Control register */
#define CORE_MPU_BASE                         ( 0xE000ED90U )
#define CORE_ACTLR_BASE                       ( 0xE000E008U )
#define CORE_SYST_BASE                        ( 0xE000E010U )
#define CORE_NVIC_BASE                        ( 0xE000E100U )
#define CORE_SCB_BASE                         ( 0xE000ED00U ) 

/* ----- MPU (Memory Protection Unit) ----- */ 

#define CORE_MPU                              ( ( volatile uint8_t * ) CORE_MPU_BASE )

typedef volatile struct tag_stm32f407_mpu {
    uint32_t        typer;
    uint32_t        ctrl;
    uint32_t        rnr;
    uint32_t        rbar;
    uint32_t        rasr;
    uint32_t        rbar_a1;
    uint32_t        rasr_a1;
    uint32_t        rbar_a2;
    uint32_t        rasr_a2;
    uint32_t        rbar_a3;
    uint32_t        rasr_a3;
} stm32f407_mpu_t;

#define STM32F407_MPU_PTR                     ( ( stm32f407_mpu_t * ) CORE_MPU )

/* Биты регистра TYPER */
#define CORE_MPU_TYPER_BITS                   ( 0xFFFF01U )
#define CORE_MPU_TYPER_IREGION_MASK           ( 0xFF0000U )
#define CORE_MPU_TYPER_DREGION_MASK           ( 0xFF00U )
#define CORE_MPU_TYPER_SEPARATE               ( 0x1U )

/* Биты регистра CTRL */
#define CORE_MPU_CTRL_BITS                    ( 0x7U )
#define CORE_MPU_CTRL_PRIVDEFENA              ( 0x4U )
#define CORE_MPU_CTRL_HFNMIENA                ( 0x2U )
#define CORE_MPU_CTRL_ENABLE                  ( 0x1U )

/* Биты регистра RNR */
#define CORE_MPU_RNR_BITS                     ( 0xFFU )
#define CORE_MPU_RNR_REGION_MASK              ( 0xFFU )

/* Биты регистра RBAR */ //(?)
#define CORE_MPU_RBAR_BITS                    ( 0xFFFFFFFFU )
#define CORE_MPU_RBAR_ADDR_MASK               ( 0xFFFFFFEU )
#define CORE_MPU_RBAR_VALID                   ( 0x10U )
#define CORE_MPU_RBAR_REGION_MASK             ( 0xFU )

/* Биты регистра RASR */ 
#define CORE_MPU_RASR_BITS                    ( 0x173FFF3FU )
#define CORE_MPU_RASR_XN                      ( 0x10000000U )
#define CORE_MPU_RASR_AP_MASK                 ( 0x7000000U )
#define CORE_MPU_RASR_TEX_MASK                ( 0x380000U )
#define CORE_MPU_RASR_S                       ( 0x40000U )
#define CORE_MPU_RASR_C                       ( 0x20000U )
#define CORE_MPU_RASR_B                       ( 0x10000U )
#define CORE_MPU_RASR_SRD_MASK                ( 0xFF00U )
#define CORE_MPU_RASR_SIZE                    ( 0x3EU )
#define CORE_MPU_RASR_ENABLE                  ( 0x1U )

/* Биты регистра RBAR_A1 */ 
#define CORE_MPU_RBAR_A1_BITS                 ( 0xFFFFFFFFU )
#define CORE_MPU_RBAR_A1_ADDR_MASK            ( 0xFFFFFFEU )
#define CORE_MPU_RBAR_A1_VALID                ( 0x10U )
#define CORE_MPU_RBAR_A1_REGION_MASK          ( 0xFU )

/* Биты регистра RASR_A1 */ 
#define CORE_MPU_RASR_A1_BITS                 ( 0x173FFF3FU )
#define CORE_MPU_RASR_A1_XN                   ( 0x10000000U )
#define CORE_MPU_RASR_A1_AP_MASK              ( 0x7000000U )
#define CORE_MPU_RASR_A1_TEX_MASK             ( 0x380000U )
#define CORE_MPU_RASR_A1_S                    ( 0x40000U )
#define CORE_MPU_RASR_A1_C                    ( 0x20000U )
#define CORE_MPU_RASR_A1_B                    ( 0x10000U )
#define CORE_MPU_RASR_A1_SRD_MASK             ( 0xFF00U )
#define CORE_MPU_RASR_A1_SIZE                 ( 0x3EU )
#define CORE_MPU_RASR_A1_ENABLE               ( 0x1U )

/* Биты регистра RBAR_A2 */ 
#define CORE_MPU_RBAR_A2_BITS                 ( 0xFFFFFFFFU )
#define CORE_MPU_RBAR_A2_ADDR_MASK            ( 0xFFFFFFEU )
#define CORE_MPU_RBAR_A2_VALID                ( 0x10U )
#define CORE_MPU_RBAR_A2_REGION_MASK          ( 0xFU )

/* Биты регистра RASR_A2 */ 
#define CORE_MPU_RASR_A2_BITS                 ( 0x173FFF3FU )
#define CORE_MPU_RASR_A2_XN                   ( 0x10000000U )
#define CORE_MPU_RASR_A2_AP_MASK              ( 0x7000000U )
#define CORE_MPU_RASR_A2_TEX_MASK             ( 0x380000U )
#define CORE_MPU_RASR_A2_S                    ( 0x40000U )
#define CORE_MPU_RASR_A2_C                    ( 0x20000U )
#define CORE_MPU_RASR_A2_B                    ( 0x10000U )
#define CORE_MPU_RASR_A2_SRD_MASK             ( 0xFF00U )
#define CORE_MPU_RASR_A2_SIZE                 ( 0x3EU )
#define CORE_MPU_RASR_A2_ENABLE               ( 0x1U )

/* Биты регистра RBAR_A3 */ 
#define CORE_MPU_RBAR_A3_BITS                 ( 0xFFFFFFFFU )
#define CORE_MPU_RBAR_A3_ADDR_MASK            ( 0xFFFFFFEU )
#define CORE_MPU_RBAR_A3_VALID                ( 0x10U )
#define CORE_MPU_RBAR_A3_REGION_MASK          ( 0xFU )

/* Биты регистра RASR_A3 */ 
#define CORE_MPU_RASR_A3_BITS                 ( 0x173FFF3FU )
#define CORE_MPU_RASR_A3_XN                   ( 0x10000000U )
#define CORE_MPU_RASR_A3_AP_MASK              ( 0x7000000U )
#define CORE_MPU_RASR_A3_TEX_MASK             ( 0x380000U )
#define CORE_MPU_RASR_A3_S                    ( 0x40000U )
#define CORE_MPU_RASR_A3_C                    ( 0x20000U )
#define CORE_MPU_RASR_A3_B                    ( 0x10000U )
#define CORE_MPU_RASR_A3_SRD_MASK             ( 0xFF00U )
#define CORE_MPU_RASR_A3_SIZE                 ( 0x3EU )
#define CORE_MPU_RASR_A3_ENABLE               ( 0x1U )

/* ----- SysT (SysTick) ----- */
#define CORE_ACTLR                            ( ( volatile uint8_t * ) CORE_ACTLR_BASE )

typedef volatile struct tag_stm32f407_actlr {
    uint32_t        actlr;
} stm32f407_actlr_t;

#define STM32F407_ACTLR_PTR                   ( ( stm32f407_systick_t * ) CORE_ACTLR )

/* Биты регистра ACTLR */
#define CORE_SCB_ACTLR_BITS                   ( 0x307U )
#define CORE_SCB_ACTLR_DISOOFP                ( 0x200U )
#define CORE_SCB_ACTLR_DISFPCA                ( 0x100U )
#define CORE_SCB_ACTLR_DISFOLD                ( 0x4U )
#define CORE_SCB_ACTLR_DISDEFWBUF             ( 0x2U )
#define CORE_SCB_ACTLR_DISMCYCINT             ( 0x1U )

/* ----- SysT (SysTick) ----- */      // управление системным таймером
#define CORE_SYST                             ( ( volatile uint8_t * ) CORE_SYST_BASE )

typedef volatile struct tag_stm32f407_systick {
    uint32_t        ctrl;
    uint32_t        load;
    uint32_t        val;
    uint32_t        calib;
} stm32f407_systick_t;

#define STM32F407_SYSTICK_PTR                 ( ( stm32f407_systick_t * ) CORE_SYST )

/* Биты регистра CTRL */
#define CORE_SYSTICK_CTRL_BITS                ( 0x10007U )
#define CORE_SYSTICK_CTRL_COUNTFLAG           ( 0x10000U )
#define CORE_SYSTICK_CTRL_CLKSOURCE           ( 0x4U )
#define CORE_SYSTICK_CTRL_TICKINT             ( 0x2U )
#define CORE_SYSTICK_CTRL_ENABLE              ( 0x1U )

/* Биты регистра LOAD */
#define CORE_SYSTICK_LOAD_BITS                ( 0XFFFFFFU)
#define CORE_SYSTICK_LOAD_RELOAD_MASK         ( 0xFFFFFFU )

/* Биты регистра VAL */
#define CORE_SYSTICK_VAL_BITS                 ( 0xFFFFFFU )
#define CORE_SYSTICK_VAL_CURRENT_MASK         ( 0xFFFFFFU )

/* Биты регистра CALIB */
#define CORE_SYSTICK_CALIB_BITS               ( 0xC0FFFFFFU )
#define CORE_SYSTICK_CALIB_NOREF              ( 0x80000000U )
#define CORE_SYSTICK_CALIB_SKEW               ( 0x40000000U )
#define CORE_SYSTICK_CALIB_TENMS_MASK         ( 0xFFFFFFU )


/* ----- NVIC (Nested Vectored Interrupt Controller) ----- */
#define CORE_NVIC                             ( ( volatile uint8_t * ) CORE_NVIC_BASE )

typedef volatile struct tag_stm32f407_nvic {
    uint32_t        iser[  8 ];  /* Interrupt Set Enable Register */
    uint32_t        gap0[ 24 ];
    uint32_t        icer[  8 ];  /* Interrupt Clear Enable Register */
    uint32_t        gap1[ 24 ];
    uint32_t        ispr[  8 ];  /* Interrupt Set Pending Register */
    uint32_t        gap2[ 24 ];
    uint32_t        icpr[  8 ];  /* Interrupt Clear Pending Register */
    uint32_t        gap3[ 24 ];
    uint32_t        iabr[  8 ];  /* Interrupt MCU_ACTive bit Register */
    uint32_t        gap4[ 56 ];
    uint32_t         ipr[ 21 ];  /* Interrupt Priority Register */
    uint32_t       gap5[ 644 ];
    uint32_t        stir;        /* Software Trigger Interrupt Register */
} stm32f407_nvic_t;

#define STM32F407_NVIC_PTR                    ( ( stm32f407_nvic_t * ) CORE_NVIC )

/* Биты регистра ISER */
#define CORE_NVIC_ISER_WWDG                   ( 0x1U )
#define CORE_NVIC_ISER_PVD                    ( 0x2U )
#define CORE_NVIC_ISER_TAMP_STAMP             ( 0x4U )
#define CORE_NVIC_ISER_RTC_WKUP               ( 0x8U )
#define CORE_NVIC_ISER_FLASH                  ( 0x10U )
#define CORE_NVIC_ISER_RCC                    ( 0x20U )
#define CORE_NVIC_ISER_EXTI0                  ( 0x40U )
#define CORE_NVIC_ISER_EXTI1                  ( 0x80U )
#define CORE_NVIC_ISER_EXTI2                  ( 0x100U )
#define CORE_NVIC_ISER_EXTI3                  ( 0x200U )
#define CORE_NVIC_ISER_EXTI4                  ( 0x400U )
#define CORE_NVIC_ISER_DMA1_STR0              ( 0x800U )
#define CORE_NVIC_ISER_DMA1_STR1              ( 0x1000U )
#define CORE_NVIC_ISER_DMA1_STR2              ( 0x2000U )
#define CORE_NVIC_ISER_DMA1_STR3              ( 0x4000U )
#define CORE_NVIC_ISER_DMA1_STR4              ( 0x8000U )
#define CORE_NVIC_ISER_DMA1_STR5              ( 0x10000U )
#define CORE_NVIC_ISER_DMA1_STR6              ( 0x20000U )
#define CORE_NVIC_ISER_ADC                    ( 0x40000U )
#define CORE_NVIC_ISER_CAN1_TX                ( 0x80000U )
#define CORE_NVIC_ISER_CAN1_RX0               ( 0x100000U )
#define CORE_NVIC_ISER_CAN1_RX1               ( 0x200000U )
#define CORE_NVIC_ISER_CAN1_SCE               ( 0x400000U )
#define CORE_NVIC_ISER_EXTI9_5                ( 0x800000U )
#define CORE_NVIC_ISER_TIM1_BRK_TIM9          ( 0x1000000U )
#define CORE_NVIC_ISER_TIM1_UP_TIM10          ( 0x2000000U ) 
#define CORE_NVIC_ISER_TIM1_TRG_COM_TIM11     ( 0x4000000U ) 
#define CORE_NVIC_ISER_TIM1_CC                ( 0x8000000U )
#define CORE_NVIC_ISER_TIM2                   ( 0x10000000U )
#define CORE_NVIC_ISER_TIM3                   ( 0x20000000U )
#define CORE_NVIC_ISER_TIM4                   ( 0x40000000U )
#define CORE_NVIC_ISER_I2C1_EV                ( 0x80000000U )

#define CORE_NVIC_ISER_I2C1_ER                ( 0x1U )
#define CORE_NVIC_ISER_I2C2_EV                ( 0x2U )
#define CORE_NVIC_ISER_I2C2_ER                ( 0x4U )
#define CORE_NVIC_ISER_SPI1                   ( 0x8U )
#define CORE_NVIC_ISER_SPI2                   ( 0x10U )
#define CORE_NVIC_ISER_USART1                 ( 0x20U )
#define CORE_NVIC_ISER_USART2                 ( 0x40U )
#define CORE_NVIC_ISER_USART3                 ( 0x80U )
#define CORE_NVIC_ISER_EXTI15_10              ( 0x100U )
#define CORE_NVIC_ISER_RTC_ALARM              ( 0x200U )
#define CORE_NVIC_ISER_OTG_FS_WKUP            ( 0x400U )
#define CORE_NVIC_ISER_TIM8_BRK_TIM12         ( 0x800U )
#define CORE_NVIC_ISER_TIM8_UP_TIM13          ( 0x1000U )
#define CORE_NVIC_ISER_TIM8_TRG_COM_TIM14     ( 0x2000U )
#define CORE_NVIC_ISER_TIM8_CC                ( 0x4000U )
#define CORE_NVIC_ISER_DMA1_STR7              ( 0x8000U )
#define CORE_NVIC_ISER_FSMC                   ( 0x10000U )
#define CORE_NVIC_ISER_SDIO                   ( 0x20000U )
#define CORE_NVIC_ISER_TIM5                   ( 0x40000U )
#define CORE_NVIC_ISER_SPI3                   ( 0x80000U )
#define CORE_NVIC_ISER_UART4                  ( 0x100000U )
#define CORE_NVIC_ISER_UART5                  ( 0x200000U )
#define CORE_NVIC_ISER_TIM6_DAC               ( 0x400000U )
#define CORE_NVIC_ISER_TIM7                   ( 0x800000U )
#define CORE_NVIC_ISER_DMA2_STR0              ( 0x1000000U )
#define CORE_NVIC_ISER_DMA2_STR1              ( 0x2000000U ) 
#define CORE_NVIC_ISER_DMA2_STR2              ( 0x4000000U ) 
#define CORE_NVIC_ISER_DMA2_STR3              ( 0x8000000U )
#define CORE_NVIC_ISER_DMA2_STR4              ( 0x10000000U )
#define CORE_NVIC_ISER_ETH                    ( 0x20000000U )
#define CORE_NVIC_ISER_ETH_WKUP               ( 0x40000000U )
#define CORE_NVIC_ISER_CAN2_TX                ( 0x80000000U )

#define CORE_NVIC_ISER_CAN2_RX0               ( 0x1U )
#define CORE_NVIC_ISER_CAN2_RX1               ( 0x2U )
#define CORE_NVIC_ISER_CAN2_SCE               ( 0x4U )
#define CORE_NVIC_ISER_OTG_FS                 ( 0x8U )
#define CORE_NVIC_ISER_DMA2_STR5              ( 0x10U )
#define CORE_NVIC_ISER_DMA2_STR6              ( 0x20U )
#define CORE_NVIC_ISER_DMA2_STR7              ( 0x40U )
#define CORE_NVIC_ISER_USART6                 ( 0x80U )
#define CORE_NVIC_ISER_I2C3_EV                ( 0x100U )
#define CORE_NVIC_ISER_I2C3_ER                ( 0x200U )
#define CORE_NVIC_ISER_OTG_HS_EP1_OUT         ( 0x400U )
#define CORE_NVIC_ISER_OTG_HS_EP1_IN          ( 0x800U )
#define CORE_NVIC_ISER_OTG_HS_WKUP            ( 0x1000U )
#define CORE_NVIC_ISER_OTG_HS                 ( 0x2000U )
#define CORE_NVIC_ISER_DCMI                   ( 0x4000U )
#define CORE_NVIC_ISER_CRYP                   ( 0x8000U )
#define CORE_NVIC_ISER_HASH_RNG               ( 0x10000U )
#define CORE_NVIC_ISER_FPU                    ( 0x20000U )


/* Биты регистра ICER */
#define CORE_NVIC_ICER_WWDG                 CORE_NVIC_ISER_WWDG
#define CORE_NVIC_ICER_PVD                  CORE_NVIC_ISER_PVD
#define CORE_NVIC_ICER_TAMP_STAMP           CORE_NVIC_ISER_TAMP_STAMP
#define CORE_NVIC_ICER_RTC_WKUP             CORE_NVIC_ISER_RTC_WKUP
#define CORE_NVIC_ICER_FLASH                CORE_NVIC_ISER_FLASH
#define CORE_NVIC_ICER_RCC                  CORE_NVIC_ISER_RCC
#define CORE_NVIC_ICER_EXTI0                CORE_NVIC_ISER_EXTI0
#define CORE_NVIC_ICER_EXTI1                CORE_NVIC_ISER_EXTI1
#define CORE_NVIC_ICER_EXTI2                CORE_NVIC_ISER_EXTI2
#define CORE_NVIC_ICER_EXTI3                CORE_NVIC_ISER_EXTI3
#define CORE_NVIC_ICER_EXTI4                CORE_NVIC_ISER_EXTI4
#define CORE_NVIC_ICER_DMA1_STR0            CORE_NVIC_ISER_MCU_DMA1_STR0
#define CORE_NVIC_ICER_DMA1_STR1            CORE_NVIC_ISER_MCU_DMA1_STR1
#define CORE_NVIC_ICER_DMA1_STR2            CORE_NVIC_ISER_MCU_DMA1_STR2
#define CORE_NVIC_ICER_DMA1_STR3            CORE_NVIC_ISER_MCU_DMA1_STR3
#define CORE_NVIC_ICER_DMA1_STR4            CORE_NVIC_ISER_MCU_DMA1_STR4
#define CORE_NVIC_ICER_DMA1_STR5            CORE_NVIC_ISER_MCU_DMA1_STR5
#define CORE_NVIC_ICER_DMA1_STR6            CORE_NVIC_ISER_MCU_DMA1_STR6
#define CORE_NVIC_ICER_ADC                  CORE_NVIC_ISER_ADC
#define CORE_NVIC_ICER_CAN1_TX              CORE_NVIC_ISER_CAN1_TX
#define CORE_NVIC_ICER_CAN1_RX0             CORE_NVIC_ISER_CAN1_RX0
#define CORE_NVIC_ICER_CAN1_RX1             CORE_NVIC_ISER_CAN1_RX1
#define CORE_NVIC_ICER_CAN1_SCE             CORE_NVIC_ISER_CAN1_SCE
#define CORE_NVIC_ICER_EXTI9_5              CORE_NVIC_ISER_EXTI9_5
#define CORE_NVIC_ICER_TIM1_BRK_TIM9        CORE_NVIC_ISER_TIM1_BRK_TIM9
#define CORE_NVIC_ICER_TIM1_UP_TIM10        CORE_NVIC_ISER_TIM1_UP_TIM10 
#define CORE_NVIC_ICER_TIM1_TRG_COM_TIM11   CORE_NVIC_ISER_TIM1_TRG_COM_TIM11 
#define CORE_NVIC_ICER_TIM1_CC              CORE_NVIC_ISER_TIM1_CC
#define CORE_NVIC_ICER_TIM2                 CORE_NVIC_ISER_TIM2
#define CORE_NVIC_ICER_TIM3                 CORE_NVIC_ISER_TIM3
#define CORE_NVIC_ICER_TIM4                 CORE_NVIC_ISER_TIM4
#define CORE_NVIC_ICER_I2C1_EV              CORE_NVIC_ISER_I2C1_EV

#define CORE_NVIC_ICER_I2C1_ER              CORE_NVIC_ISER_I2C1_ER
#define CORE_NVIC_ICER_I2C2_EV              CORE_NVIC_ISER_I2C2_EV
#define CORE_NVIC_ICER_I2C2_ER              CORE_NVIC_ISER_I2C2_ER
#define CORE_NVIC_ICER_SPI1                 CORE_NVIC_ISER_SPI1
#define CORE_NVIC_ICER_SPI2                 CORE_NVIC_ISER_SPI2
#define CORE_NVIC_ICER_USART1               CORE_NVIC_ISER_MCU_USART1
#define CORE_NVIC_ICER_USART2               CORE_NVIC_ISER_MCU_USART2
#define CORE_NVIC_ICER_USART3               CORE_NVIC_ISER_MCU_USART3
#define CORE_NVIC_ICER_EXTI15_10            CORE_NVIC_ISER_EXTI15_10
#define CORE_NVIC_ICER_RTC_ALARM            CORE_NVIC_ISER_RTC_ALARM
#define CORE_NVIC_ICER_OTG_FS_WKUP          CORE_NVIC_ISER_OTG_FS_WKUP
#define CORE_NVIC_ICER_TIM8_BRK_TIM12       CORE_NVIC_ISER_TIM8_BRK_TIM12
#define CORE_NVIC_ICER_TIM8_UP_TIM13        CORE_NVIC_ISER_TIM8_UP_TIM13
#define CORE_NVIC_ICER_TIM8_TRG_COM_TIM14   CORE_NVIC_ISER_TIM8_TRG_COM_TIM14
#define CORE_NVIC_ICER_TIM8_CC              CORE_NVIC_ISER_TIM8_CC
#define CORE_NVIC_ICER_DMA1_STR7            CORE_NVIC_ISER_MCU_DMA1_STR7
#define CORE_NVIC_ICER_FSMC                 CORE_NVIC_ISER_FSMC
#define CORE_NVIC_ICER_SDIO                 CORE_NVIC_ISER_SDIO
#define CORE_NVIC_ICER_TIM5                 CORE_NVIC_ISER_TIM5
#define CORE_NVIC_ICER_SPI3                 CORE_NVIC_ISER_SPI3
#define CORE_NVIC_ICER_UART4                CORE_NVIC_ISER_UART4
#define CORE_NVIC_ICER_UART5                CORE_NVIC_ISER_UART5
#define CORE_NVIC_ICER_TIM6_DAC             CORE_NVIC_ISER_TIM6_DAC
#define CORE_NVIC_ICER_TIM7                 CORE_NVIC_ISER_TIM7
#define CORE_NVIC_ICER_DMA2_STR0            CORE_NVIC_ISER_MCU_DMA2_STR0
#define CORE_NVIC_ICER_DMA2_STR1            CORE_NVIC_ISER_MCU_DMA2_STR1
#define CORE_NVIC_ICER_DMA2_STR2            CORE_NVIC_ISER_MCU_DMA2_STR2
#define CORE_NVIC_ICER_DMA2_STR3            CORE_NVIC_ISER_MCU_DMA2_STR3
#define CORE_NVIC_ICER_DMA2_STR4            CORE_NVIC_ISER_MCU_DMA2_STR4
#define CORE_NVIC_ICER_ETH                  CORE_NVIC_ISER_ETH
#define CORE_NVIC_ICER_ETH_WKUP             CORE_NVIC_ISER_ETH_WKUP
#define CORE_NVIC_ICER_CAN2_TX              CORE_NVIC_ISER_CAN2_TX

#define CORE_NVIC_ICER_CAN2_RX0             CORE_NVIC_ISER_CAN2_RX0
#define CORE_NVIC_ICER_CAN2_RX1             CORE_NVIC_ISER_CAN2_RX1
#define CORE_NVIC_ICER_CAN2_SCE             CORE_NVIC_ISER_CAN2_SCE
#define CORE_NVIC_ICER_OTG_FS               CORE_NVIC_ISER_OTG_FS
#define CORE_NVIC_ICER_DMA2_STR5            CORE_NVIC_ISER_MCU_DMA2_STR5
#define CORE_NVIC_ICER_DMA2_STR6            CORE_NVIC_ISER_MCU_DMA2_STR6
#define CORE_NVIC_ICER_DMA2_STR7            CORE_NVIC_ISER_MCU_DMA2_STR7
#define CORE_NVIC_ICER_USART6               CORE_NVIC_ISER_MCU_USART6
#define CORE_NVIC_ICER_I2C3_EV              CORE_NVIC_ISER_I2C3_EV
#define CORE_NVIC_ICER_I2C3_ER              CORE_NVIC_ISER_I2C3_ER
#define CORE_NVIC_ICER_OTG_HS_EP1_OUT       CORE_NVIC_ISER_OTG_HS_EP1_OUT
#define CORE_NVIC_ICER_OTG_HS_EP1_IN        CORE_NVIC_ISER_OTG_HS_EP1_IN
#define CORE_NVIC_ICER_OTG_HS_WKUP          CORE_NVIC_ISER_OTG_HS_WKUP
#define CORE_NVIC_ICER_OTG_HS               CORE_NVIC_ISER_OTG_HS
#define CORE_NVIC_ICER_DCMI                 CORE_NVIC_ISER_DCMI
#define CORE_NVIC_ICER_CRYP                 CORE_NVIC_ISER_CRYP
#define CORE_NVIC_ICER_HASH_RNG             CORE_NVIC_ISER_HASH_RNG
#define CORE_NVIC_ICER_FPU                  CORE_NVIC_ISER_FPU


/* Биты регистра ISPR */
#define CORE_NVIC_ISPR_WWDG                 CORE_NVIC_ISER_WWDG
#define CORE_NVIC_ISPR_PVD                  CORE_NVIC_ISER_PVD
#define CORE_NVIC_ISPR_TAMP_STAMP           CORE_NVIC_ISER_TAMP_STAMP
#define CORE_NVIC_ISPR_RTC_WKUP             CORE_NVIC_ISER_RTC_WKUP
#define CORE_NVIC_ISPR_FLASH                CORE_NVIC_ISER_FLASH
#define CORE_NVIC_ISPR_RCC                  CORE_NVIC_ISER_RCC
#define CORE_NVIC_ISPR_EXTI0                CORE_NVIC_ISER_EXTI0
#define CORE_NVIC_ISPR_EXTI1                CORE_NVIC_ISER_EXTI1
#define CORE_NVIC_ISPR_EXTI2                CORE_NVIC_ISER_EXTI2
#define CORE_NVIC_ISPR_EXTI3                CORE_NVIC_ISER_EXTI3
#define CORE_NVIC_ISPR_EXTI4                CORE_NVIC_ISER_EXTI4
#define CORE_NVIC_ISPR_DMA1_STR0            CORE_NVIC_ISER_MCU_DMA1_STR0
#define CORE_NVIC_ISPR_DMA1_STR1            CORE_NVIC_ISER_MCU_DMA1_STR1
#define CORE_NVIC_ISPR_DMA1_STR2            CORE_NVIC_ISER_MCU_DMA1_STR2
#define CORE_NVIC_ISPR_DMA1_STR3            CORE_NVIC_ISER_MCU_DMA1_STR3
#define CORE_NVIC_ISPR_DMA1_STR4            CORE_NVIC_ISER_MCU_DMA1_STR4
#define CORE_NVIC_ISPR_DMA1_STR5            CORE_NVIC_ISER_MCU_DMA1_STR5
#define CORE_NVIC_ISPR_DMA1_STR6            CORE_NVIC_ISER_MCU_DMA1_STR6
#define CORE_NVIC_ISPR_ADC                  CORE_NVIC_ISER_ADC
#define CORE_NVIC_ISPR_CAN1_TX              CORE_NVIC_ISER_CAN1_TX
#define CORE_NVIC_ISPR_CAN1_RX0             CORE_NVIC_ISER_CAN1_RX0
#define CORE_NVIC_ISPR_CAN1_RX1             CORE_NVIC_ISER_CAN1_RX1
#define CORE_NVIC_ISPR_CAN1_SCE             CORE_NVIC_ISER_CAN1_SCE
#define CORE_NVIC_ISPR_EXTI9_5              CORE_NVIC_ISER_EXTI9_5
#define CORE_NVIC_ISPR_TIM1_BRK_TIM9        CORE_NVIC_ISER_TIM1_BRK_TIM9
#define CORE_NVIC_ISPR_TIM1_UP_TIM10        CORE_NVIC_ISER_TIM1_UP_TIM10 
#define CORE_NVIC_ISPR_TIM1_TRG_COM_TIM11   CORE_NVIC_ISER_TIM1_TRG_COM_TIM11 
#define CORE_NVIC_ISPR_TIM1_CC              CORE_NVIC_ISER_TIM1_CC
#define CORE_NVIC_ISPR_TIM2                 CORE_NVIC_ISER_TIM2
#define CORE_NVIC_ISPR_TIM3                 CORE_NVIC_ISER_TIM3
#define CORE_NVIC_ISPR_TIM4                 CORE_NVIC_ISER_TIM4
#define CORE_NVIC_ISPR_I2C1_EV              CORE_NVIC_ISER_I2C1_EV

#define CORE_NVIC_ISPR_I2C1_ER              CORE_NVIC_ISER_I2C1_ER
#define CORE_NVIC_ISPR_I2C2_EV              CORE_NVIC_ISER_I2C2_EV
#define CORE_NVIC_ISPR_I2C2_ER              CORE_NVIC_ISER_I2C2_ER
#define CORE_NVIC_ISPR_SPI1                 CORE_NVIC_ISER_SPI1
#define CORE_NVIC_ISPR_SPI2                 CORE_NVIC_ISER_SPI2
#define CORE_NVIC_ISPR_USART1               CORE_NVIC_ISER_MCU_USART1
#define CORE_NVIC_ISPR_USART2               CORE_NVIC_ISER_MCU_USART2
#define CORE_NVIC_ISPR_USART3               CORE_NVIC_ISER_MCU_USART3
#define CORE_NVIC_ISPR_EXTI15_10            CORE_NVIC_ISER_EXTI15_10
#define CORE_NVIC_ISPR_RTC_ALARM            CORE_NVIC_ISER_RTC_ALARM
#define CORE_NVIC_ISPR_OTG_FS_WKUP          CORE_NVIC_ISER_OTG_FS_WKUP
#define CORE_NVIC_ISPR_TIM8_BRK_TIM12       CORE_NVIC_ISER_TIM8_BRK_TIM12
#define CORE_NVIC_ISPR_TIM8_UP_TIM13        CORE_NVIC_ISER_TIM8_UP_TIM13
#define CORE_NVIC_ISPR_TIM8_TRG_COM_TIM14   CORE_NVIC_ISER_TIM8_TRG_COM_TIM14
#define CORE_NVIC_ISPR_TIM8_CC              CORE_NVIC_ISER_TIM8_CC
#define CORE_NVIC_ISPR_DMA1_STR7            CORE_NVIC_ISER_MCU_DMA1_STR7
#define CORE_NVIC_ISPR_FSMC                 CORE_NVIC_ISER_FSMC
#define CORE_NVIC_ISPR_SDIO                 CORE_NVIC_ISER_SDIO
#define CORE_NVIC_ISPR_TIM5                 CORE_NVIC_ISER_TIM5
#define CORE_NVIC_ISPR_SPI3                 CORE_NVIC_ISER_SPI3
#define CORE_NVIC_ISPR_UART4                CORE_NVIC_ISER_UART4
#define CORE_NVIC_ISPR_UART5                CORE_NVIC_ISER_UART5
#define CORE_NVIC_ISPR_TIM6_DAC             CORE_NVIC_ISER_TIM6_DAC
#define CORE_NVIC_ISPR_TIM7                 CORE_NVIC_ISER_TIM7
#define CORE_NVIC_ISPR_DMA2_STR0            CORE_NVIC_ISER_MCU_DMA2_STR0
#define CORE_NVIC_ISPR_DMA2_STR1            CORE_NVIC_ISER_MCU_DMA2_STR1
#define CORE_NVIC_ISPR_DMA2_STR2            CORE_NVIC_ISER_MCU_DMA2_STR2
#define CORE_NVIC_ISPR_DMA2_STR3            CORE_NVIC_ISER_MCU_DMA2_STR3
#define CORE_NVIC_ISPR_DMA2_STR4            CORE_NVIC_ISER_MCU_DMA2_STR4
#define CORE_NVIC_ISPR_ETH                  CORE_NVIC_ISER_ETH
#define CORE_NVIC_ISPR_ETH_WKUP             CORE_NVIC_ISER_ETH_WKUP
#define CORE_NVIC_ISPR_CAN2_TX              CORE_NVIC_ISER_CAN2_TX

#define CORE_NVIC_ISPR_CAN2_RX0             CORE_NVIC_ISER_CAN2_RX0
#define CORE_NVIC_ISPR_CAN2_RX1             CORE_NVIC_ISER_CAN2_RX1
#define CORE_NVIC_ISPR_CAN2_SCE             CORE_NVIC_ISER_CAN2_SCE
#define CORE_NVIC_ISPR_OTG_FS               CORE_NVIC_ISER_OTG_FS
#define CORE_NVIC_ISPR_DMA2_STR5            CORE_NVIC_ISER_MCU_DMA2_STR5
#define CORE_NVIC_ISPR_DMA2_STR6            CORE_NVIC_ISER_MCU_DMA2_STR6
#define CORE_NVIC_ISPR_DMA2_STR7            CORE_NVIC_ISER_MCU_DMA2_STR7
#define CORE_NVIC_ISPR_USART6               CORE_NVIC_ISER_MCU_USART6
#define CORE_NVIC_ISPR_I2C3_EV              CORE_NVIC_ISER_I2C3_EV
#define CORE_NVIC_ISPR_I2C3_ER              CORE_NVIC_ISER_I2C3_ER
#define CORE_NVIC_ISPR_OTG_HS_EP1_OUT       CORE_NVIC_ISER_OTG_HS_EP1_OUT
#define CORE_NVIC_ISPR_OTG_HS_EP1_IN        CORE_NVIC_ISER_OTG_HS_EP1_IN
#define CORE_NVIC_ISPR_OTG_HS_WKUP          CORE_NVIC_ISER_OTG_HS_WKUP
#define CORE_NVIC_ISPR_OTG_HS               CORE_NVIC_ISER_OTG_HS
#define CORE_NVIC_ISPR_DCMI                 CORE_NVIC_ISER_DCMI
#define CORE_NVIC_ISPR_CRYP                 CORE_NVIC_ISER_CRYP
#define CORE_NVIC_ISPR_HASH_RNG             CORE_NVIC_ISER_HASH_RNG
#define CORE_NVIC_ISPR_FPU                  CORE_NVIC_ISER_FPU

/* Биты регистра ICPR */
#define CORE_NVIC_ICPR_WWDG                 CORE_NVIC_ISER_WWDG
#define CORE_NVIC_ICPR_PVD                  CORE_NVIC_ISER_PVD
#define CORE_NVIC_ICPR_TAMP_STAMP           CORE_NVIC_ISER_TAMP_STAMP
#define CORE_NVIC_ICPR_RTC_WKUP             CORE_NVIC_ISER_RTC_WKUP
#define CORE_NVIC_ICPR_FLASH                CORE_NVIC_ISER_FLASH
#define CORE_NVIC_ICPR_RCC                  CORE_NVIC_ISER_RCC
#define CORE_NVIC_ICPR_EXTI0                CORE_NVIC_ISER_EXTI0
#define CORE_NVIC_ICPR_EXTI1                CORE_NVIC_ISER_EXTI1
#define CORE_NVIC_ICPR_EXTI2                CORE_NVIC_ISER_EXTI2
#define CORE_NVIC_ICPR_EXTI3                CORE_NVIC_ISER_EXTI3
#define CORE_NVIC_ICPR_EXTI4                CORE_NVIC_ISER_EXTI4
#define CORE_NVIC_ICPR_DMA1_STR0            CORE_NVIC_ISER_MCU_DMA1_STR0
#define CORE_NVIC_ICPR_DMA1_STR1            CORE_NVIC_ISER_MCU_DMA1_STR1
#define CORE_NVIC_ICPR_DMA1_STR2            CORE_NVIC_ISER_MCU_DMA1_STR2
#define CORE_NVIC_ICPR_DMA1_STR3            CORE_NVIC_ISER_MCU_DMA1_STR3
#define CORE_NVIC_ICPR_DMA1_STR4            CORE_NVIC_ISER_MCU_DMA1_STR4
#define CORE_NVIC_ICPR_DMA1_STR5            CORE_NVIC_ISER_MCU_DMA1_STR5
#define CORE_NVIC_ICPR_DMA1_STR6            CORE_NVIC_ISER_MCU_DMA1_STR6
#define CORE_NVIC_ICPR_ADC                  CORE_NVIC_ISER_ADC
#define CORE_NVIC_ICPR_CAN1_TX              CORE_NVIC_ISER_CAN1_TX
#define CORE_NVIC_ICPR_CAN1_RX0             CORE_NVIC_ISER_CAN1_RX0
#define CORE_NVIC_ICPR_CAN1_RX1             CORE_NVIC_ISER_CAN1_RX1
#define CORE_NVIC_ICPR_CAN1_SCE             CORE_NVIC_ISER_CAN1_SCE
#define CORE_NVIC_ICPR_EXTI9_5              CORE_NVIC_ISER_EXTI9_5
#define CORE_NVIC_ICPR_TIM1_BRK_TIM9        CORE_NVIC_ISER_TIM1_BRK_TIM9
#define CORE_NVIC_ICPR_TIM1_UP_TIM10        CORE_NVIC_ISER_TIM1_UP_TIM10 
#define CORE_NVIC_ICPR_TIM1_TRG_COM_TIM11   CORE_NVIC_ISER_TIM1_TRG_COM_TIM11 
#define CORE_NVIC_ICPR_TIM1_CC              CORE_NVIC_ISER_TIM1_CC
#define CORE_NVIC_ICPR_TIM2                 CORE_NVIC_ISER_TIM2
#define CORE_NVIC_ICPR_TIM3                 CORE_NVIC_ISER_TIM3
#define CORE_NVIC_ICPR_TIM4                 CORE_NVIC_ISER_TIM4
#define CORE_NVIC_ICPR_I2C1_EV              CORE_NVIC_ISER_I2C1_EV

#define CORE_NVIC_ICPR_I2C1_ER              CORE_NVIC_ISER_I2C1_ER
#define CORE_NVIC_ICPR_I2C2_EV              CORE_NVIC_ISER_I2C2_EV
#define CORE_NVIC_ICPR_I2C2_ER              CORE_NVIC_ISER_I2C2_ER
#define CORE_NVIC_ICPR_SPI1                 CORE_NVIC_ISER_SPI1
#define CORE_NVIC_ICPR_SPI2                 CORE_NVIC_ISER_SPI2
#define CORE_NVIC_ICPR_USART1               CORE_NVIC_ISER_MCU_USART1
#define CORE_NVIC_ICPR_USART2               CORE_NVIC_ISER_MCU_USART2
#define CORE_NVIC_ICPR_USART3               CORE_NVIC_ISER_MCU_USART3
#define CORE_NVIC_ICPR_EXTI15_10            CORE_NVIC_ISER_EXTI15_10
#define CORE_NVIC_ICPR_RTC_ALARM            CORE_NVIC_ISER_RTC_ALARM
#define CORE_NVIC_ICPR_OTG_FS_WKUP          CORE_NVIC_ISER_OTG_FS_WKUP
#define CORE_NVIC_ICPR_TIM8_BRK_TIM12       CORE_NVIC_ISER_TIM8_BRK_TIM12
#define CORE_NVIC_ICPR_TIM8_UP_TIM13        CORE_NVIC_ISER_TIM8_UP_TIM13
#define CORE_NVIC_ICPR_TIM8_TRG_COM_TIM14   CORE_NVIC_ISER_TIM8_TRG_COM_TIM14
#define CORE_NVIC_ICPR_TIM8_CC              CORE_NVIC_ISER_TIM8_CC
#define CORE_NVIC_ICPR_DMA1_STR7            CORE_NVIC_ISER_MCU_DMA1_STR7
#define CORE_NVIC_ICPR_FSMC                 CORE_NVIC_ISER_FSMC
#define CORE_NVIC_ICPR_SDIO                 CORE_NVIC_ISER_SDIO
#define CORE_NVIC_ICPR_TIM5                 CORE_NVIC_ISER_TIM5
#define CORE_NVIC_ICPR_SPI3                 CORE_NVIC_ISER_SPI3
#define CORE_NVIC_ICPR_UART4                CORE_NVIC_ISER_UART4
#define CORE_NVIC_ICPR_UART5                CORE_NVIC_ISER_UART5
#define CORE_NVIC_ICPR_TIM6_DAC             CORE_NVIC_ISER_TIM6_DAC
#define CORE_NVIC_ICPR_TIM7                 CORE_NVIC_ISER_TIM7
#define CORE_NVIC_ICPR_DMA2_STR0            CORE_NVIC_ISER_MCU_DMA2_STR0
#define CORE_NVIC_ICPR_DMA2_STR1            CORE_NVIC_ISER_MCU_DMA2_STR1
#define CORE_NVIC_ICPR_DMA2_STR2            CORE_NVIC_ISER_MCU_DMA2_STR2
#define CORE_NVIC_ICPR_DMA2_STR3            CORE_NVIC_ISER_MCU_DMA2_STR3
#define CORE_NVIC_ICPR_DMA2_STR4            CORE_NVIC_ISER_MCU_DMA2_STR4
#define CORE_NVIC_ICPR_ETH                  CORE_NVIC_ISER_ETH
#define CORE_NVIC_ICPR_ETH_WKUP             CORE_NVIC_ISER_ETH_WKUP
#define CORE_NVIC_ICPR_CAN2_TX              CORE_NVIC_ISER_CAN2_TX

#define CORE_NVIC_ICPR_CAN2_RX0             CORE_NVIC_ISER_CAN2_RX0
#define CORE_NVIC_ICPR_CAN2_RX1             CORE_NVIC_ISER_CAN2_RX1
#define CORE_NVIC_ICPR_CAN2_SCE             CORE_NVIC_ISER_CAN2_SCE
#define CORE_NVIC_ICPR_OTG_FS               CORE_NVIC_ISER_OTG_FS
#define CORE_NVIC_ICPR_DMA2_STR5            CORE_NVIC_ISER_MCU_DMA2_STR5
#define CORE_NVIC_ICPR_DMA2_STR6            CORE_NVIC_ISER_MCU_DMA2_STR6
#define CORE_NVIC_ICPR_DMA2_STR7            CORE_NVIC_ISER_MCU_DMA2_STR7
#define CORE_NVIC_ICPR_USART6               CORE_NVIC_ISER_MCU_USART6
#define CORE_NVIC_ICPR_I2C3_EV              CORE_NVIC_ISER_I2C3_EV
#define CORE_NVIC_ICPR_I2C3_ER              CORE_NVIC_ISER_I2C3_ER
#define CORE_NVIC_ICPR_OTG_HS_EP1_OUT       CORE_NVIC_ISER_OTG_HS_EP1_OUT
#define CORE_NVIC_ICPR_OTG_HS_EP1_IN        CORE_NVIC_ISER_OTG_HS_EP1_IN
#define CORE_NVIC_ICPR_OTG_HS_WKUP          CORE_NVIC_ISER_OTG_HS_WKUP
#define CORE_NVIC_ICPR_OTG_HS               CORE_NVIC_ISER_OTG_HS
#define CORE_NVIC_ICPR_DCMI                 CORE_NVIC_ISER_DCMI
#define CORE_NVIC_ICPR_CRYP                 CORE_NVIC_ISER_CRYP
#define CORE_NVIC_ICPR_HASH_RNG             CORE_NVIC_ISER_HASH_RNG
#define CORE_NVIC_ICPR_FPU                  CORE_NVIC_ISER_FPU

/* Биты регистра IPR */
#define CORE_NVIC_IPR_WWDG_MASK               ( 0xF0U )
#define CORE_NVIC_IPR_PVD_MASK                ( 0xF000U )
#define CORE_NVIC_IPR_TAMP_STAMP_MASK         ( 0xF00000U )
#define CORE_NVIC_IPR_RTC_WKUP_MASK           ( 0xF0000000U )
#define CORE_NVIC_IPR_FLASH_MASK              ( 0xF0U )
#define CORE_NVIC_IPR_RCC_MASK                ( 0xF000U )
#define CORE_NVIC_IPR_EXTI0_MASK              ( 0xF00000U )
#define CORE_NVIC_IPR_EXTI1_MASK              ( 0xF0000000U )
#define CORE_NVIC_IPR_EXTI2_MASK              ( 0xF0U )
#define CORE_NVIC_IPR_EXTI3_MASK              ( 0xF000U )
#define CORE_NVIC_IPR_EXTI4_MASK              ( 0xF00000U )
#define CORE_NVIC_IPR_DMA1_STR0_MASK          ( 0xF0000000U )
#define CORE_NVIC_IPR_DMA1_STR1_MASK          ( 0xF0U )
#define CORE_NVIC_IPR_DMA1_STR2_MASK          ( 0xF000U )
#define CORE_NVIC_IPR_DMA1_STR3_MASK          ( 0xF00000U )
#define CORE_NVIC_IPR_DMA1_STR4_MASK          ( 0xF0000000U )
#define CORE_NVIC_IPR_DMA1_STR5_MASK          ( 0xF0U )
#define CORE_NVIC_IPR_DMA1_STR6_MASK          ( 0xF000U )
#define CORE_NVIC_IPR_ADC_MASK                ( 0xF00000U )
#define CORE_NVIC_IPR_CAN1_TX_MASK            ( 0xF0000000U )
#define CORE_NVIC_IPR_CAN1_RX0_MASK           ( 0xF0U )
#define CORE_NVIC_IPR_CAN1_RX1_MASK           ( 0xF000U )
#define CORE_NVIC_IPR_CAN1_SCE_MASK           ( 0xF00000U )
#define CORE_NVIC_IPR_EXTI9_5_MASK            ( 0xF0000000U )
#define CORE_NVIC_IPR_TIM1_BRK_TIM9_MASK      ( 0xF0U )
#define CORE_NVIC_IPR_TIM1_UP_TIM10_MASK      ( 0xF000U ) 
#define CORE_NVIC_IPR_TIM1_TRG_COM_TIM11_MASK ( 0xF00000U ) 
#define CORE_NVIC_IPR_TIM1_CC_MASK            ( 0xF0000000U )
#define CORE_NVIC_IPR_TIM2_MASK               ( 0xF0U )
#define CORE_NVIC_IPR_TIM3_MASK               ( 0xF000U )
#define CORE_NVIC_IPR_TIM4_MASK               ( 0xF00000U ) 
#define CORE_NVIC_IPR_I2C1_EV_MASK            ( 0xF0000000U )

#define CORE_NVIC_IPR_I2C1_ER_MASK            ( 0xF0U )
#define CORE_NVIC_IPR_I2C2_EV_MASK            ( 0xF000U )
#define CORE_NVIC_IPR_I2C2_ER_MASK            ( 0xF00000U )
#define CORE_NVIC_IPR_SPI1_MASK               ( 0xF0000000U )
#define CORE_NVIC_IPR_SPI2_MASK               ( 0xF0U )
#define CORE_NVIC_IPR_USART1_MASK             ( 0xF000U )
#define CORE_NVIC_IPR_USART2_MASK             ( 0xF00000U )
#define CORE_NVIC_IPR_USART3_MASK             ( 0xF0000000U )
#define CORE_NVIC_IPR_EXTI15_10_MASK          ( 0xF0U )
#define CORE_NVIC_IPR_RTC_ALARM_MASK          ( 0xF000U )
#define CORE_NVIC_IPR_OTG_FS_WKUP_MASK        ( 0xF00000U )
#define CORE_NVIC_IPR_TIM8_BRK_TIM12_MASK     ( 0xF0000000U )
#define CORE_NVIC_IPR_TIM8_UP_TIM13_MASK      ( 0xF0U )
#define CORE_NVIC_IPR_TIM8_TRG_COM_TIM14_MASK ( 0xF000U )
#define CORE_NVIC_IPR_TIM8_CC_MASK            ( 0xF00000U )
#define CORE_NVIC_IPR_DMA1_STR7_MASK          ( 0xF0000000U )
#define CORE_NVIC_IPR_FSMC_MASK               ( 0xF0U )
#define CORE_NVIC_IPR_SDIO_MASK               ( 0xF000U )
#define CORE_NVIC_IPR_TIM5_MASK               ( 0xF00000U )
#define CORE_NVIC_IPR_SPI3_MASK               ( 0xF0000000U )
#define CORE_NVIC_IPR_UART4_MASK              ( 0xF0U )
#define CORE_NVIC_IPR_UART5_MASK              ( 0xF000U )
#define CORE_NVIC_IPR_TIM6_DAC_MASK           ( 0xF00000U )
#define CORE_NVIC_IPR_TIM7_MASK               ( 0xF0000000U )
#define CORE_NVIC_IPR_DMA2_STR0_MASK          ( 0xF0U )
#define CORE_NVIC_IPR_DMA2_STR1_MASK          ( 0xF000U ) 
#define CORE_NVIC_IPR_DMA2_STR2_MASK          ( 0xF00000U ) 
#define CORE_NVIC_IPR_DMA2_STR3_MASK          ( 0xF0000000U )
#define CORE_NVIC_IPR_DMA2_STR4_MASK          ( 0xF0U )
#define CORE_NVIC_IPR_ETH_MASK                ( 0xF000U )
#define CORE_NVIC_IPR_ETH_WKUP_MASK           ( 0xF00000U )
#define CORE_NVIC_IPR_CAN2_TX_MASK            ( 0xF0000000U )

#define CORE_NVIC_IPR_CAN2_RX0_MASK           ( 0xF0U )
#define CORE_NVIC_IPR_CAN2_RX1_MASK           ( 0xF000U )
#define CORE_NVIC_IPR_CAN2_SCE_MASK           ( 0xF00000U )
#define CORE_NVIC_IPR_OTG_FS_MASK             ( 0xF0000000U )
#define CORE_NVIC_IPR_DMA2_STR5_MASK          ( 0xF0U )
#define CORE_NVIC_IPR_DMA2_STR6_MASK          ( 0xF000U )
#define CORE_NVIC_IPR_DMA2_STR7_MASK          ( 0xF00000U )
#define CORE_NVIC_IPR_USART6_MASK             ( 0xF0000000U )
#define CORE_NVIC_IPR_I2C3_EV_MASK            ( 0xF0U )
#define CORE_NVIC_IPR_I2C3_ER_MASK            ( 0xF000U )
#define CORE_NVIC_IPR_OTG_HS_EP1_OUT_MASK     ( 0xF00000U )
#define CORE_NVIC_IPR_OTG_HS_EP1_IN_MASK      ( 0xF0000000U )
#define CORE_NVIC_IPR_OTG_HS_WKUP_MASK        ( 0xF0U )
#define CORE_NVIC_IPR_OTG_HS_MASK             ( 0xF000U )
#define CORE_NVIC_IPR_DCMI_MASK               ( 0xF00000U )
#define CORE_NVIC_IPR_CRYP_MASK               ( 0xF0000000U )
#define CORE_NVIC_IPR_HASH_RNG_MASK           ( 0xF0U )
#define CORE_NVIC_IPR_FPU_MASK                ( 0xF000U )

/* Биты регистра STIR */
#define CORE_NVIC_STIR_WWDG                   ( 0x0U )
#define CORE_NVIC_STIR_PVD                    ( 0x1U )
#define CORE_NVIC_STIR_TAMP_STAMP             ( 0x2U )
#define CORE_NVIC_STIR_RTC_WKUP               ( 0x3U )
#define CORE_NVIC_STIR_FLASH                  ( 0x4U )
#define CORE_NVIC_STIR_RCC                    ( 0x5U )
#define CORE_NVIC_STIR_EXTI0                  ( 0x6U )
#define CORE_NVIC_STIR_EXTI1                  ( 0x7U )
#define CORE_NVIC_STIR_EXTI2                  ( 0x8U )
#define CORE_NVIC_STIR_EXTI3                  ( 0x9U )
#define CORE_NVIC_STIR_EXTI4                  ( 0xAU )
#define CORE_NVIC_STIR_DMA1_STR0              ( 0xBU )
#define CORE_NVIC_STIR_DMA1_STR1              ( 0xCU )
#define CORE_NVIC_STIR_DMA1_STR2              ( 0xDU )
#define CORE_NVIC_STIR_DMA1_STR3              ( 0xEU )
#define CORE_NVIC_STIR_DMA1_STR4              ( 0xFU )
#define CORE_NVIC_STIR_DMA1_STR5              ( 0x10U )
#define CORE_NVIC_STIR_DMA1_STR6              ( 0x11U )
#define CORE_NVIC_STIR_ADC                    ( 0x12U )
#define CORE_NVIC_STIR_CAN1_TX                ( 0x13U )
#define CORE_NVIC_STIR_CAN1_RX0               ( 0x14U )
#define CORE_NVIC_STIR_CAN1_RX1               ( 0x15U )
#define CORE_NVIC_STIR_CAN1_SCE               ( 0x16U )
#define CORE_NVIC_STIR_EXTI9_5                ( 0x17U )
#define CORE_NVIC_STIR_TIM1_BRK_TIM9          ( 0x18U )
#define CORE_NVIC_STIR_TIM1_UP_TIM10          ( 0x19U ) 
#define CORE_NVIC_STIR_TIM1_TRG_COM_TIM11     ( 0x1AU ) 
#define CORE_NVIC_STIR_TIM1_CC                ( 0x1BU )
#define CORE_NVIC_STIR_TIM2                   ( 0x1CU )
#define CORE_NVIC_STIR_TIM3                   ( 0x1DU )
#define CORE_NVIC_STIR_TIM4                   ( 0x1EU )
#define CORE_NVIC_STIR_I2C1_EV                ( 0x1FU )

#define CORE_NVIC_STIR_I2C1_ER                ( 0x20U )
#define CORE_NVIC_STIR_I2C2_EV                ( 0x21U )
#define CORE_NVIC_STIR_I2C2_ER                ( 0x22U )
#define CORE_NVIC_STIR_SPI1                   ( 0x23U )
#define CORE_NVIC_STIR_SPI2                   ( 0x24U )
#define CORE_NVIC_STIR_USART1                 ( 0x25U )
#define CORE_NVIC_STIR_USART2                 ( 0x26U )
#define CORE_NVIC_STIR_USART3                 ( 0x27U )
#define CORE_NVIC_STIR_EXTI15_10              ( 0x28U )
#define CORE_NVIC_STIR_RTC_ALARM              ( 0x29U )
#define CORE_NVIC_STIR_OTG_FS_WKUP            ( 0x2AU )
#define CORE_NVIC_STIR_TIM8_BRK_TIM12         ( 0x2BU )
#define CORE_NVIC_STIR_TIM8_UP_TIM13          ( 0x2CU )
#define CORE_NVIC_STIR_TIM8_TRG_COM_TIM14     ( 0x2DU )
#define CORE_NVIC_STIR_TIM8_CC                ( 0x2EU )
#define CORE_NVIC_STIR_DMA1_STR7              ( 0x2FU )
#define CORE_NVIC_STIR_FSMC                   ( 0x30U )
#define CORE_NVIC_STIR_SDIO                   ( 0x31U )
#define CORE_NVIC_STIR_TIM5                   ( 0x32U )
#define CORE_NVIC_STIR_SPI3                   ( 0x33U )
#define CORE_NVIC_STIR_UART4                  ( 0x34U )
#define CORE_NVIC_STIR_UART5                  ( 0x35U )
#define CORE_NVIC_STIR_TIM6_DAC               ( 0x36U )
#define CORE_NVIC_STIR_TIM7                   ( 0x37U )
#define CORE_NVIC_STIR_DMA2_STR0              ( 0x38U )
#define CORE_NVIC_STIR_DMA2_STR1              ( 0x39U ) 
#define CORE_NVIC_STIR_DMA2_STR2              ( 0x3AU ) 
#define CORE_NVIC_STIR_DMA2_STR3              ( 0x3BU )
#define CORE_NVIC_STIR_DMA2_STR4              ( 0x3CU )
#define CORE_NVIC_STIR_ETH                    ( 0x3DU )
#define CORE_NVIC_STIR_ETH_WKUP               ( 0x3EU )
#define CORE_NVIC_STIR_CAN2_TX                ( 0x3FU )

#define CORE_NVIC_STIR_CAN2_RX0               ( 0x40U )
#define CORE_NVIC_STIR_CAN2_RX1               ( 0x41U )
#define CORE_NVIC_STIR_CAN2_SCE               ( 0x42U )
#define CORE_NVIC_STIR_OTG_FS                 ( 0x43U )
#define CORE_NVIC_STIR_DMA2_STR5              ( 0x44U )
#define CORE_NVIC_STIR_DMA2_STR6              ( 0x45U )
#define CORE_NVIC_STIR_DMA2_STR7              ( 0x46U )
#define CORE_NVIC_STIR_USART6                 ( 0x47U )
#define CORE_NVIC_STIR_I2C3_EV                ( 0x48U )
#define CORE_NVIC_STIR_I2C3_ER                ( 0x49U )
#define CORE_NVIC_STIR_OTG_HS_EP1_OUT         ( 0x4AU )
#define CORE_NVIC_STIR_OTG_HS_EP1_IN          ( 0x4BU )
#define CORE_NVIC_STIR_OTG_HS_WKUP            ( 0x4CU )
#define CORE_NVIC_STIR_OTG_HS                 ( 0x4DU )
#define CORE_NVIC_STIR_DCMI                   ( 0x4EU )
#define CORE_NVIC_STIR_CRYP                   ( 0x4FU )
#define CORE_NVIC_STIR_HASH_RNG               ( 0x50U )
#define CORE_NVIC_STIR_FPU                    ( 0x51U )



/* ----- SCB (System control block) ----- */
#define CORE_SCB                              ( ( volatile uint8_t * ) CORE_SCB_BASE )

typedef volatile struct tag_stm32f407_scb {
    uint32_t        cpuid;
    uint32_t        icsr;
    uint32_t        vtor;       
    uint32_t        aircr;
    uint32_t        scr;        
    uint32_t        ccr;
    uint32_t        shpr1;
    uint32_t        shpr2;
    uint32_t        shpr3;
    uint32_t        shcsr;
    uint32_t        cfsr;
    uint32_t        hfsr;
    uint32_t        mmfar;
    uint32_t        bfar;
    uint32_t        afsr;
    uint32_t        per[ 2 ]; 
    uint32_t        dfr;       
    uint32_t        adr;
    uint32_t        gap0[ 2 ];    
    uint32_t        mmfr[ 4 ];
    uint32_t        isar[ 5 ];
    uint32_t        gap1[ 4 ];
    uint32_t        cpacr;

} stm32f407_scb_t;

#define STM32F407_SCB_PTR                     ( ( stm32f407_scb_t * ) CORE_SCB )

/* Биты регистра CPUID */
#define CORE_SCB_CPUID_BITS                   ( 0xFFFFFFFFU )
#define CORE_SCB_CPUID_IMPLEMENTER_MASK       ( 0xFF000000U ) 
#define CORE_SCB_CPUID_VARIANT_MASK           ( 0xF00000U ) 
#define CORE_SCB_CPUID_CONSTANT_MASK          ( 0xF0000U ) 
#define CORE_SCB_CPUID_PARTNO_MASK            ( 0xFFF0U ) 
#define CORE_SCB_CPUID_REVISION_MASK          ( 0xFU ) 

/* Биты регистра ICSR */
#define CORE_SCB_ICSR_BITS                    ( 0x9E47F9FFU )
#define CORE_SCB_ICSR_NMIPENDSET              ( 0x80000000U )
#define CORE_SCB_ICSR_PENDSVSET               ( 0x10000000U )
#define CORE_SCB_ICSR_PENDSVCLR               ( 0x8000000U ) 
#define CORE_SCB_ICSR_PENDSTSET               ( 0x4000000U )
#define CORE_SCB_ICSR_PENDSTCLR               ( 0x2000000U )
#define CORE_SCB_ICSR_ISRPENDING              ( 0x400000U )
#define CORE_SCB_ICSR_VECTPENDING_MASK        ( 0x7F000U )
#define CORE_SCB_ISCR_RETOBASE                ( 0x800U ) 
#define CORE_SCB_ICSR_VECTACTIVE_MASK         ( 0x1FFU )

/* Биты регистра VTOR */
#define CORE_SCB_VTOR_BITS                    ( 0x3FFFFE00U )
#define CORE_SCB_VTOR_TABLEOFF                ( 0x3FFFFE00U )

/* Биты регистра AIRCR */
#define CORE_SCB_AIRCR_BITS                   ( 0xFFFF8707U )
#define CORE_SCB_AIRCR_VECTKEY                ( 0xFFFF0000U )
#define CORE_SCB_AIRCR_ENDIANNESS             ( 0x8000U )
#define CORE_SCB_AIRCR_PRIGROUP               ( 0x700U )
#define CORE_SCB_AIRCR_SYSRESETREQ            ( 0x4U )
#define CORE_SCB_AIRCR_VECTCLRACTIVE          ( 0x2U ) 
#define CORE_SCB_AIRCR_VECTRESET              ( 0x1U ) 

/* Биты регистра SCR */
#define CORE_SCB_SCR_BITS                     ( 0x16U )
#define CORE_SCB_SCR_SEVONPEND                ( 0x10U ) 
#define CORE_SCB_SCR_SLEEPDEEP                ( 0x4U ) 
#define CORE_SCB_SCR_SLEEPONEXIT              ( 0x2U ) 

/* Биты регистра CCR */
#define CORE_SCB_CCR_BITS                     ( 0x31BU )
#define CORE_SCB_CCR_STKALIGN                 ( 0x200U ) 
#define CORE_SCB_CCR_BFHFNIGN                 ( 0x100U ) 
#define CORE_SCB_CCR_DIV_0_TRP                ( 0x10U ) 
#define CORE_SCB_CCR_UNALIGN_TRP              ( 0x8U ) 
#define CORE_SCB_CCR_USERSETMPEND             ( 0x2U ) 
#define CORE_SCB_CCR_NONBASETHRDENA           ( 0x1U ) 

/* Биты регистра SHPR1 */
#define CORE_SHPR1_BITS                       ( 0xFFFFFFU )
#define CORE_SHPR1_PRI_6_MASK                 ( 0xFF0000U ) 
#define CORE_SHPR1_PRI_5_MASK                 ( 0xFF00U )
#define CORE_SHPR1_PRI_4_MASK                 ( 0xFFU )

/* Биты регистра SHPR2 */
#define CORE_SHPR2_BITS                       ( 0xFF000000U )
#define CORE_SHPR2_PRI_11_MASK                ( 0xFF000000U ) 

/* Биты регистра SHPR3 */
#define CORE_SHPR3_BITS                       ( 0xFFFF0000U )
#define CORE_SHPR3_PRI_15_MASK                ( 0xFF000000U ) 
#define CORE_SHPR3_PRI_14_MASK                ( 0xFF0000U ) 

/* Биты регистра SHCSR */
#define CORE_SHCSR_BITS                       ( 0x7FD8BU )
#define CORE_SHCSR_USG_FAULT_ENA              ( 0x40000U )
#define CORE_SHCSR_BUS_FAULT_ENA              ( 0x20000U )
#define CORE_SHCSR_MEM_FAULT_ENA              ( 0x10000U )
#define CORE_SHCSR_SV_CALL_PENDED             ( 0x8000U )
#define CORE_SHCSR_BUS_FAULT_PENDED           ( 0x4000U )
#define CORE_SHCSR_MEM_FAULT_PENDED           ( 0x2000U )
#define CORE_SHCSR_USG_FAULT_PENDED           ( 0x1000U )
#define CORE_SHCSR_SYS_TICK_ACT               ( 0x800U )  
#define CORE_SHCSR_PENDSV ACT                 ( 0x400U ) 
#define CORE_SHCSR_MONITOR_ACT                ( 0x100U ) 
#define CORE_SHCSR_SV_CALL_ACT                ( 0x80U ) 
#define CORE_SHCSR_USG_FAULT_ACT              ( 0x8U ) 
#define CORE_SHCSR_BUS_FAULT_ACT              ( 0x2U )
#define CORE_SHCSR_MEM_FAULT_ACT              ( 0x1U )

/* Биты регистра CFSR */
#define CORE_CFSR_BITS                        ( 0xFFFFFFFFU )
#define CORE_CFSR_UFSR                        ( 0xFFFF0000U )
#define CORE_CFSR_BFSR                        ( 0xFF00U )
#define CORE_CFSR_MMFSR                       ( 0xFFU )

/* Биты регистра HFSR */
#define CORE_HFSR_BITS                        ( 0xC0000002U )
#define CORE_HFSR_DEBUG_VT                    ( 0x80000000U )
#define CORE_HFSR_FORCED                      ( 0x40000000U )
#define CORE_HFSR_VECTTBL                     ( 0x2U )

/* Биты регистра MMFAR */
#define CORE_MMFAR_BITS                       ( 0xFFFFFFFFU )
#define CORE_MMFAR_MMAR                       ( 0xFFFFFFFFU )

/* Биты регистра BFAR */
#define CORE_BFAR_BITS                        ( 0xFFFFFFFFU )
#define CORE_BFAR_BFAR                        ( 0xFFFFFFFFU )

/* Биты регистра AFSR */
#define CORE_AFSR_BITS                        ( 0xFFFFFFFFU )
#define CORE_AFSR_AFSR                        ( 0xFFFFFFFFU )

/* ----------------------------------------------------------------------------
 * Регистры встроенной периферии микропроцессора
 * ----------------------------------------------------------------------------
 */

/* ----- ADC (Analog-to-digital converter) ----- */
typedef volatile struct tag_stm32f407_adc { 
    uint32_t        sr;
    uint32_t        cr1;
    uint32_t        cr2;
    uint32_t        smpr1;
    uint32_t        smpr2;
    uint32_t        jofr;
    uint32_t        htr;
    uint32_t        ltr;
    uint32_t        sqr1;
    uint32_t        sqr2;
    uint32_t        sqr3;
    uint32_t        jsqr;
    uint32_t        jdr;
    uint32_t        dr;
    uint32_t        csr;
    uint32_t        ccr;
    uint32_t        cdr;
} stm32f407_adc_t;

#define STM32F407_ADC_1_PTR                   ( ( stm32f407_adc_t * ) 0x40012000U )
#define STM32F407_ADC_2_PTR                   ( ( stm32f407_adc_t * ) 0x40012100U )
#define STM32F407_ADC_3_PTR                   ( ( stm32f407_adc_t * ) 0x40012200U )
#define STM32F407_ADC_COMMON_PTR              ( ( stm32f407_adc_t * ) 0x40012300U )

/* Биты регистра ADC_SR (ADC status register) */
#define MCU_ADC_SR_BITS                       ( 0x3FU )
#define MCU_ADC_SR_OVR                        ( 0x20U )
#define MCU_ADC_SR_STRT		                  ( 0x10U )
#define MCU_ADC_SR_JSTRT                      ( 0x8U )
#define MCU_ADC_SR_JEOC                       ( 0x4U )
#define MCU_ADC_SR_EOC                        ( 0x2U )
#define MCU_ADC_SR_AWD                        ( 0x1U )

/* Биты регистра ADC_CR1 (ADC control register 1) */
#define MCU_ADC_CR1_BITS      			      ( 0x7C0FFFFU )
#define MCU_ADC_CR1_OVRIE       			  ( 0x4000000U )
#define MCU_ADC_CR1_RES_6   				  ( 0x3000000U )
#define MCU_ADC_CR1_RES_8   				  ( 0x2000000U )
#define MCU_ADC_CR1_RES_10   				  ( 0x1000000U )
#define MCU_ADC_CR1_RES_12   				  ( 0x0U )
#define MCU_ADC_CR1_RES_MASK   				  ( 0x3000000U )
#define MCU_ADC_CR1_AWDEN       		      ( 0x800000U )
#define MCU_ADC_CR1_JAWDEN       			  ( 0x400000U )
#define MCU_ADC_CR1_DISCNUM_8CH  	          ( 0xE000U )
#define MCU_ADC_CR1_DISCNUM_7CH  	          ( 0xC000U )
#define MCU_ADC_CR1_DISCNUM_6CH  	          ( 0xA000U )
#define MCU_ADC_CR1_DISCNUM_5CH  	          ( 0x8000U )
#define MCU_ADC_CR1_DISCNUM_4CH  	          ( 0x6000U )
#define MCU_ADC_CR1_DISCNUM_3CH  	          ( 0x4000U )
#define MCU_ADC_CR1_DISCNUM_2CH  	          ( 0x2000U )
#define MCU_ADC_CR1_DISCNUM_1CH  	          ( 0x0U )
#define MCU_ADC_CR1_DISCNUM_MASK  	          ( 0xE000U )
#define MCU_ADC_CR1_JDISCEN       	          ( 0x1000U )
#define MCU_ADC_CR1_DISCEN       		      ( 0x800U )
#define MCU_ADC_CR1_JAUTO       		      ( 0x400U )
#define MCU_ADC_CR1_AWDSGL       			  ( 0x200U )
#define MCU_ADC_CR1_SCAN       			      ( 0x100U )
#define MCU_ADC_CR1_JEOCIE       		      ( 0x80U )
#define MCU_ADC_CR1_AWDIE       			  ( 0x40U )
#define MCU_ADC_CR1_EOCIE       			  ( 0x20U )
#define MCU_ADC_CR1_AWDCH_18CH    		      ( 0x12U )
#define MCU_ADC_CR1_AWDCH_17CH    		      ( 0x11U )
#define MCU_ADC_CR1_AWDCH_16CH    		      ( 0x10U )
#define MCU_ADC_CR1_AWDCH_15CH    		      ( 0xFU )
#define MCU_ADC_CR1_AWDCH_14CH    		      ( 0xEU )
#define MCU_ADC_CR1_AWDCH_13CH    		      ( 0xDU )
#define MCU_ADC_CR1_AWDCH_12CH    		      ( 0xCU )
#define MCU_ADC_CR1_AWDCH_11CH    		      ( 0xBU )
#define MCU_ADC_CR1_AWDCH_10CH    		      ( 0xAU )
#define MCU_ADC_CR1_AWDCH_9CH    		      ( 0x9U )
#define MCU_ADC_CR1_AWDCH_8CH    		      ( 0x8U )
#define MCU_ADC_CR1_AWDCH_7CH    		      ( 0x7U )
#define MCU_ADC_CR1_AWDCH_6CH    		      ( 0x6U )
#define MCU_ADC_CR1_AWDCH_5CH    		      ( 0x5U )
#define MCU_ADC_CR1_AWDCH_4CH    		      ( 0x4U )
#define MCU_ADC_CR1_AWDCH_3CH    		      ( 0x3U )
#define MCU_ADC_CR1_AWDCH_2CH    		      ( 0x2U )
#define MCU_ADC_CR1_AWDCH_1CH    		      ( 0x1U )
#define MCU_ADC_CR1_AWDCH_0CH    		      ( 0x0U )
#define MCU_ADC_CR1_AWDCH_MASK    		      ( 0x1FU )

/* Биты регистра ADC_CR2 (ADC control register 2) */
#define MCU_ADC_CR2_BITS      			      ( 0x7F7F0F03U )
#define MCU_ADC_CR2_SWSTART                   ( 0x40000000U )
#define MCU_ADC_CR2_EXTEN_RISING_FALLING	  ( 0x30000000U )
#define MCU_ADC_CR2_EXTEN_FALLING       	  ( 0x20000000U )
#define MCU_ADC_CR2_EXTEN_RISING       	      ( 0x10000000U )
#define MCU_ADC_CR2_EXTEN_DISABLED       	  ( 0x0U )
#define MCU_ADC_CR2_EXTEN_MASK       	      ( 0x30000000U )
#define MCU_ADC_CR2_EXTSEL_EXTI11  		      ( 0xF000000U )
#define MCU_ADC_CR2_EXTSEL_8TRGO  		      ( 0xE000000U )
#define MCU_ADC_CR2_EXTSEL_8CC1  		      ( 0xD000000U )
#define MCU_ADC_CR2_EXTSEL_5CC3  		      ( 0xC000000U )
#define MCU_ADC_CR2_EXTSEL_5CC2  		      ( 0xB000000U )
#define MCU_ADC_CR2_EXTSEL_5CC1  		      ( 0xA000000U )
#define MCU_ADC_CR2_EXTSEL_4CC4  		      ( 0x9000000U )
#define MCU_ADC_CR2_EXTSEL_3TRGO  		      ( 0x8000000U )
#define MCU_ADC_CR2_EXTSEL_3CC1  		      ( 0x7000000U )
#define MCU_ADC_CR2_EXTSEL_2TRGO  		      ( 0x6000000U )
#define MCU_ADC_CR2_EXTSEL_2CC4  		      ( 0x5000000U )
#define MCU_ADC_CR2_EXTSEL_2CC3  		      ( 0x4000000U )
#define MCU_ADC_CR2_EXTSEL_2CC2  		      ( 0x3000000U )
#define MCU_ADC_CR2_EXTSEL_1CC3  		      ( 0x2000000U )
#define MCU_ADC_CR2_EXTSEL_1CC2  		      ( 0x1000000U )
#define MCU_ADC_CR2_EXTSEL_1CC1  		      ( 0x0U )
#define MCU_ADC_CR2_EXTSEL_MASK  		      ( 0xF000000U )
#define MCU_ADC_CR2_JSWSTART       	          ( 0x400000U )
#define MCU_ADC_CR2_JEXTEN_RISING_FALLING     ( 0x300000U )
#define MCU_ADC_CR2_JEXTEN_FALLING            ( 0x200000U )
#define MCU_ADC_CR2_JEXTEN_RISING             ( 0x100000U )
#define MCU_ADC_CR2_JEXTEN_DISABLED           ( 0x0U )
#define MCU_ADC_CR2_JEXTEN_MASK               ( 0x300000U )
#define MCU_ADC_CR2_JEXTSEL_EXTI15            ( 0xF0000U )
#define MCU_ADC_CR2_JEXTSEL_8CC4              ( 0xE0000U )
#define MCU_ADC_CR2_JEXTSEL_8CC3              ( 0xD0000U )
#define MCU_ADC_CR2_JEXTSEL_8CC2              ( 0xC0000U )
#define MCU_ADC_CR2_JEXTSEL_5TRGO             ( 0xB0000U )
#define MCU_ADC_CR2_JEXTSEL_5CC4              ( 0xA0000U )
#define MCU_ADC_CR2_JEXTSEL_4TRGO             ( 0x90000U )
#define MCU_ADC_CR2_JEXTSEL_4CC3              ( 0x80000U )
#define MCU_ADC_CR2_JEXTSEL_4CC2              ( 0x70000U )
#define MCU_ADC_CR2_JEXTSEL_4CC1              ( 0x60000U )
#define MCU_ADC_CR2_JEXTSEL_3CC4              ( 0x50000U )
#define MCU_ADC_CR2_JEXTSEL_3CC2              ( 0x40000U )
#define MCU_ADC_CR2_JEXTSEL_2TRGO             ( 0x30000U )
#define MCU_ADC_CR2_JEXTSEL_2CC1              ( 0x20000U )
#define MCU_ADC_CR2_JEXTSEL_1TRGO             ( 0x10000U )
#define MCU_ADC_CR2_JEXTSEL_1CC4              ( 0x0U )
#define MCU_ADC_CR2_JEXTSEL_MASK              ( 0xF0000U )
#define MCU_ADC_CR2_ALIGN       		      ( 0x800U )
#define MCU_ADC_CR2_EOCS       			      ( 0x400U )
#define MCU_ADC_CR2_DDS       			      ( 0x200U )
#define MCU_ADC_CR2_DMA       			      ( 0x100U )
#define MCU_ADC_CR2_CONT       			      ( 0x2U )
#define MCU_ADC_CR2_ADON   					  ( 0x1U )

/* Биты регистра ADC_SMPR1 (ADC sample time register 1) */
#define MCU_ADC_SMPR1_BITS      			  ( 0x7FFFFFFU )
#define MCU_ADC_SMPR1_SMP18_MASK              ( 0x7000000U )
#define MCU_ADC_SMPR1_SMP17_MASK              ( 0xE00000U )
#define MCU_ADC_SMPR1_SMP16_MASK              ( 0x1C0000U )
#define MCU_ADC_SMPR1_SMP16_MASK              ( 0x1C0000U )
#define MCU_ADC_SMPR1_SMP15_MASK              ( 0x38000U )
#define MCU_ADC_SMPR1_SMP14_MASK              ( 0x7000U )
#define MCU_ADC_SMPR1_SMP13_MASK              ( 0xE00U )
#define MCU_ADC_SMPR1_SMP12_MASK              ( 0x1C0U )
#define MCU_ADC_SMPR1_SMP11_MASK              ( 0x38U )
#define MCU_ADC_SMPR1_SMP10_MASK              ( 0x7U )

/* Вывод порта 480 циклов */
#define MCU_ADC_SMPR1_480                     ( 0x7U )
#define MCU_ADC_SMPR1_SMP10_480               ( ( uint32_t ) MCU_ADC_SMPR1_480 << 0U  )
#define MCU_ADC_SMPR1_SMP11_480               ( ( uint32_t ) MCU_ADC_SMPR1_480 << 3U  )
#define MCU_ADC_SMPR1_SMP12_480               ( ( uint32_t ) MCU_ADC_SMPR1_480 << 6U  )
#define MCU_ADC_SMPR1_SMP13_480               ( ( uint32_t ) MCU_ADC_SMPR1_480 << 9U  )
#define MCU_ADC_SMPR1_SMP14_480               ( ( uint32_t ) MCU_ADC_SMPR1_480 << 12U  )
#define MCU_ADC_SMPR1_SMP15_480               ( ( uint32_t ) MCU_ADC_SMPR1_480 << 15U  )
#define MCU_ADC_SMPR1_SMP16_480               ( ( uint32_t ) MCU_ADC_SMPR1_480 << 18U  )
#define MCU_ADC_SMPR1_SMP17_480               ( ( uint32_t ) MCU_ADC_SMPR1_480 << 21U  )
#define MCU_ADC_SMPR1_SMP18_480               ( ( uint32_t ) MCU_ADC_SMPR1_480 << 24U  )

/* Вывод порта 144 циклов */
#define MCU_ADC_SMPR1_144                     ( 0x6U )
#define MCU_ADC_SMPR1_SMP10_144               ( ( uint32_t ) MCU_ADC_SMPR1_144 << 0U  )
#define MCU_ADC_SMPR1_SMP11_144               ( ( uint32_t ) MCU_ADC_SMPR1_144 << 3U  )
#define MCU_ADC_SMPR1_SMP12_144               ( ( uint32_t ) MCU_ADC_SMPR1_144 << 6U  )
#define MCU_ADC_SMPR1_SMP13_144               ( ( uint32_t ) MCU_ADC_SMPR1_144 << 9U  )
#define MCU_ADC_SMPR1_SMP14_144               ( ( uint32_t ) MCU_ADC_SMPR1_144 << 12U  )
#define MCU_ADC_SMPR1_SMP15_144               ( ( uint32_t ) MCU_ADC_SMPR1_144 << 15U  )
#define MCU_ADC_SMPR1_SMP16_144               ( ( uint32_t ) MCU_ADC_SMPR1_144 << 18U  )
#define MCU_ADC_SMPR1_SMP17_144               ( ( uint32_t ) MCU_ADC_SMPR1_144 << 21U  )
#define MCU_ADC_SMPR1_SMP18_144               ( ( uint32_t ) MCU_ADC_SMPR1_144 << 24U  )

/* Вывод порта 112 циклов */
#define MCU_ADC_SMPR1_112                     ( 0x5U )
#define MCU_ADC_SMPR1_SMP10_112               ( ( uint32_t ) MCU_ADC_SMPR1_112 << 0U  )
#define MCU_ADC_SMPR1_SMP11_112               ( ( uint32_t ) MCU_ADC_SMPR1_112 << 3U  )
#define MCU_ADC_SMPR1_SMP12_112               ( ( uint32_t ) MCU_ADC_SMPR1_112 << 6U  )
#define MCU_ADC_SMPR1_SMP13_112               ( ( uint32_t ) MCU_ADC_SMPR1_112 << 9U  )
#define MCU_ADC_SMPR1_SMP14_112               ( ( uint32_t ) MCU_ADC_SMPR1_112 << 12U  )
#define MCU_ADC_SMPR1_SMP15_112               ( ( uint32_t ) MCU_ADC_SMPR1_112 << 15U  )
#define MCU_ADC_SMPR1_SMP16_112               ( ( uint32_t ) MCU_ADC_SMPR1_112 << 18U  )
#define MCU_ADC_SMPR1_SMP17_112               ( ( uint32_t ) MCU_ADC_SMPR1_112 << 21U  )
#define MCU_ADC_SMPR1_SMP18_112               ( ( uint32_t ) MCU_ADC_SMPR1_112 << 24U  )

/* Вывод порта 84 циклов */
#define MCU_ADC_SMPR1_84                      ( 0x4U )
#define MCU_ADC_SMPR1_SMP10_84                ( ( uint32_t ) MCU_ADC_SMPR1_84 << 0U  )
#define MCU_ADC_SMPR1_SMP11_84                ( ( uint32_t ) MCU_ADC_SMPR1_84 << 3U  )
#define MCU_ADC_SMPR1_SMP12_84                ( ( uint32_t ) MCU_ADC_SMPR1_84 << 6U  )
#define MCU_ADC_SMPR1_SMP13_84                ( ( uint32_t ) MCU_ADC_SMPR1_84 << 9U  )
#define MCU_ADC_SMPR1_SMP14_84                ( ( uint32_t ) MCU_ADC_SMPR1_84 << 12U  )
#define MCU_ADC_SMPR1_SMP15_84                ( ( uint32_t ) MCU_ADC_SMPR1_84 << 15U  )
#define MCU_ADC_SMPR1_SMP16_84                ( ( uint32_t ) MCU_ADC_SMPR1_84 << 18U  )
#define MCU_ADC_SMPR1_SMP17_84                ( ( uint32_t ) MCU_ADC_SMPR1_84 << 21U  )
#define MCU_ADC_SMPR1_SMP18_84                ( ( uint32_t ) MCU_ADC_SMPR1_84 << 24U  )

/* Вывод порта 56 циклов */
#define MCU_ADC_SMPR1_56                      ( 0x3U )
#define MCU_ADC_SMPR1_SMP10_56                ( ( uint32_t ) MCU_ADC_SMPR1_56 << 0U  )
#define MCU_ADC_SMPR1_SMP11_56                ( ( uint32_t ) MCU_ADC_SMPR1_56 << 3U  )
#define MCU_ADC_SMPR1_SMP12_56                ( ( uint32_t ) MCU_ADC_SMPR1_56 << 6U  )
#define MCU_ADC_SMPR1_SMP13_56                ( ( uint32_t ) MCU_ADC_SMPR1_56 << 9U  )
#define MCU_ADC_SMPR1_SMP14_56                ( ( uint32_t ) MCU_ADC_SMPR1_56 << 12U  )
#define MCU_ADC_SMPR1_SMP15_56                ( ( uint32_t ) MCU_ADC_SMPR1_56 << 15U  )
#define MCU_ADC_SMPR1_SMP16_56                ( ( uint32_t ) MCU_ADC_SMPR1_56 << 18U  )
#define MCU_ADC_SMPR1_SMP17_56                ( ( uint32_t ) MCU_ADC_SMPR1_56 << 21U  )
#define MCU_ADC_SMPR1_SMP18_56                ( ( uint32_t ) MCU_ADC_SMPR1_56 << 24U  )

/* Вывод порта 28 циклов */
#define MCU_ADC_SMPR1_28                      ( 0x2U )
#define MCU_ADC_SMPR1_SMP10_28                ( ( uint32_t ) MCU_ADC_SMPR1_28 << 0U  )
#define MCU_ADC_SMPR1_SMP11_28                ( ( uint32_t ) MCU_ADC_SMPR1_28 << 3U  )
#define MCU_ADC_SMPR1_SMP12_28                ( ( uint32_t ) MCU_ADC_SMPR1_28 << 6U  )
#define MCU_ADC_SMPR1_SMP13_28                ( ( uint32_t ) MCU_ADC_SMPR1_28 << 9U  )
#define MCU_ADC_SMPR1_SMP14_28                ( ( uint32_t ) MCU_ADC_SMPR1_28 << 12U  )
#define MCU_ADC_SMPR1_SMP15_28                ( ( uint32_t ) MCU_ADC_SMPR1_28 << 15U  )
#define MCU_ADC_SMPR1_SMP16_28                ( ( uint32_t ) MCU_ADC_SMPR1_28 << 18U  )
#define MCU_ADC_SMPR1_SMP17_28                ( ( uint32_t ) MCU_ADC_SMPR1_28 << 21U  )
#define MCU_ADC_SMPR1_SMP18_28                ( ( uint32_t ) MCU_ADC_SMPR1_28 << 24U  )

/* Вывод порта 15 циклов */
#define MCU_ADC_SMPR1_15                      ( 0x1U )
#define MCU_ADC_SMPR1_SMP10_15                ( ( uint32_t ) MCU_ADC_SMPR1_15 << 0U  )
#define MCU_ADC_SMPR1_SMP11_15                ( ( uint32_t ) MCU_ADC_SMPR1_15 << 3U  )
#define MCU_ADC_SMPR1_SMP12_15                ( ( uint32_t ) MCU_ADC_SMPR1_15 << 6U  )
#define MCU_ADC_SMPR1_SMP13_15                ( ( uint32_t ) MCU_ADC_SMPR1_15 << 9U  )
#define MCU_ADC_SMPR1_SMP14_15                ( ( uint32_t ) MCU_ADC_SMPR1_15 << 12U  )
#define MCU_ADC_SMPR1_SMP15_15                ( ( uint32_t ) MCU_ADC_SMPR1_15 << 15U  )
#define MCU_ADC_SMPR1_SMP16_15                ( ( uint32_t ) MCU_ADC_SMPR1_15 << 18U  )
#define MCU_ADC_SMPR1_SMP17_15                ( ( uint32_t ) MCU_ADC_SMPR1_15 << 21U  )
#define MCU_ADC_SMPR1_SMP18_15                ( ( uint32_t ) MCU_ADC_SMPR1_15 << 24U  )

/* Вывод порта 3 цикла */
#define MCU_ADC_SMPR1_3                       ( 0x0U )
#define MCU_ADC_SMPR1_SMP10_3                 ( ( uint32_t ) MCU_ADC_SMPR1_3 << 0U  )
#define MCU_ADC_SMPR1_SMP11_3                 ( ( uint32_t ) MCU_ADC_SMPR1_3 << 3U  )
#define MCU_ADC_SMPR1_SMP12_3                 ( ( uint32_t ) MCU_ADC_SMPR1_3 << 6U  )
#define MCU_ADC_SMPR1_SMP13_3                 ( ( uint32_t ) MCU_ADC_SMPR1_3 << 9U  )
#define MCU_ADC_SMPR1_SMP14_3                 ( ( uint32_t ) MCU_ADC_SMPR1_3 << 12U  )
#define MCU_ADC_SMPR1_SMP15_3                 ( ( uint32_t ) MCU_ADC_SMPR1_3 << 15U  )
#define MCU_ADC_SMPR1_SMP16_3                 ( ( uint32_t ) MCU_ADC_SMPR1_3 << 18U  )
#define MCU_ADC_SMPR1_SMP17_3                 ( ( uint32_t ) MCU_ADC_SMPR1_3 << 21U  )
#define MCU_ADC_SMPR1_SMP18_3                 ( ( uint32_t ) MCU_ADC_SMPR1_3 << 24U  )

/* Биты регистра ADC_SMPR2 (ADC sample time register 2) */
#define MCU_ADC_SMPR2_BITS      			  ( 0x3FFFFFFFU )
#define MCU_ADC_SMPR2_SMP9_MASK               ( 0x38000000U )
#define MCU_ADC_SMPR2_SMP8_MASK               ( 0x7000000U )
#define MCU_ADC_SMPR2_SMP7_MASK               ( 0xE00000U )
#define MCU_ADC_SMPR2_SMP6_MASK               ( 0x1C0000U )
#define MCU_ADC_SMPR2_SMP5_MASK               ( 0x38000U )
#define MCU_ADC_SMPR2_SMP4_MASK               ( 0x7000U )
#define MCU_ADC_SMPR2_SMP3_MASK               ( 0xE00U )
#define MCU_ADC_SMPR2_SMP2_MASK               ( 0x1C0U )
#define MCU_ADC_SMPR2_SMP1_MASK               ( 0x38U )
#define MCU_ADC_SMPR2_SMP0_MASK               ( 0x7U )

/* Вывод порта 480 циклов */
#define MCU_ADC_SMPR2_480                     ( 0x7U )
#define MCU_ADC_SMPR2_SMP0_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 0U  )
#define MCU_ADC_SMPR2_SMP1_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 3U  )
#define MCU_ADC_SMPR2_SMP2_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 6U  )
#define MCU_ADC_SMPR2_SMP3_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 9U  )
#define MCU_ADC_SMPR2_SMP4_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 12U  )
#define MCU_ADC_SMPR2_SMP5_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 15U  )
#define MCU_ADC_SMPR2_SMP6_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 18U  )
#define MCU_ADC_SMPR2_SMP7_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 21U  )
#define MCU_ADC_SMPR2_SMP8_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 24U  )
#define MCU_ADC_SMPR2_SMP9_480                ( ( uint32_t ) MCU_ADC_SMPR2_480 << 27U  )

/* Вывод порта 144 циклов */
#define MCU_ADC_SMPR2_144                     ( 0x6U )
#define MCU_ADC_SMPR2_SMP0_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 0U  )
#define MCU_ADC_SMPR2_SMP1_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 3U  )
#define MCU_ADC_SMPR2_SMP2_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 6U  )
#define MCU_ADC_SMPR2_SMP3_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 9U  )
#define MCU_ADC_SMPR2_SMP4_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 12U  )
#define MCU_ADC_SMPR2_SMP5_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 15U  )
#define MCU_ADC_SMPR2_SMP6_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 18U  )
#define MCU_ADC_SMPR2_SMP7_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 21U  )
#define MCU_ADC_SMPR2_SMP8_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 24U  )
#define MCU_ADC_SMPR2_SMP9_144                ( ( uint32_t ) MCU_ADC_SMPR2_144 << 27U  )

/* Вывод порта 112 циклов */
#define MCU_ADC_SMPR2_112                     ( 0x5U )
#define MCU_ADC_SMPR2_SMP0_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 0U  )
#define MCU_ADC_SMPR2_SMP1_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 3U  )
#define MCU_ADC_SMPR2_SMP2_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 6U  )
#define MCU_ADC_SMPR2_SMP3_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 9U  )
#define MCU_ADC_SMPR2_SMP4_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 12U  )
#define MCU_ADC_SMPR2_SMP5_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 15U  )
#define MCU_ADC_SMPR2_SMP6_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 18U  )
#define MCU_ADC_SMPR2_SMP7_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 21U  )
#define MCU_ADC_SMPR2_SMP8_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 24U  )
#define MCU_ADC_SMPR2_SMP9_112                ( ( uint32_t ) MCU_ADC_SMPR2_12 << 27U  )

/* Вывод порта 84 циклов */
#define MCU_ADC_SMPR2_84                      ( 0x4U )
#define MCU_ADC_SMPR2_SMP0_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 0U  )
#define MCU_ADC_SMPR2_SMP1_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 3U  )
#define MCU_ADC_SMPR2_SMP2_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 6U  )
#define MCU_ADC_SMPR2_SMP3_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 9U  )
#define MCU_ADC_SMPR2_SMP4_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 12U  )
#define MCU_ADC_SMPR2_SMP5_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 15U  )
#define MCU_ADC_SMPR2_SMP6_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 18U  )
#define MCU_ADC_SMPR2_SMP7_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 21U  )
#define MCU_ADC_SMPR2_SMP8_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 24U  )
#define MCU_ADC_SMPR2_SMP9_84                 ( ( uint32_t ) MCU_ADC_SMPR2_84 << 27U  )

/* Вывод порта 56 циклов */
#define MCU_ADC_SMPR2_56                      ( 0x3U )
#define MCU_ADC_SMPR2_SMP0_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 0U  )
#define MCU_ADC_SMPR2_SMP1_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 3U  )
#define MCU_ADC_SMPR2_SMP2_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 6U  )
#define MCU_ADC_SMPR2_SMP3_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 9U  )
#define MCU_ADC_SMPR2_SMP4_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 12U  )
#define MCU_ADC_SMPR2_SMP5_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 15U  )
#define MCU_ADC_SMPR2_SMP6_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 18U  )
#define MCU_ADC_SMPR2_SMP7_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 21U  )
#define MCU_ADC_SMPR2_SMP8_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 24U  )
#define MCU_ADC_SMPR2_SMP9_56                 ( ( uint32_t ) MCU_ADC_SMPR2_56 << 27U  )

/* Вывод порта 28 циклов */
#define MCU_ADC_SMPR2_28                      ( 0x2U )
#define MCU_ADC_SMPR2_SMP0_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 0U  )
#define MCU_ADC_SMPR2_SMP1_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 3U  )
#define MCU_ADC_SMPR2_SMP2_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 6U  )
#define MCU_ADC_SMPR2_SMP3_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 9U  )
#define MCU_ADC_SMPR2_SMP4_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 12U  )
#define MCU_ADC_SMPR2_SMP5_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 15U  )
#define MCU_ADC_SMPR2_SMP6_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 18U  )
#define MCU_ADC_SMPR2_SMP7_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 21U  )
#define MCU_ADC_SMPR2_SMP8_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 24U  )
#define MCU_ADC_SMPR2_SMP9_28                 ( ( uint32_t ) MCU_ADC_SMPR2_28 << 27U  )

/* Вывод порта 15 циклов */
#define MCU_ADC_SMPR2_15                      ( 0x1U )
#define MCU_ADC_SMPR2_SMP0_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 0U  )
#define MCU_ADC_SMPR2_SMP1_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 3U  )
#define MCU_ADC_SMPR2_SMP2_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 6U  )
#define MCU_ADC_SMPR2_SMP3_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 9U  )
#define MCU_ADC_SMPR2_SMP4_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 12U  )
#define MCU_ADC_SMPR2_SMP5_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 15U  )
#define MCU_ADC_SMPR2_SMP6_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 18U  )
#define MCU_ADC_SMPR2_SMP7_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 21U  )
#define MCU_ADC_SMPR2_SMP8_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 24U  )
#define MCU_ADC_SMPR2_SMP9_15                 ( ( uint32_t ) MCU_ADC_SMPR2_15 << 27U  )

/* Вывод порта 3 цикла */
#define MCU_ADC_SMPR2_3                       ( 0x0U )
#define MCU_ADC_SMPR2_SMP0_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 0U  )
#define MCU_ADC_SMPR2_SMP1_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 3U  )
#define MCU_ADC_SMPR2_SMP2_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 6U  )
#define MCU_ADC_SMPR2_SMP3_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 9U  )
#define MCU_ADC_SMPR2_SMP4_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 12U  )
#define MCU_ADC_SMPR2_SMP5_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 15U  )
#define MCU_ADC_SMPR2_SMP6_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 18U  )
#define MCU_ADC_SMPR2_SMP7_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 21U  )
#define MCU_ADC_SMPR2_SMP8_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 24U  )
#define MCU_ADC_SMPR2_SMP9_3                  ( ( uint32_t ) MCU_ADC_SMPR2_3 << 27U  )

/* Биты регистра ADC_JOFR (ADC injected channel data offset register) */
#define MCU_ADC_JOFR_BITS      			  	  ( 0xFFFU )
#define MCU_ADC_JOFR_JOFFSET_MASK             ( 0xFFFU )

/* Биты регистра ADC_HTR (ADC watchdog higher threshold register) */
#define MCU_ADC_HTR_BITS      			  	  ( 0xFFFU )
#define MCU_ADC_HTR_HT_MASK          	      ( 0xFFFU )

/* Биты регистра ADC_LTR (ADC watchdog lower threshold register) */
#define MCU_ADC_LTR_BITS      			  	  ( 0xFFFU )
#define MCU_ADC_LTR_LT_MASK          	      ( 0xFFFU )

/* Биты регистра ADC_SQR1 (ADC regular sequence register 1) */
#define MCU_ADC_SQR1_BITS      			  	  ( 0xFFFFFFU )
#define MCU_ADC_SQR1_L_16         			  ( 0xF00000U )
#define MCU_ADC_SQR1_L_15         			  ( 0xE00000U )
#define MCU_ADC_SQR1_L_14         			  ( 0xD00000U )
#define MCU_ADC_SQR1_L_13         			  ( 0xC00000U )
#define MCU_ADC_SQR1_L_12         		      ( 0xB00000U )
#define MCU_ADC_SQR1_L_11         			  ( 0xA00000U )
#define MCU_ADC_SQR1_L_10         			  ( 0x900000U )
#define MCU_ADC_SQR1_L_9         			  ( 0x800000U )
#define MCU_ADC_SQR1_L_8         			  ( 0x700000U )
#define MCU_ADC_SQR1_L_7         			  ( 0x600000U )
#define MCU_ADC_SQR1_L_6         			  ( 0x500000U )
#define MCU_ADC_SQR1_L_5         			  ( 0x400000U )
#define MCU_ADC_SQR1_L_4         			  ( 0x300000U )
#define MCU_ADC_SQR1_L_3         			  ( 0x200000U )
#define MCU_ADC_SQR1_L_2         			  ( 0x100000U )
#define MCU_ADC_SQR1_L_1         			  ( 0x0U )
#define MCU_ADC_SQR1_L_MASK         	      ( 0xF00000U )
#define MCU_ADC_SQR1_SQ16_MASK        		  ( 0xF0000U )
#define MCU_ADC_SQR1_SQ16_0                   ( 0x8000U )
#define MCU_ADC_SQR1_SQ15_MASK         		  ( 0x7C00U )
#define MCU_ADC_SQR1_SQ14_MASK         		  ( 0x3E0U )
#define MCU_ADC_SQR1_SQ13_MASK         		  ( 0x1FU )

/* Биты регистра ADC_SQR2 (ADC regular sequence register 2) */
#define MCU_ADC_SQR2_BITS      			  	  ( 0x3FFFFFFFU )
#define MCU_ADC_SQR2_SQ12_MASK         		  ( 0x3E000000U )
#define MCU_ADC_SQR2_SQ11_MASK         		  ( 0x1F00000U )
#define MCU_ADC_SQR2_SQ10_MASK         		  ( 0xF0000U )
#define MCU_ADC_SQR2_SQ10_0         	      ( 0x8000U )
#define MCU_ADC_SQR2_SQ9_MASK         		  ( 0x7C00U )
#define MCU_ADC_SQR2_SQ8_MASK         		  ( 0x3E0U )
#define MCU_ADC_SQR2_SQ7_MASK         		  ( 0x1FU )

/* Биты регистра ADC_SQR3 (ADC regular sequence register 3) */
#define MCU_ADC_SQR3_BITS      			  	  ( 0x3FFFFFFFU )
#define MCU_ADC_SQR3_SQ6_MASK         		  ( 0x3E000000U )
#define MCU_ADC_SQR3_SQ5_MASK         		  ( 0x1F00000U )
#define MCU_ADC_SQR3_SQ4_MASK         		  ( 0xF0000U )
#define MCU_ADC_SQR3_SQ4_0         			  ( 0x8000U )
#define MCU_ADC_SQR3_SQ3_MASK         		  ( 0x7C00U )
#define MCU_ADC_SQR3_SQ2_MASK         		  ( 0x3E0U )
#define MCU_ADC_SQR3_SQ1_MASK         		  ( 0x1FU )

/* Биты регистра ADC_JSQR (ADC injected sequence register) */
#define MCU_ADC_JSQR_BITS      			  	  ( 0x3FFFFFU )
#define MCU_ADC_JSQR_JL_4         		      ( 0x300000U )
#define MCU_ADC_JSQR_JL_3         		      ( 0x200000U )
#define MCU_ADC_JSQR_JL_2         		      ( 0x100000U )
#define MCU_ADC_JSQR_JL_1         		      ( 0x0U )
#define MCU_ADC_JSQR_JL_MASK         		  ( 0x300000U )
#define MCU_ADC_JSQR_JSQ4_MASK         		  ( 0xF0000U )
#define MCU_ADC_JSQR_JSQ4_0                   ( 0x8000U )
#define MCU_ADC_JSQR_JSQ3_MASK         		  ( 0x7C00U )
#define MCU_ADC_JSQR_JSQ2_MASK         		  ( 0x3E0U )
#define MCU_ADC_JSQR_JSQ1_MASK         		  ( 0x1FU )

/* Биты регистра ADC_JDR (ADC injected data register) */
#define MCU_ADC_JDR_BITS      			  	  ( 0xFFFFU )
#define MCU_ADC_JDR_JDATA_MASK          	  ( 0xFFFFU )

/* Биты регистра ADC_DR (ADC regular data register) */
#define MCU_ADC_DR_BITS      			  	  ( 0xFFFFU )
#define MCU_ADC_DR_DATA_MASK          	      ( 0xFFFFU )

/* Биты регистра ADC_CSR (ADC common status register) */
#define MCU_ADC_CSR_BITS      			      ( ( uint32_t ) STM32F407_ADC_COMMON_PTR << 0U  )
#define MCU_ADC_CSR_ADC3_OVR3 				  ( 0x200000U )
#define MCU_ADC_CSR_ADC3_STRT3   	          ( 0x100000U )
#define MCU_ADC_CSR_ADC3_JSTRT3               ( 0x080000U )
#define MCU_ADC_CSR_ADC3_JEOC3     	          ( 0x40000U )
#define MCU_ADC_CSR_ADC3_EOC3     	          ( 0x20000U )
#define MCU_ADC_CSR_ADC3_AWD3       	      ( 0x10000U )
#define MCU_ADC_CSR_ADC2_OVR2        	      ( 0x2000U )
#define MCU_ADC_CSR_ADC2_STRT2     	          ( 0x1000U )
#define MCU_ADC_CSR_ADC2_JSTRT2  	          ( 0x800U )
#define MCU_ADC_CSR_ADC2_JEOC2    	          ( 0x400U )
#define MCU_ADC_CSR_ADC2_EOC2  	 			  ( 0x200U )
#define MCU_ADC_CSR_ADC2_AWD2      	 	      ( 0x100U )
#define MCU_ADC_CSR_ADC1_OVR1                 ( 0x20U )
#define MCU_ADC_CSR_ADC1_STRT1   	          ( 0x10U )
#define MCU_ADC_CSR_ADC1_JSTRT1               ( 0x8U )
#define MCU_ADC_CSR_ADC1_JEOC1                ( 0x4U )
#define MCU_ADC_CSR_ADC1_EOC1        	      ( 0x2U )
#define MCU_ADC_CSR_ADC1_AWD1        	      ( 0x1U )

/* Биты регистра ADC_CCR (ADC common control register) */
#define MCU_ADC_CCR_BITS      			      ( ( uint32_t ) STM32F407_ADC_COMMON_PTR << 4U  )
#define MCU_ADC_CCR_TSVREFE 				  ( 0x800000U )
#define MCU_ADC_CCR_VBATE 				      ( 0x400000U )
#define MCU_ADC_CCR_ADCPRE_DIV_8       		  ( 0x30000U )
#define MCU_ADC_CCR_ADCPRE_DIV_6       		  ( 0x20000U )
#define MCU_ADC_CCR_ADCPRE_DIV_4       		  ( 0x10000U )
#define MCU_ADC_CCR_ADCPRE_DIV_2      		  ( 0x0U )
#define MCU_ADC_CCR_ADCPRE_MASK 		      ( 0x30000U )
#define MCU_ADC_CCR_DMA_EN4 				  ( 0xC000U )
#define MCU_ADC_CCR_DMA_EN3 				  ( 0x8000U )
#define MCU_ADC_CCR_DMA_EN2 				  ( 0x4000U )
#define MCU_ADC_CCR_DMA_EN1 				  ( 0x0U )
#define MCU_ADC_CCR_DMA_MASK 				  ( 0xC000U )
#define MCU_ADC_CCR_DDS 				      ( 0x2000U )
#define MCU_ADC_CCR_DELAY_20 				  ( 0xF00U )
#define MCU_ADC_CCR_DELAY_19 				  ( 0xE00U )
#define MCU_ADC_CCR_DELAY_18 				  ( 0xD00U )
#define MCU_ADC_CCR_DELAY_17 				  ( 0xC00U )
#define MCU_ADC_CCR_DELAY_16 				  ( 0xB00U )
#define MCU_ADC_CCR_DELAY_15 				  ( 0xA00U )
#define MCU_ADC_CCR_DELAY_14 				  ( 0x900U )
#define MCU_ADC_CCR_DELAY_13 				  ( 0x800U )
#define MCU_ADC_CCR_DELAY_12 				  ( 0x700U )
#define MCU_ADC_CCR_DELAY_11 				  ( 0x600U )
#define MCU_ADC_CCR_DELAY_10 				  ( 0x500U )
#define MCU_ADC_CCR_DELAY_9 				  ( 0x400U )
#define MCU_ADC_CCR_DELAY_8 				  ( 0x300U )
#define MCU_ADC_CCR_DELAY_7 				  ( 0x200U )
#define MCU_ADC_CCR_DELAY_6 				  ( 0x100U )
#define MCU_ADC_CCR_DELAY_5 				  ( 0x0U )
#define MCU_ADC_CCR_DELAY_MASK 				  ( 0xF00U )
#define MCU_ADC_CCR_MULTI_T_ALTER_TRIGGER 	  ( 0x19U )
#define MCU_ADC_CCR_MULTI_T_INTERLEAVED 	  ( 0x17U )
#define MCU_ADC_CCR_MULTI_T_REGUL_SIMUL 	  ( 0x16U )
#define MCU_ADC_CCR_MULTI_T_ONLY_INJEC 	      ( 0x15U )
#define MCU_ADC_CCR_MULTI_T_RESERVED 		  ( 0x13U )
#define MCU_ADC_CCR_MULTI_T_TRIG_MODE 		  ( 0x12U )
#define MCU_ADC_CCR_MULTI_T_INJEC_SIMUL 	  ( 0x11U )
#define MCU_ADC_CCR_MULTI_D_ALTER_TRIGGER 	  ( 0x9U )
#define MCU_ADC_CCR_MULTI_D_INTERLEAVED       ( 0x7U )
#define MCU_ADC_CCR_MULTI_D_REGUL_SIMUL 	  ( 0x6U )
#define MCU_ADC_CCR_MULTI_D_ONLY_INJEC 		  ( 0x5U )
#define MCU_ADC_CCR_MULTI_D_RESERVED 		  ( 0x3U )
#define MCU_ADC_CCR_MULTI_D_TRIG_MODE 		  ( 0x2U )
#define MCU_ADC_CCR_MULTI_D_INJEC_SIMUL       ( 0x1U )
#define MCU_ADC_CCR_MULTI_D_INDEP 		      ( 0x0U )
#define MCU_ADC_CCR_MULTI_MASK 				  ( 0x1FU )

/* Биты регистра ADC_CDR (ADC common regular data register for dual and triple modes) */
#define MCU_ADC_CDR_BITS      			      ( ( uint32_t ) STM32F407_ADC_COMMON_PTR << 8U  )
#define MCU_ADC_CDR_DATA2_MASK                ( 0xFFFF0000U )
#define MCU_ADC_CDR_DATA1_MASK                ( 0xFFFFU )

/* ----- DAC (Digital-to-analog converter) ----- */
typedef volatile struct tag_stm32f407_dac { 
    uint32_t        cr;
    uint32_t        swtrigr;
    uint32_t        dhr12r1;
    uint32_t        dhr12l1;
    uint32_t        dhr8r1;
    uint32_t        dhr12r2;
    uint32_t        dhr12l2;
    uint32_t        dhr8r2;
    uint32_t        dhr12rd;
    uint32_t        dhr12ld;
    uint32_t        dhr8rd;
    uint32_t        dor1;
    uint32_t        dor2;
    uint32_t        sr;
} stm32f407_dac_t;

#define STM32F407_DAC_PTR                     ( ( stm32f407_dac_t * ) 0x40007400U)

/* Биты регистра DAC_CR (DAC control register) */
#define MCU_DAC_CR_BITS      			      ( 0x3FFF3FFFU )
#define MCU_DAC_CR_DMAUDRIE2                  ( 0x20000000U )
#define MCU_DAC_CR_DMAEN2         	          ( 0x10000000U )
#define MCU_DAC_CR_MAMP2_EQ_4095              ( 0xB000B00U )
#define MCU_DAC_CR_MAMP2_EQ_2047              ( 0xA000000U )
#define MCU_DAC_CR_MAMP2_EQ_1023              ( 0x9000000U )
#define MCU_DAC_CR_MAMP2_EQ_511               ( 0x8000000U )
#define MCU_DAC_CR_MAMP2_EQ_255               ( 0x7000000U )
#define MCU_DAC_CR_MAMP2_EQ_127               ( 0x6000000U )
#define MCU_DAC_CR_MAMP2_EQ_63                ( 0x5000000U )
#define MCU_DAC_CR_MAMP2_EQ_31                ( 0x4000000U )
#define MCU_DAC_CR_MAMP2_EQ_15                ( 0x3000000U )
#define MCU_DAC_CR_MAMP2_EQ_7                 ( 0x2000000U )
#define MCU_DAC_CR_MAMP2_EQ_3                 ( 0x1000000U )
#define MCU_DAC_CR_MAMP2_EQ_1                 ( 0x0U )
#define MCU_DAC_CR_MAMP2_MASK                 ( 0xF000000U )
#define MCU_DAC_CR_WAVE2_MASK                 ( 0xC00000U )
#define MCU_DAC_CR_TSEL2_SOFT_TRIG            ( 0x380000U )
#define MCU_DAC_CR_TSEL2_EXT_LN9              ( 0x300000U )
#define MCU_DAC_CR_TSEL2_TIM4                 ( 0x280000U )
#define MCU_DAC_CR_TSEL2_TIM2                 ( 0x200000U )
#define MCU_DAC_CR_TSEL2_TIM5                 ( 0x180000U )
#define MCU_DAC_CR_TSEL2_TIM7                 ( 0x100000U )
#define MCU_DAC_CR_TSEL2_TIM8                 ( 0x80000U )
#define MCU_DAC_CR_TSEL2_TIM6                 ( 0x0U )
#define MCU_DAC_CR_TSEL2_MASK                 ( 0x380000U )
#define MCU_DAC_CR_TEN2          		      ( 0x40000U )
#define MCU_DAC_CR_BOFF2         		      ( 0x20000U )
#define MCU_DAC_CR_EN2         			      ( 0x10000U )
#define MCU_DAC_CR_DMAUDRIE1                  ( 0x2000U )
#define MCU_DAC_CR_DMAEN1         	          ( 0x1000U )
#define MCU_DAC_CR_MAMP1_EQ_4095              ( 0xB00U )
#define MCU_DAC_CR_MAMP1_EQ_2047              ( 0xA00U )
#define MCU_DAC_CR_MAMP1_EQ_1023              ( 0x900U )
#define MCU_DAC_CR_MAMP1_EQ_511               ( 0x800U )
#define MCU_DAC_CR_MAMP1_EQ_255               ( 0x700U )
#define MCU_DAC_CR_MAMP1_EQ_127               ( 0x600U )
#define MCU_DAC_CR_MAMP1_EQ_63                ( 0x500U )
#define MCU_DAC_CR_MAMP1_EQ_31                ( 0x400U )
#define MCU_DAC_CR_MAMP1_EQ_15                ( 0x300U )
#define MCU_DAC_CR_MAMP1_EQ_7                 ( 0x200U )
#define MCU_DAC_CR_MAMP1_EQ_3                 ( 0x100U )
#define MCU_DAC_CR_MAMP1_EQ_1                 ( 0x0U )
#define MCU_DAC_CR_MAMP1_MASK                 ( 0xF00U )
#define MCU_DAC_CR_WAVE1_MASK                 ( 0xC0U )
#define MCU_DAC_CR_TSEL1_SOFT_TRIG            ( 0x38U )
#define MCU_DAC_CR_TSEL1_EXT_LN9              ( 0x30U )
#define MCU_DAC_CR_TSEL1_TIM4                 ( 0x28U )
#define MCU_DAC_CR_TSEL1_TIM2                 ( 0x20U )
#define MCU_DAC_CR_TSEL1_TIM5                 ( 0x18U )
#define MCU_DAC_CR_TSEL1_TIM7                 ( 0x10U )
#define MCU_DAC_CR_TSEL1_TIM8                 ( 0x8U )
#define MCU_DAC_CR_TSEL1_TIM6                 ( 0x0U )
#define MCU_DAC_CR_TSEL1_MASK                 ( 0x38U )
#define MCU_DAC_CR_TEN1         		      ( 0x4U )
#define MCU_DAC_CR_BOFF1         		      ( 0x2U )
#define MCU_DAC_CR_EN1           		      ( 0x1U )

/* Биты регистра DAC_SWTRIGR (DAC software trigger register) */
#define MCU_DAC_SWTRIGR_BITS      	          ( 0x3U )
#define MCU_DAC_SWTRIGR_SWTRIG2               ( 0x2U )
#define MCU_DAC_SWTRIGR_SWTRIG1               ( 0x1U )

/* Биты регистра DAC_DHR12R1 (DAC channel_1 12-bit right-aligned data holding register) */
#define MCU_DAC_DHR12R1_BITS      	 		  ( 0xFFFU )
#define MCU_DAC_DHR12R1_DACC1DHR_MASK         ( 0xFFFU )

/* Биты регистра DAC_DHR12L1 (DAC channel_1 12-bit left-aligned data holding register) */
#define MCU_DAC_DHR12L1_BITS      	 	      ( 0xFFF0U )
#define MCU_DAC_DHR12L1_DACC1DHR_MASK         ( 0xFFF0U )

/* Биты регистра DAC_DHR8R1 (DAC channel_1 8-bit right-aligned data holding register) */
#define MCU_DAC_DHR8R1_BITS      	 		  ( 0xFFU )
#define MCU_DAC_DHR8R1_DACC1DHR_MASK          ( 0xFFU )

/* Биты регистра DAC_DHR12R2 (DAC channel_2 12-bit right-aligned data holding register) */
#define MCU_DAC_DHR12R2_BITS      	 		  ( 0xFFFU )
#define MCU_DAC_DHR12R2_DACC2DHR_MASK         ( 0xFFFU )

/* Биты регистра DAC_DHR12L2 (DAC channel_2 12-bit left-aligned data holding register) */
#define MCU_DAC_DHR12L2_BITS      	 		  ( 0xFFF0U )
#define MCU_DAC_DHR12L2_DACC2DHR_MASK         ( 0xFFF0U )

/* Биты регистра DAC_DHR8R2 (DAC channel_2 8-bit right-aligned data holding register) */
#define MCU_DAC_DHR8R2_BITS      	 		  ( 0xFFU )
#define MCU_DAC_DHR8R2_DACC2DHR_MASK          ( 0xFFU )

/* Биты регистра DAC_DHR12RD (dual DAC 12-bit right-aligned data holding register) */
#define MCU_DAC_DHR12RD_BITS      	 		  ( 0xFFF0FFFU )
#define MCU_DAC_DHR12RD_DACC2DHR_MASK         ( 0xFFF0000U )
#define MCU_DAC_DHR12RD_DACC1DHR_MASK         ( 0xFFFU )

/* Биты регистра DAC_DHR12LD (dual DAC 12-bit left-aligned data holding register) */
#define MCU_DAC_DHR12LD_BITS      	 	      ( 0xFFF0FFF0U )
#define MCU_DAC_DHR12LD_DACC2DHR_MASK         ( 0xFFF00000U )
#define MCU_DAC_DHR12LD_DACC1DHR_MASK         ( 0xFFF0U )

/* Биты регистра DAC_DHR8RD (dual DAC 8-bit right-aligned data holding register) */
#define MCU_DAC_DHR8RD_BITS      	 	      ( 0xFFFFU )
#define MCU_DAC_DHR8RD_DACC2DHR_MASK          ( 0xFF00U )
#define MCU_DAC_DHR8RD_DACC1DHR_MASK          ( 0xFFU )

/* Биты регистра DAC_DOR1 (DAC channel_1 data output register) */
#define MCU_DAC_DOR1_BITS      	 			  ( 0xFFFU )
#define MCU_DAC_DOR1_DACC1DOR_MASK            ( 0xFFFU )

/* Биты регистра DAC_DOR2 (DAC channel_2 data output register) */
#define MCU_DAC_DOR2_BITS      	 			  ( 0xFFFU )
#define MCU_DAC_DOR2_DACC2DOR_MASK            ( 0xFFFU )

/* Биты регистра DAC_SR (DAC status register) */
#define MCU_DAC_SR_BITS      	              ( 0x20002000U )
#define MCU_DAC_SR_DMAUDR2                    ( 0x20000000U )
#define MCU_DAC_SR_DMAUDR1                    ( 0x2000U )
/* ----- Контроллер Flash-памяти программ (FLASH) ----- */
typedef volatile struct tag_stm32f407_flash { 
    uint32_t        acr;
    uint32_t        keyr;
    uint32_t        optkeyr;
    uint32_t        sr;
    uint32_t        cr;
    uint32_t        optcr;
} stm32f407_flash_t;

#define STM32F407_FLASH_PTR                   ( ( stm32f407_flash_t * ) 0x40023C00U )

/* Биты регистра ACR (access control register) */
#define MCU_FLASH_ACR_BITS                    ( 0x1F07U )
#define MCU_FLASH_ACR_DCRST                   ( 0x1000U )
#define MCU_FLASH_ACR_ICRST                   ( 0x800U )
#define MCU_FLASH_ACR_DCEN                    ( 0x400U )
#define MCU_FLASH_ACR_ICEN                    ( 0x200U )
#define MCU_FLASH_ACR_PRFTEN                  ( 0x100U )
#define MCU_FLASH_ACR_LATENCY_MASK            ( 0x7U )

/* Биты регистра KEYR (key register) */
#define MCU_FLASH_KEYR_BITS                   ( 0xFFFFFFFFU )
#define MCU_FLASH_KEYR_KEY1                   ( 0x45670123U )
#define MCU_FLASH_KEYR_KEY2                   ( 0xCDEF89ABU )

/* Биты регистра OPTKEYR (option key register) */
#define MCU_FLASH_OPTKEYR_BITS                ( 0xFFFFFFFFU )
#define MCU_FLASH_OPTKEYR_KEY1                ( 0x08192A3BU )
#define MCU_FLASH_OPTKEYR_KEY2                ( 0x4C5D6E7FU )

/* Биты регистра SR (status register) */
#define MCU_FLASH_SR_BITS                     ( 0x100F3U )
#define MCU_FLASH_SR_BSY                      ( 0x10000U )
#define MCU_FLASH_SR_PGSERR                   ( 0x80U )
#define MCU_FLASH_SR_PGPERR                   ( 0x40U )
#define MCU_FLASH_SR_PGAERR                   ( 0x20U )
#define MCU_FLASH_SR_WRPERR                   ( 0x10U )
#define MCU_FLASH_SR_OPERR                    ( 0x2U )
#define MCU_FLASH_SR_EOP                      ( 0x1U )

/* Биты регистра CR (control register) */
#define MCU_FLASH_CR_BITS                     ( 0x8301037FU )
#define MCU_FLASH_CR_LOCK                     ( 0x80000000U )
#define MCU_FLASH_CR_ERRIE                    ( 0x2000000U )
#define MCU_FLASH_CR_EOPIE                    ( 0x1000000U )
#define MCU_FLASH_CR_STRT                     ( 0x10000U )
#define MCU_FLASH_CR_PSIZE_MASK               ( 0x300U )
#define MCU_FLASH_CR_SNB_MASK                 ( 0x78U )
#define MCU_FLASH_CR_SNB_11                   ( 0x58 )
#define MCU_FLASH_CR_SNB_10                   ( 0x50 )
#define MCU_FLASH_CR_SNB_9                    ( 0x48 )
#define MCU_FLASH_CR_SNB_8                    ( 0x40 )
#define MCU_FLASH_CR_SNB_7                    ( 0x38 )
#define MCU_FLASH_CR_SNB_6                    ( 0x30 )
#define MCU_FLASH_CR_SNB_5                    ( 0x28 )
#define MCU_FLASH_CR_SNB_4                    ( 0x20 )
#define MCU_FLASH_CR_SNB_3                    ( 0x18 )
#define MCU_FLASH_CR_SNB_2                    ( 0x10 )
#define MCU_FLASH_CR_SNB_1                    ( 0x08 )
#define MCU_FLASH_CR_SNB_0                    ( 0x00 )
#define MCU_FLASH_CR_MER                      ( 0x4U )
#define MCU_FLASH_CR_SER                      ( 0x2U )
#define MCU_FLASH_CR_PG                       ( 0x1U )

/* Биты регистра OTPCR (option control register) */
#define MCU_FLASH_OPTCR_BITS                  ( 0xFFFFFEFU )
#define MCU_FLASH_OPTCR_NWRP                  ( 0xFFF0000U )
#define MCU_FLASH_OPTCR_RDP                   ( 0xFF00U )
#define MCU_FLASH_OPTCR_nRST_STDBY            ( 0x80U )
#define MCU_FLASH_OPTCR_nRST_STOP             ( 0x40U )
#define MCU_FLASH_OPTCR_WDG_SW                ( 0x20U )
#define MCU_FLASH_OPTCR_BOR_LEV_MASK          ( 0xCU )
#define MCU_FLASH_OPTCR_OPTSTRT               ( 0x2U )
#define MCU_FLASH_OPTCR_OPTLOCK               ( 0x1U )


/* ----- CRC (cyclik redundancy check) ----- */
typedef volatile struct tag_stm32f407_crc { 
    uint32_t        dr;
    uint32_t        idr;
    uint32_t        cr;
} stm32f407_crc_t;

#define STM32F407_CRC_PTR                     ( ( stm32f407_crc_t * ) 0x40023000U )

/* Биты регистра DR (data register) */
#define MCU_CRC_DR_BITS                       ( 0xFFFFFFFFU )
#define MCU_CRC_DR_DR                         ( 0xFFFFFFFFU )

/* Биты регистра IDR (independent data register) */
#define MCU_CRC_IDR_BITS                      ( 0xFFU )
#define MCU_CRC_IDR_IDR                       ( 0xFFU )

/* Биты регистра CR (control register) */
#define MCU_CRC_CR_BITS                       ( 0x1U )
#define MCU_CRC_CR_RESET                      ( 0x1U )


/* ----- PWR (power controller) ----- */
typedef volatile struct tag_stm32f407_pwr { 
    uint32_t        cr;
    uint32_t        csr;
} stm32f407_pwr_t;

#define STM32F407_PWR_PTR                     ( ( stm32f407_pwr_t * ) 0x40007000U )

/* Биты регистра CR (control register) */
#define MCU_PWR_CR_BITS                       ( 0x43FFU )
#define MCU_PWR_CR_VOS                        ( 0x4000U )
#define MCU_PWR_CR_FPDS                       ( 0x200U )
#define MCU_PWR_CR_DBP                        ( 0x100U )
#define MCU_PWR_CR_PLS                        ( 0xE0U )
#define MCU_PWR_CR_PVDE                       ( 0x10U )
#define MCU_PWR_CR_CSBF                       ( 0x8U )
#define MCU_PWR_CR_CWUF                       ( 0x4U )
#define MCU_PWR_CR_PDDS                       ( 0x2U )
#define MCU_PWR_CR_LPDS                       ( 0x1U )

/* Биты регистра CSR (control/status register) */
#define MCU_PWR_CSR_BITS                      ( 0x430FU )
#define MCU_PWR_CSR_VOSRDY                    ( 0x4000U )
#define MCU_PWR_CSR_BRE                       ( 0x200U )
#define MCU_PWR_CSR_EWUP                      ( 0x100U )
#define MCU_PWR_CSR_BRR                       ( 0x8U )
#define MCU_PWR_CSR_PVDO                      ( 0x4U )
#define MCU_PWR_CSR_SBF                       ( 0x2U )
#define MCU_PWR_CSR_WUF                       ( 0x1U )


/* ----- RCC (reset and clock control) ----- */
typedef volatile struct tag_stm32f407_rcc {
    uint32_t        cr;
    uint32_t        pllcfgr;
    uint32_t        cfgr;
    uint32_t        cir;
    uint32_t        ahb1rstr;
    uint32_t        ahb2rstr;
    uint32_t        ahb3rstr;
    uint32_t        gap0;
    uint32_t        apb1rstr;
    uint32_t        apb2rstr;
    uint32_t        gap1[ 2 ];
    uint32_t        ahb1enr;
    uint32_t        ahb2enr;
    uint32_t        ahb3enr;
    uint32_t        gap2;
    uint32_t        apb1enr;
    uint32_t        apb2enr;
    uint32_t        gap3[ 2 ];
    uint32_t        ahb1lpenr;
    uint32_t        ahb2lpenr;
    uint32_t        ahb3lpenr;
    uint32_t        gap4;
    uint32_t        apb1lpenr;
    uint32_t        apb2lpenr;
    uint32_t        gap5[ 2 ];
    uint32_t        bdcr;
    uint32_t        csr;
    uint32_t        gap6[ 2 ];
    uint32_t        sscgr;
    uint32_t        plli2scfgr;
} stm32f407_rcc_t;

#define STM32F407_RCC_PTR                     ( ( stm32f407_rcc_t * ) 0x40023800U )

// Биты регистра CR (clock control register)
#define MCU_RCC_CR_BITS                       ( 0xF0FFFFBU )
#define MCU_RCC_CR_PLLI2SRDY                  ( 0x8000000U )
#define MCU_RCC_CR_PLLI2SON                   ( 0x4000000U )
#define MCU_RCC_CR_PLLRDY                     ( 0x2000000U )
#define MCU_RCC_CR_PLLON                      ( 0x1000000U )
#define MCU_RCC_CR_CSSON                      ( 0x80000U )
#define MCU_RCC_CR_HSEBYP                     ( 0x40000U )
#define MCU_RCC_CR_HSERDY                     ( 0x20000U )
#define MCU_RCC_CR_HSEON                      ( 0x10000U )
#define MCU_RCC_CR_HSICAL7                    ( 0x8000U )
#define MCU_RCC_CR_HSICAL6                    ( 0x4000U )
#define MCU_RCC_CR_HSICAL5                    ( 0x2000U )
#define MCU_RCC_CR_HSICAL4                    ( 0x1000U )
#define MCU_RCC_CR_HSICAL3                    ( 0x800U )
#define MCU_RCC_CR_HSICAL2                    ( 0x400U )
#define MCU_RCC_CR_HSICAL1                    ( 0x200U )
#define MCU_RCC_CR_HSICAL0                    ( 0x100U )
#define MCU_RCC_CR_HSITRIM4                   ( 0x80U )
#define MCU_RCC_CR_HSITRIM3                   ( 0x40U )
#define MCU_RCC_CR_HSITRIM2                   ( 0x20U )
#define MCU_RCC_CR_HSITRIM1                   ( 0x10U )
#define MCU_RCC_CR_HSITRIM0                   ( 0x8U )
#define MCU_RCC_CR_HSIRDY                     ( 0x2U )
#define MCU_RCC_CR_HSION                      ( 0x1U )

// Биты регистра PLLCFGR (PLL configuration register)
#define MCU_RCC_PLLCFGR_BITS                  ( 0xF437FFFU )
#define MCU_RCC_PLLCFGR_PLLQ_BITS             ( 0xF000000U )
#define MCU_RCC_PLLCFGR_PLLQ3                 ( 0x8000000U )
#define MCU_RCC_PLLCFGR_PLLQ2                 ( 0x4000000U )
#define MCU_RCC_PLLCFGR_PLLQ1                 ( 0x2000000U )
#define MCU_RCC_PLLCFGR_PLLQ0                 ( 0x1000000U )
#define MCU_RCC_PLLCFGR_PLLSRC                ( 0x400000U )
#define MCU_RCC_PLLCFGR_PLLP_BITS             ( 0x30000U )
#define MCU_RCC_PLLCFGR_PLLP1                 ( 0x20000U )
#define MCU_RCC_PLLCFGR_PLLP0                 ( 0x10000U )
#define MCU_RCC_PLLCFGR_PLLN_BITS             ( 0x7FC0U )
#define MCU_RCC_PLLCFGR_PLLN8                 ( 0x4000U )
#define MCU_RCC_PLLCFGR_PLLN7                 ( 0x2000U )
#define MCU_RCC_PLLCFGR_PLLN6                 ( 0x1000U )
#define MCU_RCC_PLLCFGR_PLLN5                 ( 0x800U )
#define MCU_RCC_PLLCFGR_PLLN4                 ( 0x400U )
#define MCU_RCC_PLLCFGR_PLLN3                 ( 0x200U )
#define MCU_RCC_PLLCFGR_PLLN2                 ( 0x100U )
#define MCU_RCC_PLLCFGR_PLLN1                 ( 0x80U )
#define MCU_RCC_PLLCFGR_PLLN0                 ( 0x40U )
#define MCU_RCC_PLLCFGR_PLLM_BITS             ( 0x3FU )
#define MCU_RCC_PLLCFGR_PLLM5                 ( 0x20U )
#define MCU_RCC_PLLCFGR_PLLM4                 ( 0x10U )
#define MCU_RCC_PLLCFGR_PLLM3                 ( 0x8U )
#define MCU_RCC_PLLCFGR_PLLM2                 ( 0x4U )
#define MCU_RCC_PLLCFGR_PLLM1                 ( 0x2U )
#define MCU_RCC_PLLCFGR_PLLM0                 ( 0x1U )

// Биты регистра CFGR (clock configuration register)
#define MCU_RCC_CFGR_BITS                     ( 0xFFFFFCFFU )
#define MCU_RCC_CFGR_MCO2_MASK                ( 0xC0000000U )
#define MCU_RCC_CFGR_MCO2_1                   ( 0x80000000U )
#define MCU_RCC_CFGR_MCO2_0                   ( 0x40000000U )
#define MCU_RCC_CFGR_MCO2PRE_MASK             ( 0x31000000U )
#define MCU_RCC_CFGR_MCO2PRE2                 ( 0x20000000U )
#define MCU_RCC_CFGR_MCO2PRE1                 ( 0x10000000U )
#define MCU_RCC_CFGR_MCO2PRE0                 ( 0x8000000U )
#define MCU_RCC_CFGR_MCO1PRE_MASK             ( 0x7000000U )
#define MCU_RCC_CFGR_MCO1PRE2                 ( 0x4000000U )
#define MCU_RCC_CFGR_MCO1PRE1                 ( 0x2000000U )
#define MCU_RCC_CFGR_MCO1PRE0                 ( 0x1000000U )
#define MCU_RCC_CFGR_I2SSRC                   ( 0x800000U )
#define MCU_RCC_CFGR_MCO1_MASK                ( 0x600000U )
#define MCU_RCC_CFGR_MCO1_1                   ( 0x400000U )
#define MCU_RCC_CFGR_MCO1_0                   ( 0x200000U )
#define MCU_RCC_CFGR_RTCPRE_MASK              ( 0x1F0000U )
#define MCU_RCC_CFGR_RTCPRE_4                 ( 0x100000U )
#define MCU_RCC_CFGR_RTCPRE_3                 ( 0x80000U )
#define MCU_RCC_CFGR_RTCPRE_2                 ( 0x40000U )
#define MCU_RCC_CFGR_RTCPRE_1                 ( 0x20000U )
#define MCU_RCC_CFGR_RTCPRE_0                 ( 0x10000U )
#define MCU_RCC_CFGR_PPRE2_2                  ( 0x8000U )
#define MCU_RCC_CFGR_PPRE2_1                  ( 0x4000U )
#define MCU_RCC_CFGR_PPRE2_0                  ( 0x2000U )
#define MCU_RCC_CFGR_PPRE1_2                  ( 0x1000U )
#define MCU_RCC_CFGR_PPRE1_1                  ( 0x800U )
#define MCU_RCC_CFGR_PPRE1_0                  ( 0x400U )
#define MCU_RCC_CFGR_HPRE_MASK     	          ( 0xF0U )
#define MCU_RCC_CFGR_HPRE_3                   ( 0x80U )
#define MCU_RCC_CFGR_HPRE_2                   ( 0x40U )
#define MCU_RCC_CFGR_HPRE_1                   ( 0x20U )
#define MCU_RCC_CFGR_HPRE_0                   ( 0x10U )
#define MCU_RCC_CFGR_SWS_MASK                 ( 0xCU )
#define MCU_RCC_CFGR_SWS_1                    ( 0x8U )
#define MCU_RCC_CFGR_SWS_0                    ( 0x4U )
#define MCU_RCC_CFGR_SW_MASK                  ( 0x3U )
#define MCU_RCC_CFGR_SW_1                     ( 0x2U )
#define MCU_RCC_CFGR_SW_0                     ( 0x1U )

// Биты регистра CIR (clock interrupt register)
#define MCU_RCC_CIR_BITS                      ( 0xBF3FBFU )
#define MCU_RCC_CIR_CSSC                      ( 0x800000U )
#define MCU_RCC_CIR_PLLI2SRDYC                ( 0x200000U )
#define MCU_RCC_CIR_PLLRDYC                   ( 0x100000U )
#define MCU_RCC_CIR_HSERDYC                   ( 0x80000U )
#define MCU_RCC_CIR_HSIRDYC                   ( 0x40000U )
#define MCU_RCC_CIR_LSERDYC                   ( 0x20000U )
#define MCU_RCC_CIR_LSIRDYC                   ( 0x10000U )
#define MCU_RCC_CIR_PLLI2SRDYIE               ( 0x2000U )
#define MCU_RCC_CIR_PLLRDYIE                  ( 0x1000U )
#define MCU_RCC_CIR_HSERDYIE                  ( 0x800U )
#define MCU_RCC_CIR_HSIRDYIE                  ( 0x400U )
#define MCU_RCC_CIR_LSERDYIE                  ( 0x200U )
#define MCU_RCC_CIR_LSIRDYIE                  ( 0x100U )
#define MCU_RCC_CIR_CSSF                      ( 0x80U )
#define MCU_RCC_CIR_PLLI2SRDYF                ( 0x20U )
#define MCU_RCC_CIR_PLLRDYF                   ( 0x10U )
#define MCU_RCC_CIR_HSERDYF                   ( 0x8U )
#define MCU_RCC_CIR_HSIRDYF                   ( 0x4U )
#define MCU_RCC_CIR_LSERDYF                   ( 0x2U )
#define MCU_RCC_CIR_LSIRDYF                   ( 0x1U )

// Биты регистра AHB1RSTR (AHB1 peripheral reset register)
#define MCU_RCC_AHB1RSTR_BITS                 ( 0x226011FFU )
#define MCU_RCC_AHB1RSTR_OTGHSRST             ( 0x20000000U )
#define MCU_RCC_AHB1RSTR_ETHMACRST            ( 0x2000000U )
#define MCU_RCC_AHB1RSTR_DMA2RST              ( 0x400000U )
#define MCU_RCC_AHB1RSTR_DMA1RST              ( 0x200000U )
#define MCU_RCC_AHB1RSTR_CRCRST               ( 0x1000U )
#define MCU_RCC_AHB1RSTR_GPIOIRST             ( 0x100U )
#define MCU_RCC_AHB1RSTR_GPIOHRST             ( 0x80U )
#define MCU_RCC_AHB1RSTR_GPIOGRST             ( 0x40U )
#define MCU_RCC_AHB1RSTR_GPIOFRST             ( 0x20U )
#define MCU_RCC_AHB1RSTR_GPIOERST             ( 0x10U )
#define MCU_RCC_AHB1RSTR_GPIODRST             ( 0x8U )
#define MCU_RCC_AHB1RSTR_GPIOCRST             ( 0x4U )
#define MCU_RCC_AHB1RSTR_GPIOBRST             ( 0x2U )
#define MCU_RCC_AHB1RSTR_GPIOARST             ( 0x1U )

// Биты регистра AHB2RSTR (AHB2 peripheral reset register)
#define MCU_RCC_AHB2RSTR_BITS                 ( 0xF1U )
#define MCU_RCC_AHB2RSTR_OTGFSRS              ( 0x80U )
#define MCU_RCC_AHB2RSTR_RNGRST               ( 0x40U )
#define MCU_RCC_AHB2RSTR_HSAHRST              ( 0x20U )
#define MCU_RCC_AHB2RSTR_CRYPRST              ( 0x10U )
#define MCU_RCC_AHB2RSTR_DCMIRST              ( 0x1U )

// Биты регистра AHB3RSTR (AHB3 peripheral reset register)
#define MCU_RCC_AHB3RSTR_BITS                 ( 0x1U )
#define MCU_RCC_AHB3RSTR_FSMCRST              ( 0x1U )

// Биты регистра APB1RSTR (APB1 peripheral reset register)
#define MCU_RCC_APB1RSTR_BITS                 ( 0x36FEC9FFU )
#define MCU_RCC_APB1RSTR_DACRST               ( 0x20000000U )
#define MCU_RCC_APB1RSTR_PWRRST               ( 0x10000000U )
#define MCU_RCC_APB1RSTR_CAN2RST              ( 0x4000000U )
#define MCU_RCC_APB1RSTR_CAN1RST              ( 0x2000000U )
#define MCU_RCC_APB1RSTR_I2C3RST              ( 0x800000U )
#define MCU_RCC_APB1RSTR_I2C2RST              ( 0x400000U )
#define MCU_RCC_APB1RSTR_I2C1RST              ( 0x200000U )
#define MCU_RCC_APB1RSTR_UART5RST             ( 0x100000U )
#define MCU_RCC_APB1RSTR_UART4RST             ( 0x80000U )
#define MCU_RCC_APB1RSTR_UART3RST             ( 0x40000U )
#define MCU_RCC_APB1RSTR_UART2RST             ( 0x20000U )
#define MCU_RCC_APB1RSTR_SPI3RST              ( 0x8000U )
#define MCU_RCC_APB1RSTR_SPI2RST              ( 0x4000U )
#define MCU_RCC_APB1RSTR_WWDGRST              ( 0x800U )
#define MCU_RCC_APB1RSTR_TIM14RST             ( 0x100U )
#define MCU_RCC_APB1RSTR_TIM13RST             ( 0x80U )
#define MCU_RCC_APB1RSTR_TIM12RST             ( 0x40U )
#define MCU_RCC_APB1RSTR_TIM7RST              ( 0x20U )
#define MCU_RCC_APB1RSTR_TIM6RST              ( 0x10U )
#define MCU_RCC_APB1RSTR_TIM5RST              ( 0x8U )
#define MCU_RCC_APB1RSTR_TIM4RST              ( 0x4U )
#define MCU_RCC_APB1RSTR_TIM3RST              ( 0x2U )
#define MCU_RCC_APB1RSTR_TIM2RST              ( 0x1U )

// Биты регистра APB2RSTR (APB2 peripheral reset register)
#define MCU_RCC_APB2RSTR_BITS                 ( 0x75933U )
#define MCU_RCC_APB2RSTR_TIM11RST             ( 0x40000U )
#define MCU_RCC_APB2RSTR_TIM10RST             ( 0x20000U )
#define MCU_RCC_APB2RSTR_TIM9RST              ( 0x10000U )
#define MCU_RCC_APB2RSTR_SYSCFGRST            ( 0x4000U )
#define MCU_RCC_APB2RSTR_SPI1RST              ( 0x1000U )
#define MCU_RCC_APB2RSTR_SDIORST              ( 0x800U )
#define MCU_RCC_APB2RSTR_ADCRST               ( 0x100U )
#define MCU_RCC_APB2RSTR_USART6RST            ( 0x20U )
#define MCU_RCC_APB2RSTR_USART1RST            ( 0x10U )
#define MCU_RCC_APB2RSTR_TIM8RST              ( 0x2U )
#define MCU_RCC_APB2RSTR_TIM1RST              ( 0x1U )

// Биты регистра AHB1ENR (AHB1 peripheral clock enable register)
#define MCU_RCC_AHB1ENR_BITS                  ( 0x7E7411FFU )
#define MCU_RCC_AHB1ENR_OTGHSULPIEN           ( 0x40000000U )
#define MCU_RCC_AHB1ENR_OTGHSEN               ( 0x20000000U )
#define MCU_RCC_AHB1ENR_ETHMACPTPEN           ( 0x10000000U )
#define MCU_RCC_AHB1ENR_ETHMACRXEN            ( 0x8000000U )
#define MCU_RCC_AHB1ENR_ETHMACTXEN            ( 0x4000000U )
#define MCU_RCC_AHB1ENR_ETHMACEN              ( 0x2000000U )
#define MCU_RCC_AHB1ENR_DMA2EN                ( 0x400000U )
#define MCU_RCC_AHB1ENR_DMA1EN                ( 0x200000U )
#define MCU_RCC_AHB1ENR_CCMDATARAMEN          ( 0x100000U )
#define MCU_RCC_AHB1ENR_BKPSRAMEN             ( 0x40000U )
#define MCU_RCC_AHB1ENR_CRCEN                 ( 0x1000U )
#define MCU_RCC_AHB1ENR_GPIOIEN               ( 0x100U )
#define MCU_RCC_AHB1ENR_GPIOHEN               ( 0x80U )
#define MCU_RCC_AHB1ENR_GPIOGEN               ( 0x40U )
#define MCU_RCC_AHB1ENR_GPIOFEN               ( 0x20U )
#define MCU_RCC_AHB1ENR_GPIOEEN               ( 0x10U )
#define MCU_RCC_AHB1ENR_GPIODEN               ( 0x8U )
#define MCU_RCC_AHB1ENR_GPIOCEN               ( 0x4U )
#define MCU_RCC_AHB1ENR_GPIOBEN               ( 0x2U )
#define MCU_RCC_AHB1ENR_GPIOAEN               ( 0x1U )

// Биты регистра AHB2ENR (AHB2 peripheral clock enable register)
#define MCU_RCC_AHB2ENR_BITS                  ( 0xF1U )
#define MCU_RCC_AHB2ENR_OTGFSEN               ( 0x80U )
#define MCU_RCC_AHB2ENR_RNGEN                 ( 0x40U )
#define MCU_RCC_AHB2ENR_HASHEN                ( 0x20U )
#define MCU_RCC_AHB2ENR_CRYPEN                ( 0x10U )
#define MCU_RCC_AHB2ENR_DCMIEN                ( 0x1U )

// Биты регистра AHB3ENR (AHB3 peripheral clock enable register)
#define MCU_RCC_AHB3ENR_BITS                  ( 0x1U )
#define MCU_RCC_AHB3ENR_FMCEN                 ( 0x1U )

// Биты регистра APB1ENR (APB1 peripheral clock enable register)
#define MCU_RCC_APB1ENR_BITS                  ( 0x36FEC9FFU )
#define MCU_RCC_APB1ENR_DACEN                 ( 0x20000000U )
#define MCU_RCC_APB1ENR_PWREN                 ( 0x10000000U )
#define MCU_RCC_APB1ENR_CAN2EN                ( 0x4000000U )
#define MCU_RCC_APB1ENR_CAN1EN                ( 0x2000000U )
#define MCU_RCC_APB1ENR_I2C3EN                ( 0x800000U )
#define MCU_RCC_APB1ENR_I2C2EN                ( 0x400000U )
#define MCU_RCC_APB1ENR_I2C1EN                ( 0x200000U )
#define MCU_RCC_APB1ENR_USART5EN              ( 0x100000U )
#define MCU_RCC_APB1ENR_USART4EN              ( 0x80000U )
#define MCU_RCC_APB1ENR_USART3EN              ( 0x40000U )
#define MCU_RCC_APB1ENR_USART2EN              ( 0x20000U )
#define MCU_RCC_APB1ENR_SPI3EN                ( 0x8000U )
#define MCU_RCC_APB1ENR_SPI2EN                ( 0x4000U )
#define MCU_RCC_APB1ENR_WWDGEN                ( 0x800U )
#define MCU_RCC_APB1ENR_TIM14EN               ( 0x100U )
#define MCU_RCC_APB1ENR_TIM13EN               ( 0x80U )
#define MCU_RCC_APB1ENR_TIM12EN               ( 0x40U )
#define MCU_RCC_APB1ENR_TIM7EN                ( 0x20U )
#define MCU_RCC_APB1ENR_TIM6EN                ( 0x10U )
#define MCU_RCC_APB1ENR_TIM5EN                ( 0x8U )
#define MCU_RCC_APB1ENR_TIM4EN                ( 0x4U )
#define MCU_RCC_APB1ENR_TIM3EN                ( 0x2U )
#define MCU_RCC_APB1ENR_TIM2EN                ( 0x1U )

// Биты регистра APB2ENR (APB2 peripheral clock enable register)
#define MCU_RCC_APB2ENR_BITS                  ( 0x75F33U )
#define MCU_RCC_APB2ENR_TIM11EN               ( 0x40000U )
#define MCU_RCC_APB2ENR_TIM10EN               ( 0x20000U )
#define MCU_RCC_APB2ENR_TIM9EN                ( 0x10000U )
#define MCU_RCC_APB2ENR_SYSCFGEN              ( 0x4000U )
#define MCU_RCC_APB2ENR_SPI1EN                ( 0x1000U )
#define MCU_RCC_APB2ENR_SDIOEN                ( 0x800U )
#define MCU_RCC_APB2ENR_ADC3EN                ( 0x400U )
#define MCU_RCC_APB2ENR_ADC2EN                ( 0x200U )
#define MCU_RCC_APB2ENR_ADC1EN                ( 0x100U )
#define MCU_RCC_APB2ENR_USART6EN              ( 0x20U )
#define MCU_RCC_APB2ENR_USART1EN              ( 0x10U )
#define MCU_RCC_APB2ENR_TIM8EN                ( 0x2U )
#define MCU_RCC_APB2ENR_TIM1EN                ( 0x1U )

// Биты регистра AHB1LPENR (AHB1  peripheral clock enable in low power mode register)
#define MCU_RCC_AHB1LPENR_BITS                ( 0x7E6791FFU )
#define MCU_RCC_AHB1LPENR_OTGHSULPILPEN       ( 0x40000000U )
#define MCU_RCC_AHB1LPENR_OTGHSLPEN           ( 0x20000000U )
#define MCU_RCC_AHB1LPENR_ETHMACPTPLPEN       ( 0x10000000U )
#define MCU_RCC_AHB1LPENR_ETHMACRXLPEN        ( 0x8000000U )
#define MCU_RCC_AHB1LPENR_ETHMACTXLPEN        ( 0x4000000U )
#define MCU_RCC_AHB1LPENR_ETHMACLPEN          ( 0x2000000U )
#define MCU_RCC_AHB1LPENR_DMA2LPEN            ( 0x400000U )
#define MCU_RCC_AHB1LPENR_DMA1LPEN            ( 0x200000U )
#define MCU_RCC_AHB1LPENR_BKPSRAMLPEN         ( 0x40000U )
#define MCU_RCC_AHB1LPENR_SRAM2LPEN           ( 0x20000U )
#define MCU_RCC_AHB1LPENR_SRAM1LPEN           ( 0x10000U )
#define MCU_RCC_AHB1LPENR_FLITFLPEN           ( 0x8000U )
#define MCU_RCC_AHB1LPENR_CRCLPEN             ( 0x1000U )
#define MCU_RCC_AHB1LPENR_GPIOILPEN           ( 0x100U )
#define MCU_RCC_AHB1LPENR_GPIOHLPEN           ( 0x80U )
#define MCU_RCC_AHB1LPENR_GPIOGLPEN           ( 0x40U )
#define MCU_RCC_AHB1LPENR_GPIOFLPEN           ( 0x20U )
#define MCU_RCC_AHB1LPENR_GPIOELPEN           ( 0x10U )
#define MCU_RCC_AHB1LPENR_GPIODLPEN           ( 0x8U )
#define MCU_RCC_AHB1LPENR_GPIOCLPEN           ( 0x4U )
#define MCU_RCC_AHB1LPENR_GPIOBLPEN           ( 0x2U )
#define MCU_RCC_AHB1LPENR_GPIOALPEN           ( 0x1U )

// Биты регистра AHB2LPENR (AHB2  peripheral clock enable in low power mode register)
#define MCU_RCC_AHB2LPENR_BITS                ( 0xF1U )
#define MCU_RCC_AHB2LPENR_OTGFSLPEN           ( 0x80U )
#define MCU_RCC_AHB2LPENR_RNGLPEN             ( 0x40U )
#define MCU_RCC_AHB2LPENR_HASHLPEN            ( 0x20U )
#define MCU_RCC_AHB2LPENR_CRYPLPEN            ( 0x10U )
#define MCU_RCC_AHB2LPENR_DCMILPEN            ( 0x1U )

// Биты регистра AHB3LPENR (AHB3  peripheral clock enable in low power mode register)
#define MCU_RCC_AHB3LPENR_BITS                ( 0x1U )
#define MCU_RCC_AHB3LPENR_FSMCLPEN            ( 0x1U )

// Биты регистра APB1LPENR (APB1  peripheral clock enable in low power mode register)
#define MCU_RCC_APB1LPENR_BITS                ( 0x36FEC9FFU )
#define MCU_RCC_APB1LPENR_DACLPEN             ( 0x20000000U )
#define MCU_RCC_APB1LPENR_PWRLPEN             ( 0x10000000U )
#define MCU_RCC_APB1LPENR_CAN2LPEN            ( 0x4000000U )
#define MCU_RCC_APB1LPENR_CAN1LPEN            ( 0x2000000U )
#define MCU_RCC_APB1LPENR_I2C3LPEN            ( 0x800000U )
#define MCU_RCC_APB1LPENR_I2C2LPEN            ( 0x400000U )
#define MCU_RCC_APB1LPENR_I2C1LPEN            ( 0x200000U )
#define MCU_RCC_APB1LPENR_USART5LPEN          ( 0x100000U )
#define MCU_RCC_APB1LPENR_USART4LPEN          ( 0x80000U )
#define MCU_RCC_APB1LPENR_USART3LPEN          ( 0x40000U )
#define MCU_RCC_APB1LPENR_USART2LPEN          ( 0x20000U )
#define MCU_RCC_APB1LPENR_SPI3LPEN            ( 0x8000U )
#define MCU_RCC_APB1LPENR_SPI2LPEN            ( 0x4000U )
#define MCU_RCC_APB1LPENR_WWDGLPEN            ( 0x800U )
#define MCU_RCC_APB1LPENR_TIM14LPEN           ( 0x100U )
#define MCU_RCC_APB1LPENR_TIM13LPEN           ( 0x80U )
#define MCU_RCC_APB1LPENR_TIM12LPEN           ( 0x40U )
#define MCU_RCC_APB1LPENR_TIM7LPEN            ( 0x20U )
#define MCU_RCC_APB1LPENR_TIM6LPEN            ( 0x10U )
#define MCU_RCC_APB1LPENR_TIM5LPEN            ( 0x8U )
#define MCU_RCC_APB1LPENR_TIM4LPEN            ( 0x4U )
#define MCU_RCC_APB1LPENR_TIM3LPEN            ( 0x2U )
#define MCU_RCC_APB1LPENR_TIM2LPEN            ( 0x1U )

// Биты регистра APB2LPENR (APB2  peripheral clock enable in low power mode register)
#define MCU_RCC_APB2LPENR_BITS                ( 0x75F33U )
#define MCU_RCC_APB2LPENR_TIM11LPEN           ( 0x40000U )
#define MCU_RCC_APB2LPENR_TIM10LPEN           ( 0x20000U )
#define MCU_RCC_APB2LPENR_TIM9LPEN            ( 0x10000U )
#define MCU_RCC_APB2LPENR_SYSCFGLPEN          ( 0x4000U )
#define MCU_RCC_APB2LPENR_SPI1LPEN            ( 0x1000U )
#define MCU_RCC_APB2LPENR_SDIOLPEN            ( 0x800U )
#define MCU_RCC_APB2LPENR_ADC3LPEN            ( 0x400U )
#define MCU_RCC_APB2LPENR_ADC2LPEN            ( 0x200U )
#define MCU_RCC_APB2LPENR_ADC1LPEN            ( 0x100U )
#define MCU_RCC_APB2LPENR_USART6LPEN          ( 0x20U )
#define MCU_RCC_APB2LPENR_USART1LPEN          ( 0x10U )
#define MCU_RCC_APB2LPENR_TIM8LPEN            ( 0x2U )
#define MCU_RCC_APB2LPENR_TIM1LPEN            ( 0x1U )

// Биты регистра MCU_RCC_BDCR (RCC Backup domain control register)
#define MCU_RCC_BDCR_BITS                     ( 0x18307U )
#define MCU_RCC_BDCR_BDRST                    ( 0x10000U )
#define MCU_RCC_BDCR_RTCEN                    ( 0x8000U )
#define MCU_RCC_BDCR_RTCSEL_MASK              ( 0x300U )
#define MCU_RCC_BDCR_LSEBYP                   ( 0x4U )
#define MCU_RCC_BDCR_LSERDY                   ( 0x2U )
#define MCU_RCC_BDCR_LSEON                    ( 0x1U )

// Биты регистра CSR (clock control & status register)
#define MCU_RCC_CSR_BITS                      ( 0xFF000003U )
#define MCU_RCC_CSR_LPWRRSTF                  ( 0x80000000U )
#define MCU_RCC_CSR_WWDGRSTF                  ( 0x40000000U )
#define MCU_RCC_CSR_IWDGRSTF                  ( 0x20000000U )
#define MCU_RCC_CSR_SFTRSTF                   ( 0x10000000U )
#define MCU_RCC_CSR_PORRSTF                   ( 0x8000000U )
#define MCU_RCC_CSR_PINRSTF                   ( 0x4000000U )
#define MCU_RCC_CSR_BORRSTF                   ( 0x2000000U )
#define MCU_RCC_CSR_RMVF                      ( 0x1000000U )
#define MCU_RCC_CSR_LSIRDY                    ( 0x2U )
#define MCU_RCC_CSR_LSION                     ( 0x1U )

// Биты регистра SSCGR (spread spectrum clock generation register)
#define MCU_RCC_SSCGR_BITS                    ( 0xCFFFFFFFU )
#define MCU_RCC_SSCGR_SSCGEN                  ( 0x80000000U )
#define MCU_RCC_SSCGR_SPREADSEL               ( 0x40000000U )
#define MCU_RCC_SSCGR_INCSTEP                 ( 0xFFFEU )
#define MCU_RCC_SSCGR_MODPER                  ( 0x1FFFU )

// Биты регистра PLLI2SCFGR (PLLI2S configuration register)
#define MCU_RCC_PLLI2SCFGR_BITS               ( 0x70007FC0U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SR2           ( 0x40000000U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SR1           ( 0x20000000U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SR0           ( 0x10000000U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SN8           ( 0x4000U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SN7           ( 0x2000U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SN6           ( 0x1000U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SN5           ( 0x800U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SN4           ( 0x400U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SN3           ( 0x200U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SN2           ( 0x100U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SN1           ( 0x80U )
#define MCU_RCC_PLLI2SCFGR_PLLI2SN0           ( 0x40U )


/* ----- Контроллер портов ввода/вывода GPIO ----- */
typedef volatile struct tag_stm32f407_gpio {
    uint32_t        moder;    
    uint32_t        otyper;  
    uint32_t        ospeedr;  
    uint32_t        pupdr;   
    uint32_t        idr;      
    uint32_t        odr;      
    uint32_t        bsrr;     
    uint32_t        lckr;    
    uint32_t        afrl;    
    uint32_t        afrh;    
} stm32f407_gpio_t;


#define STM32F407_GPIOA_PTR                   ( ( stm32f407_gpio_t * ) 0x40020000U )
#define STM32F407_GPIOB_PTR                   ( ( stm32f407_gpio_t * ) 0x40020400U )
#define STM32F407_GPIOC_PTR                   ( ( stm32f407_gpio_t * ) 0x40020800U )
#define STM32F407_GPIOD_PTR                   ( ( stm32f407_gpio_t * ) 0x40020C00U )
#define STM32F407_GPIOE_PTR                   ( ( stm32f407_gpio_t * ) 0x40021000U )
#define STM32F407_GPIOF_PTR                   ( ( stm32f407_gpio_t * ) 0x40021400U )
#define STM32F407_GPIOG_PTR                   ( ( stm32f407_gpio_t * ) 0x40021800U )
#define STM32F407_GPIOH_PTR                   ( ( stm32f407_gpio_t * ) 0x40021C00U )
#define STM32F407_GPIOI_PTR                   ( ( stm32f407_gpio_t * ) 0x40022000U )

// Биты регистра MODER (port mode register)
#define MCU_GPIO_MODER_BITS                   ( 0xFFFFFFFFU )
#define MCU_GPIO_MODER_15_MASK                ( 0xC0000000U )
#define MCU_GPIO_MODER_14_MASK                ( 0x30000000U )
#define MCU_GPIO_MODER_13_MASK                ( 0xC000000U )
#define MCU_GPIO_MODER_12_MASK                ( 0x3000000U )
#define MCU_GPIO_MODER_11_MASK                ( 0xC00000U )
#define MCU_GPIO_MODER_10_MASK                ( 0x300000U )
#define MCU_GPIO_MODER_9_MASK                 (	0xC0000U )
#define MCU_GPIO_MODER_8_MASK                 ( 0x30000U )
#define MCU_GPIO_MODER_7_MASK                 ( 0xC000U )
#define MCU_GPIO_MODER_6_MASK                 ( 0x3000U )
#define MCU_GPIO_MODER_5_MASK                 ( 0xC00U )
#define MCU_GPIO_MODER_4_MASK                 ( 0x300U )
#define MCU_GPIO_MODER_3_MASK                 ( 0xC0U )
#define MCU_GPIO_MODER_2_MASK                 ( 0x30U )
#define MCU_GPIO_MODER_1_MASK                 ( 0xCU )
#define MCU_GPIO_MODER_0_MASK                 ( 0x3U )

/* Вывод порта работает в режиме "Аналог" */
#define MCU_GPIO_MODER_ANALOG                 ( 0x3U )
#define MCU_GPIO_MODER_0_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 0U  )
#define MCU_GPIO_MODER_1_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 2U  )
#define MCU_GPIO_MODER_2_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 4U  )
#define MCU_GPIO_MODER_3_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 6U  )
#define MCU_GPIO_MODER_4_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 8U  )
#define MCU_GPIO_MODER_5_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 10U )
#define MCU_GPIO_MODER_6_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 12U )
#define MCU_GPIO_MODER_7_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 14U )
#define MCU_GPIO_MODER_8_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 16U )
#define MCU_GPIO_MODER_9_ANALOG               ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 18U )
#define MCU_GPIO_MODER_10_ANALOG              ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 20U )
#define MCU_GPIO_MODER_11_ANALOG              ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 22U )
#define MCU_GPIO_MODER_12_ANALOG              ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 24U )
#define MCU_GPIO_MODER_13_ANALOG              ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 26U )
#define MCU_GPIO_MODER_14_ANALOG              ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 28U )
#define MCU_GPIO_MODER_15_ANALOG              ( ( uint32_t ) MCU_GPIO_MODER_ANALOG << 30U )

/* Вывод порта работает в режиме "альтернативня функция */
#define MCU_GPIO_MODER_ALT                    ( 0x2U )
#define MCU_GPIO_MODER_0_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 0U  )
#define MCU_GPIO_MODER_1_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 2U  )
#define MCU_GPIO_MODER_2_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 4U  )
#define MCU_GPIO_MODER_3_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 6U  )
#define MCU_GPIO_MODER_4_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 8U  )
#define MCU_GPIO_MODER_5_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 10U )
#define MCU_GPIO_MODER_6_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 12U )
#define MCU_GPIO_MODER_7_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 14U )
#define MCU_GPIO_MODER_8_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 16U )
#define MCU_GPIO_MODER_9_ALT                  ( ( uint32_t ) MCU_GPIO_MODER_ALT << 18U )
#define MCU_GPIO_MODER_10_ALT                 ( ( uint32_t ) MCU_GPIO_MODER_ALT << 20U )
#define MCU_GPIO_MODER_11_ALT                 ( ( uint32_t ) MCU_GPIO_MODER_ALT << 22U )
#define MCU_GPIO_MODER_12_ALT                 ( ( uint32_t ) MCU_GPIO_MODER_ALT << 24U )
#define MCU_GPIO_MODER_13_ALT                 ( ( uint32_t ) MCU_GPIO_MODER_ALT << 26U )
#define MCU_GPIO_MODER_14_ALT                 ( ( uint32_t ) MCU_GPIO_MODER_ALT << 28U )
#define MCU_GPIO_MODER_15_ALT                 ( ( uint32_t ) MCU_GPIO_MODER_ALT << 30U )

/* Вывод порта работает в режиме "порт вывода*/
#define MCU_GPIO_MODER_OUT                    ( 0x1U )
#define MCU_GPIO_MODER_0_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 0U  )
#define MCU_GPIO_MODER_1_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 2U  )
#define MCU_GPIO_MODER_2_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 4U  )
#define MCU_GPIO_MODER_3_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 6U  )
#define MCU_GPIO_MODER_4_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 8U  )
#define MCU_GPIO_MODER_5_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 10U )
#define MCU_GPIO_MODER_6_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 12U )
#define MCU_GPIO_MODER_7_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 14U )
#define MCU_GPIO_MODER_8_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 16U )
#define MCU_GPIO_MODER_9_OUT                  ( ( uint32_t ) MCU_GPIO_MODER_OUT << 18U )
#define MCU_GPIO_MODER_10_OUT                 ( ( uint32_t ) MCU_GPIO_MODER_OUT << 20U )
#define MCU_GPIO_MODER_11_OUT                 ( ( uint32_t ) MCU_GPIO_MODER_OUT << 22U )
#define MCU_GPIO_MODER_12_OUT                 ( ( uint32_t ) MCU_GPIO_MODER_OUT << 24U )
#define MCU_GPIO_MODER_13_OUT                 ( ( uint32_t ) MCU_GPIO_MODER_OUT << 26U )
#define MCU_GPIO_MODER_14_OUT                 ( ( uint32_t ) MCU_GPIO_MODER_OUT << 28U )
#define MCU_GPIO_MODER_15_OUT                 ( ( uint32_t ) MCU_GPIO_MODER_OUT << 30U )

/* Вывод порта работает в режиме "порт ввода" */
#define MCU_GPIO_MODER_IN                     ( 0x0U )
#define MCU_GPIO_MODER_0_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 0U  )
#define MCU_GPIO_MODER_1_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 2U  )
#define MCU_GPIO_MODER_2_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 4U  )
#define MCU_GPIO_MODER_3_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 6U  )
#define MCU_GPIO_MODER_4_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 8U  )
#define MCU_GPIO_MODER_5_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 10U )
#define MCU_GPIO_MODER_6_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 12U )
#define MCU_GPIO_MODER_7_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 14U )
#define MCU_GPIO_MODER_8_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 16U )
#define MCU_GPIO_MODER_9_IN                   ( ( uint32_t ) MCU_GPIO_MODER_IN << 18U )
#define MCU_GPIO_MODER_10_IN                  ( ( uint32_t ) MCU_GPIO_MODER_IN << 20U )
#define MCU_GPIO_MODER_11_IN                  ( ( uint32_t ) MCU_GPIO_MODER_IN << 22U )
#define MCU_GPIO_MODER_12_IN                  ( ( uint32_t ) MCU_GPIO_MODER_IN << 24U )
#define MCU_GPIO_MODER_13_IN                  ( ( uint32_t ) MCU_GPIO_MODER_IN << 26U )
#define MCU_GPIO_MODER_14_IN                  ( ( uint32_t ) MCU_GPIO_MODER_IN << 28U )
#define MCU_GPIO_MODER_15_IN                  ( ( uint32_t ) MCU_GPIO_MODER_IN << 30U )

// Биты регистра OTYPER (port output type register)
#define MCU_GPIO_OTYPER_BITS                  ( 0xFFFFU )
#define MCU_GPIO_OTYPER_OT15                  ( 0x8000U )
#define MCU_GPIO_OTYPER_OT14                  ( 0x4000U )
#define MCU_GPIO_OTYPER_OT13                  ( 0x2000U )
#define MCU_GPIO_OTYPER_OT12                  ( 0x1000U )
#define MCU_GPIO_OTYPER_OT11                  ( 0x800U )
#define MCU_GPIO_OTYPER_OT10                  ( 0x400U )
#define MCU_GPIO_OTYPER_OT9                   ( 0x200U )
#define MCU_GPIO_OTYPER_OT8                   ( 0x100U )
#define MCU_GPIO_OTYPER_OT7                   ( 0x80U )
#define MCU_GPIO_OTYPER_OT6                   ( 0x40U )
#define MCU_GPIO_OTYPER_OT5                   ( 0x20U )
#define MCU_GPIO_OTYPER_OT4                   ( 0x10U )
#define MCU_GPIO_OTYPER_OT3                   ( 0x8U )
#define MCU_GPIO_OTYPER_OT2                   ( 0x4U )
#define MCU_GPIO_OTYPER_OT1                   ( 0x2U )
#define MCU_GPIO_OTYPER_OT0                   ( 0x1U )

/* Вывод порта работает в режиме "open-drain" */
#define MCU_GPIO_OTYPER_0_OP_DR               MCU_GPIO_OTYPER_OT0
#define MCU_GPIO_OTYPER_1_OP_DR               MCU_GPIO_OTYPER_OT1
#define MCU_GPIO_OTYPER_2_OP_DR               MCU_GPIO_OTYPER_OT2
#define MCU_GPIO_OTYPER_3_OP_DR               MCU_GPIO_OTYPER_OT3
#define MCU_GPIO_OTYPER_4_OP_DR               MCU_GPIO_OTYPER_OT4
#define MCU_GPIO_OTYPER_5_OP_DR               MCU_GPIO_OTYPER_OT5
#define MCU_GPIO_OTYPER_6_OP_DR               MCU_GPIO_OTYPER_OT6
#define MCU_GPIO_OTYPER_7_OP_DR               MCU_GPIO_OTYPER_OT7
#define MCU_GPIO_OTYPER_8_OP_DR               MCU_GPIO_OTYPER_OT8
#define MCU_GPIO_OTYPER_9_OP_DR               MCU_GPIO_OTYPER_OT9
#define MCU_GPIO_OTYPER_10_OP_DR              MCU_GPIO_OTYPER_OT10
#define MCU_GPIO_OTYPER_11_OP_DR              MCU_GPIO_OTYPER_OT11
#define MCU_GPIO_OTYPER_12_OP_DR              MCU_GPIO_OTYPER_OT12
#define MCU_GPIO_OTYPER_13_OP_DR              MCU_GPIO_OTYPER_OT13
#define MCU_GPIO_OTYPER_14_OP_DR              MCU_GPIO_OTYPER_OT14
#define MCU_GPIO_OTYPER_15_OP_DR              MCU_GPIO_OTYPER_OT15

/* Вывод порта работает в режиме "push-pull" */
#define MCU_GPIO_OTYPER_0_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_1_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_2_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_3_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_4_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_5_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_6_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_7_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_8_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_9_PUSH_PULL           ( 0U )
#define MCU_GPIO_OTYPER_10_PUSH_PULL          ( 0U )
#define MCU_GPIO_OTYPER_11_PUSH_PULL          ( 0U )
#define MCU_GPIO_OTYPER_12_PUSH_PULL          ( 0U )
#define MCU_GPIO_OTYPER_13_PUSH_PULL          ( 0U )
#define MCU_GPIO_OTYPER_14_PUSH_PULL          ( 0U )
#define MCU_GPIO_OTYPER_15_PUSH_PULL          ( 0U )

// Биты регистра OSPEEDR (port output speed register)
#define MCU_GPIO_OSPEEDR_BITS                 ( 0xFFFFFFFFU )
#define MCU_GPIO_OSPEEDR_15_MASK              ( 0xC0000000U )
#define MCU_GPIO_OSPEEDR_14_MASK              ( 0x30000000U )
#define MCU_GPIO_OSPEEDR_13_MASK              ( 0xC000000U )
#define MCU_GPIO_OSPEEDR_12_MASK              ( 0x3000000U )
#define MCU_GPIO_OSPEEDR_11_MASK              ( 0xC00000U )
#define MCU_GPIO_OSPEEDR_10_MASK              ( 0x300000U )
#define MCU_GPIO_OSPEEDR_9_MASK               ( 0xC0000U )
#define MCU_GPIO_OSPEEDR_8_MASK               ( 0x30000U )
#define MCU_GPIO_OSPEEDR_7_MASK               ( 0xC000U )
#define MCU_GPIO_OSPEEDR_6_MASK               ( 0x3000U )
#define MCU_GPIO_OSPEEDR_5_MASK               ( 0xC00U )
#define MCU_GPIO_OSPEEDR_4_MASK               ( 0x300U )
#define MCU_GPIO_OSPEEDR_3_MASK               ( 0xC0U )
#define MCU_GPIO_OSPEEDR_2_MASK               ( 0x30U )
#define MCU_GPIO_OSPEEDR_1_MASK               ( 0xCU )
#define MCU_GPIO_OSPEEDR_0_MASK               ( 0x3U )

/* Вывод порта работает в режиме "максимально быстрый фронт" */
#define MCU_GPIO_OSPEEDR_MAX                  ( 0x3U )
#define MCU_GPIO_OSPEEDR_0_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 0U  )
#define MCU_GPIO_OSPEEDR_1_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 2U  )
#define MCU_GPIO_OSPEEDR_2_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 4U  )
#define MCU_GPIO_OSPEEDR_3_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 6U  )
#define MCU_GPIO_OSPEEDR_4_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 8U  )
#define MCU_GPIO_OSPEEDR_5_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 10U )
#define MCU_GPIO_OSPEEDR_6_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 12U )
#define MCU_GPIO_OSPEEDR_7_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 14U )
#define MCU_GPIO_OSPEEDR_8_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 16U )
#define MCU_GPIO_OSPEEDR_9_MAX                ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 18U )
#define MCU_GPIO_OSPEEDR_10_MAX               ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 20U )
#define MCU_GPIO_OSPEEDR_11_MAX               ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 22U )
#define MCU_GPIO_OSPEEDR_12_MAX               ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 24U )
#define MCU_GPIO_OSPEEDR_13_MAX               ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 26U )
#define MCU_GPIO_OSPEEDR_14_MAX               ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 28U )
#define MCU_GPIO_OSPEEDR_15_MAX               ( ( uint32_t ) MCU_GPIO_OSPEEDR_MAX << 30U )

/* Вывод порта работает в режиме "быстрый фронт" */
#define MCU_GPIO_OSPEEDR_FAST                 ( 0x2U )
#define MCU_GPIO_OSPEEDR_0_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 0U  )
#define MCU_GPIO_OSPEEDR_1_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 2U  )
#define MCU_GPIO_OSPEEDR_2_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 4U  )
#define MCU_GPIO_OSPEEDR_3_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 6U  )
#define MCU_GPIO_OSPEEDR_4_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 8U  )
#define MCU_GPIO_OSPEEDR_5_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 10U )
#define MCU_GPIO_OSPEEDR_6_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 12U )
#define MCU_GPIO_OSPEEDR_7_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 14U )
#define MCU_GPIO_OSPEEDR_8_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 16U )
#define MCU_GPIO_OSPEEDR_9_FAST               ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 18U )
#define MCU_GPIO_OSPEEDR_10_FAST              ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 20U )
#define MCU_GPIO_OSPEEDR_11_FAST              ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 22U )
#define MCU_GPIO_OSPEEDR_12_FAST              ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 24U )
#define MCU_GPIO_OSPEEDR_13_FAST              ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 26U )
#define MCU_GPIO_OSPEEDR_14_FAST              ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 28U )
#define MCU_GPIO_OSPEEDR_15_FAST              ( ( uint32_t ) MCU_GPIO_OSPEEDR_FAST << 30U )

/* Вывод порта работает в режиме "средний фронт" */
#define MCU_GPIO_OSPEEDR_MEDIUM               ( 0x1U )
#define MCU_GPIO_OSPEEDR_0_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 0U  )
#define MCU_GPIO_OSPEEDR_1_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 2U  )
#define MCU_GPIO_OSPEEDR_2_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 4U  )
#define MCU_GPIO_OSPEEDR_3_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 6U  )
#define MCU_GPIO_OSPEEDR_4_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 8U  )
#define MCU_GPIO_OSPEEDR_5_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 10U )
#define MCU_GPIO_OSPEEDR_6_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 12U )
#define MCU_GPIO_OSPEEDR_7_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 14U )
#define MCU_GPIO_OSPEEDR_8_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 16U )
#define MCU_GPIO_OSPEEDR_9_MEDIUM             ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 18U )
#define MCU_GPIO_OSPEEDR_10_MEDIUM            ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 20U )
#define MCU_GPIO_OSPEEDR_11_MEDIUM            ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 22U )
#define MCU_GPIO_OSPEEDR_12_MEDIUM            ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 24U )
#define MCU_GPIO_OSPEEDR_13_MEDIUM            ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 26U )
#define MCU_GPIO_OSPEEDR_14_MEDIUM            ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 28U )
#define MCU_GPIO_OSPEEDR_15_MEDIUM            ( ( uint32_t ) MCU_GPIO_OSPEEDR_MEDIUM << 30U )

/* Вывод порта работает в режиме "медленный фронт" */
#define MCU_GPIO_OSPEEDR_SLOW                 ( 0x0U )
#define MCU_GPIO_OSPEEDR_0_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 0U  )
#define MCU_GPIO_OSPEEDR_1_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 2U  )
#define MCU_GPIO_OSPEEDR_2_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 4U  )
#define MCU_GPIO_OSPEEDR_3_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 6U  )
#define MCU_GPIO_OSPEEDR_4_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 8U  )
#define MCU_GPIO_OSPEEDR_5_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 10U )
#define MCU_GPIO_OSPEEDR_6_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 12U )
#define MCU_GPIO_OSPEEDR_7_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 14U )
#define MCU_GPIO_OSPEEDR_8_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 16U )
#define MCU_GPIO_OSPEEDR_9_SLOW               ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 18U )
#define MCU_GPIO_OSPEEDR_10_SLOW              ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 20U )
#define MCU_GPIO_OSPEEDR_11_SLOW              ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 22U )
#define MCU_GPIO_OSPEEDR_12_SLOW              ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 24U )
#define MCU_GPIO_OSPEEDR_13_SLOW              ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 26U )
#define MCU_GPIO_OSPEEDR_14_SLOW              ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 28U )
#define MCU_GPIO_OSPEEDR_15_SLOW              ( ( uint32_t ) MCU_GPIO_OSPEEDR_SLOW << 30U )

// Биты регистра PUPDR (port pull-up/pull-down register)
#define MCU_GPIO_PUPDR_BITS                   ( 0xFFFFFFFFU )
#define MCU_GPIO_PUPDR_PUPDR15_MASK           ( 0xC0000000U )
#define MCU_GPIO_PUPDR_PUPDR14_MASK           ( 0x30000000U )
#define MCU_GPIO_PUPDR_PUPDR13_MASK           ( 0xC000000U )
#define MCU_GPIO_PUPDR_PUPDR12_MASK           ( 0x3000000U )
#define MCU_GPIO_PUPDR_PUPDR11_MASK           ( 0xC00000U )
#define MCU_GPIO_PUPDR_PUPDR10_MASK           ( 0x300000U )
#define MCU_GPIO_PUPDR_PUPDR9_MASK            ( 0xC0000U )
#define MCU_GPIO_PUPDR_PUPDR8_MASK            ( 0x30000U )
#define MCU_GPIO_PUPDR_PUPDR7_MASK            ( 0xC000U )
#define MCU_GPIO_PUPDR_PUPDR6_MASK            ( 0x3000U )
#define MCU_GPIO_PUPDR_PUPDR5_MASK            ( 0xC00U )
#define MCU_GPIO_PUPDR_PUPDR4_MASK            ( 0x300U )
#define MCU_GPIO_PUPDR_PUPDR3_MASK            ( 0xC0U )
#define MCU_GPIO_PUPDR_PUPDR2_MASK            ( 0x30U )
#define MCU_GPIO_PUPDR_PUPDR1_MASK            ( 0xCU )
#define MCU_GPIO_PUPDR_PUPDR0_MASK            ( 0x3U )

/* Вывод порта работает в режиме "pull-down" */
#define MCU_GPIO_PUPDR_DOWN                   ( 0x2U )
#define MCU_GPIO_PUPDR_0_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 0U  )
#define MCU_GPIO_PUPDR_1_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 2U  )
#define MCU_GPIO_PUPDR_2_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 4U  )
#define MCU_GPIO_PUPDR_3_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 6U  )
#define MCU_GPIO_PUPDR_4_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 8U  )
#define MCU_GPIO_PUPDR_5_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 10U )
#define MCU_GPIO_PUPDR_6_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 12U )
#define MCU_GPIO_PUPDR_7_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 14U )
#define MCU_GPIO_PUPDR_8_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 16U )
#define MCU_GPIO_PUPDR_9_DOWN                 ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 18U )
#define MCU_GPIO_PUPDR_10_DOWN                ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 20U )
#define MCU_GPIO_PUPDR_11_DOWN                ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 22U )
#define MCU_GPIO_PUPDR_12_DOWN                ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 24U )
#define MCU_GPIO_PUPDR_13_DOWN                ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 26U )
#define MCU_GPIO_PUPDR_14_DOWN                ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 28U )
#define MCU_GPIO_PUPDR_15_DOWN                ( ( uint32_t ) MCU_GPIO_PUPDR_DOWN << 30U )

/* Вывод порта работает в режиме "pull-up" */
#define MCU_GPIO_PUPDR_UP                     ( 0x1U )
#define MCU_GPIO_PUPDR_0_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 0U  )
#define MCU_GPIO_PUPDR_1_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 2U  )
#define MCU_GPIO_PUPDR_2_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 4U  )
#define MCU_GPIO_PUPDR_3_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 6U  )
#define MCU_GPIO_PUPDR_4_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 8U  )
#define MCU_GPIO_PUPDR_5_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 10U )
#define MCU_GPIO_PUPDR_6_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 12U )
#define MCU_GPIO_PUPDR_7_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 14U )
#define MCU_GPIO_PUPDR_8_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 16U )
#define MCU_GPIO_PUPDR_9_UP                   ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 18U )
#define MCU_GPIO_PUPDR_10_UP                  ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 20U )
#define MCU_GPIO_PUPDR_11_UP                  ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 22U )
#define MCU_GPIO_PUPDR_12_UP                  ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 24U )
#define MCU_GPIO_PUPDR_13_UP                  ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 26U )
#define MCU_GPIO_PUPDR_14_UP                  ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 28U )
#define MCU_GPIO_PUPDR_15_UP                  ( ( uint32_t ) MCU_GPIO_PUPDR_UP << 30U )

/* Вывод порта работает в режиме "no pull-up, pull-down" */
#define MCU_GPIO_PUPDR_NO                     ( 0U )
#define MCU_GPIO_PUPDR_0_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 0U  )
#define MCU_GPIO_PUPDR_1_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 2U  )
#define MCU_GPIO_PUPDR_2_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 4U  )
#define MCU_GPIO_PUPDR_3_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 6U  )
#define MCU_GPIO_PUPDR_4_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 8U  )
#define MCU_GPIO_PUPDR_5_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 10U )
#define MCU_GPIO_PUPDR_6_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 12U )
#define MCU_GPIO_PUPDR_7_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 14U )
#define MCU_GPIO_PUPDR_8_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 16U )
#define MCU_GPIO_PUPDR_9_NO                   ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 18U )
#define MCU_GPIO_PUPDR_10_NO                  ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 20U )
#define MCU_GPIO_PUPDR_11_NO                  ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 22U )
#define MCU_GPIO_PUPDR_12_NO                  ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 24U )
#define MCU_GPIO_PUPDR_13_NO                  ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 28U )
#define MCU_GPIO_PUPDR_15_NO                  ( ( uint32_t ) MCU_GPIO_PUPDR_NO << 30U )

// Биты регистра IDR (port input data register)
#define MCU_GPIO_IDR_BITS                     ( 0x0000FFFFU )
#define MCU_GPIO_IDR_IDR15                    ( 0x8000U )
#define MCU_GPIO_IDR_IDR14                    ( 0x4000U )
#define MCU_GPIO_IDR_IDR13                    ( 0x2000U )
#define MCU_GPIO_IDR_IDR12                    ( 0x1000U )
#define MCU_GPIO_IDR_IDR11                    ( 0x800U )
#define MCU_GPIO_IDR_IDR10                    ( 0x400U )
#define MCU_GPIO_IDR_IDR9                     ( 0x200U )
#define MCU_GPIO_IDR_IDR8                     ( 0x100U )
#define MCU_GPIO_IDR_IDR7                     ( 0x80U )
#define MCU_GPIO_IDR_IDR6                     ( 0x40U )
#define MCU_GPIO_IDR_IDR5                     ( 0x20U )
#define MCU_GPIO_IDR_IDR4                     ( 0x10U )
#define MCU_GPIO_IDR_IDR3                     ( 0x8U )
#define MCU_GPIO_IDR_IDR2                     ( 0x4U )
#define MCU_GPIO_IDR_IDR1                     ( 0x2U )
#define MCU_GPIO_IDR_IDR0                     ( 0x1U )

// Биты регистра ODR (port output data register)
#define MCU_GPIO_ODR_BITS                     ( 0x0000FFFFU )
#define MCU_GPIO_ODR_ODR15                    ( 0x8000U )
#define MCU_GPIO_ODR_ODR14                    ( 0x4000U )
#define MCU_GPIO_ODR_ODR13                    ( 0x2000U )
#define MCU_GPIO_ODR_ODR12                    ( 0x1000U )
#define MCU_GPIO_ODR_ODR11                    ( 0x800U )
#define MCU_GPIO_ODR_ODR10                    ( 0x400U )
#define MCU_GPIO_ODR_ODR9                     ( 0x200U )
#define MCU_GPIO_ODR_ODR8                     ( 0x100U )
#define MCU_GPIO_ODR_ODR7                     ( 0x80U )
#define MCU_GPIO_ODR_ODR6                     ( 0x40U )
#define MCU_GPIO_ODR_ODR5                     ( 0x20U )
#define MCU_GPIO_ODR_ODR4                     ( 0x10U )
#define MCU_GPIO_ODR_ODR3                     ( 0x8U )
#define MCU_GPIO_ODR_ODR2                     ( 0x4U )
#define MCU_GPIO_ODR_ODR1                     ( 0x2U )
#define MCU_GPIO_ODR_ODR0                     ( 0x1U )

// Биты регистра BSRR (port bit set/reset register)
#define MCU_GPIO_BSRR_BITS                    ( 0xFFFFFFFFU )
#define MCU_GPIO_BSRR_RES_15                  ( 0x80000000U )
#define MCU_GPIO_BSRR_RES_14                  ( 0x40000000U )
#define MCU_GPIO_BSRR_RES_13                  ( 0x20000000U )
#define MCU_GPIO_BSRR_RES_12                  ( 0x10000000U )
#define MCU_GPIO_BSRR_RES_11                  ( 0x8000000U )
#define MCU_GPIO_BSRR_RES_10                  ( 0x4000000U )
#define MCU_GPIO_BSRR_RES_9                   ( 0x2000000U )
#define MCU_GPIO_BSRR_RES_8                   ( 0x1000000U )
#define MCU_GPIO_BSRR_RES_7                   ( 0x800000U )
#define MCU_GPIO_BSRR_RES_6                   ( 0x400000U )
#define MCU_GPIO_BSRR_RES_5                   ( 0x200000U )
#define MCU_GPIO_BSRR_RES_4                   ( 0x100000U )
#define MCU_GPIO_BSRR_RES_3                   ( 0x80000U )
#define MCU_GPIO_BSRR_RES_2                   ( 0x40000U )
#define MCU_GPIO_BSRR_RES_1                   ( 0x20000U )
#define MCU_GPIO_BSRR_RES_0                   ( 0x10000U )
#define MCU_GPIO_BSRR_SET_15                  ( 0x8000U )
#define MCU_GPIO_BSRR_SET_14                  ( 0x4000U )
#define MCU_GPIO_BSRR_SET_13                  ( 0x2000U )
#define MCU_GPIO_BSRR_SET_12                  ( 0x1000U )
#define MCU_GPIO_BSRR_SET_11                  ( 0x800U )
#define MCU_GPIO_BSRR_SET_10                  ( 0x400U )
#define MCU_GPIO_BSRR_SET_9                   ( 0x200U )
#define MCU_GPIO_BSRR_SET_8                   ( 0x100U )
#define MCU_GPIO_BSRR_SET_7                   ( 0x80U )
#define MCU_GPIO_BSRR_SET_6                   ( 0x40U )
#define MCU_GPIO_BSRR_SET_5                   ( 0x20U )
#define MCU_GPIO_BSRR_SET_4                   ( 0x10U )
#define MCU_GPIO_BSRR_SET_3                   ( 0x8U )
#define MCU_GPIO_BSRR_SET_2                   ( 0x4U )
#define MCU_GPIO_BSRR_SET_1                   ( 0x2U )
#define MCU_GPIO_BSRR_SET_0                   ( 0x1U )

// Биты регистра LCKR (port configuration lock register)
#define MCU_GPIO_LCKR_BITS                    ( 0x1FFFFU )
#define MCU_GPIO_LCKR_LCKK                    (	0x10000U )
#define MCU_GPIO_LCKR_LCK15                   (	0x8000U )
#define MCU_GPIO_LCKR_LCK14                   ( 0x4000U )
#define MCU_GPIO_LCKR_LCK13                   ( 0x2000U )
#define MCU_GPIO_LCKR_LCK12                   ( 0x1000U )
#define MCU_GPIO_LCKR_LCK11                   ( 0x800U )
#define MCU_GPIO_LCKR_LCK10                   ( 0x400U )
#define MCU_GPIO_LCKR_LCK9                    ( 0x200U )
#define MCU_GPIO_LCKR_LCK8                    ( 0x100U )
#define MCU_GPIO_LCKR_LCK7                    ( 0x80U )
#define MCU_GPIO_LCKR_LCK6                    ( 0x40U )
#define MCU_GPIO_LCKR_LCK5                    ( 0x20U )
#define MCU_GPIO_LCKR_LCK4                    ( 0x10U )
#define MCU_GPIO_LCKR_LCK3                    ( 0x8U )
#define MCU_GPIO_LCKR_LCK2                    ( 0x4U )
#define MCU_GPIO_LCKR_LCK1                    ( 0x2U )
#define MCU_GPIO_LCKR_LCK0                    ( 0x1U )

/* Вывод порта "Port configuration locked" */
#define MCU_GPIO_LCKR_0_LOCK                  MCU_GPIO_LCKR_0
#define MCU_GPIO_LCKR_1_LOCK                  MCU_GPIO_LCKR_1
#define MCU_GPIO_LCKR_2_LOCK                  MCU_GPIO_LCKR_2
#define MCU_GPIO_LCKR_3_LOCK                  MCU_GPIO_LCKR_3
#define MCU_GPIO_LCKR_4_LOCK                  MCU_GPIO_LCKR_4
#define MCU_GPIO_LCKR_5_LOCK                  MCU_GPIO_LCKR_5
#define MCU_GPIO_LCKR_6_LOCK                  MCU_GPIO_LCKR_6
#define MCU_GPIO_LCKR_7_LOCK                  MCU_GPIO_LCKR_7
#define MCU_GPIO_LCKR_8_LOCK                  MCU_GPIO_LCKR_8
#define MCU_GPIO_LCKR_9_LOCK                  MCU_GPIO_LCKR_9
#define MCU_GPIO_LCKR_10_LOCK                 MCU_GPIO_LCKR_10
#define MCU_GPIO_LCKR_11_LOCK                 MCU_GPIO_LCKR_11
#define MCU_GPIO_LCKR_12_LOCK                 MCU_GPIO_LCKR_12
#define MCU_GPIO_LCKR_13_LOCK                 MCU_GPIO_LCKR_13
#define MCU_GPIO_LCKR_14_LOCK                 MCU_GPIO_LCKR_14
#define MCU_GPIO_LCKR_15_LOCK                 MCU_GPIO_LCKR_15

/* Вывод порта "Port configuration not locked" */
#define MCU_GPIO_LCKR_0_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_1_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_2_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_3_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_4_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_5_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_6_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_7_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_8_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_9_UNLOCK                ( 0U )
#define MCU_GPIO_LCKR_10_UNLOCK               ( 0U )
#define MCU_GPIO_LCKR_11_UNLOCK               ( 0U )
#define MCU_GPIO_LCKR_12_UNLOCK               ( 0U )
#define MCU_GPIO_LCKR_13_UNLOCK               ( 0U )
#define MCU_GPIO_LCKR_14_UNLOCK               ( 0U )
#define MCU_GPIO_LCKR_15_UNLOCK               ( 0U )

// Биты регистра AFRL (alternate function low register)
#define MCU_GPIO_AFRL_BITS                    ( 0xFFFFFFFFU )
#define MCU_GPIO_AFRL_7_MASK                  ( 0xF0000000U )
#define MCU_GPIO_AFRL_6_MASK                  ( 0xF000000U )
#define MCU_GPIO_AFRL_5_MASK                  ( 0xF00000U )
#define MCU_GPIO_AFRL_4_MASK                  ( 0xF0000U )
#define MCU_GPIO_AFRL_3_MASK                  ( 0xF000U )
#define MCU_GPIO_AFRL_2_MASK                  ( 0xF00U )
#define MCU_GPIO_AFRL_1_MASK                  ( 0xF0U )
#define MCU_GPIO_AFRL_0_MASK                  ( 0xFU )

/* Вывод порта альтернативная функция "AF15" */
#define MCU_GPIO_AFRL_AF15                    ( 0xFU )
#define MCU_GPIO_AFRL_0_AF15                  ( ( uint32_t ) MCU_GPIO_AFRL_AF15 << 0U  )
#define MCU_GPIO_AFRL_1_AF15                  ( ( uint32_t ) MCU_GPIO_AFRL_AF15 << 4U  )
#define MCU_GPIO_AFRL_2_AF15                  ( ( uint32_t ) MCU_GPIO_AFRL_AF15 << 8U  )
#define MCU_GPIO_AFRL_3_AF15                  ( ( uint32_t ) MCU_GPIO_AFRL_AF15 << 12U )
#define MCU_GPIO_AFRL_4_AF15                  ( ( uint32_t ) MCU_GPIO_AFRL_AF15 << 16U )
#define MCU_GPIO_AFRL_5_AF15                  ( ( uint32_t ) MCU_GPIO_AFRL_AF15 << 20U )
#define MCU_GPIO_AFRL_6_AF15                  ( ( uint32_t ) MCU_GPIO_AFRL_AF15 << 24U )
#define MCU_GPIO_AFRL_7_AF15                  ( ( uint32_t ) MCU_GPIO_AFRL_AF15 << 28U )

/* Вывод порта альтернативная функция "AF14" */
#define MCU_GPIO_AFRL_AF14                    ( 0xEU )
#define MCU_GPIO_AFRL_0_AF14                  ( ( uint32_t ) MCU_GPIO_AFRL_AF14 << 0U  )
#define MCU_GPIO_AFRL_1_AF14                  ( ( uint32_t ) MCU_GPIO_AFRL_AF14 << 4U  )
#define MCU_GPIO_AFRL_2_AF14                  ( ( uint32_t ) MCU_GPIO_AFRL_AF14 << 8U  )
#define MCU_GPIO_AFRL_3_AF14                  ( ( uint32_t ) MCU_GPIO_AFRL_AF14 << 12U )
#define MCU_GPIO_AFRL_4_AF14                  ( ( uint32_t ) MCU_GPIO_AFRL_AF14 << 16U )
#define MCU_GPIO_AFRL_5_AF14                  ( ( uint32_t ) MCU_GPIO_AFRL_AF14 << 20U )
#define MCU_GPIO_AFRL_6_AF14                  ( ( uint32_t ) MCU_GPIO_AFRL_AF14 << 24U )
#define MCU_GPIO_AFRL_7_AF14                  ( ( uint32_t ) MCU_GPIO_AFRL_AF14 << 28U )

/* Вывод порта альтернативная функция "AF13" */
#define MCU_GPIO_AFRL_AF13                    ( 0xDU )
#define MCU_GPIO_AFRL_0_AF13                  ( ( uint32_t ) MCU_GPIO_AFRL_AF13 << 0U  )
#define MCU_GPIO_AFRL_1_AF13                  ( ( uint32_t ) MCU_GPIO_AFRL_AF13 << 4U  )
#define MCU_GPIO_AFRL_2_AF13                  ( ( uint32_t ) MCU_GPIO_AFRL_AF13 << 8U  )
#define MCU_GPIO_AFRL_3_AF13                  ( ( uint32_t ) MCU_GPIO_AFRL_AF13 << 12U )
#define MCU_GPIO_AFRL_4_AF13                  ( ( uint32_t ) MCU_GPIO_AFRL_AF13 << 16U )
#define MCU_GPIO_AFRL_5_AF13                  ( ( uint32_t ) MCU_GPIO_AFRL_AF13 << 20U )
#define MCU_GPIO_AFRL_6_AF13                  ( ( uint32_t ) MCU_GPIO_AFRL_AF13 << 24U )
#define MCU_GPIO_AFRL_7_AF13                  ( ( uint32_t ) MCU_GPIO_AFRL_AF13 << 28U )

/* Вывод порта альтернативная функция "AF12" */
#define MCU_GPIO_AFRL_AF12                    ( 0xCU )
#define MCU_GPIO_AFRL_0_AF12                  ( ( uint32_t ) MCU_GPIO_AFRL_AF12 << 0U  )
#define MCU_GPIO_AFRL_1_AF12                  ( ( uint32_t ) MCU_GPIO_AFRL_AF12 << 4U  )
#define MCU_GPIO_AFRL_2_AF12                  ( ( uint32_t ) MCU_GPIO_AFRL_AF12 << 8U  )
#define MCU_GPIO_AFRL_3_AF12                  ( ( uint32_t ) MCU_GPIO_AFRL_AF12 << 12U )
#define MCU_GPIO_AFRL_4_AF12                  ( ( uint32_t ) MCU_GPIO_AFRL_AF12 << 16U )
#define MCU_GPIO_AFRL_5_AF12                  ( ( uint32_t ) MCU_GPIO_AFRL_AF12 << 20U )
#define MCU_GPIO_AFRL_6_AF12                  ( ( uint32_t ) MCU_GPIO_AFRL_AF12 << 24U )
#define MCU_GPIO_AFRL_7_AF12                  ( ( uint32_t ) MCU_GPIO_AFRL_AF12 << 28U )

/* Вывод порта альтернативная функция "AF11" */
#define MCU_GPIO_AFRL_AF11                    ( 0xBU )
#define MCU_GPIO_AFRL_0_AF11                  ( ( uint32_t ) MCU_GPIO_AFRL_AF11 << 0U  )
#define MCU_GPIO_AFRL_1_AF11                  ( ( uint32_t ) MCU_GPIO_AFRL_AF11 << 4U  )
#define MCU_GPIO_AFRL_2_AF11                  ( ( uint32_t ) MCU_GPIO_AFRL_AF11 << 8U  )
#define MCU_GPIO_AFRL_3_AF11                  ( ( uint32_t ) MCU_GPIO_AFRL_AF11 << 12U )
#define MCU_GPIO_AFRL_4_AF11                  ( ( uint32_t ) MCU_GPIO_AFRL_AF11 << 16U )
#define MCU_GPIO_AFRL_5_AF11                  ( ( uint32_t ) MCU_GPIO_AFRL_AF11 << 20U )
#define MCU_GPIO_AFRL_6_AF11                  ( ( uint32_t ) MCU_GPIO_AFRL_AF11 << 24U )
#define MCU_GPIO_AFRL_7_AF11                  ( ( uint32_t ) MCU_GPIO_AFRL_AF11 << 28U )

/* Вывод порта альтернативная функция "AF10" */
#define MCU_GPIO_AFRL_AF10                    ( 0xAU )
#define MCU_GPIO_AFRL_0_AF10                  ( ( uint32_t ) MCU_GPIO_AFRL_AF10 << 0U  )
#define MCU_GPIO_AFRL_1_AF10                  ( ( uint32_t ) MCU_GPIO_AFRL_AF10 << 4U  )
#define MCU_GPIO_AFRL_2_AF10                  ( ( uint32_t ) MCU_GPIO_AFRL_AF10 << 8U  )
#define MCU_GPIO_AFRL_3_AF10                  ( ( uint32_t ) MCU_GPIO_AFRL_AF10 << 12U )
#define MCU_GPIO_AFRL_4_AF10                  ( ( uint32_t ) MCU_GPIO_AFRL_AF10 << 16U )
#define MCU_GPIO_AFRL_5_AF10                  ( ( uint32_t ) MCU_GPIO_AFRL_AF10 << 20U )
#define MCU_GPIO_AFRL_6_AF10                  ( ( uint32_t ) MCU_GPIO_AFRL_AF10 << 24U )
#define MCU_GPIO_AFRL_7_AF10                  ( ( uint32_t ) MCU_GPIO_AFRL_AF10 << 28U )

/* Вывод порта альтернативная функция "AF9" */
#define MCU_GPIO_AFRL_AF9                     ( 0x9U )
#define MCU_GPIO_AFRL_0_AF9                   ( ( uint32_t ) MCU_GPIO_AFRL_AF9 << 0U  )
#define MCU_GPIO_AFRL_1_AF9                   ( ( uint32_t ) MCU_GPIO_AFRL_AF9 << 4U  )
#define MCU_GPIO_AFRL_2_AF9                   ( ( uint32_t ) MCU_GPIO_AFRL_AF9 << 8U  )
#define MCU_GPIO_AFRL_3_AF9                   ( ( uint32_t ) MCU_GPIO_AFRL_AF9 << 12U )
#define MCU_GPIO_AFRL_4_AF9                   ( ( uint32_t ) MCU_GPIO_AFRL_AF9 << 16U )
#define MCU_GPIO_AFRL_5_AF9                   ( ( uint32_t ) MCU_GPIO_AFRL_AF9 << 20U )
#define MCU_GPIO_AFRL_6_AF9                   ( ( uint32_t ) MCU_GPIO_AFRL_AF9 << 24U )
#define MCU_GPIO_AFRL_7_AF9                   ( ( uint32_t ) MCU_GPIO_AFRL_AF9 << 28U )

/* Вывод порта альтернативная функция "AF8" */
#define MCU_GPIO_AFRL_AF8                     ( 0x8U )
#define MCU_GPIO_AFRL_0_AF8                   ( ( uint32_t ) MCU_GPIO_AFRL_AF8 << 0U  )
#define MCU_GPIO_AFRL_1_AF8                   ( ( uint32_t ) MCU_GPIO_AFRL_AF8 << 4U  )
#define MCU_GPIO_AFRL_2_AF8                   ( ( uint32_t ) MCU_GPIO_AFRL_AF8 << 8U  )
#define MCU_GPIO_AFRL_3_AF8                   ( ( uint32_t ) MCU_GPIO_AFRL_AF8 << 12U )
#define MCU_GPIO_AFRL_4_AF8                   ( ( uint32_t ) MCU_GPIO_AFRL_AF8 << 16U )
#define MCU_GPIO_AFRL_5_AF8                   ( ( uint32_t ) MCU_GPIO_AFRL_AF8 << 20U )
#define MCU_GPIO_AFRL_6_AF8                   ( ( uint32_t ) MCU_GPIO_AFRL_AF8 << 24U )
#define MCU_GPIO_AFRL_7_AF8                   ( ( uint32_t ) MCU_GPIO_AFRL_AF8 << 28U )

/* Вывод порта альтернативная функция "AF7" */
#define MCU_GPIO_AFRL_AF7                     ( 0x7U )
#define MCU_GPIO_AFRL_0_AF7                   ( ( uint32_t ) MCU_GPIO_AFRL_AF7 << 0U  )
#define MCU_GPIO_AFRL_1_AF7                   ( ( uint32_t ) MCU_GPIO_AFRL_AF7 << 4U  )
#define MCU_GPIO_AFRL_2_AF7                   ( ( uint32_t ) MCU_GPIO_AFRL_AF7 << 8U  )
#define MCU_GPIO_AFRL_3_AF7                   ( ( uint32_t ) MCU_GPIO_AFRL_AF7 << 12U )
#define MCU_GPIO_AFRL_4_AF7                   ( ( uint32_t ) MCU_GPIO_AFRL_AF7 << 16U )
#define MCU_GPIO_AFRL_5_AF7                   ( ( uint32_t ) MCU_GPIO_AFRL_AF7 << 20U )
#define MCU_GPIO_AFRL_6_AF7                   ( ( uint32_t ) MCU_GPIO_AFRL_AF7 << 24U )
#define MCU_GPIO_AFRL_7_AF7                   ( ( uint32_t ) MCU_GPIO_AFRL_AF7 << 28U )

/* Вывод порта альтернативная функция "AF6" */
#define MCU_GPIO_AFRL_AF6                     ( 0x6U )
#define MCU_GPIO_AFRL_0_AF6                   ( ( uint32_t ) MCU_GPIO_AFRL_AF6 << 0U  )
#define MCU_GPIO_AFRL_1_AF6                   ( ( uint32_t ) MCU_GPIO_AFRL_AF6 << 4U  )
#define MCU_GPIO_AFRL_2_AF6                   ( ( uint32_t ) MCU_GPIO_AFRL_AF6 << 8U  )
#define MCU_GPIO_AFRL_3_AF6                   ( ( uint32_t ) MCU_GPIO_AFRL_AF6 << 12U )
#define MCU_GPIO_AFRL_4_AF6                   ( ( uint32_t ) MCU_GPIO_AFRL_AF6 << 16U )
#define MCU_GPIO_AFRL_5_AF6                   ( ( uint32_t ) MCU_GPIO_AFRL_AF6 << 20U )
#define MCU_GPIO_AFRL_6_AF6                   ( ( uint32_t ) MCU_GPIO_AFRL_AF6 << 24U )
#define MCU_GPIO_AFRL_7_AF6                   ( ( uint32_t ) MCU_GPIO_AFRL_AF6 << 28U )

/* Вывод порта альтернативная функция "AF5" */
#define MCU_GPIO_AFRL_AF5                     ( 0x5U )
#define MCU_GPIO_AFRL_0_AF5                   ( ( uint32_t ) MCU_GPIO_AFRL_AF5 << 0U  )
#define MCU_GPIO_AFRL_1_AF5                   ( ( uint32_t ) MCU_GPIO_AFRL_AF5 << 4U  )
#define MCU_GPIO_AFRL_2_AF5                   ( ( uint32_t ) MCU_GPIO_AFRL_AF5 << 8U  )
#define MCU_GPIO_AFRL_3_AF5                   ( ( uint32_t ) MCU_GPIO_AFRL_AF5 << 12U )
#define MCU_GPIO_AFRL_4_AF5                   ( ( uint32_t ) MCU_GPIO_AFRL_AF5 << 16U )
#define MCU_GPIO_AFRL_5_AF5                   ( ( uint32_t ) MCU_GPIO_AFRL_AF5 << 20U )
#define MCU_GPIO_AFRL_6_AF5                   ( ( uint32_t ) MCU_GPIO_AFRL_AF5 << 24U )
#define MCU_GPIO_AFRL_7_AF5                   ( ( uint32_t ) MCU_GPIO_AFRL_AF5 << 28U )

/* Вывод порта альтернативная функция "AF4" */
#define MCU_GPIO_AFRL_AF4                     ( 0x4U )
#define MCU_GPIO_AFRL_0_AF4                   ( ( uint32_t ) MCU_GPIO_AFRL_AF4 << 0U  )
#define MCU_GPIO_AFRL_1_AF4                   ( ( uint32_t ) MCU_GPIO_AFRL_AF4 << 4U  )
#define MCU_GPIO_AFRL_2_AF4                   ( ( uint32_t ) MCU_GPIO_AFRL_AF4 << 8U  )
#define MCU_GPIO_AFRL_3_AF4                   ( ( uint32_t ) MCU_GPIO_AFRL_AF4 << 12U )
#define MCU_GPIO_AFRL_4_AF4                   ( ( uint32_t ) MCU_GPIO_AFRL_AF4 << 16U )
#define MCU_GPIO_AFRL_5_AF4                   ( ( uint32_t ) MCU_GPIO_AFRL_AF4 << 20U )
#define MCU_GPIO_AFRL_6_AF4                   ( ( uint32_t ) MCU_GPIO_AFRL_AF4 << 24U )
#define MCU_GPIO_AFRL_7_AF4                   ( ( uint32_t ) MCU_GPIO_AFRL_AF4 << 28U )

/* Вывод порта альтернативная функция "AF3" */
#define MCU_GPIO_AFRL_AF3                     ( 0x3U )
#define MCU_GPIO_AFRL_0_AF3                   ( ( uint32_t ) MCU_GPIO_AFRL_AF3 << 0U  )
#define MCU_GPIO_AFRL_1_AF3                   ( ( uint32_t ) MCU_GPIO_AFRL_AF3 << 4U  )
#define MCU_GPIO_AFRL_2_AF3                   ( ( uint32_t ) MCU_GPIO_AFRL_AF3 << 8U  )
#define MCU_GPIO_AFRL_3_AF3                   ( ( uint32_t ) MCU_GPIO_AFRL_AF3 << 12U )
#define MCU_GPIO_AFRL_4_AF3                   ( ( uint32_t ) MCU_GPIO_AFRL_AF3 << 16U )
#define MCU_GPIO_AFRL_5_AF3                   ( ( uint32_t ) MCU_GPIO_AFRL_AF3 << 20U )
#define MCU_GPIO_AFRL_6_AF3                   ( ( uint32_t ) MCU_GPIO_AFRL_AF3 << 24U )
#define MCU_GPIO_AFRL_7_AF3                   ( ( uint32_t ) MCU_GPIO_AFRL_AF3 << 28U )

/* Вывод порта альтернативная функция "AF2" */
#define MCU_GPIO_AFRL_AF2                     ( 0x2U )
#define MCU_GPIO_AFRL_0_AF2                   ( ( uint32_t ) MCU_GPIO_AFRL_AF2 << 0U  )
#define MCU_GPIO_AFRL_1_AF2                   ( ( uint32_t ) MCU_GPIO_AFRL_AF2 << 4U  )
#define MCU_GPIO_AFRL_2_AF2                   ( ( uint32_t ) MCU_GPIO_AFRL_AF2 << 8U  )
#define MCU_GPIO_AFRL_3_AF2                   ( ( uint32_t ) MCU_GPIO_AFRL_AF2 << 12U )
#define MCU_GPIO_AFRL_4_AF2                   ( ( uint32_t ) MCU_GPIO_AFRL_AF2 << 16U )
#define MCU_GPIO_AFRL_5_AF2                   ( ( uint32_t ) MCU_GPIO_AFRL_AF2 << 20U )
#define MCU_GPIO_AFRL_6_AF2                   ( ( uint32_t ) MCU_GPIO_AFRL_AF2 << 24U )
#define MCU_GPIO_AFRL_7_AF2                   ( ( uint32_t ) MCU_GPIO_AFRL_AF2 << 28U )

/* Вывод порта альтернативная функция "AF1" */
#define MCU_GPIO_AFRL_AF1                     ( 0x1U )
#define MCU_GPIO_AFRL_0_AF1                   ( ( uint32_t ) MCU_GPIO_AFRL_AF1 << 0U  )
#define MCU_GPIO_AFRL_1_AF1                   ( ( uint32_t ) MCU_GPIO_AFRL_AF1 << 4U  )
#define MCU_GPIO_AFRL_2_AF1                   ( ( uint32_t ) MCU_GPIO_AFRL_AF1 << 8U  )
#define MCU_GPIO_AFRL_3_AF1                   ( ( uint32_t ) MCU_GPIO_AFRL_AF1 << 12U )
#define MCU_GPIO_AFRL_4_AF1                   ( ( uint32_t ) MCU_GPIO_AFRL_AF1 << 16U )
#define MCU_GPIO_AFRL_5_AF1                   ( ( uint32_t ) MCU_GPIO_AFRL_AF1 << 20U )
#define MCU_GPIO_AFRL_6_AF1                   ( ( uint32_t ) MCU_GPIO_AFRL_AF1 << 24U )
#define MCU_GPIO_AFRL_7_AF1                   ( ( uint32_t ) MCU_GPIO_AFRL_AF1 << 28U )

/* Вывод порта альтернативная функция "AF0" */
#define MCU_GPIO_AFRL_AF0                     ( 0x0U )
#define MCU_GPIO_AFRL_0_AF0                   ( ( uint32_t ) MCU_GPIO_AFRL_AF0 << 0U  )
#define MCU_GPIO_AFRL_1_AF0                   ( ( uint32_t ) MCU_GPIO_AFRL_AF0 << 4U  )
#define MCU_GPIO_AFRL_2_AF0                   ( ( uint32_t ) MCU_GPIO_AFRL_AF0 << 8U  )
#define MCU_GPIO_AFRL_3_AF0                   ( ( uint32_t ) MCU_GPIO_AFRL_AF0 << 12U )
#define MCU_GPIO_AFRL_4_AF0                   ( ( uint32_t ) MCU_GPIO_AFRL_AF0 << 16U )
#define MCU_GPIO_AFRL_5_AF0                   ( ( uint32_t ) MCU_GPIO_AFRL_AF0 << 20U )
#define MCU_GPIO_AFRL_6_AF0                   ( ( uint32_t ) MCU_GPIO_AFRL_AF0 << 24U )
#define MCU_GPIO_AFRL_7_AF0                   ( ( uint32_t ) MCU_GPIO_AFRL_AF0 << 28U )


// Биты регистра AFRH (alternate function high register)
#define MCU_GPIO_AFRH_BITS                    ( 0xFFFFFFFFU )
#define MCU_GPIO_AFRH_15_MASK                 ( 0xF0000000U )
#define MCU_GPIO_AFRH_14_MASK                 ( 0xF000000U )
#define MCU_GPIO_AFRH_13_MASK                 ( 0xF00000U )
#define MCU_GPIO_AFRH_12_MASK                 ( 0xF0000U )
#define MCU_GPIO_AFRH_11_MASK                 ( 0xF000U )
#define MCU_GPIO_AFRH_10_MASK                 ( 0xF00U )
#define MCU_GPIO_AFRH_9_MASK                  ( 0xF0U )
#define MCU_GPIO_AFRH_8_MASK                  ( 0xFU )

/* Вывод порта альтернативная функция "AF15" */
#define MCU_GPIO_AFRH_AF15                    ( 0xFU )
#define MCU_GPIO_AFRH_8_AF15                  ( ( uint32_t ) MCU_GPIO_AFRH_AF15 << 0U  )
#define MCU_GPIO_AFRH_9_AF15                  ( ( uint32_t ) MCU_GPIO_AFRH_AF15 << 4U  )
#define MCU_GPIO_AFRH_10_AF15                 ( ( uint32_t ) MCU_GPIO_AFRH_AF15 << 8U  )
#define MCU_GPIO_AFRH_11_AF15                 ( ( uint32_t ) MCU_GPIO_AFRH_AF15 << 12U )
#define MCU_GPIO_AFRH_12_AF15                 ( ( uint32_t ) MCU_GPIO_AFRH_AF15 << 16U )
#define MCU_GPIO_AFRH_13_AF15                 ( ( uint32_t ) MCU_GPIO_AFRH_AF15 << 20U )
#define MCU_GPIO_AFRH_14_AF15                 ( ( uint32_t ) MCU_GPIO_AFRH_AF15 << 24U )
#define MCU_GPIO_AFRH_15_AF15                 ( ( uint32_t ) MCU_GPIO_AFRH_AF15 << 28U )

/* Вывод порта альтернативная функция "AF14" */
#define MCU_GPIO_AFRH_AF14                    ( 0xEU )
#define MCU_GPIO_AFRH_8_AF14                  ( ( uint32_t ) MCU_GPIO_AFRH_AF14 << 0U  )
#define MCU_GPIO_AFRH_9_AF14                  ( ( uint32_t ) MCU_GPIO_AFRH_AF14 << 4U  )
#define MCU_GPIO_AFRH_10_AF14                 ( ( uint32_t ) MCU_GPIO_AFRH_AF14 << 8U  )
#define MCU_GPIO_AFRH_11_AF14                 ( ( uint32_t ) MCU_GPIO_AFRH_AF14 << 12U )
#define MCU_GPIO_AFRH_12_AF14                 ( ( uint32_t ) MCU_GPIO_AFRH_AF14 << 16U )
#define MCU_GPIO_AFRH_13_AF14                 ( ( uint32_t ) MCU_GPIO_AFRH_AF14 << 20U )
#define MCU_GPIO_AFRH_14_AF14                 ( ( uint32_t ) MCU_GPIO_AFRH_AF14 << 24U )
#define MCU_GPIO_AFRH_15_AF14                 ( ( uint32_t ) MCU_GPIO_AFRH_AF14 << 28U )

/* Вывод порта альтернативная функция "AF13" */
#define MCU_GPIO_AFRH_AF13                    ( 0xDU )
#define MCU_GPIO_AFRH_8_AF13                  ( ( uint32_t ) MCU_GPIO_AFRH_AF13 << 0U  )
#define MCU_GPIO_AFRH_9_AF13                  ( ( uint32_t ) MCU_GPIO_AFRH_AF13 << 4U  )
#define MCU_GPIO_AFRH_10_AF13                 ( ( uint32_t ) MCU_GPIO_AFRH_AF13 << 8U  )
#define MCU_GPIO_AFRH_11_AF13                 ( ( uint32_t ) MCU_GPIO_AFRH_AF13 << 12U )
#define MCU_GPIO_AFRH_12_AF13                 ( ( uint32_t ) MCU_GPIO_AFRH_AF13 << 16U )
#define MCU_GPIO_AFRH_13_AF13                 ( ( uint32_t ) MCU_GPIO_AFRH_AF13 << 20U )
#define MCU_GPIO_AFRH_14_AF13                 ( ( uint32_t ) MCU_GPIO_AFRH_AF13 << 24U )
#define MCU_GPIO_AFRH_15_AF13                 ( ( uint32_t ) MCU_GPIO_AFRH_AF13 << 28U )

/* Вывод порта альтернативная функция "AF12" */
#define MCU_GPIO_AFRH_AF12                    ( 0xCU )
#define MCU_GPIO_AFRH_8_AF12                  ( ( uint32_t ) MCU_GPIO_AFRH_AF12 << 0U  )
#define MCU_GPIO_AFRH_9_AF12                  ( ( uint32_t ) MCU_GPIO_AFRH_AF12 << 4U  )
#define MCU_GPIO_AFRH_10_AF12                 ( ( uint32_t ) MCU_GPIO_AFRH_AF12 << 8U  )
#define MCU_GPIO_AFRH_11_AF12                 ( ( uint32_t ) MCU_GPIO_AFRH_AF12 << 12U )
#define MCU_GPIO_AFRH_12_AF12                 ( ( uint32_t ) MCU_GPIO_AFRH_AF12 << 16U )
#define MCU_GPIO_AFRH_13_AF12                 ( ( uint32_t ) MCU_GPIO_AFRH_AF12 << 20U )
#define MCU_GPIO_AFRH_14_AF12                 ( ( uint32_t ) MCU_GPIO_AFRH_AF12 << 24U )
#define MCU_GPIO_AFRH_15_AF12                 ( ( uint32_t ) MCU_GPIO_AFRH_AF12 << 28U )

/* Вывод порта альтернативная функция "AF11" */
#define MCU_GPIO_AFRH_AF11                    ( 0xBU )
#define MCU_GPIO_AFRH_8_AF11                  ( ( uint32_t ) MCU_GPIO_AFRH_AF11 << 0U  )
#define MCU_GPIO_AFRH_9_AF11                  ( ( uint32_t ) MCU_GPIO_AFRH_AF11 << 4U  )
#define MCU_GPIO_AFRH_10_AF11                 ( ( uint32_t ) MCU_GPIO_AFRH_AF11 << 8U  )
#define MCU_GPIO_AFRH_11_AF11                 ( ( uint32_t ) MCU_GPIO_AFRH_AF11 << 12U )
#define MCU_GPIO_AFRH_12_AF11                 ( ( uint32_t ) MCU_GPIO_AFRH_AF11 << 16U )
#define MCU_GPIO_AFRH_13_AF11                 ( ( uint32_t ) MCU_GPIO_AFRH_AF11 << 20U )
#define MCU_GPIO_AFRH_14_AF11                 ( ( uint32_t ) MCU_GPIO_AFRH_AF11 << 24U )
#define MCU_GPIO_AFRH_15_AF11                 ( ( uint32_t ) MCU_GPIO_AFRH_AF11 << 28U )

/* Вывод порта альтернативная функция "AF10" */
#define MCU_GPIO_AFRH_AF10                    ( 0xAU )
#define MCU_GPIO_AFRH_8_AF10                  ( ( uint32_t ) MCU_GPIO_AFRH_AF10 << 0U  )
#define MCU_GPIO_AFRH_9_AF10                  ( ( uint32_t ) MCU_GPIO_AFRH_AF10 << 4U  )
#define MCU_GPIO_AFRH_10_AF10                 ( ( uint32_t ) MCU_GPIO_AFRH_AF10 << 8U  )
#define MCU_GPIO_AFRH_11_AF10                 ( ( uint32_t ) MCU_GPIO_AFRH_AF10 << 12U )
#define MCU_GPIO_AFRH_12_AF10                 ( ( uint32_t ) MCU_GPIO_AFRH_AF10 << 16U )
#define MCU_GPIO_AFRH_13_AF10                 ( ( uint32_t ) MCU_GPIO_AFRH_AF10 << 20U )
#define MCU_GPIO_AFRH_14_AF10                 ( ( uint32_t ) MCU_GPIO_AFRH_AF10 << 24U )
#define MCU_GPIO_AFRH_15_AF10                 ( ( uint32_t ) MCU_GPIO_AFRH_AF10 << 28U )

/* Вывод порта альтернативная функция "AF9" */
#define MCU_GPIO_AFRH_AF9                     ( 0x9U )
#define MCU_GPIO_AFRH_8_AF9                   ( ( uint32_t ) MCU_GPIO_AFRH_AF9 << 0U  )
#define MCU_GPIO_AFRH_9_AF9                   ( ( uint32_t ) MCU_GPIO_AFRH_AF9 << 4U  )
#define MCU_GPIO_AFRH_10_AF9                  ( ( uint32_t ) MCU_GPIO_AFRH_AF9 << 8U  )
#define MCU_GPIO_AFRH_11_AF9                  ( ( uint32_t ) MCU_GPIO_AFRH_AF9 << 12U )
#define MCU_GPIO_AFRH_12_AF9                  ( ( uint32_t ) MCU_GPIO_AFRH_AF9 << 16U )
#define MCU_GPIO_AFRH_13_AF9                  ( ( uint32_t ) MCU_GPIO_AFRH_AF9 << 20U )
#define MCU_GPIO_AFRH_14_AF9                  ( ( uint32_t ) MCU_GPIO_AFRH_AF9 << 24U )
#define MCU_GPIO_AFRH_15_AF9                  ( ( uint32_t ) MCU_GPIO_AFRH_AF9 << 28U )

/* Вывод порта альтернативная функция "AF8" */
#define MCU_GPIO_AFRH_AF8                     ( 0x8U )
#define MCU_GPIO_AFRH_8_AF8                   ( ( uint32_t ) MCU_GPIO_AFRH_AF8 << 0U  )
#define MCU_GPIO_AFRH_9_AF8                   ( ( uint32_t ) MCU_GPIO_AFRH_AF8 << 4U  )
#define MCU_GPIO_AFRH_10_AF8                  ( ( uint32_t ) MCU_GPIO_AFRH_AF8 << 8U  )
#define MCU_GPIO_AFRH_11_AF8                  ( ( uint32_t ) MCU_GPIO_AFRH_AF8 << 12U )
#define MCU_GPIO_AFRH_12_AF8                  ( ( uint32_t ) MCU_GPIO_AFRH_AF8 << 16U )
#define MCU_GPIO_AFRH_13_AF8                  ( ( uint32_t ) MCU_GPIO_AFRH_AF8 << 20U )
#define MCU_GPIO_AFRH_14_AF8                  ( ( uint32_t ) MCU_GPIO_AFRH_AF8 << 24U )
#define MCU_GPIO_AFRH_15_AF8                  ( ( uint32_t ) MCU_GPIO_AFRH_AF8 << 28U )

/* Вывод порта альтернативная функция "AF7" */
#define MCU_GPIO_AFRH_AF7                     ( 0x7U )
#define MCU_GPIO_AFRH_8_AF7                   ( ( uint32_t ) MCU_GPIO_AFRH_AF7 << 0U  )
#define MCU_GPIO_AFRH_9_AF7                   ( ( uint32_t ) MCU_GPIO_AFRH_AF7 << 4U  )
#define MCU_GPIO_AFRH_10_AF7                  ( ( uint32_t ) MCU_GPIO_AFRH_AF7 << 8U  )
#define MCU_GPIO_AFRH_11_AF7                  ( ( uint32_t ) MCU_GPIO_AFRH_AF7 << 12U )
#define MCU_GPIO_AFRH_12_AF7                  ( ( uint32_t ) MCU_GPIO_AFRH_AF7 << 16U )
#define MCU_GPIO_AFRH_13_AF7                  ( ( uint32_t ) MCU_GPIO_AFRH_AF7 << 20U )
#define MCU_GPIO_AFRH_14_AF7                  ( ( uint32_t ) MCU_GPIO_AFRH_AF7 << 24U )
#define MCU_GPIO_AFRH_15_AF7                  ( ( uint32_t ) MCU_GPIO_AFRH_AF7 << 28U )

/* Вывод порта альтернативная функция "AF6" */
#define MCU_GPIO_AFRH_AF6                     ( 0x6U )
#define MCU_GPIO_AFRH_8_AF6                   ( ( uint32_t ) MCU_GPIO_AFRH_AF6 << 0U  )
#define MCU_GPIO_AFRH_9_AF6                   ( ( uint32_t ) MCU_GPIO_AFRH_AF6 << 4U  )
#define MCU_GPIO_AFRH_10_AF6                  ( ( uint32_t ) MCU_GPIO_AFRH_AF6 << 8U  )
#define MCU_GPIO_AFRH_11_AF6                  ( ( uint32_t ) MCU_GPIO_AFRH_AF6 << 12U )
#define MCU_GPIO_AFRH_12_AF6                  ( ( uint32_t ) MCU_GPIO_AFRH_AF6 << 16U )
#define MCU_GPIO_AFRH_13_AF6                  ( ( uint32_t ) MCU_GPIO_AFRH_AF6 << 20U )
#define MCU_GPIO_AFRH_14_AF6                  ( ( uint32_t ) MCU_GPIO_AFRH_AF6 << 24U )
#define MCU_GPIO_AFRH_15_AF6                  ( ( uint32_t ) MCU_GPIO_AFRH_AF6 << 28U )

/* Вывод порта альтернативная функция "AF5" */
#define MCU_GPIO_AFRH_AF5                     ( 0x5U )
#define MCU_GPIO_AFRH_8_AF5                   ( ( uint32_t ) MCU_GPIO_AFRH_AF5 << 0U  )
#define MCU_GPIO_AFRH_9_AF5                   ( ( uint32_t ) MCU_GPIO_AFRH_AF5 << 4U  )
#define MCU_GPIO_AFRH_10_AF5                  ( ( uint32_t ) MCU_GPIO_AFRH_AF5 << 8U  )
#define MCU_GPIO_AFRH_11_AF5                  ( ( uint32_t ) MCU_GPIO_AFRH_AF5 << 12U )
#define MCU_GPIO_AFRH_12_AF5                  ( ( uint32_t ) MCU_GPIO_AFRH_AF5 << 16U )
#define MCU_GPIO_AFRH_13_AF5                  ( ( uint32_t ) MCU_GPIO_AFRH_AF5 << 20U )
#define MCU_GPIO_AFRH_14_AF5                  ( ( uint32_t ) MCU_GPIO_AFRH_AF5 << 24U )
#define MCU_GPIO_AFRH_15_AF5                  ( ( uint32_t ) MCU_GPIO_AFRH_AF5 << 28U )

/* Вывод порта альтернативная функция "AF4" */
#define MCU_GPIO_AFRH_AF4                     ( 0x4U )
#define MCU_GPIO_AFRH_8_AF4                   ( ( uint32_t ) MCU_GPIO_AFRH_AF4 << 0U  )
#define MCU_GPIO_AFRH_9_AF4                   ( ( uint32_t ) MCU_GPIO_AFRH_AF4 << 4U  )
#define MCU_GPIO_AFRH_10_AF4                  ( ( uint32_t ) MCU_GPIO_AFRH_AF4 << 8U  )
#define MCU_GPIO_AFRH_11_AF4                  ( ( uint32_t ) MCU_GPIO_AFRH_AF4 << 12U )
#define MCU_GPIO_AFRH_12_AF4                  ( ( uint32_t ) MCU_GPIO_AFRH_AF4 << 16U )
#define MCU_GPIO_AFRH_13_AF4                  ( ( uint32_t ) MCU_GPIO_AFRH_AF4 << 20U )
#define MCU_GPIO_AFRH_14_AF4                  ( ( uint32_t ) MCU_GPIO_AFRH_AF4 << 24U )
#define MCU_GPIO_AFRH_15_AF4                  ( ( uint32_t ) MCU_GPIO_AFRH_AF4 << 28U )

/* Вывод порта альтернативная функция "AF3" */
#define MCU_GPIO_AFRH_AF3                     ( 0x3U )
#define MCU_GPIO_AFRH_8_AF3                   ( ( uint32_t ) MCU_GPIO_AFRH_AF3 << 0U  )
#define MCU_GPIO_AFRH_9_AF3                   ( ( uint32_t ) MCU_GPIO_AFRH_AF3 << 4U  )
#define MCU_GPIO_AFRH_10_AF3                  ( ( uint32_t ) MCU_GPIO_AFRH_AF3 << 8U  )
#define MCU_GPIO_AFRH_11_AF3                  ( ( uint32_t ) MCU_GPIO_AFRH_AF3 << 12U )
#define MCU_GPIO_AFRH_12_AF3                  ( ( uint32_t ) MCU_GPIO_AFRH_AF3 << 16U )
#define MCU_GPIO_AFRH_13_AF3                  ( ( uint32_t ) MCU_GPIO_AFRH_AF3 << 20U )
#define MCU_GPIO_AFRH_14_AF3                  ( ( uint32_t ) MCU_GPIO_AFRH_AF3 << 24U )
#define MCU_GPIO_AFRH_15_AF3                  ( ( uint32_t ) MCU_GPIO_AFRH_AF3 << 28U )

/* Вывод порта альтернативная функция "AF2" */
#define MCU_GPIO_AFRH_AF2                     ( 0x2U )
#define MCU_GPIO_AFRH_8_AF2                   ( ( uint32_t ) MCU_GPIO_AFRH_AF2 << 0U  )
#define MCU_GPIO_AFRH_9_AF2                   ( ( uint32_t ) MCU_GPIO_AFRH_AF2 << 4U  )
#define MCU_GPIO_AFRH_10_AF2                  ( ( uint32_t ) MCU_GPIO_AFRH_AF2 << 8U  )
#define MCU_GPIO_AFRH_11_AF2                  ( ( uint32_t ) MCU_GPIO_AFRH_AF2 << 12U )
#define MCU_GPIO_AFRH_12_AF2                  ( ( uint32_t ) MCU_GPIO_AFRH_AF2 << 16U )
#define MCU_GPIO_AFRH_13_AF2                  ( ( uint32_t ) MCU_GPIO_AFRH_AF2 << 20U )
#define MCU_GPIO_AFRH_14_AF2                  ( ( uint32_t ) MCU_GPIO_AFRH_AF2 << 24U )
#define MCU_GPIO_AFRH_15_AF2                  ( ( uint32_t ) MCU_GPIO_AFRH_AF2 << 28U )

/* Вывод порта альтернативная функция "AF1" */
#define MCU_GPIO_AFRH_AF1                     ( 0x1U )
#define MCU_GPIO_AFRH_8_AF1                   ( ( uint32_t ) MCU_GPIO_AFRH_AF1 << 0U  )
#define MCU_GPIO_AFRH_9_AF1                   ( ( uint32_t ) MCU_GPIO_AFRH_AF1 << 4U  )
#define MCU_GPIO_AFRH_10_AF1                  ( ( uint32_t ) MCU_GPIO_AFRH_AF1 << 8U  )
#define MCU_GPIO_AFRH_11_AF1                  ( ( uint32_t ) MCU_GPIO_AFRH_AF1 << 12U )
#define MCU_GPIO_AFRH_12_AF1                  ( ( uint32_t ) MCU_GPIO_AFRH_AF1 << 16U )
#define MCU_GPIO_AFRH_13_AF1                  ( ( uint32_t ) MCU_GPIO_AFRH_AF1 << 20U )
#define MCU_GPIO_AFRH_14_AF1                  ( ( uint32_t ) MCU_GPIO_AFRH_AF1 << 24U )
#define MCU_GPIO_AFRH_15_AF1                  ( ( uint32_t ) MCU_GPIO_AFRH_AF1 << 28U )

/* Вывод порта альтернативная функция "AF0" */
#define MCU_GPIO_AFRH_AF0                     ( 0x0U )
#define MCU_GPIO_AFRH_8_AF0                   ( ( uint32_t ) MCU_GPIO_AFRH_AF0 << 0U  )
#define MCU_GPIO_AFRH_9_AF0                   ( ( uint32_t ) MCU_GPIO_AFRH_AF0 << 4U  )
#define MCU_GPIO_AFRH_10_AF0                  ( ( uint32_t ) MCU_GPIO_AFRH_AF0 << 8U  )
#define MCU_GPIO_AFRH_11_AF0                  ( ( uint32_t ) MCU_GPIO_AFRH_AF0 << 12U )
#define MCU_GPIO_AFRH_12_AF0                  ( ( uint32_t ) MCU_GPIO_AFRH_AF0 << 16U )
#define MCU_GPIO_AFRH_13_AF0                  ( ( uint32_t ) MCU_GPIO_AFRH_AF0 << 20U )
#define MCU_GPIO_AFRH_14_AF0                  ( ( uint32_t ) MCU_GPIO_AFRH_AF0 << 24U )
#define MCU_GPIO_AFRH_15_AF0                  ( ( uint32_t ) MCU_GPIO_AFRH_AF0 << 28U )


/* ----- SYSCFG (system configuration control) ----- */
typedef volatile struct tag_stm32f407_syscfg {
    uint32_t        memrmp;
    uint32_t        pmc;
    uint32_t        exticr1;
    uint32_t        exticr2;
    uint32_t        exticr3;
    uint32_t        exticr4;
    uint32_t        cmpcr;
} stm32f407_syscfg_t;

#define STM32F407_SYSCFG_PTR                  ( ( stm32f407_syscfg_t * ) 0x40013800U )

/* Биты регистра MEMRMP (memory remap register) */
#define MCU_SYSCFG_MEMRMP_BITS                ( 0x3U )
#define MCU_SYSCFG_MEMRMP_MEM_MODE            ( 0x3U )

/* Биты регистра PMC (peripheral mode configuration register) */
#define MCU_SYSCFG_PMC_BITS                   ( 0x800000U )
#define MCU_SYSCFG_PMC_MII_RMII_SEL           ( 0x800000U )

/* Биты регистра EXTICR1 (external interrupt configuration register 1) */
#define MCU_SYSCFG_EXTICR1_RESET_BITS         ( 0xFFFFFFFFU )
#define MCU_SYSCFG_EXTICR1_BITS               ( 0xFFFFU )
#define MCU_SYSCFG_EXTICR1_EXTI3_MASK         ( 0xF000U )
#define MCU_SYSCFG_EXTICR1_EXTI2_MASK         ( 0xF00U )
#define MCU_SYSCFG_EXTICR1_EXTI1_MASK         ( 0xF0U )
#define MCU_SYSCFG_EXTICR1_EXTI0_MASK         ( 0xFU )

/* Биты регистра EXTICR2 (external interrupt configuration register 2) */
#define MCU_SYSCFG_EXTICR2_BITS               ( 0xFFFFU )
#define MCU_SYSCFG_EXTICR2_EXTI7_MASK         ( 0xF000U )
#define MCU_SYSCFG_EXTICR2_EXTI6_MASK         ( 0xF00U )
#define MCU_SYSCFG_EXTICR2_EXTI5_MASK         ( 0xF0U )
#define MCU_SYSCFG_EXTICR2_EXTI4_MASK         ( 0xFU )

/* Биты регистра EXTICR3 (external interrupt configuration register 3) */
#define MCU_SYSCFG_EXTICR3_BITS               ( 0xFFFFU )
#define MCU_SYSCFG_EXTICR3_EXTI11_MASK        ( 0xF000U )
#define MCU_SYSCFG_EXTICR3_EXTI10_MASK        ( 0xF00U )
#define MCU_SYSCFG_EXTICR3_EXTI9_MASK         ( 0xF0U )
#define MCU_SYSCFG_EXTICR3_EXTI8_MASK         ( 0xFU )

/* Биты регистра EXTICR4 (external interrupt configuration register 4) */
#define MCU_SYSCFG_EXTICR4_BITS               ( 0xFFFFU )
#define MCU_SYSCFG_EXTICR4_EXTI15_MASK        ( 0xF000U )
#define MCU_SYSCFG_EXTICR4_EXTI14_MASK        ( 0xF00U )
#define MCU_SYSCFG_EXTICR4_EXTI13_MASK        ( 0xF0U )
#define MCU_SYSCFG_EXTICR4_EXTI12_MASK        ( 0xFU )

/* Биты регистра CMPCR (compensation cell control register) */
#define MCU_SYSCFG_CMPCR_BITS                 ( 0x101U )
#define MCU_SYSCFG_CMPCR_READY                ( 0x100U )
#define MCU_SYSCFG_CMPCR_CMP_PD               ( 0x1U )


/* ----- MCU_DMA1 (direct memory access controller 1) ----- */
typedef volatile struct tag_stm32f407_dma1 {
    uint32_t        lisr;
    uint32_t        hisr;
    uint32_t        lifcr;
    uint32_t        hifcr;
    uint32_t        s0cr;
    uint32_t        s0ndtr;
    uint32_t        s0par;
    uint32_t        s0m0ar;
    uint32_t        s0m1ar;
    uint32_t        s0fcr;
    uint32_t        s1cr;
    uint32_t        s1ndtr;
    uint32_t        s1par;
    uint32_t        s1m0ar;
    uint32_t        s1m1ar;
    uint32_t        s1fcr;
    uint32_t        s2cr;
    uint32_t        s2ndtr;
    uint32_t        s2par;
    uint32_t        s2m0ar;
    uint32_t        s2m1ar;
    uint32_t        s2fcr;
    uint32_t        s3cr;
    uint32_t        s3ndtr;
    uint32_t        s3par;
    uint32_t        s3m0ar;
    uint32_t        s3m1ar;
    uint32_t        s3fcr;
    uint32_t        s4cr;
    uint32_t        s4ndtr;
    uint32_t        s4par;
    uint32_t        s4m0ar;
    uint32_t        s4m1ar;
    uint32_t        s4fcr;
    uint32_t        s5cr;
    uint32_t        s5ndtr;
    uint32_t        s5par;
    uint32_t        s5m0ar;
    uint32_t        s5m1ar;
    uint32_t        s5fcr;
    uint32_t        s6cr;
    uint32_t        s6ndtr;
    uint32_t        s6par;
    uint32_t        s6m0ar;
    uint32_t        s6m1ar;
    uint32_t        s6fcr;
    uint32_t        s7cr;
    uint32_t        s7ndtr;
    uint32_t        s7par;
    uint32_t        s7m0ar;
    uint32_t        s7m1ar;
    uint32_t        s7fcr;
} stm32f407_dma1_t;

#define STM32F407_DMA1_PTR                    ( ( stm32f407_dma1_t * ) 0x40026000U )

/* Биты регистра LISR (low interrupt status register) */
#define MCU_DMA1_LISR_BITS                    ( 0xF7D0F7DU )
#define MCU_DMA1_LISR_TCIF3                   ( 0x8000000U )
#define MCU_DMA1_LISR_HTIF3                   ( 0x4000000U )
#define MCU_DMA1_LISR_TEIF3                   ( 0x2000000U )
#define MCU_DMA1_LISR_DMEIF3                  ( 0x1000000U )
#define MCU_DMA1_LISR_FEIF3                   ( 0x400000U )
#define MCU_DMA1_LISR_TCIF2                   ( 0x200000U )
#define MCU_DMA1_LISR_HTIF2                   ( 0x100000U )
#define MCU_DMA1_LISR_TEIF2                   ( 0x80000U )
#define MCU_DMA1_LISR_DMEIF2                  ( 0x40000U )
#define MCU_DMA1_LISR_FEIF2                   ( 0x10000U )
#define MCU_DMA1_LISR_TCIF1                   ( 0x800U )
#define MCU_DMA1_LISR_HTIF1                   ( 0x400U )
#define MCU_DMA1_LISR_TEIF1                   ( 0x200U )
#define MCU_DMA1_LISR_DMEIF1                  ( 0x100U )
#define MCU_DMA1_LISR_FEIF1                   ( 0x40U )
#define MCU_DMA1_LISR_TCIF0                   ( 0x20U )
#define MCU_DMA1_LISR_HTIF0                   ( 0x10U )
#define MCU_DMA1_LISR_TEIF0                   ( 0x8U )
#define MCU_DMA1_LISR_DMEIF0                  ( 0x4U )
#define MCU_DMA1_LISR_FEIF0                   ( 0x1U )

/* Биты регистра HISR (high interrupt status register) */
#define MCU_DMA1_HISR_BITS                    ( 0xF7D0F7DU )
#define MCU_DMA1_HISR_TCIF7                   ( 0x8000000U )
#define MCU_DMA1_HISR_HTIF7                   ( 0x4000000U )
#define MCU_DMA1_HISR_TEIF7                   ( 0x2000000U )
#define MCU_DMA1_HISR_DMEIF7                  ( 0x1000000U )
#define MCU_DMA1_HISR_FEIF7                   ( 0x400000U )
#define MCU_DMA1_HISR_TCIF6                   ( 0x200000U )
#define MCU_DMA1_HISR_HTIF6                   ( 0x100000U )
#define MCU_DMA1_HISR_TEIF6                   ( 0x80000U )
#define MCU_DMA1_HISR_DMEIF6                  ( 0x40000U )
#define MCU_DMA1_HISR_FEIF6                   ( 0x10000U )
#define MCU_DMA1_HISR_TCIF5                   ( 0x800U )
#define MCU_DMA1_HISR_HTIF5                   ( 0x400U )
#define MCU_DMA1_HISR_TEIF5                   ( 0x200U )
#define MCU_DMA1_HISR_DMEIF5                  ( 0x100U )
#define MCU_DMA1_HISR_FEIF5                   ( 0x40U )
#define MCU_DMA1_HISR_TCIF4                   ( 0x20U )
#define MCU_DMA1_HISR_HTIF4                   ( 0x10U )
#define MCU_DMA1_HISR_TEIF4                   ( 0x8U )
#define MCU_DMA1_HISR_DMEIF4                  ( 0x4U )
#define MCU_DMA1_HISR_FEIF4                   ( 0x1U )

/* Биты регистра LIFCR (low interrupt flag clear register) */
#define MCU_DMA1_LIFCR_BITS                   ( 0xF7D0F7DU )
#define MCU_DMA1_LIFCR_CTCIF3                 ( 0x8000000U )
#define MCU_DMA1_LIFCR_CHTIF3                 ( 0x4000000U )
#define MCU_DMA1_LIFCR_CTEIF3                 ( 0x2000000U )
#define MCU_DMA1_LIFCR_CDMEIF3                ( 0x1000000U )
#define MCU_DMA1_LIFCR_CFEIF3                 ( 0x400000U )
#define MCU_DMA1_LIFCR_CTCIF2                 ( 0x200000U )
#define MCU_DMA1_LIFCR_CHTIF2                 ( 0x100000U )
#define MCU_DMA1_LIFCR_CTEIF2                 ( 0x80000U )
#define MCU_DMA1_LIFCR_CDMEIF2                ( 0x40000U )
#define MCU_DMA1_LIFCR_CFEIF2                 ( 0x10000U )
#define MCU_DMA1_LIFCR_CTCIF1                 ( 0x800U )
#define MCU_DMA1_LIFCR_CHTIF1                 ( 0x400U )
#define MCU_DMA1_LIFCR_CTEIF1                 ( 0x200U )
#define MCU_DMA1_LIFCR_CDMEIF1                ( 0x100U )
#define MCU_DMA1_LIFCR_CFEIF1                 ( 0x40U )
#define MCU_DMA1_LIFCR_CTCIF0                 ( 0x20U )
#define MCU_DMA1_LIFCR_CHTIF0                 ( 0x10U )
#define MCU_DMA1_LIFCR_CTEIF0                 ( 0x8U )
#define MCU_DMA1_LIFCR_CDMEIF0                ( 0x4U )
#define MCU_DMA1_LIFCR_CFEIF0                 ( 0x1U )

/* Биты регистра HIFCR (high interrupt flag clear register) */
#define MCU_DMA1_HIFCR_BITS                   ( 0xF7D0F7DU )
#define MCU_DMA1_HIFCR_CTCIF7                 ( 0x8000000U )
#define MCU_DMA1_HIFCR_CHTIF7                 ( 0x4000000U )
#define MCU_DMA1_HIFCR_CTEIF7                 ( 0x2000000U )
#define MCU_DMA1_HIFCR_CDMEIF7                ( 0x1000000U )
#define MCU_DMA1_HIFCR_CFEIF7                 ( 0x400000U )
#define MCU_DMA1_HIFCR_CTCIF6                 ( 0x200000U )
#define MCU_DMA1_HIFCR_CHTIF6                 ( 0x100000U )
#define MCU_DMA1_HIFCR_CTEIF6                 ( 0x80000U )
#define MCU_DMA1_HIFCR_CDMEIF6                ( 0x40000U )
#define MCU_DMA1_HIFCR_CFEIF6                 ( 0x10000U )
#define MCU_DMA1_HIFCR_CTCIF5                 ( 0x800U )
#define MCU_DMA1_HIFCR_CHTIF5                 ( 0x400U )
#define MCU_DMA1_HIFCR_CTEIF5                 ( 0x200U )
#define MCU_DMA1_HIFCR_CDMEIF5                ( 0x100U )
#define MCU_DMA1_HIFCR_CFEIF5                 ( 0x40U )
#define MCU_DMA1_HIFCR_CTCIF4                 ( 0x20U )
#define MCU_DMA1_HIFCR_CHTIF4                 ( 0x10U )
#define MCU_DMA1_HIFCR_CTEIF4                 ( 0x8U )
#define MCU_DMA1_HIFCR_CDMEIF4                ( 0x4U )
#define MCU_DMA1_HIFCR_CFEIF4                 ( 0x1U )

/* Биты регистра S0CR (stream 0 configuration register) */
#define MCU_DMA1_S0CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA1_S0CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA1_S0CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA1_S0CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA1_S0CR_CT                      ( 0x80000U )
#define MCU_DMA1_S0CR_DBM                     ( 0x40000U )
#define MCU_DMA1_S0CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA1_S0CR_PINCOS                  ( 0x8000U )
#define MCU_DMA1_S0CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA1_S0CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA1_S0CR_MINC                    ( 0x400U )
#define MCU_DMA1_S0CR_PINC                    ( 0x200U )
#define MCU_DMA1_S0CR_CIRC                    ( 0x100U )
#define MCU_DMA1_S0CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA1_S0CR_PFCTRL                  ( 0x20U )
#define MCU_DMA1_S0CR_TCIE                    ( 0x10U )
#define MCU_DMA1_S0CR_HTIE                    ( 0x8U )
#define MCU_DMA1_S0CR_TEIE                    ( 0x4U )
#define MCU_DMA1_S0CR_DMEIE                   ( 0x2U )
#define MCU_DMA1_S0CR_EN                      ( 0x1U )

/* Биты регистра S0NDTR (stream 0 number of data register) */
#define MCU_DMA1_S0NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA1_S0NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S0PAR (stream 0 peripheral address register) */
#define MCU_DMA1_S0PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA1_S0PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S0M0AR (stream 0 memory 0 address register) */
#define MCU_DMA1_S0M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S0M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S0M1AR (tream 0 memory 1 address register) */
#define MCU_DMA1_S0M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S0M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S0FCR (stream 0 FIFO control register) */
#define MCU_DMA1_S0FCR_BITS                   ( 0xBFU )
#define MCU_DMA1_S0FCR_FEIE                   ( 0x80U )
#define MCU_DMA1_S0FCR_FS_MASK                ( 0x38U )
#define MCU_DMA1_S0FCR_DMDIS                  ( 0x4U )
#define MCU_DMA1_S0FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S1CR (stream 1 configuration register) */
#define MCU_DMA1_S1CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA1_S1CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA1_S1CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA1_S1CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA1_S1CR_CT                      ( 0x80000U )
#define MCU_DMA1_S1CR_DBM                     ( 0x40000U )
#define MCU_DMA1_S1CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA1_S1CR_PINCOS                  ( 0x8000U )
#define MCU_DMA1_S1CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA1_S1CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA1_S1CR_MINC                    ( 0x400U )
#define MCU_DMA1_S1CR_PINC                    ( 0x200U )
#define MCU_DMA1_S1CR_CIRC                    ( 0x100U )
#define MCU_DMA1_S1CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA1_S1CR_PFCTRL                  ( 0x20U )
#define MCU_DMA1_S1CR_TCIE                    ( 0x10U )
#define MCU_DMA1_S1CR_HTIE                    ( 0x8U )
#define MCU_DMA1_S1CR_TEIE                    ( 0x4U )
#define MCU_DMA1_S1CR_DMEIE                   ( 0x2U )
#define MCU_DMA1_S1CR_EN                      ( 0x1U )

/* Биты регистра S1NDTR (stream 1 number of data register) */
#define MCU_DMA1_S1NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA1_S1NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S1PAR (stream 1 peripheral address register) */
#define MCU_DMA1_S1PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA1_S1PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S1M0AR (stream 1 memory 0 address register) */
#define MCU_DMA1_S1M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S1M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S1M1AR (stream 1 memory 1 address register) */
#define MCU_DMA1_S1M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S1M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S1FCR (stream 1 FIFO control register) */
#define MCU_DMA1_S1FCR_BITS                   ( 0xBFU )
#define MCU_DMA1_S1FCR_FEIE                   ( 0x80U )
#define MCU_DMA1_S1FCR_FS_MASK                ( 0x38U )
#define MCU_DMA1_S1FCR_DMDIS                  ( 0x4U )
#define MCU_DMA1_S1FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S2CR (stream 2 configuration register) */
#define MCU_DMA1_S2CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA1_S2CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA1_S2CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA1_S2CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA1_S2CR_CT                      ( 0x80000U )
#define MCU_DMA1_S2CR_DBM                     ( 0x40000U )
#define MCU_DMA1_S2CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA1_S2CR_PINCOS                  ( 0x8000U )
#define MCU_DMA1_S2CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA1_S2CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA1_S2CR_MINC                    ( 0x400U )
#define MCU_DMA1_S2CR_PINC                    ( 0x200U )
#define MCU_DMA1_S2CR_CIRC                    ( 0x100U )
#define MCU_DMA1_S2CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA1_S2CR_PFCTRL                  ( 0x20U )
#define MCU_DMA1_S2CR_TCIE                    ( 0x10U )
#define MCU_DMA1_S2CR_HTIE                    ( 0x8U )
#define MCU_DMA1_S2CR_TEIE                    ( 0x4U )
#define MCU_DMA1_S2CR_DMEIE                   ( 0x2U )
#define MCU_DMA1_S2CR_EN                      ( 0x1U )

/* Биты регистра S2NDTR (stream 2 number of data register) */
#define MCU_DMA1_S2NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA1_S2NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S2PAR (stream 2 peripheral address register) */
#define MCU_DMA1_S2PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA1_S2PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S2M0AR (stream 2 memory 0 address register) */
#define MCU_DMA1_S2M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S2M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S2M1AR (stream 2 memory 1 address register) */
#define MCU_DMA1_S2M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S2M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S2FCR (stream 2 FIFO control register) */
#define MCU_DMA1_S2FCR_BITS                   ( 0xBFU )
#define MCU_DMA1_S2FCR_FEIE                   ( 0x80U )
#define MCU_DMA1_S2FCR_FS_MASK                ( 0x38U )
#define MCU_DMA1_S2FCR_DMDIS                  ( 0x4U )
#define MCU_DMA1_S2FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S3CR (stream 3 configuration register) */
#define MCU_DMA1_S3CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA1_S3CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA1_S3CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA1_S3CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA1_S3CR_CT                      ( 0x80000U )
#define MCU_DMA1_S3CR_DBM                     ( 0x40000U )
#define MCU_DMA1_S3CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA1_S3CR_PINCOS                  ( 0x8000U )
#define MCU_DMA1_S3CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA1_S3CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA1_S3CR_MINC                    ( 0x400U )
#define MCU_DMA1_S3CR_PINC                    ( 0x200U )
#define MCU_DMA1_S3CR_CIRC                    ( 0x100U )
#define MCU_DMA1_S3CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA1_S3CR_PFCTRL                  ( 0x20U )
#define MCU_DMA1_S3CR_TCIE                    ( 0x10U )
#define MCU_DMA1_S3CR_HTIE                    ( 0x8U )
#define MCU_DMA1_S3CR_TEIE                    ( 0x4U )
#define MCU_DMA1_S3CR_DMEIE                   ( 0x2U )
#define MCU_DMA1_S3CR_EN                      ( 0x1U )

/* Биты регистра S3NDTR (stream 3 number of data register) */
#define MCU_DMA1_S3NDTR_BITS                  ( 0xFFFFU )
#define MCU_DMA1_S3NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S3PAR (stream 3 peripheral address register) */
#define MCU_DMA1_S3PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA1_S3PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S3M0AR (stream 3 memory 0 address register) */
#define MCU_DMA1_S3M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S3M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S3M1AR (stream 3 memory 1 address register) */
#define MCU_DMA1_S3M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S3M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S3FCR (stream 3 FIFO control register) */
#define MCU_DMA1_S3FCR_BITS                   ( 0xBFU )
#define MCU_DMA1_S3FCR_FEIE                   ( 0x80U )
#define MCU_DMA1_S3FCR_FS_MASK                ( 0x38U )
#define MCU_DMA1_S3FCR_DMDIS                  ( 0x4U )
#define MCU_DMA1_S3FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S4CR (stream 4 configuration register) */
#define MCU_DMA1_S4CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA1_S4CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA1_S4CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA1_S4CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA1_S4CR_CT                      ( 0x80000U )
#define MCU_DMA1_S4CR_DBM                     ( 0x40000U )
#define MCU_DMA1_S4CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA1_S4CR_PINCOS                  ( 0x8000U )
#define MCU_DMA1_S4CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA1_S4CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA1_S4CR_MINC                    ( 0x400U )
#define MCU_DMA1_S4CR_PINC                    ( 0x200U )
#define MCU_DMA1_S4CR_CIRC                    ( 0x100U )
#define MCU_DMA1_S4CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA1_S4CR_PFCTRL                  ( 0x20U )
#define MCU_DMA1_S4CR_TCIE                    ( 0x10U )
#define MCU_DMA1_S4CR_HTIE                    ( 0x8U )
#define MCU_DMA1_S4CR_TEIE                    ( 0x4U )
#define MCU_DMA1_S4CR_DMEIE                   ( 0x2U )
#define MCU_DMA1_S4CR_EN                      ( 0x1U )

/* Биты регистра S4NDTR (stream 4 number of data register) */
#define MCU_DMA1_S4NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA1_S4NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S4PAR (stream 4 peripheral address register) */
#define MCU_DMA1_S4PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA1_S4PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S4M0AR (stream 4 memory 0 address register) */
#define MCU_DMA1_S4M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S4M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S4M1AR (stream 4 memory 1 address register) */
#define MCU_DMA1_S4M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S4M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S4FCR (stream 4 FIFO control register) */
#define MCU_DMA1_S4FCR_BITS                   ( 0xBFU )
#define MCU_DMA1_S4FCR_FEIE                   ( 0x80U )
#define MCU_DMA1_S4FCR_FS_MASK                ( 0x38U )
#define MCU_DMA1_S4FCR_DMDIS                  ( 0x4U )
#define MCU_DMA1_S4FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S5CR (stream 5 configuration register) */
#define MCU_DMA1_S5CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA1_S5CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA1_S5CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA1_S5CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA1_S5CR_CT                      ( 0x80000U )
#define MCU_DMA1_S5CR_DBM                     ( 0x40000U )
#define MCU_DMA1_S5CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA1_S5CR_PINCOS                  ( 0x8000U )
#define MCU_DMA1_S5CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA1_S5CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA1_S5CR_MINC                    ( 0x400U )
#define MCU_DMA1_S5CR_PINC                    ( 0x200U )
#define MCU_DMA1_S5CR_CIRC                    ( 0x100U )
#define MCU_DMA1_S5CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA1_S5CR_PFCTRL                  ( 0x20U )
#define MCU_DMA1_S5CR_TCIE                    ( 0x10U )
#define MCU_DMA1_S5CR_HTIE                    ( 0x8U )
#define MCU_DMA1_S5CR_TEIE                    ( 0x4U )
#define MCU_DMA1_S5CR_DMEIE                   ( 0x2U )
#define MCU_DMA1_S5CR_EN                      ( 0x1U )

/* Биты регистра S5NDTR (stream 5 number of data register) */
#define MCU_DMA1_S5NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA1_S5NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S5PAR (stream 5 peripheral address register) */
#define MCU_DMA1_S5PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA1_S5PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S5M0AR (stream 5 memory 0 address register) */
#define MCU_DMA1_S5M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S5M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S5M1AR (stream 5 memory 1 address register) */
#define MCU_DMA1_S5M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S5M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S5FCR (stream 5 FIFO control register) */
#define MCU_DMA1_S5FCR_BITS                   ( 0xBFU )
#define MCU_DMA1_S5FCR_FEIE                   ( 0x80U )
#define MCU_DMA1_S5FCR_FS_MASK                ( 0x38U )
#define MCU_DMA1_S5FCR_DMDIS                  ( 0x4U )
#define MCU_DMA1_S5FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S6CR (stream 6 configuration register) */
#define MCU_DMA1_S6CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA1_S6CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA1_S6CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA1_S6CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA1_S6CR_CT                      ( 0x80000U )
#define MCU_DMA1_S6CR_DBM                     ( 0x40000U )
#define MCU_DMA1_S6CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA1_S6CR_PINCOS                  ( 0x8000U )
#define MCU_DMA1_S6CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA1_S6CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA1_S6CR_MINC                    ( 0x400U )
#define MCU_DMA1_S6CR_PINC                    ( 0x200U )
#define MCU_DMA1_S6CR_CIRC                    ( 0x100U )
#define MCU_DMA1_S6CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA1_S6CR_PFCTRL                  ( 0x20U )
#define MCU_DMA1_S6CR_TCIE                    ( 0x10U )
#define MCU_DMA1_S6CR_HTIE                    ( 0x8U )
#define MCU_DMA1_S6CR_TEIE                    ( 0x4U )
#define MCU_DMA1_S6CR_DMEIE                   ( 0x2U )
#define MCU_DMA1_S6CR_EN                      ( 0x1U )

/* Биты регистра S6NDTR (stream 6 number of data register) */
#define MCU_DMA1_S6NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA1_S6NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S6PAR (stream 6 peripheral address register) */
#define MCU_DMA1_S6PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA1_S6PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S6M0AR (stream 6 memory 0 address register) */
#define MCU_DMA1_S6M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S6M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S6M1AR (stream 6 memory 1 address register) */
#define MCU_DMA1_S6M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S6M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S6FCR (stream 6 FIFO control register) */
#define MCU_DMA1_S6FCR_BITS                   ( 0xBFU )
#define MCU_DMA1_S6FCR_FEIE                   ( 0x80U )
#define MCU_DMA1_S6FCR_FS_MASK                ( 0x38U )
#define MCU_DMA1_S6FCR_DMDIS                  ( 0x4U )
#define MCU_DMA1_S6FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S7CR (stream 7 configuration register) */
#define MCU_DMA1_S7CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA1_S7CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA1_S7CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA1_S7CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA1_S7CR_CT                      ( 0x80000U )
#define MCU_DMA1_S7CR_DBM                     ( 0x40000U )
#define MCU_DMA1_S7CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA1_S7CR_PINCOS                  ( 0x8000U )
#define MCU_DMA1_S7CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA1_S7CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA1_S7CR_MINC                    ( 0x400U )
#define MCU_DMA1_S7CR_PINC                    ( 0x200U )
#define MCU_DMA1_S7CR_CIRC                    ( 0x100U )
#define MCU_DMA1_S7CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA1_S7CR_PFCTRL                  ( 0x20U )
#define MCU_DMA1_S7CR_TCIE                    ( 0x10U )
#define MCU_DMA1_S7CR_HTIE                    ( 0x8U )
#define MCU_DMA1_S7CR_TEIE                    ( 0x4U )
#define MCU_DMA1_S7CR_DMEIE                   ( 0x2U )
#define MCU_DMA1_S7CR_EN                      ( 0x1U )

/* Биты регистра S7NDTR (stream 7 number of data register) */
#define MCU_DMA1_S7NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA1_S7NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра SPAR (stream 7 peripheral address register) */
#define MCU_DMA1_S7PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA1_S7PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S7M0AR (stream 7 memory 0 address register) */
#define MCU_DMA1_S7M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S7M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S7M1AR (stream 7 memory 1 address register) */
#define MCU_DMA1_S7M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA1_S7M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра MCU_DMA1_S7FCR (stream 7 FIFO control register) */
#define MCU_DMA1_S7FCR_BITS                   ( 0xBFU )
#define MCU_DMA1_S7FCR_FEIE                   ( 0x80U )
#define MCU_DMA1_S7FCR_FS_MASK                ( 0x38U )
#define MCU_DMA1_S7FCR_DMDIS                  ( 0x4U )
#define MCU_DMA1_S7FCR_FTH_MASK               ( 0x3U )


/* ----- MCU_DMA2 (direct memory access controller 2) ----- */
typedef volatile struct tag_stm32f407_DMA2 {
    uint32_t        lisr;
    uint32_t        hisr;
    uint32_t        lifcr;
    uint32_t        hifcr;
    uint32_t        s0cr;
    uint32_t        s0ndtr;
    uint32_t        s0par;
    uint32_t        s0m0ar;
    uint32_t        s0m1ar;
    uint32_t        s0fcr;
    uint32_t        s1cr;
    uint32_t        s1ndtr;
    uint32_t        s1par;
    uint32_t        s1m0ar;
    uint32_t        s1m1ar;
    uint32_t        s1fcr;
    uint32_t        s2cr;
    uint32_t        s2ndtr;
    uint32_t        s2par;
    uint32_t        s2m0ar;
    uint32_t        s2m1ar;
    uint32_t        s2fcr;
    uint32_t        s3cr;
    uint32_t        s3ndtr;
    uint32_t        s3par;
    uint32_t        s3m0ar;
    uint32_t        s3m1ar;
    uint32_t        s3fcr;
    uint32_t        s4cr;
    uint32_t        s4ndtr;
    uint32_t        s4par;
    uint32_t        s4m0ar;
    uint32_t        s4m1ar;
    uint32_t        s4fcr;
    uint32_t        s5cr;
    uint32_t        s5ndtr;
    uint32_t        s5par;
    uint32_t        s5m0ar;
    uint32_t        s5m1ar;
    uint32_t        s5fcr;
    uint32_t        s6cr;
    uint32_t        s6ndtr;
    uint32_t        s6par;
    uint32_t        s6m0ar;
    uint32_t        s6m1ar;
    uint32_t        s6fcr;
    uint32_t        s7cr;
    uint32_t        s7ndtr;
    uint32_t        s7par;
    uint32_t        s7m0ar;
    uint32_t        s7m1ar;
    uint32_t        s7fcr;
} stm32f407_dma2_t;

#define STM32F407_DMA2_PTR                    ( ( stm32f407_dma2_t * ) 0x40026400U )

/* Биты регистра LISR (low interrupt status register) */
#define MCU_DMA2_LISR_BITS                    ( 0xF7D0F7DU )
#define MCU_DMA2_LISR_TCIF3                   ( 0x8000000U )
#define MCU_DMA2_LISR_HTIF3                   ( 0x4000000U )
#define MCU_DMA2_LISR_TEIF3                   ( 0x2000000U )
#define MCU_DMA2_LISR_DMEIF3                  ( 0x1000000U )
#define MCU_DMA2_LISR_FEIF3                   ( 0x400000U )
#define MCU_DMA2_LISR_TCIF2                   ( 0x200000U )
#define MCU_DMA2_LISR_HTIF2                   ( 0x100000U )
#define MCU_DMA2_LISR_TEIF2                   ( 0x80000U )
#define MCU_DMA2_LISR_DMEIF2                  ( 0x40000U )
#define MCU_DMA2_LISR_FEIF2                   ( 0x10000U )
#define MCU_DMA2_LISR_TCIF1                   ( 0x800U )
#define MCU_DMA2_LISR_HTIF1                   ( 0x400U )
#define MCU_DMA2_LISR_TEIF1                   ( 0x200U )
#define MCU_DMA2_LISR_DMEIF1                  ( 0x100U )
#define MCU_DMA2_LISR_FEIF1                   ( 0x40U )
#define MCU_DMA2_LISR_TCIF0                   ( 0x20U )
#define MCU_DMA2_LISR_HTIF0                   ( 0x10U )
#define MCU_DMA2_LISR_TEIF0                   ( 0x8U )
#define MCU_DMA2_LISR_DMEIF0                  ( 0x4U )
#define MCU_DMA2_LISR_FEIF0                   ( 0x1U )

/* Биты регистра HISR (high interrupt status register) */
#define MCU_DMA2_HISR_BITS                    ( 0xF7D0F7DU )
#define MCU_DMA2_HISR_TCIF7                   ( 0x8000000U )
#define MCU_DMA2_HISR_HTIF7                   ( 0x4000000U )
#define MCU_DMA2_HISR_TEIF7                   ( 0x2000000U )
#define MCU_DMA2_HISR_DMEIF7                  ( 0x1000000U )
#define MCU_DMA2_HISR_FEIF7                   ( 0x400000U )
#define MCU_DMA2_HISR_TCIF6                   ( 0x200000U )
#define MCU_DMA2_HISR_HTIF6                   ( 0x100000U )
#define MCU_DMA2_HISR_TEIF6                   ( 0x80000U )
#define MCU_DMA2_HISR_DMEIF6                  ( 0x40000U )
#define MCU_DMA2_HISR_FEIF6                   ( 0x10000U )
#define MCU_DMA2_HISR_TCIF5                   ( 0x800U )
#define MCU_DMA2_HISR_HTIF5                   ( 0x400U )
#define MCU_DMA2_HISR_TEIF5                   ( 0x200U )
#define MCU_DMA2_HISR_DMEIF5                  ( 0x100U )
#define MCU_DMA2_HISR_FEIF5                   ( 0x40U )
#define MCU_DMA2_HISR_TCIF4                   ( 0x20U )
#define MCU_DMA2_HISR_HTIF4                   ( 0x10U )
#define MCU_DMA2_HISR_TEIF4                   ( 0x8U )
#define MCU_DMA2_HISR_DMEIF4                  ( 0x4U )
#define MCU_DMA2_HISR_FEIF4                   ( 0x1U )

/* Биты регистра LIFCR (low interrupt flag clear register) */
#define MCU_DMA2_LIFCR_BITS                   ( 0xF7D0F7DU )
#define MCU_DMA2_LIFCR_CTCIF3                 ( 0x8000000U )
#define MCU_DMA2_LIFCR_CHTIF3                 ( 0x4000000U )
#define MCU_DMA2_LIFCR_CTEIF3                 ( 0x2000000U )
#define MCU_DMA2_LIFCR_CDMEIF3                ( 0x1000000U )
#define MCU_DMA2_LIFCR_CFEIF3                 ( 0x400000U )
#define MCU_DMA2_LIFCR_CTCIF2                 ( 0x200000U )
#define MCU_DMA2_LIFCR_CHTIF2                 ( 0x100000U )
#define MCU_DMA2_LIFCR_CTEIF2                 ( 0x80000U )
#define MCU_DMA2_LIFCR_CDMEIF2                ( 0x40000U )
#define MCU_DMA2_LIFCR_CFEIF2                 ( 0x10000U )
#define MCU_DMA2_LIFCR_CTCIF1                 ( 0x800U )
#define MCU_DMA2_LIFCR_CHTIF1                 ( 0x400U )
#define MCU_DMA2_LIFCR_CTEIF1                 ( 0x200U )
#define MCU_DMA2_LIFCR_CDMEIF1                ( 0x100U )
#define MCU_DMA2_LIFCR_CFEIF1                 ( 0x40U )
#define MCU_DMA2_LIFCR_CTCIF0                 ( 0x20U )
#define MCU_DMA2_LIFCR_CHTIF0                 ( 0x10U )
#define MCU_DMA2_LIFCR_CTEIF0                 ( 0x8U )
#define MCU_DMA2_LIFCR_CDMEIF0                ( 0x4U )
#define MCU_DMA2_LIFCR_CFEIF0                 ( 0x1U )

/* Биты регистра HIFCR (high interrupt flag clear register) */
#define MCU_DMA2_HIFCR_BITS                   ( 0xF7D0F7DU )
#define MCU_DMA2_HIFCR_CTCIF7                 ( 0x8000000U )
#define MCU_DMA2_HIFCR_CHTIF7                 ( 0x4000000U )
#define MCU_DMA2_HIFCR_CTEIF7                 ( 0x2000000U )
#define MCU_DMA2_HIFCR_CDMEIF7                ( 0x1000000U )
#define MCU_DMA2_HIFCR_CFEIF7                 ( 0x400000U )
#define MCU_DMA2_HIFCR_CTCIF6                 ( 0x200000U )
#define MCU_DMA2_HIFCR_CHTIF6                 ( 0x100000U )
#define MCU_DMA2_HIFCR_CTEIF6                 ( 0x80000U )
#define MCU_DMA2_HIFCR_CDMEIF6                ( 0x40000U )
#define MCU_DMA2_HIFCR_CFEIF6                 ( 0x10000U )
#define MCU_DMA2_HIFCR_CTCIF5                 ( 0x800U )
#define MCU_DMA2_HIFCR_CHTIF5                 ( 0x400U )
#define MCU_DMA2_HIFCR_CTEIF5                 ( 0x200U )
#define MCU_DMA2_HIFCR_CDMEIF5                ( 0x100U )
#define MCU_DMA2_HIFCR_CFEIF5                 ( 0x40U )
#define MCU_DMA2_HIFCR_CTCIF4                 ( 0x20U )
#define MCU_DMA2_HIFCR_CHTIF4                 ( 0x10U )
#define MCU_DMA2_HIFCR_CTEIF4                 ( 0x8U )
#define MCU_DMA2_HIFCR_CDMEIF4                ( 0x4U )
#define MCU_DMA2_HIFCR_CFEIF4                 ( 0x1U )

/* Биты регистра S0CR (stream 0 configuration register) */
#define MCU_DMA2_S0CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA2_S0CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA2_S0CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA2_S0CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA2_S0CR_CT                      ( 0x80000U )
#define MCU_DMA2_S0CR_DBM                     ( 0x40000U )
#define MCU_DMA2_S0CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA2_S0CR_PINCOS                  ( 0x8000U )
#define MCU_DMA2_S0CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA2_S0CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA2_S0CR_MINC                    ( 0x400U )
#define MCU_DMA2_S0CR_PINC                    ( 0x200U )
#define MCU_DMA2_S0CR_CIRC                    ( 0x100U )
#define MCU_DMA2_S0CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA2_S0CR_PFCTRL                  ( 0x20U )
#define MCU_DMA2_S0CR_TCIE                    ( 0x10U )
#define MCU_DMA2_S0CR_HTIE                    ( 0x8U )
#define MCU_DMA2_S0CR_TEIE                    ( 0x4U )
#define MCU_DMA2_S0CR_DMEIE                   ( 0x2U )
#define MCU_DMA2_S0CR_EN                      ( 0x1U )

/* Биты регистра S0NDTR (stream 0 number of data register) */
#define MCU_DMA2_S0NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA2_S0NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S0PAR (stream 0 peripheral address register) */
#define MCU_DMA2_S0PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA2_S0PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S0M0AR (stream 0 memory 0 address register) */
#define MCU_DMA2_S0M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S0M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S0M1AR (tream 0 memory 1 address register) */
#define MCU_DMA2_S0M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S0M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S0FCR (stream 0 FIFO control register) */
#define MCU_DMA2_S0FCR_BITS                   ( 0xBFU )
#define MCU_DMA2_S0FCR_FEIE                   ( 0x80U )
#define MCU_DMA2_S0FCR_FS_MASK                ( 0x38U )
#define MCU_DMA2_S0FCR_DMDIS                  ( 0x4U )
#define MCU_DMA2_S0FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S1CR (stream 1 configuration register) */
#define MCU_DMA2_S1CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA2_S1CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA2_S1CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA2_S1CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA2_S1CR_CT                      ( 0x80000U )
#define MCU_DMA2_S1CR_DBM                     ( 0x40000U )
#define MCU_DMA2_S1CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA2_S1CR_PINCOS                  ( 0x8000U )
#define MCU_DMA2_S1CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA2_S1CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA2_S1CR_MINC                    ( 0x400U )
#define MCU_DMA2_S1CR_PINC                    ( 0x200U )
#define MCU_DMA2_S1CR_CIRC                    ( 0x100U )
#define MCU_DMA2_S1CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA2_S1CR_PFCTRL                  ( 0x20U )
#define MCU_DMA2_S1CR_TCIE                    ( 0x10U )
#define MCU_DMA2_S1CR_HTIE                    ( 0x8U )
#define MCU_DMA2_S1CR_TEIE                    ( 0x4U )
#define MCU_DMA2_S1CR_DMEIE                   ( 0x2U )
#define MCU_DMA2_S1CR_EN                      ( 0x1U )

/* Биты регистра S1NDTR (stream 1 number of data register) */
#define MCU_DMA2_S1NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA2_S1NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S1PAR (stream 1 peripheral address register) */
#define MCU_DMA2_S1PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA2_S1PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S1M0AR (stream 1 memory 0 address register) */
#define MCU_DMA2_S1M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S1M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S1M1AR (stream 1 memory 1 address register) */
#define MCU_DMA2_S1M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S1M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S1FCR (stream 1 FIFO control register) */
#define MCU_DMA2_S1FCR_BITS                   ( 0xBFU )
#define MCU_DMA2_S1FCR_FEIE                   ( 0x80U )
#define MCU_DMA2_S1FCR_FS_MASK                ( 0x38U )
#define MCU_DMA2_S1FCR_DMDIS                  ( 0x4U )
#define MCU_DMA2_S1FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S2CR (stream 2 configuration register) */
#define MCU_DMA2_S2CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA2_S2CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA2_S2CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA2_S2CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA2_S2CR_CT                      ( 0x80000U )
#define MCU_DMA2_S2CR_DBM                     ( 0x40000U )
#define MCU_DMA2_S2CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA2_S2CR_PINCOS                  ( 0x8000U )
#define MCU_DMA2_S2CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA2_S2CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA2_S2CR_MINC                    ( 0x400U )
#define MCU_DMA2_S2CR_PINC                    ( 0x200U )
#define MCU_DMA2_S2CR_CIRC                    ( 0x100U )
#define MCU_DMA2_S2CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA2_S2CR_PFCTRL                  ( 0x20U )
#define MCU_DMA2_S2CR_TCIE                    ( 0x10U )
#define MCU_DMA2_S2CR_HTIE                    ( 0x8U )
#define MCU_DMA2_S2CR_TEIE                    ( 0x4U )
#define MCU_DMA2_S2CR_DMEIE                   ( 0x2U )
#define MCU_DMA2_S2CR_EN                      ( 0x1U )

/* Биты регистра S2NDTR (stream 2 number of data register) */
#define MCU_DMA2_S2NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA2_S2NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S2PAR (stream 2 peripheral address register) */
#define MCU_DMA2_S2PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA2_S2PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S2M0AR (stream 2 memory 0 address register) */
#define MCU_DMA2_S2M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S2M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S2M1AR (stream 2 memory 1 address register) */
#define MCU_DMA2_S2M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S2M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S2FCR (stream 2 FIFO control register) */
#define MCU_DMA2_S2FCR_BITS                   ( 0xBFU )
#define MCU_DMA2_S2FCR_FEIE                   ( 0x80U )
#define MCU_DMA2_S2FCR_FS_MASK                ( 0x38U )
#define MCU_DMA2_S2FCR_DMDIS                  ( 0x4U )
#define MCU_DMA2_S2FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S3CR (stream 3 configuration register) */
#define MCU_DMA2_S3CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA2_S3CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA2_S3CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA2_S3CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA2_S3CR_CT                      ( 0x80000U )
#define MCU_DMA2_S3CR_DBM                     ( 0x40000U )
#define MCU_DMA2_S3CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA2_S3CR_PINCOS                  ( 0x8000U )
#define MCU_DMA2_S3CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA2_S3CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA2_S3CR_MINC                    ( 0x400U )
#define MCU_DMA2_S3CR_PINC                    ( 0x200U )
#define MCU_DMA2_S3CR_CIRC                    ( 0x100U )
#define MCU_DMA2_S3CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA2_S3CR_PFCTRL                  ( 0x20U )
#define MCU_DMA2_S3CR_TCIE                    ( 0x10U )
#define MCU_DMA2_S3CR_HTIE                    ( 0x8U )
#define MCU_DMA2_S3CR_TEIE                    ( 0x4U )
#define MCU_DMA2_S3CR_DMEIE                   ( 0x2U )
#define MCU_DMA2_S3CR_EN                      ( 0x1U )

/* Биты регистра S3NDTR (stream 3 number of data register) */
#define MCU_DMA2_S3NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA2_S3NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S3PAR (stream 3 peripheral address register) */
#define MCU_DMA2_S3PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA2_S3PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S3M0AR (stream 3 memory 0 address register) */
#define MCU_DMA2_S3M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S3M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S3M1AR (stream 3 memory 1 address register) */
#define MCU_DMA2_S3M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S3M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S3FCR (stream 3 FIFO control register) */
#define MCU_DMA2_S3FCR_BITS                   ( 0xBFU )
#define MCU_DMA2_S3FCR_FEIE                   ( 0x80U )
#define MCU_DMA2_S3FCR_FS_MASK                ( 0x38U )
#define MCU_DMA2_S3FCR_DMDIS                  ( 0x4U )
#define MCU_DMA2_S3FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S4CR (stream 4 configuration register) */
#define MCU_DMA2_S4CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA2_S4CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA2_S4CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA2_S4CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA2_S4CR_CT                      ( 0x80000U )
#define MCU_DMA2_S4CR_DBM                     ( 0x40000U )
#define MCU_DMA2_S4CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA2_S4CR_PINCOS                  ( 0x8000U )
#define MCU_DMA2_S4CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA2_S4CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA2_S4CR_MINC                    ( 0x400U )
#define MCU_DMA2_S4CR_PINC                    ( 0x200U )
#define MCU_DMA2_S4CR_CIRC                    ( 0x100U )
#define MCU_DMA2_S4CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA2_S4CR_PFCTRL                  ( 0x20U )
#define MCU_DMA2_S4CR_TCIE                    ( 0x10U )
#define MCU_DMA2_S4CR_HTIE                    ( 0x8U )
#define MCU_DMA2_S4CR_TEIE                    ( 0x4U )
#define MCU_DMA2_S4CR_DMEIE                   ( 0x2U )
#define MCU_DMA2_S4CR_EN                      ( 0x1U )

/* Биты регистра S4NDTR (stream 4 number of data register) */
#define MCU_DMA2_S4NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA2_S4NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S4PAR (stream 4 peripheral address register) */
#define MCU_DMA2_S4PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA2_S4PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S4M0AR (stream 4 memory 0 address register) */
#define MCU_DMA2_S4M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S4M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S4M1AR (stream 4 memory 1 address register) */
#define MCU_DMA2_S4M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S4M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S4FCR (stream 4 FIFO control register) */
#define MCU_DMA2_S4FCR_BITS                   ( 0xBFU )
#define MCU_DMA2_S4FCR_FEIE                   ( 0x80U )
#define MCU_DMA2_S4FCR_FS_MASK                ( 0x38U )
#define MCU_DMA2_S4FCR_DMDIS                  ( 0x4U )
#define MCU_DMA2_S4FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S5CR (stream 5 configuration register) */
#define MCU_DMA2_S5CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA2_S5CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA2_S5CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA2_S5CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA2_S5CR_CT                      ( 0x80000U )
#define MCU_DMA2_S5CR_DBM                     ( 0x40000U )
#define MCU_DMA2_S5CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA2_S5CR_PINCOS                  ( 0x8000U )
#define MCU_DMA2_S5CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA2_S5CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA2_S5CR_MINC                    ( 0x400U )
#define MCU_DMA2_S5CR_PINC                    ( 0x200U )
#define MCU_DMA2_S5CR_CIRC                    ( 0x100U )
#define MCU_DMA2_S5CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA2_S5CR_PFCTRL                  ( 0x20U )
#define MCU_DMA2_S5CR_TCIE                    ( 0x10U )
#define MCU_DMA2_S5CR_HTIE                    ( 0x8U )
#define MCU_DMA2_S5CR_TEIE                    ( 0x4U )
#define MCU_DMA2_S5CR_DMEIE                   ( 0x2U )
#define MCU_DMA2_S5CR_EN                      ( 0x1U )

/* Биты регистра S5NDTR (stream 5 number of data register) */
#define MCU_DMA2_S5NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA2_S5NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S5PAR (stream 5 peripheral address register) */
#define MCU_DMA2_S5PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA2_S5PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S5M0AR (stream 5 memory 0 address register) */
#define MCU_DMA2_S5M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S5M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S5M1AR (stream 5 memory 1 address register) */
#define MCU_DMA2_S5M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S5M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S5FCR (stream 5 FIFO control register) */
#define MCU_DMA2_S5FCR_BITS                   ( 0xBFU )
#define MCU_DMA2_S5FCR_FEIE                   ( 0x80U )
#define MCU_DMA2_S5FCR_FS_MASK                ( 0x38U )
#define MCU_DMA2_S5FCR_DMDIS                  ( 0x4U )
#define MCU_DMA2_S5FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S6CR (stream 6 configuration register) */
#define MCU_DMA2_S6CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA2_S6CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA2_S6CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA2_S6CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA2_S6CR_CT                      ( 0x80000U )
#define MCU_DMA2_S6CR_DBM                     ( 0x40000U )
#define MCU_DMA2_S6CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA2_S6CR_PINCOS                  ( 0x8000U )
#define MCU_DMA2_S6CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA2_S6CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA2_S6CR_MINC                    ( 0x400U )
#define MCU_DMA2_S6CR_PINC                    ( 0x200U )
#define MCU_DMA2_S6CR_CIRC                    ( 0x100U )
#define MCU_DMA2_S6CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA2_S6CR_PFCTRL                  ( 0x20U )
#define MCU_DMA2_S6CR_TCIE                    ( 0x10U )
#define MCU_DMA2_S6CR_HTIE                    ( 0x8U )
#define MCU_DMA2_S6CR_TEIE                    ( 0x4U )
#define MCU_DMA2_S6CR_DMEIE                   ( 0x2U )
#define MCU_DMA2_S6CR_EN                      ( 0x1U )

/* Биты регистра S6NDTR (stream 6 number of data register) */
#define MCU_DMA2_S6NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA2_S6NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра S6PAR (stream 6 peripheral address register) */
#define MCU_DMA2_S6PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA2_S6PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S6M0AR (stream 6 memory 0 address register) */
#define MCU_DMA2_S6M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S6M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S6M1AR (stream 6 memory 1 address register) */
#define MCU_DMA2_S6M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S6M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S6FCR (stream 6 FIFO control register) */
#define MCU_DMA2_S6FCR_BITS                   ( 0xBFU )
#define MCU_DMA2_S6FCR_FEIE                   ( 0x80U )
#define MCU_DMA2_S6FCR_FS_MASK                ( 0x38U )
#define MCU_DMA2_S6FCR_DMDIS                  ( 0x4U )
#define MCU_DMA2_S6FCR_FTH_MASK               ( 0x3U )

/* Биты регистра S7CR (stream 7 configuration register) */
#define MCU_DMA2_S7CR_BITS                    ( 0xFEFFFFFU )
#define MCU_DMA2_S7CR_CHSEL_MASK              ( 0xE000000U )
#define MCU_DMA2_S7CR_MBURST_MASK             ( 0x1800000U )
#define MCU_DMA2_S7CR_PBURST_MASK             ( 0x600000U )
#define MCU_DMA2_S7CR_CT                      ( 0x80000U )
#define MCU_DMA2_S7CR_DBM                     ( 0x40000U )
#define MCU_DMA2_S7CR_PL_MASK                 ( 0x30000U )
#define MCU_DMA2_S7CR_PINCOS                  ( 0x8000U )
#define MCU_DMA2_S7CR_MSIZE_MASK              ( 0x6000U )
#define MCU_DMA2_S7CR_PSIZE_MASK              ( 0x1800U )
#define MCU_DMA2_S7CR_MINC                    ( 0x400U )
#define MCU_DMA2_S7CR_PINC                    ( 0x200U )
#define MCU_DMA2_S7CR_CIRC                    ( 0x100U )
#define MCU_DMA2_S7CR_DIR_MASK                ( 0xC0U )
#define MCU_DMA2_S7CR_PFCTRL                  ( 0x20U )
#define MCU_DMA2_S7CR_TCIE                    ( 0x10U )
#define MCU_DMA2_S7CR_HTIE                    ( 0x8U )
#define MCU_DMA2_S7CR_TEIE                    ( 0x4U )
#define MCU_DMA2_S7CR_DMEIE                   ( 0x2U )
#define MCU_DMA2_S7CR_EN                      ( 0x1U )

/* Биты регистра S7NDTR (stream 7 number of data register) */
#define MCU_DMA2_S7NDTR_BITS                  (	0xFFFFU )
#define MCU_DMA2_S7NDTR_NDT_MASK              ( 0xFFFFU )

/* Биты регистра SPAR (stream 7 peripheral address register) */
#define MCU_DMA2_S7PAR_BITS                   ( 0xFFFFFFFFU )
#define MCU_DMA2_S7PAR_PAR_MASK               ( 0xFFFFFFFFU )

/* Биты регистра S7M0AR (stream 7 memory 0 address register) */
#define MCU_DMA2_S7M0AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S7M0AR_M0A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра S7M1AR (stream 7 memory 1 address register) */
#define MCU_DMA2_S7M1AR_BITS                  ( 0xFFFFFFFFU )
#define MCU_DMA2_S7M1AR_M1A_MASK              ( 0xFFFFFFFFU )

/* Биты регистра MCU_DMA2_S7FCR (stream 7 FIFO control register) */
#define MCU_DMA2_S7FCR_BITS                   ( 0xBFU )
#define MCU_DMA2_S7FCR_FEIE                   ( 0x80U )
#define MCU_DMA2_S7FCR_FS_MASK                ( 0x38U )
#define MCU_DMA2_S7FCR_DMDIS                  ( 0x4U )
#define MCU_DMA2_S7FCR_FTH_MASK               ( 0x3U )


/* ----- EXTI (external interrupt/event controller) ----- */
typedef volatile struct tag_stm32f407_exti {
    uint32_t        imr;
    uint32_t        emr;
    uint32_t        rtsr;
    uint32_t        ftsr;
    uint32_t        swier;
    uint32_t        pr;
} stm32f407_exti_t;

#define STM32F407_EXTI_PTR                    ( ( stm32f407_exti_t * ) 0x40013C00U )

/* Биты регистра EXTI_IMR (interrupt mask register) */
#define MCU_EXTI_IMR_RESET_BITS               ( 0xFFFFFFFFU )
#define MCU_EXTI_IMR_BITS                     ( 0x7FFFFFU )
#define MCU_EXTI_IMR_MR_MASK                  ( 0x7FFFFFU )

/* Биты регистра EXTI_EMR (event mask register) */
#define MCU_EXTI_EMR_BITS                     ( 0x7FFFFFU )
#define MCU_EXTI_EMR_MR_MASK                  ( 0x7FFFFFU )

/* Биты регистра EXTI_RTSR (rising trigger selection register) */
#define MCU_EXTI_RTSR_BITS                    ( 0x7FFFFFU )
#define MCU_EXTI_RTSR_MR_MASK                 ( 0x7FFFFFU )

/* Биты регистра EXTI_FTSR (falling trigger selection register) */
#define MCU_EXTI_FTSR_RESET_BITS              ( 0xFFFFFFFFU )
#define MCU_EXTI_FTSR_BITS                    ( 0x7FFFFFU )
#define MCU_EXTI_FTSR_MR_MASK                 ( 0x7FFFFFU )

/* Биты регистра EXTI_SWIER (software interrupt event register) */
#define MCU_EXTI_SWIER_BITS                   ( 0x7FFFFFU )
#define MCU_EXTI_SWIER_MR_MASK                ( 0x7FFFFFU )

/* Биты регистра EXTI_PR (pending register) */
#define MCU_EXTI_PR_RESET_BITS              ( 0xFFFFFFFFU )
#define MCU_EXTI_PR_BITS                      ( 0x7FFFFFU )
#define MCU_EXTI_PR_MR_MASK                   ( 0x7FFFFFU )


/* ----- DCMI (digital camera interface) ----- */
typedef volatile struct tag_stm32f407_dcmi {
    uint32_t        cr;
    uint32_t        sr;
    uint32_t        ris;
    uint32_t        ier;
    uint32_t        mis;
    uint32_t        icr;
    uint32_t        escr;
    uint32_t        esur;
    uint32_t        cwstrt;
    uint32_t        cwsize;
    uint32_t        dr;
} stm32f407_dcmi_t;

#define STM32F407_DCMI_PTR                    ( ( stm32f407_dcmi_t * ) 0x50050000U )

/* Биты регистра DCMI_CR (DCMI control register 1) */
#define MCU_DCMI_CR_BITS                      ( 0x4FFFU )
#define MCU_DCMI_CR_ENABLE                    ( 0x4000U )
#define MCU_DCMI_CR_EDM                       ( 0xC00U )
#define MCU_DCMI_CR_FCRC                      ( 0x300U )
#define MCU_DCMI_CR_VSPOL                     ( 0x80U )
#define MCU_DCMI_CR_HSPOL                     ( 0x40U )
#define MCU_DCMI_CR_PCKPOL                    ( 0x20U )
#define MCU_DCMI_CR_ESS                       ( 0x10U )
#define MCU_DCMI_CR_JPEG                      ( 0x8U )
#define MCU_DCMI_CR_CROP                      ( 0x4U )
#define MCU_DCMI_CR_CM                        ( 0x2U )
#define MCU_DCMI_CR_CAPTURE                   ( 0x1U )

/* Биты регистра DCMI_SR (DCMI status register 1) */
#define MCU_DCMI_SR_BITS                      ( 0x7U )
#define MCU_DCMI_SR_FNE                       ( 0x4U )
#define MCU_DCMI_SR_VSYNC                     ( 0x2U )
#define MCU_DCMI_SR_HSYNC                     ( 0x1U )

/* Биты регистра DCMI_RIS (DCMI status register 1) */
#define MCU_DCMI_RIS_BITS                     ( 0x1FU )
#define MCU_DCMI_RIS_LINE_RIS                 ( 0x10U )
#define MCU_DCMI_RIS_VSYNC_RIS                ( 0x8U )
#define MCU_DCMI_RIS_ERR_RIS                  ( 0x4U )
#define MCU_DCMI_RIS_OVR_RIS                  ( 0x2U )
#define MCU_DCMI_RIS_FRAME_RIS                ( 0x1U )

/* Биты регистра DCMI_IER (DCMI interrupt enable register) */
#define MCU_DCMI_IER_BITS                     ( 0x1FU )
#define MCU_DCMI_IER_LINE_IE                  ( 0x10U )
#define MCU_DCMI_IER_VSYNC_IE                 ( 0x8U )
#define MCU_DCMI_IER_ERR_IE                   ( 0x4U )
#define MCU_DCMI_IER_OVR_IE                   ( 0x2U )
#define MCU_DCMI_IER_FRAME_IE                 ( 0x1U )

/* Биты регистра DCMI_MIS (DCMI masked interrupt status register) */
#define MCU_DCMI_MIS_BITS                     ( 0x1FU )
#define MCU_DCMI_MIS_LINE_MIS                 ( 0x10U )
#define MCU_DCMI_MIS_VSYNC_MIS                ( 0x8U )
#define MCU_DCMI_MIS_ERR_MIS                  ( 0x4U )
#define MCU_DCMI_MIS_OVR_MIS                  ( 0x2U )
#define MCU_DCMI_MIS_FRAME_MIS                ( 0x1U )

/* Биты регистра DCMI_ICR (DCMI interrupt clear register) */
#define MCU_DCMI_ICR_BITS                     ( 0x1FU )
#define MCU_DCMI_ICR_LINE_ISC                 ( 0x10U )
#define MCU_DCMI_ICR_VSYNC_ISC                ( 0x8U )
#define MCU_DCMI_ICR_ERR_ISC                  ( 0x4U )
#define MCU_DCMI_ICR_OVR_ISC                  ( 0x2U )
#define MCU_DCMI_ICR_FRAME_ISC                ( 0x1U )

/* Биты регистра DCMI_ESCR (DCMI embedded synchronization code register) */
#define MCU_DCMI_ESCR_BITS                    ( 0xFFFFFFFFU )
#define MCU_DCMI_ESCR_FEC                     ( 0xFF000000U )
#define MCU_DCMI_ESCR_LEC                     ( 0xFF0000U )
#define MCU_DCMI_ESCR_LSC                     ( 0xFF00U )
#define MCU_DCMI_ESCR_FSC                     ( 0xFFU )

/* Биты регистра DCMI_ESUR (DCMI embedded synchronization unmask register) */
#define MCU_DCMI_ESUR_BITS                    ( 0xFFFFFFFFU )
#define MCU_DCMI_ESUR_FEU                     ( 0xFF000000U )
#define MCU_DCMI_ESUR_LEU                     ( 0xFF0000U )
#define MCU_DCMI_ESUR_LSU                     ( 0xFF00U )
#define MCU_DCMI_ESUR_FSU                     ( 0xFFU )

/* Биты регистра DCMI_CWSTRT (DCMI crop window start) */
#define MCU_DCMI_CWSTRT_BITS                  ( 0x1FFF3FFFU )
#define MCU_DCMI_CWSTRT_VST                   ( 0x1FFF0000U )
#define MCU_DCMI_CWSTRT_HOFFCNT               ( 0x3FFFU )

/* Биты регистра DCMI_CWSIZE (DCMI crop window size) */
#define MCU_DCMI_CWSIZE_BITS                  ( 0x3FFF3FFFU )
#define MCU_DCMI_CWSIZE_VLINE                 ( 0x3FFF0000U )
#define MCU_DCMI_CWSIZE_CAPCNT                ( 0x3FFFU )

/* Биты регистра DCMI_DR (DCMI data register) */
#define MCU_DCMI_DR_BITS                      ( 0xFFFFFFFFU )
#define MCU_DCMI_DR_BYTE3                     ( 0xFF000000U )
#define MCU_DCMI_DR_BYTE2                     ( 0xFF0000U )
#define MCU_DCMI_DR_BYTE1                     ( 0xFF00U )
#define MCU_DCMI_DR_BYTE0                     ( 0xFFU )


/* ----- ACT (advanced-control timers (TIM1 AND TIM8)) ----- */
typedef volatile struct tag_stm32f407_act {
    uint32_t        cr1;
    uint32_t        cr2;
    uint32_t        smcr;
    uint32_t        dier;
    uint32_t        sr;
    uint32_t        egr;
    uint32_t        ccmr1_ocm;
    uint32_t        ccmr1_icm;
    uint32_t        ccmr2_ocm;
    uint32_t        ccmr2_icm;
    uint32_t        ccer;
    uint32_t        cnt;
    uint32_t        psc;
    uint32_t        arr;
    uint32_t        rcr;
    uint32_t        ccrx[4];
    uint32_t        bdtr;
    uint32_t        dcr;
    uint32_t        dmar;
} stm32f407_act_t;

#define STM32F407_TIM1_PTR                    ( ( stm32f407_gpt_t * ) 0x40010000U )
#define STM32F407_TIM8_PTR                    ( ( stm32f407_gpt_t * ) 0x40010400U )

/* Биты регистра CR1 (control register 1) */
#define MCU_ACT_CR1_BITS                      ( 0x3FFU )
#define MCU_ACT_CR1_CKD_RESERVED              ( 0x300U )
#define MCU_ACT_CR1_CKD_4CK_INT               ( 0x200U )
#define MCU_ACT_CR1_CKD_2CK_INT               ( 0x100U )
#define MCU_ACT_CR1_CKD_CK_INT                ( 0x0U )
#define MCU_ACT_CR1_CKD_MASK                  ( 0x300U )
#define MCU_ACT_CR1_ARPE                      ( 0x80U )
#define MCU_ACT_CR1_CMS_CENTRAL_ALIGNED_3     ( 0x60U )
#define MCU_ACT_CR1_CMS_CENTRAL_ALIGNED_2     ( 0x40U )
#define MCU_ACT_CR1_CMS_CENTRAL_ALIGNED_1     ( 0x20U )
#define MCU_ACT_CR1_CMS_EDGE_ALIGNED          ( 0x0U )
#define MCU_ACT_CR1_CMS_MASK                  ( 0x60U )
#define MCU_ACT_CR1_DIR                       ( 0x10U )
#define MCU_ACT_CR1_OPM                       ( 0x8U )
#define MCU_ACT_CR1_URS                       ( 0x4U )
#define MCU_ACT_CR1_UDIS                      ( 0x2U )
#define MCU_ACT_CR1_CEN                       ( 0x1U )

/* Биты регистра CR2 (control register 2) */
#define MCU_ACT_CR2_BITS                      ( 0x7FFDU )
#define MCU_ACT_CR2_OIS4                      ( 0x4000U )
#define MCU_ACT_CR2_OIS3N                     ( 0x2000U )
#define MCU_ACT_CR2_OIS3                      ( 0x1000U )
#define MCU_ACT_CR2_OIS2N                     ( 0x800U )
#define MCU_ACT_CR2_OIS2                      ( 0x400U )
#define MCU_ACT_CR2_OIS1N                     ( 0x200U )
#define MCU_ACT_CR2_OIS1                      ( 0x100U )
#define MCU_ACT_CR2_TI1S                      ( 0x80U )
#define MCU_ACT_CR2_MMS_COMPARE_OC4REF        ( 0x70U )
#define MCU_ACT_CR2_MMS_COMPARE_OC3REF        ( 0x60U )
#define MCU_ACT_CR2_MMS_COMPARE_OC2REF        ( 0x50U )
#define MCU_ACT_CR2_MMS_COMPARE_OC1REF        ( 0x40U )
#define MCU_ACT_CR2_MMS_COMPARE_PULSE         ( 0x30U )
#define MCU_ACT_CR2_MMS_UPDATE                ( 0x20U )
#define MCU_ACT_CR2_MMS_ENABLE                ( 0x10U )
#define MCU_ACT_CR2_MMS_RESET                 ( 0x0U )
#define MCU_ACT_CR2_MMS_MASK                  ( 0x70U )
#define MCU_ACT_CR2_CCDS                      ( 0x8U )
#define MCU_ACT_CR2_CCUS                      ( 0x4U )
#define MCU_ACT_CR2_CCPC                      ( 0x1U )

/* Биты регистра SMCR (slave mode control register) */
#define MCU_ACT_SMCR_BITS                     ( 0xFFF7U )
#define MCU_ACT_SMCR_ETP                      ( 0x8000U )
#define MCU_ACT_SMCR_ECE                      ( 0x4000U )
#define MCU_ACT_SMCR_ETPS_DIV_8               ( 0x3000U )
#define MCU_ACT_SMCR_ETPS_DIV_4               ( 0x2000U )
#define MCU_ACT_SMCR_ETPS_DIV_2               ( 0x1000U )
#define MCU_ACT_SMCR_ETPS_OFF                 ( 0x0U )
#define MCU_ACT_SMCR_ETPS_MASK                ( 0x3000U )
#define MCU_ACT_SMCR_ETF_DTS_32_8             ( 0xF00U )
#define MCU_ACT_SMCR_ETF_DTS_32_6             ( 0xE00U )
#define MCU_ACT_SMCR_ETF_DTS_32_5             ( 0xD00U )
#define MCU_ACT_SMCR_ETF_DTS_16_8             ( 0xC00U )
#define MCU_ACT_SMCR_ETF_DTS_16_6             ( 0xB00U )
#define MCU_ACT_SMCR_ETF_DTS_16_5             ( 0xA00U )
#define MCU_ACT_SMCR_ETF_DTS_8_8              ( 0x900U )
#define MCU_ACT_SMCR_ETF_DTS_8_6              ( 0x800U )
#define MCU_ACT_SMCR_ETF_DTS_4_8              ( 0x700U )
#define MCU_ACT_SMCR_ETF_DTS_4_6              ( 0x600U )
#define MCU_ACT_SMCR_ETF_DTS_2_8              ( 0x500U )
#define MCU_ACT_SMCR_ETF_DTS_2_6              ( 0x400U )
#define MCU_ACT_SMCR_ETF_CK_INT_8             ( 0x300U )
#define MCU_ACT_SMCR_ETF_CK_INT_4             ( 0x200U )
#define MCU_ACT_SMCR_ETF_CK_INT_2             ( 0x100U )
#define MCU_ACT_SMCR_ETF_NO_FILTER            ( 0x0U )
#define MCU_ACT_SMCR_ETF_MASK                 ( 0xF00U )
#define MCU_ACT_SMCR_MSM                      ( 0x80U )
#define MCU_ACT_SMCR_TS_EXT_TRIG_INPUT        ( 0x70U )
#define MCU_ACT_SMCR_TS_FILT_INPUT2           ( 0x60U )
#define MCU_ACT_SMCR_TS_FILT_INPUT1           ( 0x50U )
#define MCU_ACT_SMCR_TS_TI1_EDGE              ( 0x40U )
#define MCU_ACT_SMCR_TS_TRIG_3                ( 0x30U )
#define MCU_ACT_SMCR_TS_TRIG_2                ( 0x20U )
#define MCU_ACT_SMCR_TS_TRIG_1                ( 0x10U )
#define MCU_ACT_SMCR_TS_TRIG_0                ( 0x0U )
#define MCU_ACT_SMCR_TS_MASK                  ( 0x70U )
#define MCU_ACT_SMCR_SMS_EXTERNAL             ( 0x7U )
#define MCU_ACT_SMCR_SMS_TRIGGER              ( 0x6U )
#define MCU_ACT_SMCR_SMS_GATED                ( 0x5U )
#define MCU_ACT_SMCR_SMS_RESET                ( 0x4U )
#define MCU_ACT_SMCR_SMS_ENC_3                ( 0x3U )
#define MCU_ACT_SMCR_SMS_ENC_2                ( 0x2U )
#define MCU_ACT_SMCR_SMS_ENC_1                ( 0x1U )
#define MCU_ACT_SMCR_SMS_DISABLED             ( 0x0U )
#define MCU_ACT_SMCR_SMS_MASK                 ( 0x7U )

/* Биты регистра DIER (DMA/interrupt enable register) */
#define MCU_ACT_DIER_BITS                     ( 0x7FFFU )
#define MCU_ACT_DIER_TDE                      ( 0x4000U )
#define MCU_ACT_DIER_COMDE                    ( 0x2000U )
#define MCU_ACT_DIER_CC4DE                    ( 0x1000U )
#define MCU_ACT_DIER_CC3DE                    ( 0x800U )
#define MCU_ACT_DIER_CC2DE                    ( 0x400U )
#define MCU_ACT_DIER_CC1DE                    ( 0x200U )
#define MCU_ACT_DIER_UDE                      ( 0x100U )
#define MCU_ACT_DIER_BIE                      ( 0x80U )
#define MCU_ACT_DIER_TIE                      ( 0x40U )
#define MCU_ACT_DIER_COMIE                    ( 0x20U )
#define MCU_ACT_DIER_CC4IE                    ( 0x10U )
#define MCU_ACT_DIER_CC3IE                    ( 0x8U )
#define MCU_ACT_DIER_CC2IE                    ( 0x4U )
#define MCU_ACT_DIER_CC1IE                    ( 0x2U )
#define MCU_ACT_DIER_UIE                      ( 0x1U )

/* Биты регистра SR (status register) */
#define MCU_ACT_SR_BITS                       ( 0x1EFFU )
#define MCU_ACT_SR_CC4OF                      ( 0x1000U )
#define MCU_ACT_SR_CC3OF                      ( 0x800U )
#define MCU_ACT_SR_CC2OF                      ( 0x400U )
#define MCU_ACT_SR_CC1OF                      ( 0x200U )
#define MCU_ACT_SR_BIF                        ( 0x80U )
#define MCU_ACT_SR_TIF                        ( 0x40U )
#define MCU_ACT_SR_COMIF                      ( 0x20U )
#define MCU_ACT_SR_CC4IF                      ( 0x10U )
#define MCU_ACT_SR_CC3IF                      ( 0x8U )
#define MCU_ACT_SR_CC2IF                      ( 0x4U )
#define MCU_ACT_SR_CC1IF                      ( 0x2U )
#define MCU_ACT_SR_UIF                        ( 0x1U )

/* Биты регистра EGR (event generation register) */
#define MCU_ACT_EGR_BITS                      ( 0xFFU )
#define MCU_ACT_EGR_BG                        ( 0x80U )
#define MCU_ACT_EGR_TG                        ( 0x40U )
#define MCU_ACT_EGR_COMG                      ( 0x20U )
#define MCU_ACT_EGR_CC4G                      ( 0x10U )
#define MCU_ACT_EGR_CC3G                      ( 0x8U )
#define MCU_ACT_EGR_CC2G                      ( 0x4U )
#define MCU_ACT_EGR_CC1G                      ( 0x2U )
#define MCU_ACT_EGR_UG                        ( 0x1U )

/* Биты регистра CCMR1_OCM (output compare mode register 1) */
#define MCU_ACT_CCMR1_OCM_BITS                ( 0xFFFFU )
#define MCU_ACT_CCMR1_OCM_OC2CE               ( 0x8000U )
#define MCU_ACT_CCMR1_OCM_OC2M_MASK           ( 0x7000U )
#define MCU_ACT_CCMR1_OCM_OC2PE               ( 0x800U )
#define MCU_ACT_CCMR1_OCM_OC2FE               ( 0x400U )
#define MCU_ACT_CCMR1_OCM_CC2S_TRC            ( 0x300U )
#define MCU_ACT_CCMR1_OCM_CC2S_TI1            ( 0x200U )
#define MCU_ACT_CCMR1_OCM_CC2S_TI2            ( 0x100U )
#define MCU_ACT_CCMR1_OCM_CC2S_OUTPUT         ( 0x0U )
#define MCU_ACT_CCMR1_OCM_CC2S_MASK           ( 0x300U )
#define MCU_ACT_CCMR1_OCM_OC1CE               ( 0x80U )
#define MCU_ACT_CCMR1_OCM_OC1M_PWM_MODE_2     ( 0x70U )
#define MCU_ACT_CCMR1_OCM_OC1M_PWM_MODE_1     ( 0x60U )
#define MCU_ACT_CCMR1_OCM_OC1M_FORCE_ACTIVE   ( 0x50U )
#define MCU_ACT_CCMR1_OCM_OC1M_FORCE_INACTIVE ( 0x40U )
#define MCU_ACT_CCMR1_OCM_OC1M_TOGGLE         ( 0x30U )
#define MCU_ACT_CCMR1_OCM_OC1M_INACTIVE_LVL   ( 0x20U )
#define MCU_ACT_CCMR1_OCM_OC1M_ACTIVE_LVL     ( 0x10U )
#define MCU_ACT_CCMR1_OCM_OC1M_FROZEN         ( 0x0U )
#define MCU_ACT_CCMR1_OCM_OC1M_MASK           ( 0x70U )
#define MCU_ACT_CCMR1_OCM_OC1PE               ( 0x8U )
#define MCU_ACT_CCMR1_OCM_OC1FE               ( 0x4U )
#define MCU_ACT_CCMR1_OCM_CC1S_TRC            ( 0x3U )
#define MCU_ACT_CCMR1_OCM_CC1S_TI2            ( 0x2U )
#define MCU_ACT_CCMR1_OCM_CC1S_TI1            ( 0x1U )
#define MCU_ACT_CCMR1_OCM_CC1S_OUTPUT         ( 0x0U )
#define MCU_ACT_CCMR1_OCM_CC1S_MASK           ( 0x3U )

/* Биты регистра CCMR1_ICM (input capture mode register 1) */
#define MCU_ACT_CCMR1_ICM_BITS                ( 0xFFFFU )
#define MCU_ACT_CCMR1_ICM_IC2F_MASK           ( 0xF000U )
#define MCU_ACT_CCMR1_ICM_IC2PSC_MASK         ( 0xC00U )
#define MCU_ACT_CCMR1_ICM_CC2S_TRC            ( 0x300U )
#define MCU_ACT_CCMR1_ICM_CC2S_TI1            ( 0x200U )
#define MCU_ACT_CCMR1_ICM_CC2S_TI2            ( 0x100U )
#define MCU_ACT_CCMR1_ICM_CC2S_OUTPUT         ( 0x000U )
#define MCU_ACT_CCMR1_ICM_CC2S_MASK           ( 0x300U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_32_8       ( 0xF0U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_32_6       ( 0xE0U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_32_5       ( 0xD0U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_16_8       ( 0xC0U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_16_6       ( 0xB0U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_16_5       ( 0xA0U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_8_8        ( 0x90U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_8_6        ( 0x80U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_4_8        ( 0x70U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_4_6        ( 0x60U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_2_8        ( 0x50U )
#define MCU_ACT_CCMR1_ICM_IC1F_DTS_2_6        ( 0x40U )
#define MCU_ACT_CCMR1_ICM_IC1F_CK_INT_8       ( 0x30U )
#define MCU_ACT_CCMR1_ICM_IC1F_CK_INT_4       ( 0x20U )
#define MCU_ACT_CCMR1_ICM_IC1F_CK_INT_2       ( 0x10U )
#define MCU_ACT_CCMR1_ICM_IC1F_NO_FILTER      ( 0x00U )
#define MCU_ACT_CCMR1_ICM_IC1F_MASK           ( 0xF0U )
#define MCU_ACT_CCMR1_ICM_IC1PSC_8_EVENTS     ( 0xCU )
#define MCU_ACT_CCMR1_ICM_IC1PSC_4_EVENTS     ( 0x8U )
#define MCU_ACT_CCMR1_ICM_IC1PSC_2_EVENTS     ( 0x4U )
#define MCU_ACT_CCMR1_ICM_IC1PSC_NO_PRESCALER ( 0x0U )
#define MCU_ACT_CCMR1_ICM_IC1PSC_MASK         ( 0xCU )
#define MCU_ACT_CCMR1_ICM_CC1S_TRC            ( 0x3U )
#define MCU_ACT_CCMR1_ICM_CC1S_TI2            ( 0x2U )
#define MCU_ACT_CCMR1_ICM_CC1S_TI1            ( 0x1U )
#define MCU_ACT_CCMR1_ICM_CC1S_OUTPUT         ( 0x0U )
#define MCU_ACT_CCMR1_ICM_CC1S_MASK           ( 0x3U )

/* Биты регистра CCMR2_OCM (output compare mode register 2) */
#define MCU_ACT_CCMR2_OCM_BITS                ( 0xFFFFU )
#define MCU_ACT_CCMR2_OCM_OC4CE               ( 0x8000U )
#define MCU_ACT_CCMR2_OCM_OC4M_MASK           ( 0x7000U )
#define MCU_ACT_CCMR2_OCM_OC4PE               ( 0x800U )
#define MCU_ACT_CCMR2_OCM_OC4FE               ( 0x400U )
#define MCU_ACT_CCMR2_OCM_CC4S_TRC            ( 0x300U )
#define MCU_ACT_CCMR2_OCM_CC4S_TI3            ( 0x200U )
#define MCU_ACT_CCMR2_OCM_CC4S_TI4            ( 0x100U )
#define MCU_ACT_CCMR2_OCM_CC4S_OUTPUT         ( 0x0U )
#define MCU_ACT_CCMR2_OCM_CC4S_MASK           ( 0x300U )
#define MCU_ACT_CCMR2_OCM_OC3CE               ( 0x80U )
#define MCU_ACT_CCMR2_OCM_OC3M_MASK           ( 0x70U )
#define MCU_ACT_CCMR2_OCM_OC3PE               ( 0x8U )
#define MCU_ACT_CCMR2_OCM_OC3FE               ( 0x4U )
#define MCU_ACT_CCMR2_OCM_CC3S_TRC            ( 0x3U )
#define MCU_ACT_CCMR2_OCM_CC3S_TI4            ( 0x2U )
#define MCU_ACT_CCMR2_OCM_CC3S_TI3            ( 0x1U )
#define MCU_ACT_CCMR2_OCM_CC3S_OUTPUT         ( 0x0U )
#define MCU_ACT_CCMR2_OCM_CC3S_MASK           ( 0x3U )

/* Биты регистра CCMR2_ICM (input capture mode register 2) */
#define MCU_ACT_CCMR2_ICM_BITS                ( 0xFFFFU )
#define MCU_ACT_CCMR2_ICM_IC4F_MASK           ( 0xF000U )
#define MCU_ACT_CCMR2_ICM_IC4PSC_MASK         ( 0xC00U )
#define MCU_ACT_CCMR2_ICM_CC4S_TRC            ( 0x300U )
#define MCU_ACT_CCMR2_ICM_CC4S_TI3            ( 0x200U )
#define MCU_ACT_CCMR2_ICM_CC4S_TI4            ( 0x100U )
#define MCU_ACT_CCMR2_ICM_CC4S_OUTPUT         ( 0x0U )
#define MCU_ACT_CCMR2_ICM_CC4S_MASK           ( 0x300U )
#define MCU_ACT_CCMR2_ICM_IC3F_MASK           ( 0xF0U )
#define MCU_ACT_CCMR2_ICM_IC3PSC_MASK         ( 0xCU )
#define MCU_ACT_CCMR2_ICM_CC3S_TRC            ( 0x3U )
#define MCU_ACT_CCMR2_ICM_CC3S_TI4            ( 0x2U )
#define MCU_ACT_CCMR2_ICM_CC3S_TI3            ( 0x1U )
#define MCU_ACT_CCMR2_ICM_CC3S_OUTPUT         ( 0x0U )
#define MCU_ACT_CCMR2_ICM_CC3S_MASK           ( 0x3U )

/* Биты регистра CCER (capture/compare enable register) */
#define MCU_ACT_CCER_BITS                     ( 0xBFFFU )
#define MCU_ACT_CCER_CC4NP                    ( 0x8000U )
#define MCU_ACT_CCER_CC4P                     ( 0x2000U )
#define MCU_ACT_CCER_CC4E                     ( 0x1000U )
#define MCU_ACT_CCER_CC3NP                    ( 0x800U )
#define MCU_ACT_CCER_CC3NE                    ( 0x400U )
#define MCU_ACT_CCER_CC3P                     ( 0x200U )
#define MCU_ACT_CCER_CC3E                     ( 0x100U )
#define MCU_ACT_CCER_CC2NP                    ( 0x80U )
#define MCU_ACT_CCER_CC2NE                    ( 0x40U )
#define MCU_ACT_CCER_CC2P                     ( 0x20U )
#define MCU_ACT_CCER_CC2E                     ( 0x10U )
#define MCU_ACT_CCER_CC1NP                    ( 0x8U )
#define MCU_ACT_CCER_CC1NE                    ( 0x4U )
#define MCU_ACT_CCER_CC1P                     ( 0x2U )
#define MCU_ACT_CCER_CC1E                     ( 0x1U )

/* Биты регистра CNT (counter) */
#define MCU_ACT_CNT_BITS                      ( 0xFFFFU )
#define MCU_ACT_CNT_CNT_MASK                  ( 0xFFFFU )

/* Биты регистра PSC (prescaler) */
#define MCU_ACT_PSC_BITS                      ( 0xFFFFU )
#define MCU_ACT_PSC_PSC_MASK                  ( 0xFFFFU )

/* Биты регистра ARR (auto-reload register) */
#define MCU_ACT_ARR_BITS                      ( 0xFFFFU )
#define MCU_ACT_ARR_ARR_MASK                  ( 0xFFFFU )

/* Биты регистра RCR (repetition counter register) */
#define MCU_ACT_RCR_BITS                      ( 0xFFU )
#define MCU_ACT_RCR_REP_MASK                  ( 0xFFU )

/* Биты регистра CCRX (capture/compare register 1-4) */
#define MCU_ACT_CCRX_BITS                     ( 0xFFFFU )
#define MCU_ACT_CCRX_CCRX_MASK                ( 0xFFFFU )

/* Биты регистра BDTR (break and dead-time register) */
#define MCU_ACT_BDTR_BITS                     ( 0xFFFFU )
#define MCU_ACT_BDTR_MOE                      ( 0x8000U )
#define MCU_ACT_BDTR_AOE                      ( 0x4000U )
#define MCU_ACT_BDTR_BKP                      ( 0x2000U )
#define MCU_ACT_BDTR_BKE                      ( 0x1000U )
#define MCU_ACT_BDTR_OSSR                     ( 0x800U )
#define MCU_ACT_BDTR_OSSI                     ( 0x400U )
#define MCU_ACT_BDTR_LOCK_LVL3                ( 0x300U )
#define MCU_ACT_BDTR_LOCK_LVL2                ( 0x200U )
#define MCU_ACT_BDTR_LOCK_LVL1                ( 0x100U )
#define MCU_ACT_BDTR_LOCK_OFF                 ( 0x0U )
#define MCU_ACT_BDTR_LOCK_MASK                ( 0x300U )
#define MCU_ACT_BDTR_DTG_MASK                 ( 0xFFU )

/* Биты регистра DCR (DMA control register) */
#define MCU_ACT_DCR_BITS                      ( 0x1F1FU )
#define MCU_ACT_DCR_DBL_18                    ( 0x1100U )
#define MCU_ACT_DCR_DBL_17                    ( 0x1000U )
#define MCU_ACT_DCR_DBL_16                    ( 0xF00U )
#define MCU_ACT_DCR_DBL_15                    ( 0xE00U )
#define MCU_ACT_DCR_DBL_14                    ( 0xD00U )
#define MCU_ACT_DCR_DBL_13                    ( 0xC00U )
#define MCU_ACT_DCR_DBL_12                    ( 0xB00U )
#define MCU_ACT_DCR_DBL_11                    ( 0xA00U )
#define MCU_ACT_DCR_DBL_10                    ( 0x900U )
#define MCU_ACT_DCR_DBL_9                     ( 0x800U )
#define MCU_ACT_DCR_DBL_8                     ( 0x700U )
#define MCU_ACT_DCR_DBL_7                     ( 0x600U )
#define MCU_ACT_DCR_DBL_6                     ( 0x500U )
#define MCU_ACT_DCR_DBL_5                     ( 0x400U )
#define MCU_ACT_DCR_DBL_4                     ( 0x300U )
#define MCU_ACT_DCR_DBL_3                     ( 0x200U )
#define MCU_ACT_DCR_DBL_2                     ( 0x100U )
#define MCU_ACT_DCR_DBL_1                     ( 0x0U )
#define MCU_ACT_DCR_DBL_MASK                  ( 0x1F00U )
#define MCU_ACT_DCR_DBA_TIMX_DMAR             ( 0x15U )
#define MCU_ACT_DCR_DBA_TIMX_DCR              ( 0x14U )
#define MCU_ACT_DCR_DBA_TIMX_BDTR             ( 0x13U )
#define MCU_ACT_DCR_DBA_TIMX_CCRX4            ( 0x12U )
#define MCU_ACT_DCR_DBA_TIMX_CCRX3            ( 0x11U )
#define MCU_ACT_DCR_DBA_TIMX_CCRX2            ( 0x10U )
#define MCU_ACT_DCR_DBA_TIMX_CCRX1            ( 0xFU )
#define MCU_ACT_DCR_DBA_TIMX_RCR              ( 0xEU )
#define MCU_ACT_DCR_DBA_TIMX_ARR              ( 0xDU )
#define MCU_ACT_DCR_DBA_TIMX_PSC              ( 0xCU )
#define MCU_ACT_DCR_DBA_TIMX_CNT              ( 0xBU )
#define MCU_ACT_DCR_DBA_TIMX_CCER             ( 0xAU )
#define MCU_ACT_DCR_DBA_TIMX_CCMR2_ICM        ( 0x9U )
#define MCU_ACT_DCR_DBA_TIMX_CCMR2_OCM        ( 0x8U )
#define MCU_ACT_DCR_DBA_TIMX_CCMR1_ICM        ( 0x7U )
#define MCU_ACT_DCR_DBA_TIMX_CCMR1_OCM        ( 0x6U )
#define MCU_ACT_DCR_DBA_TIMX_EGR              ( 0x5U )
#define MCU_ACT_DCR_DBA_TIMX_SR               ( 0x4U )
#define MCU_ACT_DCR_DBA_TIMX_DIER             ( 0x3U )
#define MCU_ACT_DCR_DBA_TIMX_SMCR             ( 0x2U )
#define MCU_ACT_DCR_DBA_TIMX_CR2              ( 0x1U )
#define MCU_ACT_DCR_DBA_TIMX_CR1              ( 0x0U )
#define MCU_ACT_DCR_DBA_MASK                  ( 0x1FU )

/* Биты регистра DMAR (DMA address for full transfer) */
#define MCU_ACT_DMAR_BITS                     ( 0xFFFFFFFFU )
#define MCU_ACT_DMAR_DMAB_MASK                ( 0xFFFFFFFFU )


/* ----- GPT (general-purpose timers (TIM2 to TIM5)) ----- */
typedef volatile struct tag_stm32f407_gpt {
    uint32_t        cr1;
    uint32_t        cr2;
    uint32_t        smcr;
    uint32_t        dier;
    uint32_t        sr;
    uint32_t        egr;
    uint32_t        ccmr1;
    uint32_t        ccmr2;
    uint32_t        ccer;
    uint32_t        cnt;
    uint32_t        psc;
    uint32_t        arr;
    uint32_t        gap0;
    uint32_t        ccr1;
    uint32_t        ccr2;
    uint32_t        ccr3;
    uint32_t        ccr4;
    uint32_t        gap1;  
    uint32_t        dcr;
    uint32_t        dmar;
    uint32_t        tim2_or;
    uint32_t        tim5_or;
} stm32f407_gpt_t;

#define STM32F407_TIM2_PTR                    ( ( stm32f407_gpt_t * ) 0x40000000U )
#define STM32F407_TIM3_PTR                    ( ( stm32f407_gpt_t * ) 0x40000400U )
#define STM32F407_TIM4_PTR                    ( ( stm32f407_gpt_t * ) 0x40000800U )
#define STM32F407_TIM5_PTR                    ( ( stm32f407_gpt_t * ) 0x40000C00U )

/* Биты регистра CR1 (control register 1) */
#define MCU_GPT_CR1_BITS                      ( 0x3FFU )
#define MCU_GPT_CR1_CKD_MASK                  ( 0x300U )
#define MCU_GPT_CR1_ARPE                      ( 0x80U )
#define MCU_GPT_CR1_CMS_MASK                  ( 0x60U )
#define MCU_GPT_CR1_DIR                       ( 0x10U )
#define MCU_GPT_CR1_OPM                       ( 0x8U )
#define MCU_GPT_CR1_URS                       ( 0x4U )
#define MCU_GPT_CR1_UDIS                      ( 0x2U )
#define MCU_GPT_CR1_CEN                       (	0x1U )

/* Биты регистра CR2 (control register 2) */
#define MCU_GPT_CR2_BITS                      ( 0xF8U )
#define MCU_GPT_CR2_TI1S                      ( 0x80U )
#define MCU_GPT_CR2_MMS_MASK                  ( 0x70U )
#define MCU_GPT_CR2_CCDS                      ( 0x8U )

/* Биты регистра SMCR (slave mode control register) */
#define MCU_GPT_SMCR_BITS                     ( 0xFFF7U )
#define MCU_GPT_SMCR_ETP                      ( 0x8000U )
#define MCU_GPT_SMCR_ECE                      ( 0x4000U )
#define MCU_GPT_SMCR_ETPS_MASK                ( 0x3000U )
#define MCU_GPT_SMCR_ETF_MASK                 ( 0xF00U )
#define MCU_GPT_SMCR_MSM                      ( 0x80U )
#define MCU_GPT_SMCR_TS_MASK                  ( 0x70U )
#define MCU_GPT_SMCR_SMS_MASK                 ( 0x7U )

/* Биты регистра DIER (DMA/interrupt enable register) */
#define MCU_GPT_DIER_BITS                     ( 0x7F5FU )
#define MCU_GPT_DIER_TDE                      ( 0x4000U )
#define MCU_GPT_DIER_COMDE                    ( 0x2000U )
#define MCU_GPT_DIER_CC4DE                    ( 0x1000U )
#define MCU_GPT_DIER_CC3DE                    ( 0x800U )
#define MCU_GPT_DIER_CC2DE                    ( 0x400U )
#define MCU_GPT_DIER_CC1DE                    ( 0x200U )
#define MCU_GPT_DIER_UDE                      ( 0x100U )
#define MCU_GPT_DIER_TIE                      ( 0x40U )
#define MCU_GPT_DIER_CC4IE                    ( 0x10U )
#define MCU_GPT_DIER_CC3IE                    ( 0x8U )
#define MCU_GPT_DIER_CC2IE                    ( 0x4U )
#define MCU_GPT_DIER_CC1IE                    ( 0x2U )
#define MCU_GPT_DIER_UIE                      ( 0x1U )

/* Биты регистра SR (status register) */
#define MCU_GPT_SR_BITS                       ( 0x1E5FU )
#define MCU_GPT_SR_CC4OF                      ( 0x1000U )
#define MCU_GPT_SR_CC3OF                      ( 0x800U )
#define MCU_GPT_SR_CC2OF                      ( 0x400U )
#define MCU_GPT_SR_CC1OF                      ( 0x200U )
#define MCU_GPT_SR_TIF                        ( 0x40U )
#define MCU_GPT_SR_CC4IF                      ( 0x10U )
#define MCU_GPT_SR_CC3IF                      ( 0x8U )
#define MCU_GPT_SR_CC2IF                      ( 0x4U )
#define MCU_GPT_SR_CC1IF                      ( 0x2U )
#define MCU_GPT_SR_UIF                        ( 0x1U )

/* Биты регистра EGR (event generation register) */
#define MCU_GPT_EGR_BITS                      ( 0xFFU )
#define MCU_GPT_EGR_TG                        ( 0x40U )
#define MCU_GPT_EGR_CC4G                      ( 0x10U )
#define MCU_GPT_EGR_CC3G                      ( 0x8U )
#define MCU_GPT_EGR_CC2G                      ( 0x4U )
#define MCU_GPT_EGR_CC1G                      ( 0x2U )
#define MCU_GPT_EGR_UG                        ( 0x1U )

/* Биты регистра CCMR1_OCM (output compare mode register 1) */
#define MCU_GPT_CCMR1_OCM_BITS                ( 0xFFFFU )
#define MCU_GPT_CCMR1_OCM_OC2CE               ( 0x8000U )
#define MCU_GPT_CCMR1_OCM_OC2M_MASK           ( 0x7000U )
#define MCU_GPT_CCMR1_OCM_OC2PE               ( 0x800U )
#define MCU_GPT_CCMR1_OCM_OC2FE               ( 0x400U )
#define MCU_GPT_CCMR1_OCM_CC2S_MASK           ( 0x300U )
#define MCU_GPT_CCMR1_OCM_OC1CE               ( 0x80U )
#define MCU_GPT_CCMR1_OCM_OC1M_MASK           ( 0x70U )
#define MCU_GPT_CCMR1_OCM_OC1M_MOD1           ( 0x60U )
#define MCU_GPT_CCMR1_OCM_OC1PE               ( 0x8U )
#define MCU_GPT_CCMR1_OCM_OC1FE               ( 0x4U )
#define MCU_GPT_CCMR1_OCM_CC1S_MASK           ( 0x3U )

/* Биты регистра CCMR1_ICM (input capture mode register 1) */
#define MCU_GPT_CCMR1_ICM_BITS                ( 0xFFFFU )
#define MCU_GPT_CCMR1_ICM_IC2F_MASK           ( 0xF000U )
#define MCU_GPT_CCMR1_ICM_IC2PSC_MASK         ( 0xC00U )
#define MCU_GPT_CCMR1_ICM_CC2S_MASK           ( 0x300U )
#define MCU_GPT_CCMR1_ICM_IC1F_MASK           ( 0xF0U )
#define MCU_GPT_CCMR1_ICM_IC1PSC_MASK         ( 0xCU )
#define MCU_GPT_CCMR1_ICM_CC1S_MASK           ( 0x3U )

/* Биты регистра CCMR2_OCM (output compare mode register 2) */
#define MCU_GPT_CCMR2_OCM_BITS                ( 0xFFFFU )
#define MCU_GPT_CCMR2_OCM_OC4CE               ( 0x8000U )
#define MCU_GPT_CCMR2_OCM_OC4M_MASK           ( 0x7000U )
#define MCU_GPT_CCMR2_OCM_OC4PE               ( 0x800U )
#define MCU_GPT_CCMR2_OCM_OC4FE               ( 0x400U )
#define MCU_GPT_CCMR2_OCM_CC4S_MASK           ( 0x300U )
#define MCU_GPT_CCMR2_OCM_OC3CE               ( 0x80U )
#define MCU_GPT_CCMR2_OCM_OC3M_MASK           ( 0x70U )
#define MCU_GPT_CCMR2_OCM_OC3M_MOD1           ( 0x60U )
#define MCU_GPT_CCMR2_OCM_OC3PE               ( 0x8U )
#define MCU_GPT_CCMR2_OCM_OC3FE               ( 0xCU )
#define MCU_GPT_CCMR2_OCM_CC3S_MASK           ( 0x3U )

/* Биты регистра CCMR2_ICM (input capture mode register 2) */
#define MCU_GPT_CCMR2_ICM_BITS                ( 0xFFFFU )
#define MCU_GPT_CCMR2_ICM_IC4F_MASK           ( 0xF000U )
#define MCU_GPT_CCMR2_ICM_IC4PSC_MASK         ( 0xC00U )
#define MCU_GPT_CCMR2_ICM_CC4S_MASK           ( 0x300U )
#define MCU_GPT_CCMR2_ICM_IC3F_MASK           ( 0xF0U )
#define MCU_GPT_CCMR2_ICM_IC3PSC_MASK         ( 0xCU )
#define MCU_GPT_CCMR2_ICM_CC3S_MASK           ( 0x3U )

/* Биты регистра CCER (capture/compare enable register) */
#define MCU_GPT_CCER_BITS                     ( 0xBBBBU )
#define MCU_GPT_CCER_CC4NP                    ( 0x8000U )
#define MCU_GPT_CCER_CC4P                     ( 0x2000U )
#define MCU_GPT_CCER_CC4E                     ( 0x1000U )
#define MCU_GPT_CCER_CC3NP                    ( 0x800U )
#define MCU_GPT_CCER_CC3P                     ( 0x200U )
#define MCU_GPT_CCER_CC3E                     ( 0x100U )
#define MCU_GPT_CCER_CC2NP                    ( 0x80U )
#define MCU_GPT_CCER_CC2P                     ( 0x20U )
#define MCU_GPT_CCER_CC2E                     ( 0x10U )
#define MCU_GPT_CCER_CC1NP                    ( 0x8U )
#define MCU_GPT_CCER_CC1P                     ( 0x2U )
#define MCU_GPT_CCER_CC1E                     ( 0x1U )

/* Биты регистра CNT (counter) */
#define MCU_GPT_CNT_BITS                      ( 0xFFFFU )
#define MCU_GPT_CNT_CNT_MASK                  ( 0xFFFFU )

/* Биты регистра PSC (prescaler) */
#define MCU_GPT_PSC_BITS                      ( 0xFFFFU )
#define MCU_GPT_PSC_PSC_MASK                  ( 0xFFFFU )

/* Биты регистра ARR (auto-reload register) */
#define MCU_GPT_ARR_BITS                      ( 0xFFFFFFFFU )
#define MCU_GPT_ARR_ARR_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра CCR1 (capture/compare register 1) */
#define MCU_GPT_CCR1_BITS                     ( 0xFFFFFFFFU )
#define MCU_GPT_CCR1_CCR1_MASK                ( 0xFFFFFFFFU )

/* Биты регистра CCR2 (capture/compare register 2) */
#define MCU_GPT_CCR2_BITS                     ( 0xFFFFFFFFU )
#define MCU_GPT_CCR2_CCR2_MASK                ( 0xFFFFFFFFU )

/* Биты регистра CCR3 (capture/compare register 3) */
#define MCU_GPT_CCR3_BITS                     ( 0xFFFFFFFFU )
#define MCU_GPT_CCR3_CCR3_MASK                ( 0xFFFFFFFFU )

/* Биты регистра CCR4 (capture/compare register 4) */
#define MCU_GPT_CCR4_BITS                     ( 0xFFFFFFFFU )
#define MCU_GPT_CCR4_CCR4_MASK                ( 0xFFFFFFFFU )

/* Биты регистра DCR (DMA control register) */
#define MCU_GPT_DCR_BITS                      ( 0x1F1FU )
#define MCU_GPT_DCR_DBL_MASK                  ( 0x1F00U )
#define MCU_GPT_DCR_DBA_MASK                  ( 0x1FU )

/* Биты регистра DMAR (DMA address for full transfer) */
#define MCU_GPT_DMAR_BITS                     ( 0xFFFFU )
#define MCU_GPT_DMAR_DMAB_MASK                ( 0xFFFFU )

/* Биты регистра TIM2_OR (TIM2 option register) */
#define MCU_GPT_TIM_2_OR_BITS                 ( 0xC000U )
#define MCU_GPT_TIM_2_OR_ITR1_RMP             ( 0xC000U )

/* Биты регистра TIM5_OR (TIM5 option register) */
#define MCU_GPT_TIM_5_OR_BITS                 ( 0xC0U )
#define MCU_GPT_TIM_5_OR_TI4_RMP              ( 0xC0U )


/* ----- TIM9_12 (general-purpose timers TIM9 and TIM12) ----- */
typedef volatile struct tag_stm32f407_tim9_12 {
    uint32_t        cr1;
    uint32_t        smcr;
    uint32_t        dier;
    uint32_t        sr;
    uint32_t        egr;
    uint32_t        ccmr1_ocm;
    uint32_t        ccmr1_icm;
    uint32_t        gap0;
    uint32_t        ccer;
    uint32_t        cnt;
    uint32_t        psc;
    uint32_t        arr;
    uint32_t        gap1;
    uint32_t        ccr1;
    uint32_t        ccr2;
    uint32_t        gap3[3];
} stm32f407_tim9_12_t;

#define STM32F407_TIM9_12_PTR                 ( ( stm32f407_tim9_12_t * ) 0x40014000U )

/* Биты регистра CR1 (control register 1) */
#define MCU_TIM9_12_CR1_BITS                  ( 0x3FFU )
#define MCU_TIM9_12_CR1_CKD_MASK              ( 0x300U )
#define MCU_TIM9_12_CR1_ARPE                  ( 0x80U )
#define MCU_TIM9_12_CR1_OPM                   ( 0x8U )
#define MCU_TIM9_12_CR1_URS                   ( 0x4U )
#define MCU_TIM9_12_CR1_UDIS                  ( 0x2U )
#define MCU_TIM9_12_CR1_CEN                   ( 0x1U )

/* Биты регистра SMCR (slave mode control register) */
#define MCU_TIM9_12_SMCR_BITS                 ( 0xF7U )
#define MCU_TIM9_12_SMCR_MSM                  ( 0x80U )
#define MCU_TIM9_12_SMCR_TS_MASK              ( 0x70U )
#define MCU_TIM9_12_SMCR_SMS_MASK             ( 0x7U )

/* Биты регистра DIER (DMA/interrupt enable register) */
#define MCU_TIM9_12_DIER_BITS                 ( 0x47U )
#define MCU_TIM9_12_DIER_TIE                  (	0x40U )
#define MCU_TIM9_12_DIER_CC2IE                ( 0x4U )
#define MCU_TIM9_12_DIER_CC1IE                ( 0x2U )
#define MCU_TIM9_12_DIER_UIE                  ( 0x1U )

/* Биты регистра SR (status register) */
#define MCU_TIM9_12_SR_BITS                   ( 0x647U )
#define MCU_TIM9_12_SR_CC2OF                  ( 0x400U )
#define MCU_TIM9_12_SR_CC1OF                  ( 0x200U )
#define MCU_TIM9_12_SR_TIF                    ( 0x40U )
#define MCU_TIM9_12_SR_CC2IF                  ( 0x4U )
#define MCU_TIM9_12_SR_CC1IF                  ( 0x2U )
#define MCU_TIM9_12_SR_UIF                    ( 0x1U )

/* Биты регистра EGR (event generation register) */
#define MCU_TIM9_12_EGR_BITS                  ( 0x47U )
#define MCU_TIM9_12_EGR_TG                    ( 0x40U )
#define MCU_TIM9_12_EGR_CC2G                  ( 0x4U )
#define MCU_TIM9_12_EGR_CC1G                  ( 0x2U )
#define MCU_TIM9_12_EGR_UG                    ( 0x1U )

/* Биты регистра CCMR1_OCM (output compare mode register) */
#define MCU_TIM9_12_CCMR1_OCM_BITS            ( 0x7F7FU )
#define MCU_TIM9_12_CCMR1_OCM_OC2M_MASK       ( 0x7000U )
#define MCU_TIM9_12_CCMR1_OCM_OC2PE           ( 0x800U )
#define MCU_TIM9_12_CCMR1_OCM_OC2FE           ( 0x400U )
#define MCU_TIM9_12_CCMR1_OCM_CC2S_MASK       ( 0x300U )
#define MCU_TIM9_12_CCMR1_OCM_OC1M_MASK       ( 0x70U )
#define MCU_TIM9_12_CCMR1_OCM_OC1PE           ( 0x8U )
#define MCU_TIM9_12_CCMR1_OCM_OC1FE           ( 0x4U )
#define MCU_TIM9_12_CCMR1_OCM_CC1S_MASK       ( 0x3U )

/* Биты регистра CCMR1_ICM (input capture mode register) */
#define MCU_TIM9_12_CCMR1_ICM_BITS            ( 0xFFFFU )
#define MCU_TIM9_12_CCMR1_ICM_IC2F_MASK       ( 0xF000U )
#define MCU_TIM9_12_CCMR1_ICM_IC2PSC_MASK     ( 0xC00U )
#define MCU_TIM9_12_CCMR1_ICM_CC2S_MASK       ( 0x300U )
#define MCU_TIM9_12_CCMR1_ICM_IC1F_MASK       ( 0xF0U )
#define MCU_TIM9_12_CCMR1_ICM_IC1PSC_MASK     ( 0xCU )
#define MCU_TIM9_12_CCMR1_ICM_CC1S_MASK       ( 0x3U )

/* Биты регистра CCER (capture/compare enable register) */
#define MCU_TIM9_12_CCER_BITS                 ( 0xBBU )
#define MCU_TIM9_12_CCER_CC2NP                ( 0x80U )
#define MCU_TIM9_12_CCER_CC2P                 ( 0x20U )
#define MCU_TIM9_12_CCER_CC2E                 ( 0x10U )
#define MCU_TIM9_12_CCER_CC1NP                ( 0x8U )
#define MCU_TIM9_12_CCER_CC1P                 ( 0x2U )
#define MCU_TIM9_12_CCER_CC1E                 ( 0x1U )

/* Биты регистра CNT (counter) */
#define MCU_TIM9_12_CNT_BITS                  ( 0xFFFFU )
#define MCU_TIM9_12_CNT_CNT_MASK              ( 0xFFFFU )

/* Биты регистра PSC (prescaler) */
#define MCU_TIM9_12_PSC_BITS                  ( 0xFFFFU )
#define MCU_TIM9_12_PSC_PSC_MASK              ( 0xFFFFU )

/* Биты регистра ARR (auto-reload register) */
#define MCU_TIM9_12_ARR_BITS                  ( 0xFFFFU )
#define MCU_TIM9_12_ARR_ARR_MASK              ( 0xFFFFU )

/* Биты регистра CCR1 (capture/compare register 1) */
#define MCU_TIM9_12_CCR1_BITS                 ( 0xFFFFU )
#define MCU_TIM9_12_CCR1_CCR1_MASK            ( 0xFFFFU )

/* Биты регистра CCR2 (capture/compare register 2) */
#define MCU_TIM9_12_CCR2_BITS                 ( 0xFFFFU )
#define MCU_TIM9_12_CCR2_CCR2_MASK            ( 0xFFFFU )


/* ----- TIM10_14 (general-purpose timers TIM10,11,13,14) ----- */
typedef volatile struct tag_stm32f407_tim10_14 {
    uint32_t        cr1;
    uint32_t        smcr;
    uint32_t        dier;
    uint32_t        sr;
    uint32_t        egr;
    uint32_t        ccmr1_ocm;
    uint32_t        ccmr1_icm;
    uint32_t        gap0;
    uint32_t        ccer;
    uint32_t        cnt;
    uint32_t        psc;
    uint32_t        arr;
    uint32_t        gap1;
    uint32_t        ccr1;
    uint32_t        gap2[4];
    uint32_t        tim11_or;
} stm32f407_tim10_14_t;

#define STM32F407_TIM10_14_PTR                ( ( stm32f407_tim10_14_t * ) 0x40014400U )

//* Биты регистра CR1 (control register 1) */
#define MCU_TIM10_14_CR1_BITS                 ( 0x38FU )
#define MCU_TIM10_14_CR1_CKD_MASK             ( 0x300U )
#define MCU_TIM10_14_CR1_ARPE                 ( 0x80U )
#define MCU_TIM10_14_CR1_OPM                  ( 0x8U )
#define MCU_TIM10_14_CR1_URS                  ( 0x4U )
#define MCU_TIM10_14_CR1_UDIS                 ( 0x2U )
#define MCU_TIM10_14_CR1_CEN                  ( 0x1U )

/* Биты регистра DIER (DMA/interrupt enable register) */
#define MCU_TIM10_14_DIER_BITS                ( 0x3U )
#define MCU_TIM10_14_DIER_CC1IE               ( 0x2U )
#define MCU_TIM10_14_DIER_UIE                 ( 0x1U )

/* Биты регистра SR (status register) */
#define MCU_TIM10_14_SR_BITS                  ( 0x203U )
#define MCU_TIM10_14_SR_CC1OF                 ( 0x200U )
#define MCU_TIM10_14_SR_CC1IF                 ( 0x2U )
#define MCU_TIM10_14_SR_UIF                   ( 0x1U )

/* Биты регистра EGR (event generation register) */
#define MCU_TIM10_14_EGR_BITS                 ( 0x3U )
#define MCU_TIM10_14_EGR_CC1G                 ( 0x2U )
#define MCU_TIM10_14_EGR_UG                   ( 0x1U )

/* Биты регистра CCMR1_OCM (output compare mode register  1) */
#define MCU_TIM10_14_CCMR1_OCM_BITS           ( 0x7FU )
#define MCU_TIM10_14_CCMR1_OCM_OC1M_MASK      ( 0x70U )
#define MCU_TIM10_14_CCMR1_OCM_OC1PE          ( 0x8U )
#define MCU_TIM10_14_CCMR1_OCM_OC1FE          ( 0x4U )
#define MCU_TIM10_14_CCMR1_OCM_CC1S_MASK      ( 0x3U )

/* Биты регистра CCMR1_ICM (input capture mode register 1) */
#define MCU_TIM10_14_CCMR1_ICM_BITS           ( 0xFFU )
#define MCU_TIM10_14_CCMR1_ICM_IC1F_MASK      ( 0xF0U )
#define MCU_TIM10_14_CCMR1_ICM_IC1PSC_MASK    ( 0xCU )
#define MCU_TIM10_14_CCMR1_ICM_CC1S_MASK      ( 0x3U )

/* Биты регистра CCER (capture/compare enable register) */
#define MCU_TIM10_14_CCER_BITS                ( 0xBU )
#define MCU_TIM10_14_CCER_CC1NP               ( 0x8U )
#define MCU_TIM10_14_CCER_CC1P                ( 0x2U )
#define MCU_TIM10_14_CCER_CC1E                ( 0x1U )

/* Биты регистра CNT (counter) */
#define MCU_TIM10_14_CNT_BITS                 ( 0xFFFFU )
#define MCU_TIM10_14_CNT_CNT_MASK             ( 0xFFFFU )

/* Биты регистра PSC (prescaler) */
#define MCU_TIM10_14_PSC_BITS                 ( 0xFFFFU )
#define MCU_TIM10_14_PSC_PSC_MASK             ( 0xFFFFU )

/* Биты регистра ARR (auto-reload register) */
#define MCU_TIM10_14_ARR_BITS                 ( 0xFFFFFFFFU )
#define MCU_TIM10_14_ARR_ARR_MASK             ( 0xFFFFFFFFU )

/* Биты регистра CCR1 (capture/compare register 1) */
#define MCU_TIM10_14_CCR1_BITS                ( 0xFFFFU )
#define MCU_TIM10_14_CCR1_CCR1_MASK           ( 0xFFFFU )

/* Биты регистра TIM11_OR (option register 1) */
#define MCU_TIM10_14_TIM11_OR_BITS            ( 0x3U )
#define MCU_TIM10_14_TIM11_OR_TI1_RMP         ( 0x3U )


/* ----- BT (basic timers TIM6 and TIM7) ----- */
typedef volatile struct tag_stm32f407_bt {
    uint32_t        cr1;
    uint32_t        cr2;
    uint32_t        gap0;
    uint32_t        dier;
    uint32_t        sr;
    uint32_t        egr;
    uint32_t        gap1[3];
    uint32_t        cnt;
    uint32_t        psc;
    uint32_t        arr;
} stm32f407_bt_t;

#define STM32F407_BT_PTR                      ( ( stm32f407_bt_t * ) 0x40001000U )

/* Биты регистра BT_CR1 (TIM6 and TIM7 control register 1) */
#define MCU_BT_CR1_BITS                       ( 0x8FU )
#define MCU_BT_CR1_ARPE                       ( 0x80U )
#define MCU_BT_CR1_OPM                        ( 0x8U )
#define MCU_BT_CR1_URS                        ( 0x4U )
#define MCU_BT_CR1_UDIS                       ( 0x2U )
#define MCU_BT_CR1_CEN                        ( 0x1U )

/* Биты регистра BT_CR2 (TIM6 and TIM7 control register 2) */
#define MCU_BT_CR2_BITS                       ( 0x70U )
#define MCU_BT_CR2_MMS_MASK                   ( 0x70U )

/* Биты регистра BT_DIER (TIM6 and TIM7 DMA/interrupt enable register) */
#define MCU_BT_DIER_BITS                      ( 0x101U )
#define MCU_BT_DIER_UDE                       ( 0x100U )
#define MCU_BT_DIER_UIE                       ( 0x1U )

/* Биты регистра BT_SR (TIM6 and TIM7 status register) */
#define MCU_BT_SR_BITS                        ( 0x1U )
#define MCU_BT_SR_UIF                         ( 0x1U )

/* Биты регистра BT_EGR (TIM6 and TIM7 event generation register) */
#define MCU_BT_EGR_BITS                       ( 0x1U )
#define MCU_BT_EGR_UG                         ( 0x1U )

/* Биты регистра BT_CNT (TIM6 and TIM7 counter) */
#define MCU_BT_CNT_BITS                       ( 0xFFFFU )
#define MCU_BT_CNT_CNT_MASK                   ( 0xFFFFU )

/* Биты регистра BT_PSC (TIM6 and TIM7 prescaler) */
#define MCU_BT_PSC_BITS                       ( 0xFFFFU )
#define MCU_BT_PSC_PSC_MASK                   ( 0xFFFFU )

/* Биты регистра BT_ARR (TIM6 and TIM7 auto-reload register) */
#define MCU_BT_ARR_BITS                       ( 0xFFFFU )
#define MCU_BT_ARR_ARR_MASK                   ( 0xFFFFU )


/* ----- IWDG (independent watchdog) ----- */
typedef volatile struct tag_stm32f407_iwdg {
    uint32_t        kr;
    uint32_t        pr;
    uint32_t        rlr;
    uint32_t        sr;
} stm32f407_iwdg_t;

#define STM32F407_IWDG_PTR                    ( ( stm32f407_iwdg_t * ) 0x40003000U )

/* Биты регистра KR */
#define MCU_IWDG_KR_BITS                      ( 0xFFFFU )
#define MCU_IWDG_KR_KEY_RESET                 ( 0xAAAAU )
#define MCU_IWDG_KR_KEY_WRITE                 ( 0x5555U )
#define MCU_IWDG_KR_KEY_ENABLE                ( 0xCCCCU )

/* Биты регистра PR */
#define MCU_IWDG_PR_BITS                      ( 0x7U )
#define MCU_IWDG_PR_MASK                      ( 0x7U )
#define MCU_IWDG_PR_DIV4                      ( 0x0U )
#define MCU_IWDG_PR_DIV8                      ( 0x1U )
#define MCU_IWDG_PR_DIV16                     ( 0x2U )
#define MCU_IWDG_PR_DIV32                     ( 0x3U )
#define MCU_IWDG_PR_DIV64                     ( 0x4U )
#define MCU_IWDG_PR_DIV128                    ( 0x5U )
#define MCU_IWDG_PR_DIV256                    ( 0x6U )

/* Биты регистра RLR */
#define MCU_IWDG_RLR_BITS                     ( 0xFFFU )
#define MCU_IWDG_RLR_RL_MASK                  ( 0xFFFU )

/* Биты регистра SR */
#define MCU_IWDG_SR_BITS                      ( 0x3U )
#define MCU_IWDG_SR_RVU                       ( 0x2U )
#define MCU_IWDG_SR_PVU                       ( 0x1U )


/* ----- RNG (random number generator) ----- */
typedef volatile struct tag_stm32f407_rng {
    uint32_t        cr;
    uint32_t        sr;
    uint32_t        dr;
} stm32f407_rng_t;

#define STM32F407_RNG_PTR                     ( ( stm32f407_rng_t * ) 0x50060800U )

/* Биты регистра CR (control register) */
#define MCU_RNG_CR_BITS                       ( 0x6U )
#define MCU_RNG_CR_IE                         ( 0x4U )
#define MCU_RNG_CR_RNGEN                      ( 0x2U )

/* Биты регистра SR (status register) */
#define MCU_RNG_SR_BITS                       ( 0x67U )
#define MCU_RNG_SR_SEIS                       ( 0x40U )
#define MCU_RNG_SR_CEIS                       ( 0x20U )
#define MCU_RNG_SR_SECS                       ( 0x4U )
#define MCU_RNG_SR_CECS                       ( 0x2U )
#define MCU_RNG_SR_DRDY                       ( 0x1U )

/* Биты регистра DR (data register) */
#define MCU_RNG_DR_BITS                       ( 0xFFFFFFFFU )
#define MCU_RNG_DR_RNDATA                     ( 0xFFFFFFFFU )


/* ----- SPI (serial peripheral interface) ----- */
typedef volatile struct tag_stm32f407_spi {
    uint32_t        cr1;
    uint32_t        cr2;
    uint32_t        sr;
    uint32_t        dr;
    uint32_t        crcpr;
    uint32_t        rxcrcr;
    uint32_t        txcrcr;
    uint32_t        i2scfgr;
    uint32_t        i2spr;
} stm32f407_spi_t;

#define STM32F407_SPI1_PTR                     ( ( stm32f407_spi_t * ) 0x40013000U )
#define STM32F407_SPI2_PTR                     ( ( stm32f407_spi_t * ) 0x40003800U )
#define STM32F407_SPI3_PTR                     ( ( stm32f407_spi_t * ) 0x40003C00U )

/* Биты регистра CR1 (control register 1) */
#define MCU_SPI_CR1_BITS                      ( 0xFFFFU )
#define MCU_SPI_CR1_BIDIMODE                  ( 0x8000U )
#define MCU_SPI_CR1_BIDIOE                    ( 0x4000U )
#define MCU_SPI_CR1_CRCEN                     ( 0x2000U )
#define MCU_SPI_CR1_CRCNEXT                   ( 0x1000U )
#define MCU_SPI_CR1_DFF                       ( 0x800U )
#define MCU_SPI_CR1_RXONLY                    ( 0x400U )
#define MCU_SPI_CR1_SSM                       ( 0x200U )
#define MCU_SPI_CR1_SSI                       ( 0x100U )
#define MCU_SPI_CR1_LSBFIRST                  ( 0x80U )
#define MCU_SPI_CR1_SPE                       ( 0x40U )
#define MCU_SPI_CR1_BR_MASK                   ( 0x38U )
#define MCU_SPI_CR1_MSTR                      ( 0x4U )
#define MCU_SPI_CR1_CPOL                      ( 0x2U )
#define MCU_SPI_CR1_CPHA                      ( 0x1U )

/* Биты регистра CR2 (control register 2) */
#define MCU_SPI_CR2_BITS                      ( 0xF7U )
#define MCU_SPI_CR2_TXEIE                     ( 0x80U )
#define MCU_SPI_CR2_RXNEIE                    ( 0x40U )
#define MCU_SPI_CR2_ERRIE                     ( 0x20U )
#define MCU_SPI_CR2_FRF                       ( 0x10U )
#define MCU_SPI_CR2_SSOE                      ( 0x4U )
#define MCU_SPI_CR2_TXDMAEN                   ( 0x2U )
#define MCU_SPI_CR2_RXDMAEN                   ( 0x1U )

/* Биты регистра SR (status register) */
#define MCU_SPI_SR_BITS                       ( 0x1FFU )
#define MCU_SPI_SR_FRE                        ( 0x100U )
#define MCU_SPI_SR_BSY                        ( 0x80U )
#define MCU_SPI_SR_OVR                        ( 0x40U )
#define MCU_SPI_SR_MODF                       ( 0x20U )
#define MCU_SPI_SR_CRCERR                     ( 0x10U )
#define MCU_SPI_SR_UDR                        ( 0x8U )
#define MCU_SPI_SR_CHSIDE                     ( 0x4U )
#define MCU_SPI_SR_TXE                        ( 0x2U )
#define MCU_SPI_SR_RXNE                       ( 0x1U )

/* Биты регистра DR (data register) */
#define MCU_SPI_DR_BITS                       ( 0xFFFFU )
#define MCU_SPI_DR_DR_MASK                    ( 0xFFFFU )

/* Биты регистра CRCPR (CRC polynomial register) */
#define MCU_SPI_CRCPR_BITS                    ( 0xFFFFU )
#define MCU_SPI_CRCPR_CRCPOLY_MASK            ( 0xFFFFU )

/* Биты регистра RXCRCR (RX CRC register) */
#define MCU_SPI_RXCRCR_BITS                   ( 0xFFFFU )
#define MCU_SPI_RXCRCR_RXCRC_MASK             ( 0xFFFFU )

/* Биты регистра TXCRCR (TX CRC register) */
#define MCU_SPI_TXCRCR_BITS                   ( 0xFFFFU )
#define MCU_SPI_TXCRCR_TXCRC_MASK             ( 0xFFFFU )

/* Биты регистра I2SCFGR (I2S configuration register) */
#define MCU_SPI_I2SCFGR_BITS                  ( 0xFBFU )
#define MCU_SPI_I2SCFGR_I2SMOD                ( 0x800U )
#define MCU_SPI_I2SCFGR_I2SE                  ( 0x400U )
#define MCU_SPI_I2SCFGR_I2SSCFG               ( 0x300U )
#define MCU_SPI_I2SCFGR_PCMSYNC               ( 0x80U )
#define MCU_SPI_I2SCFGR_I2SSTD                ( 0x30U )
#define MCU_SPI_I2SCFGR_CKPOL                 ( 0x8U )
#define MCU_SPI_I2SCFGR_DATLEN                ( 0x6U )
#define MCU_SPI_I2SCFGR_CHLEN                 ( 0x1U )

/* Биты регистра I2SPR (I2S prescaler register) */
#define MCU_SPI_I2SPR_BITS                    ( 0x3FFU )
#define MCU_SPI_I2SPR_MCKOE                   ( 0x200U )
#define MCU_SPI_I2SPR_ODD                     ( 0x100U )
#define MCU_SPI_I2SPR_I2SDIV                  ( 0xFFU )


/* ----- USART (universal synchronous asynchronous receiver transmitter) ----- */
typedef volatile struct tag_stm32f407_usart {
    uint32_t        sr;
    uint32_t        dr;
    uint32_t        brr;
    uint32_t        cr1;
    uint32_t        cr2;
    uint32_t        cr3;
    uint32_t        gtpr;
} stm32f407_usart_t;

#define STM32F407_USART1_PTR                  ( ( stm32f407_usart_t * ) 0x40011000U )
#define STM32F407_USART2_PTR                  ( ( stm32f407_usart_t * ) 0x40004400U )
#define STM32F407_USART3_PTR                  ( ( stm32f407_usart_t * ) 0x40004800U )
#define STM32F407_UART4_PTR                   ( ( stm32f407_usart_t * ) 0x40004C00U )
#define STM32F407_UART5_PTR                   ( ( stm32f407_usart_t * ) 0x40005000U )
#define STM32F407_USART6_PTR                  ( ( stm32f407_usart_t * ) 0x40011400U )
#define STM32F407_USART7_PTR                  ( ( stm32f407_usart_t * ) 0x40007800U )  //Не совсем понятно в reference manual указано что это uart
#define STM32F407_USART8_PTR                  ( ( stm32f407_usart_t * ) 0x40007C00U )  //Не совсем понятно в reference manual указано что это uart

/* Биты регистра SR (status register) */
#define MCU_USART_SR_BITS                     ( 0x3FFU )
#define MCU_USART_SR_CTS                      ( 0x200U )
#define MCU_USART_SR_LBD                      ( 0x100U )
#define MCU_USART_SR_TXE                      ( 0x80U )
#define MCU_USART_SR_TC                       ( 0x40U )
#define MCU_USART_SR_RXNE                     ( 0x20U )
#define MCU_USART_SR_IDLE                     ( 0x10U )
#define MCU_USART_SR_ORE                      ( 0x8U )
#define MCU_USART_SR_NF                       ( 0x4U )
#define MCU_USART_SR_FE                       ( 0x2U )
#define MCU_USART_SR_PE                       ( 0x1U )

/* Биты регистра DR (data register) */
#define MCU_USART_DR_BITS                     ( 0x1FFU )
#define MCU_USART_DR_DR_MASK                  ( 0x1FFU )

/* Биты регистра BRR (baud rate register) */
#define MCU_USART_BRR_BITS                    ( 0xFFFFU )
#define MCU_USART_BRR_DIV_Mantissa_MASK       ( 0xFFF0U )
#define MCU_USART_BRR_DIV_Fraction_MASK       ( 0xFU )

/* Биты регистра CR1 (control register 1) */
#define MCU_USART_CR1_BITS                    ( 0xBFFFU )
#define MCU_USART_CR1_OVER8                   ( 0x8000U )
#define MCU_USART_CR1_UE                      ( 0x2000U )
#define MCU_USART_CR1_M                       ( 0x1000U )
#define MCU_USART_CR1_WAKE                    ( 0x800U )
#define MCU_USART_CR1_PCE                     ( 0x400U )
#define MCU_USART_CR1_PS                      ( 0x200U )
#define MCU_USART_CR1_PEIE                    ( 0x100U )
#define MCU_USART_CR1_TXEIE                   ( 0x80U )
#define MCU_USART_CR1_TCIE                    ( 0x40U )
#define MCU_USART_CR1_RXNEIE                  ( 0x20U )
#define MCU_USART_CR1_IDLEIE                  ( 0x10U )
#define MCU_USART_CR1_TE                      ( 0x8U )
#define MCU_USART_CR1_RE                      ( 0x4U )
#define MCU_USART_CR1_RWU                     ( 0x2U )
#define MCU_USART_CR1_SBK                     ( 0x1U )

/* Биты регистра CR2 (control register 2) */
#define MCU_USART_CR2_BITS                    ( 0x7F6FU )
#define MCU_USART_CR2_LINEN                   ( 0x4000U )
#define MCU_USART_CR2_STOP_MASK               ( 0x3000U )
#define MCU_USART_CR2_STOP_2                  ( 0x2000U )
#define MCU_USART_CR2_STOP_1                  ( 0x1000U )
#define MCU_USART_CR2_CLKEN                   ( 0x800U )
#define MCU_USART_CR2_CPOL                    (	0x400U )
#define MCU_USART_CR2_CPHA                    ( 0x200U )
#define MCU_USART_CR2_LBCL                    ( 0x100U )
#define MCU_USART_CR2_LBDIE                   ( 0x40U )
#define MCU_USART_CR2_LBDL                    ( 0x20U )
#define MCU_USART_CR2_ADD_MASK                ( 0xFU )

/* Биты регистра CR3 (control register 3) */
#define MCU_USART_CR3_BITS                    ( 0xFFFU )
#define MCU_USART_CR3_ONEBIT                  ( 0x800U )
#define MCU_USART_CR3_CTSIE                   ( 0x400U )
#define MCU_USART_CR3_CTSE                    ( 0x200U )
#define MCU_USART_CR3_RTSE                    ( 0x100U )
#define MCU_USART_CR3_DMAT                    ( 0x80U )
#define MCU_USART_CR3_DMAR                    ( 0x40U )
#define MCU_USART_CR3_SCEN                    ( 0x20U )
#define MCU_USART_CR3_NACK                    ( 0x10U )
#define MCU_USART_CR3_HDSEL                   ( 0x8U )
#define MCU_USART_CR3_IRLP                    ( 0x4U )
#define MCU_USART_CR3_IREN                    ( 0x2U )
#define MCU_USART_CR3_EIE                     ( 0x1U )

/* Биты регистра GTPR (guard time and prescaler register) */
#define MCU_USART_GTPR_BITS                   ( 0xFFFFU )
#define MCU_USART_GTPR_GT_MASK                ( 0xFF00U )
#define MCU_USART_GTPR_PSC_MASK               ( 0xFFU )


/* ----- SDIO (secure digital input/output interface) ----- */
typedef volatile struct tag_stm32f407_sdio {
    uint32_t        power;
    uint32_t        clkcr;
    uint32_t        arg;
    uint32_t        cmd;
    uint32_t        respcmd;
    uint32_t        resp1;
    uint32_t        resp2;
    uint32_t        resp3;
    uint32_t        resp4;
    uint32_t        dtimer;
    uint32_t        dlen;
    uint32_t        dctrl;
    uint32_t        dcount;
    uint32_t        sta;
    uint32_t        icr;
    uint32_t        mask;
    uint32_t        fifocnt;
    uint32_t        fifo;
} stm32f407_sdio_t;

#define STM32F407_SDIO_PTR                    ( ( stm32f407_sdio_t * ) 0x40012C00U )

/* Биты регистра SDIO_POWER (SDIO power control register) */
#define MCU_SDIO_POWER_BITS                   ( 0x3U )
#define MCU_SDIO_POWER_PWRCTRL                ( 0x3U )

/* Биты регистра SDIO_CLKCR (SDIO clock control register) */
#define MCU_SDIO_CLKCR_BITS                   ( 0x7FFFU )
#define MCU_SDIO_CLKCR_HWFC_EN                ( 0x4000U )
#define MCU_SDIO_CLKCR_NEGEDGE                ( 0x2000U )
#define MCU_SDIO_CLKCR_WIDBUS                 ( 0x1800U )
#define MCU_SDIO_CLKCR_BYPASS                 ( 0x400U )
#define MCU_SDIO_CLKCR_PWRSAV                 ( 0x200U )
#define MCU_SDIO_CLKCR_CLKEN                  ( 0x100U )
#define MCU_SDIO_CLKCR_CLKDIV                 ( 0xFFU )

/* Биты регистра SDIO_ARG (SDIO argument register) */
#define MCU_SDIO_ARG_BITS                     ( 0xFFFFFFFFU )
#define MCU_SDIO_ARG_CMDARG                   ( 0xFFFFFFFFU )

/* Биты регистра SDIO_CMD (SDIO command register) */
#define MCU_SDIO_CMD_BITS                     ( 0x7FFFU )
#define MCU_SDIO_CMD_CEATACMD                 ( 0x4000U )
#define MCU_SDIO_CMD_nIEN                     ( 0x2000U )
#define MCU_SDIO_CMD_ENCMDcompl               ( 0x1000U )
#define MCU_SDIO_CMD_SDIOSuspend              ( 0x800U )
#define MCU_SDIO_CMD_CPSMEN                   ( 0x400U )
#define MCU_SDIO_CMD_WAITPEND                 ( 0x200U )
#define MCU_SDIO_CMD_WAITINT                  ( 0x100U )
#define MCU_SDIO_CMD_WAITRESP                 ( 0xC0U )
#define MCU_SDIO_CMD_CMDINDEX                 ( 0x3FU )

/* Биты регистра SDIO_RESPCMD (SDIO command responce register) */
#define MCU_SDIO_RESPCMD_BITS                 ( 0x3FU )
#define MCU_SDIO_RESPCMD_RESPCMD              ( 0x3FU )

/* Биты регистра SDIO_RESP1 (SDIO responce 1 register) */
#define MCU_SDIO_RESP1_BITS                   ( 0xFFFFFFFFU )
#define MCU_SDIO_RESP1_CARDSTATUS1            ( 0xFFFFFFFFU )

/* Биты регистра SDIO_RESP2 (SDIO responce 2 register) */
#define MCU_SDIO_RESP2_BITS                   ( 0xFFFFFFFFU )
#define MCU_SDIO_RESP2_CARDSTATUS2            ( 0xFFFFFFFFU )

/* Биты регистра SDIO_RESP3 (SDIO responce 3 register) */
#define MCU_SDIO_RESP3_BITS                   ( 0xFFFFFFFFU )
#define MCU_SDIO_RESP3_CARDSTATUS3            ( 0xFFFFFFFFU )

/* Биты регистра SDIO_RESP4 (SDIO responce 4 register) */
#define MCU_SDIO_RESP4_BITS                   ( 0xFFFFFFFFU )
#define MCU_SDIO_RESP4_CARDSTATUS4            ( 0xFFFFFFFFU )

/* Биты регистра SDIO_DTIMER (SDIO data timer register) */
#define MCU_SDIO_DTIMER_BITS                  ( 0xFFFFFFFFU )
#define MCU_SDIO_DTIMER_DATATIME              ( 0xFFFFFFFFU )

/* Биты регистра SDIO_DLEN (SDIO data lengt register) */
#define MCU_SDIO_DLEN_BITS                    ( 0x1FFFFFFU )
#define MCU_SDIO_DLEN_DATALENGTH              ( 0x1FFFFFFU )

/* Биты регистра SDIO_DCTRL (SDIO data control register) */
#define MCU_SDIO_DCTRL_BITS                   ( 0xFFFU )
#define MCU_SDIO_DCTRL_SDIOEN                 ( 0x800U )
#define MCU_SDIO_DCTRL_RWMOD                  ( 0x400U )
#define MCU_SDIO_DCTRL_RWSTOP                 ( 0x200U )
#define MCU_SDIO_DCTRL_RWSTART                ( 0x100U )
#define MCU_SDIO_DCTRL_DBLOCKSIZE             ( 0xF0U )
#define MCU_SDIO_DCTRL_DMAEN                  ( 0x8U )
#define MCU_SDIO_DCTRL_DTMODE                 ( 0x4U )
#define MCU_SDIO_DCTRL_DTDIR                  ( 0x2U )
#define MCU_SDIO_DCTRL_DTEN                   ( 0x1U )

/* Биты регистра SDIO_DCOUNT (SDIO data counter register) */
#define MCU_SDIO_DCOUNT_BITS                  ( 0x1FFFFFFU )
#define MCU_SDIO_DCOUNT_DATACOUNT             ( 0x1FFFFFFU )

/* Биты регистра SDIO_STA (SDIO status register) */
#define MCU_SDIO_STA_BITS                     ( 0xFFFFFFU )
#define MCU_SDIO_STA_CEATAEND                 ( 0x800000U )
#define MCU_SDIO_STA_SDIOIT                   ( 0x400000U )
#define MCU_SDIO_STA_RXDAVL                   ( 0x200000U )
#define MCU_SDIO_STA_TXDAVL                   ( 0x100000U )
#define MCU_SDIO_STA_RXFIFOE                  ( 0x80000U )
#define MCU_SDIO_STA_TXFIFOE                  ( 0x40000U )
#define MCU_SDIO_STA_RXFIFOF                  ( 0x20000U )
#define MCU_SDIO_STA_TXFIFOF                  ( 0x10000U )
#define MCU_SDIO_STA_RXFIFOHF                 ( 0x8000U )
#define MCU_SDIO_STA_TXFIFOHE                 ( 0x4000U )
#define MCU_SDIO_STA_RXACT                    ( 0x2000U )
#define MCU_SDIO_STA_TXACT                    ( 0x1000U )
#define MCU_SDIO_STA_CMDACT                   ( 0x800U )
#define MCU_SDIO_STA_DBCKEND                  ( 0x400U )
#define MCU_SDIO_STA_STBITERR                 ( 0x200U )
#define MCU_SDIO_STA_DATAEND                  ( 0x100U )
#define MCU_SDIO_STA_CMDSENT                  ( 0x80U )
#define MCU_SDIO_STA_CMDREND                  ( 0x40U )
#define MCU_SDIO_STA_RXOVERR                  ( 0x20U )
#define MCU_SDIO_STA_TXUNDERR                 ( 0x10U )
#define MCU_SDIO_STA_DTIMEOUT                 ( 0x8U )
#define MCU_SDIO_STA_CTIMEOUT                 ( 0x4U )
#define MCU_SDIO_STA_DCRCFAIL                 ( 0x2U )
#define MCU_SDIO_STA_CCRCFAIL                 ( 0x1U )

/* Биты регистра SDIO_ICR (SDIO interrupt clear register) */
#define MCU_SDIO_ICR_BITS                     ( 0xC007FFU )
#define MCU_SDIO_ICR_CEATAENDC                ( 0x800000U )
#define MCU_SDIO_ICR_SDIOITC                  ( 0x400000U )
#define MCU_SDIO_ICR_DBCKENDC                 ( 0x400U )
#define MCU_SDIO_ICR_STBITERRC                ( 0x200U )
#define MCU_SDIO_ICR_DATAENDC                 ( 0x100U )
#define MCU_SDIO_ICR_CMDSENTC                 ( 0x80U )
#define MCU_SDIO_ICR_CMDRENDC                 ( 0x40U )
#define MCU_SDIO_ICR_RXOVERRC                 ( 0x20U )
#define MCU_SDIO_ICR_TXUNDERRC                ( 0x10U )
#define MCU_SDIO_ICR_DTIMEOUTC                ( 0x8U )
#define MCU_SDIO_ICR_CTIMEOUTC                ( 0x4U )
#define MCU_SDIO_ICR_DCRCFAILC                ( 0x2U )
#define MCU_SDIO_ICR_CCRCFAILC                ( 0x1U )

/* Биты регистра SDIO_MASK (SDIO mask register) */
#define MCU_SDIO_MASK_BITS                    ( 0xFFFFFFU )
#define MCU_SDIO_MASK_CEATAENDIE              ( 0x800000U )
#define MCU_SDIO_MASK_SDIOITIE                ( 0x400000U )
#define MCU_SDIO_MASK_RXDAVLIE                ( 0x200000U )
#define MCU_SDIO_MASK_TXDAVLIE                ( 0x100000U )
#define MCU_SDIO_MASK_RXFIFOEIE               ( 0x80000U )
#define MCU_SDIO_MASK_TXFIFOEIE               ( 0x40000U )
#define MCU_SDIO_MASK_RXFIFOFIE               ( 0x20000U )
#define MCU_SDIO_MASK_TXFIFOFIE               ( 0x10000U )
#define MCU_SDIO_MASK_RXFIFOHFIE              ( 0x8000U )
#define MCU_SDIO_MASK_TXFIFOHEIE              ( 0x4000U )
#define MCU_SDIO_MASK_RX_ACTIE                ( 0x2000U )
#define MCU_SDIO_MASK_TX_ACTIE                ( 0x1000U )
#define MCU_SDIO_MASK_CMD_ACTIE               ( 0x800U )
#define MCU_SDIO_MASK_DBCKENDIE               ( 0x400U )
#define MCU_SDIO_MASK_STBITERRIE              ( 0x200U )
#define MCU_SDIO_MASK_DATAENDIE               ( 0x100U )
#define MCU_SDIO_MASK_CMDSENTIE               ( 0x80U )
#define MCU_SDIO_MASK_CMDRENDIE               ( 0x40U )
#define MCU_SDIO_MASK_RXOVERRIE               ( 0x20U )
#define MCU_SDIO_MASK_TXUNDERRIE              ( 0x10U )
#define MCU_SDIO_MASK_DTIMEOUTIE              ( 0x8U )
#define MCU_SDIO_MASK_CTIMEOUTIE              ( 0x4U )
#define MCU_SDIO_MASK_DCRCFAILIE              ( 0x2U )
#define MCU_SDIO_MASK_CCRCFAILIE              ( 0x1U )

/* Биты регистра SDIO_FIFOCNT (SDIO FIFO counter register) */
#define MCU_SDIO_FIFOCNT_BITS                 ( 0xFFFFFFU )
#define MCU_SDIO_FIFOCNT_FIFOCOUNT            ( 0xFFFFFFU )

/* Биты регистра SDIO_FIFO (SDIO data FIFO register) */
#define MCU_SDIO_FIFO_BITS                    ( 0xFFFFFFFFU )
#define MCU_SDIO_FIFO_FIFOData                ( 0xFFFFFFFFU )


/* ----- bxCAN (controller area network) ----- */
typedef volatile struct tag_stm32f407_can {
    uint32_t        mcr;
    uint32_t        msr;
    uint32_t        tsr;
    uint32_t        rf0r;
    uint32_t        rf1r;
    uint32_t        ier;
    uint32_t        esr;
    uint32_t        btr;
    uint32_t        gap0[89];
    uint32_t        ti0r;
    uint32_t        tdt0r;
    uint32_t        tdl0r;
    uint32_t        tdh0r;
    uint32_t        ti1r;
    uint32_t        tdt1r;
    uint32_t        tdl1r;
    uint32_t        tdh1r;
    uint32_t        ti2r;
    uint32_t        tdt2r;
    uint32_t        tdl2r;
    uint32_t        tdh2r;
    uint32_t        ri0r;
    uint32_t        rdt0r;
    uint32_t        rdl0r;
    uint32_t        rdh0r;
    uint32_t        ri1r;
    uint32_t        rdt1r;
    uint32_t        rdl1r;
    uint32_t        rdh1r;
    uint32_t        gap1[13];
    uint32_t        fmr;
    uint32_t        fm1r;
    uint32_t        gap2;
    uint32_t        fs1r;
    uint32_t        gap3;
    uint32_t        ffa1r;
    uint32_t        gap4;
    uint32_t        fa1r;
    uint32_t        gap5;
    uint32_t        gap6[8];
    uint32_t        f0r1;
    uint32_t        f0r2;
    uint32_t        f1r1;
    uint32_t        f1r2;
    uint32_t        f2r1;
    uint32_t        f2r2;
    uint32_t        f3r1;
    uint32_t        f3r2;
    uint32_t        f4r1;
    uint32_t        f4r2;
    uint32_t        f5r1;
    uint32_t        f5r2;
    uint32_t        f6r1;
    uint32_t        f6r2;
    uint32_t        f7r1;
    uint32_t        f7r2;
    uint32_t        f8r1;
    uint32_t        f8r2;
    uint32_t        f9r1;
    uint32_t        f9r2;
    uint32_t        f10r1;
    uint32_t        f10r2;
    uint32_t        f11r1;
    uint32_t        f11r2;
    uint32_t        f12r1;
    uint32_t        f12r2;
    uint32_t        f13r1;
    uint32_t        f13r2;
    uint32_t        f14r1;
    uint32_t        f14r2;
    uint32_t        f15r1;
    uint32_t        f15r2;
    uint32_t        f16r1;
    uint32_t        f16r2;
    uint32_t        f17r1;
    uint32_t        f17r2;
    uint32_t        f18r1;
    uint32_t        f18r2;
    uint32_t        f19r1;
    uint32_t        f19r2;
    uint32_t        f20r1;
    uint32_t        f20r2;
    uint32_t        f21r1;
    uint32_t        f21r2;
    uint32_t        f22r1;
    uint32_t        f22r2;
    uint32_t        f23r1;
    uint32_t        f23r2;
    uint32_t        f24r1;
    uint32_t        f24r2;
    uint32_t        f25r1;
    uint32_t        f25r2;
    uint32_t        f26r1;
    uint32_t        f26r2;
    uint32_t        f27r1;
    uint32_t        f27r2;
} stm32f407_can_t;

#define STM32F407_CAN_PTR                     ( ( stm32f407_can_t * ) 0x40006400U )

/* Биты регистра MCR (master control register) */
#define MCU_CAN_MCR_BITS                      ( 0x180FFU )
#define MCU_CAN_MCR_DBF                       ( 0x10000U )
#define MCU_CAN_MCR_RESET                     ( 0x8000U ) 
#define MCU_CAN_MCR_TTCM                      ( 0x80U ) 
#define MCU_CAN_MCR_ABOM                      ( 0x40U ) 
#define MCU_CAN_MCR_AWUM                      ( 0x20U ) 
#define MCU_CAN_MCR_NART                      ( 0x10U ) 
#define MCU_CAN_MCR_RFLM                      ( 0x8U ) 
#define MCU_CAN_MCR_TXFP                      ( 0x4U ) 
#define MCU_CAN_MCR_SLEEP                     ( 0x2U ) 
#define MCU_CAN_MCR_INRQ                      ( 0x1U ) 

/* Биты регистра MSR (master status register) */
#define MCU_CAN_MSR_BITS                      ( 0xF1FU )
#define MCU_CAN_MSR_RX                        ( 0x800U )
#define MCU_CAN_MSR_SAMP                      ( 0x400U )
#define MCU_CAN_MSR_RXM                       ( 0x200U )
#define MCU_CAN_MSR_TXM                       ( 0x100U )
#define MCU_CAN_MSR_SLAKI                     ( 0x10U )
#define MCU_CAN_MSR_WKUI                      ( 0x8U )
#define MCU_CAN_MSR_ERRI                      ( 0x4U )
#define MCU_CAN_MSR_SLAK                      ( 0x2U )
#define MCU_CAN_MSR_INAK                      ( 0x1U )

/* Биты регистра TSR (transmit status register) */
#define MCU_CAN_TSR_BITS                      ( 0xFF8F8F8FU )
#define MCU_CAN_TSR_LOW2                      ( 0x80000000U )
#define MCU_CAN_TSR_LOW1                      ( 0x40000000U )
#define MCU_CAN_TSR_LOW0                      ( 0x20000000U )
#define MCU_CAN_TSR_TME2                      ( 0x10000000U )
#define MCU_CAN_TSR_TME1                      ( 0x8000000U )
#define MCU_CAN_TSR_TME0                      ( 0x4000000U )
#define MCU_CAN_TSR_CODE_MASK                 ( 0x3000000U )
#define MCU_CAN_TSR_ABRQ2                     ( 0x800000U )
#define MCU_CAN_TSR_TERR2                     ( 0x80000U )
#define MCU_CAN_TSR_ALST2                     ( 0x40000U )
#define MCU_CAN_TSR_TXOK2                     ( 0x20000U )
#define MCU_CAN_TSR_RQCP2                     ( 0x10000U )
#define MCU_CAN_TSR_ABRQ1                     ( 0x8000U )
#define MCU_CAN_TSR_TERR1                     ( 0x800U )
#define MCU_CAN_TSR_ALST1                     ( 0x400U )
#define MCU_CAN_TSR_TXOK1                     ( 0x200U )
#define MCU_CAN_TSR_RQCP1                     ( 0x100U )
#define MCU_CAN_TSR_ABRQ0                     ( 0x80U )
#define MCU_CAN_TSR_TERR0                     ( 0x8U )
#define MCU_CAN_TSR_ALST0                     ( 0x4U )
#define MCU_CAN_TSR_TXOK0                     ( 0x2U )
#define MCU_CAN_TSR_RQCP0                     ( 0x1U )

/* Биты регистра RF0R (receive FIFO 0 register) */
#define MCU_CAN_RF0R_BITS                     ( 0x3BU )
#define MCU_CAN_RF0R_RFOM0                    ( 0x20U )
#define MCU_CAN_RF0R_FOVR0                    ( 0x10U )
#define MCU_CAN_RF0R_FULL0                    ( 0x8U )
#define MCU_CAN_RF0R_FMP0_MASK                ( 0x3U )

/* Биты регистра RF1R (receive FIFO 1 register) */
#define MCU_CAN_RF1R_BITS                     ( 0x3BU )
#define MCU_CAN_RF1R_RFOM1                    ( 0x20U )
#define MCU_CAN_RF1R_FOVR1                    ( 0x10U )
#define MCU_CAN_RF1R_FULL1                    ( 0x8U )
#define MCU_CAN_RF1R_FMP1_MASK                ( 0x3U )

/* Биты регистра IER (interrupt enable register) */
#define MCU_CAN_IER_BITS                      ( 0x38F7FU )
#define MCU_CAN_IER_SLKIE                     ( 0x20000U )
#define MCU_CAN_IER_WKUIE                     ( 0x10000U )
#define MCU_CAN_IER_ERRIE                     ( 0x8000U )
#define MCU_CAN_IER_LECIE                     ( 0x800U )
#define MCU_CAN_IER_BOFIE                     ( 0x400U )
#define MCU_CAN_IER_EPVIE                     ( 0x200U )
#define MCU_CAN_IER_EWGIE                     ( 0x100U )
#define MCU_CAN_IER_FOVIE1                    ( 0x40U )
#define MCU_CAN_IER_FFIE1                     ( 0x20U )
#define MCU_CAN_IER_FMPIE1                    ( 0x10U )
#define MCU_CAN_IER_FOVIE0                    ( 0x8U )
#define MCU_CAN_IER_FFIE0                     ( 0x4U )
#define MCU_CAN_IER_FMPIE0                    ( 0x2U )
#define MCU_CAN_IER_TMEIE                     ( 0x1U )

/* Биты регистра ESR (error status register) */
#define MCU_CAN_ESR_BITS                      ( 0xFFFF0077U )
#define MCU_CAN_ESR_REC_MASK                  ( 0xFF000000U )
#define MCU_CAN_ESR_TEC_MASK                  ( 0xFF0000U )
#define MCU_CAN_ESR_LEC_MASK                  ( 0x70U )
#define MCU_CAN_ESR_BOFF                      ( 0x4U )
#define MCU_CAN_ESR_EPVF                      ( 0x2U )
#define MCU_CAN_ESR_EWGF                      ( 0x1U )

/* Биты регистра BTR (bit timing register) */
#define MCU_CAN_BTR_BITS                      ( 0xC37F03FFU )
#define MCU_CAN_BTR_SILM                      ( 0x80000000U )
#define MCU_CAN_BTR_LBKM                      ( 0x40000000U )
#define MCU_CAN_BTR_SJW_MASK                  ( 0x3000000U )
#define MCU_CAN_BTR_TS2_MASK                  ( 0x700000U )
#define MCU_CAN_BTR_TS1_MASK                  ( 0xF0000U )
#define MCU_CAN_BTR_BRP_MASK                  ( 0x3FFU )

/* Биты регистра TI0R (TX mailbox identifier register 0) */
#define MCU_CAN_TI0R_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_TI0R_STID_EXID_MASK           ( 0xFFE00000U )
#define MCU_CAN_TI0R_EXID_MASK                ( 0x1FFFF8U )
#define MCU_CAN_TI0R_IDE                      ( 0x4U )
#define MCU_CAN_TI0R_RTR                      ( 0x2U )
#define MCU_CAN_TI0R_TXRQ                     ( 0x1U )

/* Биты регистра TDT0R (mailbox data length control and time stamp register 0) */
#define MCU_CAN_TDT0R_BITS                    ( 0xFFFF010FU )
#define MCU_CAN_TDT0R_TIME_MASK               ( 0xFFFF0000U )
#define MCU_CAN_TDT0R_TGT                     ( 0x100U )
#define MCU_CAN_TDT0R_DLC_MASK                ( 0xFU )

/* Биты регистра TDL0R (mailbox data low register 0) */
#define MCU_CAN_TDL0R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_TDL0R_DATA3_MASK              ( 0xFF000000U )
#define MCU_CAN_TDL0R_DATA2_MASK              ( 0xFF0000U )
#define MCU_CAN_TDL0R_DATA1_MASK              ( 0xFF00U )
#define MCU_CAN_TDL0R_DATA0_MASK              ( 0xFFU )

/* Биты регистра TDH0R (mailbox data high register 0) */
#define MCU_CAN_TDH0R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_TDH0R_DATA7_MASK              ( 0xFF000000U )
#define MCU_CAN_TDH0R_DATA6_MASK              ( 0xFF0000U )
#define MCU_CAN_TDH0R_DATA5_MASK              ( 0xFF00U )
#define MCU_CAN_TDH0R_DATA4_MASK              ( 0xFFU )

/* Биты регистра TI1R (TX mailbox identifier register 1) */
#define MCU_CAN_TI1R_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_TI1R_STID_EXID_MASK           ( 0xFFE00000U )
#define MCU_CAN_TI1R_EXID_MASK                ( 0x1FFFF8U )
#define MCU_CAN_TI1R_IDE                      ( 0x4U )
#define MCU_CAN_TI1R_RTR                      ( 0x2U )
#define MCU_CAN_TI1R_TXRQ                     ( 0x1U )

/* Биты регистра TDT1R (mailbox data length control and time stamp register 1) */
#define MCU_CAN_TDT1R_BITS                    ( 0xFFFF010FU )
#define MCU_CAN_TDT1R_TIME_MASK               ( 0xFFFF0000U )
#define MCU_CAN_TDT1R_TGT                     ( 0x100U )
#define MCU_CAN_TDT1R_DLC_MASK                ( 0xFU )

/* Биты регистра TDL1R (mailbox data low register 1) */
#define MCU_CAN_TDL1R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_TDL1R_DATA3_MASK              ( 0xFF000000U )
#define MCU_CAN_TDL1R_DATA2_MASK              ( 0xFF0000U )
#define MCU_CAN_TDL1R_DATA1_MASK              ( 0xFF00U )
#define MCU_CAN_TDL1R_DATA0_MASK              ( 0xFFU )

/* Биты регистра TDH1R (mailbox data high register 1) */
#define MCU_CAN_TDH1R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_TDH1R_DATA7_MASK              ( 0xFF000000U )
#define MCU_CAN_TDH1R_DATA6_MASK              ( 0xFF0000U )
#define MCU_CAN_TDH1R_DATA5_MASK              ( 0xFF00U )
#define MCU_CAN_TDH1R_DATA4_MASK              ( 0xFFU )

/* Биты регистра TI2R (TX mailbox identifier register 2) */
#define MCU_CAN_TI2R_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_TI2R_STID_EXID_MASK           ( 0xFFE00000U )
#define MCU_CAN_TI2R_EXID_MASK                ( 0x1FFFF8U )
#define MCU_CAN_TI2R_IDE                      ( 0x4U )
#define MCU_CAN_TI2R_RTR                      ( 0x2U )
#define MCU_CAN_TI2R_TXRQ                     ( 0x1U )

/* Биты регистра TDT2R (mailbox data length control and time stamp register 2) */
#define MCU_CAN_TDT2R_BITS                    ( 0xFFFF010FU )
#define MCU_CAN_TDT2R_TIME_MASK               ( 0xFFFF0000U )
#define MCU_CAN_TDT2R_TGT                     ( 0x100U )
#define MCU_CAN_TDT2R_DLC_MASK                ( 0xFU )

/* Биты регистра TDL2R (mailbox data low register 2) */
#define MCU_CAN_TDL2R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_TDL2R_DATA3_MASK              ( 0xFF000000U )
#define MCU_CAN_TDL2R_DATA2_MASK              ( 0xFF0000U )
#define MCU_CAN_TDL2R_DATA1_MASK              ( 0xFF00U )
#define MCU_CAN_TDL2R_DATA0_MASK              ( 0xFFU )

/* Биты регистра TDH2R (mailbox data high register 2) */
#define MCU_CAN_TDH2R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_TDH2R_DATA7_MASK              ( 0xFF000000U )
#define MCU_CAN_TDH2R_DATA6_MASK              ( 0xFF0000U )
#define MCU_CAN_TDH2R_DATA5_MASK              ( 0xFF00U )
#define MCU_CAN_TDH2R_DATA4_MASK              ( 0xFFU )

/* Биты регистра RI0R (receive FIFO mailbox identifier register 0) */
#define MCU_CAN_RI0R_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_RI0R_STID_EXID_MASK           ( 0xFFE00000U )
#define MCU_CAN_RI0R_EXID_MASK                ( 0x1FFFF8U )
#define MCU_CAN_RI0R_IDE                      ( 0x4U )
#define MCU_CAN_RI0R_RTR                      ( 0x2U )

/* Биты регистра RDT0R (receive FIFO mailbox data length control and time stamp register 0) */
#define MCU_CAN_RDT0R_BITS                    ( 0xFFFFFF0FU )
#define MCU_CAN_RDT0R_TIME_MASK               ( 0xFFFF0000U )
#define MCU_CAN_RDT0R_FMI_MASK                ( 0xFF00U )
#define MCU_CAN_RDT0R_DLC_MASK                ( 0xFU )

/* Биты регистра RDL0R (receive FIFO mailbox data low register 0) */
#define MCU_CAN_RDL0R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_RDL0R_DATA3_MASK              ( 0xFF000000U )
#define MCU_CAN_RDL0R_DATA2_MASK              ( 0xFF0000U )
#define MCU_CAN_RDL0R_DATA1_MASK              ( 0xFF00U )
#define MCU_CAN_RDL0R_DATA0_MASK              ( 0xFFU )

/* Биты регистра RDH0R (receive FIFO mailbox data high register 0) */
#define MCU_CAN_RDH0R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_RDH0R_DATA7_MASK              ( 0xFF000000U )
#define MCU_CAN_RDH0R_DATA6_MASK              ( 0xFF0000U )
#define MCU_CAN_RDH0R_DATA5_MASK              ( 0xFF00U )
#define MCU_CAN_RDH0R_DATA4_MASK              ( 0xFFU )

/* Биты регистра RI1R (receive FIFO mailbox identifier register 1) */
#define MCU_CAN_RI1R_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_RI1R_STID_EXID_MASK           ( 0xFFE00000U )
#define MCU_CAN_RI1R_EXID_MASK                ( 0x1FFFF8U )
#define MCU_CAN_RI1R_IDE                      ( 0x4U )
#define MCU_CAN_RI1R_RTR                      ( 0x2U )

/* Биты регистра RDT1R (receive FIFO mailbox data length control and time stamp register 1) */
#define MCU_CAN_RDT1R_BITS                    ( 0xFFFFFF0FU )
#define MCU_CAN_RDT1R_TIME_MASK               ( 0xFFFF0000U )
#define MCU_CAN_RDT1R_FMI_MASK                ( 0xFF00U )
#define MCU_CAN_RDT1R_DLC_MASK                ( 0xFU )

/* Биты регистра RDL1R (receive FIFO mailbox data low register 1) */
#define MCU_CAN_RDL1R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_RDL1R_DATA3_MASK              ( 0xFF000000U )
#define MCU_CAN_RDL1R_DATA2_MASK              ( 0xFF0000U )
#define MCU_CAN_RDL1R_DATA1_MASK              ( 0xFF00U )
#define MCU_CAN_RDL1R_DATA0_MASK              ( 0xFFU )

/* Биты регистра RDH1R (receive FIFO mailbox data high register 1) */
#define MCU_CAN_RDH1R_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_RDH1R_DATA7_MASK              ( 0xFF000000U )
#define MCU_CAN_RDH1R_DATA6_MASK              ( 0xFF0000U )
#define MCU_CAN_RDH1R_DATA5_MASK              ( 0xFF00U )
#define MCU_CAN_RDH1R_DATA4_MASK              ( 0xFFU )

/* Биты регистра FMR (filter master register) */
#define MCU_CAN_FMR_BITS                      ( 0x3F01U )
#define MCU_CAN_FMR_CAN2SB_MASK               ( 0x3F00U )
#define MCU_CAN_FMR_FINIT                     ( 0x1U )

/* Биты регистра FM1R (filter mode register) */
#define MCU_CAN_FM1R_BITS                     ( 0xFFFFFFFU )
#define MCU_CAN_FM1R_FBM27                    ( 0x8000000U )
#define MCU_CAN_FM1R_FBM26                    ( 0x4000000U )
#define MCU_CAN_FM1R_FBM25                    ( 0x2000000U )
#define MCU_CAN_FM1R_FBM24                    ( 0x1000000U )
#define MCU_CAN_FM1R_FBM23                    ( 0x800000U )
#define MCU_CAN_FM1R_FBM22                    ( 0x400000U )
#define MCU_CAN_FM1R_FBM21                    ( 0x200000U )
#define MCU_CAN_FM1R_FBM20                    ( 0x100000U )
#define MCU_CAN_FM1R_FBM19                    ( 0x80000U )
#define MCU_CAN_FM1R_FBM18                    ( 0x40000U )
#define MCU_CAN_FM1R_FBM17                    ( 0x20000U )
#define MCU_CAN_FM1R_FBM16                    ( 0x10000U )
#define MCU_CAN_FM1R_FBM15                    ( 0x8000U )
#define MCU_CAN_FM1R_FBM14                    ( 0x4000U )
#define MCU_CAN_FM1R_FBM13                    ( 0x2000U )
#define MCU_CAN_FM1R_FBM12                    ( 0x1000U )
#define MCU_CAN_FM1R_FBM11                    ( 0x800U )
#define MCU_CAN_FM1R_FBM10                    ( 0x400U )
#define MCU_CAN_FM1R_FBM9                     ( 0x200U )
#define MCU_CAN_FM1R_FBM8                     ( 0x100U )
#define MCU_CAN_FM1R_FBM7                     ( 0x80U )
#define MCU_CAN_FM1R_FBM6                     ( 0x40U )
#define MCU_CAN_FM1R_FBM5                     ( 0x20U )
#define MCU_CAN_FM1R_FBM4                     ( 0x10U )
#define MCU_CAN_FM1R_FBM3                     ( 0x8U )
#define MCU_CAN_FM1R_FBM2                     ( 0x4U )
#define MCU_CAN_FM1R_FBM1                     ( 0x2U )
#define MCU_CAN_FM1R_FBM0                     ( 0x1U )

/* Биты регистра FS1R (filter scale register) */
#define MCU_CAN_FS1R_BITS                     ( 0xFFFFFFFU )
#define MCU_CAN_FS1R_FSC27                    ( 0x8000000U )
#define MCU_CAN_FS1R_FSC26                    ( 0x4000000U )
#define MCU_CAN_FS1R_FSC25                    ( 0x2000000U )
#define MCU_CAN_FS1R_FSC24                    ( 0x1000000U )
#define MCU_CAN_FS1R_FSC23                    ( 0x800000U )
#define MCU_CAN_FS1R_FSC22                    ( 0x400000U )
#define MCU_CAN_FS1R_FSC21                    ( 0x200000U )
#define MCU_CAN_FS1R_FSC20                    ( 0x100000U )
#define MCU_CAN_FS1R_FSC19                    ( 0x80000U )
#define MCU_CAN_FS1R_FSC18                    ( 0x40000U )
#define MCU_CAN_FS1R_FSC17                    ( 0x20000U )
#define MCU_CAN_FS1R_FSC16                    ( 0x10000U )
#define MCU_CAN_FS1R_FSC15                    ( 0x8000U )
#define MCU_CAN_FS1R_FSC14                    ( 0x4000U )
#define MCU_CAN_FS1R_FSC13                    ( 0x2000U )
#define MCU_CAN_FS1R_FSC12                    ( 0x1000U )
#define MCU_CAN_FS1R_FSC11                    ( 0x800U )
#define MCU_CAN_FS1R_FSC10                    ( 0x400U )
#define MCU_CAN_FS1R_FSC9                     ( 0x200U )
#define MCU_CAN_FS1R_FSC8                     ( 0x100U )
#define MCU_CAN_FS1R_FSC7                     ( 0x80U )
#define MCU_CAN_FS1R_FSC6                     ( 0x40U )
#define MCU_CAN_FS1R_FSC5                     ( 0x20U )
#define MCU_CAN_FS1R_FSC4                     ( 0x10U )
#define MCU_CAN_FS1R_FSC3                     ( 0x8U )
#define MCU_CAN_FS1R_FSC2                     ( 0x4U )
#define MCU_CAN_FS1R_FSC1                     ( 0x2U )
#define MCU_CAN_FS1R_FSC0                     ( 0x1U )

/* Биты регистра FFA1R (filter FIFO assignment register) */
#define MCU_CAN_FFA1R_BITS                    ( 0xFFFFFFFU )
#define MCU_CAN_FFA1R_FFA27                   ( 0x8000000U )
#define MCU_CAN_FFA1R_FFA26                   ( 0x4000000U )
#define MCU_CAN_FFA1R_FFA25                   ( 0x2000000U )
#define MCU_CAN_FFA1R_FFA24                   ( 0x1000000U )
#define MCU_CAN_FFA1R_FFA23                   ( 0x800000U )
#define MCU_CAN_FFA1R_FFA22                   ( 0x400000U )
#define MCU_CAN_FFA1R_FFA21                   ( 0x200000U )
#define MCU_CAN_FFA1R_FFA20                   ( 0x100000U )
#define MCU_CAN_FFA1R_FFA19                   ( 0x80000U )
#define MCU_CAN_FFA1R_FFA18                   ( 0x40000U )
#define MCU_CAN_FFA1R_FFA17                   ( 0x20000U )
#define MCU_CAN_FFA1R_FFA16                   ( 0x10000U )
#define MCU_CAN_FFA1R_FFA15                   ( 0x8000U )
#define MCU_CAN_FFA1R_FFA14                   ( 0x4000U )
#define MCU_CAN_FFA1R_FFA13                   ( 0x2000U )
#define MCU_CAN_FFA1R_FFA12                   ( 0x1000U )
#define MCU_CAN_FFA1R_FFA11                   ( 0x800U )
#define MCU_CAN_FFA1R_FFA10                   ( 0x400U )
#define MCU_CAN_FFA1R_FFA9                    ( 0x200U )
#define MCU_CAN_FFA1R_FFA8                    ( 0x100U )
#define MCU_CAN_FFA1R_FFA7                    ( 0x80U )
#define MCU_CAN_FFA1R_FFA6                    ( 0x40U )
#define MCU_CAN_FFA1R_FFA5                    ( 0x20U )
#define MCU_CAN_FFA1R_FFA4                    ( 0x10U )
#define MCU_CAN_FFA1R_FFA3                    ( 0x8U )
#define MCU_CAN_FFA1R_FFA2                    ( 0x4U )
#define MCU_CAN_FFA1R_FFA1                    ( 0x2U )
#define MCU_CAN_FFA1R_FFA0                    ( 0x1U )

/* Биты регистра FA1R (filter MCU_ACTivation register) */
#define MCU_CAN_FA1R_BITS                     ( 0xFFFFFFFU )
#define MCU_CAN_FA1R_FMCU_ACT27               ( 0x8000000U )
#define MCU_CAN_FA1R_FMCU_ACT26               ( 0x4000000U )
#define MCU_CAN_FA1R_FMCU_ACT25               ( 0x2000000U )
#define MCU_CAN_FA1R_FMCU_ACT24               ( 0x1000000U )
#define MCU_CAN_FA1R_FMCU_ACT23               ( 0x800000U )
#define MCU_CAN_FA1R_FMCU_ACT22               ( 0x400000U )
#define MCU_CAN_FA1R_FMCU_ACT21               ( 0x200000U )
#define MCU_CAN_FA1R_FMCU_ACT20               ( 0x100000U )
#define MCU_CAN_FA1R_FMCU_ACT19               ( 0x80000U )
#define MCU_CAN_FA1R_FMCU_ACT18               ( 0x40000U )
#define MCU_CAN_FA1R_FMCU_ACT17               ( 0x20000U )
#define MCU_CAN_FA1R_FMCU_ACT16               ( 0x10000U )
#define MCU_CAN_FA1R_FMCU_ACT15               ( 0x8000U )
#define MCU_CAN_FA1R_FMCU_ACT14               ( 0x4000U )
#define MCU_CAN_FA1R_FMCU_ACT13               ( 0x2000U )
#define MCU_CAN_FA1R_FMCU_ACT12               ( 0x1000U )
#define MCU_CAN_FA1R_FMCU_ACT11               ( 0x800U )
#define MCU_CAN_FA1R_FMCU_ACT10               ( 0x400U )
#define MCU_CAN_FA1R_FMCU_ACT9                ( 0x200U )
#define MCU_CAN_FA1R_FMCU_ACT8                ( 0x100U )
#define MCU_CAN_FA1R_FMCU_ACT7                ( 0x80U )
#define MCU_CAN_FA1R_FMCU_ACT6                ( 0x40U )
#define MCU_CAN_FA1R_FMCU_ACT5                ( 0x20U )
#define MCU_CAN_FA1R_FMCU_ACT4                ( 0x10U )
#define MCU_CAN_FA1R_FMCU_ACT3                ( 0x8U )
#define MCU_CAN_FA1R_FMCU_ACT2                ( 0x4U )
#define MCU_CAN_FA1R_FMCU_ACT1                ( 0x2U )
#define MCU_CAN_FA1R_FMCU_ACT0                ( 0x1U )

/* Биты регистра F0R1 (Filter bank 0 register 1) */
#define MCU_CAN_F0R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F0R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F0R2 (Filter bank 0 register 2) */
#define MCU_CAN_F0R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F0R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F1R1 (Filter bank 1 register 1) */
#define MCU_CAN_F1R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F1R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F1R2 (Filter bank 1 register 2) */
#define MCU_CAN_F1R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F1R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F2R1 (Filter bank 2 register 1) */
#define MCU_CAN_F2R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F2R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F2R2 (Filter bank 2 register 2) */
#define MCU_CAN_F2R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F2R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F3R1 (Filter bank 3 register 1) */
#define MCU_CAN_F3R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F3R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F3R2 (Filter bank 3 register 2) */
#define MCU_CAN_F3R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F3R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F4R1 (Filter bank 4 register 1) */
#define MCU_CAN_F4R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F4R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F4R2 (Filter bank 4 register 2) */
#define MCU_CAN_F4R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F4R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F5R1 (Filter bank 5 register 1) */
#define MCU_CAN_F5R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F5R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F5R2 (Filter bank 5 register 2) */
#define MCU_CAN_F5R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F5R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F6R1 (Filter bank 6 register 1) */
#define MCU_CAN_F6R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F6R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F6R2 (Filter bank 6 register 2) */
#define MCU_CAN_F6R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F6R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F7R1 (Filter bank 7 register 1) */
#define MCU_CAN_F7R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F7R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F7R2 (Filter bank 7 register 2) */
#define MCU_CAN_F7R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F7R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F8R1 (Filter bank 8 register 1) */
#define MCU_CAN_F8R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F8R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F8R2 (Filter bank 8 register 2) */
#define MCU_CAN_F8R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F8R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F9R1 (Filter bank 9 register 1) */
#define MCU_CAN_F9R1_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F9R1_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F9R2 (Filter bank 9 register 2) */
#define MCU_CAN_F9R2_BITS                     ( 0xFFFFFFFFU )
#define MCU_CAN_F9R2_FB_MASK                  ( 0xFFFFFFFFU )

/* Биты регистра F10R1 (Filter bank 10 register 1) */
#define MCU_CAN_F10R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F10R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F10R2 (Filter bank 10 register 2) */
#define MCU_CAN_F10R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F10R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F11R1 (Filter bank 11 register 1) */
#define MCU_CAN_F11R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F11R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F11R2 (Filter bank 11 register 2) */
#define MCU_CAN_F11R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F11R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F12R1 (Filter bank 12 register 1) */
#define MCU_CAN_F12R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F12R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F12R2 (Filter bank 12 register 2) */
#define MCU_CAN_F12R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F12R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F13R1 (Filter bank 13 register 1) */
#define MCU_CAN_F13R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F13R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F13R2 (Filter bank 13 register 2) */
#define MCU_CAN_F13R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F13R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F14R1 (Filter bank 14 register 1) */
#define MCU_CAN_F14R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F14R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F14R2 (Filter bank 14 register 2) */
#define MCU_CAN_F14R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F14R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F15R1 (Filter bank 15 register 1) */
#define MCU_CAN_F15R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F15R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F15R2 (Filter bank 15 register 2) */
#define MCU_CAN_F15R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F15R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F16R1 (Filter bank 16 register 1) */
#define MCU_CAN_F16R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F16R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F16R2 (Filter bank 16 register 2) */
#define MCU_CAN_F16R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F16R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F17R1 (Filter bank 17 register 1) */
#define MCU_CAN_F17R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F17R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F17R2 (Filter bank 17 register 2) */
#define MCU_CAN_F17R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F17R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F18R1 (Filter bank 18 register 1) */
#define MCU_CAN_F18R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F18R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F18R2 (Filter bank 18 register 2) */
#define MCU_CAN_F18R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F18R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F19R1 (Filter bank 19 register 1) */
#define MCU_CAN_F19R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F19R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F19R2 (Filter bank 19 register 2) */
#define MCU_CAN_F19R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F19R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F20R1 (Filter bank 20 register 1) */
#define MCU_CAN_F20R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F20R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F20R2 (Filter bank 20 register 2) */
#define MCU_CAN_F20R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F20R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F21R1 (Filter bank 21 register 1) */
#define MCU_CAN_F21R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F21R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F21R2 (Filter bank 21 register 2) */
#define MCU_CAN_F21R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F21R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F21R2 (Filter bank 22 register 1) */
#define MCU_CAN_F22R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F22R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F22R2 (Filter bank 22 register 2) */
#define MCU_CAN_F22R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F22R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F23R1 (Filter bank 23 register 1) */
#define MCU_CAN_F23R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F23R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F23R2 (Filter bank 23 register 2) */
#define MCU_CAN_F23R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F23R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F24R1 (Filter bank 24 register 1) */
#define MCU_CAN_F24R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F24R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F24R2 (Filter bank 24 register 2) */
#define MCU_CAN_F24R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F24R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F25R1 (Filter bank 25 register 1) */
#define MCU_CAN_F25R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F25R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F25R2 (Filter bank 25 register 2) */
#define MCU_CAN_F25R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F25R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F26R1 (Filter bank 26 register 1) */
#define MCU_CAN_F26R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F26R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F26R2 (Filter bank 26 register 2) */
#define MCU_CAN_F26R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F26R2_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F27R1 (Filter bank 27 register 1) */
#define MCU_CAN_F27R1_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F27R1_FB_MASK                 ( 0xFFFFFFFFU )

/* Биты регистра F27R2 (Filter bank 27 register 2) */
#define MCU_CAN_F27R2_BITS                    ( 0xFFFFFFFFU )
#define MCU_CAN_F27R2_FB_MASK                 ( 0xFFFFFFFFU )

#define BIT_0								    0x1U 
#define BIT_1									0x2U
#define BIT_2									0x4U
#define BIT_3									0x8U
#define BIT_4									0x10U
#define BIT_5									0x20U
#define BIT_6									0x40U
#define BIT_7									0x80U
#define BIT_8									0x100U
#define BIT_9									0x200U
#define BIT_10									0x400U
#define BIT_11									0x800U
#define BIT_12									0x1000U
#define BIT_13									0x2000U
#define BIT_14									0x4000U
#define BIT_15									0x8000U
#define BIT_16									0x10000U
#define BIT_17									0x20000U
#define BIT_18									0x40000U
#define BIT_19									0x80000U
#define BIT_20									0x100000U
#define BIT_21									0x200000U
#define BIT_22									0x400000U
#define BIT_23									0x800000U
#define BIT_24									0x1000000U
#define BIT_25									0x2000000U
#define BIT_26									0x4000000U
#define BIT_27									0x8000000U
#define BIT_28									0x10000000U
#define BIT_29									0x20000000U
#define BIT_30									0x40000000U
#define BIT_31									0x80000000U

#ifdef __cplusplus
}
#endif

#endif /* __STM32F407_H__ */
