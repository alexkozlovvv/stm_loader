#include "sections_bsp.h"
#include "base_types.h"
#include "os_def.h"
#include "eumw.h"
#include "STM32F407.h"
#include "irq.h"
#include "irq_levels.h"
#include "init.h"
#include "init_mem.h"
#include "timer.h"

/* ��������� ���������� �����
   NO_ERROR - ��� ������ */
LOADER_DATA error_t init_error;

#define MCU_RCC_PLLCFGR_PLLM_4     ( 0x00000004U )
#define MCU_RCC_PLLCFGR_PLLN_100   ( 0x00001900U )
#define MCU_RCC_PLLCFGR_PLLP_2     ( 0x00000000U )
#define MCU_RCC_PLLCFGR_PLLQ_4     ( 0x04000000U )
#define MCU_FLASH_ACR_LATENCY_3WS  ( 0x00000003U )

/* ��������� �������������� ��������� �������� ���������� */
void un_init_hardware( void )
{

    /* �������������� �������� ����������� ���������� */
    un_init_irq();

    /* �������������� �������� CLK */
    un_init_clk();

    /* �������������� �������� GPIO */
    un_init_gpio();

}

/* ������������ ������������� ����������:
   1. ������������� FPU;
   2. ��������� PLL;
   3. ��������� ������/������� ���������������;
   ����������:
   1. ���������� �� ������ ������������ init_startup;
   2. ������������� ������� ������������ ������������ ��� �������������
      ������������� �� �������
 */
void init_hardware( void )
{

    /* ������������� ����� � ��������� ������ (FPU). 
       ��������� 20-23 ��� � Coprocessor Access Control Register, 
       ���������� ������� ������� � ������������� CP10 � CP11 */
    STM32F407_SCB_PTR->cpacr |= ((3UL << 10*2)|(3UL << 11*2));

    /* ��������� ������ */
    init_clk();

    /* ��������� ������� ���������� */
    init_gpio();  

    /* ������� ������ �������� ��������� �� �������� ������� */

    init_error = NO_ERROR;

}

/* ������������ ������������� �� */
void init_software( void )
{
    /* ������������� ����������� ���������� */
    if ( NO_ERROR != init_irq() ) {
        /* ������ ������������� */
        init_error = CRITICAL_ERROR;
    }

    /* ������� ������ ��� ���������� ������������� ��� ���������,
       �� ��������� ��������� � MSR */
}


/*  ������������ ������������� ������ 
    System Clock source            | PLL (HSE)
    SYSCLK(Hz)                     | 100000000
    HCLK(Hz)                       | 100000000
    AHB Prescaler                  | 1
    APB1 Prescaler                 | 4
    APB2 Prescaler                 | 4
    HSE Frequency(Hz)              | 8000000
    PLL_M                          | 4
    PLL_N                          | 100
    PLL_P                          | 2
    PLL_Q                          | 4
    PLLI2S_N                       | NA
    PLLI2S_R                       | NA
    I2S input clock                | NA
    VDD(V)                         | 3.3
    Main regulator output voltage  | Scale1 mode
    Flash Latency(WS)              | 3
    Prefetch Buffer                | ON
    Instruction cache              | ON
    Data cache                     | ON
    Require 48MHz for USB OTG FS,  | Disabled
    SDIO and RNG clock             |
*/
static void init_clk( void )
{
    stm32f407_rcc_t *ptr_reg_rcc;
    stm32f407_pwr_t *ptr_reg_pwr;
    stm32f407_flash_t *ptr_reg_flash;

    uint32_t        temp;

    /* ��������� �� ������ �������� ��������� */
    ptr_reg_rcc = STM32F407_RCC_PTR;               /* Reset and clock control */
    ptr_reg_pwr = STM32F407_PWR_PTR;               /* Power controller */
    ptr_reg_flash = STM32F407_FLASH_PTR;           /* Flash interface */

    /* ��������� HSI */
    temp  = IN_REG( ptr_reg_rcc->cr ); 
    temp |= MCU_RCC_CR_HSION;
    OUT_REG( ptr_reg_rcc->cr, temp );              /* ��������� ����������� ���������������� ���������� */

    while ( 0U == ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_HSIRDY ) ) {
        /* �������� ������ HSI �� ������� ����� */
        ;
    } 
    
    /* ����� cfgr �������� */
    OUT_REG( ptr_reg_rcc->cfgr, 0x0U );            /* ����� �������������� � ����� ��������� � �������� ��������� ������������ 
                                                      ��������� ���� ���������� ��������� 16 ��� */

    /* ���������� HSEON, CSSON � PLLON */
    temp  = IN_REG( ptr_reg_rcc->cr ); 
    temp &= ~ ( uint32_t )  ( MCU_RCC_CR_PLLON     /* ���������� ��������, ���� PLL ����� ��� �������� �� ��������� ��������� ���� */
                            | MCU_RCC_CR_CSSON     /* Clock security system */
                            | MCU_RCC_CR_HSEON );  /* ���������� �������� �� �������� � PLL */
    OUT_REG( ptr_reg_rcc->cr, temp );

    /* ����� PLLCFGR */
    temp = 0x24003010U;                            /* �������� �������� �� ��������� (����� reset) */
    OUT_REG( ptr_reg_rcc->pllcfgr, temp );         /* ����� �������� �������������� � ����� ������ ������������� �������/��������� ������ */

    /* ����� HSEBYP */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp &= ~ ( uint32_t ) MCU_RCC_CR_HSEBYP;
    OUT_REG( ptr_reg_rcc->cr, temp );              /* ���������� ����������� ���������� ������ � �������� ����������,
                                                      �������� ������ ����� ���������� HSE */

    /* ���������� ���� ���������� */
    OUT_REG( ptr_reg_rcc->cir, 0x0U );             /* ���������� ���������� ����������� ������ � ������������ ������ */     
                               
    /* ��������� HSE ( 8 ��� ) */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp |= MCU_RCC_CR_HSEON;
    OUT_REG( ptr_reg_rcc->cr, temp );

    while ( 0U == ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_HSERDY ) ) {
        /* �������� ������ HSE �� ������� ����� */
        ;
    }

    /* ��������� ������������ ����� ���������� �������� */
    temp  = IN_REG( ptr_reg_rcc->apb1enr );
    temp |= MCU_RCC_APB1ENR_PWREN;
    OUT_REG( ptr_reg_rcc->apb1enr, temp );         /* ��������� ���� PWREN ( Power interface clock enable) */

    /* ����� ������ ������ ����������� ���������� ���������� */
    temp  = IN_REG( ptr_reg_pwr->cr );
    temp |= MCU_PWR_CR_VOS;                        /* ����� ������ "VOS = 1 (�� ���������) */
    OUT_REG( ptr_reg_pwr->cr, temp );
    
    /* HCLK = SYSCLK / 1 */ 
    temp  = IN_REG( ptr_reg_rcc->cfgr );
    temp |= ( MCU_RCC_CFGR_PPRE1_0
            | MCU_RCC_CFGR_PPRE1_2                 /* ����������� ������� APB1 = 4 */ 
            | MCU_RCC_CFGR_PPRE2_0
            | MCU_RCC_CFGR_PPRE2_2 );              /* ����������� ������� APB2 = 4 */
    OUT_REG( ptr_reg_rcc->cfgr, temp );            /* HCLK = SYSCLK */
    
    /* ��������  PLL_M, PLL_N, PLL_P, PLL_Q � RCC_PLLCFGR_PLLSRC_HSE */
    temp  = IN_REG( ptr_reg_rcc->pllcfgr );
    temp &= ~ ( uint32_t ) 
            ( MCU_RCC_PLLCFGR_PLLM_BITS 
            | MCU_RCC_PLLCFGR_PLLN_BITS 
            | MCU_RCC_PLLCFGR_PLLP_BITS
            | MCU_RCC_PLLCFGR_PLLQ_BITS );

    temp |= ( MCU_RCC_PLLCFGR_PLLM_4               /* ���������� � ������� �� 4 */
            | MCU_RCC_PLLCFGR_PLLN_100             /* ���������� N ��������� �� 100 */
            | MCU_RCC_PLLCFGR_PLLP_2               /* ���������� P ������� �� 2 */
            | MCU_RCC_PLLCFGR_PLLQ_4               /* ���������� Q ������� �� 4 */
            | MCU_RCC_PLLCFGR_PLLSRC );
    OUT_REG( ptr_reg_rcc->pllcfgr, temp );

    /* ��������� PLL */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp |= MCU_RCC_CR_PLLON;
    OUT_REG( ptr_reg_rcc->cr, temp );

    while ( 0U == ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_PLLRDY ) ) {
    /* �������� ������ �� ������� ����� PLL*/
        ;
    }

    /* ��������  Flash prefetch, Instruction cache, Data cache � wait state */
    temp  = IN_REG( ptr_reg_flash->acr );   
    temp |= ( MCU_FLASH_ACR_PRFTEN                 /* ��������� ������ ������ ��������������� ������� ���������� */
            | MCU_FLASH_ACR_ICEN                   /* ��������� ������ ���� ���������� */
            | MCU_FLASH_ACR_DCEN                   /* ��������� ������ ���� ������ */
            | MCU_FLASH_ACR_LATENCY_3WS );         /* ������ ���������� ������, ��������� ��� ������� � flash-������ 
                                                      ( 4 CPU cycles) */
    OUT_REG( ptr_reg_flash->acr, temp );

    /* ���������� ������������� PLL ��� SYSCLK */
    temp  = IN_REG( ptr_reg_rcc->cfgr );
    temp &= ~ ( uint32_t )  ( MCU_RCC_CFGR_SW_MASK );
    temp |= MCU_RCC_CFGR_SW_1;
    OUT_REG( ptr_reg_rcc->cfgr, temp );

    while ( ( IN_REG( ptr_reg_rcc->cfgr ) & MCU_RCC_CFGR_SWS_MASK ) != MCU_RCC_CFGR_SWS_1 ) {
    /* �������� ������ PLL �� ������� ����� */
        ;
    }
}

/* ������������ ������������� ������ �����/������ GPIO */
static void init_gpio( void )
{
    stm32f407_gpio_t *ptr_reg_gpio;
    stm32f407_rcc_t  *ptr_reg_rcc;
    uint32_t         temp;

    ptr_reg_rcc = STM32F407_RCC_PTR;
    
    /* ������������� ������ A - E, H */
    temp  = IN_REG( ptr_reg_rcc->ahb1enr );
    temp |=
          MCU_RCC_AHB1ENR_GPIOAEN
        | MCU_RCC_AHB1ENR_GPIOBEN
        | MCU_RCC_AHB1ENR_GPIOCEN
        | MCU_RCC_AHB1ENR_GPIODEN
        | MCU_RCC_AHB1ENR_GPIOEEN
        | MCU_RCC_AHB1ENR_GPIOHEN;
    OUT_REG( ptr_reg_rcc->ahb1enr, temp );

    temp  = IN_REG( ptr_reg_rcc->apb2enr ); 
    temp |= 
          MCU_RCC_APB2ENR_USART1EN;
    OUT_REG( ptr_reg_rcc->apb2enr, temp );         /* ��������� ������������ ��� USART1 */

    /* ����� ������ GPIO */
    temp  = IN_REG( ptr_reg_rcc->ahb1rstr );
    temp |=
          MCU_RCC_AHB1RSTR_GPIOARST
        | MCU_RCC_AHB1RSTR_GPIOBRST
        | MCU_RCC_AHB1RSTR_GPIOCRST
        | MCU_RCC_AHB1RSTR_GPIODRST
        | MCU_RCC_AHB1RSTR_GPIOERST
        | MCU_RCC_AHB1RSTR_GPIOHRST;
    OUT_REG( ptr_reg_rcc->ahb1rstr, temp );        /* ��������� ������� � ��������� ����� */

    temp &= ~( uint32_t ) (
          MCU_RCC_AHB1RSTR_GPIOARST
        | MCU_RCC_AHB1RSTR_GPIOBRST
        | MCU_RCC_AHB1RSTR_GPIOCRST
        | MCU_RCC_AHB1RSTR_GPIODRST
        | MCU_RCC_AHB1RSTR_GPIOERST
        | MCU_RCC_AHB1RSTR_GPIOHRST);
    OUT_REG( ptr_reg_rcc->ahb1rstr, temp );        /* ����� ��������� ����� */

/* ��������� ����� A */
    
    /* ��������� �� ������ ������� ��������� ����� � */
    ptr_reg_gpio = STM32F407_GPIOA_PTR;
    
    /* ����� �������� �������� ������ */ 
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* ����� ������� ������ ������� ����� 
       0 - 4, 8, 11, 12 pin ��� �������, 
       4 - 7, 9, 10, 13, 14 pin ��� ��������������, 
       15 pin �������� */
    temp = IN_REG( ptr_reg_gpio->moder );
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS; 
    temp |=
          MCU_GPIO_MODER_0_IN                  /* INT_10KHz */   // PA0 ������ � ���������� ����������� ���������� 5545��2�3 (D65)
        | MCU_GPIO_MODER_1_IN                  /* ���_��� */     // PA1 ���� ������� ������� ������ ������ ���
        | MCU_GPIO_MODER_2_IN                  /* ���_��� */     // PA2 ���� ������ ������� ������� ������� �������
        | MCU_GPIO_MODER_3_IN                  /* ���_��� */     // PA3 ���� ������ ������� ������� ������� ���
        | MCU_GPIO_MODER_4_ALT                 /* SPI1_NSS */    // PA4 ����� ������� ������ ������������ ����������. ���������� ����������.
                                                                 // STM ��������� � �������� slave.
        | MCU_GPIO_MODER_5_ALT                 /* SPI1_SCK */    // PA5 ������ ������������ ���������� �������� SPI1
        | MCU_GPIO_MODER_6_ALT                 /* SPI1_MISO */   // PA6 �������� ������ SPI1
        | MCU_GPIO_MODER_7_ALT                 /* SPI1_MOSI */   // PA7 ������� ������ SPI1
        | MCU_GPIO_MODER_8_IN                  /* �� ������������ */
        | MCU_GPIO_MODER_9_ALT                 /* USART1_TX */   // PA9 UART1
        | MCU_GPIO_MODER_10_ALT                /* USART1_RX */   // PA10 UART1
        | MCU_GPIO_MODER_11_IN                 /* �� ������������ */
        | MCU_GPIO_MODER_12_IN                 /* �� ������������ */
        | MCU_GPIO_MODER_13_ALT                /* ����/����� SWD */
        | MCU_GPIO_MODER_14_ALT                /* ������������ SWD */
        | MCU_GPIO_MODER_15_OUT;               /* SPI3_NSS */    // PA15 ��������� ������������ ���������� ������ ������������ ����������.
                                                                 // STM ��������� � �������� master.
    OUT_REG( ptr_reg_gpio->moder, temp ); 
    
    /* ����� ����� �������� ������� �����, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );   
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp );		/* ��� �������� ���� ��������� ��� push-pull 
                                                   ( ��������������� ���� ������������) */
    
    /* ��������� �������� ������ ������� */
    temp = IN_REG( ptr_reg_gpio->ospeedr );
    temp &= ~( uint32_t ) MCU_GPIO_OSPEEDR_BITS;
    temp |=
          MCU_GPIO_OSPEEDR_0_MAX   
        | MCU_GPIO_OSPEEDR_1_MAX  
        | MCU_GPIO_OSPEEDR_2_MAX
        | MCU_GPIO_OSPEEDR_3_MAX
        | MCU_GPIO_OSPEEDR_4_MAX
        | MCU_GPIO_OSPEEDR_5_MAX
        | MCU_GPIO_OSPEEDR_6_MAX
        | MCU_GPIO_OSPEEDR_7_MAX
        | MCU_GPIO_OSPEEDR_9_MAX
        | MCU_GPIO_OSPEEDR_10_MAX
        | MCU_GPIO_OSPEEDR_13_MAX
        | MCU_GPIO_OSPEEDR_14_MAX
        | MCU_GPIO_OSPEEDR_15_MAX;
    OUT_REG( ptr_reg_gpio->ospeedr, temp );
    
    /* ��������� ������������� ���������� 
       9, 10, 13 �������� � �������; 
       8, 11, 12, 14 �������� � �����; 
       ��������� ������ ��� �������� */    
    temp = ~( uint32_t ) MCU_GPIO_PUPDR_BITS;
    temp |=
          MCU_GPIO_PUPDR_4_UP                   /* SPI1_NSS */
        | MCU_GPIO_PUPDR_8_DOWN
        | MCU_GPIO_PUPDR_9_UP                   /* �������� 9 � 10 ������� ��� USART1 */
        | MCU_GPIO_PUPDR_10_UP
        | MCU_GPIO_PUPDR_11_DOWN
        | MCU_GPIO_PUPDR_12_DOWN
        | MCU_GPIO_PUPDR_13_UP
        | MCU_GPIO_PUPDR_14_DOWN
        | MCU_GPIO_PUPDR_15_UP;                 /* SPI3_NSS */
    OUT_REG( ptr_reg_gpio->pupdr, temp );
    
    /* ��������� �������� ������ �� ������ SPI3_NSS */
	OUT_REG( STM32F407_GPIOA_PTR->bsrr, MCU_GPIO_BSRR_SET_15 );
        
    /* ��������� �������������� ������� ����� spi1, usart1, swd */
    /* ������ �������� ������� ����� */
    temp = IN_REG( ptr_reg_gpio->afrl );
    temp &= ~( uint32_t ) MCU_GPIO_AFRL_BITS;
    temp |=
          MCU_GPIO_AFRL_4_AF5            /* SPI1_NSS */
        | MCU_GPIO_AFRL_5_AF5            /* SPI1_SCK */
        | MCU_GPIO_AFRL_6_AF5            /* SPI1_MISO */
        | MCU_GPIO_AFRL_7_AF5;           /* SPI1_MOSI */
    OUT_REG( ptr_reg_gpio->afrl, temp );
    /* ������ �������� ������� ����� */
    temp = IN_REG( ptr_reg_gpio->afrh );
    temp &= ~( uint32_t ) MCU_GPIO_AFRH_BITS;
    temp |=
          MCU_GPIO_AFRH_9_AF7            /* USART1_TX */ 
        | MCU_GPIO_AFRH_10_AF7;          /* USART1_RX */
    OUT_REG( ptr_reg_gpio->afrh, temp );

/* ��������� ����� B */
    
    /* ��������� �� ������ ������� ��������� ����� B */
    ptr_reg_gpio = STM32F407_GPIOB_PTR;
    
    /* ����� �������� �������� ������ */ 
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* ����� ������� ������ ������� ����� 
       0, 2, 8 - 10 pin ��� �������, 
       3, 13 - 15 pin ��� ��������������, 
       1, 4-7, 11, 12  pin �������� */
    temp = IN_REG( ptr_reg_gpio->moder );
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS;
    temp |=
          MCU_GPIO_MODER_0_IN                   /* �� ������������ */
        | MCU_GPIO_MODER_1_OUT                  /* ���� ������ ��������� �������������� ������� ZeroCross, ������������ �� ���� */
        | MCU_GPIO_MODER_2_IN                   /* BOOT1 */      // ��������� � ���� GND --> ����� ������� �������� �� BOOT2 
        | MCU_GPIO_MODER_3_ALT                  /* ������������ ������� SWD - TRACESWO */
        | MCU_GPIO_MODER_4_OUT                  /* ����1 */      // �������� ��������������� ��
        | MCU_GPIO_MODER_5_OUT                  /* ����2 */
        | MCU_GPIO_MODER_6_OUT                  /* ����3 */
        | MCU_GPIO_MODER_7_OUT                  /* ����4 */ 
        | MCU_GPIO_MODER_8_IN                   /* EOC_ADC3 */   // �������������� �������� �� ��� D61
        | MCU_GPIO_MODER_9_IN                   /* EOC_ADC4 */   // �������������� �������� �� ��� D64
        | MCU_GPIO_MODER_10_IN                  /* ADC_CONV */   // ������ ��������������. �������� ��� ���� ��������� 
                                                                 // � �������� ������ ���������� ��� ������������� �������� 
                                                                 // ������ ��������������. ��� ����� ��������� ����.
        | MCU_GPIO_MODER_11_OUT                 /* RFS_ADC4 */   // ������ ������ ��� D64
        | MCU_GPIO_MODER_12_OUT                 /* RFS_ADC3 */   // ������ ������ ��� D61
        | MCU_GPIO_MODER_13_ALT                 /* SPI2_SCK */   // ������ ������������ SPI2
        | MCU_GPIO_MODER_14_ALT                 /* SPI2_MISO */  // ����� ���������������� ������ �� ���
        | MCU_GPIO_MODER_15_ALT;                /* SPI2_MOSI */  // ������ ������ �� �������. �� ���������.
    OUT_REG( ptr_reg_gpio->moder, temp );    
    
    /* ����� ����� �������� ������� ����� 
       6, 7 pin open drain
       ��� ��������� push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    temp |= 
          MCU_GPIO_OTYPER_OT6
        | MCU_GPIO_OTYPER_OT7;
    OUT_REG( ptr_reg_gpio->otyper, temp );
    
    /* ��������� �������� ������ ������� */
    temp = IN_REG( ptr_reg_gpio->ospeedr );
    temp &= ~( uint32_t ) MCU_GPIO_OSPEEDR_BITS;
    temp |=
          MCU_GPIO_OSPEEDR_0_SLOW
        | MCU_GPIO_OSPEEDR_1_MAX
        | MCU_GPIO_OSPEEDR_2_SLOW
        | MCU_GPIO_OSPEEDR_3_MAX
        | MCU_GPIO_OSPEEDR_4_MAX 
        | MCU_GPIO_OSPEEDR_5_MAX
        | MCU_GPIO_OSPEEDR_6_MAX
        | MCU_GPIO_OSPEEDR_7_MAX
        | MCU_GPIO_OSPEEDR_8_MAX
        | MCU_GPIO_OSPEEDR_9_MAX
        | MCU_GPIO_OSPEEDR_10_MAX
        | MCU_GPIO_OSPEEDR_11_MAX
        | MCU_GPIO_OSPEEDR_12_MAX
        | MCU_GPIO_OSPEEDR_13_MAX
        | MCU_GPIO_OSPEEDR_14_MAX
        | MCU_GPIO_OSPEEDR_15_SLOW;
    OUT_REG( ptr_reg_gpio->ospeedr, temp );
    
    /* ��������� ������������� ���������� 
       11, 12 �������� � �������; 
       0, 15 �������� � �����; 
       ��������� ������ ��� �������� */ 
    temp = IN_REG( ptr_reg_gpio->pupdr );
    temp &= ~( uint32_t ) MCU_GPIO_PUPDR_BITS;
    temp |=
          MCU_GPIO_PUPDR_0_DOWN
        | MCU_GPIO_PUPDR_11_UP
        | MCU_GPIO_PUPDR_12_UP
        | MCU_GPIO_PUPDR_15_DOWN;    
    OUT_REG( ptr_reg_gpio->pupdr, temp );
    
    /* ��������� ����� � ������ ��������� (������ STM � �������� ��������� ������) */
    OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_RES_1 );
    
    /* ��������� � ������� ��������� �������� ������ ��� D61, D64 */
	OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_SET_11 );
	OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_SET_12 );
    
    /* ��������� �������������� ������� ����� swd, spi2 */
    temp = IN_REG( ptr_reg_gpio->afrh );        
    temp &= ~( uint32_t ) MCU_GPIO_AFRH_BITS;
    temp |=
          /* ����� 3 ������������ �� ������ ��� SWD - TRACESWO */
          MCU_GPIO_AFRH_13_AF5          /* SPI2_SCK */
        | MCU_GPIO_AFRH_14_AF5          /* SPI2_MISO */
        | MCU_GPIO_AFRH_15_AF5;         /* SPI2_MOSI */
    OUT_REG( ptr_reg_gpio->afrh, temp );

/* ��������� ����� C */
    
    /* ��������� �� ������ ������� ��������� ����� C */
    ptr_reg_gpio = STM32F407_GPIOC_PTR;
    
    /* ����� �������� �������� ������ */
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* ����� ������� ������ ������� ����� 
       8, 9, 13 - 15 pin ��� �������, 
       10 - 12 pin ��� ��������������, 
       0 - 7 pin �������� */
    temp = IN_REG( ptr_reg_gpio->moder );          
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS;
    temp |=
          MCU_GPIO_MODER_0_OUT                  /* ���1 */ // ������� ���������� ��������� �������� ��������� ���1 - ���8 
        | MCU_GPIO_MODER_1_OUT                  /* ���2 */
        | MCU_GPIO_MODER_2_OUT                  /* ���3 */
        | MCU_GPIO_MODER_3_OUT                  /* ���4 */
        | MCU_GPIO_MODER_4_OUT                  /* ���5 */
        | MCU_GPIO_MODER_5_OUT                  /* ���6 */
        | MCU_GPIO_MODER_6_OUT                  /* ���7 */
        | MCU_GPIO_MODER_7_OUT                  /* ���8 */
        | MCU_GPIO_MODER_8_IN                   /* �� ������������ */
        | MCU_GPIO_MODER_9_IN                   /* �� ������������ */
        | MCU_GPIO_MODER_10_ALT                 /* SPI3_SCK */  // ������ ������������ SPI3
        | MCU_GPIO_MODER_11_ALT                 /* SPI3_MISO */ // ����� ���������������� ������ �� ������������ D60, �63
        | MCU_GPIO_MODER_12_ALT                 /* SPI3_MOSI */ // �������� ������ �� ����������� D60, �63
        | MCU_GPIO_MODER_13_IN                  /* �� ������������ */
        | MCU_GPIO_MODER_14_IN                  /* �� ������������ */
        | MCU_GPIO_MODER_15_IN;                 /* �� ������������ */
    OUT_REG( ptr_reg_gpio->moder, temp );
    
    /* ����� ����� �������� ������� �����, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp );
    
    /* ��������� �������� ������ ������� */
    temp = IN_REG( ptr_reg_gpio->ospeedr );
    temp &= ~( uint32_t ) MCU_GPIO_OSPEEDR_BITS;
    temp |=
          MCU_GPIO_OSPEEDR_0_MAX
        | MCU_GPIO_OSPEEDR_1_MAX
        | MCU_GPIO_OSPEEDR_2_MAX
        | MCU_GPIO_OSPEEDR_3_MAX
        | MCU_GPIO_OSPEEDR_4_MAX
        | MCU_GPIO_OSPEEDR_5_MAX
        | MCU_GPIO_OSPEEDR_6_MAX
        | MCU_GPIO_OSPEEDR_7_MAX
        | MCU_GPIO_OSPEEDR_10_MAX
        | MCU_GPIO_OSPEEDR_11_MAX
        | MCU_GPIO_OSPEEDR_12_MAX;
    OUT_REG( ptr_reg_gpio->ospeedr, temp );
    
    /* ��������� ������������� ����������  
       8, 9, 13-15 �������� � �����; 
       ��������� ������ ��� �������� */
    temp = IN_REG( ptr_reg_gpio->pupdr );    
    temp &= ~( uint32_t ) MCU_GPIO_PUPDR_BITS;
    temp |=
          MCU_GPIO_PUPDR_8_DOWN
        | MCU_GPIO_PUPDR_9_DOWN
        | MCU_GPIO_PUPDR_13_DOWN
        | MCU_GPIO_PUPDR_14_DOWN
        | MCU_GPIO_PUPDR_15_DOWN;
    OUT_REG( ptr_reg_gpio->pupdr, temp );
    
    /* ��������� �������������� ������� ����� spi3 */
    temp = IN_REG( ptr_reg_gpio->afrh );
    temp &= ~( uint32_t ) MCU_GPIO_AFRH_BITS;
    temp |=
          MCU_GPIO_AFRH_10_AF6              /* SPI3_SCK */  
        | MCU_GPIO_AFRH_11_AF6              /* SPI3_MISO */
        | MCU_GPIO_AFRH_12_AF6;             /* SPI3_MOSI */
    OUT_REG( ptr_reg_gpio->afrh, temp );

/* ��������� ����� D */
    
    /* ��������� �� ������ ������� ��������� ����� D */
    ptr_reg_gpio = STM32F407_GPIOD_PTR;
    
    /* ����� �������� �������� ������ */
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* ����� ������� ������ ������� ����� 
       0-8, 10, 11, 13 pin ��� �������, 
       12 pin ��� ��������������, 
       9, 14, 15 pin �������� */
    temp = IN_REG( ptr_reg_gpio->moder );
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS;
    temp |=
          MCU_GPIO_MODER_0_IN                  /* ���1 */          // ������� ������� ������ ���1 - ���8 
        | MCU_GPIO_MODER_1_IN                  /* ���2 */
        | MCU_GPIO_MODER_2_IN                  /* ���3 */
        | MCU_GPIO_MODER_3_IN                  /* ���4 */
        | MCU_GPIO_MODER_4_IN                  /* ���5 */
        | MCU_GPIO_MODER_5_IN                  /* ���6 */
        | MCU_GPIO_MODER_6_IN                  /* ���7 */
        | MCU_GPIO_MODER_7_IN                  /* ���8 */
        | MCU_GPIO_MODER_8_IN                  /* �� ������������ */
        | MCU_GPIO_MODER_9_OUT                 /* HL2 */           // ���������� ������� ����������� HL2 */
        | MCU_GPIO_MODER_10_IN                 /* ZeroCross1 */    // ������ � ��������� ��������������� ������� D15
        | MCU_GPIO_MODER_11_IN                 /* ZeroCross2 */    // ������ � ��������� ��������������� ������� D65
        | MCU_GPIO_MODER_12_ALT                /* Meandr10kHz_2 */ // �������� ������ 10 ��� �� �������� ������ D65 */
        | MCU_GPIO_MODER_13_IN                 /* ��� �.�.��� */   // ���������� ������ ��� ��������
        | MCU_GPIO_MODER_14_OUT                /* TR_RST */        // ����� ������� ��������� D-��������� D66, D67 */
        | MCU_GPIO_MODER_15_OUT;               /* I_TCLK */        // ������ ������������ ��������� D-��������� D66, D67 */       
    OUT_REG( ptr_reg_gpio->moder, temp );
    
    /* ����� ����� �������� ������� �����, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp );
    
    /* ��������� �������� ������ ������� */
    temp = IN_REG( ptr_reg_gpio->ospeedr );
    temp &= ~( uint32_t ) MCU_GPIO_OSPEEDR_BITS;
    temp |=
          MCU_GPIO_OSPEEDR_0_MAX
        | MCU_GPIO_OSPEEDR_1_MAX
        | MCU_GPIO_OSPEEDR_2_MAX
        | MCU_GPIO_OSPEEDR_3_MAX
        | MCU_GPIO_OSPEEDR_4_MAX
        | MCU_GPIO_OSPEEDR_5_MAX
        | MCU_GPIO_OSPEEDR_6_MAX
        | MCU_GPIO_OSPEEDR_7_MAX
        | MCU_GPIO_OSPEEDR_9_MAX
        | MCU_GPIO_OSPEEDR_10_MAX
        | MCU_GPIO_OSPEEDR_11_MAX
        | MCU_GPIO_OSPEEDR_12_MAX
        | MCU_GPIO_OSPEEDR_13_MAX
        | MCU_GPIO_OSPEEDR_14_MAX
        | MCU_GPIO_OSPEEDR_15_MAX;
    OUT_REG( ptr_reg_gpio->ospeedr, temp );
    
    /* ��������� ������������� ���������� 
       14 �������� � �������; 
       0 - 5, 8, 9 �������� � �����; 
       ��������� ������ ��� �������� */
    temp = IN_REG( ptr_reg_gpio->pupdr );    
    temp &= ~( uint32_t ) MCU_GPIO_PUPDR_BITS;
    temp |=
          MCU_GPIO_PUPDR_0_DOWN
        | MCU_GPIO_PUPDR_1_DOWN
        | MCU_GPIO_PUPDR_2_DOWN
        | MCU_GPIO_PUPDR_3_DOWN
        | MCU_GPIO_PUPDR_4_DOWN
        | MCU_GPIO_PUPDR_5_DOWN 
        | MCU_GPIO_PUPDR_8_DOWN
        | MCU_GPIO_PUPDR_9_DOWN
        | MCU_GPIO_PUPDR_14_UP;
    OUT_REG( ptr_reg_gpio->pupdr, temp );
    
    /* ��������� �������������� ������� ����� TIM4_CH1 */
    temp = IN_REG( ptr_reg_gpio->afrh );
    temp &= ~( uint32_t ) MCU_GPIO_AFRH_BITS;
    temp |= MCU_GPIO_AFRH_12_AF2;
    OUT_REG( ptr_reg_gpio->afrh, temp );

    /* ������������� D-��������� ���������� ��� */
    /* ��������� ������� ������������ D-��������� � ��������� ��������� */
    OUT_REG( STM32F407_GPIOD_PTR->bsrr, MCU_GPIO_BSRR_RES_15 );
    
    /* ������ D-��������� */
    OUT_REG( STM32F407_GPIOD_PTR->bsrr, MCU_GPIO_BSRR_SET_14 );
    delay_mcs( 3U );
    OUT_REG( STM32F407_GPIOD_PTR->bsrr, MCU_GPIO_BSRR_RES_14 );
    delay_mcs( 3U );
    OUT_REG( STM32F407_GPIOD_PTR->bsrr, MCU_GPIO_BSRR_SET_14 );
    
    /* ������ ��� ��� ����������� ��������������� ������� */
    timer_sinus_meandr_init(TIME_STEP_100_MCS);

/* ��������� ����� E */

    /* ��������� �� ������ ������� ��������� ����� E */
    ptr_reg_gpio = STM32F407_GPIOE_PTR;
    
    /* ����� �������� �������� ������ */
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* ����� ������� ������ ������� �����, ��� pin ��� ������� */
    temp = IN_REG( ptr_reg_gpio->moder );
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS;
    temp |=
          MCU_GPIO_MODER_0_IN                  /* I_DDT1 */ // ���������� ������ �� �������� ����
        | MCU_GPIO_MODER_1_IN                  /* I_DDT2 */
        | MCU_GPIO_MODER_2_IN                  /* I_DDT3 */
        | MCU_GPIO_MODER_3_IN                  /* I_DDT4 */
        | MCU_GPIO_MODER_4_IN                  /* I_DDT5 */
        | MCU_GPIO_MODER_5_IN                  /* I_DDT6 */
        | MCU_GPIO_MODER_6_IN                  /* I_DDT7 */
        | MCU_GPIO_MODER_7_IN                  /* I_DDT8 */
        | MCU_GPIO_MODER_8_IN                  /* ���1 */  // ���-�������� ���1 - ���8
        | MCU_GPIO_MODER_9_IN                  /* ���2 */
        | MCU_GPIO_MODER_10_IN                 /* ���3 */
        | MCU_GPIO_MODER_11_IN                 /* ���4 */
        | MCU_GPIO_MODER_12_IN                 /* ���5 */
        | MCU_GPIO_MODER_13_IN                 /* ���6 */
        | MCU_GPIO_MODER_14_IN                 /* ���7 */
        | MCU_GPIO_MODER_15_IN;                /* ���8 */
    OUT_REG( ptr_reg_gpio->moder, temp );
    
    /* ����� ����� �������� ������� �����, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp );
    
    /* ��������� �������� ������ ������� */
    temp = IN_REG( ptr_reg_gpio->ospeedr );
    temp &= ~( uint32_t ) MCU_GPIO_OSPEEDR_BITS;
    temp |=
          MCU_GPIO_OSPEEDR_0_MAX
        | MCU_GPIO_OSPEEDR_1_MAX
        | MCU_GPIO_OSPEEDR_2_MAX
        | MCU_GPIO_OSPEEDR_3_MAX
        | MCU_GPIO_OSPEEDR_4_MAX
        | MCU_GPIO_OSPEEDR_5_MAX
        | MCU_GPIO_OSPEEDR_6_MAX
        | MCU_GPIO_OSPEEDR_7_MAX
        | MCU_GPIO_OSPEEDR_8_MAX
        | MCU_GPIO_OSPEEDR_9_MAX
        | MCU_GPIO_OSPEEDR_10_MAX
        | MCU_GPIO_OSPEEDR_11_MAX
        | MCU_GPIO_OSPEEDR_12_MAX
        | MCU_GPIO_OSPEEDR_13_MAX
        | MCU_GPIO_OSPEEDR_14_MAX
        | MCU_GPIO_OSPEEDR_15_MAX;
    OUT_REG( ptr_reg_gpio->ospeedr, temp );
    
    /* ��������� ������������� ����������  
       ��� ������ ��� �������� */
    temp = IN_REG( ptr_reg_gpio->pupdr );    
    temp &= ~( uint32_t ) MCU_GPIO_PUPDR_BITS;
    OUT_REG( ptr_reg_gpio->pupdr, temp );

/* ��������� ����� H */

    /* ��������� �� ������ ������� ��������� ����� H */
    ptr_reg_gpio = STM32F407_GPIOH_PTR;
    
    /* ����� �������� �������� ������ */
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* ����� ������� ������ ������� �����, ��� pin ��� ������� */
    OUT_REG( ptr_reg_gpio->moder, 0x0U );
    
    /* ����� ����� �������� ������� �����, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp ); 
    
    /* ��������� �������� ������ ������� */
    temp = IN_REG( ptr_reg_gpio->ospeedr ); 
    temp &= ~( uint32_t ) MCU_GPIO_OSPEEDR_BITS;
    temp |= MCU_GPIO_OSPEEDR_0_MAX;
    OUT_REG( ptr_reg_gpio->ospeedr, temp );
    
    /* ��������� ������������� ����������  
       ��� ������ ��� �������� */
    OUT_REG( ptr_reg_gpio->pupdr, 0x0U );

}
/*  ������������ ������������� ������ �� Reset */
static void un_init_clk( void )
{
    uint32_t        temp;

    stm32f407_rcc_t *ptr_reg_rcc;
    stm32f407_pwr_t *ptr_reg_pwr;
    stm32f407_flash_t *ptr_reg_flash;

    ptr_reg_rcc = STM32F407_RCC_PTR;
    ptr_reg_pwr = STM32F407_PWR_PTR;
    ptr_reg_flash = STM32F407_FLASH_PTR;
      
    /* �������� HSI */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp |= MCU_RCC_CR_HSION;
    OUT_REG( ptr_reg_rcc->cr, temp );

    while ( 0U == ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_HSIRDY ) ) {
        /* �������� ������ HSI �� ������� ����� */
        ;
    }  
  
    /* ����� cfgr �������� */
    OUT_REG( ptr_reg_rcc->cfgr, 0x0U );

    while ( ( IN_REG( ptr_reg_rcc->cfgr ) & MCU_RCC_CFGR_SWS_MASK ) != 0x0U ) {
    /* �������� ��������� HSI � �������� system clock source */
        ;
    }

    /* ���������� CSS � HSE */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp &= ~ ( uint32_t )  ( MCU_RCC_CR_CSSON 
                            | MCU_RCC_CR_HSEON );
    OUT_REG( ptr_reg_rcc->cr, temp );

    while ( ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_HSERDY ) != 0x0U ) {
    /* �������� ���������� HSE */
        ;
    }

    /* ����� HSEBYP */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp &= ~ ( uint32_t ) MCU_RCC_CR_HSEBYP;
    OUT_REG( ptr_reg_rcc->cr, temp );

    /* ����� cr �������� */
    OUT_REG( ptr_reg_pwr->cr, 0x00004000U );
    
    /* ����� cr �������� */
    OUT_REG( ptr_reg_rcc->cr, 0x00000083U );

    /* ����� pllcfgr �������� */
    OUT_REG( ptr_reg_rcc->pllcfgr, 0x24003010U ); 
    
    /* ����� cir �������� */
    OUT_REG( ptr_reg_rcc->cir, 0x0U );

    /* ����� apb1enr �������� */
    OUT_REG( ptr_reg_rcc->apb1enr, 0x0U );

    /* ����� acr �������� */
    OUT_REG( ptr_reg_flash->acr, 0x0U );

}
/* ������������ ������������� PORTx �� Reset (������ ���������� �������� �/��) */
static void un_init_gpio( void )
{
    stm32f407_gpio_t *ptr_reg_gpio;

    /* ��������� ����� A */
    ptr_reg_gpio = STM32F407_GPIOA_PTR;
    OUT_REG( ptr_reg_gpio->moder,   0xA8000000U );
    OUT_REG( ptr_reg_gpio->otyper,  0x0000U );
    OUT_REG( ptr_reg_gpio->ospeedr, 0x0C000000U );
    OUT_REG( ptr_reg_gpio->pupdr,   0x64000000U );
    OUT_REG( ptr_reg_gpio->odr,     0x0000U );
    OUT_REG( ptr_reg_gpio->bsrr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->lckr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrl,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrh,    0x00000000U );

    /* ��������� ����� B */
    ptr_reg_gpio = STM32F407_GPIOB_PTR;
    OUT_REG( ptr_reg_gpio->moder,   0x00000280U );
    OUT_REG( ptr_reg_gpio->otyper,  0x0000U );
    OUT_REG( ptr_reg_gpio->ospeedr, 0x000000C0U );
    OUT_REG( ptr_reg_gpio->pupdr,   0x00000100U );
    OUT_REG( ptr_reg_gpio->odr,     0x0000U );
    OUT_REG( ptr_reg_gpio->bsrr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->lckr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrl,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrh,    0x00000000U );

    /* ��������� ����� C */
    ptr_reg_gpio = STM32F407_GPIOC_PTR;
    OUT_REG( ptr_reg_gpio->moder,   0x00000000U );
    OUT_REG( ptr_reg_gpio->otyper,  0x0000U );
    OUT_REG( ptr_reg_gpio->ospeedr, 0x00000000U );
    OUT_REG( ptr_reg_gpio->pupdr,   0x00000000U );
    OUT_REG( ptr_reg_gpio->odr,     0x0000U );
    OUT_REG( ptr_reg_gpio->bsrr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->lckr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrl,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrh,    0x00000000U );

    /* ��������� ����� D */
    ptr_reg_gpio = STM32F407_GPIOD_PTR;
    OUT_REG( ptr_reg_gpio->moder,   0x00000000U );
    OUT_REG( ptr_reg_gpio->otyper,  0x0000U );
    OUT_REG( ptr_reg_gpio->ospeedr, 0x00000000U );
    OUT_REG( ptr_reg_gpio->pupdr,   0x00000000U );
    OUT_REG( ptr_reg_gpio->odr,     0x0000U );
    OUT_REG( ptr_reg_gpio->bsrr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->lckr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrl,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrh,    0x00000000U );

    /* ��������� ����� E */
    ptr_reg_gpio = STM32F407_GPIOE_PTR;
    OUT_REG( ptr_reg_gpio->moder,   0x00000000U );
    OUT_REG( ptr_reg_gpio->otyper,  0x0000U );
    OUT_REG( ptr_reg_gpio->ospeedr, 0x00000000U );
    OUT_REG( ptr_reg_gpio->pupdr,   0x00000000U );
    OUT_REG( ptr_reg_gpio->odr,     0x0000U );
    OUT_REG( ptr_reg_gpio->bsrr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->lckr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrl,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrh,    0x00000000U );

    /* ��������� ����� F */
    ptr_reg_gpio = STM32F407_GPIOF_PTR;
    OUT_REG( ptr_reg_gpio->moder,   0x00000000U );
    OUT_REG( ptr_reg_gpio->otyper,  0x0000U );
    OUT_REG( ptr_reg_gpio->ospeedr, 0x00000000U );
    OUT_REG( ptr_reg_gpio->pupdr,   0x00000000U );
    OUT_REG( ptr_reg_gpio->odr,     0x0000U );
    OUT_REG( ptr_reg_gpio->bsrr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->lckr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrl,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrh,    0x00000000U );

    /* ��������� ����� G */
    ptr_reg_gpio = STM32F407_GPIOG_PTR;
    OUT_REG( ptr_reg_gpio->moder,   0x00000000U );
    OUT_REG( ptr_reg_gpio->otyper,  0x0000U );
    OUT_REG( ptr_reg_gpio->ospeedr, 0x00000000U );
    OUT_REG( ptr_reg_gpio->pupdr,   0x00000000U );
    OUT_REG( ptr_reg_gpio->odr,     0x0000U );
    OUT_REG( ptr_reg_gpio->bsrr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->lckr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrl,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrh,    0x00000000U );

    /* ��������� ����� H */
    ptr_reg_gpio = STM32F407_GPIOH_PTR;
    OUT_REG( ptr_reg_gpio->moder,   0x00000000U );
    OUT_REG( ptr_reg_gpio->otyper,  0x0000U );
    OUT_REG( ptr_reg_gpio->ospeedr, 0x00000000U );
    OUT_REG( ptr_reg_gpio->pupdr,   0x00000000U );
    OUT_REG( ptr_reg_gpio->odr,     0x0000U );
    OUT_REG( ptr_reg_gpio->bsrr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->lckr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrl,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrh,    0x00000000U );

    /* ��������� ����� I */
    ptr_reg_gpio = STM32F407_GPIOI_PTR;
    OUT_REG( ptr_reg_gpio->moder,   0x00000000U );
    OUT_REG( ptr_reg_gpio->otyper,  0x0000U );
    OUT_REG( ptr_reg_gpio->ospeedr, 0x00000000U );
    OUT_REG( ptr_reg_gpio->pupdr,   0x00000000U );
    OUT_REG( ptr_reg_gpio->odr,     0x0000U );
    OUT_REG( ptr_reg_gpio->bsrr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->lckr,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrl,    0x00000000U );
    OUT_REG( ptr_reg_gpio->afrh,    0x00000000U );
}
