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

/* Результат начального пуска
   NO_ERROR - нет ошибок */
LOADER_DATA error_t init_error;

#define MCU_RCC_PLLCFGR_PLLM_4     ( 0x00000004U )
#define MCU_RCC_PLLCFGR_PLLN_100   ( 0x00001900U )
#define MCU_RCC_PLLCFGR_PLLP_2     ( 0x00000000U )
#define MCU_RCC_PLLCFGR_PLLQ_4     ( 0x04000000U )
#define MCU_FLASH_ACR_LATENCY_3WS  ( 0x00000003U )

/* Программа восстановления начальных настроек аппаратуры */
void un_init_hardware( void )
{

    /* Восстановление настроек контроллера прерываний */
    un_init_irq();

    /* Восстановление настроек CLK */
    un_init_clk();

    /* Восстановление настроек GPIO */
    un_init_gpio();

}

/* Подпрограмма инициализации аппаратуры:
   1. Инициализация FPU;
   2. Настройка PLL;
   3. Настройка вводов/выводов микропроцессора;
   Примечания:
   1. Вызывается из модуля подпрограммы init_startup;
   2. Инициализация прочего оборудования производится при инициализации
      обслуживающих их модулей
 */
void init_hardware( void )
{

    /* Инициализация блока с плавающей точкой (FPU). 
       Установка 20-23 бит в Coprocessor Access Control Register, 
       разрешение полного доступа к сопроцессорам CP10 и CP11 */
    STM32F407_SCB_PTR->cpacr |= ((3UL << 10*2)|(3UL << 11*2));

    /* Настройка частот */
    init_clk();

    /* Настройка выводов процессора */
    init_gpio();  

    /* Начиная отсюда доступно обращения по реальным адресам */

    init_error = NO_ERROR;

}

/* Подпрограмма инициализации ПО */
void init_software( void )
{
    /* Инициализация контроллера прерываний */
    if ( NO_ERROR != init_irq() ) {
        /* Ошибка инициализации */
        init_error = CRITICAL_ERROR;
    }

    /* Начиная отсюда все прерывания замаскированы или выключены,
       но разрешены глобально в MSR */
}


/*  Подпрограмма инициализации частот 
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

    /* Указатели на начало областей регистров */
    ptr_reg_rcc = STM32F407_RCC_PTR;               /* Reset and clock control */
    ptr_reg_pwr = STM32F407_PWR_PTR;               /* Power controller */
    ptr_reg_flash = STM32F407_FLASH_PTR;           /* Flash interface */

    /* Включение HSI */
    temp  = IN_REG( ptr_reg_rcc->cr ); 
    temp |= MCU_RCC_CR_HSION;
    OUT_REG( ptr_reg_rcc->cr, temp );              /* Включение внутреннего высокочастотного генератора */

    while ( 0U == ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_HSIRDY ) ) {
        /* Ожидание выхода HSI на рабочий режим */
        ;
    } 
    
    /* Сброс cfgr регистра */
    OUT_REG( ptr_reg_rcc->cfgr, 0x0U );            /* Сброс осуществляется с целью установки в качестве источника тактирования 
                                                      системной шины внутренний генератор 16 МГц */

    /* Выключение HSEON, CSSON и PLLON */
    temp  = IN_REG( ptr_reg_rcc->cr ); 
    temp &= ~ ( uint32_t )  ( MCU_RCC_CR_PLLON     /* Отключение возможно, если PLL прямо или косвенно не тактирует системную шину */
                            | MCU_RCC_CR_CSSON     /* Clock security system */
                            | MCU_RCC_CR_HSEON );  /* Отключение возможно по аналогии с PLL */
    OUT_REG( ptr_reg_rcc->cr, temp );

    /* Сброс PLLCFGR */
    temp = 0x24003010U;                            /* Значение регистра по умолчанию (после reset) */
    OUT_REG( ptr_reg_rcc->pllcfgr, temp );         /* Сброс регистра осуществляется с целью сброса коэффициентов деления/умножения частот */

    /* Сброс HSEBYP */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp &= ~ ( uint32_t ) MCU_RCC_CR_HSEBYP;
    OUT_REG( ptr_reg_rcc->cr, temp );              /* Отключение возможности пропускать сигнал с внешнего генератора,
                                                      возможно только после отключения HSE */

    /* Выключение всех прерываний */
    OUT_REG( ptr_reg_rcc->cir, 0x0U );             /* Отключение прерываний выставления флагов о стабилизации частот */     
                               
    /* Включение HSE ( 8 МГц ) */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp |= MCU_RCC_CR_HSEON;
    OUT_REG( ptr_reg_rcc->cr, temp );

    while ( 0U == ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_HSERDY ) ) {
        /* Ожидание выхода HSE на рабочий режим */
        ;
    }

    /* Включение тактирования блока управления питанием */
    temp  = IN_REG( ptr_reg_rcc->apb1enr );
    temp |= MCU_RCC_APB1ENR_PWREN;
    OUT_REG( ptr_reg_rcc->apb1enr, temp );         /* Установка бита PWREN ( Power interface clock enable) */

    /* Выбор режима работы внутреннего регулятора напряжения */
    temp  = IN_REG( ptr_reg_pwr->cr );
    temp |= MCU_PWR_CR_VOS;                        /* Выбор режима "VOS = 1 (по умолчанию) */
    OUT_REG( ptr_reg_pwr->cr, temp );
    
    /* HCLK = SYSCLK / 1 */ 
    temp  = IN_REG( ptr_reg_rcc->cfgr );
    temp |= ( MCU_RCC_CFGR_PPRE1_0
            | MCU_RCC_CFGR_PPRE1_2                 /* Коэффициент деления APB1 = 4 */ 
            | MCU_RCC_CFGR_PPRE2_0
            | MCU_RCC_CFGR_PPRE2_2 );              /* Коэффициент деления APB2 = 4 */
    OUT_REG( ptr_reg_rcc->cfgr, temp );            /* HCLK = SYSCLK */
    
    /* Настойка  PLL_M, PLL_N, PLL_P, PLL_Q и RCC_PLLCFGR_PLLSRC_HSE */
    temp  = IN_REG( ptr_reg_rcc->pllcfgr );
    temp &= ~ ( uint32_t ) 
            ( MCU_RCC_PLLCFGR_PLLM_BITS 
            | MCU_RCC_PLLCFGR_PLLN_BITS 
            | MCU_RCC_PLLCFGR_PLLP_BITS
            | MCU_RCC_PLLCFGR_PLLQ_BITS );

    temp |= ( MCU_RCC_PLLCFGR_PLLM_4               /* Выставляем М деление на 4 */
            | MCU_RCC_PLLCFGR_PLLN_100             /* Выставляем N умножение на 100 */
            | MCU_RCC_PLLCFGR_PLLP_2               /* Выставляем P деление на 2 */
            | MCU_RCC_PLLCFGR_PLLQ_4               /* Выставляем Q деление на 4 */
            | MCU_RCC_PLLCFGR_PLLSRC );
    OUT_REG( ptr_reg_rcc->pllcfgr, temp );

    /* Включение PLL */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp |= MCU_RCC_CR_PLLON;
    OUT_REG( ptr_reg_rcc->cr, temp );

    while ( 0U == ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_PLLRDY ) ) {
    /* Ожидание выхода на рабочий режим PLL*/
        ;
    }

    /* Настойка  Flash prefetch, Instruction cache, Data cache и wait state */
    temp  = IN_REG( ptr_reg_flash->acr );   
    temp |= ( MCU_FLASH_ACR_PRFTEN                 /* Разрешаем работу буфера предварительной выборки инструкций */
            | MCU_FLASH_ACR_ICEN                   /* Разрешаем работу кэша инструкций */
            | MCU_FLASH_ACR_DCEN                   /* Разрешаем работу кэша данных */
            | MCU_FLASH_ACR_LATENCY_3WS );         /* Задаем количество тактов, требуемых для доступа к flash-памяти 
                                                      ( 4 CPU cycles) */
    OUT_REG( ptr_reg_flash->acr, temp );

    /* Разрешение использования PLL для SYSCLK */
    temp  = IN_REG( ptr_reg_rcc->cfgr );
    temp &= ~ ( uint32_t )  ( MCU_RCC_CFGR_SW_MASK );
    temp |= MCU_RCC_CFGR_SW_1;
    OUT_REG( ptr_reg_rcc->cfgr, temp );

    while ( ( IN_REG( ptr_reg_rcc->cfgr ) & MCU_RCC_CFGR_SWS_MASK ) != MCU_RCC_CFGR_SWS_1 ) {
    /* Ожидание выхода PLL на рабочий режим */
        ;
    }
}

/* Подпрограмма инициализации портов ввода/вывода GPIO */
static void init_gpio( void )
{
    stm32f407_gpio_t *ptr_reg_gpio;
    stm32f407_rcc_t  *ptr_reg_rcc;
    uint32_t         temp;

    ptr_reg_rcc = STM32F407_RCC_PTR;
    
    /* Инициализация портов A - E, H */
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
    OUT_REG( ptr_reg_rcc->apb2enr, temp );         /* Запускаем тактирование для USART1 */

    /* Сброс портов GPIO */
    temp  = IN_REG( ptr_reg_rcc->ahb1rstr );
    temp |=
          MCU_RCC_AHB1RSTR_GPIOARST
        | MCU_RCC_AHB1RSTR_GPIOBRST
        | MCU_RCC_AHB1RSTR_GPIOCRST
        | MCU_RCC_AHB1RSTR_GPIODRST
        | MCU_RCC_AHB1RSTR_GPIOERST
        | MCU_RCC_AHB1RSTR_GPIOHRST;
    OUT_REG( ptr_reg_rcc->ahb1rstr, temp );        /* Установка поротов в состояние ресет */

    temp &= ~( uint32_t ) (
          MCU_RCC_AHB1RSTR_GPIOARST
        | MCU_RCC_AHB1RSTR_GPIOBRST
        | MCU_RCC_AHB1RSTR_GPIOCRST
        | MCU_RCC_AHB1RSTR_GPIODRST
        | MCU_RCC_AHB1RSTR_GPIOERST
        | MCU_RCC_AHB1RSTR_GPIOHRST);
    OUT_REG( ptr_reg_rcc->ahb1rstr, temp );        /* Сброс состояния ресет */

/* Настройка порта A */
    
    /* Указатель на начало области регистров порта А */
    ptr_reg_gpio = STM32F407_GPIOA_PTR;
    
    /* Сброс регистра выходных данных */ 
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* Выбор режимов работы выводов порта 
       0 - 4, 8, 11, 12 pin как входные, 
       4 - 7, 9, 10, 13, 14 pin как альтернативные, 
       15 pin выходной */
    temp = IN_REG( ptr_reg_gpio->moder );
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS; 
    temp |=
          MCU_GPIO_MODER_0_IN                  /* INT_10KHz */   // PA0 Сигнал с микросхемы компаратора напряжения 5545СА2У3 (D65)
        | MCU_GPIO_MODER_1_IN                  /* РКП_КТА */     // PA1 Вход разовой команды выбора режима КТА
        | MCU_GPIO_MODER_2_IN                  /* РКП_Мон */     // PA2 Вход приема входной разовой команды Монитор
        | MCU_GPIO_MODER_3_IN                  /* РКП_СНП */     // PA3 Вход приема входной разовой команды СНП
        | MCU_GPIO_MODER_4_ALT                 /* SPI1_NSS */    // PA4 Прием сигнала выбора подчиненного устройства. Аппаратное управление.
                                                                 // STM выступает в качестве slave.
        | MCU_GPIO_MODER_5_ALT                 /* SPI1_SCK */    // PA5 сигнал тактирования выдаваемый мастером SPI1
        | MCU_GPIO_MODER_6_ALT                 /* SPI1_MISO */   // PA6 Выходные данные SPI1
        | MCU_GPIO_MODER_7_ALT                 /* SPI1_MOSI */   // PA7 Входные данные SPI1
        | MCU_GPIO_MODER_8_IN                  /* Не используется */
        | MCU_GPIO_MODER_9_ALT                 /* USART1_TX */   // PA9 UART1
        | MCU_GPIO_MODER_10_ALT                /* USART1_RX */   // PA10 UART1
        | MCU_GPIO_MODER_11_IN                 /* Не используется */
        | MCU_GPIO_MODER_12_IN                 /* Не используется */
        | MCU_GPIO_MODER_13_ALT                /* Вход/выход SWD */
        | MCU_GPIO_MODER_14_ALT                /* Тактирование SWD */
        | MCU_GPIO_MODER_15_OUT;               /* SPI3_NSS */    // PA15 Настройка программного управления выбора подчиненного устройства.
                                                                 // STM выступает в качестве master.
    OUT_REG( ptr_reg_gpio->moder, temp ); 
    
    /* Выбор типов выходных выводов порта, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );   
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp );		/* Все выходные пины настроены как push-pull 
                                                   ( комплементарная пара транзисторов) */
    
    /* Настройка скорости работы выводов */
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
    
    /* Настройка подтягивающих резисторов 
       9, 10, 13 подтяжка к питанию; 
       8, 11, 12, 14 подтяжка к земле; 
       остальные выводы без подтяжки */    
    temp = ~( uint32_t ) MCU_GPIO_PUPDR_BITS;
    temp |=
          MCU_GPIO_PUPDR_4_UP                   /* SPI1_NSS */
        | MCU_GPIO_PUPDR_8_DOWN
        | MCU_GPIO_PUPDR_9_UP                   /* подтяжка 9 и 10 выводов для USART1 */
        | MCU_GPIO_PUPDR_10_UP
        | MCU_GPIO_PUPDR_11_DOWN
        | MCU_GPIO_PUPDR_12_DOWN
        | MCU_GPIO_PUPDR_13_UP
        | MCU_GPIO_PUPDR_14_DOWN
        | MCU_GPIO_PUPDR_15_UP;                 /* SPI3_NSS */
    OUT_REG( ptr_reg_gpio->pupdr, temp );
    
    /* Установка высокого уровня на выходе SPI3_NSS */
	OUT_REG( STM32F407_GPIOA_PTR->bsrr, MCU_GPIO_BSRR_SET_15 );
        
    /* Настройка альтернативных функций порта spi1, usart1, swd */
    /* Первая половина выводов порта */
    temp = IN_REG( ptr_reg_gpio->afrl );
    temp &= ~( uint32_t ) MCU_GPIO_AFRL_BITS;
    temp |=
          MCU_GPIO_AFRL_4_AF5            /* SPI1_NSS */
        | MCU_GPIO_AFRL_5_AF5            /* SPI1_SCK */
        | MCU_GPIO_AFRL_6_AF5            /* SPI1_MISO */
        | MCU_GPIO_AFRL_7_AF5;           /* SPI1_MOSI */
    OUT_REG( ptr_reg_gpio->afrl, temp );
    /* Вторая половина выводов порта */
    temp = IN_REG( ptr_reg_gpio->afrh );
    temp &= ~( uint32_t ) MCU_GPIO_AFRH_BITS;
    temp |=
          MCU_GPIO_AFRH_9_AF7            /* USART1_TX */ 
        | MCU_GPIO_AFRH_10_AF7;          /* USART1_RX */
    OUT_REG( ptr_reg_gpio->afrh, temp );

/* Настройка порта B */
    
    /* Указатель на начало области регистров порта B */
    ptr_reg_gpio = STM32F407_GPIOB_PTR;
    
    /* Сброс регистра выходных данных */ 
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* Выбор режимов работы выводов порта 
       0, 2, 8 - 10 pin как входные, 
       3, 13 - 15 pin как альтернативные, 
       1, 4-7, 11, 12  pin выходные */
    temp = IN_REG( ptr_reg_gpio->moder );
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS;
    temp |=
          MCU_GPIO_MODER_0_IN                   /* Не используется */
        | MCU_GPIO_MODER_1_OUT                  /* Флаг выбора источника периодического сигнала ZeroCross, поступающего на ПЛИС */
        | MCU_GPIO_MODER_2_IN                   /* BOOT1 */      // Подключен к цепи GND --> Режим запуска задается на BOOT2 
        | MCU_GPIO_MODER_3_ALT                  /* Опциональная функция SWD - TRACESWO */
        | MCU_GPIO_MODER_4_OUT                  /* ТРКВ1 */      // Выходные технологические РК
        | MCU_GPIO_MODER_5_OUT                  /* ТРКВ2 */
        | MCU_GPIO_MODER_6_OUT                  /* ТРКВ3 */
        | MCU_GPIO_MODER_7_OUT                  /* ТРКВ4 */ 
        | MCU_GPIO_MODER_8_IN                   /* EOC_ADC3 */   // Преобразование окончено от АЦП D61
        | MCU_GPIO_MODER_9_IN                   /* EOC_ADC4 */   // Преобразование окончено от АЦП D64
        | MCU_GPIO_MODER_10_IN                  /* ADC_CONV */   // Начать преобразование. Настроен как вход поскольку 
                                                                 // в процессе работы загрузчика нет необходимости выдавать 
                                                                 // сигнал самостоятельно. Это будет выполнять ПЛИС.
        | MCU_GPIO_MODER_11_OUT                 /* RFS_ADC4 */   // Сигнал выбора АЦП D64
        | MCU_GPIO_MODER_12_OUT                 /* RFS_ADC3 */   // Сигнал выбора АЦП D61
        | MCU_GPIO_MODER_13_ALT                 /* SPI2_SCK */   // Сигнал тактирования SPI2
        | MCU_GPIO_MODER_14_ALT                 /* SPI2_MISO */  // Прием последовательных данных от АЦП
        | MCU_GPIO_MODER_15_ALT;                /* SPI2_MOSI */  // Выдача данных от мастера. Не подключен.
    OUT_REG( ptr_reg_gpio->moder, temp );    
    
    /* Выбор типов выходных выводов порта 
       6, 7 pin open drain
       все остальные push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    temp |= 
          MCU_GPIO_OTYPER_OT6
        | MCU_GPIO_OTYPER_OT7;
    OUT_REG( ptr_reg_gpio->otyper, temp );
    
    /* Настройка скорости работы выводов */
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
    
    /* Настройка подтягивающих резисторов 
       11, 12 подтяжка к питанию; 
       0, 15 подтяжка к земле; 
       остальные выводы без подтяжки */ 
    temp = IN_REG( ptr_reg_gpio->pupdr );
    temp &= ~( uint32_t ) MCU_GPIO_PUPDR_BITS;
    temp |=
          MCU_GPIO_PUPDR_0_DOWN
        | MCU_GPIO_PUPDR_11_UP
        | MCU_GPIO_PUPDR_12_UP
        | MCU_GPIO_PUPDR_15_DOWN;    
    OUT_REG( ptr_reg_gpio->pupdr, temp );
    
    /* Установка флага в низкое состояние (выбран STM в качестве источника синуса) */
    OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_RES_1 );
    
    /* Установка в высокое состояние сигналов выбора АЦП D61, D64 */
	OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_SET_11 );
	OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_SET_12 );
    
    /* Настройка альтернативных функций порта swd, spi2 */
    temp = IN_REG( ptr_reg_gpio->afrh );        
    temp &= ~( uint32_t ) MCU_GPIO_AFRH_BITS;
    temp |=
          /* вывод 3 натраивается по ресету как SWD - TRACESWO */
          MCU_GPIO_AFRH_13_AF5          /* SPI2_SCK */
        | MCU_GPIO_AFRH_14_AF5          /* SPI2_MISO */
        | MCU_GPIO_AFRH_15_AF5;         /* SPI2_MOSI */
    OUT_REG( ptr_reg_gpio->afrh, temp );

/* Настройка порта C */
    
    /* Указатель на начало области регистров порта C */
    ptr_reg_gpio = STM32F407_GPIOC_PTR;
    
    /* Сброс регистра выходных данных */
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* Выбор режимов работы выводов порта 
       8, 9, 13 - 15 pin как входные, 
       10 - 12 pin как альтернативные, 
       0 - 7 pin выходные */
    temp = IN_REG( ptr_reg_gpio->moder );          
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS;
    temp |=
          MCU_GPIO_MODER_0_OUT                  /* РКВ1 */ // Сигналы управления выходными разовыми командами РКВ1 - РКВ8 
        | MCU_GPIO_MODER_1_OUT                  /* РКВ2 */
        | MCU_GPIO_MODER_2_OUT                  /* РКВ3 */
        | MCU_GPIO_MODER_3_OUT                  /* РКВ4 */
        | MCU_GPIO_MODER_4_OUT                  /* РКВ5 */
        | MCU_GPIO_MODER_5_OUT                  /* РКВ6 */
        | MCU_GPIO_MODER_6_OUT                  /* РКВ7 */
        | MCU_GPIO_MODER_7_OUT                  /* РКВ8 */
        | MCU_GPIO_MODER_8_IN                   /* Не используется */
        | MCU_GPIO_MODER_9_IN                   /* Не используется */
        | MCU_GPIO_MODER_10_ALT                 /* SPI3_SCK */  // Сигнал тактирования SPI3
        | MCU_GPIO_MODER_11_ALT                 /* SPI3_MISO */ // Прием последовательных данных от коммутаторов D60, В63
        | MCU_GPIO_MODER_12_ALT                 /* SPI3_MOSI */ // Отправка данных на коммутаторы D60, В63
        | MCU_GPIO_MODER_13_IN                  /* Не используется */
        | MCU_GPIO_MODER_14_IN                  /* Не используется */
        | MCU_GPIO_MODER_15_IN;                 /* Не используется */
    OUT_REG( ptr_reg_gpio->moder, temp );
    
    /* Выбор типов выходных выводов порта, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp );
    
    /* Настройка скорости работы выводов */
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
    
    /* Настройка подтягивающих резисторов  
       8, 9, 13-15 подтяжка к земле; 
       остальные выводы без подтяжки */
    temp = IN_REG( ptr_reg_gpio->pupdr );    
    temp &= ~( uint32_t ) MCU_GPIO_PUPDR_BITS;
    temp |=
          MCU_GPIO_PUPDR_8_DOWN
        | MCU_GPIO_PUPDR_9_DOWN
        | MCU_GPIO_PUPDR_13_DOWN
        | MCU_GPIO_PUPDR_14_DOWN
        | MCU_GPIO_PUPDR_15_DOWN;
    OUT_REG( ptr_reg_gpio->pupdr, temp );
    
    /* Настройка альтернативных функций порта spi3 */
    temp = IN_REG( ptr_reg_gpio->afrh );
    temp &= ~( uint32_t ) MCU_GPIO_AFRH_BITS;
    temp |=
          MCU_GPIO_AFRH_10_AF6              /* SPI3_SCK */  
        | MCU_GPIO_AFRH_11_AF6              /* SPI3_MISO */
        | MCU_GPIO_AFRH_12_AF6;             /* SPI3_MOSI */
    OUT_REG( ptr_reg_gpio->afrh, temp );

/* Настройка порта D */
    
    /* Указатель на начало области регистров порта D */
    ptr_reg_gpio = STM32F407_GPIOD_PTR;
    
    /* Сброс регистра выходных данных */
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* Выбор режимов работы выводов порта 
       0-8, 10, 11, 13 pin как входные, 
       12 pin как альтернативный, 
       9, 14, 15 pin выходные */
    temp = IN_REG( ptr_reg_gpio->moder );
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS;
    temp |=
          MCU_GPIO_MODER_0_IN                  /* РКП1 */          // Разовые команды приема РКП1 - РКП8 
        | MCU_GPIO_MODER_1_IN                  /* РКП2 */
        | MCU_GPIO_MODER_2_IN                  /* РКП3 */
        | MCU_GPIO_MODER_3_IN                  /* РКП4 */
        | MCU_GPIO_MODER_4_IN                  /* РКП5 */
        | MCU_GPIO_MODER_5_IN                  /* РКП6 */
        | MCU_GPIO_MODER_6_IN                  /* РКП7 */
        | MCU_GPIO_MODER_7_IN                  /* РКП8 */
        | MCU_GPIO_MODER_8_IN                  /* Не используется */
        | MCU_GPIO_MODER_9_OUT                 /* HL2 */           // Управление внешним светодиодом HL2 */
        | MCU_GPIO_MODER_10_IN                 /* ZeroCross1 */    // Сигнал с детектора синусоидального сигнала D15
        | MCU_GPIO_MODER_11_IN                 /* ZeroCross2 */    // Сигнал с детектора синусоидального сигнала D65
        | MCU_GPIO_MODER_12_ALT                /* Meandr10kHz_2 */ // Выходной меандр 10 КГц на детектор синуса D65 */
        | MCU_GPIO_MODER_13_IN                 /* РКП Р.з.ПЗУ */   // Разрешение записи ПЗУ программ
        | MCU_GPIO_MODER_14_OUT                /* TR_RST */        // Сброс внешних микросхем D-триггеров D66, D67 */
        | MCU_GPIO_MODER_15_OUT;               /* I_TCLK */        // Сигнал тактирования микросхем D-триггеров D66, D67 */       
    OUT_REG( ptr_reg_gpio->moder, temp );
    
    /* Выбор типов выходных выводов порта, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp );
    
    /* Настройка скорости работы выводов */
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
    
    /* Настройка подтягивающих резисторов 
       14 подтяжка к питанию; 
       0 - 5, 8, 9 подтяжка к земле; 
       остальные выводы без подтяжки */
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
    
    /* Настройка альтернативной функции порта TIM4_CH1 */
    temp = IN_REG( ptr_reg_gpio->afrh );
    temp &= ~( uint32_t ) MCU_GPIO_AFRH_BITS;
    temp |= MCU_GPIO_AFRH_12_AF2;
    OUT_REG( ptr_reg_gpio->afrh, temp );

    /* Инициализация D-триггеров управления РКВ */
    /* Установка сигнала тактирования D-триггеров в начальное состояние */
    OUT_REG( STM32F407_GPIOD_PTR->bsrr, MCU_GPIO_BSRR_RES_15 );
    
    /* Сброса D-триггеров */
    OUT_REG( STM32F407_GPIOD_PTR->bsrr, MCU_GPIO_BSRR_SET_14 );
    delay_mcs( 3U );
    OUT_REG( STM32F407_GPIOD_PTR->bsrr, MCU_GPIO_BSRR_RES_14 );
    delay_mcs( 3U );
    OUT_REG( STM32F407_GPIOD_PTR->bsrr, MCU_GPIO_BSRR_SET_14 );
    
    /* Запуск ШИМ для организации синусоидального сигнала */
    timer_sinus_meandr_init(TIME_STEP_100_MCS);

/* Настройка порта E */

    /* Указатель на начало области регистров порта E */
    ptr_reg_gpio = STM32F407_GPIOE_PTR;
    
    /* Сброс регистра выходных данных */
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* Выбор режимов работы выводов порта, все pin как входные */
    temp = IN_REG( ptr_reg_gpio->moder );
    temp &= ~( uint32_t ) MCU_GPIO_MODER_BITS;
    temp |=
          MCU_GPIO_MODER_0_IN                  /* I_DDT1 */ // Дискретные данные от датчиков тока
        | MCU_GPIO_MODER_1_IN                  /* I_DDT2 */
        | MCU_GPIO_MODER_2_IN                  /* I_DDT3 */
        | MCU_GPIO_MODER_3_IN                  /* I_DDT4 */
        | MCU_GPIO_MODER_4_IN                  /* I_DDT5 */
        | MCU_GPIO_MODER_5_IN                  /* I_DDT6 */
        | MCU_GPIO_MODER_6_IN                  /* I_DDT7 */
        | MCU_GPIO_MODER_7_IN                  /* I_DDT8 */
        | MCU_GPIO_MODER_8_IN                  /* РКП1 */  // эхо-контроль РКВ1 - РКВ8
        | MCU_GPIO_MODER_9_IN                  /* РКП2 */
        | MCU_GPIO_MODER_10_IN                 /* РКП3 */
        | MCU_GPIO_MODER_11_IN                 /* РКП4 */
        | MCU_GPIO_MODER_12_IN                 /* РКП5 */
        | MCU_GPIO_MODER_13_IN                 /* РКП6 */
        | MCU_GPIO_MODER_14_IN                 /* РКП7 */
        | MCU_GPIO_MODER_15_IN;                /* РКП8 */
    OUT_REG( ptr_reg_gpio->moder, temp );
    
    /* Выбор типов выходных выводов порта, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp );
    
    /* Настройка скорости работы выводов */
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
    
    /* Настройка подтягивающих резисторов  
       все выводы без подтяжек */
    temp = IN_REG( ptr_reg_gpio->pupdr );    
    temp &= ~( uint32_t ) MCU_GPIO_PUPDR_BITS;
    OUT_REG( ptr_reg_gpio->pupdr, temp );

/* Настройка порта H */

    /* Указатель на начало области регистров порта H */
    ptr_reg_gpio = STM32F407_GPIOH_PTR;
    
    /* Сброс регистра выходных данных */
    OUT_REG( ptr_reg_gpio->odr, 0x0U );
    
    /* Выбор режимов работы выводов порта, все pin как входные */
    OUT_REG( ptr_reg_gpio->moder, 0x0U );
    
    /* Выбор типов выходных выводов порта, push-pull */
    temp = IN_REG( ptr_reg_gpio->otyper );    
    temp &= ~( uint32_t ) MCU_GPIO_OTYPER_BITS;
    OUT_REG( ptr_reg_gpio->otyper, temp ); 
    
    /* Настройка скорости работы выводов */
    temp = IN_REG( ptr_reg_gpio->ospeedr ); 
    temp &= ~( uint32_t ) MCU_GPIO_OSPEEDR_BITS;
    temp |= MCU_GPIO_OSPEEDR_0_MAX;
    OUT_REG( ptr_reg_gpio->ospeedr, temp );
    
    /* Настройка подтягивающих резисторов  
       все выводы без подтяжек */
    OUT_REG( ptr_reg_gpio->pupdr, 0x0U );

}
/*  Подпрограмма инициализации частот по Reset */
static void un_init_clk( void )
{
    uint32_t        temp;

    stm32f407_rcc_t *ptr_reg_rcc;
    stm32f407_pwr_t *ptr_reg_pwr;
    stm32f407_flash_t *ptr_reg_flash;

    ptr_reg_rcc = STM32F407_RCC_PTR;
    ptr_reg_pwr = STM32F407_PWR_PTR;
    ptr_reg_flash = STM32F407_FLASH_PTR;
      
    /* Включаем HSI */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp |= MCU_RCC_CR_HSION;
    OUT_REG( ptr_reg_rcc->cr, temp );

    while ( 0U == ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_HSIRDY ) ) {
        /* Ожидание выхода HSI на рабочий режим */
        ;
    }  
  
    /* Сброс cfgr регистра */
    OUT_REG( ptr_reg_rcc->cfgr, 0x0U );

    while ( ( IN_REG( ptr_reg_rcc->cfgr ) & MCU_RCC_CFGR_SWS_MASK ) != 0x0U ) {
    /* Ожидание установки HSI в качестве system clock source */
        ;
    }

    /* Отключение CSS и HSE */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp &= ~ ( uint32_t )  ( MCU_RCC_CR_CSSON 
                            | MCU_RCC_CR_HSEON );
    OUT_REG( ptr_reg_rcc->cr, temp );

    while ( ( IN_REG( ptr_reg_rcc->cr ) & MCU_RCC_CR_HSERDY ) != 0x0U ) {
    /* Ожидание отключения HSE */
        ;
    }

    /* Сброс HSEBYP */
    temp  = IN_REG( ptr_reg_rcc->cr );
    temp &= ~ ( uint32_t ) MCU_RCC_CR_HSEBYP;
    OUT_REG( ptr_reg_rcc->cr, temp );

    /* Сброс cr регистра */
    OUT_REG( ptr_reg_pwr->cr, 0x00004000U );
    
    /* Сброс cr регистра */
    OUT_REG( ptr_reg_rcc->cr, 0x00000083U );

    /* Сброс pllcfgr регистра */
    OUT_REG( ptr_reg_rcc->pllcfgr, 0x24003010U ); 
    
    /* Сброс cir регситра */
    OUT_REG( ptr_reg_rcc->cir, 0x0U );

    /* Сброс apb1enr регистра */
    OUT_REG( ptr_reg_rcc->apb1enr, 0x0U );

    /* Сброс acr регистра */
    OUT_REG( ptr_reg_flash->acr, 0x0U );

}
/* Подпрограмма инициализации PORTx по Reset (модуль управления выводами м/сх) */
static void un_init_gpio( void )
{
    stm32f407_gpio_t *ptr_reg_gpio;

    /* Настройка порта A */
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

    /* Настройка порта B */
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

    /* Настройка порта C */
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

    /* Настройка порта D */
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

    /* Настройка порта E */
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

    /* Настройка порта F */
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

    /* Настройка порта G */
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

    /* Настройка порта H */
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

    /* Настройка порта I */
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
