#include "sections_bsp.h"
#include "base_types.h"
#include "os_def.h"
#include "eumw.h"
#include "STM32F407.h"
#include "timer.h"
#include "irq.h"
#include "irq_levels.h"

/* Тип данных - описание подпрограммы обработки прерываний */
typedef struct tag_irq_item {
    /* Указатель подпрограмму обработки прерывания
     *  (in)  frame     - указатель на область сохранения регистров при
     *                    входе в прерывание
     *  (in)  address   - адрес возврата
     *  (in)  param     - значение регистра ISPR
     */
    void         ( *program )( uint32_t frame[], const void *address, uint32_t param );
} irq_item_t;

#include "irq_mem.h"


#define IRQ_TABLE_ENTRY         ( 98U )

/* Счётчики вызовов обработчиков прерываний */
static LOADER_DATA uint_t volatile irq_call_cnt[ IRQ_TABLE_ENTRY ];

/* Таблица описаний подпрограмм обработки прерываний */
static const LOADER_CONST irq_item_t irq_item[ IRQ_TABLE_ENTRY ] = {

    /* ВНИМАНИЕ!!! В описание архитектуры сказано, что внесение изменений в
       настройки системы прерываний должны производится при запрещнных
       прерываниях. Это необходимо учитывать во всех программах, управляющих
       настройкой системы прерываний */

    { &kern_irq_default }, /*  0. Top of Stack main      */
    { &kern_irq_default }, /*  1. Reset Handler          */
    { &kern_irq_default }, /*  2. NMI Handler            */
    { &kern_irq_default }, /*  3. Hard Fault Handler     */
    { &kern_irq_default }, /*  4. Mem Manage Handler     */
    { &kern_irq_default }, /*  5. Bus Fault Handler      */
    { &kern_irq_default }, /*  6. Usage Fault Handler    */
    { &kern_irq_default }, /*  7. Reserved */
    { &kern_irq_default }, /*  8. Reserved */
    { &kern_irq_default }, /*  9. Reserved */
    { &kern_irq_default }, /* 10. Reserved */
    { &kern_irq_default }, /* 11. SVCall Handler         */
    { &kern_irq_default }, /* 12. Debug Monitor Handler  */
    { &kern_irq_default }, /* 13. Reserved */
    { &kern_irq_default }, /* 14. PendSV Handler         */
    { &kern_irq_default }, /* 15. SysTick Handler        */
    /* External Interrupts */
    { &kern_irq_default }, /* 16. Window WatchDog Handler */
    { &kern_irq_default }, /* 17. PVD through EXTI Line detection Handler */
    { &kern_irq_default }, /* 18. Tamper and TimeStamps through the EXTI line Handler */
    { &kern_irq_default }, /* 19. RTC Wakeup through the EXTI line Handler */
    { &kern_irq_default }, /* 20. FLASH Handler          */
    { &kern_irq_default }, /* 21. RCC Handler            */
    { &kern_irq_default }, /* 22. EXTI Line0 Handler     */
    { &kern_irq_default }, /* 23. EXTI Line1 Handler     */
    { &kern_irq_default }, /* 24. EXTI Line2 Handler     */
    { &kern_irq_default }, /* 25. EXTI Line3 Handler     */
    { &kern_irq_default }, /* 26. EXTI Line4 Handler     */
    { &kern_irq_default }, /* 27. DMA1 Stream 0 Handler  */
    { &kern_irq_default }, /* 28. DMA1 Stream 1 Handler  */
    { &kern_irq_default }, /* 29. DMA1 Stream 2 Handler  */
    { &kern_irq_default }, /* 30. DMA1 Stream 3 Handler  */
    { &kern_irq_default }, /* 31. DMA1 Stream 4 Handler  */
    { &kern_irq_default }, /* 32. DMA1 Stream 5 Handler  */
    { &kern_irq_default }, /* 33. DMA1 Stream 6 Handler  */
    { &kern_irq_default }, /* 34. ADC1, ADC2 and ADC3s Handler   */
    { &kern_irq_default }, /* 35. CAN1 TX Handler        */
    { &kern_irq_default }, /* 36. CAN1 RX0 Handler       */
    { &kern_irq_default }, /* 37. CAN1 RX1 Handler       */
    { &kern_irq_default }, /* 38. CAN1 SCE Handler       */
    { &kern_irq_default }, /* 39. External Line[9:5]s Handler     */
    { &kern_irq_default }, /* 40. TIM1 Break and TIM9 Handler     */
    { &kern_irq_default }, /* 41. TIM1 Update and TIM10 Handler   */
    { &kern_irq_default }, /* 42. TIM1 Trigger and Commutation and TIM11 Handler */
    { &kern_irq_default }, /* 43. TIM1 Capture Compare Handler    */
    { &timer_wait_eoc_irq }, /* 44. TIM2 Handler           */
    { &kern_irq_default }, /* 45. TIM3 Handler           */
    { &kern_irq_default }, /* 46. TIM4 Handler           */
    { &kern_irq_default }, /* 47. I2C1 Event Handler     */
    { &kern_irq_default }, /* 48. I2C1 Error Handler     */
    { &kern_irq_default }, /* 49. I2C2 Event Handler     */
    { &kern_irq_default }, /* 50. I2C2 Error Handler     */
    { &kern_irq_default }, /* 51. SPI1 Handler           */
    { &kern_irq_default }, /* 52. SPI2 Handler           */
    { &kern_irq_default }, /* 53. USART1 Handler         */
    { &kern_irq_default }, /* 54. USART2 Handler         */
    { &kern_irq_default }, /* 55. USART3 Handler         */
    { &kern_irq_default }, /* 56. External Line[15:10]s Handler     */
    { &kern_irq_default }, /* 57. RTC Alarm (A and B) through EXTI Line Handler   */
    { &kern_irq_default }, /* 58. USB OTG FS Wakeup through EXTI line Handler     */
    { &kern_irq_default }, /* 59. TIM8 Break and TIM12 Handler      */
    { &kern_irq_default }, /* 60. TIM8 Update and TIM13 Handler     */
    { &kern_irq_default }, /* 61. TIM8 Trigger and Commutation and TIM14 Handler  */
    { &kern_irq_default }, /* 62. TIM8 Capture Compare Handler      */
    { &kern_irq_default }, /* 63. DMA1 Stream7 Handler   */
    { &kern_irq_default }, /* 64. FSMC Handler           */
    { &kern_irq_default }, /* 65. SDIO Handler           */
    { &timer_irq },        /* 66. TIM5 Handler           */
    { &kern_irq_default }, /* 67. SPI3 Handler           */
    { &kern_irq_default }, /* 68. UART4 Handler          */
    { &kern_irq_default }, /* 69. UART5 Handler          */
    { &kern_irq_default }, /* 70. TIM6 and DAC1&2 underrun errors Handler     */
    { &kern_irq_default }, /* 71. TIM7 Handler           */
    { &kern_irq_default }, /* 72. DMA2 Stream 0 Handler  */
    { &kern_irq_default }, /* 73. DMA2 Stream 1 Handler  */
    { &kern_irq_default }, /* 74. DMA2 Stream 2 Handler  */
    { &kern_irq_default }, /* 75. DMA2 Stream 3 Handler  */
    { &kern_irq_default }, /* 76. DMA2 Stream 4 Handler  */
    { &kern_irq_default }, /* 77. Ethernet Handler       */
    { &kern_irq_default }, /* 78. Ethernet Wakeup through EXTI line Handler     */
    { &kern_irq_default }, /* 79. CAN2 TX Handler        */
    { &kern_irq_default }, /* 80. CAN2 RX0 Handler       */
    { &kern_irq_default }, /* 81. CAN2 RX1 Handler       */
    { &kern_irq_default }, /* 82. CAN2 SCE Handler       */
    { &kern_irq_default }, /* 83. USB OTG FS Handler     */
    { &kern_irq_default }, /* 84. DMA2 Stream 5 Handler  */
    { &kern_irq_default }, /* 85. DMA2 Stream 6 Handler  */
    { &kern_irq_default }, /* 86. DMA2 Stream 7 Handler  */
    { &kern_irq_default }, /* 87. USART6 Handler         */
    { &kern_irq_default }, /* 88. I2C3 event Handler     */
    { &kern_irq_default }, /* 89. I2C3 error Handler     */
    { &kern_irq_default }, /* 90. USB OTG HS End Point 1 Out Handler      */
    { &kern_irq_default }, /* 91. USB OTG HS End Point 1 In Handler       */
    { &kern_irq_default }, /* 92. USB OTG HS Wakeup through EXTI Handler  */
    { &kern_irq_default }, /* 93. USB OTG HS Handler     */
    { &kern_irq_default }, /* 94. DCMI Handler           */
    { &kern_irq_default }, /* 95. CRYP crypto Handler    */
    { &kern_irq_default }, /* 96. Hash and Rng Handler   */
    { &kern_irq_default }  /* 97. FPU Handler            */
};

/* Программа восстановления начальной настройки системы прерываний */
void un_init_irq( void )
{
    stm32f407_nvic_t *ptr_nvic;
    stm32f407_scb_t *ptr_scb;
    stm32f407_systick_t *ptr_systick;
    
    uint32_t        msr;
    uint32_t        temp;
    uint_t          idx;

    ptr_nvic = STM32F407_NVIC_PTR;
    ptr_scb  = STM32F407_SCB_PTR;
    ptr_systick = STM32F407_SYSTICK_PTR;

    for ( idx = 0U; IRQ_TABLE_ENTRY > idx; idx++ ) {
        irq_call_cnt[ idx ] = 0U;
    }

    msr = kern_irq_disable_interrupt();

    /* Сброс SysTick control and status register */
    OUT_REG( ptr_systick->ctrl, 0x0U );

    /* Сброс регистров разрешения прерываний */
    ptr_nvic = STM32F407_NVIC_PTR;
    OUT_REG( ptr_nvic->icer[0], 0xFFFFFFFFU );          // отключаем все прерывания
    OUT_REG( ptr_nvic->icer[1], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[2], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[3], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[4], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[5], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[6], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[7], 0xFFFFFFFFU );
    
    /* Сброс ожидания на обслуживание прерывания */
    ptr_nvic = STM32F407_NVIC_PTR;
    OUT_REG( ptr_nvic->icpr[0], 0xFFFFFFFFU );          
    OUT_REG( ptr_nvic->icpr[1], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icpr[2], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icpr[3], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icpr[4], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icpr[5], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icpr[6], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icpr[7], 0xFFFFFFFFU );
  
    /* Сброс регистра контроля и состояния прерываний */
    OUT_REG( ptr_scb->icsr, 0x00000000U );

    /* Сброс регистров уровней прерываний */
    OUT_REG( ptr_scb->shpr2, 0x00000000U ); 
    OUT_REG( ptr_scb->shpr3, 0x00000000U );
    for ( idx = 0U; 21 > idx; idx++ ) {
        temp  = IN_REG( ptr_nvic->ipr[ idx ] );
        temp &= 0x00000000U;
        OUT_REG( ptr_nvic->ipr[ idx ], temp );
    }
    
    /* Глобальное разрешение прерываний */
    msr &= ~ ( uint32_t ) MCU_PRIMASK_PRIMASK;
    kern_irq_set_msr( msr );    
}

/* Подпрограмма инициализации контроллера прерываний
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 */
error_t init_irq( void )
{
    stm32f407_nvic_t *ptr_nvic;
    stm32f407_scb_t *ptr_scb;
    uint32_t        msr;
    uint32_t        temp;
    uint_t          idx;

    /* Nested Vectored Interrupt Controller */
    ptr_nvic = STM32F407_NVIC_PTR;
    
    /* System control block */
    ptr_scb  = STM32F407_SCB_PTR;

    /* ВНИМАНИЕ!!! В описание архитектуры сказано, что внесение изменений в
       настройки системы прерываний должны производится при запрещнных
       прерываниях. Это необходимо учитывать во всех программах, управляющих
       настройкой системы прерываний */
    
    /* Обнуление счетчиков прерываний */
    for ( idx = 0U; IRQ_TABLE_ENTRY > idx; idx++ ) {
        irq_call_cnt[ idx ] = 0U;
    }

    /* Глобальный запрет прерываний */
    msr = kern_irq_disable_interrupt(); 

    /* Отключение SysTick */
    temp  = IN_REG( STM32F407_SYSTICK_PTR->ctrl );
    temp &= ~ ( uint32_t ) CORE_SYSTICK_CTRL_ENABLE;
    OUT_REG( STM32F407_SYSTICK_PTR->ctrl, temp );     

    /* Маскирование прерываний */
    OUT_REG( ptr_nvic->icer[0], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[1], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[2], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[3], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[4], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[5], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[6], 0xFFFFFFFFU );
    OUT_REG( ptr_nvic->icer[7], 0xFFFFFFFFU );

    /* Снятие запросов прерываний от SysTick и PendSV */ 
    OUT_REG( ptr_scb->icsr, ( CORE_SCB_ICSR_PENDSVCLR | CORE_SCB_ICSR_PENDSTCLR ) ); 

    /* Задание уровней приоритета прерываний 
       SVCall, PendSV, SysTick exception */
    temp  = IN_REG( ptr_scb->shpr2 );
    temp &= ~ ( uint32_t ) CORE_SHPR2_BITS;
    temp |= IRQ_SV_LEVEL;                               /* Выставляем 192 уровень прерывания SVCall ( Supervisor Call ) */ 
    OUT_REG( ptr_scb->shpr2, temp );
    temp  = IN_REG( ptr_scb->shpr3 );
    temp &= ~ ( uint32_t ) CORE_SHPR3_BITS;
    temp |= IRQ_SYST_LEVEL | IRQ_PENDSV_LEVEL;
    OUT_REG( ptr_scb->shpr3, temp );                    /* Выставляем уровень 128 и 192 SysTick exception, PendSV соответственно */
    
    /* Задание для каждого из 81 внешнего прерывания 
       приоритет 240 */
    for ( idx = 0U; 21 > idx; idx++ ) {
        temp  = IN_REG( ptr_nvic->ipr[ idx ] );
        temp |= 0xF0F0F0F0U;
        OUT_REG( ptr_nvic->ipr[ idx ], temp );
    }

    /* Глобальное разрешение прерываний */
    msr &= ~ ( uint32_t ) MCU_PRIMASK_PRIMASK;
    kern_irq_set_msr( msr );                            /* Выставляем ноль в бит PRIMASK Priority mask register */

    return NO_ERROR;
}

/* Подпрограмма обработки прерывания по умолчанию
 *  (in)  frame     - указатель на область сохранения регистров при
 *                    входе в прерывание
 *  (in)  address   - адрес возврата
 *  (in)  param     - значение регистра ISPR
 */
static void kern_irq_default(
    uint32_t frame[], const void *address, uint32_t param )
{
    /* Останов по критической ошибке */
    kern_irq_stop();
}

/* Подпрограмма считывания номера внешнего (относительно ядра) прерывания */
static uint_t kern_irq_get_nvic( void )
{
    uint32_t        irq_no;
    uint32_t        msr_old;

    do {
        msr_old = kern_irq_disable_interrupt();
        irq_no = ( IN_REG( STM32F407_SCB_PTR->icsr ) & CORE_SCB_ICSR_VECTPENDING_MASK ) >> 12;
        kern_irq_set_msr( msr_old );
    }
    while (
           ( 29U == irq_no ) /* Запрос прерываний от таймров 3 и 4 игнорируем */
        || ( 32U == irq_no ) /* Обработчики запустятся самостоятельно за счет более высокого приоритета */
    );

    return ( uint_t ) irq_no;
}

/* Подпрограмма прекращения дальнейшей работы */
static void kern_irq_stop( void )
{
    ( void ) kern_irq_disable_interrupt();
    for ( ; ; ) {
#if ( 0 == EUMW_DEBUG ) /* Release */
        ;
#elif ( 1 == EUMW_DEBUG ) /* Debug */
        BREAKPOINT;
#else
  #error "Unknown OSMPM_DEBUG"
#endif
    }
}

/* Подпрограмма диспетчера прерываний
 *  (in)  frame     - указатель на область сохранения регистров при
 *                    входе в прерывание
 *  (in)  address   - адрес возврата
 *  (in)  param     - значение регистра IPSR
 */
void kern_irq_manager(
    uint32_t frame[], const void *address, uint32_t param )
{
    register const irq_item_t *ptr_irq_item;
    uint_t          irq_no;

    if ( IRQ_TABLE_ENTRY <= param ) {
        /* Критическая ошибка */
        kern_irq_stop();
    }
    irq_no = 0U;
    do {
        if ( 0U == param ) {
            /* Все прерывания обработаны */
            irq_no = 0U;
        } else if ( 16U > param ) {
            /* Прочие источники прерываний и исключений */
            irq_no = param;
            param = 0U;
        } else if ( 66U == param ) {
            /* Прерывание от таймера модуля учета времени */
            irq_no = param;
            param = 0U;
        } else if ( IRQ_TABLE_ENTRY > param ) {
            /* Активное прерывание от контроллера NVIC */
            irq_no = param;
            param = IRQ_TABLE_ENTRY;
        } else {
            /* Проверка наличия запрашиваемых прерываний от контроллера NVIC */
            irq_no = kern_irq_get_nvic();
            /* Из-за особенностей архитектуры и ПО при выходе на данную ветвь
               программы можно получить номер прерывания, соответствующий
               исключению SVCall и PendSV. Так как за них отвечают отдельные
               обработчики, которые будут вызваны при первой возможность, то
               игнорируем их. Вызов этих обработчиков их данного менеджера
               прерываний не предусмотрен */
            if (   ( 11U == irq_no )
                || ( 14U == irq_no )
            ) {
                irq_no = 0U;
            }
        }
        if ( 0U == irq_no ) {
            /* Нет запросов прерываний */
            ;
        } else {
            /* Обнаружено наличие необработанного прерывания */
            irq_call_cnt[ irq_no ]++;
            ptr_irq_item = &( irq_item[ irq_no ] );
            ptr_irq_item->program( frame, address, param );
        }
    }
    while ( 0U != irq_no );
}
