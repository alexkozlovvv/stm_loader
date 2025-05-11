#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"
#include "eumw.h"
#include "STM32F407.h"
#include "rs232.h"
#include "irq.h"

#include "timer.h"
#include "timer_mem.h"

#define MAIN_TECH_RK_BLCK_WDT                0x00000004U

/* Режим обработки прерывания от таймера IT:
 * TIMER_NORMAL   - нормальный режим работы. По прерыванию от таймера выдаётся
 *                  сигнал сброса в WDT
 * TIMED_TEST_WDT - режима контроля WDT. По прерыванию от таймера не выдаётся
 *                  сигнал сброса в WDT.
 */
static LOADER_DATA uint32_t timer_mode;
static LOADER_DATA uint32_t timer_control_meandr_mode;
static LOADER_DATA uint32_t timer_first_start_flag;

/* Указатель на флаг окончания периода отсчёта */
static LOADER_DATA volatile uint32_t *timer_ptr_timeout;
static LOADER_DATA volatile uint32_t *timer_control_meandr_ptr_timeout;

/* Счетчик срабатывания светодиода */
static LOADER_DATA volatile uint32_t cnt_led;
static LOADER_DATA volatile uint32_t cnt_flag;

/* Подпрограмма инициализации сторожевого таймера */
void timer_wdt_init( void )
{
    /* Разрешение записи в регистры WDT */
    OUT_REG( STM32F407_IWDG_PTR->kr, MCU_IWDG_KR_KEY_WRITE );       // enable access to the IWDG_PR and IWDG_RLR registers
    while ( 0U != (   IN_REG( STM32F407_IWDG_PTR->sr )                 // ждем когда Watchdog prescaler value update что бы получить возможность выставлять коэффициент деления в регистре pr
                    & ( MCU_IWDG_SR_PVU ) ) 
    ) {
        ;
    }
    /* Задание делителя частоты */    
    OUT_REG( STM32F407_IWDG_PTR->pr, MCU_IWDG_PR_DIV4 );            // выставляем деление на 4 т.е. 32/4 = 8 кГц  (32 кГц это частота внутреннего независимого генератора WDT)
    while ( 0U != (   IN_REG( STM32F407_IWDG_PTR->sr )              // что бы появилась возможность вносить изменения в rlr необходимо снова ждать когда Watchdog counter reload value update в SR 
                    & ( MCU_IWDG_SR_RVU ) ) 
    ) {
        ;
    }
    /* Задание периода срабатывания 125 мс. */    
    OUT_REG( STM32F407_IWDG_PTR->rlr, ( 1000U ) );

    /* Разрешение работы WDT */
    OUT_REG( STM32F407_IWDG_PTR->kr, MCU_IWDG_KR_KEY_ENABLE );      // начало работы wdt с величины 0xFFF
}

/* Подпрограмма выдачи сигнала cброс в WDT */
void timer_wdt_reset( void )
{
    OUT_REG( STM32F407_IWDG_PTR->kr, MCU_IWDG_KR_KEY_RESET );
}

/* Подпрограмма переключения светодиода */
static void timer_led ( void )
{
    uint32_t        temp;
    cnt_led++;
    if ( cnt_led == 50U ) {
        temp = STM32F407_GPIOD_PTR->odr;
        if ( cnt_flag == 0 ) {
            temp &= ~MCU_GPIO_ODR_ODR9;
        } else {
            temp |= MCU_GPIO_ODR_ODR9;
        }
        OUT_REG ( STM32F407_GPIOD_PTR->odr, temp );                
        cnt_led = 0U;
        cnt_flag ^= 1U;
    } 
}

/* Подпрограмма обработчик прерываний от таймера TIM5 
 *  (in)  frame     - указатель на область сохранения регистров при
 *                    входе в прерывание
 *  (in)  address   - адрес возврата
 *  (in)  param     - значение регистра ISPR
 */
void timer_irq(
    uint32_t frame[], const void *address, uint32_t param )
{
    uint32_t        temp;
    
    if ( TIMER_NORMAL == timer_mode ) {
        /* Нормальный режим */
        /* Сброс сторожевого таймера */
        timer_wdt_reset();
        /* Переключение светодиода */
        timer_led();
    }
    
    if ( NULL != timer_ptr_timeout ) {
        /* Установка флага окончания периода отсчёта */
        *timer_ptr_timeout = 1U;
    }
    
    /* Сброс запросов прерываний */
    temp  = IN_REG( STM32F407_TIM5_PTR->sr );
    temp = (uint16_t) ~MCU_GPT_SR_UIF;
    OUT_REG( STM32F407_TIM5_PTR->sr, temp );
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ICPR_TIM5 );
    
    /* Ожидание окончания работы предыдущей команды */
    __asm__( "isb" );

}

/* Подпрограмма инициализации таймера сброса WDT 
 * (in) period       - период срабатывания таймера
 *                    PERIOD_XXX
 * (in) flag_wdt_parm - режим работы:
 *                    TIMER_NORMAL   - обслуживание WDT
 *                    TIMER_TEST_WDT - тест WDT
 * (in) *ptr_timeout - указатель на флаг окончания периода отсчёта
 *                    NULL - не устанавливать флаг
 */
void timer_init(
    uint32_t period, uint32_t flag_wdt_parm, volatile uint32_t *ptr_timeout )
{
    uint32_t         msr_old;
    uint32_t         temp;
    uint32_t         time_cnt;
    stm32f407_gpt_t  *ptr_reg_tim;

    ptr_reg_tim = STM32F407_TIM5_PTR;

    /* Настройка режима работы таймера */
    timer_mode = flag_wdt_parm;
    /* Задание указателя на флаг окончания периода отсчёта */
    timer_ptr_timeout = ptr_timeout;
    /* Задание периода срабатывания */
    if ( period == TIME_STEP_200 ) {
        time_cnt = TIME_CNT_200;
    } else {
        time_cnt = TIME_CNT_10;
    }
    /* Инициализация счетчика переключения светодиода */
    cnt_led = 0;
    cnt_flag = 0;
    
    /* Запрет прерываний от таймера */
    msr_old = kern_irq_disable_interrupt();
    STM32F407_NVIC_PTR->icer[1] = CORE_NVIC_ICER_TIM5;          // Запрещаем прерывания от таймера (66 номер прерывания в общем порядке и 51 без системных)
    STM32F407_NVIC_PTR->icpr[1] = CORE_NVIC_ICPR_TIM5;          // Отчистка флага ожидания обработки прерывания от таймера
    
    /* Включение */
    /* TIM_CLK = HCLK / 2 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp |= MCU_RCC_APB1ENR_TIM5EN;                             // Включаем тактирвоние таймера
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );

    /* Останов */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;                     // Выключаем таймер
    OUT_REG( ptr_reg_tim->cr1, temp );

    /* Сброс TIM5 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1rstr );
    temp |= MCU_RCC_APB1RSTR_TIM5RST;                           // выставляем в состояние ресет
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );
    temp &= ~( uint32_t ) ( MCU_RCC_APB1RSTR_TIM5RST );         // сбрасываем состояние ресет
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );	

    kern_irq_set_msr( msr_old );                                // разрешаем глобальные прерывания (почему мы тут разрешаем прерывания если через 3 инструкции снова запретим???

    /* Настройка */
    OUT_REG( ptr_reg_tim->psc, 0U );                            // Устанавливаем значение для делителя частоты
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // Начальное значение счетчика????
    OUT_REG( ptr_reg_tim->arr, time_cnt);                       // (каждые 10 или 200 мс) Выставляем значение до которого должен считать счетчик в соответствии с выбранным периодом срабатывания 


    /* Задание уровня прерывания */
    msr_old = kern_irq_disable_interrupt();
    temp  = IN_REG( STM32F407_NVIC_PTR->ipr[ 13 ] );
    temp &= ~ ( uint32_t ) CORE_NVIC_IPR_TIM5_MASK;
    temp |= IRQ_TIM5_LEVEL_0;                                   // выставляем уровень прерывания 0
    OUT_REG( STM32F407_NVIC_PTR->ipr[ 13 ], temp );              // Почему 8 регистр ipr ( это же максимум 32 прерывания) нужен 13????
    kern_irq_set_msr( msr_old );                                //                               

    /* Сброс запросов прерываний */
    temp  = IN_REG( ptr_reg_tim->sr );
    temp &= ~ ( uint32_t ) MCU_GPT_SR_BITS;                     // Сбрасываем регстр статуса таймера с запросами прерываний
    OUT_REG( ptr_reg_tim->sr, temp );
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ICPR_TIM5 );  // Отчистка флага ожидания обработки прерывания от таймера  (Зачем ????)

    /* Разрешение прерываний от таймера */
    msr_old = kern_irq_disable_interrupt();                        // снова запрещаем прерывания????
    temp  = IN_REG( ptr_reg_tim->dier );                            // 
    temp &= ~ ( uint32_t ) MCU_GPT_DIER_BITS;                       // Обнуляем биты регистра dier
    temp |= MCU_GPT_DIER_UIE;                                       // Update interrupt enabled  ????
    OUT_REG( ptr_reg_tim->dier, temp );
    OUT_REG( STM32F407_NVIC_PTR->iser[1], CORE_NVIC_ISER_TIM5 );       // разрешаем прерывания от TIM5
    kern_irq_set_msr( msr_old );                                        // Разрешаем глобальные прерывания

    /* Пуск таймера */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp |= MCU_GPT_CR1_CEN;
    OUT_REG( ptr_reg_tim->cr1, temp );                                  // Включаем таймер
}

/* Подпрограмма прекращения работы таймера */
void timer_close( void )
{
    uint32_t        temp;
    uint32_t        msr_old;

    msr_old = kern_irq_disable_interrupt();
    /* Запрет прерываний */
    temp  = IN_REG( STM32F407_TIM5_PTR->dier );
    temp &= ~ ( uint32_t ) MCU_GPT_DIER_BITS;                           // Обнуляем биты регистра dier
    OUT_REG( STM32F407_TIM5_PTR->dier, temp );
    OUT_REG( STM32F407_NVIC_PTR->icer[1], CORE_NVIC_ICER_TIM5 );        // Запрещаем прерывания от таймера (66 номер прерывания в общем порядке и 51 без системных)
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ICPR_TIM5 );        // Отчистка флага ожидания обработки прерывания от таймера
    /* Останов */
    temp  = IN_REG( STM32F407_TIM5_PTR->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;
    OUT_REG( STM32F407_TIM5_PTR->cr1, temp );                           // Выключаем таймер

    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp &= ~ ( uint32_t ) MCU_RCC_APB1ENR_TIM5EN;                      // Выключаем тактирвоние таймера
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );
    kern_irq_set_msr( msr_old );                                        // Разрешаем прерывания
}

/* Подпрограмма обработчик прерываний от таймера TIM2. Приоритет должен быть ниже чем у таймера сброса WDT */
void timer_wait_eoc_irq(
    uint32_t frame[], const void *address, uint32_t param )
{
    
    if ( NULL != timer_control_meandr_ptr_timeout ) {
        /* Установка флага окончания периода отсчёта */
        *timer_control_meandr_ptr_timeout = 1U;
    }

    timer_wait_eoc_close();

}

/* Подпрограмма инициализации таймера ожидания сигналов EOC 
 * (in) period       - период срабатывания таймера
 *                    PERIOD_XXX
 * (in) *ptr_timeout - указатель на флаг окончания периода отсчёта
 *                    NULL - не устанавливать флаг
 */
void timer_wait_eoc_init( uint32_t period, volatile uint32_t *ptr_timeout)
{
    uint32_t         msr_old;
    uint32_t         temp;
    uint32_t         time_cnt;
    stm32f407_gpt_t  *ptr_reg_tim;

    ptr_reg_tim = STM32F407_TIM2_PTR;

    /* Задание указателя на флаг окончания периода отсчёта */
    timer_control_meandr_ptr_timeout = ptr_timeout;
    
    /* Задание периода срабатывания */
    if ( period == TIME_STEP_1_S ) {
        time_cnt = TIME_CNT_1_S;
    } else {
        time_cnt = TIME_CNT_5_S;
    }
    
    /* Запрет прерываний от таймера */
    msr_old = kern_irq_disable_interrupt();
    STM32F407_NVIC_PTR->icer[0] = CORE_NVIC_ICER_TIM2;          // Запрещаем прерывания от таймера (66 номер прерывания в общем порядке и 51 без системных)
    STM32F407_NVIC_PTR->icpr[0] = CORE_NVIC_ICPR_TIM2;          // Отчистка флага ожидания обработки прерывания от таймера
    
    /* Включение */
    /* TIM_CLK = HCLK / 2 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp |= MCU_RCC_APB1ENR_TIM2EN;                             // Включаем тактирвоние таймера
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );

    /* Останов */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;                     // Выключаем таймер
    OUT_REG( ptr_reg_tim->cr1, temp );

    /* Сброс TIM2 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1rstr );
    temp |= MCU_RCC_APB1RSTR_TIM2RST;                           // выставляем в состояние ресет
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );
    temp &= ~( uint32_t ) ( MCU_RCC_APB1RSTR_TIM2RST );         // сбрасываем состояние ресет
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );	

    kern_irq_set_msr( msr_old );                                // разрешаем глобальные прерывания (почему мы тут разрешаем прерывания если через 3 инструкции снова запретим???

    /* Настройка */
    OUT_REG( ptr_reg_tim->psc, 0U );                            // Устанавливаем значение для делителя частоты
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // Начальное значение счетчика????
    OUT_REG( ptr_reg_tim->arr, time_cnt);                       // (каждые 60 мс или 5 с) Выставляем значение до которого должен считать счетчик в соответствии с выбранным периодом срабатывания 


    /* Задание уровня прерывания */
    msr_old = kern_irq_disable_interrupt();
    temp  = IN_REG( STM32F407_NVIC_PTR->ipr[ 7 ] );
    temp &= ~ ( uint32_t ) CORE_NVIC_IPR_TIM2_MASK;
    temp |= IRQ_TIM2_LEVEL_16;                                   // выставляем уровень прерывания 16
    OUT_REG( STM32F407_NVIC_PTR->ipr[ 7 ], temp );              // 
    kern_irq_set_msr( msr_old );                                //                               

    /* Сброс запросов прерываний */
    temp  = IN_REG( ptr_reg_tim->sr );
    temp &= ~ ( uint32_t ) MCU_GPT_SR_BITS;                     // Сбрасываем регстр статуса таймера с запросами прерываний
    OUT_REG( ptr_reg_tim->sr, temp );
    OUT_REG( STM32F407_NVIC_PTR->icpr[0], CORE_NVIC_ICPR_TIM2 );  // Отчистка флага ожидания обработки прерывания от таймера  (Зачем ????)

    /* Разрешение прерываний от таймера */
    msr_old = kern_irq_disable_interrupt();                        // снова запрещаем прерывания????
    temp  = IN_REG( ptr_reg_tim->dier );                            // 
    temp &= ~ ( uint32_t ) MCU_GPT_DIER_BITS;                       // Обнуляем биты регистра dier
    temp |= MCU_GPT_DIER_UIE;                                       // Update interrupt enabled  ????
    OUT_REG( ptr_reg_tim->dier, temp );
    OUT_REG( STM32F407_NVIC_PTR->iser[0], CORE_NVIC_ISER_TIM2 );       // разрешаем прерывания от TIM5
    kern_irq_set_msr( msr_old );                                        // Разрешаем глобальные прерывания

    /* Пуск таймера */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp |= MCU_GPT_CR1_CEN;
    OUT_REG( ptr_reg_tim->cr1, temp );                                  // Включаем таймер
}

/* Подпрограмма прекращения работы таймера ожидания EOC */
void timer_wait_eoc_close( void )
{
    uint32_t        temp;
    uint32_t        msr_old;

    msr_old = kern_irq_disable_interrupt();
    /* Запрет прерываний */
    temp  = IN_REG( STM32F407_TIM2_PTR->dier );
    temp &= ~ ( uint32_t ) MCU_GPT_DIER_BITS;                           // Обнуляем биты регистра dier
    OUT_REG( STM32F407_TIM2_PTR->dier, temp );
    OUT_REG( STM32F407_NVIC_PTR->icer[0], CORE_NVIC_ICER_TIM2 );        // Запрещаем прерывания от таймера (66 номер прерывания в общем порядке и 51 без системных)
    OUT_REG( STM32F407_NVIC_PTR->icpr[0], CORE_NVIC_ICPR_TIM2 );        // Отчистка флага ожидания обработки прерывания от таймера
    /* Останов */
    temp  = IN_REG( STM32F407_TIM2_PTR->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;
    OUT_REG( STM32F407_TIM2_PTR->cr1, temp );                           // Выключаем таймер

    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp &= ~ ( uint32_t ) MCU_RCC_APB1ENR_TIM2EN;                      // Выключаем тактирвоние таймера
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );
    kern_irq_set_msr( msr_old );                                        // Разрешаем прерывания
}

/* При данной частоте PLL = 100 МГц и для данной реализации подпрограммы
   минимально возможная пауза 2 мкс */
/* Подпрограмма реализации паузы в микросекундах
* (in) delay_duration  - продолжтельность паузы:
*                        от 2 мкс до 1000 мкс ( интервал 1 мкс )
*/
void delay_mcs(
    uint32_t delay_duration)
{        
    stm32f407_gpt_t  *ptr_reg_tim;
    uint32_t         prescalr;
    uint32_t         time_cnt = 0;

    ptr_reg_tim = STM32F407_TIM3_PTR;

    /* Задание периода срабатывания и коэффициента деления */ 
    if ( delay_duration == 2U ) {
        time_cnt = 1U;
    } else if ( delay_duration > 2) {
        time_cnt = ((delay_duration - 2U) * 50U) - 1;
    }
    prescalr = 0U;
       
    /* Включаем тактирвоние таймера */
    /* TIM_CLK = HCLK / 2 */
    STM32F407_RCC_PTR->apb1enr |= MCU_RCC_APB1ENR_TIM3EN;
    
    /* Настройка */
    OUT_REG( ptr_reg_tim->psc, prescalr );                      // Устанавливаем значение для делителя частоты
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // Начальное значение счетчика
    OUT_REG( ptr_reg_tim->arr, time_cnt);
    
    /* Разрешаем update interrapt только по переполнению */
    ptr_reg_tim->cr1 |= MCU_GPT_CR1_URS; 
    
    /* Вызываем событие обновления */
    ptr_reg_tim->egr |= MCU_GPT_EGR_UG;
    
    /* Пуск таймера */
    ptr_reg_tim->cr1 |= MCU_GPT_CR1_CEN;
    
    while(!(IN_REG( ptr_reg_tim->sr) & MCU_GPT_SR_UIF));
     
    ptr_reg_tim->sr &= ~( uint32_t ) ( MCU_GPT_SR_UIF );
    
    /* Останов */
    ptr_reg_tim->cr1 &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;
}

/* Подпрограмма реализации паузы в миллисекундах
* (in) delay_duration  - продолжтельность паузы:
*                        от 1 мс до 1000 мс ( интервал 1 мс )
*/
void delay_ms(
    uint32_t delay_duration)
{
    uint32_t         time_cnt;
    uint32_t         prescalr = 0;         
    stm32f407_gpt_t  *ptr_reg_tim;

    ptr_reg_tim = STM32F407_TIM3_PTR;

    /* Задание периода срабатывания и коэффициента деления */
    time_cnt = (delay_duration * 5U) - 1;
    prescalr = 9999U;
       
    /* Включаем тактирвоние таймера */
    /* TIM_CLK = HCLK / 2 */
    STM32F407_RCC_PTR->apb1enr |= MCU_RCC_APB1ENR_TIM3EN;
    
    /* Настройка */
    OUT_REG( ptr_reg_tim->psc, prescalr );                      // Устанавливаем значение для делителя частоты
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // Начальное значение счетчика
    OUT_REG( ptr_reg_tim->arr, time_cnt);
    
    /* Разрешаем update interrapt только по переполнению */
    ptr_reg_tim->cr1 |= MCU_GPT_CR1_URS; 
    
    /* Вызываем событие обновления */
    ptr_reg_tim->egr |= MCU_GPT_EGR_UG;
    
    /* Пуск таймера */
    ptr_reg_tim->cr1 |= MCU_GPT_CR1_CEN;
    
    while(!(IN_REG( ptr_reg_tim->sr) & MCU_GPT_SR_UIF));
     
    ptr_reg_tim->sr &= ~( uint32_t ) ( MCU_GPT_SR_UIF );
    
    /* Останов */
    ptr_reg_tim->cr1 &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;

}
 
/* Подпрограмма инициализации PWM
* (in) period  - период выходного меандра (мкс)
*/
void timer_sinus_meandr_init( uint32_t period )
{
    uint32_t         msr_old;
    uint32_t         temp;
    uint32_t         time_cnt;
    uint32_t         half_period;
    stm32f407_gpt_t  *ptr_reg_tim;

    ptr_reg_tim = STM32F407_TIM4_PTR;

    /* Задание периода срабатывания */
    if ( period == TIME_STEP_100_MCS ) {
        time_cnt = TIME_CNT_100_MCS;
    } else {
        time_cnt = TIME_CNT_100_MCS;
    }
    /* Момент переключения меандра */
    half_period = time_cnt / 2;
    
    /* Запрет прерываний от таймера */
    msr_old = kern_irq_disable_interrupt();
    STM32F407_NVIC_PTR->icer[0] = CORE_NVIC_ICER_TIM4;          // Запрещаем прерывания от таймера (44 номер прерывания в общем порядке и 28 без системных)
    STM32F407_NVIC_PTR->icpr[0] = CORE_NVIC_ICPR_TIM4;          // Отчистка флага ожидания обработки прерывания от таймера
    
    /* Включение */
    /* TIM_CLK = HCLK / 2 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp |= MCU_RCC_APB1ENR_TIM4EN;                             // Включаем тактирвоние таймера
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );

    /* Останов */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;                     // Выключаем таймер
    OUT_REG( ptr_reg_tim->cr1, temp );

    /* Сброс TIM4 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1rstr );
    temp |= MCU_RCC_APB1RSTR_TIM4RST;                           // выставляем в состояние ресет
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );
    temp &= ~( uint32_t ) ( MCU_RCC_APB1RSTR_TIM4RST );         // сбрасываем состояние ресет
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );	

    /* Настройка режима PWM */
    /* Настраиваем 3 канал таймера как выход */ 
    temp  = IN_REG( ptr_reg_tim->ccmr1 );
    temp &= ~( uint32_t ) MCU_GPT_CCMR1_OCM_CC1S_MASK;
    OUT_REG( ptr_reg_tim->ccmr1, temp );

    /* Определяем режим работы PWM (mode - 1) */
    temp  = IN_REG( ptr_reg_tim->ccmr1 );
    temp &= ~( uint32_t ) MCU_GPT_CCMR1_OCM_OC1M_MASK;
    temp |= MCU_GPT_CCMR1_OCM_OC1M_MOD1;
    OUT_REG( ptr_reg_tim->ccmr1, temp );
    
    /* Настраиваем соответствующий preload register для регистра ccmr2 */
    temp  = IN_REG( ptr_reg_tim->ccmr1 );
    temp |= MCU_GPT_CCMR1_OCM_OC1PE;
    OUT_REG( ptr_reg_tim->ccmr1, temp );
    
    /* Разрешаем обновление рабочих регистров только когда произойдет событие обновления */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp |= ( uint32_t )MCU_GPT_CR1_ARPE;
    OUT_REG( ptr_reg_tim->cr1, temp );
    
    /* Настраиваем основные регистрв таймера */
    OUT_REG( ptr_reg_tim->psc, 0U );                            // Устанавливаем значение для делителя частоты
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // Начальное значение счетчика
    OUT_REG( ptr_reg_tim->ccr1, half_period );                  // Устанавливаем значение с которым происходит сравнение
    OUT_REG( ptr_reg_tim->arr, time_cnt);                       // Выставляем значение до которого должен считать счетчик в соответствии с выбранным периодом срабатывания 
    
    /* Реинициализация рабочих регистров перед запуском таймера */
    temp  = IN_REG( ptr_reg_tim->egr );
    temp |= MCU_GPT_EGR_UG;
    OUT_REG( ptr_reg_tim->egr, temp );
    
    /* Активируем работу соответвующего выходного пина */
    temp  = IN_REG( ptr_reg_tim->ccer );
    temp |= MCU_GPT_CCER_CC1E;
    OUT_REG( ptr_reg_tim->ccer, temp );                               

    /* Сброс запросов прерываний */
    temp  = IN_REG( ptr_reg_tim->sr );
    temp &= ~ ( uint32_t ) MCU_GPT_SR_BITS;                     // Сбрасываем регстр статуса таймера с запросами прерываний
    OUT_REG( ptr_reg_tim->sr, temp );
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ICPR_TIM4 );  // Отчистка флага ожидания обработки прерывания от таймера  (Зачем ????)

    /* Пуск таймера */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp |= MCU_GPT_CR1_CEN;
    OUT_REG( ptr_reg_tim->cr1, temp );                            // Включаем таймер
    
    kern_irq_set_msr( msr_old );
}
