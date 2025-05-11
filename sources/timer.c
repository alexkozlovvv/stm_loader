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

/* ����� ��������� ���������� �� ������� IT:
 * TIMER_NORMAL   - ���������� ����� ������. �� ���������� �� ������� �������
 *                  ������ ������ � WDT
 * TIMED_TEST_WDT - ������ �������� WDT. �� ���������� �� ������� �� �������
 *                  ������ ������ � WDT.
 */
static LOADER_DATA uint32_t timer_mode;
static LOADER_DATA uint32_t timer_control_meandr_mode;
static LOADER_DATA uint32_t timer_first_start_flag;

/* ��������� �� ���� ��������� ������� ������� */
static LOADER_DATA volatile uint32_t *timer_ptr_timeout;
static LOADER_DATA volatile uint32_t *timer_control_meandr_ptr_timeout;

/* ������� ������������ ���������� */
static LOADER_DATA volatile uint32_t cnt_led;
static LOADER_DATA volatile uint32_t cnt_flag;

/* ������������ ������������� ����������� ������� */
void timer_wdt_init( void )
{
    /* ���������� ������ � �������� WDT */
    OUT_REG( STM32F407_IWDG_PTR->kr, MCU_IWDG_KR_KEY_WRITE );       // enable access to the IWDG_PR and IWDG_RLR registers
    while ( 0U != (   IN_REG( STM32F407_IWDG_PTR->sr )                 // ���� ����� Watchdog prescaler value update ��� �� �������� ����������� ���������� ����������� ������� � �������� pr
                    & ( MCU_IWDG_SR_PVU ) ) 
    ) {
        ;
    }
    /* ������� �������� ������� */    
    OUT_REG( STM32F407_IWDG_PTR->pr, MCU_IWDG_PR_DIV4 );            // ���������� ������� �� 4 �.�. 32/4 = 8 ���  (32 ��� ��� ������� ����������� ������������ ���������� WDT)
    while ( 0U != (   IN_REG( STM32F407_IWDG_PTR->sr )              // ��� �� ��������� ����������� ������� ��������� � rlr ���������� ����� ����� ����� Watchdog counter reload value update � SR 
                    & ( MCU_IWDG_SR_RVU ) ) 
    ) {
        ;
    }
    /* ������� ������� ������������ 125 ��. */    
    OUT_REG( STM32F407_IWDG_PTR->rlr, ( 1000U ) );

    /* ���������� ������ WDT */
    OUT_REG( STM32F407_IWDG_PTR->kr, MCU_IWDG_KR_KEY_ENABLE );      // ������ ������ wdt � �������� 0xFFF
}

/* ������������ ������ ������� c���� � WDT */
void timer_wdt_reset( void )
{
    OUT_REG( STM32F407_IWDG_PTR->kr, MCU_IWDG_KR_KEY_RESET );
}

/* ������������ ������������ ���������� */
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

/* ������������ ���������� ���������� �� ������� TIM5 
 *  (in)  frame     - ��������� �� ������� ���������� ��������� ���
 *                    ����� � ����������
 *  (in)  address   - ����� ��������
 *  (in)  param     - �������� �������� ISPR
 */
void timer_irq(
    uint32_t frame[], const void *address, uint32_t param )
{
    uint32_t        temp;
    
    if ( TIMER_NORMAL == timer_mode ) {
        /* ���������� ����� */
        /* ����� ����������� ������� */
        timer_wdt_reset();
        /* ������������ ���������� */
        timer_led();
    }
    
    if ( NULL != timer_ptr_timeout ) {
        /* ��������� ����� ��������� ������� ������� */
        *timer_ptr_timeout = 1U;
    }
    
    /* ����� �������� ���������� */
    temp  = IN_REG( STM32F407_TIM5_PTR->sr );
    temp = (uint16_t) ~MCU_GPT_SR_UIF;
    OUT_REG( STM32F407_TIM5_PTR->sr, temp );
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ICPR_TIM5 );
    
    /* �������� ��������� ������ ���������� ������� */
    __asm__( "isb" );

}

/* ������������ ������������� ������� ������ WDT 
 * (in) period       - ������ ������������ �������
 *                    PERIOD_XXX
 * (in) flag_wdt_parm - ����� ������:
 *                    TIMER_NORMAL   - ������������ WDT
 *                    TIMER_TEST_WDT - ���� WDT
 * (in) *ptr_timeout - ��������� �� ���� ��������� ������� �������
 *                    NULL - �� ������������� ����
 */
void timer_init(
    uint32_t period, uint32_t flag_wdt_parm, volatile uint32_t *ptr_timeout )
{
    uint32_t         msr_old;
    uint32_t         temp;
    uint32_t         time_cnt;
    stm32f407_gpt_t  *ptr_reg_tim;

    ptr_reg_tim = STM32F407_TIM5_PTR;

    /* ��������� ������ ������ ������� */
    timer_mode = flag_wdt_parm;
    /* ������� ��������� �� ���� ��������� ������� ������� */
    timer_ptr_timeout = ptr_timeout;
    /* ������� ������� ������������ */
    if ( period == TIME_STEP_200 ) {
        time_cnt = TIME_CNT_200;
    } else {
        time_cnt = TIME_CNT_10;
    }
    /* ������������� �������� ������������ ���������� */
    cnt_led = 0;
    cnt_flag = 0;
    
    /* ������ ���������� �� ������� */
    msr_old = kern_irq_disable_interrupt();
    STM32F407_NVIC_PTR->icer[1] = CORE_NVIC_ICER_TIM5;          // ��������� ���������� �� ������� (66 ����� ���������� � ����� ������� � 51 ��� ���������)
    STM32F407_NVIC_PTR->icpr[1] = CORE_NVIC_ICPR_TIM5;          // �������� ����� �������� ��������� ���������� �� �������
    
    /* ��������� */
    /* TIM_CLK = HCLK / 2 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp |= MCU_RCC_APB1ENR_TIM5EN;                             // �������� ����������� �������
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );

    /* ������� */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;                     // ��������� ������
    OUT_REG( ptr_reg_tim->cr1, temp );

    /* ����� TIM5 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1rstr );
    temp |= MCU_RCC_APB1RSTR_TIM5RST;                           // ���������� � ��������� �����
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );
    temp &= ~( uint32_t ) ( MCU_RCC_APB1RSTR_TIM5RST );         // ���������� ��������� �����
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );	

    kern_irq_set_msr( msr_old );                                // ��������� ���������� ���������� (������ �� ��� ��������� ���������� ���� ����� 3 ���������� ����� ��������???

    /* ��������� */
    OUT_REG( ptr_reg_tim->psc, 0U );                            // ������������� �������� ��� �������� �������
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // ��������� �������� ��������????
    OUT_REG( ptr_reg_tim->arr, time_cnt);                       // (������ 10 ��� 200 ��) ���������� �������� �� �������� ������ ������� ������� � ������������ � ��������� �������� ������������ 


    /* ������� ������ ���������� */
    msr_old = kern_irq_disable_interrupt();
    temp  = IN_REG( STM32F407_NVIC_PTR->ipr[ 13 ] );
    temp &= ~ ( uint32_t ) CORE_NVIC_IPR_TIM5_MASK;
    temp |= IRQ_TIM5_LEVEL_0;                                   // ���������� ������� ���������� 0
    OUT_REG( STM32F407_NVIC_PTR->ipr[ 13 ], temp );              // ������ 8 ������� ipr ( ��� �� �������� 32 ����������) ����� 13????
    kern_irq_set_msr( msr_old );                                //                               

    /* ����� �������� ���������� */
    temp  = IN_REG( ptr_reg_tim->sr );
    temp &= ~ ( uint32_t ) MCU_GPT_SR_BITS;                     // ���������� ������ ������� ������� � ��������� ����������
    OUT_REG( ptr_reg_tim->sr, temp );
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ICPR_TIM5 );  // �������� ����� �������� ��������� ���������� �� �������  (����� ????)

    /* ���������� ���������� �� ������� */
    msr_old = kern_irq_disable_interrupt();                        // ����� ��������� ����������????
    temp  = IN_REG( ptr_reg_tim->dier );                            // 
    temp &= ~ ( uint32_t ) MCU_GPT_DIER_BITS;                       // �������� ���� �������� dier
    temp |= MCU_GPT_DIER_UIE;                                       // Update interrupt enabled  ????
    OUT_REG( ptr_reg_tim->dier, temp );
    OUT_REG( STM32F407_NVIC_PTR->iser[1], CORE_NVIC_ISER_TIM5 );       // ��������� ���������� �� TIM5
    kern_irq_set_msr( msr_old );                                        // ��������� ���������� ����������

    /* ���� ������� */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp |= MCU_GPT_CR1_CEN;
    OUT_REG( ptr_reg_tim->cr1, temp );                                  // �������� ������
}

/* ������������ ����������� ������ ������� */
void timer_close( void )
{
    uint32_t        temp;
    uint32_t        msr_old;

    msr_old = kern_irq_disable_interrupt();
    /* ������ ���������� */
    temp  = IN_REG( STM32F407_TIM5_PTR->dier );
    temp &= ~ ( uint32_t ) MCU_GPT_DIER_BITS;                           // �������� ���� �������� dier
    OUT_REG( STM32F407_TIM5_PTR->dier, temp );
    OUT_REG( STM32F407_NVIC_PTR->icer[1], CORE_NVIC_ICER_TIM5 );        // ��������� ���������� �� ������� (66 ����� ���������� � ����� ������� � 51 ��� ���������)
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ICPR_TIM5 );        // �������� ����� �������� ��������� ���������� �� �������
    /* ������� */
    temp  = IN_REG( STM32F407_TIM5_PTR->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;
    OUT_REG( STM32F407_TIM5_PTR->cr1, temp );                           // ��������� ������

    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp &= ~ ( uint32_t ) MCU_RCC_APB1ENR_TIM5EN;                      // ��������� ����������� �������
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );
    kern_irq_set_msr( msr_old );                                        // ��������� ����������
}

/* ������������ ���������� ���������� �� ������� TIM2. ��������� ������ ���� ���� ��� � ������� ������ WDT */
void timer_wait_eoc_irq(
    uint32_t frame[], const void *address, uint32_t param )
{
    
    if ( NULL != timer_control_meandr_ptr_timeout ) {
        /* ��������� ����� ��������� ������� ������� */
        *timer_control_meandr_ptr_timeout = 1U;
    }

    timer_wait_eoc_close();

}

/* ������������ ������������� ������� �������� �������� EOC 
 * (in) period       - ������ ������������ �������
 *                    PERIOD_XXX
 * (in) *ptr_timeout - ��������� �� ���� ��������� ������� �������
 *                    NULL - �� ������������� ����
 */
void timer_wait_eoc_init( uint32_t period, volatile uint32_t *ptr_timeout)
{
    uint32_t         msr_old;
    uint32_t         temp;
    uint32_t         time_cnt;
    stm32f407_gpt_t  *ptr_reg_tim;

    ptr_reg_tim = STM32F407_TIM2_PTR;

    /* ������� ��������� �� ���� ��������� ������� ������� */
    timer_control_meandr_ptr_timeout = ptr_timeout;
    
    /* ������� ������� ������������ */
    if ( period == TIME_STEP_1_S ) {
        time_cnt = TIME_CNT_1_S;
    } else {
        time_cnt = TIME_CNT_5_S;
    }
    
    /* ������ ���������� �� ������� */
    msr_old = kern_irq_disable_interrupt();
    STM32F407_NVIC_PTR->icer[0] = CORE_NVIC_ICER_TIM2;          // ��������� ���������� �� ������� (66 ����� ���������� � ����� ������� � 51 ��� ���������)
    STM32F407_NVIC_PTR->icpr[0] = CORE_NVIC_ICPR_TIM2;          // �������� ����� �������� ��������� ���������� �� �������
    
    /* ��������� */
    /* TIM_CLK = HCLK / 2 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp |= MCU_RCC_APB1ENR_TIM2EN;                             // �������� ����������� �������
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );

    /* ������� */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;                     // ��������� ������
    OUT_REG( ptr_reg_tim->cr1, temp );

    /* ����� TIM2 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1rstr );
    temp |= MCU_RCC_APB1RSTR_TIM2RST;                           // ���������� � ��������� �����
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );
    temp &= ~( uint32_t ) ( MCU_RCC_APB1RSTR_TIM2RST );         // ���������� ��������� �����
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );	

    kern_irq_set_msr( msr_old );                                // ��������� ���������� ���������� (������ �� ��� ��������� ���������� ���� ����� 3 ���������� ����� ��������???

    /* ��������� */
    OUT_REG( ptr_reg_tim->psc, 0U );                            // ������������� �������� ��� �������� �������
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // ��������� �������� ��������????
    OUT_REG( ptr_reg_tim->arr, time_cnt);                       // (������ 60 �� ��� 5 �) ���������� �������� �� �������� ������ ������� ������� � ������������ � ��������� �������� ������������ 


    /* ������� ������ ���������� */
    msr_old = kern_irq_disable_interrupt();
    temp  = IN_REG( STM32F407_NVIC_PTR->ipr[ 7 ] );
    temp &= ~ ( uint32_t ) CORE_NVIC_IPR_TIM2_MASK;
    temp |= IRQ_TIM2_LEVEL_16;                                   // ���������� ������� ���������� 16
    OUT_REG( STM32F407_NVIC_PTR->ipr[ 7 ], temp );              // 
    kern_irq_set_msr( msr_old );                                //                               

    /* ����� �������� ���������� */
    temp  = IN_REG( ptr_reg_tim->sr );
    temp &= ~ ( uint32_t ) MCU_GPT_SR_BITS;                     // ���������� ������ ������� ������� � ��������� ����������
    OUT_REG( ptr_reg_tim->sr, temp );
    OUT_REG( STM32F407_NVIC_PTR->icpr[0], CORE_NVIC_ICPR_TIM2 );  // �������� ����� �������� ��������� ���������� �� �������  (����� ????)

    /* ���������� ���������� �� ������� */
    msr_old = kern_irq_disable_interrupt();                        // ����� ��������� ����������????
    temp  = IN_REG( ptr_reg_tim->dier );                            // 
    temp &= ~ ( uint32_t ) MCU_GPT_DIER_BITS;                       // �������� ���� �������� dier
    temp |= MCU_GPT_DIER_UIE;                                       // Update interrupt enabled  ????
    OUT_REG( ptr_reg_tim->dier, temp );
    OUT_REG( STM32F407_NVIC_PTR->iser[0], CORE_NVIC_ISER_TIM2 );       // ��������� ���������� �� TIM5
    kern_irq_set_msr( msr_old );                                        // ��������� ���������� ����������

    /* ���� ������� */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp |= MCU_GPT_CR1_CEN;
    OUT_REG( ptr_reg_tim->cr1, temp );                                  // �������� ������
}

/* ������������ ����������� ������ ������� �������� EOC */
void timer_wait_eoc_close( void )
{
    uint32_t        temp;
    uint32_t        msr_old;

    msr_old = kern_irq_disable_interrupt();
    /* ������ ���������� */
    temp  = IN_REG( STM32F407_TIM2_PTR->dier );
    temp &= ~ ( uint32_t ) MCU_GPT_DIER_BITS;                           // �������� ���� �������� dier
    OUT_REG( STM32F407_TIM2_PTR->dier, temp );
    OUT_REG( STM32F407_NVIC_PTR->icer[0], CORE_NVIC_ICER_TIM2 );        // ��������� ���������� �� ������� (66 ����� ���������� � ����� ������� � 51 ��� ���������)
    OUT_REG( STM32F407_NVIC_PTR->icpr[0], CORE_NVIC_ICPR_TIM2 );        // �������� ����� �������� ��������� ���������� �� �������
    /* ������� */
    temp  = IN_REG( STM32F407_TIM2_PTR->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;
    OUT_REG( STM32F407_TIM2_PTR->cr1, temp );                           // ��������� ������

    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp &= ~ ( uint32_t ) MCU_RCC_APB1ENR_TIM2EN;                      // ��������� ����������� �������
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );
    kern_irq_set_msr( msr_old );                                        // ��������� ����������
}

/* ��� ������ ������� PLL = 100 ��� � ��� ������ ���������� ������������
   ���������� ��������� ����� 2 ��� */
/* ������������ ���������� ����� � �������������
* (in) delay_duration  - ���������������� �����:
*                        �� 2 ��� �� 1000 ��� ( �������� 1 ��� )
*/
void delay_mcs(
    uint32_t delay_duration)
{        
    stm32f407_gpt_t  *ptr_reg_tim;
    uint32_t         prescalr;
    uint32_t         time_cnt = 0;

    ptr_reg_tim = STM32F407_TIM3_PTR;

    /* ������� ������� ������������ � ������������ ������� */ 
    if ( delay_duration == 2U ) {
        time_cnt = 1U;
    } else if ( delay_duration > 2) {
        time_cnt = ((delay_duration - 2U) * 50U) - 1;
    }
    prescalr = 0U;
       
    /* �������� ����������� ������� */
    /* TIM_CLK = HCLK / 2 */
    STM32F407_RCC_PTR->apb1enr |= MCU_RCC_APB1ENR_TIM3EN;
    
    /* ��������� */
    OUT_REG( ptr_reg_tim->psc, prescalr );                      // ������������� �������� ��� �������� �������
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // ��������� �������� ��������
    OUT_REG( ptr_reg_tim->arr, time_cnt);
    
    /* ��������� update interrapt ������ �� ������������ */
    ptr_reg_tim->cr1 |= MCU_GPT_CR1_URS; 
    
    /* �������� ������� ���������� */
    ptr_reg_tim->egr |= MCU_GPT_EGR_UG;
    
    /* ���� ������� */
    ptr_reg_tim->cr1 |= MCU_GPT_CR1_CEN;
    
    while(!(IN_REG( ptr_reg_tim->sr) & MCU_GPT_SR_UIF));
     
    ptr_reg_tim->sr &= ~( uint32_t ) ( MCU_GPT_SR_UIF );
    
    /* ������� */
    ptr_reg_tim->cr1 &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;
}

/* ������������ ���������� ����� � �������������
* (in) delay_duration  - ���������������� �����:
*                        �� 1 �� �� 1000 �� ( �������� 1 �� )
*/
void delay_ms(
    uint32_t delay_duration)
{
    uint32_t         time_cnt;
    uint32_t         prescalr = 0;         
    stm32f407_gpt_t  *ptr_reg_tim;

    ptr_reg_tim = STM32F407_TIM3_PTR;

    /* ������� ������� ������������ � ������������ ������� */
    time_cnt = (delay_duration * 5U) - 1;
    prescalr = 9999U;
       
    /* �������� ����������� ������� */
    /* TIM_CLK = HCLK / 2 */
    STM32F407_RCC_PTR->apb1enr |= MCU_RCC_APB1ENR_TIM3EN;
    
    /* ��������� */
    OUT_REG( ptr_reg_tim->psc, prescalr );                      // ������������� �������� ��� �������� �������
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // ��������� �������� ��������
    OUT_REG( ptr_reg_tim->arr, time_cnt);
    
    /* ��������� update interrapt ������ �� ������������ */
    ptr_reg_tim->cr1 |= MCU_GPT_CR1_URS; 
    
    /* �������� ������� ���������� */
    ptr_reg_tim->egr |= MCU_GPT_EGR_UG;
    
    /* ���� ������� */
    ptr_reg_tim->cr1 |= MCU_GPT_CR1_CEN;
    
    while(!(IN_REG( ptr_reg_tim->sr) & MCU_GPT_SR_UIF));
     
    ptr_reg_tim->sr &= ~( uint32_t ) ( MCU_GPT_SR_UIF );
    
    /* ������� */
    ptr_reg_tim->cr1 &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;

}
 
/* ������������ ������������� PWM
* (in) period  - ������ ��������� ������� (���)
*/
void timer_sinus_meandr_init( uint32_t period )
{
    uint32_t         msr_old;
    uint32_t         temp;
    uint32_t         time_cnt;
    uint32_t         half_period;
    stm32f407_gpt_t  *ptr_reg_tim;

    ptr_reg_tim = STM32F407_TIM4_PTR;

    /* ������� ������� ������������ */
    if ( period == TIME_STEP_100_MCS ) {
        time_cnt = TIME_CNT_100_MCS;
    } else {
        time_cnt = TIME_CNT_100_MCS;
    }
    /* ������ ������������ ������� */
    half_period = time_cnt / 2;
    
    /* ������ ���������� �� ������� */
    msr_old = kern_irq_disable_interrupt();
    STM32F407_NVIC_PTR->icer[0] = CORE_NVIC_ICER_TIM4;          // ��������� ���������� �� ������� (44 ����� ���������� � ����� ������� � 28 ��� ���������)
    STM32F407_NVIC_PTR->icpr[0] = CORE_NVIC_ICPR_TIM4;          // �������� ����� �������� ��������� ���������� �� �������
    
    /* ��������� */
    /* TIM_CLK = HCLK / 2 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
    temp |= MCU_RCC_APB1ENR_TIM4EN;                             // �������� ����������� �������
    OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );

    /* ������� */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp &= ~ ( uint32_t ) MCU_GPT_CR1_CEN;                     // ��������� ������
    OUT_REG( ptr_reg_tim->cr1, temp );

    /* ����� TIM4 */
    temp  = IN_REG( STM32F407_RCC_PTR->apb1rstr );
    temp |= MCU_RCC_APB1RSTR_TIM4RST;                           // ���������� � ��������� �����
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );
    temp &= ~( uint32_t ) ( MCU_RCC_APB1RSTR_TIM4RST );         // ���������� ��������� �����
    OUT_REG( STM32F407_RCC_PTR->apb1rstr, temp );	

    /* ��������� ������ PWM */
    /* ����������� 3 ����� ������� ��� ����� */ 
    temp  = IN_REG( ptr_reg_tim->ccmr1 );
    temp &= ~( uint32_t ) MCU_GPT_CCMR1_OCM_CC1S_MASK;
    OUT_REG( ptr_reg_tim->ccmr1, temp );

    /* ���������� ����� ������ PWM (mode - 1) */
    temp  = IN_REG( ptr_reg_tim->ccmr1 );
    temp &= ~( uint32_t ) MCU_GPT_CCMR1_OCM_OC1M_MASK;
    temp |= MCU_GPT_CCMR1_OCM_OC1M_MOD1;
    OUT_REG( ptr_reg_tim->ccmr1, temp );
    
    /* ����������� ��������������� preload register ��� �������� ccmr2 */
    temp  = IN_REG( ptr_reg_tim->ccmr1 );
    temp |= MCU_GPT_CCMR1_OCM_OC1PE;
    OUT_REG( ptr_reg_tim->ccmr1, temp );
    
    /* ��������� ���������� ������� ��������� ������ ����� ���������� ������� ���������� */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp |= ( uint32_t )MCU_GPT_CR1_ARPE;
    OUT_REG( ptr_reg_tim->cr1, temp );
    
    /* ����������� �������� �������� ������� */
    OUT_REG( ptr_reg_tim->psc, 0U );                            // ������������� �������� ��� �������� �������
    OUT_REG( ptr_reg_tim->cnt, 0U );                            // ��������� �������� ��������
    OUT_REG( ptr_reg_tim->ccr1, half_period );                  // ������������� �������� � ������� ���������� ���������
    OUT_REG( ptr_reg_tim->arr, time_cnt);                       // ���������� �������� �� �������� ������ ������� ������� � ������������ � ��������� �������� ������������ 
    
    /* ��������������� ������� ��������� ����� �������� ������� */
    temp  = IN_REG( ptr_reg_tim->egr );
    temp |= MCU_GPT_EGR_UG;
    OUT_REG( ptr_reg_tim->egr, temp );
    
    /* ���������� ������ �������������� ��������� ���� */
    temp  = IN_REG( ptr_reg_tim->ccer );
    temp |= MCU_GPT_CCER_CC1E;
    OUT_REG( ptr_reg_tim->ccer, temp );                               

    /* ����� �������� ���������� */
    temp  = IN_REG( ptr_reg_tim->sr );
    temp &= ~ ( uint32_t ) MCU_GPT_SR_BITS;                     // ���������� ������ ������� ������� � ��������� ����������
    OUT_REG( ptr_reg_tim->sr, temp );
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ICPR_TIM4 );  // �������� ����� �������� ��������� ���������� �� �������  (����� ????)

    /* ���� ������� */
    temp  = IN_REG( ptr_reg_tim->cr1 );
    temp |= MCU_GPT_CR1_CEN;
    OUT_REG( ptr_reg_tim->cr1, temp );                            // �������� ������
    
    kern_irq_set_msr( msr_old );
}
