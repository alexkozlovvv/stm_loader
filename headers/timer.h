#ifndef __TIMER_H__
#define __TIMER_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#include "eumw.h"

#ifdef __cplusplus
/* ������, ����������� ����������� ������������� ������������� ��� �� ��������
 * ����� ���������������� �.
 */
extern "C" {
#endif
    
/* ����� ������ ������� �������� WDT */
#define TIMER_TEST_WDT          ( 0x00000000U )
#define TIMER_NORMAL            ( 0x00000001U )
#define WAIT_EOCS               ( 0x00000002U )
    
/* ����� ������ ������� �������� ������� � Xlinx */
#define WAIT_MEANDR             ( 0x00000002U )
#define CONTROL_MEANDR          ( 0x00000003U )
#define WAIT                    ( 0x00000004U )
#define NOT_FIRST_WAIT          ( 0x00000005U )

/* ����� ������ ������� ����� */
#define NS                     ( 0x1U )
#define MCS                    ( 0x2U )
#define MS                     ( 0x3U )
#define SEC                    ( 0x4U )
    
/* ������������ ������ TIM5 */
/* ������ 5 �������� � ������� ���������� (DIR = 0) �� ������� TIM_CLK = HCLK / X */
/* �� ����� ������� ������� PSG �� ������ �������. ������������� PSG = 0 */
#define TIME_TIM_CLK            ( MCU_FREQ_APB1_TIM )
/* ������:
    HCLK = 8 Mhz * 25 = 100 Mhz
    TIME_TIM5_CLK = HCLK / 2
    TIME_STEP_CNT = ( 100000000 / 2 ) / 1000000 * 500 = 500 */
    
/* ������ ������������ ������� (��) - 100 mcs, 10, 20, 50, 60, 100, 200, 500 ms */
#define TIME_STEP_100_MCS   ( 100U )
#define TIME_STEP_10        ( 10000U )
#define TIME_STEP_20        ( 20000U )
#define TIME_STEP_50        ( 50000U )
#define TIME_STEP_60        ( 60000U )
#define TIME_STEP_100       ( 100000U )
#define TIME_STEP_200       ( 200000U )
#define TIME_STEP_500       ( 500000U )
/* ������ ������������ ������� (�) - 5s */
#define TIME_STEP_1_S       ( 1000000U )
#define TIME_STEP_5_S       ( 5000000U )

/* �������� �������� �������, ��������������� TIME_STEP */
#define TIME_CNT_100_MCS    ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_100_MCS ) - 1U )
#define TIME_CNT_10         ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_10 ) - 1U )   // MCU_FREQ_APB1_TIM = 50 ���
#define TIME_CNT_20         ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_20 ) - 1U )
#define TIME_CNT_50         ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_50 ) - 1U )
#define TIME_CNT_60         ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_60 ) - 1U )
#define TIME_CNT_200        ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_200 ) - 1U )
#define TIME_CNT_100        ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_100 ) - 1U )
#define TIME_CNT_500        ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_500 ) - 1U )
#define TIME_CNT_1_S        ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_1_S ) - 1U )
#define TIME_CNT_5_S        ( ( ( MCU_FREQ_APB1_TIM / 1000000U ) * TIME_STEP_5_S ) - 1U )

/* ������� ���������� ������� 2 ( ipr[ 7 ] ) */
#define IRQ_TIM2_LEVEL_0      ( ( uint32_t ) ( ( uint32_t ) 0 << 4 ) )
#define IRQ_TIM2_LEVEL_16      ( ( uint32_t ) ( ( uint32_t ) 1 << 4 ) )

/* ������� ���������� ������� 3 ( ipr[ 7 ] ) */
#define IRQ_TIM3_LEVEL_0      ( ( uint32_t ) ( ( uint32_t ) 0 << 4 ) <<  8 )
#define IRQ_TIM3_LEVEL_16      ( ( uint32_t ) ( ( uint32_t ) 1 << 4 ) <<  8 )

/* ������� ���������� ������� 5 ( ipr[ 13 ] ) */
#define IRQ_TIM5_LEVEL_0      ( ( uint32_t ) ( ( uint32_t ) 0 << 4 ) <<  16 )
#define IRQ_TIM5_LEVEL_16      ( ( uint32_t ) ( ( uint32_t ) 1 << 4 ) <<  16 )

/* ������� ���������� EXTI1 ( ipr[ 0 ] ) */
#define IRQ_EXTI1_LEVEL_16      ( ( uint32_t ) ( ( uint32_t ) 1 << 4 ) <<  24 )
#define IRQ_EXTI1_LEVEL_0      ( ( uint32_t ) ( ( uint32_t ) 0 << 4 ) <<  24 )

/* ������������ ������������� ����������� ������� */
LOADER_FUNC void timer_wdt_init( void );

/* ������������ ������ ������� c���� � WDT */
LOADER_FUNC void timer_wdt_reset( void );
    
/* ������������ ���������� ���������� �� ������� IT */
LOADER_FUNC void timer_irq(
    uint32_t frame[], const void *address, uint32_t param );
        
/* ������������ ������������� ������� */
LOADER_FUNC void timer_meandr_to_wdt_init(
    uint32_t period, uint32_t flag_wdt_parm, volatile uint32_t *ptr_timeout );

/* ������������ ����������� ������ ������� */
LOADER_FUNC void timer_close( void );

/* ������������ ������������ ���������� */
static LOADER_FUNC void timer_led ( void );

LOADER_FUNC void timer_init(
    uint32_t period, uint32_t flag_wdt_parm, volatile uint32_t *ptr_timeout );

LOADER_FUNC void timer_wait_eoc_init( uint32_t period, volatile uint32_t *ptr_timeout);

LOADER_FUNC void timer_meandr_control_irq(
    uint32_t frame[], const void *address, uint32_t param );

static LOADER_FUNC void xlinx_reset( void );

LOADER_FUNC void timer_sinus_meandr_init( uint32_t period );

LOADER_FUNC void delay_ns(
    uint32_t delay_duration);

LOADER_FUNC void delay_mcs(
    uint32_t delay_duration);

LOADER_FUNC void delay_ms(
    uint32_t delay_duration);

LOADER_FUNC void timer_wait_eoc_close( void );

LOADER_FUNC void timer_wait_eoc_irq(
    uint32_t frame[], const void *address, uint32_t param );

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H__ */
