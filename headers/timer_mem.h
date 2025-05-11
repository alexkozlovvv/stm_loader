#ifndef __TIMER_MEM_H__
#define __TIMER_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"

/* Подпрограмма переключения светодиода */
static LOADER_FUNC void timer_led( void );

/* Подпрограмма инициализации сторожевого таймера */
LOADER_FUNC void timer_wdt_init( void );

/* Подпрограмма выдачи сигнала cброс в WDT */
LOADER_FUNC void timer_wdt_reset( void );
    
/* Подпрограмма обработчик прерываний от таймера IT */
LOADER_FUNC void timer_irq(
    uint32_t frame[], const void *address, uint32_t param );
        
/* Подпрограмма инициализации таймера */
LOADER_FUNC void timer_meandr_to_wdt_init(
    uint32_t period, uint32_t flag_wdt_parm, volatile uint32_t *ptr_timeout );

/* Подпрограмма прекращения работы таймера */
LOADER_FUNC void timer_close( void );

LOADER_FUNC void delay_ns(
    uint32_t delay_duration);

LOADER_FUNC void delay_mcs(
    uint32_t delay_duration);

LOADER_FUNC void delay_ms(
    uint32_t delay_duration);

#endif /* __TIMER_MEM_H__ */
