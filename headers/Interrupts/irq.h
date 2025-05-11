#ifndef __IRQ_H__
#define __IRQ_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif

extern LOADER_FUNC void irq_reset_entry( void );
    
/* Программа восстановления начальной настройки контроллера прерываний */
extern LOADER_FUNC void un_init_irq( void );

/* Подпрограмма инициализации контроллера прерываний */
extern LOADER_FUNC error_t init_irq( void );

/* Подпрограмма запрета прерываний
 * Результат:
 *  Значение регистра MSR на момент установки признака глобального запрета
 * прерываний.
 */
extern LOADER_FUNC uint32_t kern_irq_disable_interrupt( void );

/* Подпрограмма задания значения регистра MSR
 *  (in)  msr_value - задаваемое значение регистра MSR
 */
extern LOADER_FUNC void kern_irq_set_msr(
    uint32_t msr_value );

/* -----------------------------------------------------------------------------
 * Подпрограммы обработки прерываний,
 * вызываемые из менеджера обработки прерываний
 * -----------------------------------------------------------------------------
 */

/* Подпрограмма обработки прерывания по сигналу сброса */
extern LOADER_FUNC void init_startup( void );

#ifdef __cplusplus
}
#endif

#endif /* __IRQ_H__ */
