#ifndef __IRQ_MEM_H__
#define __IRQ_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"

/* Программа восстановления начальной настройки контроллера прерываний */
LOADER_FUNC void un_init_irq( void );

/* Подпрограмма инициализации контроллера прерываний */
LOADER_FUNC error_t init_irq( void );

/* Подпрограмма обработки прерывания по умолчанию */
static LOADER_FUNC void kern_irq_default(
    uint32_t frame[], const void *address, uint32_t param );

/* Подпрограмма считывания номера внешнего (относительно ядра) прерывания */
static LOADER_FUNC uint_t kern_irq_get_nvic( void );

/* Подпрограмма прекращения дальнейшей работы */
static LOADER_FUNC void kern_irq_stop( void );

/* Подпрограмма диспетчера прерываний */
LOADER_FUNC void kern_irq_manager(
    uint32_t frame[], const void *address, uint32_t param );

/* Подпрограмма оболочка обработчика исключительной ситуации (exeption) */
LOADER_FUNC void kern_irq_exception_a( void );

/* Подпрограмма запрета прерываний */
LOADER_FUNC uint32_t kern_irq_disable_interrupt( void );

/* Подпрограмма задания значения регистра MSR */
LOADER_FUNC void kern_irq_set_msr(
    uint32_t msr_value );

#endif /* __IRQ_MEM_H__ */
