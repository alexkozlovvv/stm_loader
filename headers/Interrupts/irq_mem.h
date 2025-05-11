#ifndef __IRQ_MEM_H__
#define __IRQ_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"

/* ��������� �������������� ��������� ��������� ����������� ���������� */
LOADER_FUNC void un_init_irq( void );

/* ������������ ������������� ����������� ���������� */
LOADER_FUNC error_t init_irq( void );

/* ������������ ��������� ���������� �� ��������� */
static LOADER_FUNC void kern_irq_default(
    uint32_t frame[], const void *address, uint32_t param );

/* ������������ ���������� ������ �������� (������������ ����) ���������� */
static LOADER_FUNC uint_t kern_irq_get_nvic( void );

/* ������������ ����������� ���������� ������ */
static LOADER_FUNC void kern_irq_stop( void );

/* ������������ ���������� ���������� */
LOADER_FUNC void kern_irq_manager(
    uint32_t frame[], const void *address, uint32_t param );

/* ������������ �������� ����������� �������������� �������� (exeption) */
LOADER_FUNC void kern_irq_exception_a( void );

/* ������������ ������� ���������� */
LOADER_FUNC uint32_t kern_irq_disable_interrupt( void );

/* ������������ ������� �������� �������� MSR */
LOADER_FUNC void kern_irq_set_msr(
    uint32_t msr_value );

#endif /* __IRQ_MEM_H__ */
