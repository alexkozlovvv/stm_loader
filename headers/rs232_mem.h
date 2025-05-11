#ifndef __RS232_MEM_H__
#define __RS232_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"

/* ������������ ������ ������� � ����� RS-232 */
LOADER_FUNC void rs232_putc(
    char_t c );

/* ������������ ������ ������� �� ������ RS-232 */
LOADER_FUNC void rs232_getc(
    char_t *c );

/* ������������ ������ ������ �������� � ����� RS-232 */
LOADER_FUNC uint_t rs232_puts(
    const char_t str[] );

/* ������������ ������������� ������ RS-232 */
LOADER_FUNC error_t rs232_init(
    uint_t baud, uint_t parity, uint_t stop_bits, uint_t width,
    uint_t hw_cntr );

/* ������������ ����������� ������ ������ RS-232 */
LOADER_FUNC void rs232_close( void );

/* ������������ ������ ������ c ��������� */
LOADER_FUNC void rs232_str_out(
    const char_t str[], uint_t len );

/* ������������ ������ ����� � ��������� */
LOADER_FUNC void rs232_num_out(
    uint32_t num, uint_t base );

#endif /* __RS232_MEM_H__ */
