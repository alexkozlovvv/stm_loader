#ifndef __RS232_H__
#define __RS232_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS232_PARITY_NO         ( 0U )
#define RS232_PARITY_ODD        ( 1U )
#define RS232_PARITY_EVEN       ( 2U )


#define RS232_CTS_RTS_USE       ( 1U )
#define RS232_CTS_RTS_NOUSE     ( 0U )

#define RS232_SB_1              ( 1U )
#define RS232_SB_2              ( 2U )
	
#define RS232_BITS_8            ( 8U )
#define RS232_BITS_9            ( 9U )


/* Подпрограмма выдачи символа в канал RS-232 */
extern LOADER_FUNC  void rs232_putc(
    char_t c );

/* Подпрограмма приема символа из канала RS-232 */
extern LOADER_FUNC void rs232_getc(
    char_t *c );

/* Подпрограмма выдачи строки символов в канал RS-232 */
extern LOADER_FUNC uint_t rs232_puts(
    const char_t str[] );

extern LOADER_FUNC void rs232_get_command(char_t* buffer, int32_t max_size);

/* Подпрограмма инициализации канала RS-232
   Initialization RS232:
    baud      - example 115200
    parity    - RS232_NO_PARITY, RS232_ODD, RS232_EVEN
    stop_bits - RS232_STOPBIT1, RS232_STOPBIT1_5, RS232_STOPBIT2
    width     - RS232_BITS8, RS232_BITS7, RS232_BITS6, RS232_BITS5
    hw_cntr   - hardware flow contorl: RS232_CTS_RTS_USE, RS232_CTS_RTS_NOUSE
   Return:
    NO_ERROR      - Success
    INVALID_PARAM - Error. Invalid param.
 */
extern LOADER_FUNC error_t rs232_init(
    uint_t baud, uint_t parity, uint_t stop_bits, uint_t width,
    uint_t hw_cntr );

/* Подпрограмма прекращения работы канала RS-232 */
extern LOADER_FUNC void rs232_close( void );

/* Подпрограмма выдачи строки c контролем
 * (in) str - указатель на строку
 * (in) len - длина строки
 */
extern LOADER_FUNC void rs232_str_out(
    const char_t str[], uint_t len );

/* Подпрограмма выдачи числа с контролем
 * (in) num  - число
 * (in) base - основание системы счисления
 */
extern LOADER_FUNC void rs232_num_out(
    uint32_t num, uint_t base );

#ifdef __cplusplus
}
#endif

#endif /* __RS232_H__ */
