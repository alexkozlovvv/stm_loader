#ifndef __HEXLOAD_MEM_H__
#define __HEXLOAD_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"
#include "hexload.h"

/* Подпрограмма извлечения символа из НЕХ-файла и перекодирования его в
 * шестнадцатиричную цифру */
static LOADER_FUNC error_t hex_parse_digit(
    uint8_t *dest, struct tag_hex_record *ptr_record );

/* Подпрограмма извлечения байта (2 символа) из НЕХ-файла и перекодирования
 * его в число */
static LOADER_FUNC error_t hex_parse_byte(
    uint8_t *dest, struct tag_hex_record *ptr_record );

/* Подпрограмма извлечения слова (4 байта) из НЕХ-файла и перекодирование
 * его в число */
static LOADER_FUNC error_t hex_parse_word(
    uint32_t *dest, struct tag_hex_record *ptr_record, uint32_t flag_32 );

/* Подпрограмма извлечения блока байтов заданного размера, декодирования и
 * записи в ОЗУ по заданному адресу */
static LOADER_FUNC error_t hex_parse_block(
    volatile uint8_t dest[], sz_t size, struct tag_hex_record *ptr_record );

LOADER_FUNC uint32_t u32_swap( uint32_t data );

#endif /* __HEXLOAD_MEM_H__ */
