#ifndef __HEX_LOAD_H__
#define __HEX_LOAD_H__  1

#include "sections_bsp.h"

#include "base_types.h"

#define MAX_REC_LENG  256U

#ifdef __cplusplus
extern "C" {
#endif

/* Строка HEX файла */
typedef struct tag_hex_record {
    uint8_t         buff[ MAX_REC_LENG ];   /* Буфер                    */
    uint_t          idx_read;               /* индекс извлечения        */
    uint_t          idx_save;               /* индекс сохранения        */
    uint32_t        csum;                   /* контрольная сумма записи */
} hex_record_t;
    
/* Подпрограмма загрузки НЕХ-файла в ОЗУ для выполнения
 *  (in)  ptr_entry - указатель на значение адреса точки входа в программу
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 *  ERROR           - Ошибка
 */
extern LOADER_FUNC error_t hex_file_load(
    uint32_t *ptr_entry );

LOADER_FUNC uint32_t u32_swap( uint32_t data );

#ifdef __cplusplus
}
#endif

#endif  /* __HEX_LOAD_H__ */
