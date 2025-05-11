#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"

#include "itoa.h"
#include "itoa_mem.h"

/* Массив символов для преобразования числа в строку */
static LOADER_CONST const char_t itoa_symbols[] =
    "ZYXWVUTSRQPONMLKJIHGFEDCBA9876543210123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

/* Подпрограмма преобразования целого 32-х битового числа
 * в строку
 *  (in)  value     - преобразуемое число
 *  (in)  str       - указатель на результирующую строку
 *  (in)  str_len   - максимальный размер строки (символов)
 *                    (включая символ конца строки)
 *  (in)  base      - основание системы счисления
 *  (out) ptr_len   - указатель на число символов в результирующей строке
 *                    NULL - не возвращать число символов
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 *  INVALID_PARAM   - Ошибка. Недопустимые входные параметры
 *  ERROR           - Ошибка. Недостаточный размер результирующей строки
 * Примечание
 *  Автоматически формирует конец строки
 */
error_t i32toa(
    int32_t value, char_t str[], uint_t str_len, int32_t base,
    uint_t *ptr_len )
{
    uint_t          idx, idx1;
    char_t          tmp_char;
    int32_t         tmp_value;
    error_t         err;

    if (   ( base < 2  )
        || ( base > 36 )
        || ( str_len < 1U )
    ) {
        err = INVALID_PARAM;
    } else {
        err = NO_ERROR;
        idx = 0U;
        do {
            tmp_value = value;
            value /= base;
            str[ idx ] = itoa_symbols[ 35 + ( tmp_value - ( value * base ) ) ];
            idx++;
            if ( idx >= str_len ) {
                err = ERROR;
                break;
            }
        } while ( 0U != value );
        if ( NO_ERROR != err ) {
            ;
        } else {
            /* Apply negative sign */
            if ( tmp_value < 0 ) {
                str[ idx ] = '-';
                idx++;
            }
            if ( idx >= str_len ) {
                err = ERROR;
            } else {
                str[ idx ] = '\x0';
                if ( NULL != ptr_len ) {
                    *ptr_len = idx;
                }
                idx--;
                idx1 = 0U;
                while ( idx1 < idx ) {
                    tmp_char = str[ idx ];
                    str[ idx ] = str[ idx1 ];
                    idx--;
                    str[ idx1 ] = tmp_char;
                    idx1++;
                }
            }
        }
    }

    return err;
}

/* Подпрограмма преобразования беззнакового целого 32-х битового числа
 * в строку
 *  (in)  value     - преобразуемое число
 *  (in)  str       - указатель на результирующую строку
 *  (in)  str_len   - максимальный размер строки (символов)
 *                    (включая символ конца строки)
 *  (in)  base      - основание системы счисления
 *  (out) ptr_len   - указатель на число символов в результирующей строке
 *                    NULL - не возвращать число символов
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 *  INVALID_PARAM   - Ошибка. Недопустимые входные параметры
 *  ERROR           - Ошибка. Недостаточный размер результирующей строки
 * Примечание
 *  Автоматически формирует конец строки
 */
error_t u32toa(
    uint32_t value, char_t str[], uint_t str_len, uint_t base,
    uint_t *ptr_len )
{
    uint_t          idx, idx1;
    char_t          tmp_char;
    uint32_t        tmp_value;
    error_t         err;

    if (   ( base < 2U  )
        || ( base > 36U )
        || ( str_len < 1U )
    ) {
        err = INVALID_PARAM;
    } else {
        err = NO_ERROR;
        idx = 0U;
        do {
            tmp_value = value;
            value /= base;
            str[ idx ] = itoa_symbols[ 35U + ( tmp_value - ( value * base ) ) ];
            idx++;
            if ( idx >= str_len ) {
                err = ERROR;
                break;
            }
        } while ( value );
        if ( NO_ERROR != err ) {
            ;
        } else {
            str[ idx ] = '\x0';
            if ( NULL != ptr_len ) {
                *ptr_len = idx;
            }
            idx--;
            idx1 = 0U;
            while ( idx1 < idx ) {
                tmp_char = str[ idx ];
                str[ idx ] = str[ idx1 ];
                idx--;
                str[ idx1 ] = tmp_char;
                idx1++;
            }
        }
    }

    return err;
}
