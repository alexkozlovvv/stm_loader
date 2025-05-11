#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"

#include "itoa.h"
#include "itoa_mem.h"

/* ������ �������� ��� �������������� ����� � ������ */
static LOADER_CONST const char_t itoa_symbols[] =
    "ZYXWVUTSRQPONMLKJIHGFEDCBA9876543210123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

/* ������������ �������������� ������ 32-� �������� �����
 * � ������
 *  (in)  value     - ������������� �����
 *  (in)  str       - ��������� �� �������������� ������
 *  (in)  str_len   - ������������ ������ ������ (��������)
 *                    (������� ������ ����� ������)
 *  (in)  base      - ��������� ������� ���������
 *  (out) ptr_len   - ��������� �� ����� �������� � �������������� ������
 *                    NULL - �� ���������� ����� ��������
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  INVALID_PARAM   - ������. ������������ ������� ���������
 *  ERROR           - ������. ������������� ������ �������������� ������
 * ����������
 *  ������������� ��������� ����� ������
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

/* ������������ �������������� ������������ ������ 32-� �������� �����
 * � ������
 *  (in)  value     - ������������� �����
 *  (in)  str       - ��������� �� �������������� ������
 *  (in)  str_len   - ������������ ������ ������ (��������)
 *                    (������� ������ ����� ������)
 *  (in)  base      - ��������� ������� ���������
 *  (out) ptr_len   - ��������� �� ����� �������� � �������������� ������
 *                    NULL - �� ���������� ����� ��������
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  INVALID_PARAM   - ������. ������������ ������� ���������
 *  ERROR           - ������. ������������� ������ �������������� ������
 * ����������
 *  ������������� ��������� ����� ������
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
