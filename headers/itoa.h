#ifndef __ITOA_H__
#define __ITOA_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ������������ �������������� ������ 32-� �������� �����
 * � ������
 *  (in)  value     - ������������� �����
 *  (in)  str       - ��������� �������������� ������
 *  (in)  str_len   - ������������ ������ ������ (��������),
 *                    ������� ������ ����� ������
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
extern LOADER_FUNC error_t i32toa(
    int32_t value, char_t str[], uint_t str_len, int32_t base,
    uint_t *ptr_len );

/* ������������ �������������� ������������ ������ 32-� �������� �����
 * � ������
 *  (in)  value     - ������������� �����
 *  (in)  str       - ��������� �������������� ������
 *  (in)  str_len   - ������������ ������ ������ (��������),
 *                    ������� ������ ����� ������
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
extern LOADER_FUNC error_t u32toa(
    uint32_t value, char_t str[], uint_t str_len, uint_t base,
    uint_t *ptr_len );

#ifdef __cplusplus
}
#endif

#endif /* __ITOA_H__ */

