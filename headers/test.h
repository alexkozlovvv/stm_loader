#ifndef __TEST_H__
#define __TEST_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ����� �������� */
/* ��������� �������� */
#define TEST_MODE_START         ( 0U )
/* ������ ������������ */
#define TEST_MODE_FULL          ( 1U )

/* ������������ �������� ���
 * (in) mode - ����� �������� (��������� ��� ������ ��������)
 */
extern LOADER_FUNC void test_start_ram(
    uint_t mode );

/* ������������ �������� ���
 * (in) mode - ����� �������� (��������� ��� ������ ��������)
 */
extern LOADER_FUNC void test_start_rom(
    uint_t mode );

/* ������������ �������� ���
 * (in) mode - ����� �������� (��������� ��� ������ ��������)
 */
extern LOADER_FUNC void test_start_cpu(
    uint_t mode );

/* ������������ �������� ����������� �������
 * (in) mode - ����� �������� (��������� ��� ������ ��������)
 */
extern LOADER_FUNC error_t test_start_wdt(
    uint_t mode );
extern LOADER_FUNC void test_start_wdt_end( void );

/* ������������ ������������ ������� ���
 * (in)  - ��������� ����� ������� ���
 * (in)  - �������� ����� ������� ���
 * (out) - ������� ���������� ��� ������
 */
extern LOADER_FUNC error_t test_ram(
    uint32_t beg_address, uint32_t end_address );

/* ������������ ������������ ���������� */
extern LOADER_FUNC error_t test_cpu( void );

/* ������������ ������������ ������� ��� */
extern LOADER_FUNC error_t test_rom ( 
    uint32_t mem_arr );

/* ������������ ������������ ��� */
extern LOADER_FUNC void test_start_adc(
    uint_t mode );

extern LOADER_FUNC error_t test_get_adc(
    uint32_t pos);

extern LOADER_FUNC void test_get_temp (void);

/* ������������ �������� � �������� ����������� ����� ������� ���
 * (in) data      - ��������� �� ������� ���
 * (in) data_size - ������ �������
 * (in) csum      - ����������� �����, ���������� � ���
 * ���������:
 *  NO_ERROR      - �������� ����������
 *  ERROR         - ������
 */
extern LOADER_FUNC error_t test_rom_pass(
    const uint8_t data[], sz_t data_size, uint32_t csum );

extern LOADER_FUNC void test_rom_csum( 
    uint32_t addr, uint32_t size, uint32_t *cs );

#ifdef __cplusplus
}
#endif

#endif /* __TEST_H__ */

