#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"

#include "test.h"

/* ������� �������������� ��������������� */
#include "test_mem.h"

/* ��������� ������� */
#define TEST_RAM_CONTROL        ( 1U ) /* ��������� �������� */
#define TEST_RAM_WRITE          ( 2U ) /* ��������� ������   */


/* ������������ ������������ ���
 * (in)  - ��������� ����� ������� ���
 * (in)  - �������� ����� ������� ���
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
error_t test_ram(
    uint32_t beg_address, uint32_t end_address )
{
    uint_t          cnt, cnt_err;
    error_t         err;

    if (   ( beg_address >= end_address )
        || ( 0U != ( beg_address & 0x3U ) )       /* ����������� ������� ������ ��������� ����� ���������� 32 ��������� ���� */
        || ( 0U != ( end_address & 0x3U ) )
    ) {
        err = INVALID_PARAM;
    } else {
        cnt = 0U;
        cnt_err = 0U;
        while ( 3U > cnt ) {
            if ( NO_ERROR != test_ram_pass( beg_address, end_address ) ) {
                cnt_err++;
            }
            cnt++;
            if (   ( 0U == cnt_err )
                && ( 2U == cnt )
            ) {
                /* ��� �������� �������. ������ ������ �� ������ */
                cnt++;
            } else if (
                   ( 2U == cnt_err )
                && ( 2U == cnt )
            ) {
                /* ��� �� �������� �������. ������ ������ �� ������ */
                cnt++;
            } else {
                /* ���������� �������� */
                ;
            }
        }

        err = ( 2U <= cnt_err ) ? ERROR : NO_ERROR;
    }

    return err;
}

/* ������������ ������� ������ ��������� �����
 * (in) - ������ ��� ��������
 * (in) - ������ ��� ������
 * (in) - ��������� �����
 * (in) - �������� �����
 * (in) - ��������� �������
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
static error_t test_ram_forward_pass(
    uint32_t ptrn_c, uint32_t ptrn_w,
    uint32_t beg_address, uint32_t end_address,
    uint32_t mode )
{
    uint32_t        cur_address;     /* ������� ����� � ��� */
    uint32_t       *cadr;            /* ��������� �� ������� ����� */
    error_t         err;

    cur_address = beg_address;
    err = NO_ERROR;
    while ( NO_ERROR == err ) {

        cadr = ( uint32_t * ) cur_address;

        if ( 0U != ( mode & TEST_RAM_CONTROL ) ) {      /* ���� ����� �������� ������ */
            if ( *cadr != ptrn_c ) {
                err = ERROR;
            }
        }
        if ( 0U != ( mode & TEST_RAM_WRITE ) ) {        /* ���� ����� ������ � ������ */
            *cadr = ptrn_w;
        }
        cur_address += 4U;
        if ( cur_address >= end_address ) {
            break;
        }
    }

    return err;
}

/* ������������ ������� ����� ��������� �����
 * (in) - ������ ��� ��������
 * (in) - ������ ��� ������
 * (in) - ��������� �����
 * (in) - �������� �����
 * (in) - ��������� �������
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
static error_t test_ram_back_pass(
    uint32_t ptrn_c, uint32_t ptrn_w,
    uint32_t beg_address, uint32_t end_address,
    uint32_t mode )
{
    uint32_t        cur_address;     /* ������� ����� � ��� */
    uint32_t       *cadr;            /* ��������� �� ������� ����� */
    error_t         err;

    cur_address = end_address - 4U;

    err = NO_ERROR;
    while ( NO_ERROR == err ) {
        cadr = ( uint32_t * ) cur_address;
        if ( 0U != ( mode & TEST_RAM_CONTROL ) ) {
            if ( *cadr != ptrn_c ) {
                err = ERROR;
            }
        }
        if ( 0U != ( mode & TEST_RAM_WRITE ) ) {
            *cadr = ptrn_w;
        }
        cur_address -= 4U;
        if ( cur_address < beg_address ) {
            break;
        }
    }

    return err;
}

/* ������������ ������ ������� ����� ���
 * (in)  - ��������� ����� ������� ���
 * (in)  - �������� ����� ������� ���
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
static error_t test_ram_pass(
    uint32_t beg_address, uint32_t end_address )
{
    error_t err = NO_ERROR;

    if ( NO_ERROR !=
        test_ram_forward_pass( 0U,          0x55555555U,
                               beg_address, end_address,
                                                  TEST_RAM_WRITE )
    ) {
        err = ERROR;
    } else if ( NO_ERROR !=
        test_ram_back_pass(    0x55555555U, 0xAAAAAAAAU,
                               beg_address, end_address,
                               TEST_RAM_CONTROL | TEST_RAM_WRITE )
    ) {
        err = ERROR;
    } else if ( NO_ERROR !=
        test_ram_forward_pass( 0xAAAAAAAAU, 0xFFFFFFFFU,
                               beg_address, end_address,
                               TEST_RAM_CONTROL | TEST_RAM_WRITE )
    ) {
        err = ERROR;
    } else if ( NO_ERROR !=
        test_ram_back_pass(    0xFFFFFFFFU, 0x00000000U,
                               beg_address, end_address,
                               TEST_RAM_CONTROL | TEST_RAM_WRITE )
    ) {
        err = ERROR;
    } else if ( NO_ERROR !=
        test_ram_forward_pass( 0x00000000U, 0U,
                               beg_address, end_address,
                               TEST_RAM_CONTROL                  )
    ) {
        err = ERROR;
    } else {
        err = NO_ERROR;
    }

    return err;
}
