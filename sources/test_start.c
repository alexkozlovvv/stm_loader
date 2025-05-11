#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"
#include "STM32F407.h"
#include "loader.h"
#include "rs232.h"
#include "timer.h"
#include "irq.h"
#include "test.h"
#include "test_mem.h"

/* ������ �������� ������������ ��� */
#define TEST_RAM_START      0x10001000
#define TEST_RAM_END        0x1000C000
#define TEST_SRAM_START     0x20000000
#define TEST_SRAM_END       0x20020000

/* �������������� ��������� */
static LOADER_CONST const char_t test_str_test_sram[] =
    "\r\n���� ����������� ��� (SRAM)           ";
static LOADER_CONST const char_t test_str_test_ccmram[] =
    "\r\n���� ����������� ��� (CCM)            ";
static LOADER_CONST const char_t test_str_test_cpu[] =
    "\r\n���� ���                              ";
static LOADER_CONST const char_t test_str_test_wdt[] =
    "\r\n���� WDT                              ";
static LOADER_CONST const char_t test_str_test_srom[] =
    "\r\n���� ��� ��������� ���. ������������� ";
static LOADER_CONST const char_t test_str_test_urom[] =
    "\r\n���� ��� ������������                 ";
static LOADER_CONST const char_t test_str_test_adc[] =
    "\r\n���� ���                              ";
static LOADER_CONST const char_t test_str_err[] =
    "�����";
static LOADER_CONST const char_t test_str_ok[] =
    "�����";
static LOADER_CONST const char_t test_str_skip[] =
    "��������";
static LOADER_CONST const char_t test_str_enter[] =
    "\r\n";
static LOADER_CONST const char_t test_str_cerr[] =
    "\r\n\r\n���������� ������ ������ � ������ ����������";
static LOADER_CONST const char_t reset[] =
    "\r\n\r\n=======����� ����========\r\n\r\n";

/* ������������ �������� ���
 * (in) mode - �����:
 *             TEST_MODE_START - ��������� ��������
 *             TEST_MODE_FULL  - ������ ��������
 * ����������:
 *  1. � ��������� ������ � � ��������������� ������ ����������� ��� 
 *     SRAM, ��� � CCM ����������������;
 *  2. ����������� �������:
 *     SRAM - 0x20000000 - 0x20020000 ( 128 �� );
 *     CCM  - 0x10001000 - 0x1000C000 ( 44 �� )
 */
void test_start_ram(
    uint_t mode )
{
    uint32_t        test_result;

    /* �������� SRAM
       �������������� ��������� �� ���������� ������ ������� ������ */
    
    rs232_str_out( test_str_test_sram, ( sizeof( test_str_test_sram ) - 1U ) );        // "\r\n���� ����������� ��� (SRAM)
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* ����� ���������� ��������. ���������� ������ */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );              // "��������"
    } else {
        /* ����� ������� �������� */
        /* ����� ���������� ��������. ������ �� ���������� */
        if ( NO_ERROR != test_ram( TEST_SRAM_START, TEST_SRAM_END )
        ) {
            /* ���������� ������ */
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_SRAM;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );            // "�����"
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );              // "�����"
        }
    }

    /* �������� CCM */
    /* � CCM RAM ��������� ������ � ���� ��������������� ���������� */
    
    rs232_str_out( test_str_test_ccmram,                                               // "\r\n���� ����������� ��� (CCM)
                  ( sizeof( test_str_test_ccmram ) - 1U ) );
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* ����� ���������� ��������. ���������� ������ */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );              // "��������"
    } else {
        /* ����� ������� �������� */
        /* ����� ���������� ��������. ������ �� ���������� */
        if ( NO_ERROR != test_ram( TEST_RAM_START, TEST_RAM_END )
        ) {
            /* ���������� ������ */
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_CCM;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );            // "�����"
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );              // "�����"
        }
    }
}

/* ������������ �������� ���
 * (in) mode - �����:
 *             TEST_MODE_START - ��������� ��������
 *             TEST_MODE_FULL  - ������ ��������
 */
void test_start_rom(
    uint_t mode )
{
    uint32_t        test_result;

    /* ������� ��� (������� ��������������� ����������) */
    rs232_str_out( test_str_test_srom, ( sizeof( test_str_test_srom ) - 1U ) );         /* "\r\n���� ��� LOADER */
    test_result = MPM_LOADER_OUT_PTR->test_result;                                     
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* ����� ���������� ��������. ���������� ������ */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
    } else {
        /* ����� ������� �������� */
        /* ����� ���������� ��������. ������ �� ���������� */
        if ( NO_ERROR != test_rom( SROM_MEM_ARR ) ) {
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_SROM;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );
        }
    }

    /* ������� ��� (������� ������������) */
    rs232_str_out( test_str_test_urom, ( sizeof( test_str_test_urom ) - 1U ) );
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* ����� ���������� ��������. ���������� ������ */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
    } else {
        /* ����� ������� �������� */
        /* ����� ���������� ��������. ������ �� ���������� */
        if ( NO_ERROR != test_rom( UROM_MEM_ARR ) ) {
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_UROM;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );
        }
    }
 }

/* ������������ �������� ���
 * (in) mode - �����:
 *             TEST_MODE_START - ��������� ��������
 *             TEST_MODE_FULL  - ������ ��������
 */
void test_start_cpu(
    uint_t mode )
{
    uint32_t        test_result;

    rs232_str_out( test_str_test_cpu, ( sizeof( test_str_test_cpu ) - 1U ) );       /* "\r\n���� ��� */
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* ����� ���������� ��������. ���������� ������ */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
    } else {
        /* ����� ������� �������� */
        /* ����� ���������� ��������. ������ �� ���������� */
        if ( NO_ERROR != test_cpu() ) {
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_CPU;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );
        }
    }
}

/* ������������ �������� ���
 * (in) mode - �����:
 *             TEST_MODE_START - ��������� ��������
 *             TEST_MODE_FULL  - ������ ��������
 */
void test_start_adc(
    uint_t mode )
{
    uint32_t        test_result;

    /* ������� ��� (������� ��������������� ����������) */
    rs232_str_out( test_str_test_adc, ( sizeof( test_str_test_adc ) - 1U ) );        /* "\r\n���� ��� LOADER */
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* ����� ���������� ��������. ���������� ������ */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
    } else {
        /* ����� ������� �������� */
        /* ����� ���������� ��������. ������ �� ���������� */
        
        if ( NO_ERROR != test_adc() ) {
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_ADC;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );
        }
    }
 }

/* ������������ �������� WDT
 * (in) mode - �����:
 *             TEST_MODE_START - ��������� ��������
 *             TEST_MODE_FULL  - ������ ��������
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
error_t test_start_wdt(
    uint_t mode )
{

    volatile uint32_t flag_wdt_timeout;  /* ���� ��������� ������ ������� �������� WDT */
    uint32_t        test_result;
    error_t         err;
    uint32_t        i;

    rs232_str_out( test_str_test_wdt, ( sizeof( test_str_test_wdt ) - 1U ) );
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode ) 
        && (( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) ))
    ) {
        /* ����� ���������� ��������. ���������� ������ */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
        rs232_str_out( test_str_enter, ( sizeof( test_str_enter ) - 1U ) );
        err = NO_ACTION;

    } else {
        /* ����� ������� �������� */
        /* ����� ���������� ��������. ������ �� ���������� */
        
        for ( i = 0; i < 1000000U; i++ ) {
            /* ����� ����� ����������� ������ RS-232 */
            ;
        }
        rs232_close();
        /* ��������� ������� */
        for ( i = 0; i < 1000000U; i++ ) {
            /* ����� ����� ���������� ������ RS-232 */
            ;
        }
        timer_close();
        /* C���� �������� ��������� �������� ����������� ������� */
        flag_wdt_timeout = 0U;
        /* ��������� � ���� ������� �� ������ ������������ 200 ��,
           ��� ������ ������� ����� � ���������� ������ */
        timer_init( TIME_STEP_200, TIMER_TEST_WDT, &flag_wdt_timeout );
        /* �������� �������� ��������� �������� ����������� ������� */
        while ( flag_wdt_timeout != 1U ) {
            ;
        }
        /* �������������� �� ��� ������� �� 200 �� - ����� WDT */
        MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_WDT;
        err = ERROR;

        /* ���������� ������
         * ���������� ������ �� ����������� ��������������, ������� ����������
         * ����� ������� ����� RS-232, ������ ��������� �� ������ � �������
         * ����� */
        ( void ) rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,
                             RS232_BITS_8, RS232_CTS_RTS_NOUSE );
        rs232_str_out( test_str_err, sizeof( test_str_err ) - 1U );

    }
    
    if ( 0U != ( MPM_LOADER_OUT_PTR->test_result & TEST_RESULT_ERR_OTHER ) ) {
    /* ���������� ������ ������ */
        rs232_str_out( test_str_cerr, sizeof( test_str_cerr ) - 1U );           /*  "\r\n���������� ������ � ������ ����������"  */
    }
    
    for ( i = 0; i < 1000000U; i++ ) {
    /* ����� ����� ����������� ������ RS-232 */
        ;
    }
    
    rs232_close();
    timer_close();
    
    irq_reset_entry();

    return err;
}

void test_start_wdt_end( void )
{
    rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );               // "�����"
}
