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

/* Адреса областей тестирования ОЗУ */
#define TEST_RAM_START      0x10001000
#define TEST_RAM_END        0x1000C000
#define TEST_SRAM_START     0x20000000
#define TEST_SRAM_END       0x20020000

/* Информационные сообщения */
static LOADER_CONST const char_t test_str_test_sram[] =
    "\r\nТест внутреннего ОЗУ (SRAM)           ";
static LOADER_CONST const char_t test_str_test_ccmram[] =
    "\r\nТест внутреннего ОЗУ (CCM)            ";
static LOADER_CONST const char_t test_str_test_cpu[] =
    "\r\nТест ЦПУ                              ";
static LOADER_CONST const char_t test_str_test_wdt[] =
    "\r\nТест WDT                              ";
static LOADER_CONST const char_t test_str_test_srom[] =
    "\r\nТест ПЗУ программы нач. инициализации ";
static LOADER_CONST const char_t test_str_test_urom[] =
    "\r\nТест ПЗУ пользователя                 ";
static LOADER_CONST const char_t test_str_test_adc[] =
    "\r\nТест АЦП                              ";
static LOADER_CONST const char_t test_str_err[] =
    "Отказ";
static LOADER_CONST const char_t test_str_ok[] =
    "Успех";
static LOADER_CONST const char_t test_str_skip[] =
    "Пропущен";
static LOADER_CONST const char_t test_str_enter[] =
    "\r\n";
static LOADER_CONST const char_t test_str_cerr[] =
    "\r\n\r\nОбнаружены прочие ошибки в работе загрузчика";
static LOADER_CONST const char_t reset[] =
    "\r\n\r\n=======Новый цикл========\r\n\r\n";

/* Подпрограмма контроля ОЗУ
 * (in) mode - режим:
 *             TEST_MODE_START - стартовый контроль
 *             TEST_MODE_FULL  - полный контроль
 * Примечание:
 *  1. В стартовых тестах и в технологическом режиме тестируется как 
 *     SRAM, так и CCM микроконтроллера;
 *  2. Тестируемая область:
 *     SRAM - 0x20000000 - 0x20020000 ( 128 кБ );
 *     CCM  - 0x10001000 - 0x1000C000 ( 44 кБ )
 */
void test_start_ram(
    uint_t mode )
{
    uint32_t        test_result;

    /* Контроль SRAM
       Первоначальный загрузчик не использует данную область памяти */
    
    rs232_str_out( test_str_test_sram, ( sizeof( test_str_test_sram ) - 1U ) );        // "\r\nТест внутреннего ОЗУ (SRAM)
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* Режим стартового контроля. Обнаружены ошибки */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );              // "Пропущен"
    } else {
        /* Режим полного контроля */
        /* Режим стартового контроля. Ошибок не обнаружено */
        if ( NO_ERROR != test_ram( TEST_SRAM_START, TEST_SRAM_END )
        ) {
            /* Обнаружены ошибки */
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_SRAM;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );            // "Отказ"
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );              // "Успех"
        }
    }

    /* Контроль CCM */
    /* В CCM RAM находятся данные и стек первоначального загрузчика */
    
    rs232_str_out( test_str_test_ccmram,                                               // "\r\nТест внутреннего ОЗУ (CCM)
                  ( sizeof( test_str_test_ccmram ) - 1U ) );
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* Режим стартового контроля. Обнаружены ошибки */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );              // "Пропущен"
    } else {
        /* Режим полного контроля */
        /* Режим стартового контроля. Ошибок не обнаружено */
        if ( NO_ERROR != test_ram( TEST_RAM_START, TEST_RAM_END )
        ) {
            /* Обнаружены ошибки */
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_CCM;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );            // "Отказ"
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );              // "Успех"
        }
    }
}

/* Подпрограмма контроля ПЗУ
 * (in) mode - режим:
 *             TEST_MODE_START - стартовый контроль
 *             TEST_MODE_FULL  - полный контроль
 */
void test_start_rom(
    uint_t mode )
{
    uint32_t        test_result;

    /* Конроль ПЗУ (область первоначального загрузчика) */
    rs232_str_out( test_str_test_srom, ( sizeof( test_str_test_srom ) - 1U ) );         /* "\r\nТест ПЗУ LOADER */
    test_result = MPM_LOADER_OUT_PTR->test_result;                                     
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* Режим стартового контроля. Обнаружены ошибки */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
    } else {
        /* Режим полного контроля */
        /* Режим стартового контроля. Ошибок не обнаружено */
        if ( NO_ERROR != test_rom( SROM_MEM_ARR ) ) {
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_SROM;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );
        }
    }

    /* Конроль ПЗУ (область пользователя) */
    rs232_str_out( test_str_test_urom, ( sizeof( test_str_test_urom ) - 1U ) );
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* Режим стартового контроля. Обнаружены ошибки */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
    } else {
        /* Режим полного контроля */
        /* Режим стартового контроля. Ошибок не обнаружено */
        if ( NO_ERROR != test_rom( UROM_MEM_ARR ) ) {
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_UROM;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );
        }
    }
 }

/* Подпрограмма контроля ЦПУ
 * (in) mode - режим:
 *             TEST_MODE_START - стартовый контроль
 *             TEST_MODE_FULL  - полный контроль
 */
void test_start_cpu(
    uint_t mode )
{
    uint32_t        test_result;

    rs232_str_out( test_str_test_cpu, ( sizeof( test_str_test_cpu ) - 1U ) );       /* "\r\nТест ЦПУ */
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* Режим стартового контроля. Обнаружены ошибки */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
    } else {
        /* Режим полного контроля */
        /* Режим стартового контроля. Ошибок не обнаружено */
        if ( NO_ERROR != test_cpu() ) {
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_CPU;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );
        }
    }
}

/* Подпрограмма контроля АЦП
 * (in) mode - режим:
 *             TEST_MODE_START - стартовый контроль
 *             TEST_MODE_FULL  - полный контроль
 */
void test_start_adc(
    uint_t mode )
{
    uint32_t        test_result;

    /* Конроль ПЗУ (область первоначального загрузчика) */
    rs232_str_out( test_str_test_adc, ( sizeof( test_str_test_adc ) - 1U ) );        /* "\r\nТест ПЗУ LOADER */
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode )
        && ( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) )
    ) {
        /* Режим стартового контроля. Обнаружены ошибки */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
    } else {
        /* Режим полного контроля */
        /* Режим стартового контроля. Ошибок не обнаружено */
        
        if ( NO_ERROR != test_adc() ) {
            MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_ADC;
            rs232_str_out( test_str_err, ( sizeof( test_str_err ) - 1U ) );
        } else {
            rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );
        }
    }
 }

/* Подпрограмма контроля WDT
 * (in) mode - режим:
 *             TEST_MODE_START - стартовый контроль
 *             TEST_MODE_FULL  - полный контроль
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 *  ERROR           - Ошибка
 */
error_t test_start_wdt(
    uint_t mode )
{

    volatile uint32_t flag_wdt_timeout;  /* Флаг истечения лимита времени контроля WDT */
    uint32_t        test_result;
    error_t         err;
    uint32_t        i;

    rs232_str_out( test_str_test_wdt, ( sizeof( test_str_test_wdt ) - 1U ) );
    test_result = MPM_LOADER_OUT_PTR->test_result;
    if (   ( TEST_MODE_START == mode ) 
        && (( 0U != ( test_result & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) ) ))
    ) {
        /* Режим стартового контроля. Обнаружены ошибки */
        rs232_str_out( test_str_skip, ( sizeof( test_str_skip ) - 1U ) );
        rs232_str_out( test_str_enter, ( sizeof( test_str_enter ) - 1U ) );
        err = NO_ACTION;

    } else {
        /* Режим полного контроля */
        /* Режим стартового контроля. Ошибок не обнаружено */
        
        for ( i = 0; i < 1000000U; i++ ) {
            /* Пауза перед завершением работы RS-232 */
            ;
        }
        rs232_close();
        /* Остановка таймера */
        for ( i = 0; i < 1000000U; i++ ) {
            /* Пауза после завершения работы RS-232 */
            ;
        }
        timer_close();
        /* Cброс признака окончания контроля сторожевого таймера */
        flag_wdt_timeout = 0U;
        /* Настройка и пуск таймера на период срабатывания 200 мс,
           без выдачи сигнала сброс в сторожевой таймер */
        timer_init( TIME_STEP_200, TIMER_TEST_WDT, &flag_wdt_timeout );
        /* Ожидание признака окончания контроля сторожевого таймера */
        while ( flag_wdt_timeout != 1U ) {
            ;
        }
        /* Микропроцессор не был сброшен за 200 мс - отказ WDT */
        MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_WDT;
        err = ERROR;

        /* Обнаружены ошибки
         * Сторожевой таймер не пересбросил микропроцессор, поэтому необходимо
         * снова открыть канал RS-232, выдать сообщение об ошибке и закрыть
         * канал */
        ( void ) rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,
                             RS232_BITS_8, RS232_CTS_RTS_NOUSE );
        rs232_str_out( test_str_err, sizeof( test_str_err ) - 1U );

    }
    
    if ( 0U != ( MPM_LOADER_OUT_PTR->test_result & TEST_RESULT_ERR_OTHER ) ) {
    /* Обнаружены прочие ошибки */
        rs232_str_out( test_str_cerr, sizeof( test_str_cerr ) - 1U );           /*  "\r\nОбнаружены ошибки в работе загрузчика"  */
    }
    
    for ( i = 0; i < 1000000U; i++ ) {
    /* Пауза перед завершением работы RS-232 */
        ;
    }
    
    rs232_close();
    timer_close();
    
    irq_reset_entry();

    return err;
}

void test_start_wdt_end( void )
{
    rs232_str_out( test_str_ok, ( sizeof( test_str_ok ) - 1U ) );               // "Успех"
}
