#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"

#include "test.h"

/* Задание местоположения идентификаторов */
#include "test_mem.h"

/* Настройки прохода */
#define TEST_RAM_CONTROL        ( 1U ) /* Проводить контроль */
#define TEST_RAM_WRITE          ( 2U ) /* Проводить запись   */


/* Подпрограмма тестирования ОЗУ
 * (in)  - начальный адрес области ОЗУ
 * (in)  - конечный адрес области ОЗУ
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 *  ERROR           - Ошибка
 */
error_t test_ram(
    uint32_t beg_address, uint32_t end_address )
{
    uint_t          cnt, cnt_err;
    error_t         err;

    if (   ( beg_address >= end_address )
        || ( 0U != ( beg_address & 0x3U ) )       /* проверяемая область должна содержать целое количество 32 разрядных слов */
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
                /* Два успешный прохода. Третий проход не делаем */
                cnt++;
            } else if (
                   ( 2U == cnt_err )
                && ( 2U == cnt )
            ) {
                /* Два не успешных прохода. Третий проход не делаем */
                cnt++;
            } else {
                /* Продолжаем контроль */
                ;
            }
        }

        err = ( 2U <= cnt_err ) ? ERROR : NO_ERROR;
    }

    return err;
}

/* Подпрограмма прохода вперед маршевого теста
 * (in) - шаблон для контроля
 * (in) - шаблон для записи
 * (in) - начальный адрес
 * (in) - конечный адрес
 * (in) - настройки прохода
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 *  ERROR           - Ошибка
 */
static error_t test_ram_forward_pass(
    uint32_t ptrn_c, uint32_t ptrn_w,
    uint32_t beg_address, uint32_t end_address,
    uint32_t mode )
{
    uint32_t        cur_address;     /* Текущий адрес в ОЗУ */
    uint32_t       *cadr;            /* Указатель на текущее слово */
    error_t         err;

    cur_address = beg_address;
    err = NO_ERROR;
    while ( NO_ERROR == err ) {

        cadr = ( uint32_t * ) cur_address;

        if ( 0U != ( mode & TEST_RAM_CONTROL ) ) {      /* Если режим контроля памяти */
            if ( *cadr != ptrn_c ) {
                err = ERROR;
            }
        }
        if ( 0U != ( mode & TEST_RAM_WRITE ) ) {        /* Если режим записи в память */
            *cadr = ptrn_w;
        }
        cur_address += 4U;
        if ( cur_address >= end_address ) {
            break;
        }
    }

    return err;
}

/* Подпрограмма прохода назад маршевого теста
 * (in) - шаблон для контроля
 * (in) - шаблон для записи
 * (in) - начальный адрес
 * (in) - конечный адрес
 * (in) - Настройки прохода
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 *  ERROR           - Ошибка
 */
static error_t test_ram_back_pass(
    uint32_t ptrn_c, uint32_t ptrn_w,
    uint32_t beg_address, uint32_t end_address,
    uint32_t mode )
{
    uint32_t        cur_address;     /* Текущий адрес в ОЗУ */
    uint32_t       *cadr;            /* Указатель на текущее слово */
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

/* Подпрограмма одного прохода теста ОЗУ
 * (in)  - начальный адрес области ОЗУ
 * (in)  - конечный адрес области ОЗУ
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 *  ERROR           - Ошибка
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
