#ifndef __TEST_H__
#define __TEST_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Режим контроля */
/* Стартовый контроль */
#define TEST_MODE_START         ( 0U )
/* Полное тестирование */
#define TEST_MODE_FULL          ( 1U )

/* Подпрограмма контроля ОЗУ
 * (in) mode - режим контроля (стартовый или полный контроль)
 */
extern LOADER_FUNC void test_start_ram(
    uint_t mode );

/* Подпрограмма контроля ПЗУ
 * (in) mode - режим контроля (стартовый или полный контроль)
 */
extern LOADER_FUNC void test_start_rom(
    uint_t mode );

/* Подпрограмма контроля ЦПУ
 * (in) mode - режим контроля (стартовый или полный контроль)
 */
extern LOADER_FUNC void test_start_cpu(
    uint_t mode );

/* Подпрограмма контроля сторожевого таймреа
 * (in) mode - режим контроля (стартовый или полный контроль)
 */
extern LOADER_FUNC error_t test_start_wdt(
    uint_t mode );
extern LOADER_FUNC void test_start_wdt_end( void );

/* Подпрограмма тестирования области ОЗУ
 * (in)  - начальный адрес области ОЗУ
 * (in)  - конечный адрес области ОЗУ
 * (out) - функция возвращает код ошибки
 */
extern LOADER_FUNC error_t test_ram(
    uint32_t beg_address, uint32_t end_address );

/* Подпрограмма тестирования процессора */
extern LOADER_FUNC error_t test_cpu( void );

/* Подпрограмма тестирования области ПЗУ */
extern LOADER_FUNC error_t test_rom ( 
    uint32_t mem_arr );

/* Подпрограмма тестирования АЦП */
extern LOADER_FUNC void test_start_adc(
    uint_t mode );

extern LOADER_FUNC error_t test_get_adc(
    uint32_t pos);

extern LOADER_FUNC void test_get_temp (void);

/* Подпрограмма подсчета и проверки контрольной суммы области ПЗУ
 * (in) data      - указатель на область ПЗУ
 * (in) data_size - размер области
 * (in) csum      - контрольная сумма, записанная в ПЗУ
 * Результат:
 *  NO_ERROR      - Успешное выполнение
 *  ERROR         - Ошибка
 */
extern LOADER_FUNC error_t test_rom_pass(
    const uint8_t data[], sz_t data_size, uint32_t csum );

extern LOADER_FUNC void test_rom_csum( 
    uint32_t addr, uint32_t size, uint32_t *cs );

#ifdef __cplusplus
}
#endif

#endif /* __TEST_H__ */

