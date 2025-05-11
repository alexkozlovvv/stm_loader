#ifndef __TEST_MEM_H__
#define __TEST_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"

/* Подпрограмма контроля ОЗУ */
LOADER_FUNC void test_start_ram(
    uint_t mode );

/* Подпрограмма контроля ПЗУ */
LOADER_FUNC void test_start_rom(
    uint_t mode );

/* Подпрограмма контроля ЦПУ */
LOADER_FUNC void test_start_cpu(
    uint_t mode );

/* Подпрограмма контроля сторожевого таймера */
LOADER_FUNC error_t test_start_wdt(
    uint_t mode );
LOADER_FUNC void test_start_wdt_end( void );

/* Подпрограмма тестирования области ОЗУ */
LOADER_FUNC error_t test_ram(
    uint32_t beg_address, uint32_t end_address );
LOADER_FUNC static error_t test_ram_forward_pass(
    uint32_t ptrn_c, uint32_t ptrn_w,
    uint32_t beg_address, uint32_t end_address,
    uint32_t mode );
LOADER_FUNC static error_t test_ram_back_pass(
    uint32_t ptrn_c, uint32_t ptrn_w,
    uint32_t beg_address, uint32_t end_address,
    uint32_t mode );
LOADER_FUNC static error_t test_ram_pass(
    uint32_t beg_address, uint32_t end_address );

/* Подпрограмма тестирования процессора */
LOADER_FUNC error_t test_cpu( void );
LOADER_FUNC static error_t test_cpu_pass( void );

/* Подпрограмма тестирования области ПЗУ */
LOADER_FUNC error_t test_rom ( 
    uint32_t mem_arr );
LOADER_FUNC error_t test_rom_pass(
    const uint8_t data[], sz_t data_size, uint32_t csum );

LOADER_FUNC void test_rom_csum( 
    uint32_t addr, uint32_t size, uint32_t *cs );

LOADER_FUNC error_t test_adc( void );

LOADER_FUNC error_t test_adc_pass( void );

#endif /* __TEST_MEM_H__ */
