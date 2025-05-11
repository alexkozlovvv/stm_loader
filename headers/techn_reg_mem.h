#ifndef __TECH_REG_MEM_H__
#define __TECH_REG_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"

/* Подпрограмма технологического монитора */
LOADER_FUNC error_t tech_monitor( void );

/* Подпрограмма останова */
static LOADER_FUNC void tech_stop( void );

/* Подпрограмма выдачи строки */
static LOADER_FUNC error_t tech_str_out(
    const char_t str[], uint_t len );

/* Подпрограмма выдачи числа */
static LOADER_FUNC error_t tech_num_out(
    uint32_t num, uint_t base );

#endif /* __TECH_REG_MEM_H__ */
