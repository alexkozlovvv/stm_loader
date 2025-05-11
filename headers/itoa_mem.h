#ifndef __ITOA_MEM_H__
#define __ITOA_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"

/* ������������ �������������� ������ 32-� �������� �����
 * � ������ */
LOADER_FUNC error_t i32toa(
    int32_t value, char_t str[], uint_t str_len, int32_t base,
    uint_t *ptr_len );

/* ������������ �������������� ������������ ������ 32-� �������� �����
 * � ������ */
LOADER_FUNC error_t u32toa(
    uint32_t value, char_t str[], uint_t str_len, uint_t base,
    uint_t *ptr_len );

#endif /* __ITOA_MEM_H__ */
