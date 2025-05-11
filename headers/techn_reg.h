#ifndef __TECHN_REG_H__
#define __TECHN_REG_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#define MAX_CMD_LEN      ( 10U ) /* Максимальный размер входной команды */

#ifdef __cplusplus
extern "C" {
#endif

/* Подпрограмма технологического монитора */
extern LOADER_FUNC error_t tech_monitor( void );
    
extern LOADER_FUNC int32_t find_index_of_command(const char_t cmd_array[][ MAX_CMD_LEN ], uint_t size, 
    const char_t* word );

#ifdef __cplusplus
}
#endif

#endif /* __TECHN_REG_H__ */

