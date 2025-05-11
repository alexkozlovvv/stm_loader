#ifndef __INIT_H__
#define __INIT_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Программа восстановления начальной настройки аппаратуры */
extern LOADER_FUNC void un_init_hardware( void );

#ifdef __cplusplus
}
#endif

#endif /* __INIT_H__ */
