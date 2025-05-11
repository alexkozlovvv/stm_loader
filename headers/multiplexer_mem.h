#ifndef __MULT_MEM_H__
#define __MULT_MEM_H__ 1

#include "sections_bsp.h"
#include "base_types.h"
#include "eumw.h"

extern LOADER_FUNC error_t multiplexer_init(void);

extern LOADER_FUNC error_t multiplexer_set(uint32_t pos);

#endif /* __MULT_MEM_H__ */
