#ifndef __SECTION_BSP_H__
#define __SECTION_BSP_H__ 1

#include "sections.h"

#ifdef VECTOR_CAST_SECTIONS

#define LVL0_FUNC
#define LVL0_CONST
#define LVL0_DATA

#define LVL1_FUNC
#define LVL1_CONST
#define LVL1_DATA
#define LVL1_DMABUF

#define LVL2_FUNC
#define LVL2_CONST

#define LOADER_FUNC
#define LOADER_CONST
#define LOADER_DATA

#define PROCESS_BSP_FUNC
#define PROCESS_BSP_CONST
#define PROCESS_BSP_DATA

#else

/* Заголовки описаний подпрограмм и данных базового ПО разных уровней */

/* ПО уровня 0 */
#define LVL0_FUNC   __attribute__((section(".lvl0_text"  )))
#define LVL0_CONST  __attribute__((section(".lvl0_rodata")))
#define LVL0_BSS_   __attribute__((section(".lvl0_bss"   ))) __attribute__((zero_init))
#define LVL0_DATA_  __attribute__((section(".lvl0_data"  )))
#define LVL0_DATA   LVL0_BSS_

/* ПО уровня 1 */
#define LVL1_FUNC   __attribute__((section(".lvl1_text"  )))
#define LVL1_CONST  __attribute__((section(".lvl1_rodata")))
#define LVL1_BSS_   __attribute__((section(".lvl1_bss"   ))) __attribute__((zero_init))
#define LVL1_DATA_  __attribute__((section(".lvl1_data"  )))
#define LVL1_DATA   LVL1_BSS_
#define LVL1_DMABUF __attribute__((section(".lvl1_dmabuf"))) __attribute__((zero_init))

/* ПО уровня 2 */
#define LVL2_FUNC   __attribute__((section(".lvl2_text"  )))
#define LVL2_CONST  __attribute__((section(".lvl2_rodata")))

/* ПО первоначального загрузчика */
#define LOADER_FUNC  __attribute__((section(".loader_text"   )))
#define LOADER_CONST __attribute__((section(".loader_rodata" )))
#define LOADER_BSS_  __attribute__((section(".loader_bss"    ))) __attribute__((zero_init))
#define LOADER_DATA_ __attribute__((section(".loader_data"   )))
#define LOADER_DATA  LOADER_BSS_

/* ПО уровня 3. Процесс базового ПО */
#define PROCESS_BSP_FUNC  __attribute__((section(".process_bsp_text"   )))
#define PROCESS_BSP_CONST __attribute__((section(".process_bsp_rodata" )))
#define PROCESS_BSP_BSS_  __attribute__((section(".process_bsp_bss"    ))) __attribute__((zero_init))
#define PROCESS_BSP_DATA_ __attribute__((section(".process_bsp_data"   )))
#define PROCESS_BSP_DATA  PROCESS_BSP_BSS_

#endif

#endif /* __SECTION_BSP_H__ */
