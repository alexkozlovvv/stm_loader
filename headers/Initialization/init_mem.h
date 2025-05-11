#ifndef __INIT_MEM_H__
#define __INIT_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"

/* Программа восстановления начальной настройки аппаратуры */
LOADER_FUNC void un_init_hardware( void );

/* Подпрограмма инициализации аппаратуры */
LOADER_FUNC void init_hardware( void );

/* Подпрограмма инициализации базового ПО */
LOADER_FUNC void init_software( void );

/* Подпрограмма инициализации частот (RST_CLK, EEPROM) */
static LOADER_FUNC void init_clk( void );

/* Подпрограмма инициализации PORTx (модуль управления выводами м/сх) */
static LOADER_FUNC void init_gpio( void );

/* Подпрограмма инициализации частот по Reset (RST_CLK, EEPROM) */
static LOADER_FUNC void un_init_clk( void );

/* Подпрограмма инициализации PORTx по Reset (модуль управления выводами м/сх) */
static LOADER_FUNC void un_init_gpio( void );

#endif /* __INIT_MEM_H__ */
