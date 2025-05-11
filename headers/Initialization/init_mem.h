#ifndef __INIT_MEM_H__
#define __INIT_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"

/* ��������� �������������� ��������� ��������� ���������� */
LOADER_FUNC void un_init_hardware( void );

/* ������������ ������������� ���������� */
LOADER_FUNC void init_hardware( void );

/* ������������ ������������� �������� �� */
LOADER_FUNC void init_software( void );

/* ������������ ������������� ������ (RST_CLK, EEPROM) */
static LOADER_FUNC void init_clk( void );

/* ������������ ������������� PORTx (������ ���������� �������� �/��) */
static LOADER_FUNC void init_gpio( void );

/* ������������ ������������� ������ �� Reset (RST_CLK, EEPROM) */
static LOADER_FUNC void un_init_clk( void );

/* ������������ ������������� PORTx �� Reset (������ ���������� �������� �/��) */
static LOADER_FUNC void un_init_gpio( void );

#endif /* __INIT_MEM_H__ */
