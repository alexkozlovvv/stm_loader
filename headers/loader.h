#ifndef __LOADER_H__
#define __LOADER_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Типы устройств */
#define LOADER_EUMW                   0x9U

/* Типы процессора */
#define LOADER_XILINX                 0x7U
#define LOADER_STM32                  0x8U

/* Типы канала */    
#define LOADER_CHAN_MODEL             0x0U
#define LOADER_CHAN_IND               0x1U

/* Максимальное число образов загрузки */    
#define LOADER_IMG_MAX                0x4U
    
/* Минимальное число образов загрузки */    
#define LOADER_IMG_MIN                0x1U

    
/* Версия и редакция первоначального загрузчика */
#define VERSION                 ( 1UL )
#define RELEASE                 ( 0UL )    
    
/* Тип области ПЗУ */
#define SROM_MEM_ARR    0U
#define CROM_MEM_ARR    1U    
#define UROM_MEM_ARR    2U

/* Начальные адреса и размеры областей ПЗУ для
 * первоначального загрузчика (SROM), заголовка 
 * рабочей программы (CROM), рабочей программы (UROM) */
#define SROM_START      ROM_ADDRESS
#define SROM_SIZE       ( VECTOR_SIZE + LOADER_TEXT_SIZE )
#define CROM_START      ( ROM_ADDRESS + SROM_SIZE )
#define CROM_SIZE       0x1000U
#define UROM_START      ( ROM_ADDRESS + ( SROM_SIZE + CROM_SIZE ) )
#define UROM_SIZE       ( ROM_SIZE - ( SROM_SIZE + CROM_SIZE ) )

/* Контрольная сумма первоначального загрузчика */
#define CS_LOADER_ADDR  ( SROM_START + SROM_SIZE - 4U )
#define CS_LOADER       ( *( volatile uint32_t * )( CS_LOADER_ADDR ) )
/* Контрольная сумма бортового ПО */
#define CS_USER_ADDR    ( UROM_START - 4U )
#define CS_USER         ( *( volatile uint32_t * )( CS_USER_ADDR ) )

/* Тип данных - заголовок образа */
typedef struct tag_image_header {
    uint32_t        location;         /* Номер места соответствующего образа ПО */
    uint32_t        offs;             /* Смещение образа ПО относительно начала области CROM */
    uint32_t        size;             /* Размер образа пользовательской программы */
    uint32_t        sects;            /* Количество секции в образе */
    uint32_t        entry_address;    /* Адрес точки входа в программу       */
    uint32_t        cs;               /* Контрольная сумма образа ПО */
} image_header_t;

typedef struct tag_load_header {
    uint32_t        module;         /* Тип устройства */
    uint32_t        chan;           /* Тип канала */
    uint32_t        proc;           /* Тип процессора */
    uint32_t        img_num;        /* Количество загружаемых образов */
} load_header_t;

/* Подпрограмма диспетчера первоначального загрузчика */
LOADER_FUNC int main( void );


#ifdef __cplusplus
}
#endif

#endif /* __LOADER_H__ */
