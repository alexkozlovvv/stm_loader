#ifndef __LOADER_H__
#define __LOADER_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ���� ��������� */
#define LOADER_EUMW                   0x9U

/* ���� ���������� */
#define LOADER_XILINX                 0x7U
#define LOADER_STM32                  0x8U

/* ���� ������ */    
#define LOADER_CHAN_MODEL             0x0U
#define LOADER_CHAN_IND               0x1U

/* ������������ ����� ������� �������� */    
#define LOADER_IMG_MAX                0x4U
    
/* ����������� ����� ������� �������� */    
#define LOADER_IMG_MIN                0x1U

    
/* ������ � �������� ��������������� ���������� */
#define VERSION                 ( 1UL )
#define RELEASE                 ( 0UL )    
    
/* ��� ������� ��� */
#define SROM_MEM_ARR    0U
#define CROM_MEM_ARR    1U    
#define UROM_MEM_ARR    2U

/* ��������� ������ � ������� �������� ��� ���
 * ��������������� ���������� (SROM), ��������� 
 * ������� ��������� (CROM), ������� ��������� (UROM) */
#define SROM_START      ROM_ADDRESS
#define SROM_SIZE       ( VECTOR_SIZE + LOADER_TEXT_SIZE )
#define CROM_START      ( ROM_ADDRESS + SROM_SIZE )
#define CROM_SIZE       0x1000U
#define UROM_START      ( ROM_ADDRESS + ( SROM_SIZE + CROM_SIZE ) )
#define UROM_SIZE       ( ROM_SIZE - ( SROM_SIZE + CROM_SIZE ) )

/* ����������� ����� ��������������� ���������� */
#define CS_LOADER_ADDR  ( SROM_START + SROM_SIZE - 4U )
#define CS_LOADER       ( *( volatile uint32_t * )( CS_LOADER_ADDR ) )
/* ����������� ����� ��������� �� */
#define CS_USER_ADDR    ( UROM_START - 4U )
#define CS_USER         ( *( volatile uint32_t * )( CS_USER_ADDR ) )

/* ��� ������ - ��������� ������ */
typedef struct tag_image_header {
    uint32_t        location;         /* ����� ����� ���������������� ������ �� */
    uint32_t        offs;             /* �������� ������ �� ������������ ������ ������� CROM */
    uint32_t        size;             /* ������ ������ ���������������� ��������� */
    uint32_t        sects;            /* ���������� ������ � ������ */
    uint32_t        entry_address;    /* ����� ����� ����� � ���������       */
    uint32_t        cs;               /* ����������� ����� ������ �� */
} image_header_t;

typedef struct tag_load_header {
    uint32_t        module;         /* ��� ����������                      */
    uint32_t        chan;           /* ��� ������ */
    uint32_t        proc;           /* ��� ���������� */
    uint32_t        img_num;        /* ���������� ����������� ������� */
} load_header_t;

/* ������������ ���������� ��������������� ���������� */
LOADER_FUNC int main( void );


#ifdef __cplusplus
}
#endif

#endif /* __LOADER_H__ */
