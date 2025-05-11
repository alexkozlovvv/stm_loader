#ifndef __HEX_LOAD_H__
#define __HEX_LOAD_H__  1

#include "sections_bsp.h"

#include "base_types.h"

#define MAX_REC_LENG  256U

#ifdef __cplusplus
extern "C" {
#endif

/* ������ HEX ����� */
typedef struct tag_hex_record {
    uint8_t         buff[ MAX_REC_LENG ];   /* �����                    */
    uint_t          idx_read;               /* ������ ����������        */
    uint_t          idx_save;               /* ������ ����������        */
    uint32_t        csum;                   /* ����������� ����� ������ */
} hex_record_t;
    
/* ������������ �������� ���-����� � ��� ��� ����������
 *  (in)  ptr_entry - ��������� �� �������� ������ ����� ����� � ���������
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
extern LOADER_FUNC error_t hex_file_load(
    uint32_t *ptr_entry );

LOADER_FUNC uint32_t u32_swap( uint32_t data );

#ifdef __cplusplus
}
#endif

#endif  /* __HEX_LOAD_H__ */
