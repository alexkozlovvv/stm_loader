#ifndef __HEXLOAD_MEM_H__
#define __HEXLOAD_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"
#include "hexload.h"

/* ������������ ���������� ������� �� ���-����� � ��������������� ��� �
 * ����������������� ����� */
static LOADER_FUNC error_t hex_parse_digit(
    uint8_t *dest, struct tag_hex_record *ptr_record );

/* ������������ ���������� ����� (2 �������) �� ���-����� � ���������������
 * ��� � ����� */
static LOADER_FUNC error_t hex_parse_byte(
    uint8_t *dest, struct tag_hex_record *ptr_record );

/* ������������ ���������� ����� (4 �����) �� ���-����� � ���������������
 * ��� � ����� */
static LOADER_FUNC error_t hex_parse_word(
    uint32_t *dest, struct tag_hex_record *ptr_record, uint32_t flag_32 );

/* ������������ ���������� ����� ������ ��������� �������, ������������� �
 * ������ � ��� �� ��������� ������ */
static LOADER_FUNC error_t hex_parse_block(
    volatile uint8_t dest[], sz_t size, struct tag_hex_record *ptr_record );

LOADER_FUNC uint32_t u32_swap( uint32_t data );

#endif /* __HEXLOAD_MEM_H__ */
