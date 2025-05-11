#ifndef __LOADER_MEM_H__
#define __LOADER_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"
#include "loader.h"

/* ������������ ���������� ��������������� ���������� */
LOADER_FUNC int main( void );

/* ������������ �������� */
static LOADER_FUNC void main_stop( void );

/* ������������ ��������� ������� ��������� */
static LOADER_FUNC void main_power_on( void );

/* ������������ ��������� ����������� ��������� */
static LOADER_FUNC void main_reset( void );

/* ������������ ������ � ��������������� ������� */
static LOADER_FUNC void main_exit_monitor( void );

/* ������������ ������ � �������� ��������� */
static LOADER_FUNC void main_exit_user( image_header_t* image_header );

/* ������������ ���������� ��������� ������ �� */
static LOADER_FUNC error_t main_read_image_header ( const uint32_t start_img_address,
    image_header_t *dest );

/* ������������ ���������� ��������� ������ �������� */
static LOADER_FUNC void  main_read_load_header (
    load_header_t *dest );

/* ������������ ���������� ��������� ������ �������� */
static LOADER_FUNC error_t main_locaton_detect(
    image_header_t *ptr_image_header );

#endif /* __LOADER_MEM_H__ */
