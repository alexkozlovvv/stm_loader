#ifndef __LOADER_MEM_H__
#define __LOADER_MEM_H__ 1

#include "base_types.h"
#include "sections_bsp.h"
#include "loader.h"

/* Подпрограмма диспетчера первоначального загрузчика */
LOADER_FUNC int main( void );

/* Подпрограмма останова */
static LOADER_FUNC void main_stop( void );

/* Подпрограмма отработки первого включения */
static LOADER_FUNC void main_power_on( void );

/* Подпрограмма отработки последующих включений */
static LOADER_FUNC void main_reset( void );

/* Подпрограмма выхода в технологический монитор */
static LOADER_FUNC void main_exit_monitor( void );

/* Подпрограмма выхода в бортовую программу */
static LOADER_FUNC void main_exit_user( image_header_t* image_header );

/* Подпрограмма считывания заголовка образа ПО */
static LOADER_FUNC error_t main_read_image_header ( const uint32_t start_img_address,
    image_header_t *dest );

/* Подпрограмма считывания заголовка образа загрузки */
static LOADER_FUNC void  main_read_load_header (
    load_header_t *dest );

/* Подпрограмма считывания заголовка образа загрузки */
static LOADER_FUNC error_t main_locaton_detect(
    image_header_t *ptr_image_header );

#endif /* __LOADER_MEM_H__ */
