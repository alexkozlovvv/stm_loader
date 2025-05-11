#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"
#include "STM32F407.h"
#include "loader.h"
#include "itoa.h"
#include "test.h"
#include "test_mem.h"
#include "timer.h"
#include "hexload.h"
#include "rs232.h"
#include "init.h"
#include "irq.h"

#include "techn_reg.h"
/* ������� �������������� ��������������� */
#include "techn_reg_mem.h"

/* ��������� ������ */
#define CMD_NUMBER         ( 7U ) /* ���������� ������ */
#define MAX_CMD_LEN       ( 10U ) /* ������������ ������ ������� */


/* ��������� �� ��������� ������ */
extern image_header_t  *main_ptr_image_header;

/* �������������� ��������� */
static LOADER_CONST const char_t tech_str_bios[] =
    "\r\n\r\n������ ��-1 ����.468332.122"
    "\r\n��������� ��������� ������������� (���) ����.00461-01"
    "\r\n������ ";
static LOADER_CONST const char_t tech_str_cs_bios[] =
    ", ����������� ����� = ";
static LOADER_CONST const char_t tech_str_head[] =
    "\r\n��������������� �����\r\n"
    "\r\n��� ���������� �������� ������� ������� ��� \"�������\":";
static LOADER_CONST const char_t tech_str_help[] =
    "\r\n\r\n������ - ������ ���������� � ������ � ����������� ������"
    "\r\n�������� - �������� ���-����� � ���"
    "\r\n������ - ������ ������������ ���-�����"
    "\r\n����� - ����� �� ���������������� ������"
    "\r\n�������� - ���������� �������� ���, ���, ���, ���"
    "\r\n������� - ���������� � ��������";
static LOADER_CONST const char_t tech_str_load[] =
    "\r\n\r\n�������� ����� ������� ���...\r\n";
static LOADER_CONST const char_t tech_str_load_err[] =
    "\r\n�������� ����� ������";
static LOADER_CONST const char_t tech_str_load_ok[] =
    "\r\n�������� ����� �����";
static LOADER_CONST const char_t tech_str_unknown[] =
    "\r\n\r\n����������� �������";
static LOADER_CONST const char_t tech_str_enter_point[] =
    "\r\n\r\n.";
static LOADER_CONST const char_t tech_str_empty[] =
    "\r\n";
static LOADER_CONST const char_t tech_str_point[] =
    ".";
static LOADER_CONST const char_t tech_str_0x[] =
    "0x";
static LOADER_CONST const char_t tech_str_cerr[] =
    "\r\n\r\n���������� ������ ��� ������ � ��������";
static LOADER_CONST const char_t single_str_point[] = 
    "\r\n.";

static LOADER_CONST const char_t input_command[] = 
    "\r\n�������: ";
static LOADER_CONST const char_t index_of_command[] = 
    "\r\n������ �������: ";
static LOADER_CONST const char_t symbol[] = 
    "\r\n������: ";
static LOADER_CONST const char_t space[] = 
    " ";

static LOADER_CONST const char_t commands_list[ CMD_NUMBER ][ MAX_CMD_LEN ] = {
"������",       /* ����� ���������� � ��������� � ����������� ����� */
"��������",     /* �������� ���-����� � ���. ����� ����� ����� ����������� � ���������� entry_hex */
"������",       /* ������ ������������ ���-����� */
"�����",        /* ����� �� ���������������� ������ */
"��������",     /* ������ ������������ ���, ���, ���, ��� */
"�������",      /* ����� ������� ��������� ������ */
"\r"            /* (Enter) ������� ������� ����������� ����� ������� �� ����� ������ */
};

/* ������������ �������� */
static void tech_stop( void )
{
    ( void ) kern_irq_disable_interrupt();
    for ( ; ; ) {
#if ( 0 == EBMK_DEBUG )   /* Release */
        ;
#elif ( 1 == EBMK_DEBUG ) /* Debug */
        BREAKPOINT;
#else
  #error "Unknown EBMK_DEBUG"
#endif
    }
}

/* ������������ ���������������� �������� */
error_t tech_monitor( void )
{
    uint32_t        entry_hex;                /* ����� ����� ����� � ���������
                                                 ������������ */
    uint_t          to_exit;                  /* ������� ���������� ������ */
    error_t         err;                      /* ��� ������ */
    char_t          command_buf[MAX_CMD_LEN]; /* ����� �������� ������� ������� */
    int32_t         index;                 /* ������� */

    /* ������� ����� ����� �� ��������� */
    entry_hex = ( uint32_t )( &init_startup );

    /* ������������� ������ RS-232 � ��������������� ������ */
    if ( NO_ERROR != rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,
                                 RS232_BITS_8, RS232_CTS_RTS_NOUSE )
    ) {
        /* ������������ ������ */
        tech_stop();
    }

    /* ����� ��������� � ������������ ������ ������� */
    err = tech_str_out( tech_str_head, sizeof( tech_str_head ) - 1U );

    /* ����� ������� ����������� ����� ������� */
    rs232_puts( tech_str_enter_point );
    
    to_exit = 0U;
    
    /* ���� ������ � ���������� ������ */
    while ( 0U == to_exit ) {
        
        /* ������ ������� */
		rs232_get_command(command_buf, MAX_CMD_LEN);

        /* ������������� ���������� ������� */
		index = find_index_of_command(commands_list, CMD_NUMBER, command_buf);

        switch(index){

            /* ������� "������" */
            case(0): {
                
                err = NO_ERROR;
                /* ����� ��������� ��������������� ���������� */
                if ( NO_ERROR != tech_str_out(
                                    tech_str_bios,
                                    sizeof( tech_str_bios ) - 1U )
                ) {
                    err = ERROR;
                /* ����� ������ ������� ������ */
                } else if ( NO_ERROR != tech_num_out( VERSION, 10U ) ) {
                    err = ERROR;
                } else if ( NO_ERROR != tech_str_out(                           
                                            tech_str_point,
                                            sizeof( tech_str_point ) - 1U ) ) {
                    err = ERROR;
                /* ����� ������ ������� ������ */
                } else if ( NO_ERROR != tech_num_out( RELEASE, 10U ) ) {
                    err = ERROR;
                /* ����� ����������� ����� */
                } else if ( NO_ERROR != tech_str_out(
                                            tech_str_cs_bios,
                                            sizeof( tech_str_cs_bios ) - 1U )
                ) {
                    err = ERROR;
                } else if ( NO_ERROR != tech_str_out(
                                            tech_str_0x,
                                            sizeof( tech_str_0x ) - 1U )
                ) {
                    err = ERROR;
                } else if ( NO_ERROR != tech_num_out( CS_LOADER, 16U ) ) {
                    err = ERROR;
                }
                /* �������� ������� ������ ��� ������ � �������� */
                if (NO_ERROR != err) {                
                    ( void ) rs232_puts( tech_str_cerr );
                }                    
                ( void ) rs232_puts( tech_str_enter_point );
                break;
            }
            
            /* ������� "��������" */
            case(1): {
                
                err = NO_ERROR;
                if ( NO_ERROR != tech_str_out(
                                    tech_str_load,
                                    sizeof( tech_str_load ) - 1U )
                ) {
                    err = ERROR;
                /* ����� ���������� �������� ���-����� � ��� */    
                } else if ( NO_ERROR != hex_file_load( &entry_hex )
                ) {
                    /* � ������ ������ ������������ ����� ����� �� ��������� */
                    entry_hex = ( uint32_t )( &init_startup );
                    if ( NO_ERROR != tech_str_out(
                                        tech_str_load_err,
                                        sizeof( tech_str_load_err ) - 1U )
                    ) {
                        err = ERROR;
                    }
                } else if ( NO_ERROR != tech_str_out(
                                            tech_str_load_ok,
                                            sizeof( tech_str_load_ok ) - 1U )
                ) {
                    err = ERROR;
                } 
                if (NO_ERROR != err) {                
                    ( void ) rs232_puts( tech_str_cerr );
                }                    
                ( void ) rs232_puts( tech_str_enter_point );
                break;
            }
            
            /* ������� "������" */
            case(2): {
                
                /* ����� ����� ����������� ������ RS-232 */
                delay_ms( 100U );                
                /* ����������� ������ ���������������� ������ RS-232 */
                rs232_close();
                /* ��������� �������, ������� ����������� WDT */
                timer_close();
                /* B������������e ��������� ��������� ���������� */
                un_init_hardware();
                /* �������� ���������� ��������� ������������ ��� 
                   ������������ ������ ���������� */
                ( * ( void (*)( void ) ) entry_hex )();
            }
            
            /* ������� "�����" */
            case(3): {
                
                to_exit = 1U;
                break;
            }
            
            /* ������� "��������" */
            case(4): {
                
                err = NO_ERROR;
                if ( NO_ERROR != tech_str_out(
                                    tech_str_empty,
                                    sizeof( tech_str_empty ) - 1U )
                ) {
                    err = ERROR;
                }
                test_start_ram( TEST_MODE_FULL );
                test_start_cpu( TEST_MODE_FULL );
                test_start_rom( TEST_MODE_FULL );
                test_start_adc( TEST_MODE_FULL );
                test_get_temp();
                
                if (NO_ERROR != err) {                
                    ( void ) rs232_puts( tech_str_cerr );
                }                    
                ( void ) rs232_puts( tech_str_enter_point );
                break;
            }
            
            /* ������� "�������" */
            case(5): {

                err = NO_ERROR;
                if ( NO_ERROR != tech_str_out(
                                    tech_str_help,
                                    sizeof( tech_str_help ) - 1U )
                ) {
                    err = ERROR;
                }                 
                if (NO_ERROR != err) {                
                    ( void ) rs232_puts( tech_str_cerr );
                }                    
                ( void ) rs232_puts( tech_str_enter_point );
                break;
            }
            
            /* ������� "\r" (Enter) */
            case(6): {
                
                ( void ) rs232_puts( single_str_point );
                
                break;
            }
            
            /* ����������� ������� */
            default: {
                
                err = NO_ERROR;
                if ( NO_ERROR != tech_str_out(
                                    tech_str_unknown,
                                    sizeof( tech_str_unknown ) - 1U )
                ) {
                    err = ERROR;
                } 
                if (NO_ERROR != err) {                
                    ( void ) rs232_puts( tech_str_cerr );
                }                    
                ( void ) rs232_puts( tech_str_enter_point );
                break;
            }
        }
    }
    
    /* ����� ����� ����������� ������ RS-232 */
    delay_ms( 100U );               
    /* ����������� ������ ���������������� ������ RS-232 */
    rs232_close();
    
    return err;
}

/* ������������ ����������� ������� ��������� �������
 * ������� ���������: 
 * cmd_array (in) - ������ ������
 * size (in)      - ����������� ����� ������� (���������� ������)
 * word (in)      - ��������� �� ���������� �������
 * ���������:
 *  0 - 7         - ������ ������� �� �������
 *  -1            - ����������� �������
 */
int32_t find_index_of_command(const char_t cmd_array[][ MAX_CMD_LEN ], uint_t size, 
    const char_t* word )
{
	int32_t j,i;
	int32_t len = 0;  
    
    /* ������� ����� ���������� ������� */
	while(word[len] != '\0'){
		len++;
	}
	
    /* ������������ ��������� ������ ������� �� ������� � ���������� ������ */
	for(i = 0;i < size; i++){
		for(j = 0; cmd_array[i][j] && word[j]; j++){
			if(cmd_array[i][j] != word[j]){
				break;
			}
		}
		/* ���� ��� ������� ������� � ������������ ����������� 
           �������� �������� */
		if(j == len && !cmd_array[i][j]){
			return i;
		}
	}
	//������� �� �������
	return -1;
}

/* ������������ ������ ������
 * (in) str - ��������� �� ������
 * (in) len - ����� ������
 */
static error_t tech_str_out(
    const char_t str[], uint_t len )
{
    error_t         err;

    if ( len != rs232_puts( str ) ) {
        /* ����� �������� ���� �� ������������� ������������ �����
         * �������� � ������ */
        err = ERROR;
    } else {
        err = NO_ERROR;
    }

    return err;
}

/* ������������ ������ �����
 * (in) num  - �����
 * (in) base - ��������� ������� ���������
 */
static error_t tech_num_out(
    uint32_t num, uint_t base )
{
    char_t          str[ 129 ];
    uint_t          len;
    error_t         err;

    if ( NO_ERROR != u32toa( num, str, sizeof( str ), base, &len ) ) {
        /* ������ �������������� ����� � ������ */
        err = ERROR;
    } else {
        /* ������ */
        err = tech_str_out( str, len );
    }

    return err;
}
