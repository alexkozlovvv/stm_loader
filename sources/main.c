#include "sections_bsp.h"
#include "base_types.h"
#include "os_def.h"
#include "eumw.h"
#include "STM32F407.h"
#include "loader.h"
#include "init.h"
#include "irq.h"
#include "rs232.h"
#include "timer.h"
#include "test.h"
#include "techn_reg.h"
#include "hexload.h"
#include "loader_mem.h"

/* ��������������� ������� ������� ��� */
#define MAIN_TECH_RK_KTA                     0x00000002U
#define MAIN_TECH_RK_BLCK_WDT                0x00000008U
#define MAIN_TECH_RK_KTA_AND_BLCK_WDT        0x0000000AU

/* ��������� ��� �������� ���������� ��������� */
#define MAIN_FIRST_0            ( 0x5555AAAAU )
#define MAIN_FIRST_1            ( 0xAAAA5555U )
#define MAIN_FIRST_2            ( 0xFFFF0000U )
#define MAIN_FIRST_3            ( 0x0000FFFFU )

/* �������������� ��������� */
static LOADER_CONST const char_t main_str_bios[] =
    "\r\n\r\n\r\n����.00461-01";
static LOADER_CONST const char_t main_str_ver[] =
    "\r\n������ ";
static LOADER_CONST const char_t main_str_cs_bios[] =
    ", ����������� ����� = ";
static LOADER_CONST const char_t main_str_cs_user[] =
    ", ����������� ����� = ";
static LOADER_CONST const char_t main_str_rst[] =
    "������� ������� = ";
static LOADER_CONST const char_t main_str_work[] =
    "\r\n\r\n������ ��������� �� ����� ";
static LOADER_CONST const char_t main_str_bad_head[] =    
    "\r\n������������ ���������� ��������� ��������� ��";
static LOADER_CONST const char_t main_str_empty[] =
    "\r\n�������� �� �� �������� � ���";
static LOADER_CONST const char_t main_str_enter[] =
    "\r\n";
static LOADER_CONST const char_t main_str_point[] =
    ".";
static LOADER_CONST const char_t main_str_0x[] =
    "0�";
static LOADER_CONST const char_t main_str_cerr[] =
    "\r\n\r\n���������� ������ � ������ ����������";
static LOADER_CONST const char_t main_str_mend[] =
    "\r\n��������������� ������� �������� ������";
static LOADER_CONST const char_t main_wdt_error[] =
    "\r\n���������� ������ ��� ������������ WDT";
static LOADER_CONST const char_t main_wdt_no_test[] =
    "\r\n�������� WDT ���� ���������";
static LOADER_CONST const char_t main_str_bad_ks_head[] =
    "\r\n������ �������� �� ��������� ������ ��������";
static LOADER_CONST const char_t main_rkp_pos_err[] =
    "\r\n������ ��� ��������� ��� ���� �����";
static LOADER_CONST const char_t main_str_no_user[] =
    "\r\n�� ������ ����� ��, ��������������� ��";
static LOADER_CONST const char_t main_str_locat[] =
    "\r\n��� ����� ";
static LOADER_CONST const char_t main_str_chan_0[] =
    " ������ ���";
static LOADER_CONST const char_t main_str_user_fail[] =    
    "\r\n���������� ������ � �������� �������� ���������� \r\n"
    "���������������� ���������";
static LOADER_CONST const char_t main_str_error_start_test[] =    
    "\r\n\r\n���������� ������ �������� ��� ���������\r\n";
static LOADER_CONST const char_t test_str_cerr[] =
    "\r\n\r\n���������� ������ ������ � ������ ����������";

/* ���������� ��� ���������� ������������ �������� � ������������ ������ ������. ������������ ��� ������� */
// const uint32_t cs __attribute__ ((at(0x0800BFFC)))= 0x232BF049;  

/* ����� ����� � ������������ ���������� ��������� ��������� �������������.
 * ������������ ������������� ��� ����������� ������ ������ 
 * (������� ��� ���������������), ���������� ���������� �������� �
 * ������� ������� ��������� � ���
 */
LOADER_FUNC int32_t main( void )
{
    uint32_t        rk_kta;        
    uint32_t        a0, a1, a2, a3;   /* ��������������� ���������� */
    
    /* ������������� ������ RS-232.
     * ��������� � ����������: ����������� �������� ��������, 
       ���� ���� ���, ����� ����� ������ 8 ���, 
       ���������� ���������� �������� ������ CTS/RTS */
    if ( NO_ERROR != rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,
                                 RS232_BITS_8, RS232_CTS_RTS_NOUSE )
    ) {
        main_stop();
    }
    
    /* ������������� ������� ������ WDT � ������� �����������. 
       ������ ������������ 10 �� */
    timer_init( TIME_STEP_10, TIMER_NORMAL, NULL );

    /* ������ �������� ������� ������ ����� � */
    rk_kta = IN_REG( STM32F407_GPIOA_PTR->idr );
    
    /* ��� ��������� �� ��� � ����. WDT � ��������� "������� �������" - ������� �
     * ��������������� �����, ����� - ������� � ������� ����� ������ */
    if ( (rk_kta & (MAIN_TECH_RK_KTA)) != 0U) {               
        
        /* ������ ���������������� �������� */
        main_exit_monitor();
        
    } else {
        
        /* ������������� � ���� ����������� ������� */
        timer_wdt_init();       
        
        /* ���������� �������� ���������� ��������� */
        a0 = MPM_LOADER_OUT_PTR->first[ 0U ];                   
        a1 = MPM_LOADER_OUT_PTR->first[ 1U ];
        a2 = MPM_LOADER_OUT_PTR->first[ 2U ];
        a3 = MPM_LOADER_OUT_PTR->first[ 3U ];

        /* �������� �������� ���������� ��������� */
        if (    ( a0 != MAIN_FIRST_0 ) || ( a1 != MAIN_FIRST_1 )
             || ( a2 != MAIN_FIRST_2 ) || ( a3 != MAIN_FIRST_3 )
        ) {
            /* ������ ��������� */ 
            main_power_on();
            
        } else {
            /* ���������� */        
            main_reset();
        }
    }
    return ERROR;
}

/* ������������ �������� */
static LOADER_FUNC void main_stop( void )
{
    ( void ) kern_irq_disable_interrupt();
    for ( ; ; ) {
#if ( 0 == EUMW_DEBUG ) /* Release */
        ;
#elif ( 1 == EUMW_DEBUG ) /* Debug */
        BREAKPOINT;
#else
  #error "Unknown EUMW_DEBUG"
#endif
    }
}

/* ������������ ��������� ������� ��������� */
static LOADER_FUNC void main_power_on( void )
{
    /* ����� ����������� ���������� �������� */
    MPM_LOADER_OUT_PTR->test_result = 0U;
    
    /* ����� �������� ������������ */
    MPM_LOADER_OUT_PTR->cnt_reset = 0U;
    
    /* ��������� �������� ���������� ��������� �� */
    MPM_LOADER_OUT_PTR->image_header_addr = 0U;

    /* ����� ������������ ������ ��������� ��������� ������������� */
    /* ����.00461-01 */
    rs232_str_out( main_str_bios, ( sizeof( main_str_bios ) - 1U ) );
    /* ����� ������ ��������� ��������� ������������� */
    /* ������ VERSION.RELEASE */
    rs232_str_out( main_str_ver, ( sizeof( main_str_ver ) - 1U ) );
    rs232_num_out( VERSION, 10U );
    rs232_str_out( main_str_point, ( sizeof( main_str_point ) - 1U ) );
    rs232_num_out( RELEASE, 10U );
    /* ����� ����������� ���� */
    /* , ����������� ����� = 0xCS_LOADER */ 
    rs232_str_out( main_str_cs_bios, ( sizeof( main_str_cs_bios ) - 1U ) );
    rs232_str_out( main_str_0x, ( sizeof( main_str_0x ) - 1U ) );
    rs232_num_out( CS_LOADER, 16U );
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );

    /* ��������� �������� ���������� ��������� */
    MPM_LOADER_OUT_PTR->first[ 0U ] = MAIN_FIRST_0;
    MPM_LOADER_OUT_PTR->first[ 1U ] = MAIN_FIRST_1;
    MPM_LOADER_OUT_PTR->first[ 2U ] = MAIN_FIRST_2;
    MPM_LOADER_OUT_PTR->first[ 3U ] = MAIN_FIRST_3;

    /* ------- ��������� �������� ------- */

    /* �������� ��� */
    test_start_ram( TEST_MODE_START );

    /* �������� ��� */
    test_start_cpu( TEST_MODE_START );

    /* ������� ��� */
    test_start_rom( TEST_MODE_START );
    
    /* ������� ��� */
    test_start_adc( TEST_MODE_START );
    
    /* �������� ����������� ������� */
    test_start_wdt( TEST_MODE_START );
    
}

/* ������������ ��������� ����������� ��������� */
static LOADER_FUNC void main_reset( void )
{
    image_header_t      image_header;  /* ��������� ��������� */
    
    /* ����������� �������� ������������ */
    if ( MAX_UINT32 != MPM_LOADER_OUT_PTR->cnt_reset ) {
        MPM_LOADER_OUT_PTR->cnt_reset++;
    }

    /* ��� ������ �����������, ���� �������� ����������� �������� ���� WDT
       (�� ���� ���� WDT ������� ��������), �� ���������� ������ ���������
       �� �������� ���������� ����� WDT */
    if ( MPM_LOADER_OUT_PTR->cnt_reset != 1U ) {
        /* �� ������ ���������� */
        ;
    } else if ( 0U != ( MPM_LOADER_OUT_PTR->test_result 
                        & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) )
    ) {
        /* ���������� ������ ��� ��������� ������� */
        ;
    } else {
        /* ������ ��������� �� �������� �������� WDT */
        test_start_wdt_end();
        
        if ( 0U != ( MPM_LOADER_OUT_PTR->test_result & TEST_RESULT_ERR_OTHER ) ) {
        /* ���������� ������ ������ */
            rs232_str_out( test_str_cerr, sizeof( test_str_cerr ) - 1U );           /*  "\r\n���������� ������ � ������ ����������"  */
        }
    }
    
    /* ����� ������������ ������ ��������������� ���������� */
    rs232_str_out( main_str_bios, ( sizeof( main_str_bios ) - 1U ) );

    /* ����� �������� ������������ */
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );
    rs232_str_out( main_str_rst, ( sizeof( main_str_rst ) - 1U ) );
    rs232_num_out( MPM_LOADER_OUT_PTR->cnt_reset, 10U );
    
    /* ����� ������ ��������������� ���������� */
    rs232_str_out( main_str_ver, ( sizeof( main_str_ver ) - 1U ) );          /*  "\r\n������ " */
    rs232_num_out( VERSION, 10U );                                           /* �������������� ����� � ������ � ����� ����� ������ �� RS */
    rs232_str_out( main_str_point, ( sizeof( main_str_point ) - 1U ) );      /* "." */
    rs232_num_out( RELEASE, 10U );                                           /* ����� ����� ������ */
    
    /* ����� ����������� ����� */
    rs232_str_out( main_str_cs_bios, ( sizeof( main_str_cs_bios ) - 1U ) );  /* ", ����������� ����� = " */
    rs232_str_out( main_str_0x, ( sizeof( main_str_0x ) - 1U ) );            /* "0x" */
    rs232_num_out( CS_LOADER, 16U );                                         /* ����������� ����� */
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );      /* "\r\n" */
    
    /* �������� ��������� ��������� �� */   
    if ( NO_ERROR != main_locaton_detect( &image_header ) ) {
        /* ������ ����������� ������������ ������ �� */
        /* ������� � ��������������� ����� */
        main_exit_monitor();
    } else {
        main_exit_user(&image_header);
    }
}

/* ������������ ������ � ��������������� ������� */
static LOADER_FUNC void main_exit_monitor( void )
{
    uint32_t        i;
    
    /* ����� ������������ ������ ��������� ��������� ������������� */
    rs232_str_out( main_str_bios, ( sizeof( main_str_bios ) - 1U ) );        /* "\r\n\r\n\r\n����.00461-01" */
    /* ����� ������ ��������������� ���������� */
    rs232_str_out( main_str_ver, ( sizeof( main_str_ver ) - 1U ) );          /*  "\r\n������ " */
    rs232_num_out( VERSION, 10U );                                           /* ����� ������ ������ */
    rs232_str_out( main_str_point, ( sizeof( main_str_point ) - 1U ) );      /* "." */
    rs232_num_out( RELEASE, 10U );                                           /* ����� ����� ������ */
    /* ����� ����������� ����� */
    rs232_str_out( main_str_cs_bios, ( sizeof( main_str_cs_bios ) - 1U ) );  /* ", ����������� ����� = " */
    rs232_str_out( main_str_0x, ( sizeof( main_str_0x ) - 1U ) );            /* "0x" */
    rs232_num_out( CS_LOADER, 16U );                                         /* ����� �������� �� */
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );      /* "\r\n" */

    /* ����������� ������ ���������������� ������ RS-232 */
    for ( i = 0; i < 1000000U; i++ ) {
        /* ����� ����� ����������� ������ RS-232 */
        ;
    }
    rs232_close();

    /* �������� ���������� ���������������� �������� */
    if ( NO_ERROR != tech_monitor() ) {
        /* ���� ��������� ����� � ������� �� ���������������� ��������,
           �� ���������� ���������� ������� */
        ( void ) rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,          
                             RS232_BITS_8, RS232_CTS_RTS_NOUSE );
        rs232_str_out( main_str_cerr, sizeof( main_str_cerr ) - 1U );       /* "\r\n���������� ������ � ������ ����������" */
        main_stop();
    } else {
        /* ����� ��� ������. ���������� */
        ( void ) rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,
                             RS232_BITS_8, RS232_CTS_RTS_NOUSE );
        rs232_str_out( main_str_mend, sizeof( main_str_mend ) - 1U );       /* "\r\n��������������� ������� �������� ������" */
        for ( i = 0; i < 1000000U; i++ ) {
            /* ����� ����� ����������� ������ RS-232 */
            ;
        }
        rs232_close();
        /* ���������� */
        irq_reset_entry();
    }    
}

/* ������������ ������ � �������� ��������� */
static LOADER_FUNC void main_exit_user( image_header_t* image_header )
{
    uint32_t        addr, i;
    
    /* ����� ��������� � ������� ��������� �� */
    rs232_str_out( main_str_work, ( sizeof( main_str_work ) - 1U ) );           /* "\r\n\r\n������ ��������� �� ����� " */
    rs232_num_out( image_header->location, 10U );
    /* ����� ����������� ���� */
    rs232_str_out( main_str_cs_user, ( sizeof( main_str_cs_user ) - 1U ) );
    rs232_str_out( main_str_0x, ( sizeof( main_str_0x ) - 1U ) );
    rs232_num_out( CS_USER, 16U );
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );         /* "\r\n" */

    /* ������ ���������, ���� � �������� ������ ��������� ����
       ���������� ������ */
    if ( 0U != ( MPM_LOADER_OUT_PTR->test_result & TEST_RESULT_ERR_OTHER ) ) {
        rs232_str_out( main_str_cerr, sizeof( main_str_cerr ) - 1U );           /* "\r\n���������� ������ � ������ ����������" */
    }
    /* ����������� ������ ���������������� ������ RS-232 */
    for ( i = 0; i < 1000000U; i++ ) {
        /* ����� ����� ����������� ������ RS-232 */
        ;
    }
    rs232_close();
    /* ��������� �������, ������� ����������� WDT */
    timer_close();
    /* B������������e ��������� ��������� ���������� */
    un_init_hardware();
    /* �������� ���������� ��������� ������������ */
    addr = image_header->entry_address;
    ( ( void (*)( void ) ) addr ) ();
    
}

/* ������������ ���������� ��������� ������� ���������
 *  (in)  start_img_address  - ����� ������ ��������� ���������������� ���������
 *  (out) dest               - ��������� �� ��������� ���������
 * ���������:
 *  INVALID_PARAM    - ������. ������������ ����� ������� ���������.
 */
static LOADER_FUNC error_t main_read_image_header ( const uint32_t start_img_address,
    image_header_t *dest )
{
    error_t         err;
    uint32_t        data;
    
    err = NO_ERROR;
    data = start_img_address;
    
    if (   ( ( CROM_START ) > ( uint32_t ) start_img_address ) ||
           ( ( UROM_START - sizeof( image_header_t ) ) <= ( uint32_t ) start_img_address )
    ) {
        err = INVALID_PARAM;
    } else {
    
        dest->location           = u32_swap( * ( uint32_t * )( data +  0U ) );
        dest->offs               = u32_swap( * ( uint32_t * )( data +  4U ) );
        dest->size               = u32_swap( * ( uint32_t * )( data +  8U ) );
        dest->sects              = u32_swap( * ( uint32_t * )( data +  12U ) );    
        dest->entry_address      = u32_swap( * ( uint32_t * )( data +  16U ) );
        dest->cs                 = u32_swap( * ( uint32_t * )( data +  20U ) );
        
    }
    return err;
}

/* ������������ ���������� ��������� ������ ��������
 *  (out) dest      - ��������� ���������
 */
static LOADER_FUNC void  main_read_load_header (
    load_header_t *dest )
{
    uint32_t        data;

    data = ( uint32_t ) CROM_START;
    dest->module            = u32_swap( * ( uint32_t * )( data +  0U ) );
    dest->chan              = u32_swap( * ( uint32_t * )( data +  4U ) );
    dest->proc              = u32_swap( * ( uint32_t * )( data +  8U ) );
    dest->img_num           = u32_swap( * ( uint32_t * )( data + 12U ) );
}

/* ������������ �������� ��������� ��������� ��
 *  (in/out)  ptr_image_header  - ��������� �� ��������� ��������� ������ ��������
 * ���������:
 *  NO_ERROR         - �����
 *  ERROR            - ������
 */
static LOADER_FUNC error_t main_locaton_detect(
    image_header_t *ptr_image_header )
{
    load_header_t   load_header;           /* ��������� ������ ��������         */
    uint32_t        load_header_cs;        /* ����������� ����� ������ �������� */
    uint32_t        image_header_addr;     /* ������� �����                     */
    sz_t            load_header_full_size; /* ������ ��������� ������ ��������,
                                              ������� ��������� ������� ��      */
    uint32_t        pos_code;              /* ������� �������� ������� ����� D */
//    uint32_t        pos_code1;             /* ��� ���������������� ������ ���� ��� */
//    uint32_t        pos_code2;             /* ��� ���������������� ������ ���� ��� */
    uint32_t        parity;                /* ��� �������� */ 
    uint32_t        temp;
    error_t         err;
    stm32f407_gpio_t *ptr_reg_gpio;
    
    ptr_reg_gpio = STM32F407_GPIOD_PTR;

    /* �������� ������� ����� ���������� ��������� ����������������� �� */
    image_header_addr = MPM_LOADER_OUT_PTR->image_header_addr;
    load_header_full_size = 0U;

    if ( 0U == image_header_addr ) {
        /* ��� ����� ���������� ��������� ������ �� */
        err = NO_ERROR;
    } else {
        /* ���� ����� ��������� ��������� ������ �� */
        err = NO_ACTION;
    }

    /* �������� ��������� ������ �������� */
    main_read_load_header( &load_header );
    load_header_full_size = sizeof( load_header_t )
        + ( load_header.img_num * sizeof( image_header_t ) );
    if((CROM_START + load_header_full_size) < UROM_START) {
        load_header_cs = u32_swap(* ( uint32_t * )(
            CROM_START + load_header_full_size)
        );
    }
    if (   ( load_header.proc      != LOADER_STM32       )
        || ( load_header.module    != LOADER_EUMW )
        || (    ( load_header.chan != LOADER_CHAN_MODEL  )
           )
        || ( load_header.img_num   >  LOADER_IMG_MAX     )
        || ( load_header.img_num   <  LOADER_IMG_MIN     )
    ) {
        
        if (    ( 0xFFFFFFFFU == load_header.img_num )
             && ( 0xFFFFFFFFU == load_header_cs )
        ) {
            /* ���������� ������ �������� */
            rs232_str_out( main_str_empty,
                           ( sizeof( main_str_empty ) - 1U ) );
        } else {
            /* ������������ ���������� ��������� */
            rs232_str_out( main_str_bad_head,
                           ( sizeof( main_str_bad_head ) - 1U ) );
        }
        /* ������� � ��������������� ����� */
        main_exit_monitor();
    } else if ( NO_ACTION == err ){
        /* ���� ����� ��������� ����� ��.
           ����������� �������� ��������� ������ �������� �� ��������� */
        err = NO_ERROR;
    } else if ( NO_ERROR != test_rom_pass(
        /* ��� ����� ���������� ������ �� */
         ( uint8_t * )( CROM_START ),
         load_header_full_size,
         load_header_cs )
    ) {
        /* ������ �������� �� ��������� ������ �������� */
        rs232_str_out( main_str_bad_ks_head,
                       ( sizeof( main_str_bad_ks_head ) - 1U ) );
        err = ERROR;
    } else {
        /* �������� �������� �� */
        err = NO_ERROR;
    }

    /* ���������� ������� ������� ������ �������� ����� */
    if ( NO_ERROR != err ) {
        /* ���������� ������ */
        ;
    } else {
        /* ������ ���� */
        /* ������ ���� �������� ����� �� �������� ����� D  */
        pos_code = IN_REG( ptr_reg_gpio -> idr ) & 0x7U;
//        pos_code1 = pos_code & 0x7U;
//      pos_code2 = (pos_code & 0x38U) >> 3;
//        if ( 0U != (pos_code1 ^ pos_code2)) {       
//            rs232_str_out( main_rkp_pos_err,
//                       ( sizeof( main_rkp_pos_err ) - 1U ) );
//            err = ERROR;
//        }
//        else {
            /* �������� ���� �������� */
            parity  = ( 0U != ( pos_code & MCU_GPIO_IDR_IDR0 ) ) ? MCU_GPIO_IDR_IDR2 : 0U;
            parity ^= ( 0U != ( pos_code & MCU_GPIO_IDR_IDR1 ) ) ? MCU_GPIO_IDR_IDR2 : 0U;
            parity ^= pos_code & MCU_GPIO_IDR_IDR2;
            if ( 0U != ( parity & MCU_GPIO_IDR_IDR2 ) ) {         /* !!!!! �������� ������� �������� ���� ��������. � �������� ������� ������ ������ ���� != */
                /* �������� ������� */
                pos_code &= MCU_GPIO_IDR_IDR0 | MCU_GPIO_IDR_IDR1;
                err = NO_ERROR;
            } else {
                /* �������� �� ������� */
                rs232_str_out( main_rkp_pos_err, ( sizeof( main_rkp_pos_err ) - 1U ) );
                err = ERROR;
//            } 
        }     
    }

    /* ����� ��������� ������ �� */
    if ( NO_ERROR != err) {
        /* ���������� ������ */
        ;
    } else if ( 0U != image_header_addr) {
        /* ���� ����� ��������� ����� �� */
        ;
    } else {
        /* ����� ������ �� */
        /* ������ ������� �� ������� �� */
        image_header_addr = CROM_START + sizeof( load_header_t );
        /* ����� ������ ���������������� �������� ����� */
        err = NO_ACTION;
        do {
            temp = u32_swap(( ( image_header_t * ) image_header_addr )->location);
            /* ����� �� �����, ���� ������ ����� ������ 
               ��� ��� ������ �����������  */
            if ( temp == pos_code ) {
                /* ����� ������ */
                err = NO_ERROR;
            } else {
                /* ����� ���������� ��������� ������ �� */
                image_header_addr += sizeof( image_header_t );
                if ( image_header_addr >= ( CROM_START
                                          + load_header_full_size )
                ) {
                    /* ��� ������ ����������� */
                    break;
                } else {
                    /* ����������� ��������� ������� */
                    ;
                }
            }
        } while ( NO_ERROR != err );
    }

    /* ���������� ��������� ������ �� */
    if ( NO_ERROR != err ) {
        /* ���������� ������ */
        ;
    } else if ( NO_ERROR != main_read_image_header(
                                ( uint32_t ) image_header_addr,
                                ptr_image_header )
    ) {
        /* ���������� ������ */
        err = ERROR;
    } else if ( ptr_image_header->location != pos_code ) {             
        /* ����� �� �� ������������� ���� ���������������� */
        err = NO_ACTION;
    } else {
        /* �������� ���������� */
        ;
    }

    if ( NO_ACTION == err ) {
        /* �� ������ ����� �� ��������������� ���� ���������������� */
        rs232_str_out( main_str_no_user, ( sizeof( main_str_no_user ) - 1U ) );
        rs232_str_out( main_str_locat, ( sizeof( main_str_locat ) - 1U ) );
        rs232_num_out( pos_code, 10U );
        rs232_str_out( main_str_chan_0, ( sizeof( main_str_chan_0 ) - 1U ) );
        MPM_LOADER_OUT_PTR->image_header_addr = 0U;
        err = ERROR;
    } else if ( NO_ERROR != err ){
        /* ���������� ������ */
        rs232_str_out( main_str_user_fail, ( sizeof( main_str_user_fail ) - 1U ) );
        MPM_LOADER_OUT_PTR->image_header_addr = 0U;
    } else {
        /* �������� ���������� */
        MPM_LOADER_OUT_PTR->image_header_addr = image_header_addr;
    }

    return err;
}

