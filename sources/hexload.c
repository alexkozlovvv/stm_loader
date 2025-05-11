#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"

#include "rs232.h"
#include "hexload.h"
#include "hexload_mem.h"

//#define MAX_REC_LENG  256U

/* ���� ������ �������� HEX-����� */
#define HEX_LOAD_ERR_BASE       ( -100 )
/* .  ������� */
#define HEX_LOAD_OK             ( 0 )
/* i �������������� ������ */
#define HEX_LOAD_INFO           ( HEX_LOAD_ERR_BASE - 1 )
/* z ���������� ������ ������ */
#define HEX_LOAD_NULL           ( HEX_LOAD_ERR_BASE - 2 )
/* l ������ �������. ������������ �������� ���� Byte Count */
#define HEX_LOAD_ERR_LENG       ( HEX_LOAD_ERR_BASE )
/* n ������ �������. �������� ������ */
#define HEX_LOAD_ERR_SHORT      ( HEX_LOAD_ERR_BASE - 3 )
/* s ������ �������. ���������� ����������� ������ */
#define HEX_LOAD_ERR_SYMB       ( HEX_LOAD_ERR_BASE - 4 )
/* a �������� ���������� */
#define HEX_LOAD_ERR_ABORT      ( HEX_LOAD_ERR_BASE - 5 )
/* b ������ ���������� */
#define HEX_LOAD_ERR_BUFF       ( HEX_LOAD_ERR_BASE - 6 )
/* w ���� ������ */
#define HEX_LOAD_ERR_STOR       ( HEX_LOAD_ERR_BASE - 7 )
/* c ������ �����. ����� */
#define HEX_LOAD_ERR_CSUM       ( HEX_LOAD_ERR_BASE - 8 )
/* r ������ �� ��������� ������� ����� ������ */
#define HEX_LOAD_ERR_END        ( HEX_LOAD_ERR_BASE - 9 )
/* t ������. ��������� �� �������������� ��� ������ */
#define HEX_LOAD_ERR_TYPE       ( HEX_LOAD_ERR_BASE - 10 )

static LOADER_CONST const char_t hex_load_legend[] =
  "��������:\r\n"
  "'�' - �������������� ������\r\n"
  "'.' - ������ � �������\r\n"
  "'#' - ����������� ������\r\n"
  "'�' - ������ ������\r\n"
  "������ ��� ��������� ������:\r\n"
  "'�' - ������������ ��������� ���� Byte Count\r\n"
  "'�' - ����� ������ �� ������������ ����������\r\n"
  "'�' - ��������� ������������ ������\r\n"
  "'�' - �������� ������� ���������� ��������\r\n"
  "'�' - ����� ������ ��������� ����������\r\n"
  "'�' - ������ ������ ������ � ������\r\n"
  "'�' - ������ ����������� ����� ������\r\n"
  "'�' - �� ��������� ����� ������\r\n"
  "'�' - �� �������������� ��� ������\r\n"
  "'�' - ������ ������\r\n"
  "<�> - �������� ��������\r\n";

/* ������������ ���������� ������� �� ���-����� � ��������������� ��� �
 * ����������������� �����.
 *  (out) dest         - ��������� �� ����������, ��� ����� ��������� �����
 *  (in)  ptr_record   - ��������� �� ��������� �������� ������ ���-�����
 * ���������:
 *  HEX_LOAD_OK        - �������� ����������
 *  HEX_LOAD_ERR_ABORT - ������. ���������� ��������� �������� �������������
 *  HEX_LOAD_ERR_SYMB  - ������. ����������� ������
 */
static error_t hex_parse_digit(
    uint8_t *dest, struct tag_hex_record *ptr_record )
{
    uint8_t         symb;
    error_t         err;

    /* ������: ��������������� ����� ������ */
    if( ptr_record->idx_read >= ptr_record->idx_save ) {
        err = HEX_LOAD_ERR_SHORT;
    } else {
        /* ������� ������� �� ������ */
        symb = ptr_record->buff[ ptr_record->idx_read ];
        ptr_record->idx_read++;
        if( ( symb >= 0x30U ) && ( symb <= 0x39U ) ) {   // �.� ���� ��������� ��� ������������� ���� ��������� ���������� ���� 
            /* ���������� ������ (0-9)*/
            *dest = ( uint8_t )( symb - 0x30U );   // �� ����������� ���������� ����� � �����������������
            err = HEX_LOAD_OK;
        } else if( ( symb >= 0x41U ) && ( symb <= 0x46U ) ) {
            /* ����������������� ������ (A-F)*/
            *dest = ( uint8_t )( ( symb - 0x41U ) + 0xAU );
            err = HEX_LOAD_OK;
        } else if ( ( ( uint8_t ) 0xC9U /* 0x1BU - ESC */ ) == symb ) {
            /* ������ � */
            err = HEX_LOAD_ERR_ABORT;
        } else {
            /* ������ - ����������� ������ */
            err = HEX_LOAD_ERR_SYMB;
        }
    }

    return err;
}

/* ������������ ���������� ����� (2 �������) �� ���-����� � ���������������
 * ��� � �����
 *  (out) dest         - ��������� �� ����������, ��� ����� ��������� �����
 *  (in)  ptr_record   - ��������� �� ��������� �������� ������ ���-�����
 * ���������:
 *  HEX_LOAD_OK        - �������� ����������
 *  HEX_LOAD_ERR_ABORT - ������. ���������� ��������� �������� �������������
 *  HEX_LOAD_ERR_SYMB  - ������. ����������� ������
 */
static error_t hex_parse_byte(
    uint8_t *dest, struct tag_hex_record *ptr_record )
{
    uint8_t         part_hi, part_lo, byte;
    error_t         err;

    /* ���������� ������� �������. ���������� ������������ �������� �� ������
     * ������ ������ �� �������� ������� � ������������ � ����� */
    err = hex_parse_digit( &part_hi, ptr_record );
    if ( HEX_LOAD_OK != err ) {
        ;
    } else {
        /* ���������� ������� �������. ���������� ������������ �������� �� ������
         * ������ ������ �� �������� ������� � ������������ � ����� */
        err = hex_parse_digit( &part_lo, ptr_record );
        if ( HEX_LOAD_OK != err ) {
            ;
        } else {
            /* byte = ( part_hi << 4 ) | part_lo ) */
            part_hi <<= 4;
            byte  = part_hi;
            byte |= part_lo;
            /* ������� ����������� ����� */
            ptr_record->csum += byte;
            /* ������������ ���������� */
            if( dest != NULL ) {
                *dest = byte;
            }
            /* err = HEX_LOAD_OK */
        }
    }

    return err;
}

/* ������������ ���������� ����� (4 ����� ��� 2 �����) �� ���-����� �
 * ��������������� ��� � �����
 *  (out) dest         - ��������� �� ����������, ��� ����� ��������� �����
 *  (in)  ptr_record   - ��������� �� ��������� �������� ������ ���-�����
 *  (in)  flag_32      - �������: 1 - 32-��������� �����, 0 - 16-���������
 * ���������:
 *  HEX_LOAD_OK        - �������� ����������
 *  HEX_LOAD_ERR_ABORT - ������. ���������� ��������� �������� �������������
 *  HEX_LOAD_ERR_SYMB  - ������. ����������� ������
 */
static error_t hex_parse_word (
    uint32_t *dest, struct tag_hex_record *ptr_record, uint32_t flag_32 )
{
    uint32_t        i;
    uint32_t        numb;
    uint8_t         part;
    error_t         err;

    /* ��������� 4 ����, ������� �� �������� */
    numb = 0U;
    for ( i = 0U; i < ( 2U + ( 2U * flag_32 ) ); i++ ) {
        /* ���������� ���������� ����� */
        err = hex_parse_byte( &part, ptr_record );
        if ( HEX_LOAD_OK != err ) {
            /* ���������� ������ */
            break;
        } else {
            /* err = HEX_LOAD_OK */
            numb = ( numb << 8 ) | part;
        }
    }
    /* ������������ ���������� */
    if (   ( HEX_LOAD_OK == err )
        && ( NULL != dest )
    ) {
        *dest = numb;
    }

    return err;
}

/* ������������ ���������� ����� ������ ��������� �������, ������������� �
 * ������ � ��� �� ��������� ������
 *  (in)  dest         - ��������� ����� ����� ������ � ���
 *  (in)  size         - ������ ����� ������
 *  (in)  ptr_record   - ��������� �� ��������� �������� ������ ���-�����
 * ���������:
 *  HEX_LOAD_OK        - �������� ����������
 *  HEX_LOAD_ERR_ABORT - ������. ���������� ��������� �������� �������������
 *  HEX_LOAD_ERR_SYMB  - ������. ����������� ������
 *  HEX_LOAD_ERR_STOR  - ������. ���� ����������� ������� � ���
 */
static error_t hex_parse_block(
    volatile uint8_t dest[], sz_t size, struct tag_hex_record *ptr_record )
{
    uint8_t         i;
    uint8_t         byte;
    error_t         err;

    err = NO_ERROR;
    for( i = 0U; i < size; i++ ) {
        /* ���������� ����� */
        err = hex_parse_byte( &byte, ptr_record );
        if( HEX_LOAD_OK != err ) {
            /* ���������� ������ */
            ;
        } else {
            /* C��������� �� ��������� ������ */
            dest[ i ] = byte;
            if( dest[ i ] == byte ) {
                /* err = HEX_LOAD_OK */
                ;
            } else {
                /* ������ ���������� */
                err = HEX_LOAD_ERR_STOR;
            }
        }
        if( HEX_LOAD_OK != err ) {
            /* ���������� ������ */
            break;
        }
    }

    return err;
}

/* �������� ��������� ������ HEX ����� */
#define LOAD_PROC               ( 0U ) /* ������ ���                          */
#define LOAD_SAVE               ( 3U ) /* ������� ������ ������               */
#define LOAD_CONT               ( 1U ) /* ������� ����������� ������          */
#define LOAD_END                ( 4U ) /* ������� ���������� �����           */

/* ������������ �������� ���-����� � ��� ��� ����������
 *  (in)  ptr_entry - ��������� �� �������� ������ ����� ����� � ���������
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
error_t hex_file_load(
    uint32_t *ptr_entry )
{
    hex_record_t    hex_record; /* ������ HEX ����� */
    sz_t            reclen;     /* ����� ���� ������ � ������ (� ������) */
    uint_t          typerec;    /* ��� ������ */
    uint32_t        base_addr;  /* �������� ����� */
    uint32_t        addr;       /* ������� ����� ��� ���������� ������ */
    uint32_t        offset;     /* �������� ������ ������������ ���������
                                   �������� ������ */
    uint32_t        entry;      /* �������� ����� ������ ��������� */
    char_t          symb;       /* ��� �������, ���������� �� ������ RS-232 */
    uint_t          load_flag;  /* ������ ��������� ������ */
    uint_t          line;       /* ������� �������� � ����� */
    uint8_t         temp8;
    error_t         err, lerr;  /* ��� ������ */

    rs232_str_out( hex_load_legend, sizeof( hex_load_legend ) - 1U );

    load_flag = LOAD_PROC;
    line      = 0U;
    typerec   = 0xFFU;
    reclen    = 0U;
    base_addr = 0U;
    addr      = 0U;
    entry     = 0U;
    hex_record.idx_save = 0U;
    
    err = NO_ERROR;
    
    /* ���� �������� ���-����� (���� �� ���������� �������� ������ ���� 01
       ��� �� ����� ������� ������ '�') */
    while ( LOAD_END != load_flag ) {

        /* �������� ������ ��������� ������ �E�-����� (������ ':') */
        if ( LOAD_SAVE == load_flag ) {
            /* �������� �� ���������. ������ ':' ��� ������� */
            lerr = HEX_LOAD_OK;
        } else {
            /* �������� */
            do {
                rs232_getc( &symb );
            } while (
                ( symb != ':' )                       /* 0x3A */
                && ( symb != ( ( char_t ) 0xC9U ) )   /* � */
            );
            if ( ':' == symb ) {
                /* ������� ������ ������ ����� ������ */
                load_flag = LOAD_SAVE;
                lerr = HEX_LOAD_OK;
            } else {
                /* ������� ������ ESC. ���������� ����� hex-����� */
                load_flag = LOAD_CONT;
                lerr = HEX_LOAD_ERR_ABORT;
            }
        }

        /* ���� ������ ������ ���-�����
           ������ ������ ��������� ��������� �������� ����� ������ */
        if ( LOAD_SAVE != load_flag ) {
            /* ������� �������� */
            ;
        } else {
            hex_record.buff[ 0 ] = ( uint8_t ) ':';
            hex_record.idx_read = 1U;
            hex_record.idx_save = 1U;
            hex_record.csum = 0U;
            while ( LOAD_SAVE == load_flag ) {
                rs232_getc( &symb );
                if ( symb == ':' ) {                  /* 0x3AU */
                    /* ����������� ��������� ������� ����� ������ */
                    load_flag = LOAD_SAVE;
                    lerr = HEX_LOAD_ERR_END;
                    break;
                } else if ( hex_record.idx_save >= MAX_REC_LENG ) {
                    /* ����� ������ ������ ���������� */
                    load_flag = LOAD_CONT;
                    lerr = HEX_LOAD_ERR_BUFF;
                } else if (
                       ( symb == ( '\n' ) /* 0x0A */ )
                    || ( symb == ( '\r' ) /* 0x0D */ )
                ) {
                    /* O�������� ����� ������ */
                    hex_record.buff[ hex_record.idx_save ] = 0x00U;   // ����� ��� ��������� �������� ����� ������ ���������� � ����� ������� ����
                    load_flag = LOAD_PROC;
                    break;
                    /* lerr = HEX_LOAD_OK */
                } else {
                    /* C��������� ������� � ������ ������ */
                    hex_record.buff[ hex_record.idx_save ] = ( uint8_t ) symb;
                    hex_record.idx_save++;
                }
            }
        }

        /* ��������� �������� ������. ���� Reclen, Offset, Typerec */
        if ( LOAD_PROC != load_flag ) {
            /* ������� �������� */
            ;
        } else if ( hex_record.idx_save < ( 1U + 1U ) ) {
            /* ������ ������ ���������, �� ��������� */
            load_flag = LOAD_CONT;
            lerr = HEX_LOAD_NULL;
        } else {
            /* ����������� ����� ���� ������ � ������ */
            lerr = hex_parse_byte( &temp8, &hex_record );
            reclen = temp8;
            if ( HEX_LOAD_OK != lerr ) {
                /* ���������� ������ */
                load_flag = LOAD_CONT;
            } else {
                /* ����������� �������� ��� ������ */
                lerr = hex_parse_word( &offset, &hex_record, 0U );
                if ( HEX_LOAD_OK != lerr ) {
                    /* ���������� ������ */
                    load_flag = LOAD_CONT;
                } else {
                    /* ����������� ���� ������ */
                    lerr = hex_parse_byte( &temp8, &hex_record );
                    typerec = temp8;
                    if ( HEX_LOAD_OK != lerr ) {
                        /* ���������� ������ */
                        load_flag = LOAD_CONT;
                    } else {
                        /* lerr = HEX_LOAD_OK */
                        ;
                    }
                }
            }
        }

        /* ��������� �������� ������. ���� Address, Data
           (������� �� ���� ������ */
        if ( LOAD_PROC != load_flag ) {
            /* ������� �������� */
            ;
        } else if (
            /* Data */
               ( 0U == typerec )
        ) {
            /* ������ � �������. 16-bit offset */
            if ( reclen < 1U ) {
                /* ������������ ������ ������ */
                load_flag = LOAD_CONT;
                lerr = HEX_LOAD_ERR_LENG;
            } else {
                /* ��������� �������� ������ ������ */
                addr = base_addr | offset ;
                /* ���������� � ���������� ������ */
                lerr = hex_parse_block( ( uint8_t * ) addr, reclen, &hex_record );
                if ( HEX_LOAD_OK != lerr ) {
                    /* ���������� ������ */
                    load_flag = LOAD_CONT;
                } else {
                    /* lerr = HEX_LOAD_OK */
                    ;
                }
            }
        } else if (
            /* Entry Address */
                ( 5 == typerec )
        ) {
            /* ����� ����� ����� � ��������� */
            if ( ( reclen != 4U ) || ( offset != 0U ) ) {
                /* ������������ ������ ������ */
                load_flag = LOAD_CONT;
                lerr = HEX_LOAD_ERR_LENG;
            } else {
                /* ��������� ����� ����� � ����������� ��������� */
                lerr = hex_parse_word( &entry, &hex_record, 1U );
                if ( NO_ERROR != lerr ) {
                    /* ���������� ������ */
                    load_flag = LOAD_CONT;
                } else {
                    /* lerr = HEX_LOAD_OK */
                    ;
                }
            }
        } else if (
            /* Termination */
               ( 1U == typerec )
        ) {
            /* ����������� ������ */
            if ( ( reclen != 0U ) || ( offset != 0U ) ) {
                /* ������������ ������ ������ */
                load_flag = LOAD_CONT;
                lerr = HEX_LOAD_ERR_LENG;
            } else {
                /* lerr = HEX_LOAD_OK */
                ;
            }
        } else if (
            /* Base address */
               ( 4U == typerec )
        ) {
            /* ������ ������. �������� ������� ����� */
            if ( ( reclen != 2U ) || ( offset != 0U ) ) {
                /* ������������ ������ ������ */
                load_flag = LOAD_CONT;
                lerr = HEX_LOAD_ERR_LENG;
            } else {
                /* ��������� �������� ������ ������ */
                lerr = hex_parse_word( &base_addr, &hex_record, 0U );
                if ( NO_ERROR != lerr ) {
                    /* ���������� ������ */
                    load_flag = LOAD_CONT;
                } else {
                    /* lerr = HEX_LOAD_OK */
                    base_addr = ( base_addr << 16U );
                }
            }
        } else if (
            /* Segment address */
               ( 2U == typerec )
            /* Segment start address */
            || ( 3U == typerec )
        ) {
            /* ������ �� �������������� ������������� */
            load_flag = LOAD_CONT;
            lerr = HEX_LOAD_ERR_TYPE;
        } else {
            /* ������������ ��� */
            load_flag = LOAD_CONT;
            lerr = HEX_LOAD_ERR_TYPE;
        }

        /* �������� ����������� ����� ������ */
        /* ������ ����������� ����� (��������� ���� ������). � ��������
         * ��������� �������� ������ �������������� ��, � ������� �������
         * hex_parse_byte(NULL) ���������� �������� �� ������ (��������� ����)
         */
        if ( LOAD_PROC != load_flag ) {
            /* ������� �������� */
            ;
        } else {
            /* ���������� ����������� ����� ������.
             * �� �������� ����������� ����� ������ ������ ����� ":" � �� �� 0�00
             * ����������� ����� �� � ������������ �� ������ ���� 0
             * � ������� 8 ��������
             */
            lerr = hex_parse_byte( NULL, &hex_record );
            if ( HEX_LOAD_OK != lerr ) {
                /* ���������� ������ */
                load_flag = LOAD_CONT;
            } else {
                /* ������ ����������� ����� */
                if ( 0U != ( hex_record.csum & 0xFFU ) ) {
                    /* ������ �������� */
                    load_flag = LOAD_CONT;
                    lerr = HEX_LOAD_ERR_CSUM;
                } else {
                    /* lerr = HEX_LOAD_OK */
                    ;
                }
            }
        }

        /* ������ ���������� ��������� ������ */
        if ( HEX_LOAD_OK == lerr ) {
            if ( 0U == typerec ) {
                /* ������ � ������� ������� ���������� */
                symb = '.';
            } else if ( 4U == typerec ) {
                /* ������ � ������� ������� ������� ���������� */
                symb = '!';
            } else if ( 5U == typerec ) {
                /* ������ � ������� ������ ������� ���������� */
                symb = 'i';
            } else if ( 1U == typerec ) {
                /* ������������ ������ ������� ���������� */
                symb = '#';
                load_flag = LOAD_END;
            }
        } else if ( HEX_LOAD_NULL == lerr ) {
            symb = '�';
        } else if ( HEX_LOAD_ERR_LENG  == lerr ) {
            symb = '�';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_SHORT == lerr ) {
            symb = '�';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_SYMB  == lerr ) {
            symb = '�';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_ABORT == lerr ) {
            symb = '�';
            load_flag = LOAD_END;
            err = ERROR;
        } else if ( HEX_LOAD_ERR_BUFF  == lerr ) {
            symb = '�';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_STOR  == lerr ) {
            symb = '�';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_CSUM  == lerr ) {
            symb = '�';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_END   == lerr ) {
            symb = '�';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_TYPE  == lerr ) {
            symb = '�';
            err = ERROR;
        } else {
            /* ������ ������ */
            symb = '�';
            err = ERROR;
        }

        /* ������ � ��������������� ����� ��������� ��������� ����� */
        if ( line >= 50U ) {
            line = 0U;
            rs232_putc( '\n' );
            rs232_putc( '\r' );
        }
        rs232_putc( symb );
        line++;
    }

    if ( NO_ERROR == err ) {
        *ptr_entry = entry;
    }

    return err;
}

/* ������������ �������������� ���� ����� ������
 *  (in/out)  data - ����� ������
 */
LOADER_FUNC uint32_t u32_swap( uint32_t data )
{
    uint32_t byte[4];
    uint32_t  i;
    
    for ( i = 0; i < 4; i++) {
        switch (i) {
            case 0: 
                byte[i] = (data & 0xFF) << 24; 
                break;
            case 1: 
                byte[i] = (data & 0xFF00) << 8;
                break;
            case 2: 
                byte[i] = (data & 0xFF0000) >> 8;
                break;
            case 3: 
                byte[i] = (data & 0xFF000000) >> 24;
                break;
        }
    }
    data = 0U;
    for ( i = 0; i < 4; i++) {  
        data |= byte[i]; 
    }        
   return data;     
}
