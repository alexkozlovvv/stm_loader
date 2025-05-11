#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"
#include "STM32F407.h"
#include "eumw.h"

#include "test.h"
/* ������� �������������� ��������������� */
#include "test_mem.h"

/* �������� � ��������� ���������������� */     
#define GPIOA_MODER_VAL     0x6828AA00U
#define GPIOA_PUPDR_VAL     0x66960100U   
#define GPIOB_MODER_VAL     0xA9405584U
#define GPIOB_PUPDR_VAL     0x81400002U
#define GPIOC_MODER_VAL     0x02A05555U
#define GPIOC_PUPDR_VAL     0xA80A0000U  
#define GPIOD_MODER_VAL     0x52040000U
#define GPIOD_PUPDR_VAL     0x100A0AAAU
#define GPIOE_MODER_VAL     0x00000000U
#define GPIOE_PUPDR_VAL     0x00000000U
#define GPIOH_MODER_VAL     0x00000000U
#define GPIOH_PUPDR_VAL     0x00000000U

/* ������������ ������������ ���
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
error_t test_cpu( void )
{
    uint_t          cnt, cnt_err;
    error_t         err;

    cnt = 0U;
    cnt_err = 0U;
    while ( 3U > cnt ) {
        if ( NO_ERROR != test_cpu_pass() ) {
            cnt_err++;
        }
        cnt++;
        if (   ( 0U == cnt_err )
            && ( 2U == cnt )
        ) {
            /* ��� �������� �������. ������ ������ �� ������ */
            cnt++;
        } else if (
               ( 2U == cnt_err )
            && ( 2U == cnt )
        ) {
            /* ��� �� �������� �������. ������ ������ �� ������ */
            cnt++;
        } else {
            /* ���������� �������� */
            ;
        }
    }
    err = ( 2U <= cnt_err ) ? ERROR : NO_ERROR;

    return err;
}

/* ������������ ������ ������� ����� ���
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
static error_t test_cpu_pass( void )
{
    stm32f407_gpio_t    *ptr_reg;
    uint32_t            temp;
    error_t             err;

    err = NO_ERROR;
    
    ptr_reg = STM32F407_GPIOA_PTR;
    temp  = IN_REG( ptr_reg->moder );         
    if ( temp != GPIOA_MODER_VAL ) {
        err = ERROR;
    } 
    else {
        temp  = IN_REG( ptr_reg->pupdr );
        if ( temp != GPIOA_PUPDR_VAL ) {
            err = ERROR;
        }
    }
    
    if ( NO_ERROR == err ) {
        ptr_reg = STM32F407_GPIOB_PTR;
        temp  = IN_REG( ptr_reg->moder );         
        if ( temp != GPIOB_MODER_VAL ) {
            err = ERROR;
        } else {
            temp  = IN_REG( ptr_reg->pupdr );
            if ( temp != GPIOB_PUPDR_VAL ) {
                err = ERROR;
            }
        }
    }
    
    if ( NO_ERROR == err ) {
        ptr_reg = STM32F407_GPIOC_PTR;
        temp  = IN_REG( ptr_reg->moder );
        if ( temp != GPIOC_MODER_VAL ) {
            err = ERROR;
        } else {
            temp  = IN_REG( ptr_reg->pupdr );
            if ( temp != GPIOC_PUPDR_VAL ) {
                err = ERROR;
            }
        }
    }
    
    if ( NO_ERROR == err ) {
         ptr_reg = STM32F407_GPIOD_PTR;
         temp  = IN_REG( ptr_reg->moder );
         if ( temp != GPIOD_MODER_VAL ) {
             err = ERROR;
        } else {
            temp  = IN_REG( ptr_reg->pupdr );
            if ( temp != GPIOD_PUPDR_VAL ) {
                err = ERROR;
            }
        }
    }
    
    if ( NO_ERROR == err ) {
        ptr_reg = STM32F407_GPIOE_PTR;
        temp  = IN_REG( ptr_reg->moder );
        if ( temp != GPIOE_MODER_VAL ) {
            err = ERROR;
        } else {
            temp  = IN_REG( ptr_reg->pupdr );
            if ( temp != GPIOE_PUPDR_VAL ) {
                err = ERROR;
            }
        }
    }
    
    if ( NO_ERROR == err ) {
        ptr_reg = STM32F407_GPIOH_PTR;
        temp  = IN_REG( ptr_reg->moder );
        if ( temp != GPIOH_MODER_VAL ) {
            err = ERROR;
        } else {
            temp  = IN_REG( ptr_reg->pupdr );
            if ( temp != GPIOH_PUPDR_VAL ) {
                err = ERROR;
            }
        }
    }
    return err;
}
