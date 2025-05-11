#include "sections_bsp.h"
#include "STM32F407.h"
#include "base_types.h"
#include "os_def.h"
#include "eumw.h"

#include "loader.h"

#include "test.h"
#include "rs232.h"
#include "timer.h"
#include "adc.h"
#include "multiplexer.h"

//#include <math.h>

/* ������� �������������� ��������������� */
#include "test_mem.h"


#define ROW    5U
#define COL    2U

static LOADER_CONST const char_t str_rn[] =
    "\r\n";
//static LOADER_CONST char_t res[] =
//    "";
static LOADER_CONST const char_t minus[] =
    " - ";
static LOADER_CONST const char_t dot[] =
    ".";
static LOADER_CONST const char_t volt[] =
    "� ";
static LOADER_CONST const char_t str_err[] = 
    "err\r\n";
static LOADER_CONST const char_t verification_ok[] = 
    "�������� � �������";
static LOADER_CONST const char_t out_of_range[] = 
    "����� �� ������";  
static LOADER_CONST const char_t temp_info[] = 
    "\r\n����������� ������ �����: "; 
static LOADER_CONST const char_t celsium_sign[] = 
    "\xB0�"; 
static LOADER_CONST const char_t get_temp_error[] = 
    "�����"; 
static LOADER_CONST const f32_t voltage1[ROW][COL] = 
    { {2.97f, 3.63f}, {4.5f, 5.5f}, {1.62f, 1.98f}, {1.35f, 1.65f}, {0.0f, 0.0f} }; /* (s17 - s20) 10% �������� �� ���������� +3,3�, +5�, +1,8�, +1,5� */
static LOADER_CONST const f32_t voltage2[ROW][COL] = 
    { {0.0f, 0.0f}, {6.75f, 8.25f}, {-8.25f, -6.75f}, {0.9f, 1.1f}, {0.675f, 0.825f} };    /* (s18 - s21) +7,5�, -7,5�, +1�, +0,75� */   

/* ������������ ������������ ���
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
error_t test_adc( void )
{
    uint_t          cnt, cnt_err;
    error_t         err;

    /* ������������� ����������� �������������� � ��� � 
    ����������� ������������� */
    adc_init();
    multiplexer_init();
    
    cnt = 0U;
    cnt_err = 0U;
    while ( 3U > cnt ) {
        if ( NO_ERROR != test_adc_pass() ) {
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
    
/* ������������ ������������ ���
 * ���������:
 *  NO_ERROR        - �������� ����������
 *  ERROR           - ������
 */
error_t test_adc_pass( void )
{
    
    uint32_t            i;
    error_t             err;
    
    err = NO_ERROR;

    /* �������� ������ ��� � ������� ������� ����������
       �� ������ ������������ D60 � D63 */ 
    for ( i = 17; i <= 21; i++ ) {
        if (NO_ERROR != test_get_adc(i)) {
            err = ERROR;
            break;
        }
    }

    return err;
}

/* ===========��������� ������� �������� float � ������===============*/

/* ������� �������������� ����������� ������� */
LOADER_FUNC void reverse(char_t* str, int32_t len) 
{ 
	int32_t i = 0, j = len - 1, temp; 
	while (i < j) { 
		temp = str[i]; 
		str[i] = str[j]; 
		str[j] = temp; 
		i++; 
		j--; 
	} 
} 

/* ������� �������������� ������ ����� � ������ */
LOADER_FUNC int32_t intToStr(int32_t x, char_t str[], int32_t d) 
{ 
	int32_t i = 0; 
    
    /* ��������� ������ ����������� ��������������� ����
       ������ ����� (������� � �������� ����������� �������) */
	while (x) { 
		str[i++] = (x % 10) + '0'; 
		x = x / 10; 
	} 

	/* ���� ���������� ������� ������� ���������� ���� 
       ��� ������� � �����, �� ������� ����������� ���� */    
	while (i < d) 
		str[i++] = '0'; 

    /* �������������� ����������� ������� */
	reverse(str, i); 
	str[i] = '\0'; 
	return i; 
}

/* ������� ���������� � ������� */
LOADER_FUNC uint32_t pow_uint( uint32_t input_num, uint32_t power ) {

    uint32_t        i;
    uint32_t        result = 1U;
    
    for( i = 0U; i < power; i++) {
    
        result *= input_num;
    }
    
    return result;
}    

/* ������� �������������� ��������������� ������������ ����� � ������ */
LOADER_FUNC void ftoa(f32_t n, char_t* res, int32_t afterpoint) 
{ 
	/* ���������� ����� ����� ����� ������������ ������� */   
	int32_t ipart = (int32_t)n; 

	/* ���������� ������� ����� */
	f32_t fpart = n - (f32_t)ipart; 

	/* �������������� ����� ����� � ������ */
	int32_t i = intToStr(ipart, res, 1); 

	/* �������� �� ������������� ������ ������� ����� */
	if (afterpoint != 0) { 
		res[i] = '.';  /* ���������� ����� � ����� */

		/* ���������� � ������� ����� ���. ���������� �������� 
           � ������������ � �������� ���������� afterpoint */        
		fpart = fpart * (f32_t) pow_uint(10U, afterpoint); 

		intToStr((int32_t)fpart, res + i + 1, afterpoint); 
	} 
} 
/* ===================================================================*/

/* ������������ ��������� �������� ���������� �� ��������� ��� */
LOADER_FUNC f32_t get_volt(volatile uint32_t* adc) 
{
    f32_t           adcf = 0.0;
    
    /* ���� ������������� �������� */
    if(*adc & ( uint32_t )0x800){
        adcf = (f32_t)((~(*adc - ( uint32_t ) 0x01)) & ( uint32_t ) 0xFFF);
        adcf *= -0.0048828125f;
    } else {
        adcf = (f32_t)*adc * 0.0048828125f;
    } 
    return adcf;    
}

/* ������� ��� ���������� � �������� ��������� ��� */
error_t test_get_adc(uint32_t pos)
{
	 uint32_t        i;
	 error_t         err;
	 volatile uint32_t        adc1, adc2, multiplexer_state;
     f32_t           adcf1, adcf2 = 0.0;
	 
	 err = NO_ERROR;
	 
	 /* �������� �������� ��������� */
	 if(pos < 17U || pos > 21U ) {
		 err = INVALID_PARAM;
	 }
	 	  
     /* ������� ���������������� ���������� � ��������� */
     multiplexer_state = pos - 1;
     
     /* �������� ����� �������������� */
     if (NO_ERROR != wait_eocs()){
         err = ERROR;
     }
     
     if( NO_ERROR == err ) {
         err = multiplexer_set(multiplexer_state);
     }
     
     /* �������� ����� �������������� */
     if( NO_ERROR == err ) {
         err = wait_eocs();
     }
     
     /* ��������� ������ � ��� */
     if( NO_ERROR != err ) {
         ;
     } else {
         if( NO_ERROR != adc_read(&adc1, &adc2)) {
             err = ERROR;
         } else {

             adcf1 = get_volt(&adc1);
             adcf2 = get_volt(&adc2);            
 
             i = pos - 17;
             if ( i == 0U ) {
                 if ( !(( adcf1 >= voltage1[i][0] ) && (adcf1 <= voltage1[i][1])) ) {
                     err = ERROR;
                 }
             } else if ( i == 4U ) {
                 if ( !(( adcf2 >= voltage2[i][0] ) && (adcf2 <= voltage2[i][1])) ) {
                     err = ERROR;
                 }
             } else {
                 if ( !(( adcf1 >= voltage1[i][0] ) && (adcf1 <= voltage1[i][1])) ||
                      !(( adcf2 >= voltage2[i][0] ) && (adcf2 <= voltage2[i][1])))  {
                     err = ERROR;
                 }
             }
         }
     }
	 return err;
}

void test_get_temp (void) {
    
    volatile uint32_t        adc1, adc2;
    f32_t                    tempf;  
	volatile int32_t		     eoc1=0,eoc2=0;
    error_t                  err;
    char_t res1[] =  "";
    
    /* ������������� ����������� �������������� � ��� � 
    ����������� ������������� */
    adc_init();
    multiplexer_init();
    
    rs232_puts(str_rn);
    rs232_str_out( temp_info, ( sizeof( temp_info ) - 1U ) );  

    if (NO_ERROR != wait_eocs()){
        err = ERROR;
    }

    if( NO_ERROR == err ) {
        err = multiplexer_set(20);
    }
    
    if( NO_ERROR == err ) {
        err = wait_eocs();
    }
    
    if( NO_ERROR == err ) {
         
        if( NO_ERROR == adc_read(&adc1, &adc2)) {
            
            tempf = get_volt(&adc1);
            tempf *= 10.0f;

            if(tempf < 0){
                tempf *= -1.0f;
                ftoa(tempf, res1, 1U);
                rs232_puts(minus);
            } else {
                ftoa(tempf, res1, 1U);
            }
            rs232_puts(res1);
            rs232_str_out( celsium_sign, ( sizeof( celsium_sign ) - 1U ) );
            
        } else {
            rs232_puts(get_temp_error);
        }
        
    } else {
        rs232_puts(get_temp_error);
    }
}
