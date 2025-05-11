#include "sections_bsp.h"
#include "base_types.h"
#include "os_def.h"
#include "eumw.h"
#include "STM32F407.h"
#include "irq.h"
#include "itoa.h"
#include "rs232.h"
/* ������� �������������� ��������������� */
#include "rs232_mem.h"

/* ��������� ��� ������� �������� ������ � ������ */
#define RS232_BAUD_BASE         ( MCU_FREQ_APB1 )

/* ������� ����������������� ������ RS-232 */
static LOADER_DATA uint_t rs232_can_work;

/* ������������ ������������� ������ RS-232
 * (in) baud       - �������� ������ ������ USART (���)
 * (in) parity     - ����� ������ � ����� ��������
 * (in) stop_bits  - ���������� ���� �����
 * (in) width      - ����� ����� ������ (���)
 * (in) hw_cntr    - ������� ����������� �������� ������ CTS/RTS
 * ���������:
 * NO_ERROR        - �����
 * INVALID_CONFIG  - ������. ����������� �������� ������� ���������.
 */
error_t rs232_init(
    uint_t baud, uint_t parity, uint_t stop_bits,
    uint_t width, uint_t hw_cntr )
{
    stm32f407_usart_t *ptr_reg;
    uint32_t        dl;
    uint16_t        brr_m;
    uint16_t        brr_f;
    uint16_t        cr1 = 0, cr2 = 0, cr3 = 0;
    uint32_t        msr_old;
    uint32_t        temp;
    volatile uint32_t cnt;
    error_t         err;

    err = NO_ERROR;

    /* Initialize USART controller */
    ptr_reg = STM32F407_USART1_PTR;
    msr_old = kern_irq_disable_interrupt();
    OUT_REG( STM32F407_NVIC_PTR->icer[1], CORE_NVIC_ISER_USART1 );  // ��������� ������� ���������� �� USART1
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ISER_USART1 );  // ����� ���������� ��������� �������� ���������� �� USART1

    temp  = IN_REG( STM32F407_RCC_PTR->apb2enr );
    temp |= MCU_RCC_APB2ENR_USART1EN;
    OUT_REG( STM32F407_RCC_PTR->apb2enr, temp );                    // �������� ������������ USART1
    temp  = IN_REG( STM32F407_RCC_PTR->apb2rstr );
    temp &= ~ ( uint32_t ) MCU_RCC_APB2RSTR_USART1RST;
    OUT_REG( STM32F407_RCC_PTR->apb2rstr, temp );	                // ��������� reset ��� USART1	
    temp  = IN_REG( ptr_reg->cr1 );
    temp &= ~ ( uint32_t ) MCU_USART_CR1_UE;
    OUT_REG( ptr_reg->cr1, temp );                                  // ��������� USART
    kern_irq_set_msr( msr_old );                                    // ��������� ���������� ���������

    if ( 0U == baud ) {
        brr_m = 0U;
        brr_f = 0U;
        err = INVALID_CONFIG;
    } else {
        dl = RS232_BAUD_BASE / ( baud << 4 );
        if (   ( 0U == dl )
            || ( 0U != ( dl & 0xFFFF0000U ) )
        ) {
            brr_m = 0U;
            brr_f = 0U;
            err = INVALID_CONFIG;
        } else {
            brr_m = ( uint16_t ) dl;
            brr_f = ( uint16_t )( RS232_BAUD_BASE % ( baud << 4 ) );
        }
    }

    switch ( parity ) {
        case RS232_PARITY_NO: {                                     // � ������ ���������� �������� ���� �������� ���������� ��������������� ���
            cr1 &= ~ ( uint32_t ) MCU_USART_CR1_PCE;
            break;
        }
        case RS232_PARITY_EVEN: {
            cr1 |= MCU_USART_CR1_PCE;                               // ������������ ��� ���������� �������� ��������
            cr1 &= ~ ( uint32_t ) MCU_USART_CR1_PS;                 // �������� �� �������� ��������
            break;
        }
        case RS232_PARITY_ODD: {
            cr1 |= MCU_USART_CR1_PCE;                               // ������������ ��� ���������� �������� ��������
            cr1 |= MCU_USART_CR1_PS;                                // �������� �� �������� ����������
            break;
        }
        default: {
            err = INVALID_CONFIG;
            break;
        }
    }

    switch ( stop_bits ) {
        case RS232_SB_1: {
            cr2 &= ~ ( uint32_t ) MCU_USART_CR2_STOP_MASK;          // ���������� 1 ����-��� 
            break;
        }
        case RS232_SB_2: {
            cr2 |= MCU_USART_CR2_STOP_2;                            // ���������� 2 ����-����
            break;
        }
        default: {
            err = INVALID_CONFIG;
            break;
        }
    }

    switch ( width ) {
        case RS232_BITS_8: {
            cr1 &= ~ ( uint32_t ) MCU_USART_CR1_M;                  // ��������� ������ ����� ������ 8 ���
            break;
        }
        case RS232_BITS_9: {
            cr1 |= MCU_USART_CR1_M;                                 // ��������� ������ ����� ������ 9 ���
            break;
        }
        default: {
            err = INVALID_CONFIG;
            break;
        }
    }

    if ( 0U != hw_cntr ) {                                          // �������� �� �������� ������ ����������� ���������� �������
        cr3 = MCU_USART_CR3_CTSE | MCU_USART_CR3_RTSE;
        /* ����������� ���������� �� ���������� ��������� ������� ������ */
        err = INVALID_CONFIG;
    } else {
        cr3 &= ~ ( uint32_t ) ( MCU_USART_CR3_CTSE | MCU_USART_CR3_RTSE );      // ���� �� ��������� ����� �� ����� �������� ����
    }

    /* Configure usart1 */
    if ( NO_ERROR != err ) {
        ;
    } else {
        OUT_REG( ptr_reg->brr, ( brr_m << 4 ) | ( ( brr_f >> 12 ) & 0xF ) );    // ���������� ������������ ��� �������� ������ ������
        OUT_REG( ptr_reg->cr1, cr1 );                                           // ���������� ��������
        OUT_REG( ptr_reg->cr2, cr2 );
        OUT_REG( ptr_reg->cr3, cr3 );

        cr1 |= MCU_USART_CR1_UE | MCU_USART_CR1_TE | MCU_USART_CR1_RE;          // ��������� ������ USART, ��������� � �����������
        OUT_REG( ptr_reg->cr1, cr1 );

        
        /* ����������, ��� � ��������� ��������� ���������� ���������
           ��������� ����� ����� ������� ������. ����� ������� �����
           ���������� �� ��������� */
        for ( cnt = 0U; cnt < 100000U; cnt++ ) {
            ;
        }

    }
    
    /* ���� ����������� ��������������� �����, �� ��������� ��� ���������� ������������� */
    if ( NO_ERROR == err ) {
        rs232_can_work = ~ ( uint_t ) 0U;
    } else {
        rs232_can_work = 0U;
    }

    return err;
}

/* ������������ ����������� ������ ������ RS-232 */
void rs232_close( void )
{
    stm32f407_usart_t *ptr_reg;
    uint32_t        temp;
    uint32_t        msr_old;

    if ( 0U == rs232_can_work ) {
        ;
    } else {
        /* ������������������ ���������� ������ ���� ����� */
        ptr_reg = STM32F407_USART1_PTR;
        msr_old = kern_irq_disable_interrupt(); 
        while ( MCU_USART_SR_TC != ( IN_REG( ptr_reg->sr ) & MCU_USART_SR_TC ) ) {          // ���� ����� ���������� �������� ������� ���������
            ;
        }
        temp  = IN_REG( ptr_reg->cr1 );
        temp &= ~ ( uint32_t ) ( MCU_USART_CR1_UE | MCU_USART_CR1_TE | MCU_USART_CR1_RE );
        OUT_REG( ptr_reg->cr1, temp );                                                      // �������� ���������� �� ������ ��������� � ������������, � ��� �� ������ USART

        kern_irq_set_msr( msr_old );                            // ��������������� ���������� ���������� ����������

        msr_old = kern_irq_disable_interrupt();
        temp  = IN_REG( STM32F407_RCC_PTR->apb2enr );
        temp &= ~ ( uint32_t ) MCU_RCC_APB2ENR_USART1EN;        // ��������� ������������ ����� USART1
        OUT_REG( STM32F407_RCC_PTR->apb2enr, temp );
        temp  = IN_REG( STM32F407_RCC_PTR->apb2rstr );
        temp |= MCU_RCC_APB2RSTR_USART1RST;                     // ������������� USART1 � ��������� �����
        OUT_REG( STM32F407_RCC_PTR->apb2rstr, temp );
        kern_irq_set_msr( msr_old );                            // ��������� ���������� ����������
    }
}

/* ������������ ������ ������� � ����� RS-232
 * (in) c - ���������� ������
 */
void rs232_putc(
    char_t c )
{
    stm32f407_usart_t *ptr_reg;

    if ( 0U == rs232_can_work ) {
        ;
    } else {
        ptr_reg = STM32F407_USART1_PTR;
        /* �������� ���������� ������ ����������� */
        while ( MCU_USART_SR_TXE != ( IN_REG( ptr_reg->sr ) & MCU_USART_SR_TXE ) ) {    // ������� ����� ������ ����� ��������� �� Transmit data register � shift register
            ;
        }

        ptr_reg->dr = ( uint8_t ) c;  // ���������� ������ � Data register ����� �� ��� ������������ � output shift register
    }
}

/* ������������ ������ ������� �� ������ RS-232
 * (in) c - ��������� �� ����������, � ������� ����� ��������
 *          �������� ������
 */
void rs232_getc(
    char_t *c )
{
    stm32f407_usart_t *ptr_reg;

    if ( 0U == rs232_can_work ) {
        *c = '\x0';
    } else {
        ptr_reg = STM32F407_USART1_PTR;
        while ( MCU_USART_SR_RXNE != ( IN_REG( ptr_reg->sr ) & MCU_USART_SR_RXNE ) ) {  // ������� ����� ���������� ���� Received data is ready to be read. 
            /* �������� ���������� ������ �������� */
            ;
        }
        *c = ( char_t ) ptr_reg->dr;        // ��������� ������
    }
}

/* ������������ ������ ������� �� ������ RS-232 ��� ������ � ����������
 * (in/out) buffer - ��������� �� ������, � ������� ����� ��������� �������� �������
 * (in) max_size   - ������������ ������ ������ �������� ������� 
 */
void rs232_get_command(char_t* buffer, int32_t max_size)
{    
	char_t symb;
	int32_t len = 0;                  //������ �������� ��������� �������
	
	if ( 0U == rs232_can_work ) {
        ;
    } else {
        while(len < (max_size - 1)) { // ��������� ������� - ����� ������	
            rs232_getc( &symb );		
            if(symb == '\r') {
                if(len == 0) {
                    buffer[len++] = symb;
                }
                /* ������ Enter - ��������� ������ ������� */
                break;
            }
            else if(symb == 0x08){  /* ������ backspace */
                if(len > 0)
                {
                    rs232_putc( 0x08 );
                    rs232_putc( ' ' );
                    rs232_putc( 0x08 );
                    len--;
                }
            }
            else if( symb <= 31 ) {
                ;
            } else {
                rs232_putc( symb );
                buffer[len++] = symb;                  
            }            
        }
        // �������� ������ ����� ������
        if (len < max_size) {
            buffer[len] = '\0';
        } else {
            buffer[max_size - 1] = '\0';
        }
    }
}

/* ������������ ������ ������ �������� � ����� RS-232
 *  (in) str        - ��������� �� ���������� ������ ��������
 * ���������:
 *  len             - ���������� �������� ��������
 */
uint_t rs232_puts(
    const char_t str[] )
{
    uint_t len;

    len = 0U;
    if ( 0U == rs232_can_work ) {
        ;
    } else {
        while ( '\x0' != str[ len ] ) {
            rs232_putc( str[ len ] );
            len++;
        }
    }
    return len;
}

/* ������������ ������ ������ � ���������
 * (in) str - ��������� �� ������
 * (in) len - ���������� �������� � ������, �� ������� ������ ����� ������
 */
void rs232_str_out(
    const char_t str[], uint_t len )
{
    if ( len != rs232_puts( str ) ) {
        /* ����� �������� ���� �� ������������� ������������ ����� �������� �
           ������ */
        MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_OTHER;
    }
}

/* ������������ ������ ����� � ���������
 * (in) num  - �����
 * (in) base - ��������� ������� ���������
 */
void rs232_num_out(
    uint32_t num, uint_t base )
{
    char_t          str[ 129 ];
    uint_t          len;

    if ( NO_ERROR != u32toa( num, str, sizeof( str ), base, &len ) ) {
        /* ������ �������������� ����� � ������ */
        MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_OTHER;
    } else {
        /* ������ */
        rs232_str_out( str, len );
    }
}
