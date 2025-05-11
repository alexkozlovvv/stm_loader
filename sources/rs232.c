#include "sections_bsp.h"
#include "base_types.h"
#include "os_def.h"
#include "eumw.h"
#include "STM32F407.h"
#include "irq.h"
#include "itoa.h"
#include "rs232.h"
/* Задание местоположения идентификаторов */
#include "rs232_mem.h"

/* Константа для расчёта скорости обмена в канале */
#define RS232_BAUD_BASE         ( MCU_FREQ_APB1 )

/* Признак работоспособности канала RS-232 */
static LOADER_DATA uint_t rs232_can_work;

/* Подпрограмма инициализации канала RS-232
 * (in) baud       - скорость работы канала USART (бод)
 * (in) parity     - режим работы с битом четности
 * (in) stop_bits  - количество стоп битов
 * (in) width      - длина слова данных (бит)
 * (in) hw_cntr    - признак аппаратного контроля потока CTS/RTS
 * Результат:
 * NO_ERROR        - Успех
 * INVALID_CONFIG  - Ошибка. Неправильно заданные входные настройки.
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
    OUT_REG( STM32F407_NVIC_PTR->icer[1], CORE_NVIC_ISER_USART1 );  // Отключаем внешнее прерывание по USART1
    OUT_REG( STM32F407_NVIC_PTR->icpr[1], CORE_NVIC_ISER_USART1 );  // Сброс ожидающего обработки внешнего прерывания по USART1

    temp  = IN_REG( STM32F407_RCC_PTR->apb2enr );
    temp |= MCU_RCC_APB2ENR_USART1EN;
    OUT_REG( STM32F407_RCC_PTR->apb2enr, temp );                    // Включаем тактирование USART1
    temp  = IN_REG( STM32F407_RCC_PTR->apb2rstr );
    temp &= ~ ( uint32_t ) MCU_RCC_APB2RSTR_USART1RST;
    OUT_REG( STM32F407_RCC_PTR->apb2rstr, temp );	                // Отключаем reset для USART1	
    temp  = IN_REG( ptr_reg->cr1 );
    temp &= ~ ( uint32_t ) MCU_USART_CR1_UE;
    OUT_REG( ptr_reg->cr1, temp );                                  // Отключаем USART
    kern_irq_set_msr( msr_old );                                    // Разрешаем прерывания глобально

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
        case RS232_PARITY_NO: {                                     // в случае отсутствия контроля бита четности сбрасываем соответствующий бит
            cr1 &= ~ ( uint32_t ) MCU_USART_CR1_PCE;
            break;
        }
        case RS232_PARITY_EVEN: {
            cr1 |= MCU_USART_CR1_PCE;                               // выставляется бит разрешения контроля четности
            cr1 &= ~ ( uint32_t ) MCU_USART_CR1_PS;                 // проверка по принципу четности
            break;
        }
        case RS232_PARITY_ODD: {
            cr1 |= MCU_USART_CR1_PCE;                               // выставляется бит разрешения контроля четности
            cr1 |= MCU_USART_CR1_PS;                                // проверка по принципу нечетности
            break;
        }
        default: {
            err = INVALID_CONFIG;
            break;
        }
    }

    switch ( stop_bits ) {
        case RS232_SB_1: {
            cr2 &= ~ ( uint32_t ) MCU_USART_CR2_STOP_MASK;          // выставляем 1 стоп-бит 
            break;
        }
        case RS232_SB_2: {
            cr2 |= MCU_USART_CR2_STOP_2;                            // выставляем 2 стоп-бита
            break;
        }
        default: {
            err = INVALID_CONFIG;
            break;
        }
    }

    switch ( width ) {
        case RS232_BITS_8: {
            cr1 &= ~ ( uint32_t ) MCU_USART_CR1_M;                  // указываем размер слова дынных 8 бит
            break;
        }
        case RS232_BITS_9: {
            cr1 |= MCU_USART_CR1_M;                                 // указываем размер слова дынных 9 бит
            break;
        }
        default: {
            err = INVALID_CONFIG;
            break;
        }
    }

    if ( 0U != hw_cntr ) {                                          // Проверка на выставку режима аппаратного управления потоком
        cr3 = MCU_USART_CR3_CTSE | MCU_USART_CR3_RTSE;
        /* Разработчик аппаратуры не реализовал поддержку данного режима */
        err = INVALID_CONFIG;
    } else {
        cr3 &= ~ ( uint32_t ) ( MCU_USART_CR3_CTSE | MCU_USART_CR3_RTSE );      // если не выставлен режим то тогда обнуляем биты
    }

    /* Configure usart1 */
    if ( NO_ERROR != err ) {
        ;
    } else {
        OUT_REG( ptr_reg->brr, ( brr_m << 4 ) | ( ( brr_f >> 12 ) & 0xF ) );    // выставляем коэффициенты для скорости работы канала
        OUT_REG( ptr_reg->cr1, cr1 );                                           // выставляем регистры
        OUT_REG( ptr_reg->cr2, cr2 );
        OUT_REG( ptr_reg->cr3, cr3 );

        cr1 |= MCU_USART_CR1_UE | MCU_USART_CR1_TE | MCU_USART_CR1_RE;          // разрешаем работу USART, приемника и передатчика
        OUT_REG( ptr_reg->cr1, cr1 );

        
        /* Обнаружено, что в некоторых ситуациях необходимо подождать
           некоторое время перед началом выдачи. Иначе символы могут
           выдаваться не корректно */
        for ( cnt = 0U; cnt < 100000U; cnt++ ) {
            ;
        }

    }
    
    /* Если неправильно инициализирован канал, то запретить его дальнейшее использование */
    if ( NO_ERROR == err ) {
        rs232_can_work = ~ ( uint_t ) 0U;
    } else {
        rs232_can_work = 0U;
    }

    return err;
}

/* Подпрограмма прекращения работы канала RS-232 */
void rs232_close( void )
{
    stm32f407_usart_t *ptr_reg;
    uint32_t        temp;
    uint32_t        msr_old;

    if ( 0U == rs232_can_work ) {
        ;
    } else {
        /* Последовательность выключения ДОЛЖНА быть такой */
        ptr_reg = STM32F407_USART1_PTR;
        msr_old = kern_irq_disable_interrupt(); 
        while ( MCU_USART_SR_TC != ( IN_REG( ptr_reg->sr ) & MCU_USART_SR_TC ) ) {          // ждем когда закончится передачи фреймов сообщений
            ;
        }
        temp  = IN_REG( ptr_reg->cr1 );
        temp &= ~ ( uint32_t ) ( MCU_USART_CR1_UE | MCU_USART_CR1_TE | MCU_USART_CR1_RE );
        OUT_REG( ptr_reg->cr1, temp );                                                      // обнуляем разрешения на работы приемника и перердатчика, а так же самого USART

        kern_irq_set_msr( msr_old );                            // восстанавливаем глобальное разрешение прерываний

        msr_old = kern_irq_disable_interrupt();
        temp  = IN_REG( STM32F407_RCC_PTR->apb2enr );
        temp &= ~ ( uint32_t ) MCU_RCC_APB2ENR_USART1EN;        // отключаем тактирование блока USART1
        OUT_REG( STM32F407_RCC_PTR->apb2enr, temp );
        temp  = IN_REG( STM32F407_RCC_PTR->apb2rstr );
        temp |= MCU_RCC_APB2RSTR_USART1RST;                     // устанавливаем USART1 в состояние ресет
        OUT_REG( STM32F407_RCC_PTR->apb2rstr, temp );
        kern_irq_set_msr( msr_old );                            // разрешаем глобальные прерывания
    }
}

/* Подпрограмма выдачи символа в канал RS-232
 * (in) c - выдаваемый символ
 */
void rs232_putc(
    char_t c )
{
    stm32f407_usart_t *ptr_reg;

    if ( 0U == rs232_can_work ) {
        ;
    } else {
        ptr_reg = STM32F407_USART1_PTR;
        /* Ожидание готовности буфера передатчика */
        while ( MCU_USART_SR_TXE != ( IN_REG( ptr_reg->sr ) & MCU_USART_SR_TXE ) ) {    // ожидаем когда символ будет перемещен из Transmit data register в shift register
            ;
        }

        ptr_reg->dr = ( uint8_t ) c;  // закидываем символ в Data register затем он сам переместится в output shift register
    }
}

/* Подпрограмма приема символа из канала RS-232
 * (in) c - указатель на переменную, в которой будет сохранен
 *          принятый символ
 */
void rs232_getc(
    char_t *c )
{
    stm32f407_usart_t *ptr_reg;

    if ( 0U == rs232_can_work ) {
        *c = '\x0';
    } else {
        ptr_reg = STM32F407_USART1_PTR;
        while ( MCU_USART_SR_RXNE != ( IN_REG( ptr_reg->sr ) & MCU_USART_SR_RXNE ) ) {  // ожидаем когда выставится флаг Received data is ready to be read. 
            /* Ожидание готовности буфера приёмника */
            ;
        }
        *c = ( char_t ) ptr_reg->dr;        // считываем данные
    }
}

/* Подпрограмма приема команды из канала RS-232 для работы с терминалом
 * (in/out) buffer - указатель на буффер, в который будет сохранена принятая команда
 * (in) max_size   - максимальный размер буфера принятой команды 
 */
void rs232_get_command(char_t* buffer, int32_t max_size)
{    
	char_t symb;
	int32_t len = 0;                  //Индекс элемента введенной команды
	
	if ( 0U == rs232_can_work ) {
        ;
    } else {
        while(len < (max_size - 1)) { // последний элемент - конец строки	
            rs232_getc( &symb );		
            if(symb == '\r') {
                if(len == 0) {
                    buffer[len++] = symb;
                }
                /* символ Enter - окончание приема команды */
                break;
            }
            else if(symb == 0x08){  /* символ backspace */
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
        // добавить символ конца строки
        if (len < max_size) {
            buffer[len] = '\0';
        } else {
            buffer[max_size - 1] = '\0';
        }
    }
}

/* Подпрограмма выдачи строки символов в канал RS-232
 *  (in) str        - указатель на выдаваемую строку символов
 * Результат:
 *  len             - количество выданных символов
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

/* Подпрограмма выдачи строки с контролем
 * (in) str - указатель на строку
 * (in) len - количество символов в строке, не включая символ конца строки
 */
void rs232_str_out(
    const char_t str[], uint_t len )
{
    if ( len != rs232_puts( str ) ) {
        /* Число выданных байт не соответствует контрольному числу символов в
           строке */
        MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_OTHER;
    }
}

/* Подпрограмма выдачи числа с контролем
 * (in) num  - число
 * (in) base - основание системы счисления
 */
void rs232_num_out(
    uint32_t num, uint_t base )
{
    char_t          str[ 129 ];
    uint_t          len;

    if ( NO_ERROR != u32toa( num, str, sizeof( str ), base, &len ) ) {
        /* Ошибка преобразования число в строку */
        MPM_LOADER_OUT_PTR->test_result |= TEST_RESULT_ERR_OTHER;
    } else {
        /* Выдача */
        rs232_str_out( str, len );
    }
}
