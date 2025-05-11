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

/* Технологическая разовая команда КТА */
#define MAIN_TECH_RK_KTA                     0x00000002U
#define MAIN_TECH_RK_BLCK_WDT                0x00000008U
#define MAIN_TECH_RK_KTA_AND_BLCK_WDT        0x0000000AU

/* Константы для признака повторного включения */
#define MAIN_FIRST_0            ( 0x5555AAAAU )
#define MAIN_FIRST_1            ( 0xAAAA5555U )
#define MAIN_FIRST_2            ( 0xFFFF0000U )
#define MAIN_FIRST_3            ( 0x0000FFFFU )

/* Информационные сообщения */
static LOADER_CONST const char_t main_str_bios[] =
    "\r\n\r\n\r\nИВУК.00461-01";
static LOADER_CONST const char_t main_str_ver[] =
    "\r\nВерсия ";
static LOADER_CONST const char_t main_str_cs_bios[] =
    ", контрольная сумма = ";
static LOADER_CONST const char_t main_str_cs_user[] =
    ", контрольная сумма = ";
static LOADER_CONST const char_t main_str_rst[] =
    "Счетчик сбросов = ";
static LOADER_CONST const char_t main_str_work[] =
    "\r\n\r\nЗапуск бортового ПО места ";
static LOADER_CONST const char_t main_str_bad_head[] =    
    "\r\nНедопустимое содержимое заголовка бортового ПО";
static LOADER_CONST const char_t main_str_empty[] =
    "\r\nБортовое ПО не записано в ПЗУ";
static LOADER_CONST const char_t main_str_enter[] =
    "\r\n";
static LOADER_CONST const char_t main_str_point[] =
    ".";
static LOADER_CONST const char_t main_str_0x[] =
    "0х";
static LOADER_CONST const char_t main_str_cerr[] =
    "\r\n\r\nОбнаружены ошибки в работе загрузчика";
static LOADER_CONST const char_t main_str_mend[] =
    "\r\nТехнологический монитор завершил работу";
static LOADER_CONST const char_t main_wdt_error[] =
    "\r\nОбнаружены ошибки при тестировании WDT";
static LOADER_CONST const char_t main_wdt_no_test[] =
    "\r\nПроверка WDT была пропущена";
static LOADER_CONST const char_t main_str_bad_ks_head[] =
    "\r\nОшибка проверки КС заголовка образа загрузки";
static LOADER_CONST const char_t main_rkp_pos_err[] =
    "\r\nОшибка при получении РКП кода места";
static LOADER_CONST const char_t main_str_no_user[] =
    "\r\nНе найден образ ПО, соответствующий КП";
static LOADER_CONST const char_t main_str_locat[] =
    "\r\nДля места ";
static LOADER_CONST const char_t main_str_chan_0[] =
    " канала ИНД";
static LOADER_CONST const char_t main_str_user_fail[] =    
    "\r\nОбнаружены ошибки в процессе передачи управления \r\n"
    "пользовательской программе";
static LOADER_CONST const char_t main_str_error_start_test[] =    
    "\r\n\r\nОбнаружены ошибки контроля при включении\r\n";
static LOADER_CONST const char_t test_str_cerr[] =
    "\r\n\r\nОбнаружены прочие ошибки в работе загрузчика";

/* Инструкция для размещения необходимого значения в интересующую ячейку памяти. Используется для отладки */
// const uint32_t cs __attribute__ ((at(0x0800BFFC)))= 0x232BF049;  

/* Точка входа в подпрограмму диспетчера программы начальной инициализации.
 * Подпрограмма предназначена для определения режима работы 
 * (рабочий или технологический), проведения стартового контроля и
 * запуска рабочей программы в ПЗУ
 */
LOADER_FUNC int32_t main( void )
{
    uint32_t        rk_kta;        
    uint32_t        a0, a1, a2, a3;   /* Вспомогательные переменные */
    
    /* Инициализация канала RS-232.
     * Пояснения к настройкам: отсутствует контроль четности, 
       один стоп бит, длина слова данных 8 бит, 
       отсутсвует аппаратный контроль потока CTS/RTS */
    if ( NO_ERROR != rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,
                                 RS232_BITS_8, RS232_CTS_RTS_NOUSE )
    ) {
        main_stop();
    }
    
    /* Инициализация таймера сброса WDT и мигания светодиодом. 
       Период срабатывания 10 мс */
    timer_init( TIME_STEP_10, TIMER_NORMAL, NULL );

    /* Чтение регистра входных данных порта А */
    rk_kta = IN_REG( STM32F407_GPIOA_PTR->idr );
    
    /* При установке РК КТА и Блок. WDT в состояние "наличие команды" - переход в
     * технологический режим, иначе - переход в штатный режим работы */
    if ( (rk_kta & (MAIN_TECH_RK_KTA)) != 0U) {               
        
        /* Запуск технологического монитора */
        main_exit_monitor();
        
    } else {
        
        /* Инициализация и пуск сторожевого таймера */
        timer_wdt_init();       
        
        /* Считывание признака повторного включения */
        a0 = MPM_LOADER_OUT_PTR->first[ 0U ];                   
        a1 = MPM_LOADER_OUT_PTR->first[ 1U ];
        a2 = MPM_LOADER_OUT_PTR->first[ 2U ];
        a3 = MPM_LOADER_OUT_PTR->first[ 3U ];

        /* Проверка признака повторного включения */
        if (    ( a0 != MAIN_FIRST_0 ) || ( a1 != MAIN_FIRST_1 )
             || ( a2 != MAIN_FIRST_2 ) || ( a3 != MAIN_FIRST_3 )
        ) {
            /* Первое включение */ 
            main_power_on();
            
        } else {
            /* Перезапуск */        
            main_reset();
        }
    }
    return ERROR;
}

/* Подпрограмма останова */
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

/* Подпрограмма отработки первого включения */
static LOADER_FUNC void main_power_on( void )
{
    /* Сброс результатов стартового контроля */
    MPM_LOADER_OUT_PTR->test_result = 0U;
    
    /* Сброс счетчика перезапусков */
    MPM_LOADER_OUT_PTR->cnt_reset = 0U;
    
    /* Установка признака отсутствия бортового ПО */
    MPM_LOADER_OUT_PTR->image_header_addr = 0U;

    /* Вывод децимального номера программы начальной инициализации */
    /* ИВУК.00461-01 */
    rs232_str_out( main_str_bios, ( sizeof( main_str_bios ) - 1U ) );
    /* Вывод версии программы начальной инициализации */
    /* Версия VERSION.RELEASE */
    rs232_str_out( main_str_ver, ( sizeof( main_str_ver ) - 1U ) );
    rs232_num_out( VERSION, 10U );
    rs232_str_out( main_str_point, ( sizeof( main_str_point ) - 1U ) );
    rs232_num_out( RELEASE, 10U );
    /* Вывод контрольных сумм */
    /* , контрольная сумма = 0xCS_LOADER */ 
    rs232_str_out( main_str_cs_bios, ( sizeof( main_str_cs_bios ) - 1U ) );
    rs232_str_out( main_str_0x, ( sizeof( main_str_0x ) - 1U ) );
    rs232_num_out( CS_LOADER, 16U );
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );

    /* Установка признака повторного включения */
    MPM_LOADER_OUT_PTR->first[ 0U ] = MAIN_FIRST_0;
    MPM_LOADER_OUT_PTR->first[ 1U ] = MAIN_FIRST_1;
    MPM_LOADER_OUT_PTR->first[ 2U ] = MAIN_FIRST_2;
    MPM_LOADER_OUT_PTR->first[ 3U ] = MAIN_FIRST_3;

    /* ------- Стартовый контроль ------- */

    /* Контроль ОЗУ */
    test_start_ram( TEST_MODE_START );

    /* Контроль ЦПУ */
    test_start_cpu( TEST_MODE_START );

    /* Конроль ПЗУ */
    test_start_rom( TEST_MODE_START );
    
    /* Конроль АЦП */
    test_start_adc( TEST_MODE_START );
    
    /* Контроль сторожевого таймера */
    test_start_wdt( TEST_MODE_START );
    
}

/* Подпрограмма отработки последующих включений */
static LOADER_FUNC void main_reset( void )
{
    image_header_t      image_header;  /* структуру заголовка */
    
    /* Наращивание счетчика перезапусков */
    if ( MAX_UINT32 != MPM_LOADER_OUT_PTR->cnt_reset ) {
        MPM_LOADER_OUT_PTR->cnt_reset++;
    }

    /* При первом перезапуске, если причиной перезапуска является тест WDT
       (то есть тест WDT успешно выполнен), то необходимо выдать сообщение
       об успешном выполнении теста WDT */
    if ( MPM_LOADER_OUT_PTR->cnt_reset != 1U ) {
        /* Не первый перезапуск */
        ;
    } else if ( 0U != ( MPM_LOADER_OUT_PTR->test_result 
                        & ( ~ ( uint32_t ) TEST_RESULT_ERR_OTHER ) )
    ) {
        /* Обнаружены ошибки при начальном запуске */
        ;
    } else {
        /* Выдача сообщения об успешной проверке WDT */
        test_start_wdt_end();
        
        if ( 0U != ( MPM_LOADER_OUT_PTR->test_result & TEST_RESULT_ERR_OTHER ) ) {
        /* Обнаружены прочие ошибки */
            rs232_str_out( test_str_cerr, sizeof( test_str_cerr ) - 1U );           /*  "\r\nОбнаружены ошибки в работе загрузчика"  */
        }
    }
    
    /* Вывод децимального номера первоначального загрузчика */
    rs232_str_out( main_str_bios, ( sizeof( main_str_bios ) - 1U ) );

    /* Вывод счетчика перезапусков */
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );
    rs232_str_out( main_str_rst, ( sizeof( main_str_rst ) - 1U ) );
    rs232_num_out( MPM_LOADER_OUT_PTR->cnt_reset, 10U );
    
    /* Вывод версии первоначального загрузчика */
    rs232_str_out( main_str_ver, ( sizeof( main_str_ver ) - 1U ) );          /*  "\r\nВерсия " */
    rs232_num_out( VERSION, 10U );                                           /* преобразование числа в строку и далее вывод строки по RS */
    rs232_str_out( main_str_point, ( sizeof( main_str_point ) - 1U ) );      /* "." */
    rs232_num_out( RELEASE, 10U );                                           /* вывод номер сборки */
    
    /* Вывод контрольной суммы */
    rs232_str_out( main_str_cs_bios, ( sizeof( main_str_cs_bios ) - 1U ) );  /* ", контрольная сумма = " */
    rs232_str_out( main_str_0x, ( sizeof( main_str_0x ) - 1U ) );            /* "0x" */
    rs232_num_out( CS_LOADER, 16U );                                         /* контрольная сумма */
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );      /* "\r\n" */
    
    /* Проверка заголовка бортового ПО */   
    if ( NO_ERROR != main_locaton_detect( &image_header ) ) {
        /* Ошибка определения загружаемого образа ПО */
        /* Переход в технологический режим */
        main_exit_monitor();
    } else {
        main_exit_user(&image_header);
    }
}

/* Подпрограмма выхода в технологический монитор */
static LOADER_FUNC void main_exit_monitor( void )
{
    uint32_t        i;
    
    /* Вывод децимального номера программы начальной инициализации */
    rs232_str_out( main_str_bios, ( sizeof( main_str_bios ) - 1U ) );        /* "\r\n\r\n\r\nИВУК.00461-01" */
    /* Вывод версии первоначального загрузчика */
    rs232_str_out( main_str_ver, ( sizeof( main_str_ver ) - 1U ) );          /*  "\r\nВерсия " */
    rs232_num_out( VERSION, 10U );                                           /* вывод номера версии */
    rs232_str_out( main_str_point, ( sizeof( main_str_point ) - 1U ) );      /* "." */
    rs232_num_out( RELEASE, 10U );                                           /* вывод номер сборки */
    /* Вывод контрольной суммы */
    rs232_str_out( main_str_cs_bios, ( sizeof( main_str_cs_bios ) - 1U ) );  /* ", контрольная сумма = " */
    rs232_str_out( main_str_0x, ( sizeof( main_str_0x ) - 1U ) );            /* "0x" */
    rs232_num_out( CS_LOADER, 16U );                                         /* вывод значения КС */
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );      /* "\r\n" */

    /* Прекращение работы технологического канала RS-232 */
    for ( i = 0; i < 1000000U; i++ ) {
        /* Пауза перед завершением работы RS-232 */
        ;
    }
    rs232_close();

    /* Передача управления технологическому монитору */
    if ( NO_ERROR != tech_monitor() ) {
        /* Если произошел выход с ошибкой из технологического монитора,
           то необходимо остановить процесс */
        ( void ) rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,          
                             RS232_BITS_8, RS232_CTS_RTS_NOUSE );
        rs232_str_out( main_str_cerr, sizeof( main_str_cerr ) - 1U );       /* "\r\nОбнаружены ошибки в работе загрузчика" */
        main_stop();
    } else {
        /* Выход без ошибки. Перезапуск */
        ( void ) rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,
                             RS232_BITS_8, RS232_CTS_RTS_NOUSE );
        rs232_str_out( main_str_mend, sizeof( main_str_mend ) - 1U );       /* "\r\nТехнологический монитор завершил работу" */
        for ( i = 0; i < 1000000U; i++ ) {
            /* Пауза перед завершением работы RS-232 */
            ;
        }
        rs232_close();
        /* Перезапуск */
        irq_reset_entry();
    }    
}

/* Подпрограмма выхода в бортовую программу */
static LOADER_FUNC void main_exit_user( image_header_t* image_header )
{
    uint32_t        addr, i;
    
    /* Вывод сообщения о запуске бортового ПО */
    rs232_str_out( main_str_work, ( sizeof( main_str_work ) - 1U ) );           /* "\r\n\r\nЗапуск бортового ПО места " */
    rs232_num_out( image_header->location, 10U );
    /* Вывод контрольных сумм */
    rs232_str_out( main_str_cs_user, ( sizeof( main_str_cs_user ) - 1U ) );
    rs232_str_out( main_str_0x, ( sizeof( main_str_0x ) - 1U ) );
    rs232_num_out( CS_USER, 16U );
    rs232_str_out( main_str_enter, ( sizeof( main_str_enter ) - 1U ) );         /* "\r\n" */

    /* Выдача сообщения, если в процессе работы программы были
       обнаружены ошибки */
    if ( 0U != ( MPM_LOADER_OUT_PTR->test_result & TEST_RESULT_ERR_OTHER ) ) {
        rs232_str_out( main_str_cerr, sizeof( main_str_cerr ) - 1U );           /* "\r\nОбнаружены ошибки в работе загрузчика" */
    }
    /* Прекращение работы технологического канала RS-232 */
    for ( i = 0; i < 1000000U; i++ ) {
        /* Пауза перед завершением работы RS-232 */
        ;
    }
    rs232_close();
    /* Остановка таймера, который обслуживает WDT */
    timer_close();
    /* Bосстановлениe начальной настройки аппаратуры */
    un_init_hardware();
    /* Передача управления программе пользователя */
    addr = image_header->entry_address;
    ( ( void (*)( void ) ) addr ) ();
    
}

/* Подпрограмма считывания заголовка рабочей программы
 *  (in)  start_img_address  - адрес начала заголовка пользовательской программы
 *  (out) dest               - указатель на структуру заголовка
 * Результат:
 *  INVALID_PARAM    - Ошибка. Неправильный адрес области заголовка.
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

/* Подпрограмма считывания заголовка образа загрузки
 *  (out) dest      - считанный заголовок
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

/* Подпрограмма проверки заголовка бортового ПО
 *  (in/out)  ptr_image_header  - указатель на структуру заголовка образа загрузки
 * Результат:
 *  NO_ERROR         - Успех
 *  ERROR            - Ошибка
 */
static LOADER_FUNC error_t main_locaton_detect(
    image_header_t *ptr_image_header )
{
    load_header_t   load_header;           /* Заголовок образа загрузки         */
    uint32_t        load_header_cs;        /* Контрольная сумма образа загрузки */
    uint32_t        image_header_addr;     /* Текущий адрес                     */
    sz_t            load_header_full_size; /* Размер заголовка образа загрузки,
                                              включая заголовки образов ПО      */
    uint32_t        pos_code;              /* Входные значения выводов порта D */
//    uint32_t        pos_code1;             /* Код позиционирования первой пары РКП */
//    uint32_t        pos_code2;             /* Код позиционирования второй пары РКП */
    uint32_t        parity;                /* Бит четности */ 
    uint32_t        temp;
    error_t         err;
    stm32f407_gpio_t *ptr_reg_gpio;
    
    ptr_reg_gpio = STM32F407_GPIOD_PTR;

    /* Проверка наличия ранее выбранного заголовка пользовательского ПО */
    image_header_addr = MPM_LOADER_OUT_PTR->image_header_addr;
    load_header_full_size = 0U;

    if ( 0U == image_header_addr ) {
        /* Нет ранее выбранного заголовка образа ПО */
        err = NO_ERROR;
    } else {
        /* Есть ранее выбранный заголовок образа ПО */
        err = NO_ACTION;
    }

    /* Проверка заголовка образа загрузки */
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
            /* Отсутствие образа загрузки */
            rs232_str_out( main_str_empty,
                           ( sizeof( main_str_empty ) - 1U ) );
        } else {
            /* Недопустимое содержимое заголовка */
            rs232_str_out( main_str_bad_head,
                           ( sizeof( main_str_bad_head ) - 1U ) );
        }
        /* Переход в технологический режим */
        main_exit_monitor();
    } else if ( NO_ACTION == err ){
        /* Есть ранее выбранный образ ПО.
           Расширенная проверка заголовка образа загрузки не требуется */
        err = NO_ERROR;
    } else if ( NO_ERROR != test_rom_pass(
        /* Нет ранее выбранного образа ПО */
         ( uint8_t * )( CROM_START ),
         load_header_full_size,
         load_header_cs )
    ) {
        /* Ошибка проверки КС заголовка образа загрузки */
        rs232_str_out( main_str_bad_ks_head,
                       ( sizeof( main_str_bad_ks_head ) - 1U ) );
        err = ERROR;
    } else {
        /* Успешная проверка КС */
        err = NO_ERROR;
    }

    /* Считывание внешних разовых команд признака места */
    if ( NO_ERROR != err ) {
        /* Обнаружены ошибки */
        ;
    } else {
        /* Модуль ЭБМК */
        /* Чтение кода признака места из регистра порта D  */
        pos_code = IN_REG( ptr_reg_gpio -> idr ) & 0x7U;
//        pos_code1 = pos_code & 0x7U;
//      pos_code2 = (pos_code & 0x38U) >> 3;
//        if ( 0U != (pos_code1 ^ pos_code2)) {       
//            rs232_str_out( main_rkp_pos_err,
//                       ( sizeof( main_rkp_pos_err ) - 1U ) );
//            err = ERROR;
//        }
//        else {
            /* Проверка бита четности */
            parity  = ( 0U != ( pos_code & MCU_GPIO_IDR_IDR0 ) ) ? MCU_GPIO_IDR_IDR2 : 0U;
            parity ^= ( 0U != ( pos_code & MCU_GPIO_IDR_IDR1 ) ) ? MCU_GPIO_IDR_IDR2 : 0U;
            parity ^= pos_code & MCU_GPIO_IDR_IDR2;
            if ( 0U != ( parity & MCU_GPIO_IDR_IDR2 ) ) {         /* !!!!! Изменено условие проверки бита четности. В процессе штатной работы должно быть != */
                /* Контроль пройден */
                pos_code &= MCU_GPIO_IDR_IDR0 | MCU_GPIO_IDR_IDR1;
                err = NO_ERROR;
            } else {
                /* Контроль не пройден */
                rs232_str_out( main_rkp_pos_err, ( sizeof( main_rkp_pos_err ) - 1U ) );
                err = ERROR;
//            } 
        }     
    }

    /* Поиск заданного образа ПО */
    if ( NO_ERROR != err) {
        /* Обнаружены ошибки */
        ;
    } else if ( 0U != image_header_addr) {
        /* Есть ранее выбранный образ ПО */
        ;
    } else {
        /* Выбор образа ПО */
        /* Начало первого из образов ПО */
        image_header_addr = CROM_START + sizeof( load_header_t );
        /* Поиск образа соответствующего признака места */
        err = NO_ACTION;
        do {
            temp = u32_swap(( ( image_header_t * ) image_header_addr )->location);
            /* Выход из цикла, если нужный образ найден 
               или все образы просмотрены  */
            if ( temp == pos_code ) {
                /* Образ найден */
                err = NO_ERROR;
            } else {
                /* Адрес следующего заголовка образа ПО */
                image_header_addr += sizeof( image_header_t );
                if ( image_header_addr >= ( CROM_START
                                          + load_header_full_size )
                ) {
                    /* Все образы просмотрены */
                    break;
                } else {
                    /* Продолжение просмотра образов */
                    ;
                }
            }
        } while ( NO_ERROR != err );
    }

    /* Считывание заголовка образа ПО */
    if ( NO_ERROR != err ) {
        /* Обнаружены ошибки */
        ;
    } else if ( NO_ERROR != main_read_image_header(
                                ( uint32_t ) image_header_addr,
                                ptr_image_header )
    ) {
        /* Обнаружены ошибки */
        err = ERROR;
    } else if ( ptr_image_header->location != pos_code ) {             
        /* Образ ПО не соответствует коду позиционирования */
        err = NO_ACTION;
    } else {
        /* Успешное выполнение */
        ;
    }

    if ( NO_ACTION == err ) {
        /* Не найден образ ПО соответствующий коду позиционирования */
        rs232_str_out( main_str_no_user, ( sizeof( main_str_no_user ) - 1U ) );
        rs232_str_out( main_str_locat, ( sizeof( main_str_locat ) - 1U ) );
        rs232_num_out( pos_code, 10U );
        rs232_str_out( main_str_chan_0, ( sizeof( main_str_chan_0 ) - 1U ) );
        MPM_LOADER_OUT_PTR->image_header_addr = 0U;
        err = ERROR;
    } else if ( NO_ERROR != err ){
        /* Обнаружены ошибки */
        rs232_str_out( main_str_user_fail, ( sizeof( main_str_user_fail ) - 1U ) );
        MPM_LOADER_OUT_PTR->image_header_addr = 0U;
    } else {
        /* Успешное выполнение */
        MPM_LOADER_OUT_PTR->image_header_addr = image_header_addr;
    }

    return err;
}

