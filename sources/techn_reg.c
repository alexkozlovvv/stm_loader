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
/* Задание местоположения идентификаторов */
#include "techn_reg_mem.h"

/* Параметры команд */
#define CMD_NUMBER         ( 7U ) /* количество команд */
#define MAX_CMD_LEN       ( 10U ) /* максимальный размер команды */


/* Указатель на заголовок образа */
extern image_header_t  *main_ptr_image_header;

/* Информационные сообщения */
static LOADER_CONST const char_t tech_str_bios[] =
    "\r\n\r\nМодуль МУ-1 ИВУК.468332.122"
    "\r\nПрограмма начальной инициализации (Инд) ИВУК.00461-01"
    "\r\nверсия ";
static LOADER_CONST const char_t tech_str_cs_bios[] =
    ", контрольная сумма = ";
static LOADER_CONST const char_t tech_str_head[] =
    "\r\nТехнологический режим\r\n"
    "\r\nДля выполнения операции введите команду или \"справка\":";
static LOADER_CONST const char_t tech_str_help[] =
    "\r\n\r\nверсия - выдача информации о версии и контрольных суммах"
    "\r\nзагрузка - загрузка НЕХ-файла в ОЗУ"
    "\r\nзапуск - запуск загруженного НЕХ-файла"
    "\r\nвыход - выход из технологического режима"
    "\r\nконтроль - выполнение контроля ОЗУ, ПЗУ, ЦПУ, АЦП"
    "\r\nсправка - информация о командах";
static LOADER_CONST const char_t tech_str_load[] =
    "\r\n\r\nЗагрузка файла формата НЕХ...\r\n";
static LOADER_CONST const char_t tech_str_load_err[] =
    "\r\nЗагрузка файла ОШИБКА";
static LOADER_CONST const char_t tech_str_load_ok[] =
    "\r\nЗагрузка файла УСПЕХ";
static LOADER_CONST const char_t tech_str_unknown[] =
    "\r\n\r\nНеизвестная команда";
static LOADER_CONST const char_t tech_str_enter_point[] =
    "\r\n\r\n.";
static LOADER_CONST const char_t tech_str_empty[] =
    "\r\n";
static LOADER_CONST const char_t tech_str_point[] =
    ".";
static LOADER_CONST const char_t tech_str_0x[] =
    "0x";
static LOADER_CONST const char_t tech_str_cerr[] =
    "\r\n\r\nОбнаружены ошибки при работе с командой";
static LOADER_CONST const char_t single_str_point[] = 
    "\r\n.";

static LOADER_CONST const char_t input_command[] = 
    "\r\nКоманда: ";
static LOADER_CONST const char_t index_of_command[] = 
    "\r\nИндекс команды: ";
static LOADER_CONST const char_t symbol[] = 
    "\r\nСимвол: ";
static LOADER_CONST const char_t space[] = 
    " ";

static LOADER_CONST const char_t commands_list[ CMD_NUMBER ][ MAX_CMD_LEN ] = {
"версия",       /* Вывод информации о программе и контрольной суммы */
"загрузка",     /* Загрузка НЕХ-файла в ОЗУ. Адрес точки входа сохраняется в переменной entry_hex */
"запуск",       /* Запуск загруженного НЕХ-файла */
"выход",        /* Выход из технологического режима */
"контроль",     /* Запуск тестирования ОЗУ, ПЗУ, ЦПУ, АЦП */
"справка",      /* Вывод перечня доступных команд */
"\r"            /* (Enter) Перевод символа приглашения ввода команды на новую строку */
};

/* Подпрограмма останова */
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

/* Подпрограмма технологического монитора */
error_t tech_monitor( void )
{
    uint32_t        entry_hex;                /* Адрес точки входа в программу
                                                 пользователя */
    uint_t          to_exit;                  /* Признак завершения работы */
    error_t         err;                      /* Код ошибки */
    char_t          command_buf[MAX_CMD_LEN]; /* Буфер хранения входной команды */
    int32_t         index;                 /* Индексы */

    /* Задание точки входа по умолчанию */
    entry_hex = ( uint32_t )( &init_startup );

    /* Инициализация канала RS-232 в технологическом режиме */
    if ( NO_ERROR != rs232_init( 115200U, RS232_PARITY_NO, RS232_SB_1,
                                 RS232_BITS_8, RS232_CTS_RTS_NOUSE )
    ) {
        /* Недопустимая ошибка */
        tech_stop();
    }

    /* Вывод сообщения с приглащением ввести команду */
    err = tech_str_out( tech_str_head, sizeof( tech_str_head ) - 1U );

    /* Вывод символа приглащения ввода команды */
    rs232_puts( tech_str_enter_point );
    
    to_exit = 0U;
    
    /* Цикл приема и выполнения команд */
    while ( 0U == to_exit ) {
        
        /* Чтение команды */
		rs232_get_command(command_buf, MAX_CMD_LEN);

        /* Идентификация полученной команды */
		index = find_index_of_command(commands_list, CMD_NUMBER, command_buf);

        switch(index){

            /* Команда "версия" */
            case(0): {
                
                err = NO_ERROR;
                /* Вывод заголовка первоначального загрузчика */
                if ( NO_ERROR != tech_str_out(
                                    tech_str_bios,
                                    sizeof( tech_str_bios ) - 1U )
                ) {
                    err = ERROR;
                /* Вывод номера базовой версии */
                } else if ( NO_ERROR != tech_num_out( VERSION, 10U ) ) {
                    err = ERROR;
                } else if ( NO_ERROR != tech_str_out(                           
                                            tech_str_point,
                                            sizeof( tech_str_point ) - 1U ) ) {
                    err = ERROR;
                /* Вывод номера рабочей версии */
                } else if ( NO_ERROR != tech_num_out( RELEASE, 10U ) ) {
                    err = ERROR;
                /* Вывод контрольной суммы */
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
                /* Проверка наличия ошибок при работе с командой */
                if (NO_ERROR != err) {                
                    ( void ) rs232_puts( tech_str_cerr );
                }                    
                ( void ) rs232_puts( tech_str_enter_point );
                break;
            }
            
            /* Команда "загрузка" */
            case(1): {
                
                err = NO_ERROR;
                if ( NO_ERROR != tech_str_out(
                                    tech_str_load,
                                    sizeof( tech_str_load ) - 1U )
                ) {
                    err = ERROR;
                /* Вывод результата загрузки НЕХ-файла в ОЗУ */    
                } else if ( NO_ERROR != hex_file_load( &entry_hex )
                ) {
                    /* В случае ошибки выставляется точка входа по умолчанию */
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
            
            /* Команда "запуск" */
            case(2): {
                
                /* Пауза перед завершением работы RS-232 */
                delay_ms( 100U );                
                /* Прекращение работы технологического канала RS-232 */
                rs232_close();
                /* Остановка таймера, который обслуживает WDT */
                timer_close();
                /* Bосстановлениe начальной настройки аппаратуры */
                un_init_hardware();
                /* Передача управления программе пользователя или 
                   перезагрузка работы загрузчика */
                ( * ( void (*)( void ) ) entry_hex )();
            }
            
            /* Команда "выход" */
            case(3): {
                
                to_exit = 1U;
                break;
            }
            
            /* Команда "контроль" */
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
            
            /* Команда "справка" */
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
            
            /* Команда "\r" (Enter) */
            case(6): {
                
                ( void ) rs232_puts( single_str_point );
                
                break;
            }
            
            /* Неизвестная команда */
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
    
    /* Пауза перед завершением работы RS-232 */
    delay_ms( 100U );               
    /* Прекращение работы технологического канала RS-232 */
    rs232_close();
    
    return err;
}

/* Подпрограмма определения индекса введенной команды
 * Входные параметры: 
 * cmd_array (in) - массив команд
 * size (in)      - колличество строк массива (количество команд)
 * word (in)      - указатель на полученную команду
 * Результат:
 *  0 - 7         - Индекс команды из массива
 *  -1            - Неизвестная команда
 */
int32_t find_index_of_command(const char_t cmd_array[][ MAX_CMD_LEN ], uint_t size, 
    const char_t* word )
{
	int32_t j,i;
	int32_t len = 0;  
    
    /* Подсчет длины полученной команды */
	while(word[len] != '\0'){
		len++;
	}
	
    /* Посимвольное сравнение каждой команды из массива с полученным словом */
	for(i = 0;i < size; i++){
		for(j = 0; cmd_array[i][j] && word[j]; j++){
			if(cmd_array[i][j] != word[j]){
				break;
			}
		}
		/* Если все символы совпали и одновременно закончились 
           элементы перебора */
		if(j == len && !cmd_array[i][j]){
			return i;
		}
	}
	//Команда не найдена
	return -1;
}

/* Подпрограмма выдачи строки
 * (in) str - указатель на строку
 * (in) len - длина строки
 */
static error_t tech_str_out(
    const char_t str[], uint_t len )
{
    error_t         err;

    if ( len != rs232_puts( str ) ) {
        /* Число выданных байт не соответствует контрольному числу
         * символов в строке */
        err = ERROR;
    } else {
        err = NO_ERROR;
    }

    return err;
}

/* Подпрограмма выдачи числа
 * (in) num  - число
 * (in) base - основание системы счисления
 */
static error_t tech_num_out(
    uint32_t num, uint_t base )
{
    char_t          str[ 129 ];
    uint_t          len;
    error_t         err;

    if ( NO_ERROR != u32toa( num, str, sizeof( str ), base, &len ) ) {
        /* Ошибка преобразования числа в строку */
        err = ERROR;
    } else {
        /* Выдача */
        err = tech_str_out( str, len );
    }

    return err;
}
