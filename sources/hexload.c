#include "sections_bsp.h"

#include "base_types.h"
#include "os_def.h"

#include "rs232.h"
#include "hexload.h"
#include "hexload_mem.h"

//#define MAX_REC_LENG  256U

/* Коды ошибок загрузки HEX-файла */
#define HEX_LOAD_ERR_BASE       ( -100 )
/* .  успешно */
#define HEX_LOAD_OK             ( 0 )
/* i информационная секция */
#define HEX_LOAD_INFO           ( HEX_LOAD_ERR_BASE - 1 )
/* z обнаружена пустая запись */
#define HEX_LOAD_NULL           ( HEX_LOAD_ERR_BASE - 2 )
/* l ошибка формата. Недопустимое значение поля Byte Count */
#define HEX_LOAD_ERR_LENG       ( HEX_LOAD_ERR_BASE )
/* n ошибка формата. Короткая запись */
#define HEX_LOAD_ERR_SHORT      ( HEX_LOAD_ERR_BASE - 3 )
/* s ошибка формата. Обнаружена неформатная литера */
#define HEX_LOAD_ERR_SYMB       ( HEX_LOAD_ERR_BASE - 4 )
/* a прервано оператором */
#define HEX_LOAD_ERR_ABORT      ( HEX_LOAD_ERR_BASE - 5 )
/* b буффер переполнен */
#define HEX_LOAD_ERR_BUFF       ( HEX_LOAD_ERR_BASE - 6 )
/* w сбой памяти */
#define HEX_LOAD_ERR_STOR       ( HEX_LOAD_ERR_BASE - 7 )
/* c ошибка контр. суммы */
#define HEX_LOAD_ERR_CSUM       ( HEX_LOAD_ERR_BASE - 8 )
/* r ошибка не обнаружен признак конца записи */
#define HEX_LOAD_ERR_END        ( HEX_LOAD_ERR_BASE - 9 )
/* t ошибка. обнаружен не поддерживаемый тип записи */
#define HEX_LOAD_ERR_TYPE       ( HEX_LOAD_ERR_BASE - 10 )

static LOADER_CONST const char_t hex_load_legend[] =
  "Прогресс:\r\n"
  "'и' - информационная запись\r\n"
  "'.' - запись с данными\r\n"
  "'#' - прерывающая запись\r\n"
  "'з' - пустая запись\r\n"
  "Ошибки при обработке записи:\r\n"
  "'л' - недопустимое значением поля Byte Count\r\n"
  "'н' - длина записи не соответствет требуемому\r\n"
  "'с' - обнаружен недопустимый символ\r\n"
  "'а' - получена команда прерывания загрузки\r\n"
  "'б' - длина записи превышает допустимую\r\n"
  "'в' - ошибка записи данных в память\r\n"
  "'к' - ошибка контрольной суммы записи\r\n"
  "'р' - не обнаружен конец записи\r\n"
  "'т' - не поддерживаемый тип записи\r\n"
  "'е' - прочие ошибки\r\n"
  "<Й> - прервать загрузку\r\n";

/* Подпрограмма извлечения символа из НЕХ-файла и перекодирования его в
 * шестнадцатиричную цифру.
 *  (out) dest         - указатель на переменную, где будет сохранена цифра
 *  (in)  ptr_record   - указатель на структуру описания строки НЕХ-файла
 * Результат:
 *  HEX_LOAD_OK        - Успешное выполнение
 *  HEX_LOAD_ERR_ABORT - Ошибка. Выполнение программы прервано пользователем
 *  HEX_LOAD_ERR_SYMB  - Ошибка. Неформатная литера
 */
static error_t hex_parse_digit(
    uint8_t *dest, struct tag_hex_record *ptr_record )
{
    uint8_t         symb;
    error_t         err;

    /* Ошибка: преждевременный конец записи */
    if( ptr_record->idx_read >= ptr_record->idx_save ) {
        err = HEX_LOAD_ERR_SHORT;
    } else {
        /* Выборка символа из буфера */
        symb = ptr_record->buff[ ptr_record->idx_read ];
        ptr_record->idx_read++;
        if( ( symb >= 0x30U ) && ( symb <= 0x39U ) ) {   // Т.е если считанный код соответствует аски кодировке десятичных цифр 
            /* Десятичная литера (0-9)*/
            *dest = ( uint8_t )( symb - 0x30U );   // мы преобразуем десятичную цифру в шестнадцатеричную
            err = HEX_LOAD_OK;
        } else if( ( symb >= 0x41U ) && ( symb <= 0x46U ) ) {
            /* шестнадцатиричная литера (A-F)*/
            *dest = ( uint8_t )( ( symb - 0x41U ) + 0xAU );
            err = HEX_LOAD_OK;
        } else if ( ( ( uint8_t ) 0xC9U /* 0x1BU - ESC */ ) == symb ) {
            /* символ Й */
            err = HEX_LOAD_ERR_ABORT;
        } else {
            /* Символ - неформатная литера */
            err = HEX_LOAD_ERR_SYMB;
        }
    }

    return err;
}

/* Подпрограмма извлечения байта (2 символа) из НЕХ-файла и перекодирования
 * его в число
 *  (out) dest         - указатель на переменную, где будет сохранено число
 *  (in)  ptr_record   - указатель на структуру описания строки НЕХ-файла
 * Результат:
 *  HEX_LOAD_OK        - Успешное выполнение
 *  HEX_LOAD_ERR_ABORT - Ошибка. Выполнение программы прервано пользователем
 *  HEX_LOAD_ERR_SYMB  - Ошибка. Неформатная литера
 */
static error_t hex_parse_byte(
    uint8_t *dest, struct tag_hex_record *ptr_record )
{
    uint8_t         part_hi, part_lo, byte;
    error_t         err;

    /* Извлечение старшей тетрады. Вызываемая подпрограмма выбирает из буфера
     * приема символ по текущему индексу и перекодирует в цифру */
    err = hex_parse_digit( &part_hi, ptr_record );
    if ( HEX_LOAD_OK != err ) {
        ;
    } else {
        /* Извлечение младшей тетрады. Вызываемая подпрограмма выбирает из буфера
         * приема символ по текущему индексу и перекодирует в цифру */
        err = hex_parse_digit( &part_lo, ptr_record );
        if ( HEX_LOAD_OK != err ) {
            ;
        } else {
            /* byte = ( part_hi << 4 ) | part_lo ) */
            part_hi <<= 4;
            byte  = part_hi;
            byte |= part_lo;
            /* Подсчет контрольной суммы */
            ptr_record->csum += byte;
            /* Формирование результата */
            if( dest != NULL ) {
                *dest = byte;
            }
            /* err = HEX_LOAD_OK */
        }
    }

    return err;
}

/* Подпрограмма извлечения слова (4 байта или 2 байта) из НЕХ-файла и
 * перекодирование его в число
 *  (out) dest         - указатель на переменную, где будет сохранено число
 *  (in)  ptr_record   - указатель на структуру описания строки НЕХ-файла
 *  (in)  flag_32      - признак: 1 - 32-разрядное слово, 0 - 16-разрядное
 * Результат:
 *  HEX_LOAD_OK        - Успешное выполнение
 *  HEX_LOAD_ERR_ABORT - Ошибка. Выполнение программы прервано пользователем
 *  HEX_LOAD_ERR_SYMB  - Ошибка. Неформатная литера
 */
static error_t hex_parse_word (
    uint32_t *dest, struct tag_hex_record *ptr_record, uint32_t flag_32 )
{
    uint32_t        i;
    uint32_t        numb;
    uint8_t         part;
    error_t         err;

    /* Обработка 4 байт, начиная со старшего */
    numb = 0U;
    for ( i = 0U; i < ( 2U + ( 2U * flag_32 ) ); i++ ) {
        /* Извлечение очередного байта */
        err = hex_parse_byte( &part, ptr_record );
        if ( HEX_LOAD_OK != err ) {
            /* Обнаружены ошибки */
            break;
        } else {
            /* err = HEX_LOAD_OK */
            numb = ( numb << 8 ) | part;
        }
    }
    /* Формирование результата */
    if (   ( HEX_LOAD_OK == err )
        && ( NULL != dest )
    ) {
        *dest = numb;
    }

    return err;
}

/* Подпрограмма извлечения блока байтов заданного размера, декодирования и
 * записи в ОЗУ по заданному адресу
 *  (in)  dest         - начальный адрес места записи в ОЗУ
 *  (in)  size         - размер блока байтов
 *  (in)  ptr_record   - указатель на структуру описания строки НЕХ-файла
 * Результат:
 *  HEX_LOAD_OK        - Успешное выполнение
 *  HEX_LOAD_ERR_ABORT - Ошибка. Выполнение программы прервано пользователем
 *  HEX_LOAD_ERR_SYMB  - Ошибка. Неформатная литера
 *  HEX_LOAD_ERR_STOR  - Ошибка. Байт неправильно записан в ОЗУ
 */
static error_t hex_parse_block(
    volatile uint8_t dest[], sz_t size, struct tag_hex_record *ptr_record )
{
    uint8_t         i;
    uint8_t         byte;
    error_t         err;

    err = NO_ERROR;
    for( i = 0U; i < size; i++ ) {
        /* Извлечение байта */
        err = hex_parse_byte( &byte, ptr_record );
        if( HEX_LOAD_OK != err ) {
            /* Обнаружены ошибки */
            ;
        } else {
            /* Cохранение по заданному адресу */
            dest[ i ] = byte;
            if( dest[ i ] == byte ) {
                /* err = HEX_LOAD_OK */
                ;
            } else {
                /* Ошибка сохранения */
                err = HEX_LOAD_ERR_STOR;
            }
        }
        if( HEX_LOAD_OK != err ) {
            /* Обнаружены ошибки */
            break;
        }
    }

    return err;
}

/* Значения состояния приема HEX файла */
#define LOAD_PROC               ( 0U ) /* Ошибок нет                          */
#define LOAD_SAVE               ( 3U ) /* Признак приема записи               */
#define LOAD_CONT               ( 1U ) /* Признак обнаружения ошибки          */
#define LOAD_END                ( 4U ) /* Признак Завершения приёма           */

/* Подпрограмма загрузки НЕХ-файла в ОЗУ для выполнения
 *  (in)  ptr_entry - указатель на значение адреса точки входа в программу
 * Результат:
 *  NO_ERROR        - Успешное выполнение
 *  ERROR           - Ошибка
 */
error_t hex_file_load(
    uint32_t *ptr_entry )
{
    hex_record_t    hex_record; /* запись HEX файла */
    sz_t            reclen;     /* Длина поля данных в записи (в байтах) */
    uint_t          typerec;    /* Тип записи */
    uint32_t        base_addr;  /* Линейный адрес */
    uint32_t        addr;       /* Текущий адрес для размещения данных */
    uint32_t        offset;     /* Смещение данных относительно линейного
                                   базового адреса */
    uint32_t        entry;      /* Линейный адрес старта программы */
    char_t          symb;       /* Код символа, считанного из канала RS-232 */
    uint_t          load_flag;  /* Флажок состояния приема */
    uint_t          line;       /* Счётчик символов в линии */
    uint8_t         temp8;
    error_t         err, lerr;  /* Код ошибки */

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
    
    /* Цикл загрузки НЕХ-файла (пока не встретится конечная запись типа 01
       или не будет получен символ 'Й') */
    while ( LOAD_END != load_flag ) {

        /* Ожидание начала очередной записи НEХ-файла (символ ':') */
        if ( LOAD_SAVE == load_flag ) {
            /* Ожидание не требуется. Символ ':' уже получен */
            lerr = HEX_LOAD_OK;
        } else {
            /* Ожидание */
            do {
                rs232_getc( &symb );
            } while (
                ( symb != ':' )                       /* 0x3A */
                && ( symb != ( ( char_t ) 0xC9U ) )   /* Й */
            );
            if ( ':' == symb ) {
                /* Получен символ начала новой записи */
                load_flag = LOAD_SAVE;
                lerr = HEX_LOAD_OK;
            } else {
                /* Получен символ ESC. Завершение приёма hex-файла */
                load_flag = LOAD_CONT;
                lerr = HEX_LOAD_ERR_ABORT;
            }
        }

        /* Цикл приема записи НЕХ-файла
           Концом записи считается получение символов конца строки */
        if ( LOAD_SAVE != load_flag ) {
            /* Пропуск операции */
            ;
        } else {
            hex_record.buff[ 0 ] = ( uint8_t ) ':';
            hex_record.idx_read = 1U;
            hex_record.idx_save = 1U;
            hex_record.csum = 0U;
            while ( LOAD_SAVE == load_flag ) {
                rs232_getc( &symb );
                if ( symb == ':' ) {                  /* 0x3AU */
                    /* Неожиданное получения символа новой записи */
                    load_flag = LOAD_SAVE;
                    lerr = HEX_LOAD_ERR_END;
                    break;
                } else if ( hex_record.idx_save >= MAX_REC_LENG ) {
                    /* Буфер приема записи переполнен */
                    load_flag = LOAD_CONT;
                    lerr = HEX_LOAD_ERR_BUFF;
                } else if (
                       ( symb == ( '\n' ) /* 0x0A */ )
                    || ( symb == ( '\r' ) /* 0x0D */ )
                ) {
                    /* Oбнаружен конец записи */
                    hex_record.buff[ hex_record.idx_save ] = 0x00U;   // зачем при получении символов конца строки записывать в буфер нулевой байт
                    load_flag = LOAD_PROC;
                    break;
                    /* lerr = HEX_LOAD_OK */
                } else {
                    /* Cохранение символа в буфере приема */
                    hex_record.buff[ hex_record.idx_save ] = ( uint8_t ) symb;
                    hex_record.idx_save++;
                }
            }
        }

        /* Обработка принятой записи. Поля Reclen, Offset, Typerec */
        if ( LOAD_PROC != load_flag ) {
            /* Пропуск операции */
            ;
        } else if ( hex_record.idx_save < ( 1U + 1U ) ) {
            /* Пустые записи допустимы, но нетипичны */
            load_flag = LOAD_CONT;
            lerr = HEX_LOAD_NULL;
        } else {
            /* Определение длины поля данных в записи */
            lerr = hex_parse_byte( &temp8, &hex_record );
            reclen = temp8;
            if ( HEX_LOAD_OK != lerr ) {
                /* Обнаружены ошибки */
                load_flag = LOAD_CONT;
            } else {
                /* Определение смещения для записи */
                lerr = hex_parse_word( &offset, &hex_record, 0U );
                if ( HEX_LOAD_OK != lerr ) {
                    /* Обнаружены ошибки */
                    load_flag = LOAD_CONT;
                } else {
                    /* Определение типа записи */
                    lerr = hex_parse_byte( &temp8, &hex_record );
                    typerec = temp8;
                    if ( HEX_LOAD_OK != lerr ) {
                        /* Обнаружены ошибки */
                        load_flag = LOAD_CONT;
                    } else {
                        /* lerr = HEX_LOAD_OK */
                        ;
                    }
                }
            }
        }

        /* Обработка принятой записи. Поля Address, Data
           (зависит от типа записи */
        if ( LOAD_PROC != load_flag ) {
            /* Пропуск операции */
            ;
        } else if (
            /* Data */
               ( 0U == typerec )
        ) {
            /* Запись с данными. 16-bit offset */
            if ( reclen < 1U ) {
                /* Недопустимый формат записи */
                load_flag = LOAD_CONT;
                lerr = HEX_LOAD_ERR_LENG;
            } else {
                /* Получение базового адреса записи */
                addr = base_addr | offset ;
                /* Считывание и сохранение данных */
                lerr = hex_parse_block( ( uint8_t * ) addr, reclen, &hex_record );
                if ( HEX_LOAD_OK != lerr ) {
                    /* Обнаружены ошибки */
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
            /* Адрес точки входа в программу */
            if ( ( reclen != 4U ) || ( offset != 0U ) ) {
                /* Недопустимый формат записи */
                load_flag = LOAD_CONT;
                lerr = HEX_LOAD_ERR_LENG;
            } else {
                /* Получение точки входа в загружаемую программу */
                lerr = hex_parse_word( &entry, &hex_record, 1U );
                if ( NO_ERROR != lerr ) {
                    /* Обнаружены ошибки */
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
            /* Прерывающая запись */
            if ( ( reclen != 0U ) || ( offset != 0U ) ) {
                /* Недопустимый формат записи */
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
            /* Начало секции. Линейный базовый адрес */
            if ( ( reclen != 2U ) || ( offset != 0U ) ) {
                /* Недопустимый формат записи */
                load_flag = LOAD_CONT;
                lerr = HEX_LOAD_ERR_LENG;
            } else {
                /* Получение базового адреса секции */
                lerr = hex_parse_word( &base_addr, &hex_record, 0U );
                if ( NO_ERROR != lerr ) {
                    /* Обнаружены ошибки */
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
            /* Формат не поддерживается подпрограммой */
            load_flag = LOAD_CONT;
            lerr = HEX_LOAD_ERR_TYPE;
        } else {
            /* недопустимый тип */
            load_flag = LOAD_CONT;
            lerr = HEX_LOAD_ERR_TYPE;
        }

        /* Проверка контрольной суммы записи */
        /* Чтение контрольной суммы (последний байт записи). В процессе
         * обработки принятой записи подсчитывается КС, к которой функция
         * hex_parse_byte(NULL) прибавляет значение КС записи (последний байт)
         */
        if ( LOAD_PROC != load_flag ) {
            /* Пропуск операции */
            ;
        } else {
            /* Считывание контрольной суммы строки.
             * КС является дополнением суммы байтов записи между ":" и КС до 0х00
             * Прибавление байта КС к подсчитанной КС записи дает 0
             * в младших 8 разрядах
             */
            lerr = hex_parse_byte( NULL, &hex_record );
            if ( HEX_LOAD_OK != lerr ) {
                /* Обнаружены ошибки */
                load_flag = LOAD_CONT;
            } else {
                /* Сверка контрольной суммы */
                if ( 0U != ( hex_record.csum & 0xFFU ) ) {
                    /* Ошибка контроля */
                    load_flag = LOAD_CONT;
                    lerr = HEX_LOAD_ERR_CSUM;
                } else {
                    /* lerr = HEX_LOAD_OK */
                    ;
                }
            }
        }

        /* Анализ результата обработки записи */
        if ( HEX_LOAD_OK == lerr ) {
            if ( 0U == typerec ) {
                /* Запись с данными успешно обработана */
                symb = '.';
            } else if ( 4U == typerec ) {
                /* Запись с базовым адресом успешно обработана */
                symb = '!';
            } else if ( 5U == typerec ) {
                /* Запись с адресом старта успешно обработана */
                symb = 'i';
            } else if ( 1U == typerec ) {
                /* Терминальная запись успешно обработана */
                symb = '#';
                load_flag = LOAD_END;
            }
        } else if ( HEX_LOAD_NULL == lerr ) {
            symb = 'з';
        } else if ( HEX_LOAD_ERR_LENG  == lerr ) {
            symb = 'л';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_SHORT == lerr ) {
            symb = 'н';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_SYMB  == lerr ) {
            symb = 'с';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_ABORT == lerr ) {
            symb = 'а';
            load_flag = LOAD_END;
            err = ERROR;
        } else if ( HEX_LOAD_ERR_BUFF  == lerr ) {
            symb = 'б';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_STOR  == lerr ) {
            symb = 'в';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_CSUM  == lerr ) {
            symb = 'к';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_END   == lerr ) {
            symb = 'р';
            err = ERROR;
        } else if ( HEX_LOAD_ERR_TYPE  == lerr ) {
            symb = 'т';
            err = ERROR;
        } else {
            /* Прочие ошибки */
            symb = 'е';
            err = ERROR;
        }

        /* Выдача в технологический канал прогресса обработки файла */
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

/* Подпрограмма инвертирования байт слова данных
 *  (in/out)  data - слово данных
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
