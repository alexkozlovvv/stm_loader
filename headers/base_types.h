#ifndef __BASE_TYPES_H__
#define __BASE_TYPES_H__ 1

#ifdef __cplusplus
extern "C" {
#endif

/* Пустой указатель */
#ifndef NULL
#define NULL 0L
#endif


/* Целочисленные типы */
/* Пересечение с библиотечными типами */
typedef unsigned char       uint8_t;    /* Тип данных - без знаковый  8 бит   */
#define MAX_UINT8   ( 0xFFU )           /* Максимальное положительное
                                           значение типа  uint8_t             */

typedef   signed char        int8_t;    /* Тип данных - знаковый     8 бит    */
#define MAX_INT8    ( 0x7F  )           /* Максимальное положительное
                                           значение типа   int8_t             */

typedef unsigned short     uint16_t;    /* Тип данных - без знаковый 16 бит   */
#define MAX_UINT16  ( 0xFFFFU )         /* Максимальное положительное
                                           значение типа uint16_t             */

typedef   signed short      int16_t;    /* Тип данных - знаковый    16 бит    */
#define MAX_INT16   ( 0x7FFF  )         /* Максимальное положительное
                                           значение типа  int16_t             */

typedef unsigned int       uint32_t;    /* Тип данных - без знаковый 32 бит   */
#define MAX_UINT32  ( 0xFFFFFFFFUL )    /* Максимальное положительное
                                           значение типа uint32_t             */

typedef   signed int        int32_t;    /* Тип данных - знаковый    32 бит    */
#define MAX_INT32   ( 0x7FFFFFFFL  )    /* Максимальное положительное
                                           значение типа  int32_t             */

typedef unsigned long long uint64_t;    /* Тип данных - без знаковый 64 бит   */
#define MAX_UINT64  ( 0xFFFFFFFFFFFFFFFFULL )
                                        /* Максимальное положительное
                                           значение типа uint64_t             */

typedef   signed long long  int64_t;    /* Тип данных - знаковый    64 бит    */
#define MAX_INT64   ( 0x7FFFFFFFFFFFFFFFLL  )
                                        /* Максимальное положительное
                                           значение типа int64_t              */

/* Тип, соответствующий типу int компилятора */
typedef unsigned int         uint_t;    /* Тип данных - без знаковое целое    */
typedef   signed int          int_t;    /* Тип данных - знаковое целое        */
typedef char                 char_t;    /* Тип данных - символ                */
typedef double                f64_t;    /* Тип данных - вещественное двойной
                                                        точности              */
typedef float                 f32_t;    /* Тип данных - вещественное одинарной
                                                        точности              */

/* Тип данных - код ошибки */
typedef int_t       error_t;

/* Тип данных - размер ( байт ) */
typedef uint32_t    sz_t;

/* Тип данных - ИД объекта базового ПО */
typedef uint32_t    id_t;

/* Тип данных - системное время ( мкс ) */
typedef int64_t     sys_time_t;

#ifdef __cplusplus
}
#endif

#endif /* __BASE_TYPES_H__ */
