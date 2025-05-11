#ifndef __BASE_TYPES_H__
#define __BASE_TYPES_H__ 1

#ifdef __cplusplus
extern "C" {
#endif

/* ������ ��������� */
#ifndef NULL
#define NULL 0L
#endif


/* ������������� ���� */
/* ����������� � ������������� ������ */
typedef unsigned char       uint8_t;    /* ��� ������ - ��� ��������  8 ���   */
#define MAX_UINT8   ( 0xFFU )           /* ������������ �������������
                                           �������� ����  uint8_t             */

typedef   signed char        int8_t;    /* ��� ������ - ��������     8 ���    */
#define MAX_INT8    ( 0x7F  )           /* ������������ �������������
                                           �������� ����   int8_t             */

typedef unsigned short     uint16_t;    /* ��� ������ - ��� �������� 16 ���   */
#define MAX_UINT16  ( 0xFFFFU )         /* ������������ �������������
                                           �������� ���� uint16_t             */

typedef   signed short      int16_t;    /* ��� ������ - ��������    16 ���    */
#define MAX_INT16   ( 0x7FFF  )         /* ������������ �������������
                                           �������� ����  int16_t             */

typedef unsigned int       uint32_t;    /* ��� ������ - ��� �������� 32 ���   */
#define MAX_UINT32  ( 0xFFFFFFFFUL )    /* ������������ �������������
                                           �������� ���� uint32_t             */

typedef   signed int        int32_t;    /* ��� ������ - ��������    32 ���    */
#define MAX_INT32   ( 0x7FFFFFFFL  )    /* ������������ �������������
                                           �������� ����  int32_t             */

typedef unsigned long long uint64_t;    /* ��� ������ - ��� �������� 64 ���   */
#define MAX_UINT64  ( 0xFFFFFFFFFFFFFFFFULL )
                                        /* ������������ �������������
                                           �������� ���� uint64_t             */

typedef   signed long long  int64_t;    /* ��� ������ - ��������    64 ���    */
#define MAX_INT64   ( 0x7FFFFFFFFFFFFFFFLL  )
                                        /* ������������ �������������
                                           �������� ���� int64_t              */

/* ���, ��������������� ���� int ����������� */
typedef unsigned int         uint_t;    /* ��� ������ - ��� �������� �����    */
typedef   signed int          int_t;    /* ��� ������ - �������� �����        */
typedef char                 char_t;    /* ��� ������ - ������                */
typedef double                f64_t;    /* ��� ������ - ������������ �������
                                                        ��������              */
typedef float                 f32_t;    /* ��� ������ - ������������ ���������
                                                        ��������              */

/* ��� ������ - ��� ������ */
typedef int_t       error_t;

/* ��� ������ - ������ ( ���� ) */
typedef uint32_t    sz_t;

/* ��� ������ - �� ������� �������� �� */
typedef uint32_t    id_t;

/* ��� ������ - ��������� ����� ( ��� ) */
typedef int64_t     sys_time_t;

#ifdef __cplusplus
}
#endif

#endif /* __BASE_TYPES_H__ */
