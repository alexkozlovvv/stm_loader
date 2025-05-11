#ifndef __SECTION_H__
#define __SECTION_H__ 1

#ifdef VECTOR_CAST_SECTIONS

#define BSP_FUNC
#define BSP_DATA

#else

/* Заголовок описания подпрограммы базового ПО */
#define BSP_FUNC
/* Заголовок описания данных или констант базового ПО */
#define BSP_DATA

#endif

#endif /* __SECTION_H__ */
