#ifndef __ADC_MEM_H__
#define __ADC_MEM_H__ 1

#include "sections_bsp.h"
#include "base_types.h"
#include "eumw.h"


/* Инициализация модуля АЦП */
extern LOADER_FUNC error_t adc_init(void);
	
/* Подпрограмма чтения данных с ацп */
extern LOADER_FUNC error_t adc_read(volatile uint32_t *adc1,volatile uint32_t *adc2);	

#endif /* __ADC_MEM_H__ */


