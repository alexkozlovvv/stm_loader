#ifndef __ADC_H__
#define __ADC_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif

 /*Инициализация модуля АЦП*/
extern LOADER_FUNC error_t adc_init(void);
	
 /*Подпрограмма чтения данных с ацп*/
extern LOADER_FUNC error_t adc_read(volatile uint32_t *adc1,volatile uint32_t *adc2);
    
extern LOADER_FUNC error_t wait_eocs ( void ); 	

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
