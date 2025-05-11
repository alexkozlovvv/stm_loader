#ifndef __MULTIPLEXER_H__
#define __MULTIPLEXER_H__ 1

#include "sections_bsp.h"

#include "base_types.h"

#ifdef __cplusplus
extern "C" {
#endif
//слова для переключения мультиплексора
#define MULTIPLEXER_STATE_1      0
#define MULTIPLEXER_STATE_2      1
#define MULTIPLEXER_STATE_3      2
#define MULTIPLEXER_STATE_4      3
#define MULTIPLEXER_STATE_5      4
#define MULTIPLEXER_STATE_6      5
#define MULTIPLEXER_STATE_7      6
#define MULTIPLEXER_STATE_8      7
#define MULTIPLEXER_STATE_9      8
#define MULTIPLEXER_STATE_10     9
#define MULTIPLEXER_STATE_11     10
#define MULTIPLEXER_STATE_12     11
#define MULTIPLEXER_STATE_13     12
#define MULTIPLEXER_STATE_14     13
#define MULTIPLEXER_STATE_15     14
#define MULTIPLEXER_STATE_16     15
#define MULTIPLEXER_STATE_17     16
#define MULTIPLEXER_STATE_18     17
#define MULTIPLEXER_STATE_19     18
#define MULTIPLEXER_STATE_20     19
#define MULTIPLEXER_STATE_21     20
#define MULTIPLEXER_STATE_22     21
#define MULTIPLEXER_STATE_23     22
#define MULTIPLEXER_STATE_24     23
#define MULTIPLEXER_STATE_25     24
#define MULTIPLEXER_STATE_26     25
#define MULTIPLEXER_STATE_27     26
#define MULTIPLEXER_STATE_28     27
#define MULTIPLEXER_STATE_29     28
#define MULTIPLEXER_STATE_30     29
#define MULTIPLEXER_STATE_31     30
#define MULTIPLEXER_STATE_32     31
	
	
 /*Инициализация модуля АЦП*/
 extern LOADER_FUNC error_t multiplexer_init(void);
	
 /*Подпрограмма чтения данных с ацп*/
 extern LOADER_FUNC error_t multiplexer_set(uint32_t pos);	

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
