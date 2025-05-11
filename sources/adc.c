 #include "sections_bsp.h"

 #include "base_types.h"
 #include "os_def.h"
 #include "STM32F407.h"
 #include "irq.h"
 #include "timer.h"
 #include "adc.h"
 #include "adc_mem.h"
 #include "multiplexer.h"
 
  /*Признак работоспособности АЦП*/ 
 static LOADER_DATA uint_t adc_can_work;
 
 /*Инициализация модуля АЦП*/
 error_t adc_init(void)
 {
	 error_t         err;
	 uint32_t        temp;
	 uint32_t        msr_old;

     err = NO_ERROR;
	 
	 /* Включить тактирование SPI2 */
	 temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
	 temp |= MCU_RCC_APB1ENR_SPI2EN;
	 OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );
	 
	 /* Настройка контроллера внешних прерываний EXTI */
     /* Включить тактирование модуля SYSCFG */         
	 temp  = IN_REG( STM32F407_RCC_PTR->apb2enr );
	 temp |= MCU_RCC_APB2ENR_SYSCFGEN;
	 OUT_REG( STM32F407_RCC_PTR->apb2enr, temp );
	 
	 /* Настойка SYSCFG(внешние прерывания) */
	 /* Выключить прерывания */
	 msr_old = kern_irq_disable_interrupt();
	
	 /* Настроить линию EXTI8 (Выбираем в качестве источника внешнего прерывания пин PB8) */
	 temp  = IN_REG( STM32F407_SYSCFG_PTR->exticr3 );
	 temp &= ~( uint32_t )MCU_SYSCFG_EXTICR3_EXTI8_MASK;
	 temp |= BIT_0;
	 OUT_REG( STM32F407_SYSCFG_PTR->exticr3, temp );
	
	 /* Разрешить прерывание по линии EXTI8 (PB8) */
	 temp  = IN_REG( STM32F407_EXTI_PTR->imr );
	 temp |= BIT_8;
	 OUT_REG( STM32F407_EXTI_PTR->imr, temp );
	
	 /* Настроить прерывания на падающий фронт */
	 temp  = IN_REG( STM32F407_EXTI_PTR->ftsr );
	 temp |= BIT_8;
	 OUT_REG( STM32F407_EXTI_PTR->ftsr, temp );
	
	 /* Настроить линию EXTI9 (PB9) (Выбираем в качестве источника внешнего прерывания пин PB9) */
	 temp  = IN_REG( STM32F407_SYSCFG_PTR->exticr3 );
	 temp &= ~( uint32_t )MCU_SYSCFG_EXTICR3_EXTI9_MASK;
	 temp |= BIT_4;
	 OUT_REG( STM32F407_SYSCFG_PTR->exticr3, temp );
	
	 /* Разрешить прерывание по линии EXTI9 (PB9) */
	 temp  = IN_REG( STM32F407_EXTI_PTR->imr );
	 temp |= BIT_9;
	 OUT_REG( STM32F407_EXTI_PTR->imr, temp );
	
	 /* Настроить прерывания на падающий фронт */
	 temp  = IN_REG( STM32F407_EXTI_PTR->ftsr );
	 temp |= BIT_9;
	 OUT_REG( STM32F407_EXTI_PTR->ftsr, temp );
	
	 /* Включить прерывания */
	 kern_irq_set_msr( msr_old );
	 
     /* Настройка SPI2(по переднему фронту, старший вперед) */
	 /* сбросить все настройки */
	 OUT_REG(STM32F407_SPI2_PTR->cr1, 0U);
	
	 /* Выбор режима Мастера */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_MSTR;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* Настроить делитель частоты (настройка частоты тактового сигнала)
        Частота тактов будет равно 25/32 = 781,25 КГц */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= 0x20;  /* Коэффициент деления 32 для частоты 781,25 КГц - работает */ 
     //temp |= 0x08;        /* Коэффициент деления 4 (частота 6,25 МГц) - работает */
     //temp &= (~0x38);     /* Коэффициент деления 2 (частота 12,5 МГц) - работает */
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* Настроить полярность сигнала (в режиме ожидания держится высокий уровень) */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_CPOL;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* Настроить фазу тактового сигнала 
        (выдача данных происходит по первому фронту первого такта синхронизации) */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp &= ~( uint32_t )MCU_SPI_CR1_CPHA;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* Выбрать 16-битный формат данных фрейма */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_DFF;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* Выбрать порядок передачи данных (MSB первый) */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp &= ~( uint32_t )MCU_SPI_CR1_LSBFIRST;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* Ручное управление NSS ( с помощью ПО) */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_SSM | MCU_SPI_CR1_SSI;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);	
	
	 /* Сбросить настройки */
	 OUT_REG(STM32F407_SPI2_PTR->cr2,0);
	
	 /* Включить SPI2 */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_SPE;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);	 
	 
	 adc_can_work = 1;
	 return err;
 }
 
error_t wait_eocs ( void )
 {
     
     volatile int32_t		eoc1, eoc2;       /* признаки окончания преобразования ацп */
     uint32_t           temp, wait_eoc_timeout_flag;
     error_t            err;
     
     eoc1=0;
     eoc2=0;
     err = NO_ERROR;
     
    /* Очистить статусный регистр прерываний EXTI */
    temp  = IN_REG( STM32F407_EXTI_PTR->pr);
    temp |= BIT_8| BIT_9;
    OUT_REG(STM32F407_EXTI_PTR->pr,temp);

    /* Включение таймера ожидания EOCs */
    timer_wait_eoc_init(TIME_STEP_1_S, &wait_eoc_timeout_flag);
    
    /* Ожидание сигнала "конец преобразования" Для D61(PB8) и D64(PB9)(1->0 60нс) */
    do
    {
        eoc1 |= ((BIT_8 & IN_REG(STM32F407_EXTI_PTR->pr)));
        eoc2 |= ((BIT_9 & IN_REG(STM32F407_EXTI_PTR->pr)));
        if(1U == wait_eoc_timeout_flag) {
            err = ERROR;
            wait_eoc_timeout_flag = 0U;
        }
    } while(((eoc1==0) || (eoc2==0)) && (NO_ERROR == err));
    
    return err;
     
 }
 
/*Подпрограмма чтения данных с ацп
 *  (in/out)  *adc1     - указатель на переменную, предназначенную для 
 *                        хранения считываемых показаний АЦП (D61)
 *  (in/out)  *adc2     - указатель на переменную, предназначенную для
 *                        хранения считываемых показаний АЦП (D64)
 */
 error_t adc_read(volatile uint32_t *adc1,volatile uint32_t *adc2)
{
	uint32_t            temp;
	error_t             err; 
	volatile int32_t		eoc1=0,eoc2=0;//признак окончания преобразования ацп
	err = NO_ERROR;
	
	/*Проверка инициализации модуля*/
	if(adc_can_work != 1 ) {
		err = INVALID_MODE;
	}
        
    if(err == NO_ERROR){
        //4.Выдать сигналы (ADC_CLK)() и (RFS_ADC_3) и считать данные с АЦП (D61) 
        //выставить cs
        OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_RES_12 );
        //выдать mosi(для ответа)
        //ждать пока буфер передатчика не опустошится
        while (!(MCU_SPI_SR_TXE & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        //выдать слово(любое)
        OUT_REG(STM32F407_SPI2_PTR->dr,0x1);
        //ждать окончание передачи
        while ((MCU_SPI_SR_BSY & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        //чтение слова-ответа
        //ждать, пока регистр приемника не будет заполнен
        while (!(MCU_SPI_SR_RXNE & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        for(temp = 0;temp<3;++temp) {
             __asm__( " NOP" );//~30нан за круг
        }
        
        //5.Снять сигнал (RFS_ADC_3)(Активный уровень-низкий)
        OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_SET_12 );
        //считать слово
        *adc1 = IN_REG(STM32F407_SPI2_PTR->dr);
        

        //6.Выдать сигнал (RFS_ADC_4) и считывает данные с АЦП (D64)
        //выставить cs
        OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_RES_11 );
        //выдать mosi(для ответа)
        //ждать пока буфер передатчика не опустошится
        while (!(MCU_SPI_SR_TXE & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        //выдать слово(любое)
        OUT_REG(STM32F407_SPI2_PTR->dr,0x1);
        //ждать окончание передачи
        while ((MCU_SPI_SR_BSY & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        //чтение слова-ответа
        //ждать, пока регистр приемника не будет заполнен
        while (!(MCU_SPI_SR_RXNE & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        for(temp = 0;temp<3;++temp){
             __asm__( " NOP" );//~30нан за круг
        }

        //считать слово
        *adc2 = IN_REG(STM32F407_SPI2_PTR->dr);
        //7.Снять сигнал	(RFS_ADC_4)(Активный уровень-низкий)

        OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_SET_11 );
        //8.Снять сигнал (ADC_CLK)	снимается автоматически
    } else {
        ;
    }  
   
	return err;
   //как считать вольтаж
   //20В/4096 = 0,0048828125
   //если старший(11) бит 1, то дописать старшие(15,14,13,12) биты до 1 в uint16_t и умножить число на 0,00488...

}
