 #include "sections_bsp.h"

 #include "base_types.h"
 #include "os_def.h"
 #include "STM32F407.h"
 
 #include "multiplexer.h"
 #include "multiplexer_mem.h"
 
 /* Признак работоспособности коммутатора */
 static LOADER_DATA uint_t multiplexer_can_work;
 
 /* Подпрограмма инициализации коммутаторов */
 error_t multiplexer_init(void)
 {
	 error_t         err;
	 uint32_t        temp;

   err = NO_ERROR;
	 
	 /* Вкл тактирование spi3 */
	 temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
	 temp |= MCU_RCC_APB1ENR_SPI3EN;
	 OUT_REG( STM32F407_RCC_PTR->apb1enr, temp ); 
	 
	 /* Настройка SPI3
	    сбросить все настройки */
	 OUT_REG(STM32F407_SPI3_PTR->cr1,0);
	
	 /* Режим Мастера */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |= MCU_SPI_CR1_MSTR;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* Настроить делитель частоты(На микросхеме до 30Мгц, на шине частота процессора/4( max 42Мгц))fPCLK/2(12,5) */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |=( uint32_t ) (0x20 | 0x10 | 0x8);   /* Коэффициент деления 256 для частоты 97,6 КГц - работает */
     //temp |= 0x20;        /* Коэффициент деления 32 для частоты 781,25 КГц - работает */
     //temp |= 0x08;        /* Коэффициент деления 4 (новая частота 6,25 МГц)- работает */
     //temp &= (~0x38);     /* Коэффициент деления 2 (новая частота 12,5 МГц) - не работает */
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* Настроить полярность сигнала(SPI Clock Polarity: Low) */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp &= ~( uint32_t )MCU_SPI_CR1_CPOL;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* Настроить фазу тактового сигнала(SPI Clock Phase: Cycle Half)
        Выдача данных со второго фронта первого тактового сигнала */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |= MCU_SPI_CR1_CPHA;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* Выбрать 8-битный формат данных */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp &= ~( uint32_t ) MCU_SPI_CR1_DFF;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* Выбрать порядок передачи данных (MSB первый)(SPI Data Order: MSB First) */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp &= ~( uint32_t )MCU_SPI_CR1_LSBFIRST;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /*
	 Если же SPI модуль работает в режиме мастера, то ногу NSS нужно подтянуть к питанию или включить программное управление (SSM=1) и установить бит SSI. 
	 В противном случае - SPI модуль подумает, что появился новый мастер и сам станет слейвом. 
	 */
	
	 /* Программное управление NSS */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |= MCU_SPI_CR1_SSM | MCU_SPI_CR1_SSI;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);	
	
	 /* Сбросить настройки */
	 OUT_REG(STM32F407_SPI3_PTR->cr2,0);
	 
     /* Выставление высокого уровня сигнала NSS */
	 OUT_REG( STM32F407_GPIOA_PTR->bsrr, MCU_GPIO_BSRR_SET_15 );

	 /* Включить SPI3 */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |= MCU_SPI_CR1_SPE;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	 
	 multiplexer_can_work = 1;
	 return err;
 }
 
 /* Подпрограмма переключения коммутаторов */
 error_t multiplexer_set(uint32_t pos)
 {
     uint32_t        temp;
	 error_t         err;
	 
	 err = NO_ERROR;
	 
	  /* Проверка инициализации модуля */
	  if(multiplexer_can_work != 1 ) {
          err = INVALID_MODE;
	  }
		
	  if(err == NO_ERROR){
          
		  /* Ждать пока буфер передатчика не опустошится */
	      while (!(MCU_SPI_SR_TXE & IN_REG(STM32F407_SPI3_PTR->sr)));
	
          /* Выдать слово */
          OUT_REG(STM32F407_SPI3_PTR->dr,pos);
          
          /* Устанавливаем slave select */
          OUT_REG( STM32F407_GPIOA_PTR->bsrr, MCU_GPIO_BSRR_RES_15 );
          
          /* Ждать 400нс */  
          __asm__( " NOP" );
          __asm__( " NOP" );
          __asm__( " NOP" );
          __asm__( " NOP" );
          __asm__( " NOP" );
          __asm__( " NOP" );

          while ((MCU_SPI_SR_BSY & IN_REG(STM32F407_SPI3_PTR->sr)));

          for(temp = 0;temp<40;++temp) {
             __asm__( " NOP" );//~30нан за круг
          }       
           
          /* Снимаем сигнал slave select */          
          OUT_REG( STM32F407_GPIOA_PTR->bsrr, MCU_GPIO_BSRR_SET_15 );	 
	  }
	 
      __asm__( " NOP" );
	  __asm__( " NOP" );
	  __asm__( " NOP" );
	  __asm__( " NOP" );
	  __asm__( " NOP" );
	  __asm__( " NOP" );
      
     return err;
 }
