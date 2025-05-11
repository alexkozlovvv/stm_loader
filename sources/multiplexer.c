 #include "sections_bsp.h"

 #include "base_types.h"
 #include "os_def.h"
 #include "STM32F407.h"
 
 #include "multiplexer.h"
 #include "multiplexer_mem.h"
 
 /* ������� ����������������� ����������� */
 static LOADER_DATA uint_t multiplexer_can_work;
 
 /* ������������ ������������� ������������ */
 error_t multiplexer_init(void)
 {
	 error_t         err;
	 uint32_t        temp;

   err = NO_ERROR;
	 
	 /* ��� ������������ spi3 */
	 temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
	 temp |= MCU_RCC_APB1ENR_SPI3EN;
	 OUT_REG( STM32F407_RCC_PTR->apb1enr, temp ); 
	 
	 /* ��������� SPI3
	    �������� ��� ��������� */
	 OUT_REG(STM32F407_SPI3_PTR->cr1,0);
	
	 /* ����� ������� */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |= MCU_SPI_CR1_MSTR;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* ��������� �������� �������(�� ���������� �� 30���, �� ���� ������� ����������/4( max 42���))fPCLK/2(12,5) */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |=( uint32_t ) (0x20 | 0x10 | 0x8);   /* ����������� ������� 256 ��� ������� 97,6 ��� - �������� */
     //temp |= 0x20;        /* ����������� ������� 32 ��� ������� 781,25 ��� - �������� */
     //temp |= 0x08;        /* ����������� ������� 4 (����� ������� 6,25 ���)- �������� */
     //temp &= (~0x38);     /* ����������� ������� 2 (����� ������� 12,5 ���) - �� �������� */
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* ��������� ���������� �������(SPI Clock Polarity: Low) */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp &= ~( uint32_t )MCU_SPI_CR1_CPOL;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* ��������� ���� ��������� �������(SPI Clock Phase: Cycle Half)
        ������ ������ �� ������� ������ ������� ��������� ������� */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |= MCU_SPI_CR1_CPHA;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* ������� 8-������ ������ ������ */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp &= ~( uint32_t ) MCU_SPI_CR1_DFF;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /* ������� ������� �������� ������ (MSB ������)(SPI Data Order: MSB First) */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp &= ~( uint32_t )MCU_SPI_CR1_LSBFIRST;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	
	 /*
	 ���� �� SPI ������ �������� � ������ �������, �� ���� NSS ����� ��������� � ������� ��� �������� ����������� ���������� (SSM=1) � ���������� ��� SSI. 
	 � ��������� ������ - SPI ������ ��������, ��� �������� ����� ������ � ��� ������ �������. 
	 */
	
	 /* ����������� ���������� NSS */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |= MCU_SPI_CR1_SSM | MCU_SPI_CR1_SSI;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);	
	
	 /* �������� ��������� */
	 OUT_REG(STM32F407_SPI3_PTR->cr2,0);
	 
     /* ����������� �������� ������ ������� NSS */
	 OUT_REG( STM32F407_GPIOA_PTR->bsrr, MCU_GPIO_BSRR_SET_15 );

	 /* �������� SPI3 */
	 temp  = IN_REG( STM32F407_SPI3_PTR->cr1 );
	 temp |= MCU_SPI_CR1_SPE;
	 OUT_REG(STM32F407_SPI3_PTR->cr1,temp);
	 
	 multiplexer_can_work = 1;
	 return err;
 }
 
 /* ������������ ������������ ������������ */
 error_t multiplexer_set(uint32_t pos)
 {
     uint32_t        temp;
	 error_t         err;
	 
	 err = NO_ERROR;
	 
	  /* �������� ������������� ������ */
	  if(multiplexer_can_work != 1 ) {
          err = INVALID_MODE;
	  }
		
	  if(err == NO_ERROR){
          
		  /* ����� ���� ����� ����������� �� ����������� */
	      while (!(MCU_SPI_SR_TXE & IN_REG(STM32F407_SPI3_PTR->sr)));
	
          /* ������ ����� */
          OUT_REG(STM32F407_SPI3_PTR->dr,pos);
          
          /* ������������� slave select */
          OUT_REG( STM32F407_GPIOA_PTR->bsrr, MCU_GPIO_BSRR_RES_15 );
          
          /* ����� 400�� */  
          __asm__( " NOP" );
          __asm__( " NOP" );
          __asm__( " NOP" );
          __asm__( " NOP" );
          __asm__( " NOP" );
          __asm__( " NOP" );

          while ((MCU_SPI_SR_BSY & IN_REG(STM32F407_SPI3_PTR->sr)));

          for(temp = 0;temp<40;++temp) {
             __asm__( " NOP" );//~30��� �� ����
          }       
           
          /* ������� ������ slave select */          
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
