 #include "sections_bsp.h"

 #include "base_types.h"
 #include "os_def.h"
 #include "STM32F407.h"
 #include "irq.h"
 #include "timer.h"
 #include "adc.h"
 #include "adc_mem.h"
 #include "multiplexer.h"
 
  /*������� ����������������� ���*/ 
 static LOADER_DATA uint_t adc_can_work;
 
 /*������������� ������ ���*/
 error_t adc_init(void)
 {
	 error_t         err;
	 uint32_t        temp;
	 uint32_t        msr_old;

     err = NO_ERROR;
	 
	 /* �������� ������������ SPI2 */
	 temp  = IN_REG( STM32F407_RCC_PTR->apb1enr );
	 temp |= MCU_RCC_APB1ENR_SPI2EN;
	 OUT_REG( STM32F407_RCC_PTR->apb1enr, temp );
	 
	 /* ��������� ����������� ������� ���������� EXTI */
     /* �������� ������������ ������ SYSCFG */         
	 temp  = IN_REG( STM32F407_RCC_PTR->apb2enr );
	 temp |= MCU_RCC_APB2ENR_SYSCFGEN;
	 OUT_REG( STM32F407_RCC_PTR->apb2enr, temp );
	 
	 /* �������� SYSCFG(������� ����������) */
	 /* ��������� ���������� */
	 msr_old = kern_irq_disable_interrupt();
	
	 /* ��������� ����� EXTI8 (�������� � �������� ��������� �������� ���������� ��� PB8) */
	 temp  = IN_REG( STM32F407_SYSCFG_PTR->exticr3 );
	 temp &= ~( uint32_t )MCU_SYSCFG_EXTICR3_EXTI8_MASK;
	 temp |= BIT_0;
	 OUT_REG( STM32F407_SYSCFG_PTR->exticr3, temp );
	
	 /* ��������� ���������� �� ����� EXTI8 (PB8) */
	 temp  = IN_REG( STM32F407_EXTI_PTR->imr );
	 temp |= BIT_8;
	 OUT_REG( STM32F407_EXTI_PTR->imr, temp );
	
	 /* ��������� ���������� �� �������� ����� */
	 temp  = IN_REG( STM32F407_EXTI_PTR->ftsr );
	 temp |= BIT_8;
	 OUT_REG( STM32F407_EXTI_PTR->ftsr, temp );
	
	 /* ��������� ����� EXTI9 (PB9) (�������� � �������� ��������� �������� ���������� ��� PB9) */
	 temp  = IN_REG( STM32F407_SYSCFG_PTR->exticr3 );
	 temp &= ~( uint32_t )MCU_SYSCFG_EXTICR3_EXTI9_MASK;
	 temp |= BIT_4;
	 OUT_REG( STM32F407_SYSCFG_PTR->exticr3, temp );
	
	 /* ��������� ���������� �� ����� EXTI9 (PB9) */
	 temp  = IN_REG( STM32F407_EXTI_PTR->imr );
	 temp |= BIT_9;
	 OUT_REG( STM32F407_EXTI_PTR->imr, temp );
	
	 /* ��������� ���������� �� �������� ����� */
	 temp  = IN_REG( STM32F407_EXTI_PTR->ftsr );
	 temp |= BIT_9;
	 OUT_REG( STM32F407_EXTI_PTR->ftsr, temp );
	
	 /* �������� ���������� */
	 kern_irq_set_msr( msr_old );
	 
     /* ��������� SPI2(�� ��������� ������, ������� ������) */
	 /* �������� ��� ��������� */
	 OUT_REG(STM32F407_SPI2_PTR->cr1, 0U);
	
	 /* ����� ������ ������� */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_MSTR;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* ��������� �������� ������� (��������� ������� ��������� �������)
        ������� ������ ����� ����� 25/32 = 781,25 ��� */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= 0x20;  /* ����������� ������� 32 ��� ������� 781,25 ��� - �������� */ 
     //temp |= 0x08;        /* ����������� ������� 4 (������� 6,25 ���) - �������� */
     //temp &= (~0x38);     /* ����������� ������� 2 (������� 12,5 ���) - �������� */
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* ��������� ���������� ������� (� ������ �������� �������� ������� �������) */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_CPOL;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* ��������� ���� ��������� ������� 
        (������ ������ ���������� �� ������� ������ ������� ����� �������������) */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp &= ~( uint32_t )MCU_SPI_CR1_CPHA;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* ������� 16-������ ������ ������ ������ */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_DFF;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* ������� ������� �������� ������ (MSB ������) */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp &= ~( uint32_t )MCU_SPI_CR1_LSBFIRST;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);
	
	 /* ������ ���������� NSS ( � ������� ��) */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_SSM | MCU_SPI_CR1_SSI;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);	
	
	 /* �������� ��������� */
	 OUT_REG(STM32F407_SPI2_PTR->cr2,0);
	
	 /* �������� SPI2 */
	 temp  = IN_REG( STM32F407_SPI2_PTR->cr1 );
	 temp |= MCU_SPI_CR1_SPE;
	 OUT_REG(STM32F407_SPI2_PTR->cr1,temp);	 
	 
	 adc_can_work = 1;
	 return err;
 }
 
error_t wait_eocs ( void )
 {
     
     volatile int32_t		eoc1, eoc2;       /* �������� ��������� �������������� ��� */
     uint32_t           temp, wait_eoc_timeout_flag;
     error_t            err;
     
     eoc1=0;
     eoc2=0;
     err = NO_ERROR;
     
    /* �������� ��������� ������� ���������� EXTI */
    temp  = IN_REG( STM32F407_EXTI_PTR->pr);
    temp |= BIT_8| BIT_9;
    OUT_REG(STM32F407_EXTI_PTR->pr,temp);

    /* ��������� ������� �������� EOCs */
    timer_wait_eoc_init(TIME_STEP_1_S, &wait_eoc_timeout_flag);
    
    /* �������� ������� "����� ��������������" ��� D61(PB8) � D64(PB9)(1->0 60��) */
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
 
/*������������ ������ ������ � ���
 *  (in/out)  *adc1     - ��������� �� ����������, ��������������� ��� 
 *                        �������� ����������� ��������� ��� (D61)
 *  (in/out)  *adc2     - ��������� �� ����������, ��������������� ���
 *                        �������� ����������� ��������� ��� (D64)
 */
 error_t adc_read(volatile uint32_t *adc1,volatile uint32_t *adc2)
{
	uint32_t            temp;
	error_t             err; 
	volatile int32_t		eoc1=0,eoc2=0;//������� ��������� �������������� ���
	err = NO_ERROR;
	
	/*�������� ������������� ������*/
	if(adc_can_work != 1 ) {
		err = INVALID_MODE;
	}
        
    if(err == NO_ERROR){
        //4.������ ������� (ADC_CLK)() � (RFS_ADC_3) � ������� ������ � ��� (D61) 
        //��������� cs
        OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_RES_12 );
        //������ mosi(��� ������)
        //����� ���� ����� ����������� �� �����������
        while (!(MCU_SPI_SR_TXE & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        //������ �����(�����)
        OUT_REG(STM32F407_SPI2_PTR->dr,0x1);
        //����� ��������� ��������
        while ((MCU_SPI_SR_BSY & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        //������ �����-������
        //�����, ���� ������� ��������� �� ����� ��������
        while (!(MCU_SPI_SR_RXNE & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        for(temp = 0;temp<3;++temp) {
             __asm__( " NOP" );//~30��� �� ����
        }
        
        //5.����� ������ (RFS_ADC_3)(�������� �������-������)
        OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_SET_12 );
        //������� �����
        *adc1 = IN_REG(STM32F407_SPI2_PTR->dr);
        

        //6.������ ������ (RFS_ADC_4) � ��������� ������ � ��� (D64)
        //��������� cs
        OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_RES_11 );
        //������ mosi(��� ������)
        //����� ���� ����� ����������� �� �����������
        while (!(MCU_SPI_SR_TXE & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        //������ �����(�����)
        OUT_REG(STM32F407_SPI2_PTR->dr,0x1);
        //����� ��������� ��������
        while ((MCU_SPI_SR_BSY & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        //������ �����-������
        //�����, ���� ������� ��������� �� ����� ��������
        while (!(MCU_SPI_SR_RXNE & IN_REG(STM32F407_SPI2_PTR->sr))){
            ;
        }
        for(temp = 0;temp<3;++temp){
             __asm__( " NOP" );//~30��� �� ����
        }

        //������� �����
        *adc2 = IN_REG(STM32F407_SPI2_PTR->dr);
        //7.����� ������	(RFS_ADC_4)(�������� �������-������)

        OUT_REG( STM32F407_GPIOB_PTR->bsrr, MCU_GPIO_BSRR_SET_11 );
        //8.����� ������ (ADC_CLK)	��������� �������������
    } else {
        ;
    }  
   
	return err;
   //��� ������� �������
   //20�/4096 = 0,0048828125
   //���� �������(11) ��� 1, �� �������� �������(15,14,13,12) ���� �� 1 � uint16_t � �������� ����� �� 0,00488...

}
