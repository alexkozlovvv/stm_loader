#ifndef __IRQ_LEVELS_H__
#define __IRQ_LEVELS_H__ 1

#include "base_types.h"

/*
    ����������� ����������� ����������
    ���������� �������� ��������� �������:
    1) ����������� ����� ����������;
    2) ���� �� ������ �� ������ �������������� ������� �� ����������� ��������� � �������� 
		� ������ ��������� � �������� ��������� � ������� ���������� ����� ������, 
		�� �������������c� ������� ������� ����������.
    �� ����� ������� - ��� ���������� �� ������, ���� ���������� ����������,
������� ������ �� ���������� �� ������, �� ��� �� ���� � ����.
    3) ������� ������� ���������� ��������� ��� ����� � ���������� ���
��� ������ � ������� ICPR.
    ������ ��� �� �������� ������ ������� ������� ����������. ���� ������
�� ������ ��� ���, ��� � ������� � �������� ���������, �� ������� �������
����������� �����. �� �� � ������ Handle. ������ � ������ Thread. � ������
Handle ������� ����� ��������������� ������ ��� ��������� ������� �� �����������
��������� � ��������.
    4) ������ ���������� ��������������� ������ ��� ����������� ����������
�������� ��� ��� ������ �� ����������.
    �� ����� ������� - ��������� ������ ������������ ������� ���������� �� ������
��� ��������� ���������� ��� ������ ������ ����������.

    ����������� ������� ������ "���������� �� ������ � �� ������" � "���������
������� ������ ����������".

    �����. ������ ����� ��������� ��� ������� ���������� �������������� ������
������� ���������� ��� ���������� ������ �� ����/����� �� ����������� ����������.
��������� ����� ���� ����������� ��� ���.

    ����������� ������� �����������
    1. ��� ���� �������, ��� ���� ���������
    2. ���������� ������ ���������� 0 - F. 
*/

/* ��������������� ���������� ���� (����� ������������) */
#define IRQ_RESET_LEVEL         ( -3 )
#define IRQ_NMI_LEVEL           ( -2 )
#define IRQ_HARDFAULT           ( -1 )

/* ������������� ���������� ���� */
#define IRQ_SV_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 3 << 6 ) << 24 ) /* ( 3 << 6 ) = ������� 192 */
#define IRQ_SYST_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ( 2 << 6 ) = ������� 128 */
#define IRQ_PENDSV_LEVEL        ( ( uint32_t ) ( ( uint32_t ) 3 << 6 ) << 16 ) /* ( 3 << 6 ) = ������� 192 */

/* ������� ���������� ���� */
#define IRQ_WWDG_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 0 ] */
#define IRQ_PVD_LEVEL                ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 0 ] */
#define IRQ_TAMP_STAMP_LEVEL         ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 0 ] */
#define IRQ_RTC_WKUP_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 0 ] */

#define IRQ_FLASH_LEVEL              ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 1 ] */
#define IRQ_RCC_LEVEL                ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 1 ] */
#define IRQ_EXTI0_LEVEL              ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 1 ] */
#define IRQ_EXTI1_LEVEL              ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 1 ] */

#define IRQ_EXTI2_LEVEL              ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 2 ] */
#define IRQ_EXTI3_LEVEL              ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 2 ] */
#define IRQ_EXTI4_LEVEL              ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 2 ] */
#define IRQ_DMA1_STR0_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 2 ] */

#define IRQ_DMA1_STR1_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 3 ] */
#define IRQ_DMA1_STR2_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 3 ] */
#define IRQ_DMA1_STR3_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 3 ] */
#define IRQ_DMA1_STR4_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 3 ] */

#define IRQ_DMA1_STR5_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 4 ] */
#define IRQ_DMA1_STR6_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 4 ] */
#define IRQ_ADC_LEVEL                ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 4 ] */
#define IRQ_CAN1_TX_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 4 ] */

#define IRQ_CAN1_RX0_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 5 ] */
#define IRQ_CAN1_RX1_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 5 ] */
#define IRQ_CAN1_SCE_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 5 ] */
#define IRQ_EXTI9_5_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 5 ] */

#define IRQ_TIM1_BRK_TIM9_LEVEL      ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 6 ] */
#define IRQ_TIM1_UP_TIM10_LEVEL      ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 6 ] */
#define IRQ_TIM1_TRG_COM_TIM11_LEVEL ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 6 ] */
#define IRQ_TIM1_CC_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 6 ] */

#define IRQ_TIM2_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 7 ] */
#define IRQ_TIM3_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 7 ] */
#define IRQ_TIM4_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 7 ] */
#define IRQ_I2C1_EV_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 7 ] */

#define IRQ_I2C1_ER_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 8 ] */
#define IRQ_I2C2_EV_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 8 ] */
#define IRQ_I2C2_ER_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 8 ] */
#define IRQ_SPI1_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 8 ] */

#define IRQ_SPI2_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 9 ] */
#define IRQ_USART1_LEVEL             ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 9 ] */
#define IRQ_USART2_LEVEL             ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 9 ] */
#define IRQ_USART3_LEVEL             ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 9 ] */

#define IRQ_EXTI15_10_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 10 ] */
#define IRQ_RTC_ALARM_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 10 ] */
#define IRQ_OTG_FS_WKUP_LEVEL        ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 10 ] */
#define IRQ_TIM8_BRK_TIM12_LEVEL     ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 10 ] */

#define IRQ_TIM8_UP_TIM13_LEVEL      ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 11 ] */
#define IRQ_TIM8_TRG_COM_TIM14_LEVEL ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 11 ] */
#define IRQ_TIM8_CC_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 11 ] */
#define IRQ_DMA1_STR7_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 11 ] */

#define IRQ_FSMC_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 12 ] */
#define IRQ_SDIO_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 12 ] */
#define IRQ_TIM5_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 12 ] */
#define IRQ_SPI3_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 12 ] */

#define IRQ_UART4_LEVEL              ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 13 ] */
#define IRQ_UART5_LEVEL              ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 13 ] */
#define IRQ_TIM6_DAC_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 13 ] */
#define IRQ_TIM7_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 13 ] */

#define IRQ_DMA2_STR0_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 14 ] */
#define IRQ_DMA2_STR1_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 14 ] */
#define IRQ_DMA2_STR2_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 14 ] */
#define IRQ_DMA2_STR3_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 14 ] */

#define IRQ_DMA2_STR4_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 15 ] */
#define IRQ_ETH_LEVEL                ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 15 ] */
#define IRQ_ETH_WKUP_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 15 ] */
#define IRQ_CAN2_TX_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 15 ] */

#define IRQ_CAN2_RX0_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 16 ] */
#define IRQ_CAN2_RX1_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 16 ] */
#define IRQ_CAN2_SCE_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 16 ] */
#define IRQ_OTG_FS_LEVEL             ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 16 ] */

#define IRQ_DMA2_STR5_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 17 ] */
#define IRQ_DMA2_STR6_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 17 ] */
#define IRQ_DMA2_STR7_LEVEL          ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 17 ] */
#define IRQ_USART6_LEVEL             ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 17 ] */

#define IRQ_I2C3_EV_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 18 ] */
#define IRQ_I2C3_ER_LEVEL            ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 18 ] */
#define IRQ_OTG_HS_EP1_OUT_LEVEL     ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 18 ] */
#define IRQ_OTG_HS_EP1_IN_LEVEL      ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 18 ] */

#define IRQ_OTG_HS_WKUP_LEVEL        ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 19 ] */
#define IRQ_OTG_HS_LEVEL             ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 19 ] */
#define IRQ_DCMI_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 16 ) /* ipr[ 19 ] */
#define IRQ_CRYP_LEVEL               ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) << 24 ) /* ipr[ 19 ] */

#define IRQ_HASH_RNG_LEVEL           ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  0 ) /* ipr[ 20 ] */
#define IRQ_FPU_LEVEL                ( ( uint32_t ) ( ( uint32_t ) 2 << 6 ) <<  8 ) /* ipr[ 20 ] */

#endif /* __IRQ_LEVELS_H__ */
