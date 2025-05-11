#ifndef __EUMW_H__
#define __EUMW_H__ 1

#include "base_types.h"


#ifdef __cplusplus  // ���� ����� �������� ���������� ����� ��++
extern "C" {        // ����� �������� � ���������� ������� ���� ���������� � ������� �������� ����� ��
#endif

/* �������, ������������ � ��������������� 
    ������� ����������� ���������������� RC-���������� - 16���;
    ������� ����������� ��������������� ���������� - 32���
    */

#define MCU_FREQ_HSE                    ( 8000000U )                  /* ��������� */       //������� �������� ���������� ����������

/* ������������� ������������� ��� �������������� ������ */
#define MCU_FREQ_VCO                    ( MCU_FREQ_HSE * ( 100U / 4U ) ) /* [MCU_FREQ_HSI,MCU_FREQ_HSE]/[0..63] */
#define MCU_FREQ_PLLCLK                 ( MCU_FREQ_VCO / 2U )            /* MCU_FREQ_VCO / [2..8] �� ����� 168 ��� */
#define MCU_FREQ_AHB                    ( MCU_FREQ_PLLCLK / 1U )         /* MCU_FREQ_HSI,MCU_FREQ_HSE,MCU_FREQ_PLLCLK / [1..512] �� ����� 168 ��� */      
#define MCU_FREQ_APB1                   ( MCU_FREQ_AHB / 4U )            /* MCU_FREQ_AHB / [1..16] �� ����� 42 ��� */
#define MCU_FREQ_APB1_TIM               ( MCU_FREQ_AHB / 2U )

/*****************************************************************************
 * �������������� ������������ � ��������� ����� ������������.
 * ������ ������������� �� ��������� ��� ��������� ��������� �� ����������!!!
 ****************************************************************************/

/* ��������� �������� ��������� ����� ����������� */
/* ��� �������� �� */
extern uint8_t Image$$lvl01_stack$$ZI$$Limit;
#define LCF_SP_SSP      ( uint8_t * )( &Image$$lvl01_stack$$ZI$$Limit ) // 
/* ��� ��������������� ���������� */
#define LCF_SP_LOADER   LCF_SP_SSP
/* ��� ������� Vector CAST */
extern uint8_t Image$$lvl01_stack$$ZI$$Base;
#define LCF_SP_VCAST    ( uint8_t * )( &Image$$lvl01_stack$$ZI$$Base )

/* ----- ��������� ���������� �������� ������ ----- */

/* ��������������, ��� ����������� KEIL ��������� �������� � ���������
������� (KEIL Using Linker 4.1 ��. 4.11):
a. read-only code
b. read-only data
c. read-write code
d. read-write data
e. zero-initialized data.
 */

/* ������� ����� � ������ ������� SRAM */
extern uint8_t Image$$lvl1_dmabuf$$Base;
#define LCF_SRAM            ( uint8_t * )( &Image$$lvl1_dmabuf$$Base )
#define LCF_SRAM_SIZE       ( 0x00020000U )
#define SRAM_ADDRESS        ( ( uint32_t ) LCF_SRAM )
#define SRAM_SIZE           ( ( uint32_t ) LCF_SRAM_SIZE )

/* ������� ����� � ������ ������� ��� */
extern uint8_t Image$$interrupt_vectors$$Base;
#define LCF_ROM             ( uint8_t * )( &Image$$interrupt_vectors$$Base )
#define LCF_ROM_SIZE        ( 0x00100000U )
#define ROM_ADDRESS         ( ( uint32_t ) LCF_ROM )
#define ROM_SIZE            ( ( uint32_t ) LCF_ROM_SIZE )

/* ������� ����� � ������ ������� RAM (��� ������) */
extern uint8_t Image$$vector_table_base$$Base;
#define LCF_RAM             ( uint8_t * )( &Image$$vector_table_base$$Base )
#define LCF_RAM_SIZE        ( 0x00010000U )
#define RAM_ADDRESS         ( ( uint32_t ) LCF_RAM )
#define RAM_SIZE            ( ( uint32_t ) LCF_RAM_SIZE )

/* ----- ��������� ������� ���������� ----- */

/* ������� ����� */
extern uint8_t Image$$interrupt_vectors$$Base;
#define LCF_VECTOR_START    ( uint8_t * )( &Image$$interrupt_vectors$$Base )
#define VECTOR_START        ( ( uint32_t ) LCF_VECTOR_START )
#define VECTOR_SIZE         ( 0x00001000U )

/* ----- ��������� �������� ������ ��������������� ���������� ----- */

/* ������� ����� � ������ ������� ���� � �������� */
extern uint8_t Image$$loader_text$$Base;
#define LCF_LOADER_TEXT_START ( uint8_t * )( &Image$$loader_text$$Base )
#define LCF_LOADER_TEXT_SIZE  ( 0x0000B000U )
#define LOADER_TEXT_START     ( ( uint32_t ) LCF_LOADER_TEXT_START )
#define LOADER_TEXT_SIZE      ( ( uint32_t ) LCF_LOADER_TEXT_SIZE )
/* ������� ����� � ������ ������� ������ */
extern uint8_t Image$$loader_data$$Base;
#define LCF_LOADER_DATA_START ( uint8_t * )( &Image$$loader_data$$Base )
#define LCF_LOADER_DATA_SIZE  ( 0x00001000U )
#define LOADER_DATA_START     ( ( uint32_t ) LCF_LOADER_DATA_START )
#define LOADER_DATA_SIZE      ( ( uint32_t ) LCF_LOADER_DATA_SIZE )
/* ������� ����� � ������ ������� ����� */
extern uint8_t Image$$loader_stack$$ZI$$Base;
#define LCF_LOADER_STACK_START ( uint8_t * )( &Image$$loader_stack$$ZI$$Base )
#define LCF_LOADER_STACK_SIZE  ( 0x00001000U )
#define LOADER_STACK_START     ( ( uint32_t ) LCF_LOADER_STACK_START )
#define LOADER_STACK_SIZE      ( ( uint32_t ) LCF_LOADER_STACK_SIZE )
/* ������� ����� � ������ ������� �������� ������ */
extern uint8_t Image$$loader_out$$Base;
#define LCF_LOADER_OUT_START ( uint8_t * )( &Image$$loader_out$$Base )
#define LCF_LOADER_OUT_SIZE  ( 6U * 4U )
#define LOADER_OUT_START     ( ( uint32_t ) LCF_LOADER_OUT_START )
#define LOADER_OUT_SIZE      ( ( uint32_t ) LCF_LOADER_OUT_SIZE )

/* ----- ��������� �������� ������ �������� �� ----- */

/* �� ������� 0. ���� */
/* ������� ����� � ������ ������� ���� � �������� �������� �� */
extern uint8_t Image$$lvl0_text$$Base;
extern uint8_t Image$$lvl0_text$$Length;
#define LCF_LVL0_TEXT_START ( uint8_t * )( &Image$$lvl0_text$$Base )
#define LCF_LVL0_TEXT_SIZE  ( uint8_t * )( &Image$$lvl0_text$$Length )
#define LVL0_TEXT_START     ( ( uint32_t ) LCF_LVL0_TEXT_START )
#define LVL0_TEXT_SIZE      ( ( uint32_t ) LCF_LVL0_TEXT_SIZE )
/* ������� ����� � ������ ������� ������ �������� �� */
extern uint8_t Image$$lvl0_data$$Base;
extern uint8_t Image$$lvl0_data$$Length;
extern uint8_t Image$$lvl0_data$$ZI$$Length;
#define LCF_LVL0_DATA_START ( uint8_t * )( &Image$$lvl0_data$$Base )
#define LCF_LVL0_DATA_SIZE  ( uint8_t * )( &Image$$lvl0_data$$Length )
#define LCF_LVL0_BSS_SIZE   ( uint8_t * )( &Image$$lvl0_data$$ZI$$Length )
#define LVL0_DATA_START     ( ( uint32_t ) LCF_LVL0_DATA_START )
#define LVL0_DATA_SIZE      ( ( uint32_t ) LCF_LVL0_DATA_SIZE + ( uint32_t ) LCF_LVL0_BSS_SIZE )
/* ������� ����� � ������ ������� ����� �������� �� */
extern uint8_t Image$$lvl01_stack$$ZI$$Base;
extern uint8_t Image$$lvl01_stack$$ZI$$Length;
#define LCF_LVL01_STACK_START ( uint8_t * )( &Image$$lvl01_stack$$ZI$$Base )
#define LCF_LVL01_STACK_SIZE  ( uint8_t * )( &Image$$lvl01_stack$$ZI$$Length )
#define LVL0_STACK_START      ( ( uint32_t ) LCF_LVL01_STACK_START )
#define LVL0_STACK_SIZE       ( ( uint32_t ) LCF_LVL01_STACK_SIZE )

/* �� ������� 1. �������� � ������� */
/* ������� ����� � ������ ������� ���� � �������� �������� �� */
extern uint8_t Image$$lvl1_text$$Base;
extern uint8_t Image$$lvl1_text$$Length;
#define LCF_LVL1_TEXT_START ( uint8_t * )( &Image$$lvl1_text$$Base )
#define LCF_LVL1_TEXT_SIZE  ( uint8_t * )( &Image$$lvl1_text$$Length )
#define LVL1_TEXT_START     ( ( uint32_t ) LCF_LVL1_TEXT_START )
#define LVL1_TEXT_SIZE      ( ( uint32_t ) LCF_LVL1_TEXT_SIZE )
/* ������� ����� � ������ ������� ������ �������� ��.
   ������ ������������� � ��������� ������. */
extern uint8_t Image$$lvl1_data$$Base;
extern uint8_t Image$$lvl1_data$$Length;
extern uint8_t Image$$lvl1_data$$ZI$$Length;
extern uint8_t Image$$lvl1_dmabuf$$ZI$$Base;
extern uint8_t Image$$lvl1_dmabuf$$ZI$$Length;
#define LCF_LVL1_DATA_START ( uint8_t * )( &Image$$lvl1_data$$Base )
#define LCF_LVL1_DATA_SIZE  ( uint8_t * )( &Image$$lvl1_data$$Length )
#define LCF_LVL1_BSS_SIZE   ( uint8_t * )( &Image$$lvl1_data$$ZI$$Length )
#define LCF_LVL1_DMABUF_START ( uint8_t * )( &Image$$lvl1_dmabuf$$ZI$$Base )
#define LCF_LVL1_DMABUF_SIZE  ( uint8_t * )( &Image$$lvl1_dmabuf$$ZI$$Length )
#define LVL1_DMABUF_START     ( ( uint32_t ) LCF_LVL1_DMABUF_START )
#define LVL1_DMABUF_SIZE      ( ( uint32_t ) LCF_LVL1_DMABUF_SIZE )
#define LVL1_DATA_START       ( ( uint32_t ) LCF_LVL1_DATA_START )
#define LVL1_DATA_SIZE        ( ( uint32_t ) LCF_LVL1_DATA_SIZE + ( uint32_t ) LCF_LVL1_BSS_SIZE )
/* ������� ����� � ������ ������� ����� �������� �� */
#define LVL1_STACK_START    LVL0_STACK_START
#define LVL1_STACK_SIZE     LVL0_STACK_SIZE

/* �� ������� 2. ��������� �������������� � ���������� �� */
/* ������� ����� � ������ ������� ���� � �������� */
extern uint8_t Image$$lvl2_text$$Base;
extern uint8_t Image$$lvl2_text$$Length;
#define LCF_LVL2_TEXT_START ( uint8_t * )( &Image$$lvl2_text$$Base )
#define LCF_LVL2_TEXT_SIZE  ( uint8_t * )( &Image$$lvl2_text$$Length )
#define LVL2_TEXT_START     ( ( uint32_t ) LCF_LVL2_TEXT_START )
#define LVL2_TEXT_SIZE      ( ( uint32_t ) LCF_LVL2_TEXT_SIZE )

/* �� ������� 3. ������� �������� �� */
/* �������  ����� � ������ ������� ���� � �������� �������� �� */
extern uint8_t Image$$process_bsp_text$$Base;
extern uint8_t Image$$process_bsp_text$$Length;
#define LCF_PROCESS_BSP_TEXT_START ( uint8_t * )( &Image$$process_bsp_text$$Base )
#define LCF_PROCESS_BSP_TEXT_SIZE  ( uint8_t * )( &Image$$process_bsp_text$$Length )
#define PROCESS_BSP_TEXT_START     ( ( uint32_t ) LCF_PROCESS_BSP_TEXT_START )
#define PROCESS_BSP_TEXT_SIZE      ( ( uint32_t ) LCF_PROCESS_BSP_TEXT_SIZE )
/* ������� ����� � ������ ������� ������ �������� �� */
extern uint8_t Image$$process_bsp_data$$Base;
extern uint8_t Image$$process_bsp_data$$Length;
extern uint8_t Image$$process_bsp_data$$ZI$$Length;
#define LCF_PROCESS_BSP_DATA_START ( uint8_t * )( &Image$$process_bsp_data$$Base )
#define LCF_PROCESS_BSP_DATA_SIZE  ( uint8_t * )( &Image$$process_bsp_data$$Length )
#define LCF_PROCESS_BSP_BSS_SIZE   ( uint8_t * )( &Image$$process_bsp_data$$ZI$$Length )
#define PROCESS_BSP_DATA_START     ( ( uint32_t ) LCF_PROCESS_BSP_DATA_START )
#define PROCESS_BSP_DATA_SIZE      ( ( uint32_t ) LCF_PROCESS_BSP_DATA_SIZE + ( uint32_t ) LCF_PROCESS_BSP_BSS_SIZE )

/* �� ������� 3. ������� ����������� �� */
/* ������� ����� � ������ ������� ���� � �������� ����������� �� */
extern uint8_t Image$$process1_text$$Base;
extern uint8_t Image$$process1_text$$Length;
#define LCF_PROCESS1_TEXT_START ( uint8_t * )( &Image$$process1_text$$Base )
#define LCF_PROCESS1_TEXT_SIZE  ( uint8_t * )( &Image$$process1_text$$Length )
#define PROCESS1_TEXT_START     ( ( uint32_t ) LCF_PROCESS1_TEXT_START )
#define PROCESS1_TEXT_SIZE      ( ( uint32_t ) LCF_PROCESS1_TEXT_SIZE )
/* ������� ����� � ������ ������� ������ ����������� �� */
extern uint8_t Image$$process1_data$$Base;
extern uint8_t Image$$process1_data$$Length;
extern uint8_t Image$$process1_data$$ZI$$Length;
#define LCF_PROCESS1_DATA_START ( uint8_t * )( &Image$$process1_data$$Base )
#define LCF_PROCESS1_DATA_SIZE  ( uint8_t * )( &Image$$process1_data$$Length )
#define LCF_PROCESS1_BSS_SIZE   ( uint8_t * )( &Image$$process1_data$$ZI$$Length )
#define PROCESS1_DATA_START     ( ( uint32_t ) LCF_PROCESS1_DATA_START )
#define PROCESS1_DATA_SIZE      ( ( uint32_t ) LCF_PROCESS1_DATA_SIZE + ( uint32_t ) LCF_PROCESS1_BSS_SIZE )

/* �� ������� 3. ������� ������ ������� ��������� �������� � ����������� �� */
/* ������� ����� � ������ ������� ������ �������� � ����������� �� */
extern uint8_t Image$$thread_stack$$ZI$$Base;
extern uint8_t Image$$lvl01_stack$$ZI$$Base;
#define LCF_THREAD_STACK_START ( uint8_t * )( &Image$$thread_stack$$ZI$$Base )
#define LCF_THREAD_STACK_END   ( uint8_t * )( &Image$$lvl01_stack$$ZI$$Base )
#define THREAD_STACK_START     ( ( uint32_t ) LCF_THREAD_STACK_START )
#define THREAD_STACK_END       ( ( uint32_t ) LCF_THREAD_STACK_END )
#define THREAD_STACK_SIZE      ( THREAD_STACK_END - THREAD_STACK_START )

/* ��� ������ - �������� ���������� ������ ��������������� ���������� */
typedef struct tag_loader_out {
    uint32_t        first[ 4 ];         /* 0-3 ������� ������� ���������      */
    uint32_t        cnt_reset;          /* 4 ������� ������������             */
    uint32_t        test_result;        /* 5 ���������� ���������� ��������   */
    uint32_t        image_header_addr;  /* 6 ����� ��������� ������
                                             ������������� ��������� ��       */
    uint32_t        gap[ 64U - 7U ];    /* 7 ������ */
} loader_out_t;

extern uint8_t Image$$loader_out$$Base;
#define MPM_LOADER_OUT_PTR      ( ( loader_out_t * ) &Image$$loader_out$$Base )

/* ���� ������ TEST_RESULT */
#define TEST_RESULT_ERR_SRAM    ( 0x01U ) /* ������ �������� SRAM  */
#define TEST_RESULT_ERR_CCM     ( 0x02U ) /* ������ �������� CCM     */
#define TEST_RESULT_ERR_SROM    ( 0x04U ) /* ������ �������� ���. \
                                             �������������� ���������         */
#define TEST_RESULT_ERR_UROM    ( 0x08U ) /* ������ �������� ���. \
                                             �� ������������                  */
#define TEST_RESULT_ERR_CPU     ( 0x10U ) /* ������ �������� ���              */
#define TEST_RESULT_ERR_WDT     ( 0x20U ) /* ������ �������� WDT              */
#define TEST_RESULT_SKIP_WDT    ( 0x40U ) /* �������� WDT �� �����������      */
#define TEST_RESULT_ERR_ADC     ( 0x80U ) /* ������ �������� ���              */
#define TEST_RESULT_ERR_OTHER   ( 0x100U )/* ���������� ������ � ������ \
                                             ��������������� ����������       */

#ifdef __cplusplus
}
#endif

#endif /* __EUMW_H__ */
