#include "sections_bsp.h"

#include "base_types.h"
#include "eumw.h"

#include "irq.h"
#include "irq_mem.h"

/* Тип данных - элемент таблицы векторов прерываний */
typedef void ( *irq_table_entry_t )( void );

/* Таблица векторов прерываний
 * ColdFire содержит 97 векторов + начальный указатель стека __initial_sp.
 * В таблице 98 векторов ( входов ).
 * Вектор 1 - точка входа в программу.
 */
LOADER_CONST const irq_table_entry_t irq_table[ 98U ] = {
        /* Exceptions */
        &kern_irq_exception_a,          /*  0. Top of Stack main              */
        &kern_irq_exception_a,          /*  1. Reset Handler                  */
        &kern_irq_exception_a,          /*  2. NMI Handler                    */
        &kern_irq_exception_a,          /*  3. Hard Fault Handler             */
        &kern_irq_exception_a,          /*  4. Mem Manage Handler             */
        &kern_irq_exception_a,          /*  5. Bus Fault Handler              */
        &kern_irq_exception_a,          /*  6. Usage Fault Handler            */
        &kern_irq_exception_a,          /*  7. Reserved                       */
        &kern_irq_exception_a,          /*  8. Reserved                       */
        &kern_irq_exception_a,          /*  9. Reserved                       */
        &kern_irq_exception_a,          /* 10. Reserved                       */
        &kern_irq_exception_a,          /* 11. SVCall Handler                 */
        &kern_irq_exception_a,          /* 12. Debug Monitor Handler          */
        &kern_irq_exception_a,          /* 13. Reserved                       */
        &kern_irq_exception_a,          /* 14. PendSV Handler                 */
        &kern_irq_exception_a,          /* 15. SysTick Handler                */
        /* External Interrupts */
        &kern_irq_exception_a,          /* 16. Window WatchDog Handler        */
        &kern_irq_exception_a,          /* 17. PVD through EXTI Line detection Handler */
        &kern_irq_exception_a,          /* 18. Tamper and TimeStamps through the EXTI line Handler */
        &kern_irq_exception_a,          /* 19. RTC Wakeup through the EXTI line Handler */
        &kern_irq_exception_a,          /* 20. FLASH Handler                  */
        &kern_irq_exception_a,          /* 21. RCC Handler                    */
        &kern_irq_exception_a,          /* 22. EXTI Line0 Handler             */
        &kern_irq_exception_a,          /* 23. EXTI Line1 Handler             */
        &kern_irq_exception_a,          /* 24. EXTI Line2 Handler             */
        &kern_irq_exception_a,          /* 25. EXTI Line3 Handler             */
        &kern_irq_exception_a,          /* 26. EXTI Line4 Handler             */
        &kern_irq_exception_a,          /* 27. DMA1 Stream 0 Handler          */
        &kern_irq_exception_a,          /* 28. DMA1 Stream 1 Handler          */
        &kern_irq_exception_a,          /* 29. DMA1 Stream 2 Handler          */
        &kern_irq_exception_a,          /* 30. DMA1 Stream 3 Handler          */
        &kern_irq_exception_a,          /* 31. DMA1 Stream 4 Handler          */
        &kern_irq_exception_a,          /* 32. DMA1 Stream 5 Handler          */
        &kern_irq_exception_a,          /* 33. DMA1 Stream 6 Handler          */
        &kern_irq_exception_a,          /* 34. ADC1, ADC2 and ADC3s Handler   */
        &kern_irq_exception_a,          /* 35. CAN1 TX Handler                */
        &kern_irq_exception_a,          /* 36. CAN1 RX0 Handler               */
        &kern_irq_exception_a,          /* 37. CAN1 RX1 Handler               */
        &kern_irq_exception_a,          /* 38. CAN1 SCE Handler               */
        &kern_irq_exception_a,          /* 39. External Line[9:5]s Handler    */
        &kern_irq_exception_a,          /* 40. TIM1 Break and TIM9 Handler    */
        &kern_irq_exception_a,          /* 41. TIM1 Update and TIM10 Handler  */
        &kern_irq_exception_a,          /* 42. TIM1 Trigger and Commutation and TIM11 Handler */
        &kern_irq_exception_a,          /* 43. TIM1 Capture Compare Handler   */
        &kern_irq_exception_a,          /* 44. TIM2 Handler                   */
        &kern_irq_exception_a,          /* 45. TIM3 Handler                   */
        &kern_irq_exception_a,          /* 46. TIM4 Handler                   */
        &kern_irq_exception_a,          /* 47. I2C1 Event Handler             */
        &kern_irq_exception_a,          /* 48. I2C1 Error Handler             */
        &kern_irq_exception_a,          /* 49. I2C2 Event Handler             */
        &kern_irq_exception_a,          /* 50. I2C2 Error Handler             */
        &kern_irq_exception_a,          /* 51. SPI1 Handler                   */
        &kern_irq_exception_a,          /* 52. SPI2 Handler                   */
        &kern_irq_exception_a,          /* 53. USART1 Handler                 */
        &kern_irq_exception_a,          /* 54. USART2 Handler                 */
        &kern_irq_exception_a,          /* 55. USART3 Handler                 */
        &kern_irq_exception_a,          /* 56. External Line[15:10]s Handler  */
        &kern_irq_exception_a,          /* 57. RTC Alarm (A and B) through EXTI Line Handler */
        &kern_irq_exception_a,          /* 58. USB OTG FS Wakeup through EXTI line Handler */
        &kern_irq_exception_a,          /* 59. TIM8 Break and TIM12 Handler   */
        &kern_irq_exception_a,          /* 60. TIM8 Update and TIM13 Handler  */
        &kern_irq_exception_a,          /* 61. TIM8 Trigger and Commutation and TIM14 Handler */
        &kern_irq_exception_a,          /* 62. TIM8 Capture Compare Handler   */
        &kern_irq_exception_a,          /* 63. DMA1 Stream7 Handler           */
        &kern_irq_exception_a,          /* 64. FSMC Handler                   */
        &kern_irq_exception_a,          /* 65. SDIO Handler                   */
        &kern_irq_exception_a,          /* 66. TIM5 Handler                   */
        &kern_irq_exception_a,          /* 67. SPI3 Handler                   */
        &kern_irq_exception_a,          /* 68. UART4 Handler                  */
        &kern_irq_exception_a,          /* 69. UART5 Handler                  */
        &kern_irq_exception_a,          /* 70. TIM6 and DAC1&2 underrun errors Handler */
        &kern_irq_exception_a,          /* 71. TIM7 Handler                   */
        &kern_irq_exception_a,          /* 72. DMA2 Stream 0 Handler          */
        &kern_irq_exception_a,          /* 73. DMA2 Stream 1 Handler          */
        &kern_irq_exception_a,          /* 74. DMA2 Stream 2 Handler          */
        &kern_irq_exception_a,          /* 75. DMA2 Stream 3 Handler          */
        &kern_irq_exception_a,          /* 76. DMA2 Stream 4 Handler          */
        &kern_irq_exception_a,          /* 77. Ethernet Handler               */
        &kern_irq_exception_a,          /* 78. Ethernet Wakeup through EXTI line Handler */
        &kern_irq_exception_a,          /* 79. CAN2 TX Handler                */
        &kern_irq_exception_a,          /* 80. CAN2 RX0 Handler               */
        &kern_irq_exception_a,          /* 81. CAN2 RX1 Handler               */
        &kern_irq_exception_a,          /* 82. CAN2 SCE Handler               */
        &kern_irq_exception_a,          /* 83. USB OTG FS Handler             */
        &kern_irq_exception_a,          /* 84. DMA2 Stream 5 Handler          */
        &kern_irq_exception_a,          /* 85. DMA2 Stream 6 Handler          */
        &kern_irq_exception_a,          /* 86. DMA2 Stream 7 Handler          */
        &kern_irq_exception_a,          /* 87. USART6 Handler                 */
        &kern_irq_exception_a,          /* 88. I2C3 event Handler             */
        &kern_irq_exception_a,          /* 89. I2C3 error Handler             */
        &kern_irq_exception_a,          /* 90. USB OTG HS End Point 1 Out Handler */
        &kern_irq_exception_a,          /* 91. USB OTG HS End Point 1 In Handler */
        &kern_irq_exception_a,          /* 92. USB OTG HS Wakeup through EXTI Handler */
        &kern_irq_exception_a,          /* 93. USB OTG HS Handler             */
        &kern_irq_exception_a,          /* 94. DCMI Handler                   */
        &kern_irq_exception_a,          /* 95. CRYP crypto Handler            */
        &kern_irq_exception_a,          /* 96. Hash and Rng Handler           */
        &kern_irq_exception_a           /* 97. FPU Handler                    */
};
