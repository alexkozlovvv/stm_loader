; Процессор не имеет MMU и аппаратной защиты данных
; Куча (HEAP) общая для всех потоков
; Зона стека (STACK) описывает зону не одного, а всех потоков.

Stack_Size  EQU 0x00001000
        AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem   SPACE   Stack_Size
; __initial_sp неявным образом используется при инициализации стандартных
; библиотек компилятора. Не во всех случаях его допустимо экспортировать
__initial_sp

Heap_Size   EQU 0x00001000

    AREA        HEAP, NOINIT, READWRITE, ALIGN=2
__heap_base
Heap_Mem        SPACE Heap_Size
__heap_limit

    IMPORT      |Image$$vector_table_base$$Base|
    IMPORT      init_startup
    EXPORT      irq_table_entry
    EXPORT      irq_reset_entry	

    PRESERVE8
    THUMB

; Vector Table Mapped to Address 0 at Reset

    AREA        |.irq_vector_table|, CODE, READONLY, ALIGN=2
irq_table_entry

    ; Exeptions
    DCD         __initial_sp            ; Top of Stack
    DCD         irq_reset_entry         ; Reset Handler
    DCD         irq_nmi_entry           ; NMI Handler
    DCD         irq_hardfault_entry     ; Hard Fault Handler
    DCD         irq_memmanage_entry     ; Mem Manage Handler
    DCD         irq_busfault_entry      ; Bus Fault Handler
    DCD         irq_usagefault_entry    ; Usage Fault Handler
    DCD         irq_exeption7_entry     ; Reserved
    DCD         irq_exeption8_entry     ; Reserved
    DCD         irq_exeption9_entry     ; Reserved
    DCD         irq_exeption10_entry    ; Reserved
    DCD         irq_svc_entry           ; SVCall Handler
    DCD         irq_debugmonitor_entry  ; Debug Monitor Handler
    DCD         irq_exeption13_entry    ; Reserved
    DCD         irq_pendsv_entry        ; PendSV Handler
    DCD         irq_systick_entry       ; SysTick Handler

    ; External Interrupts
    DCD         irq_irq0_entry          ; Window WatchDog                                        
    DCD         irq_irq1_entry          ; PVD through EXTI Line detection                        
    DCD         irq_irq2_entry          ; Tamper and TimeStamps through the EXTI line            
    DCD         irq_irq3_entry          ; RTC Wakeup through the EXTI line                       
    DCD         irq_irq4_entry          ; FLASH                                           
    DCD         irq_irq5_entry          ; RCC                                             
    DCD         irq_irq6_entry          ; EXTI Line0                                             
    DCD         irq_irq7_entry          ; EXTI Line1                                             
    DCD         irq_irq8_entry          ; EXTI Line2                                             
    DCD         irq_irq9_entry          ; EXTI Line3                                             
    DCD         irq_irq10_entry         ; EXTI Line4                                             
    DCD         irq_irq11_entry         ; DMA1 Stream 0                                   
    DCD         irq_irq12_entry         ; DMA1 Stream 1                                   
    DCD         irq_irq13_entry         ; DMA1 Stream 2                                   
    DCD         irq_irq14_entry         ; DMA1 Stream 3                                   
    DCD         irq_irq15_entry         ; DMA1 Stream 4                                   
    DCD         irq_irq16_entry         ; DMA1 Stream 5                                   
    DCD         irq_irq17_entry         ; DMA1 Stream 6                                   
    DCD         irq_irq18_entry         ; ADC1, ADC2 and ADC3s                            
    DCD         irq_irq19_entry         ; CAN1 TX                                                
    DCD         irq_irq20_entry         ; CAN1 RX0                                               
    DCD         irq_irq21_entry         ; CAN1 RX1                                               
    DCD         irq_irq22_entry         ; CAN1 SCE                                               
    DCD         irq_irq23_entry         ; External Line[9:5]s                                    
    DCD         irq_irq24_entry         ; TIM1 Break and TIM9                   
    DCD         irq_irq25_entry         ; TIM1 Update and TIM10                 
    DCD         irq_irq26_entry         ; TIM1 Trigger and Commutation and TIM11
    DCD         irq_irq27_entry         ; TIM1 Capture Compare                                   
    DCD         irq_irq28_entry         ; TIM2                                            
    DCD         irq_irq29_entry         ; TIM3                                            
    DCD         irq_irq30_entry         ; TIM4                                            
    DCD         irq_irq31_entry         ; I2C1 Event                                             
    DCD         irq_irq32_entry         ; I2C1 Error                                             
    DCD         irq_irq33_entry         ; I2C2 Event                                             
    DCD         irq_irq34_entry         ; I2C2 Error                                               
    DCD         irq_irq35_entry         ; SPI1                                            
    DCD         irq_irq36_entry         ; SPI2                                            
    DCD         irq_irq37_entry         ; USART1                                          
    DCD         irq_irq38_entry         ; USART2                                          
    DCD         irq_irq39_entry         ; USART3                                          
    DCD         irq_irq40_entry         ; External Line[15:10]s                                  
    DCD         irq_irq41_entry         ; RTC Alarm (A and B) through EXTI Line                  
    DCD         irq_irq42_entry         ; USB OTG FS Wakeup through EXTI line                        
    DCD         irq_irq43_entry         ; TIM8 Break and TIM12                  
    DCD         irq_irq44_entry         ; TIM8 Update and TIM13                 
    DCD         irq_irq45_entry         ; TIM8 Trigger and Commutation and TIM14
    DCD         irq_irq46_entry         ; TIM8 Capture Compare                                   
    DCD         irq_irq47_entry         ; DMA1 Stream7                                           
    DCD         irq_irq48_entry         ; FSMC                                            
    DCD         irq_irq49_entry         ; SDIO                                            
    DCD         irq_irq50_entry         ; TIM5                                            
    DCD         irq_irq51_entry         ; SPI3                                            
    DCD         irq_irq52_entry         ; UART4                                           
    DCD         irq_irq53_entry         ; UART5                                           
    DCD         irq_irq54_entry         ; TIM6 and DAC1&2 underrun errors                   
    DCD         irq_irq55_entry         ; TIM7                   
    DCD         irq_irq56_entry         ; DMA2 Stream 0                                   
    DCD         irq_irq57_entry         ; DMA2 Stream 1                                   
    DCD         irq_irq58_entry         ; DMA2 Stream 2                                   
    DCD         irq_irq59_entry         ; DMA2 Stream 3                                   
    DCD         irq_irq60_entry         ; DMA2 Stream 4                                   
    DCD         irq_irq61_entry         ; Ethernet                                        
    DCD         irq_irq62_entry         ; Ethernet Wakeup through EXTI line                      
    DCD         irq_irq63_entry         ; CAN2 TX                                                
    DCD         irq_irq64_entry         ; CAN2 RX0                                               
    DCD         irq_irq65_entry         ; CAN2 RX1                                               
    DCD         irq_irq66_entry         ; CAN2 SCE                                               
    DCD         irq_irq67_entry         ; USB OTG FS                                      
    DCD         irq_irq68_entry         ; DMA2 Stream 5                                   
    DCD         irq_irq69_entry         ; DMA2 Stream 6                                   
    DCD         irq_irq70_entry         ; DMA2 Stream 7                                   
    DCD         irq_irq71_entry         ; USART6                                           
    DCD         irq_irq72_entry         ; I2C3 event                                             
    DCD         irq_irq73_entry         ; I2C3 error                                             
    DCD         irq_irq74_entry         ; USB OTG HS End Point 1 Out                      
    DCD         irq_irq75_entry         ; USB OTG HS End Point 1 In                       
    DCD         irq_irq76_entry         ; USB OTG HS Wakeup through EXTI                         
    DCD         irq_irq77_entry         ; USB OTG HS                                      
    DCD         irq_irq78_entry         ; DCMI                                            
    DCD         irq_irq79_entry         ; CRYP crypto                                     
    DCD         irq_irq80_entry         ; Hash and Rng
    DCD         irq_irq81_entry         ; FPU

; Макрос для перехода по программной таблице прерываний
    
    MACRO                                              ; начало области задания макроса 
    IRQ_JMP     $IDX                                   ; имя и параметр макроса
    
    LDR         r0,=|Image$$vector_table_base$$Base|   ; запись адреса таблицы веторов прерываний (0x10000000) в регистр
    LDR         r0,[r0]                                ; запись в регистр адреса первого обработчика прерывания
    ; Считывание адреса точки входа из таблицы прерывания
    MOVS        r1,#( $IDX * 4 )                       ; подсчет смещения относительно начала таблицы векторов
    LDR         r0,[r0,r1]                             ; запись в регистр адреса обработчика по соответствующему смещению
    ; (возврата из прерывания в этот модуль не предусматривается)
    BX          r0                                     ; переход в прерывание
    LTORG

    MEND

; Точка входа в прерывание Reset
irq_reset_entry PROC
    LDR         R0,=init_startup
    BX          R0
    NOP                 ; данная инструкция ничего не делает, но ее можно использовать для заполенния с целью выравнивания
    ENDP

; Точка входа в прерывание Non-maskable Interrupt
irq_nmi_entry PROC
    IRQ_JMP     2
    NOP
    ENDP

; Точка входа в прерывание Hard Fault
irq_hardfault_entry PROC
    IRQ_JMP     3
    NOP
    ENDP

; Точка входа в прерывание Memory management
irq_memmanage_entry PROC
    IRQ_JMP     4
    NOP
    ENDP

; Точка входа в прерывание BusFault
irq_busfault_entry PROC
    IRQ_JMP     5
    NOP
    ENDP

; Точка входа в прерывание UsageFault
irq_usagefault_entry PROC
    IRQ_JMP     6
    NOP
    ENDP

; Точка входа в прерывание Reserved
irq_exeption7_entry PROC
    IRQ_JMP     7
    NOP
    ENDP

; Точка входа в прерывание Reserved
irq_exeption8_entry PROC
    IRQ_JMP     8
    NOP
    ENDP

; Точка входа в прерывание Reserved
irq_exeption9_entry PROC
    IRQ_JMP     9
    NOP
    ENDP

; Точка входа в прерывание Reserved
irq_exeption10_entry PROC
    IRQ_JMP     10
    NOP
    ENDP

; Точка входа в прерывание SVC
irq_svc_entry PROC
    IRQ_JMP     11
    NOP
    ENDP

; Точка входа в прерывание Debug Monitor
irq_debugmonitor_entry PROC
    IRQ_JMP     12
    NOP
    ENDP

; Точка входа в прерывание Reserved
irq_exeption13_entry PROC
    IRQ_JMP     13
    NOP
    ENDP

; Точка входа в прерывание PendSV
irq_pendsv_entry PROC
    IRQ_JMP     14
    NOP
    ENDP

; Точка входа в прерывание SysTick
irq_systick_entry PROC
    IRQ_JMP     15
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 0
irq_irq0_entry PROC
    IRQ_JMP     16
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 1
irq_irq1_entry PROC
    IRQ_JMP     17
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 2
irq_irq2_entry PROC
    IRQ_JMP     18
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 3
irq_irq3_entry PROC
    IRQ_JMP     19
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 4
irq_irq4_entry PROC
    IRQ_JMP     20
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 5
irq_irq5_entry PROC
    IRQ_JMP     21
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 6
irq_irq6_entry PROC
    IRQ_JMP     22
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 7
irq_irq7_entry PROC
    IRQ_JMP     23
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 8
irq_irq8_entry PROC
    IRQ_JMP     24
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 9
irq_irq9_entry PROC
    IRQ_JMP     25
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 10
irq_irq10_entry PROC
    IRQ_JMP     26
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 11
irq_irq11_entry PROC
    IRQ_JMP     27
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 12
irq_irq12_entry PROC
    IRQ_JMP     28
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 13
irq_irq13_entry PROC
    IRQ_JMP     29
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 14
irq_irq14_entry PROC
    IRQ_JMP     30
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 15
irq_irq15_entry PROC
    IRQ_JMP     31
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 16
irq_irq16_entry PROC
    IRQ_JMP     32
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 17
irq_irq17_entry PROC
    IRQ_JMP     33
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 18
irq_irq18_entry PROC
    IRQ_JMP     34
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 19
irq_irq19_entry PROC
    IRQ_JMP     35
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 20
irq_irq20_entry PROC
    IRQ_JMP     36
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 21
irq_irq21_entry PROC
    IRQ_JMP     37
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 22
irq_irq22_entry PROC
    IRQ_JMP     38
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 23
irq_irq23_entry PROC
    IRQ_JMP     39
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 24
irq_irq24_entry PROC
    IRQ_JMP     40
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 25
irq_irq25_entry PROC
    IRQ_JMP     41
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 26
irq_irq26_entry PROC
    IRQ_JMP     42
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 27
irq_irq27_entry PROC
    IRQ_JMP     43
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 28
irq_irq28_entry PROC
    IRQ_JMP     44
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 29
irq_irq29_entry PROC
    IRQ_JMP     45
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 30
irq_irq30_entry PROC
    IRQ_JMP     46
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 31
irq_irq31_entry PROC
    IRQ_JMP     47
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 32
irq_irq32_entry PROC
    IRQ_JMP     48
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 33
irq_irq33_entry PROC
    IRQ_JMP     49
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 34
irq_irq34_entry PROC
    IRQ_JMP     50
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 35
irq_irq35_entry PROC
    IRQ_JMP     51
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 36
irq_irq36_entry PROC
    IRQ_JMP     52
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 37
irq_irq37_entry PROC
    IRQ_JMP     53
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 38
irq_irq38_entry PROC
    IRQ_JMP     54
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 39
irq_irq39_entry PROC
    IRQ_JMP     55
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 40
irq_irq40_entry PROC
    IRQ_JMP     56
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 41
irq_irq41_entry PROC
    IRQ_JMP     57
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 42
irq_irq42_entry PROC
    IRQ_JMP     58
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 43
irq_irq43_entry PROC
    IRQ_JMP     59
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 44
irq_irq44_entry PROC
    IRQ_JMP     60
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 45
irq_irq45_entry PROC
    IRQ_JMP     61
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 46
irq_irq46_entry PROC
    IRQ_JMP     62
    NOP
    ENDP

; Точка входа в прерывание External Interrupts 47
irq_irq47_entry PROC
    IRQ_JMP     63
    ENDP

; Точка входа в прерывание External Interrupts 48
irq_irq48_entry PROC
    IRQ_JMP     64
    ENDP

; Точка входа в прерывание External Interrupts 49
irq_irq49_entry PROC
    IRQ_JMP     65
    ENDP

; Точка входа в прерывание External Interrupts 50
irq_irq50_entry PROC
    IRQ_JMP     66
    ENDP

; Точка входа в прерывание External Interrupts 51
irq_irq51_entry PROC
    IRQ_JMP     67
    ENDP

; Точка входа в прерывание External Interrupts 52
irq_irq52_entry PROC
    IRQ_JMP     68
    ENDP

; Точка входа в прерывание External Interrupts 53
irq_irq53_entry PROC
    IRQ_JMP     69
    ENDP

; Точка входа в прерывание External Interrupts 54
irq_irq54_entry PROC
    IRQ_JMP     70
    ENDP

; Точка входа в прерывание External Interrupts 55
irq_irq55_entry PROC
    IRQ_JMP     71
    ENDP

; Точка входа в прерывание External Interrupts 56
irq_irq56_entry PROC
    IRQ_JMP     72
    ENDP

; Точка входа в прерывание External Interrupts 57
irq_irq57_entry PROC
    IRQ_JMP     73
    ENDP

; Точка входа в прерывание External Interrupts 58
irq_irq58_entry PROC
    IRQ_JMP     74
    ENDP

; Точка входа в прерывание External Interrupts 59
irq_irq59_entry PROC
    IRQ_JMP     75
    ENDP

; Точка входа в прерывание External Interrupts 60
irq_irq60_entry PROC
    IRQ_JMP     76
    ENDP

; Точка входа в прерывание External Interrupts 61
irq_irq61_entry PROC
    IRQ_JMP     77
    ENDP

; Точка входа в прерывание External Interrupts 62
irq_irq62_entry PROC
    IRQ_JMP     78
    ENDP

; Точка входа в прерывание External Interrupts 63
irq_irq63_entry PROC
    IRQ_JMP     79
    ENDP

; Точка входа в прерывание External Interrupts 64
irq_irq64_entry PROC
    IRQ_JMP     80
    ENDP

; Точка входа в прерывание External Interrupts 65
irq_irq65_entry PROC
    IRQ_JMP     81
    ENDP

; Точка входа в прерывание External Interrupts 66
irq_irq66_entry PROC
    IRQ_JMP     82
    ENDP

; Точка входа в прерывание External Interrupts 67
irq_irq67_entry PROC
    IRQ_JMP     83
    ENDP

; Точка входа в прерывание External Interrupts 68
irq_irq68_entry PROC
    IRQ_JMP     84
    ENDP

; Точка входа в прерывание External Interrupts 69
irq_irq69_entry PROC
    IRQ_JMP     85
    ENDP

; Точка входа в прерывание External Interrupts 70
irq_irq70_entry PROC
    IRQ_JMP     86
    ENDP

; Точка входа в прерывание External Interrupts 71
irq_irq71_entry PROC
    IRQ_JMP     87
    ENDP

; Точка входа в прерывание External Interrupts 72
irq_irq72_entry PROC
    IRQ_JMP     88
    ENDP

; Точка входа в прерывание External Interrupts 73
irq_irq73_entry PROC
    IRQ_JMP     89
    ENDP

; Точка входа в прерывание External Interrupts 74
irq_irq74_entry PROC
    IRQ_JMP     90
    ENDP

; Точка входа в прерывание External Interrupts 75
irq_irq75_entry PROC
    IRQ_JMP     91
    ENDP

; Точка входа в прерывание External Interrupts 76
irq_irq76_entry PROC
    IRQ_JMP     92
    ENDP

; Точка входа в прерывание External Interrupts 77
irq_irq77_entry PROC
    IRQ_JMP     93
    ENDP

; Точка входа в прерывание External Interrupts 78
irq_irq78_entry PROC
    IRQ_JMP     94
    ENDP

; Точка входа в прерывание External Interrupts 79
irq_irq79_entry PROC
    IRQ_JMP     95
    ENDP

; Точка входа в прерывание External Interrupts 80
irq_irq80_entry PROC
    IRQ_JMP     96
    ENDP

; Точка входа в прерывание External Interrupts 81
irq_irq81_entry PROC
    IRQ_JMP     97
    ENDP

    AREA        |.loader_text|, CODE, READONLY


; Пользовательские настройки стека и кучи
    IMPORT      |Image$$loader_stack$$ZI$$Base|
    IMPORT      |Image$$loader_stack$$ZI$$Limit|
    EXPORT      __user_initial_stackheap

__user_initial_stackheap

    LDR     R0, =  Heap_Mem
    LDR     R1, = |Image$$loader_stack$$ZI$$Limit|
    LDR     R2, = (Heap_Mem +  Heap_Size)
    LDR     R3, = |Image$$loader_stack$$ZI$$Base|
    BX      LR

    ALIGN
        
    END

