; В функции исключена инициализации приватной области хранения 
; используемой стандартными библиотеками при инициализации

    PRESERVE8
    THUMB
    
    IMPORT      __user_initial_stackheap
    IMPORT      main
    
    IMPORT      |Image$$loader_out$$ZI$$Length|
    IMPORT      |Image$$loader_stack$$Base|
    IMPORT      |Image$$loader_stack$$ZI$$Length|
    
    IMPORT      |Image$$loader_data$$Limit|
    IMPORT      |Image$$loader_data$$ZI$$Limit|
    IMPORT      |Image$$loader_data$$Base|
    IMPORT      |Image$$loader_data$$ZI$$Length|
    IMPORT      |Image$$loader_data$$RW$$Length|        


    EXPORT      __main
        
    AREA        |.loader_text|, CODE, READONLY
    
__main PROC 
    
    BL.W    __scatterload
    BL.W    __rt_entry 

    ENDP
       
; ============    __scatterload   ==============

; Обнуление областей стека и неинициализированных переменных          
__scatterload PROC
    
    MOV           r7,lr
    
    ldr    r0,=|Image$$loader_data$$ZI$$Limit|
    ldr    r1,=|Image$$loader_stack$$Base|
    ldr    r2,=|Image$$loader_stack$$ZI$$Length|   
   
    
    MOVS          r3,#0x00
    MOVS          r4,#0x00
    MOVS          r5,#0x00
    MOVS          r6,#0x00
    BL.W          __put_null
    
    ldr    r1,=|Image$$loader_data$$Base|
    ldr    r2,=|Image$$loader_data$$ZI$$Length|
    ldr    r3,=|Image$$loader_data$$RW$$Length|
    add    r2,r3
    
    MOVS          r3,#0x00
    
    BL.W          __put_null
    MOV           lr,r7
    BX            lr

    ENDP

__put_null PROC

put_null
    SUBS          r2,r2,#0x10  ; N = 0; Z = 0; C = 1
    IT            CS
    STMCS         r1!,{r3-r6}
    BHI           put_null  ; N = 0; Z = 1; C = 1 в конце
    LSLS          r2,r2,#29    ; N = 0; Z = 1; C = 0
    IT            CS
    STMCS         r1!,{r4-r5}
    IT            MI
    STRMI         r3,[r1,#0x00]
    BX            lr
    
    ENDP
        
; ============    __rt_entry   ==============

; Инициализация области стека и настройка floating-point system
__rt_entry PROC
    
    BL.W          __user_setup_stackheap
    MOV           r1,r2
    BL.W          __rt_lib_init 
    BL.W          main         ; Выход в пользовательскую программу main
    
    ENDP

; Задание значения sp
__user_setup_stackheap PROC

    MOV           r5,lr
    BL.W          __user_initial_stackheap
    MOV           lr,r5
    MOV           sp,r1
    BX            lr
   
    ENDP
    
__rt_lib_init PROC 
    
    PUSH          {r0-r4,lr}   ; Сохранение регистров в стек
    BL.W          __fp_init
    POP           {r0-r4,pc}   ; Восстановление регистров и переход к инструкции (BL.W  main)

    ENDP

;  Выставление рабочих настроек 
;  floating-point system
__fp_init PROC
    
    MOV           r0,#0x2000000   ; Default NaN mode enable
    VMSR          FPSCR, r0
    BX            lr

    ENDP
        
    ALIGN

    END
   