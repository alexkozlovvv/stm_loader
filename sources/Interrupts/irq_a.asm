    PRESERVE8
    THUMB

    IMPORT      kern_irq_manager
    IMPORT      |Image$$lvl01_stack$$ZI$$Limit|

    EXPORT      kern_irq_exception_a
    EXPORT      kern_irq_disable_interrupt
    EXPORT      kern_irq_set_msr

    AREA        |.loader_text|, CODE, READONLY
; Подпрограмма оболочка обработчика исключительной ситуации (exeption)
; void kern_irq_exception_a( void )
; В стеке (прерванного процесса) сохранены
; SP +0     r0
;    +4     r1
;    +8     r2
;    +12    r3
;    +16    r12
;    +20    lr
;    +24    ReturnAddress
;    +28    xPSR
; SP_old = SP - 28
kern_irq_exception_a PROC
    ; Пролог
    stmdb       sp!,{r0,r1,r4,lr}
;               [sp,#0]             ; r9
;               [sp,#4]             ; резерв
;               [sp,#8]             ; r4
;               [sp,#12]            ; lr

frame_save_sv_exp           ; пока не используется
    ; Сохранение в стеке
    ; Фрейм прерывания в стеке MSP
    add         r4,sp,#16
;   b           frame_save_exp

frame_save_user_exp         ; пока не используется
    ; Сохранение в описании контекста потока
    ; Надо сохранить только один регистр, так что сохраняем его в стеке
    ; Фрейм прерывания в стеке PSP
;   mrs         r4,PSP

frame_save_exp          ; пока не используется
    ; Частичное сохранение регистров
    ; За сохранение прочих регистров отвечает компилятор С в соответствии с
    ; соглашением о вызовах
    ; r0 - r3, r12 сохранения при входе в прерывание
    mov         r0,r9
    str         r0,[sp]

    ; sid_old = kern_mmu_set_sv_sid( MMU_LVL0_SID )
    ; Не поддерживаися аппаратурой

    ; Вызов диспетчера прерываний
    ; void kern_irq_manager(
    ;     uint32_t frame, const void *address, uint32_t param )
    mov         r0,r4               ; Первый параметр frame
    ldr         r1,[r4,#24]         ; Второй параметр address
    mrs         r2,IPSR             ; Третий параметр param
    ldr         r3,=kern_irq_manager
    blx         r3

    ; Эпилог

    ; Анализ прерванного режима работы
    ; Не требуется

restore_sv_exp                                    ; пока не используется
    ; ( void ) kern_mmu_set_sv_sid( sid_old )

restore_user_exp                                    ; пока не используется
end_exp
    ; Частичное восстановление регистров
    ; За восстановление прочих регистров отвечает компилятор С в соответствии
    ; с соглашением о вызовах
    ; r0 - r3, r12 восстановятся при выходе из прерывания
    ldr         r0,[sp]
    mov         r9,r0

    ldmia       sp!,{r0,r1,r4,pc}   ; Выход из прерывания
    ENDP

    ALIGN

    AREA        |.loader_text|, CODE, READONLY
; Подпрограмма запрета прерываний (глобального)
; Результат:
;   Значение регистра PRIMASK на момент запрета прерываний
; kern_irq_disable_interrupt( void )
kern_irq_disable_interrupt PROC 
    mrs         r0,PRIMASK              ; закидываем занчение указанного регистра сопроцессора в регистр ARM
    cpsid       i                       ; меняем состояние процессора, отключаем прерывания ??? выставляем PRIMASK PM to 1
; Согласно описанию ARMv6-M B1.4.6 синхронизация не требуется,
; однако согласно Application Note AN321 настоятельно рекомендуется.
;   dsb
    isb                                  ; данная команда описана уже
    bx          lr                       ; Branch and exchange instruction set of link register. Т.е. в нем хранится адрес возврата из подпрограммы
    ENDP

    AREA        |.loader_text|, CODE, READONLY
; Подпрограмма задания значения регистра MSR
;   (in)  msr_value - задаваемое значение регистра PRIMASK
; void kern_irq_set_msr( uint32_t msr_value )
kern_irq_set_msr PROC
    msr         PRIMASK,r0
; Согласно описанию ARMv6-M B1.4.6 синхронизация не требуется,
; однако согласно Application Note AN321 настоятельно рекомендуется.
;   dsb

    isb
    bx          lr
    ENDP

    END
