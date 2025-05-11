    PRESERVE8
    THUMB

    IMPORT      kern_irq_manager
    IMPORT      |Image$$lvl01_stack$$ZI$$Limit|

    EXPORT      kern_irq_exception_a
    EXPORT      kern_irq_disable_interrupt
    EXPORT      kern_irq_set_msr

    AREA        |.loader_text|, CODE, READONLY
; ������������ �������� ����������� �������������� �������� (exeption)
; void kern_irq_exception_a( void )
; � ����� (����������� ��������) ���������
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
    ; ������
    stmdb       sp!,{r0,r1,r4,lr}
;               [sp,#0]             ; r9
;               [sp,#4]             ; ������
;               [sp,#8]             ; r4
;               [sp,#12]            ; lr

frame_save_sv_exp           ; ���� �� ������������
    ; ���������� � �����
    ; ����� ���������� � ����� MSP
    add         r4,sp,#16
;   b           frame_save_exp

frame_save_user_exp         ; ���� �� ������������
    ; ���������� � �������� ��������� ������
    ; ���� ��������� ������ ���� �������, ��� ��� ��������� ��� � �����
    ; ����� ���������� � ����� PSP
;   mrs         r4,PSP

frame_save_exp          ; ���� �� ������������
    ; ��������� ���������� ���������
    ; �� ���������� ������ ��������� �������� ���������� � � ������������ �
    ; ����������� � �������
    ; r0 - r3, r12 ���������� ��� ����� � ����������
    mov         r0,r9
    str         r0,[sp]

    ; sid_old = kern_mmu_set_sv_sid( MMU_LVL0_SID )
    ; �� ������������� �����������

    ; ����� ���������� ����������
    ; void kern_irq_manager(
    ;     uint32_t frame, const void *address, uint32_t param )
    mov         r0,r4               ; ������ �������� frame
    ldr         r1,[r4,#24]         ; ������ �������� address
    mrs         r2,IPSR             ; ������ �������� param
    ldr         r3,=kern_irq_manager
    blx         r3

    ; ������

    ; ������ ����������� ������ ������
    ; �� ���������

restore_sv_exp                                    ; ���� �� ������������
    ; ( void ) kern_mmu_set_sv_sid( sid_old )

restore_user_exp                                    ; ���� �� ������������
end_exp
    ; ��������� �������������� ���������
    ; �� �������������� ������ ��������� �������� ���������� � � ������������
    ; � ����������� � �������
    ; r0 - r3, r12 ������������� ��� ������ �� ����������
    ldr         r0,[sp]
    mov         r9,r0

    ldmia       sp!,{r0,r1,r4,pc}   ; ����� �� ����������
    ENDP

    ALIGN

    AREA        |.loader_text|, CODE, READONLY
; ������������ ������� ���������� (�����������)
; ���������:
;   �������� �������� PRIMASK �� ������ ������� ����������
; kern_irq_disable_interrupt( void )
kern_irq_disable_interrupt PROC 
    mrs         r0,PRIMASK              ; ���������� �������� ���������� �������� ������������ � ������� ARM
    cpsid       i                       ; ������ ��������� ����������, ��������� ���������� ??? ���������� PRIMASK PM to 1
; �������� �������� ARMv6-M B1.4.6 ������������� �� ���������,
; ������ �������� Application Note AN321 ������������ �������������.
;   dsb
    isb                                  ; ������ ������� ������� ���
    bx          lr                       ; Branch and exchange instruction set of link register. �.�. � ��� �������� ����� �������� �� ������������
    ENDP

    AREA        |.loader_text|, CODE, READONLY
; ������������ ������� �������� �������� MSR
;   (in)  msr_value - ���������� �������� �������� PRIMASK
; void kern_irq_set_msr( uint32_t msr_value )
kern_irq_set_msr PROC
    msr         PRIMASK,r0
; �������� �������� ARMv6-M B1.4.6 ������������� �� ���������,
; ������ �������� Application Note AN321 ������������ �������������.
;   dsb

    isb
    bx          lr
    ENDP

    END
