	AREA test, CODE, READONLY
	
	export asm_dot_product                  ; label must be exported if it is to be used as a function in C
asm_dot_product

	PUSH{R4, R5}                    ; saving context according to calling convention
	MOV R4, R2                     	; counter for loop
	MOV R5, #0
	VMOV.f32 S4, R5
loop
	SUBS R4, R4, #1                 ; loop counter (N = N-1)
	BLT done                        ; loop has finished?
	VLDR.f32 S0, [R0]
	VLDR.f32 S1, [R1]
	VMUL.f32 S2, S0, S1
	VADD.f32 S4, S4, S2
	ADD R0, R0, #4
	ADD R1, R1, #4
	B loop

done
	VSTR.f32 S4, [R3]               ; store dot product in the pointer
	
	POP{R4, R5}                     ; restore context
	BX LR                           ; return
	
	END
