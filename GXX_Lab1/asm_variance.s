	AREA test, CODE, READONLY
	
	export asm_variance                  ; label must be exported if it is to be used as a function in C
asm_variance

	PUSH{R4 - R12}                    ; saving context according to calling convention
	MOV R4, R1                     	; counter for loop
	VMOV.f32 S7, R4
	MOV R5, #0
	VMOV.f32 S4, R5
	VMOV.f32 S5, R5
	MOV R5, R0
	VMOV.f32 S5, R1
loop
	SUBS R4, R4, #1                 ; loop counter (N = N-1)
	BLT continue                        ; loop has finished?
	VLDR.f32 S0, [R0]
	VADD.f32 S4, S4, S0
	ADD R0, R0, #4
	B loop

continue
	VDIV.f32 S4, S4, S7
	MOV R4, R1
varianceloop
	SUBS R4, R4, #1
	BLT done
	VLDR.f32 S0, [R5]
	VSUB.f32 S6, S0, S4
	VMUL.f32 S6, S6, S6
	VADD.f32 S5, S5, S6
	ADD R5, R5, #4
	B varianceloop
done
	VDIV.f32 S4, S5, S7 
	VSTR.f32 S4, [R2]               ; store dot product in the pointer
	
	POP{R4 - R12}                     ; restore context
	BX LR                           ; return
	
	END
