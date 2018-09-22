	AREA test, CODE, READONLY
	
	export asm_variance             ; label must be exported if it is to be used as a function in C
asm_variance

	PUSH{R4, R5}                    ; saving context according to calling convention since using R4, R5 
	MOV R4, R1                     	; counter for loop
	VMOV.f32 S7, R4					; use as a floating point number when dividing
	VCVT.f32.s32 S7, S7				; have to convert to floating point from signed integer
	VLDR.f32 S4, =0					; load floating point value, used as mean
	VLDR.f32 S5, =0					; load floating point value, used as variance
	MOV R5, R0						; load another register for second loop iteration, don't have to reset manually
loop
	SUBS R4, R4, #1                 ; loop counter (N = N-1)
	BLT continue                    ; loop has finished?
	VLDR.f32 S0, [R0]				; load array value into memory
	VADD.f32 S4, S4, S0				; add to the total sum
	ADD R0, R0, #4					; on next cycle get the next element in the vector
	B loop							; branch
continue
	VDIV.f32 S4, S4, S7				; divide the total by the number of elements to get the mean
	MOV R4, R1						; set R4 back to size for next loop iteration
varianceloop
	SUBS R4, R4, #1					; loop counter (N = N - 1);
	BLT done						; counter for loop
	VLDR.f32 S0, [R5]				; load array element into S0 from R5
	VSUB.f32 S6, S0, S4				; subtract element and mean
	VMUL.f32 S6, S6, S6				; multiply the element by itself to mimic squaring
	VADD.f32 S5, S5, S6				; add it to the variance element
	ADD R5, R5, #4					; get next element
	B varianceloop
done
	VDIV.f32 S4, S5, S7 			; divide by size of the array for variance
	VSTR.f32 S4, [R2]               ; store variance at the address of the floating point passed	
	POP{R4, R5}                     ; restore context
	BX LR                           ; return
	
	END
