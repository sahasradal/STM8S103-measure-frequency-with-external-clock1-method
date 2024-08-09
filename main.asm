stm8/
	;works fine ,PC7 as input,clock 1 mode. can measure up to 16711425 hz
	; 
	#include "mapping.inc"
	#include "stm8s103f.inc"

pointerX MACRO first
	ldw X,first
	MEND
pointerY MACRO first
	ldw Y,first
	MEND	



	segment byte at 100 'ram1'
buffer1 ds.b
buffer2 ds.b
buffer3 ds.b
buffer4 ds.b
buffer5 ds.b
buffer6 ds.b
buffer7 ds.b
buffer8 ds.b
buffer9 ds.b
buffer10 ds.b
buffer11 ds.b
buffer12 ds.b
buffer13 ds.b	; remainder byte 0 (LSB)
buffer14 ds.b	; remainder byte 1
buffer15 ds.b	; remainder byte 2
buffer16 ds.b	; remainder byte 3 (MSB)
buffer17 ds.b	; loop counter
captureH ds.b
captureL ds.b	
captureHS ds.b
captureLS ds.b
capture_state ds.b	
nibble1  ds.b
data	 ds.b
address  ds.b
signbit  ds.b
state    ds.b
temp1    ds.b
result4  ds.b
result3  ds.b
result2  ds.b
result1  ds.b
counter1 ds.b
counter2 ds.b
buffers  ds.b 23		
	
	segment 'rom'
main.l
	; initialize SP
	ldw X,#stack_end
	ldw SP,X

	#ifdef RAM0	
	; clear RAM0
ram0_start.b EQU $ram0_segment_start
ram0_end.b EQU $ram0_segment_end
	ldw X,#ram0_start
clear_ram0.l
	clr (X)
	incw X
	cpw X,#ram0_end	
	jrule clear_ram0
	#endif

	#ifdef RAM1
	; clear RAM1
ram1_start.w EQU $ram1_segment_start
ram1_end.w EQU $ram1_segment_end	
	ldw X,#ram1_start
clear_ram1.l
	clr (X)
	incw X
	cpw X,#ram1_end	
	jrule clear_ram1
	#endif

	; clear stack
stack_start.w EQU $stack_segment_start
stack_end.w EQU $stack_segment_end
	ldw X,#stack_start
clear_stack.l
	clr (X)
	incw X
	cpw X,#stack_end	
	jrule clear_stack

main_loop.l
	  mov CLK_CKDIVR,#$0    ; set max internal clock 16mhz
	  mov TIM1_CCMR2,#$01 	; CC2 channel is configured as input, IC2 is mapped on TI2FP2,PC7
	  bres TIM1_CCER1,#5	; Trigger on a high level or rising edge of TI3F
	  mov TIM1_SMCR,#$67	; 110: Filtered timer input 2 (TI2FP2) in  bit,111: External clock mode 1 - Rising edges of the selected trigger (TRGI) clock the counte
	  bset TIM1_IER,#0		; enable update interrupt
	  bset TIM1_CR1,#0		; enable timer1

uart_setup:
	 ;UART1_TX PD5
	 ;UART1_RX PD6
	 ld a,#$03				;$0683 = 9600 ,$008B = 115200, 
	 ld UART1_BRR2,a		; write BRR2 firdt
	 ld a,#$68
	 ld UART1_BRR1,a		; write BRR1 next
	 bset UART1_CR2,#3		; enable TX
	 bset UART1_CR2,#2		; enable RX

setup						; timer4 setup for delay 1ms base timer
	  ldw x,#500
	  ldw counter1,x
	  bset PD_DDR,#4
	  bset PD_ODR,#4
	  mov TIM4_PSCR ,#$07	; select timer 4 prescaler to 128, 7 means 2^7=128
	  mov TIM4_ARR,#249		; 16mhz/128=125000 =1s,125000/1000=125,load 125-1 as 0 is counted=124
	  bset TIM4_IER,#0		; enable update interrupt in interrupt register
	  bset TIM4_CR1,#0		; enable timer4
	  rim					; enable interrupt globally
	  
	  
	pointerX #string
stringloop:
	ld a,(X)
	incw X
	cp a,#$00
	jreq exitstringloop
	ld data,a
	call UART_TX
	jp stringloop
exitstringloop:
	nop
	
	
	
	
	  
here
	ld a,state
	cp a,#1
	jrne here
	clr state
	bset TIM1_CR1,#0		; enable timer1
	bcpl PD_ODR,#4
	
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	  
calculate_transmit:	
	ld a,buffers
	ldw X,#65535
	mul x,a
	ldw {buffers + 2},x
	ld a,#$ff
	ld xl,a
	ld a,buffers
	mul x,a					; multiply MSB with 16
	ld a,xl
	add a,{buffers + 2}
	ld {buffers +2},a
	ld a,xh
	adc a,#0
	ld {buffers +1},a
	clr buffers
	ldw X,captureH			; load word from capturH & captureL 
	addw x,{buffers +2}
	ldw {buffers +2},x
	ld a,{buffers + 1}
	adc a,#0
	ld {buffers + 1},a
	ldw x,buffers
	ldw buffer14,X
	ldw x,{buffers +2}
	ldw buffer16,x
	call bin_to_ascii		; procedure to convert binary value to ASCII , to be converted values in buffer15,16,17 msb to lsb , result in buffers to buffers+11
	pointerX #buffers   	; point X to buffers register , start point of ascii value storage, ascii values in buffers
	mov temp1,#11			; temp1 as counter, 10 values to be printed
TX_LOOP:
	ld a,(x)				; load A with value of string pointed by X
	ld data,a				; copy yte in A to data register
	call UART_TX			; call UART transmit subroutine
	incw X					; increase pointer X
	dec temp1				; decrease temp1 counter value
	jrne TX_LOOP			; loop to TX_LOOP label till temp1 is above 0	  
	  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	jp here
	  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
A32bit_subtraction:	
	ld a,buffer8
	sub a,buffer4
	ld result1,a
	ld a,buffer7
	sbc a,buffer3
	ld result2,a
	ld a,buffer6
	sbc a,buffer2
	ld result3,a
	ld a,buffer5
	sbc a,buffer1
	ld result4,a
	JRULT load_signbit_register
	clr signbit
	ret
load_signbit_register
	mov signbit,#1
	ret
	
	

A32bit_subtraction1:	
	ld a,result1
	sub a,buffer4
	ld result1,a
	ld a,result2
	sbc a,buffer3
	ld result2,a
	ld a,result3
	sbc a,buffer2
	ld result3,a
	ld a,result4
	sbc a,buffer1
	ld result4,a
	JRULT load_signbit_register1
	clr signbit
	ret
load_signbit_register1
	mov signbit,#1
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

bin_to_ascii:
	; procedure to convert binary value to ASCII , to be converted values in buffer15,16,17 msb to lsb , result in buffers to buffers+11
	ldw x,buffer16	; load low word of the value to be converted into ASCII into index register X
	ldw data,x		; result 16bit word stored in lower word buffer16 , buffer17 in data and address registers
	ld a,buffer15	; result MSB in buffer15 stored in nibble register sram, concecutive bytes
	ld nibble1,a	; result MSB in buffer15 stored in nibble register sram, concecutive bytes
	clr buffer1		; clear sram registers for bin_to_ascii calculations
	clr buffer2		; clear sram registers for bin_to_ascii calculations
	clr buffer3		; clear sram registers for bin_to_ascii calculations
	clr buffer4		; clear sram registers for bin_to_ascii calculations
	clr buffer5		; clear sram registers for bin_to_ascii calculations
	clr buffer6		; clear sram registers for bin_to_ascii calculations
	clr buffer7		; clear sram registers for bin_to_ascii calculations
	clr buffer8		; clear sram registers for bin_to_ascii calculations
	clr result4		; clear sram registers for bin_to_ascii calculations
	clr result3		; clear sram registers for bin_to_ascii calculations
	clr result2		; clear sram registers for bin_to_ascii calculations
	clr result1		; clear sram registers for bin_to_ascii calculations
	clr signbit		; clear sram registers for bin_to_ascii calculations
onecrore:
	ldw x,#$9680		; load x with low word of 10,000,000
	ldw buffer3,x		; store in buffer3 and buffer4
	ldw x,#$0098		; load x with high word of 10,000,000
	ldw buffer1,x		; store in buffer1 and buffer2,(buffer1,2,3,4 used for holding test value)
	mov buffer6,nibble1	; mov MSB of result in nibble1 to buffer6 (buffer5,6,7,8 used for holding result)
	ldw x,data			; load result word (LSB1,LSB0) to data & address register in sran (concecutive) 
	ldw buffer7,x		; load result word (LSB1,LSB0) to data & address register in sran (concecutive)
	call A32bit_subtraction	; call 32 bit subtraction routine, buffer5,6,7,8 - buffer1,2,3,4)
	inc temp1			; increase temp register to count how many 1 crrore in result
	ld a,signbit		; copy signbit register contents to accumulator
	jreq onecrore		; if signbit register is 0 (previous subtraction didnt result in negative) branch onecrore label
	dec temp1			; if negative value in subtraction , decrease temp register (we dont count)
revert_result0:	
	ld a,result1		; laod A with LSB of sutracted result1
	add a,buffer4		; add A with LSB0 of value subtracted. we reverse the result to pre negative value
	ld result1,a		; rectified LSB0 stored back in result1 
	ld a,result2		; laod A with LSB1 of sutracted result2
	adc a,buffer3		; add A with LSB1 of value subtracted. we reverse the result to pre negative value
	ld result2,a		; rectified LSB1 stored back in result2
	ld a,result3		; laod A with LSB2 of sutracted result3
	adc a,buffer2		; add A with LSB2 of value subtracted. we reverse the result to pre negative value
	ld result3,a		; rectified LSB2 stored back in result3 
	ld a,result4		; laod A with MSB of sutracted result4
	adc a,buffer1		; add A with MSB of value subtracted. we reverse the result to pre negative value
	ld result4,a		; rectified MSB stored back in result3 
	ld a,#$30			; ascii 0 loaded in A
	add a,temp1			; add temp1 (contains how many decimal places) to ascii 0 to get ascii value of poaition
	ld buffers ,a		; store result of ascii conversion of MSB position in buffers register SRAM
	clr temp1			; clear temp1 for next decimal position calculation
tenlakh:
	ldw x,#$4240
	ldw buffer3,x
	ldw x,#$000f
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq tenlakh
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 1} ,a	
	clr temp1
onelakh:
	ldw x,#$86A0
	ldw buffer3,x
	ldw x,#$0001
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq onelakh
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 2} ,a
	clr temp1
tenthousand:
	ldw x,#$2710
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq tenthousand
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 3} ,a
	clr temp1
thousand:
	ldw x,#$3e8
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq thousand
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 4} ,a
	;ld a,#'.'
	;ld {buffers + 5} ,a
	clr temp1
hundred:
	ldw x,#$0064
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq hundred
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 5} ,a
	clr temp1
ten:
	ldw x,#$000A
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq ten
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 6} ,a
	clr temp1				
	ld a,#$30			; ascii 0
	add a,result1
	ld {buffers + 7},a
	ld a,#'\n'			; new line
	ld {buffers + 8},a
	ld a,#'\n'			; new line
	ld {buffers + 9},a
	ld a,#'\r'			; carriage return
	ld {buffers + 10},a
	
	clr buffer1			; clear sram registers for bin_to_ascii calculations
	clr buffer2			; clear sram registers for bin_to_ascii calculations
	clr buffer3			; clear sram registers for bin_to_ascii calculations
	clr buffer4			; clear sram registers for bin_to_ascii calculations
	clr buffer5			; clear sram registers for bin_to_ascii calculations
	clr buffer6			; clear sram registers for bin_to_ascii calculations
	clr buffer7			; clear sram registers for bin_to_ascii calculations
	clr buffer8			; clear sram registers for bin_to_ascii calculations
	clr result4			; clear sram registers for bin_to_ascii calculations
	clr result3			; clear sram registers for bin_to_ascii calculations
	clr result2			; clear sram registers for bin_to_ascii calculations
	clr result1			; clear sram registers for bin_to_ascii calculations
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


copy_buffer5678:
	ldw x,buffer7		;copy buffer7,8 to buffer16,17
	ldw buffer16,x
	ldw x,buffer5		;copy buffer5,6 to buffer14,15
	ldw buffer14,x
	ret

move_buffer1to4_buffer14to17:
	ldw x,buffer3		; store lower word of result in buffer16,17
	ldw buffer16,X
	ldw x,buffer1		; store higher word of result in buffer14,15
	ldw buffer14,X
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


UART_TX:
	ld a,data
	ld UART1_DR,a
TC_FLAG:
	btjf UART1_SR,#6 ,TC_FLAG
	ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



string:
	  dc.B " Hello world!" ,'\n','\n','\r',0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



	Interrupt timer4_ISR
timer4_ISR
	bres TIM4_SR,#0		; clear update interrupt flag
	ldw x,counter1
	subw x,#1
	tnzw X
	jreq reload_buffer
	ldw counter1,x
	iret				; return from interrupt
reload_buffer
	ld a,TIM1_CNTRH
	ld captureH,a
	ld a,TIM1_CNTRL
	ld captureL,a
	mov buffers,capture_state
	bres TIM1_CR1,#0	; disable timer1
	ld a,#0
	ld TIM1_CNTRH,a
	ld TIM1_CNTRL,a	
	ld capture_state,a  ;tim1 counter overflow cont register
	ldw x,#500
	ldw counter1,x
	mov state,#1	
	iret
	
	
	interrupt TIM1_ISR
TIM1_ISR
	bres TIM1_SR1,#0 	; clear interrupt flag
	inc capture_state
	iret
	
	interrupt NonHandledInterrupt
NonHandledInterrupt.l
	iret

	segment 'vectit'
	dc.l {$82000000+main}									; reset
	dc.l {$82000000+NonHandledInterrupt}	; trap
	dc.l {$82000000+NonHandledInterrupt}	; irq0
	dc.l {$82000000+NonHandledInterrupt}	; irq1
	dc.l {$82000000+NonHandledInterrupt}	; irq2
	dc.l {$82000000+NonHandledInterrupt}	; irq3
	dc.l {$82000000+NonHandledInterrupt}	; irq4
	dc.l {$82000000+NonHandledInterrupt}	; irq5
	dc.l {$82000000+NonHandledInterrupt}	; irq6
	dc.l {$82000000+NonHandledInterrupt}	; irq7
	dc.l {$82000000+NonHandledInterrupt}	; irq8
	dc.l {$82000000+NonHandledInterrupt}	; irq9
	dc.l {$82000000+NonHandledInterrupt}	; irq10
	dc.l {$82000000+TIM1_ISR}	; irq11{$82000000+NonHandledInterrupt}	; irq11
	dc.l {$82000000+NonHandledInterrupt}	; irq12
	dc.l {$82000000+NonHandledInterrupt}	; irq13
	dc.l {$82000000+NonHandledInterrupt}	; irq14
	dc.l {$82000000+NonHandledInterrupt}	; irq15
	dc.l {$82000000+NonHandledInterrupt}	; irq16
	dc.l {$82000000+NonHandledInterrupt}	; irq17
	dc.l {$82000000+NonHandledInterrupt}	; irq18
	dc.l {$82000000+NonHandledInterrupt}	; irq19
	dc.l {$82000000+NonHandledInterrupt}	; irq20
	dc.l {$82000000+NonHandledInterrupt}	; irq21
	dc.l {$82000000+NonHandledInterrupt}	; irq22
	dc.l {$82000000+timer4_ISR}	; irq23	; irq23
	dc.l {$82000000+NonHandledInterrupt}	; irq24
	dc.l {$82000000+NonHandledInterrupt}	; irq25
	dc.l {$82000000+NonHandledInterrupt}	; irq26
	dc.l {$82000000+NonHandledInterrupt}	; irq27
	dc.l {$82000000+NonHandledInterrupt}	; irq28
	dc.l {$82000000+NonHandledInterrupt}	; irq29

	end
	
	