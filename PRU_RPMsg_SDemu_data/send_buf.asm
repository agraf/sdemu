;
; Copyright (C) 2017 Alexander Graf <agraf@suse.de>
;
; SPDX-License-Identifier:      GPL-2.0
;

;
; void send_buf_1bit_startend(uint8_t *buf, uint32_t len)
;
;   buf: pointer to buffer that should get sent
;   len: number of bytes to send
;

wait_for_clk_low .macro
		wbc r31, 16
	.endm

wait_for_clk_high .macro
		wbs r31, 16
	.endm

	.global send_buf_1bit_startend
send_buf_1bit_startend:

	; Args:
	;
	; R14	ptr
	; R15	len

	; Copy ptr to r0
	MOV	R0, R14

	; next_byte = *ptr
	LDI r1, 0
	LBBO &R1.b0, R0, 0, 1

	; R0	ptr (increasing)
	; R1	next byte
	; R14	current byte
	; R15	len (decreasing)

	; Check if we're in 25Mhz mode
	wait_for_clk_low
	nop
	nop
	nop
	nop
	qbbs send_buf_1bit_startend_25Mhz, r31, 16

	; Start bit
	wait_for_clk_low
	ldi r30.w0, 0
	wait_for_clk_high

send_next_byte:
	; current_byte = next_byte
	mov	r14, r1

	; Bit 7 - use gap to inc ptr
	wait_for_clk_low
	LSL R30.w0, R14, 1
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	ADD R0, R0, 1
	nop ; delay to ensure we don't get stale clock values

	; Bit 6 - use gap to load next byte
	wait_for_clk_low
	LSL R30.w0, R14, 2
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	LBBO &R1.b0, R0, 0, 1

	; Bit 5 - use gap to dec len
	wait_for_clk_low
	LSL R30.w0, R14, 3
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	SUB R15, R15, 1
	nop ; delay to ensure we don't get stale clock values

	; Bit 4 - nop in gap
	wait_for_clk_low
	LSL R30.w0, R14, 4
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	nop ; delay to ensure we don't get stale clock values
	nop ; delay to ensure we don't get stale clock values

	; Bit 3 - nop in gap
	wait_for_clk_low
	LSL R30.w0, R14, 5
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	nop ; delay to ensure we don't get stale clock values
	nop ; delay to ensure we don't get stale clock values

	; Bit 2 - nop in gap
	wait_for_clk_low
	LSL R30.w0, R14, 6
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	nop ; delay to ensure we don't get stale clock values
	nop ; delay to ensure we don't get stale clock values

	; Bit 1 - nop in gap
	wait_for_clk_low
	LSL R30.w0, R14, 7
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	nop ; delay to ensure we don't get stale clock values
	nop ; delay to ensure we don't get stale clock values

	; Bit 0 - loop in gap
	wait_for_clk_low
	LSL R30.w0, R14, 8
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	QBNE send_next_byte, R15, 0

	; End bit
	wait_for_clk_low
	LDI R30.w0, 256
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high

	jmp r3.w2




; When running in high frequency mode (25Mhz), switching the
; pins is not fast enough. We have to instead rely on our own
; clock (200Mhz) and add delay instructions to align our code
; flow with the incoming clock
;
; We know that setting a bit in R30 takes 20ns (4 instructions)
; to arrive on the wire. So we need to fire a new bit when CLK
; goes high even though the host controller reads the bit when
; CLK goes from low->high

send_buf_1bit_startend_25Mhz:

	; Align ourselves with the clock
	wait_for_clk_low
	nop
	nop
	wait_for_clk_high

	; We're now at the point where we need to submit the start
	; bit. We have 8 cycles (5ns * 8 = 40ns) for each bit.

	; Start Bit
	ldi r30.w0, 0							; 1 cycle (7 left)

	; Delay until it's time for the next bit
	nop 									; 1 cycle (6 left)
	nop 									; 1 cycle (5 left)
	nop 									; 1 cycle (4 left)
	nop 									; 1 cycle (3 left)
	nop 									; 1 cycle (2 left)
	nop 									; 1 cycle (1 left)
	nop 									; 1 cycle (0 left)

send_next_byte_startend_25Mhz:

	; Bit 0 - using next_byte rather than current_byte
	LSL R30.w0, R1, 1							; 1 cycle (7 left)

	; current_byte = next_byte
	mov	r14, r1								; 1 cycle (6 left)

	; Increase pointer
	ADD R0, R0, 1							; 1 cycle (5 left)

	; Load next_byte
	LBBO &R1.b0, R0, 0, 1					; 3 cycles (2 left)

	; Decrease len
	SUB R15, R15, 1							; 1 cycle (1 left)

	; Delay until it's time for the next bit
	nop 									; 1 cycle (0 left)

	; Bit 1
	LSL R30.w0, R14, 2							; 1 cycle (7 left)

	; Delay until it's time for the next bit
	nop 									; 1 cycle (6 left)
	nop 									; 1 cycle (5 left)
	nop 									; 1 cycle (4 left)
	nop 									; 1 cycle (3 left)
	nop 									; 1 cycle (2 left)
	nop 									; 1 cycle (1 left)
	nop 									; 1 cycle (0 left)

	; Bit 2
	LSL R30.w0, R14, 3							; 1 cycle (7 left)

	; Delay until it's time for the next bit
	nop 									; 1 cycle (6 left)
	nop 									; 1 cycle (5 left)
	nop 									; 1 cycle (4 left)
	nop 									; 1 cycle (3 left)
	nop 									; 1 cycle (2 left)
	nop 									; 1 cycle (1 left)
	nop 									; 1 cycle (0 left)

	; Bit 3
	LSL R30.w0, R14, 4							; 1 cycle (7 left)

	; Delay until it's time for the next bit
	nop 									; 1 cycle (6 left)
	nop 									; 1 cycle (5 left)
	nop 									; 1 cycle (4 left)
	nop 									; 1 cycle (3 left)
	nop 									; 1 cycle (2 left)
	nop 									; 1 cycle (1 left)
	nop 									; 1 cycle (0 left)

	; Bit 4
	LSL R30.w0, R14, 5							; 1 cycle (7 left)

	; Delay until it's time for the next bit
	nop 									; 1 cycle (6 left)
	nop 									; 1 cycle (5 left)
	nop 									; 1 cycle (4 left)
	nop 									; 1 cycle (3 left)
	nop 									; 1 cycle (2 left)
	nop 									; 1 cycle (1 left)
	nop 									; 1 cycle (0 left)

	; Bit 5
	LSL R30.w0, R14, 6							; 1 cycle (7 left)

	; Delay until it's time for the next bit
	nop 									; 1 cycle (6 left)
	nop 									; 1 cycle (5 left)
	nop 									; 1 cycle (4 left)
	nop 									; 1 cycle (3 left)
	nop 									; 1 cycle (2 left)
	nop 									; 1 cycle (1 left)
	nop 									; 1 cycle (0 left)

	; Bit 6
	LSL R30.w0, R14, 7							; 1 cycle (7 left)

	; Delay until it's time for the next bit
	nop 									; 1 cycle (6 left)
	nop 									; 1 cycle (5 left)
	nop 									; 1 cycle (4 left)
	nop 									; 1 cycle (3 left)
	nop 									; 1 cycle (2 left)
	nop 									; 1 cycle (1 left)
	nop 									; 1 cycle (0 left)

	; Bit 7
	LSL R30.w0, R14, 8							; 1 cycle (7 left)

	; Delay until it's time for the next bit
	nop 									; 1 cycle (6 left)
	nop 									; 1 cycle (5 left)
	nop 									; 1 cycle (4 left)
	nop 									; 1 cycle (3 left)
	nop 									; 1 cycle (2 left)
	nop 									; 1 cycle (1 left)
	QBNE send_next_byte_startend_25Mhz, R15, 0	; 1 cycle (0 left)

	; End Bit
	LDI R30.w0, 256

	; Delay until it's time for the next bit
	nop 									; 1 cycle (6 left)
	nop 									; 1 cycle (5 left)
	nop 									; 1 cycle (4 left)
	nop 									; 1 cycle (3 left)
	nop 									; 1 cycle (2 left)
	nop 									; 1 cycle (1 left)
	nop 									; 1 cycle (0 left)

	jmp r3.w2
