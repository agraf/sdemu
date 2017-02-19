;
; Copyright (C) 2017 Alexander Graf <agraf@suse.de>
;
; SPDX-License-Identifier:      GPL-2.0
;

;
; void send_buf_cmd(uint8_t *buf, uint32_t len)
;
;   buf: pointer to buffer that should get sent
;   len: number of bytes to send
;

wait_for_clk_low .macro
		wbc r31, 0
	.endm

wait_for_clk_high .macro
		wbs r31, 0
	.endm

; Send one bit in the CMD stream when CLK goes down
send_bit_n .macro BIT_NR
		.newblock
		QBBC $1, R14, BIT_NR
	; Bit is set
		wait_for_clk_low
		ldi r30.b0, (1<<1) | (1<<7) ; Set CMD for v1.0 and v1.1
		jmp $2
$1: ; Bit is clear
		wait_for_clk_low
		ldi r30.b0, 0x0
		jmp $2
$2:
		wait_for_clk_high
	.endm

; Send one bit in the CMD stream when CLK goes up
send_bit_rev_n .macro BIT_NR
		.newblock
		QBBC $1, R14, BIT_NR
	; Bit is set
		wait_for_clk_high
		ldi r30.b0, (1<<1) | (1<<7) ; Set CMD for v1.0 and v1.1
		jmp $2
$1: ; Bit is clear
		wait_for_clk_high
		ldi r30.b0, 0x0
		jmp $2
$2:
		wait_for_clk_low
	.endm
	.global send_buf_cmd
send_buf_cmd:

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
	wait_for_clk_high
	nop
	nop
	wait_for_clk_low
	nop
	nop
	nop
	nop
	qbbs send_buf_25Mhz, r31, 0

	; make sure we align with the clock
	wait_for_clk_high

send_next_byte:
	; current_byte = next_byte
	mov	r14, r1

	; Bit 7 - use gap to inc ptr
	send_bit_n 7
	ADD R0, R0, 1

	; Bit 6 - use gap to load next byte
	send_bit_n 6
	LBBO &R1.b0, R0, 0, 1

	; Bit 5 - use gap to dec len
	send_bit_n 5
	SUB R15, R15, 1

	; Bit 4 - nop in gap
	send_bit_n 4
	nop ; delay to ensure we don't get stale clock values

	; Bit 3 - nop in gap
	send_bit_n 3
	nop ; delay to ensure we don't get stale clock values

	; Bit 2 - nop in gap
	send_bit_n 2
	nop ; delay to ensure we don't get stale clock values

	; Bit 1 - nop in gap
	send_bit_n 1
	nop ; delay to ensure we don't get stale clock values

	; Bit 0 - loop in gap
	send_bit_n 0
	QBNE send_next_byte, R15, 0

	jmp r3.w2

; The PRUs have some internal delay between input and output.
; Measurements show that this delay is somewhere between 20 and
; 40 ns.
;
; When running in 25Mhz mode, this means that by the time we see
; the signal going down, it's already gone up and we should have
; written a proper value to the line.
;
; The workaround set in place here simply set the next bit whenever
; we see CLK going up rather than down. That way (in continuous
; CLK environments) we give ourselves 20ns head start.

send_buf_25Mhz:

	; make sure we align with the clock
	wait_for_clk_high
	nop
	nop
	wait_for_clk_low

send_next_byte_25Mhz:
	; current_byte = next_byte
	mov	r14, r1

	; Bit 7 - use gap to inc ptr
	send_bit_rev_n 7
	ADD R0, R0, 1

	; Bit 6 - use gap to load next byte
	send_bit_rev_n 6
	LBBO &R1.b0, R0, 0, 1

	; Bit 5 - use gap to dec len
	send_bit_rev_n 5
	SUB R15, R15, 1

	; Bit 4 - nop in gap
	send_bit_rev_n 4
	nop ; delay to ensure we don't get stale clock values

	; Bit 3 - nop in gap
	send_bit_rev_n 3
	nop ; delay to ensure we don't get stale clock values

	; Bit 2 - nop in gap
	send_bit_rev_n 2
	nop ; delay to ensure we don't get stale clock values

	; Bit 1 - nop in gap
	send_bit_rev_n 1
	nop ; delay to ensure we don't get stale clock values

	; Bit 0 - loop in gap
	send_bit_rev_n 0
	QBNE send_next_byte_25Mhz, R15, 0

	jmp r3.w2
