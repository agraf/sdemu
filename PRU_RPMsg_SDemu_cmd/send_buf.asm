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

	; Bit 0 - use gap to inc ptr
	wait_for_clk_low
	LSR R30.b0, R14, 6
	wait_for_clk_high
	ADD R0, R0, 1

	; Bit 1 - use gap to load next byte
	wait_for_clk_low
	LSR R30.b0, R14, 5
	wait_for_clk_high
	LBBO &R1.b0, R0, 0, 1

	; Bit 2 - use gap to dec len
	wait_for_clk_low
	LSR R30.b0, R14, 4
	wait_for_clk_high
	SUB R15, R15, 1

	; Bit 3 - nop in gap
	wait_for_clk_low
	LSR R30.b0, R14, 3
	wait_for_clk_high
	nop ; delay to ensure we don't get stale clock values

	; Bit 4 - nop in gap
	wait_for_clk_low
	LSR R30.b0, R14, 2
	wait_for_clk_high
	nop ; delay to ensure we don't get stale clock values

	; Bit 5 - nop in gap
	wait_for_clk_low
	LSR R30.b0, R14, 1
	wait_for_clk_high
	nop ; delay to ensure we don't get stale clock values

	; Bit 6 - nop in gap
	wait_for_clk_low
	LSL R30.b0, R14, 0
	wait_for_clk_high
	nop ; delay to ensure we don't get stale clock values

	; Bit 7 - loop in gap
	wait_for_clk_low
	LSL R30.b0, R14, 1
	wait_for_clk_high
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

	; Bit 0 - use gap to inc ptr
	wait_for_clk_high
	LSR R30.b0, R14, 6
	wait_for_clk_low
	ADD R0, R0, 1

	; Bit 1 - use gap to load next byte
	wait_for_clk_high
	LSR R30.b0, R14, 5
	wait_for_clk_low
	LBBO &R1.b0, R0, 0, 1

	; Bit 2 - use gap to dec len
	wait_for_clk_high
	LSR R30.b0, R14, 4
	wait_for_clk_low
	SUB R15, R15, 1

	; Bit 3 - nop in gap
	wait_for_clk_high
	LSR R30.b0, R14, 3
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values

	; Bit 4 - nop in gap
	wait_for_clk_high
	LSR R30.b0, R14, 2
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values

	; Bit 5 - nop in gap
	wait_for_clk_high
	LSR R30.b0, R14, 1
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values

	; Bit 6 - nop in gap
	wait_for_clk_high
	LSL R30.b0, R14, 0
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values

	; Bit 7 - loop in gap
	wait_for_clk_high
	LSL R30.b0, R14, 1
	wait_for_clk_low
	QBNE send_next_byte_25Mhz, R15, 0

	jmp r3.w2
