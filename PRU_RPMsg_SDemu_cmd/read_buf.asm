;
; Copyright (C) 2017 Alexander Graf <agraf@suse.de>
;
; SPDX-License-Identifier:      GPL-2.0
;

;
; void read_buf_cmd(uint8_t *buf, uint32_t len)
;
;   buf: pointer to buffer that should get read
;   len: number of bytes to read starting with a 0 bit
;
; Beware that this function will read the buffer in reverse,
; so that we can easily cast to Little Endian variables.
;

wait_for_clk_low .macro
		wbc r31, 0
	.endm

wait_for_clk_high .macro
		wbs r31, 0
	.endm

wait_for_cmd_low .macro
		wbc r31, 1
	.endm

wait_for_cmd_high .macro
		wbs r31, 1
	.endm

	.global read_buf_cmd
read_buf_cmd:

	; Args:
	;
	; R14	ptr
	; R15	len

	; Copy ptr+len to r0. We need to start at +1 because we
	; always decrement ptr per byte before store
	ADD	R0, R14, R15

	; Ensure the to-be-read byte is 0 initialized
	LDI r1, 0 ; current_byte = 0

	; R0	ptr (increasing)
	; R1	current byte (read in progress)
	; R15	len (decreasing)
	; R16	scratch register

	; wait for the cmd to start
cmd_not_started_yet:
	and r16, r31, 3
	QBNE cmd_not_started_yet, R16, 1	; Loop until CLK=1 CMD=0
	wait_for_clk_high
	jmp read_bit7_clear

read_next_byte:
	; Bit 7 - use gap to store byte, clear byte on clk=hi
	wait_for_clk_low
	SBBO &r1.b0, r0, 0, 1; store byte, skip this for the first round (jumps below)
	wait_for_clk_high
	LDI r1, 0 ; current_byte = 0
	qbbc read_bit7_clear, r31, 1
	SET r1.b0, r1.b0, 7
read_bit7_clear:

	; Bit 6 - use gap to decrement ptr
	wait_for_clk_low
	SUB R0, R0, 1
	wait_for_clk_high
	qbbc read_bit6_clear, r31, 1
	set r1.b0, r1.b0, 6
read_bit6_clear:

	; Bit 5 - use gap to decrement counter
	wait_for_clk_low
	SUB R15, R15, 1
	wait_for_clk_high
	qbbc read_bit5_clear, r31, 1
	set r1.b0, r1.b0, 5
read_bit5_clear:

	; Bit 4
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit4_clear, r31, 1
	set r1.b0, r1.b0, 4
read_bit4_clear:

	; Bit 3
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit3_clear, r31, 1
	set r1.b0, r1.b0, 3
read_bit3_clear:

	; Bit 2
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit2_clear, r31, 1
	set r1.b0, r1.b0, 2
read_bit2_clear:

	; Bit 1
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit1_clear, r31, 1
	set r1.b0, r1.b0, 1
read_bit1_clear:

	; Bit 0
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit0_clear, r31, 1
	set r1.b0, r1.b0, 0
read_bit0_clear:

	; jump to the next byte if we have more to read
	QBNE read_next_byte, R15, 0

    ; final byte write
	SBBO &r1.b0, r0, 0, 1; store final byte

	jmp r3.w2
