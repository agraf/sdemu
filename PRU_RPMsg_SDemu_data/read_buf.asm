;
; Copyright (C) 2017 Alexander Graf <agraf@suse.de>
;
; SPDX-License-Identifier:      GPL-2.0
;

;
; void read_buf_1bit_startend(uint8_t *buf, uint32_t len)
;
;   buf: pointer to buffer the data gets retreived into
;   len: number of bytes to read
;

wait_for_clk_low .macro
		wbc r31, 16
	.endm

wait_for_clk_high .macro
		wbs r31, 16
	.endm

	.global read_buf_1bit_startend
read_buf_1bit_startend:

	; Args:
	;
	; R14	ptr
	; R15	len

	; We always increment r14 in the first round, account for that
	SUB r14, r14, 1

	; R15 = ptr + len - 1 (end of stream)
	ADD r15, r14, r15

	; current_byte = 0
	LDI r1, 0

	; R1	current byte
	; R14	ptr
	; R15	end of stream (==ptr -> out)
	; R16	scratch register

	; wait for the cmd to start
	ldi r0.w0, 0x0100	; DAT0
	ldi r0.w1, 0x0001	; CLK
	ldi r1.w1, 0x0001	; CLK
dat_not_started_yet_1bit:
	and r16, r31, r0	; (DAT0 | CLK)
	QBNE dat_not_started_yet_1bit, R16, r1	; Loop until CLK=1 DAT0=0
	wait_for_clk_low
	jmp read_started_1bit

read_next_byte_1bit:
	; Bit 7 - use gap to store byte, clear byte on clk=hi
	wait_for_clk_low
	SBBO &r1.b0, r14, 0, 1; store byte, skip this for the first round (jumps below)
read_started_1bit:
	wait_for_clk_high
	LDI r1, 0 ; current_byte = 0
	qbbc read_bit7_clear, r31, 8
	SET r1.b0, r1.b0, 7
read_bit7_clear:

	; Bit 6 - use gap to increment ptr
	wait_for_clk_low
	ADD R14, R14, 1
	wait_for_clk_high
	qbbc read_bit6_clear, r31, 8
	set r1.b0, r1.b0, 6
read_bit6_clear:

	; Bit 5
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit5_clear, r31, 8
	set r1.b0, r1.b0, 5
read_bit5_clear:

	; Bit 4
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit4_clear, r31, 8
	set r1.b0, r1.b0, 4
read_bit4_clear:

	; Bit 3
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit3_clear, r31, 8
	set r1.b0, r1.b0, 3
read_bit3_clear:

	; Bit 2
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit2_clear, r31, 8
	set r1.b0, r1.b0, 2
read_bit2_clear:

	; Bit 1
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit1_clear, r31, 8
	set r1.b0, r1.b0, 1
read_bit1_clear:

	; Bit 0
	wait_for_clk_low
	nop ; delay to ensure we don't get stale clock values
	wait_for_clk_high
	qbbc read_bit0_clear, r31, 8
	set r1.b0, r1.b0, 0
read_bit0_clear:

	; jump to the next byte if we have more to read
	QBNE read_next_byte_1bit, R14, R15

    ; final byte write
	SBBO &r1.b0, r14, 0, 1; store final byte

	jmp r3.w2








;
; void read_buf_4bit_startend(uint8_t *buf, uint32_t len)
;
;   buf: pointer to buffer the data gets retreived into
;   len: number of bytes to read
;

	.global read_buf_4bit_startend
read_buf_4bit_startend:

	; Args:
	;
	; R14	ptr
	; R15	len

	; We always increment r14 in the first round, account for that
	SUB r14, r14, 1

	; R15 = ptr + len - 1 (end of stream)
	ADD r15, r14, r15

	; current_byte = 0
	LDI r1, 0

	; R1	current byte
	; R14	ptr
	; R15	end of stream (==ptr -> out)
	; R16	scratch register

	; wait for the cmd to start
	ldi r0.w0, 0x0f00	; DAT0 | DAT1 | DAT2 | DAT3
	ldi r0.w1, 0x0001	; CLK
	ldi r1.w1, 0x0001	; CLK
dat_not_started_yet_4bit:
	and r16, r31, r0	; (DAT0 | DAT1 | DAT2 | DAT3 | CLK)
	QBNE dat_not_started_yet_1bit, R16, r1	; Loop until CLK=1 DAT0=0 DAT1=0 DAT2=0 DAT3=0
	wait_for_clk_low
	jmp read_started_4bit

read_next_byte_4bit:
	; Bits [4..7] - use gap to store byte
	wait_for_clk_low
	SBBO &r1.b0, r14, 0, 1	; store byte, skip this for the first round (jumps below)
read_started_4bit:
	wait_for_clk_high
	LSL r1.b0, r31.b1, 4	; read upper 4 bits

	; Bits [0..3] - use gap to increment ptr
	wait_for_clk_low
	ADD R14, R14, 1
	wait_for_clk_high
	AND r1.b1, r31.b1, 0x0f
	OR r1.b0, r1.b1, r1.b0	; OR lower 4 bits into byte

	; jump to the next byte if we have more to read
	QBNE read_next_byte_4bit, R14, R15

    ; final byte write
	SBBO &r1.b0, r14, 0, 1; store final byte

	jmp r3.w2
