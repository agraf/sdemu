;
; Copyright (C) 2017 Alexander Graf <agraf@suse.de>
;
; SPDX-License-Identifier:      GPL-2.0
;

;
; void read_buf_cmd(uint8_t *buf)
;
;   buf: pointer to 6-byte buffer that should get read
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


analyze_n_bits .macro target_reg, nr_bits
		.newblock
		ldi target_reg, 0
		ldi r15, nr_bits
		loop $1, nr_bits

			sub r15, r15, 1
			MVIB r0.b0, *r1.b0++
			qbbc $1, r0.b0, 1
			set target_reg, target_reg, r15	; target_reg |= (1<<counter)
$1:
	.endm

; We can't put bits into their target spots quickly enough. So instead we
; save the full 8bit capture stream on every rising edge.
read_n_bytes .macro target_reg, nr_bytes
		.newblock
		loop $1, nr_bytes

			wait_for_clk_high
			MVIB *target_reg++, r31.b0
			wait_for_clk_low

$1:
	.endm


	.global read_buf_cmd
read_buf_cmd:

	; Args:
	;
	; R14	ptr (6 bytes)

	ldi r1.b0, &r17
	; R1.b0		buffer pointer

	; wait for the cmd to start
cmd_not_started_yet_raw:
	wait_for_clk_low
	wait_for_clk_high
	QBBS cmd_not_started_yet_raw, R31, 1	; Loop until CLK=1 CMD=0
	wait_for_clk_high						; Make sure we really are in the right bit
	wait_for_clk_low						; Then wait for it to finish
	; At this point we have read the start bit (0)

	read_n_bytes r1.b0, 47

	; We're done reading the bits, post-process them into readable form
	ldi r1.b0, &r17

	; Read cmd (7 bits left to go)
	analyze_n_bits r16, 7

	; Read arg (32 bits left to go)
	analyze_n_bits r17, 32

	; Read crc7 (8 bits left to go)
	analyze_n_bits r18, 8

    ; final memory write
	SBBO &r18.b0, r14, 0, 1; store crc7
	SBBO &r17,    r14, 1, 4; store arg
	SBBO &r16.b0, r14, 5, 1; store cmd

	jmp r3.w2
