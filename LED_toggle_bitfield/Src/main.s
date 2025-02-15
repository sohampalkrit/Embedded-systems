	.cpu cortex-m4
	.arch armv7e-m
	.fpu softvfp
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 6
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.file	"main.c"
	.text
	.align	1
	.global	main
	.syntax unified
	.thumb
	.thumb_func
	.type	main, %function
main:
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 1, uses_anonymous_args = 0
	@ link register save eliminated.
	push	{r7}
	sub	sp, sp, #28
	add	r7, sp, #0
	ldr	r3, .L7
	str	r3, [r7, #12]
	ldr	r3, .L7+4
	str	r3, [r7, #8]
	ldr	r3, .L7+8
	str	r3, [r7, #4]
	ldr	r2, [r7, #12]
	ldr	r3, [r2]
	orr	r3, r3, #8
	str	r3, [r2]
	ldr	r2, [r7, #8]
	ldr	r3, [r2]
	movs	r1, #1
	bfi	r3, r1, #24, #2
	str	r3, [r2]
.L6:
	movs	r3, #0
	str	r3, [r7, #20]
	b	.L2
.L3:
	ldr	r3, [r7, #20]
	adds	r3, r3, #1
	str	r3, [r7, #20]
.L2:
	ldr	r3, [r7, #20]
	ldr	r2, .L7+12
	cmp	r3, r2
	bls	.L3
	ldr	r2, [r7, #4]
	ldr	r3, [r2]
	orr	r3, r3, #4096
	str	r3, [r2]
	movs	r3, #0
	str	r3, [r7, #16]
	b	.L4
.L5:
	ldr	r3, [r7, #16]
	adds	r3, r3, #1
	str	r3, [r7, #16]
.L4:
	ldr	r3, [r7, #16]
	ldr	r2, .L7+12
	cmp	r3, r2
	bls	.L5
	ldr	r2, [r7, #4]
	ldr	r3, [r2]
	bfc	r3, #12, #1
	str	r3, [r2]
	b	.L6
.L8:
	.align	2
.L7:
	.word	1073887280
	.word	1073875968
	.word	1073875988
	.word	999999
	.size	main, .-main
	.ident	"GCC: (GNU Tools for STM32 12.3.rel1.20240306-1730) 12.3.1 20230626"
