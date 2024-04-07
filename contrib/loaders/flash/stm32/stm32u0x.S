/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2015 Uwe Bonnes                                         *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.                                        *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb

/*
 * Params :
 * r0 = workarea start, status (out)
 * r1 = workarea end
 * r2 = target address
 * r3 = count (64bit words)
 * r4 = flash base
 *
 * Clobbered:
 * r5   - rp
 * r6/7 - temp (64-bit)
 * r8   - wp, tmp
 */

#define STM32_FLASH_CR_OFFSET	0x14	/* offset of CR register in FLASH struct */
#define STM32_FLASH_SR_OFFSET	0x10	/* offset of SR register in FLASH struct */

#define STM32_PROG	 0x1	/* PG */

	.thumb_func
	.global _start
_start:
wait_fifo:
	ldr 	r7, [r0, #0]	/* read wp */
	cmp 	r7, #0		/* abort if wp == 0 */
	beq 	exit
	ldr 	r5, [r0, #4]	/* read rp */
	subs	r6, r7, r5	/* number of bytes available for read in r6*/
	bge	go_on
	add	r6, r1          /* add size of buffer */
	subs	r6, r0
go_on:
	cmp 	r6, #8		/* wait until 8 bytes are available */
	bcc 	wait_fifo

	ldr	r6, =STM32_PROG
	str	r6, [r4, #STM32_FLASH_CR_OFFSET]
	ldr	r6, [r5]
	adds	r5,#4
	ldr	r7, [r5]
	adds	r5,#4
	str 	r6, [r2]	/* write one word to dst, increment ptr */
	adds	r2,#4
	str 	r7, [r2]	/* write one word to dst, increment ptr */
	adds	r2,#4
	dsb
busy:
	ldr 	r6, [r4, #STM32_FLASH_SR_OFFSET]
	mov	r7,r6
	lsls	r7,#15
	bmi	busy
	movs	r7,#0xfa
	ands	r6,r7
	cmp	r6, #0
	bne	error

	cmp 	r5, r1		/* wrap rp at end of buffer */
	bcc	go_on2
	mov	r5,r0
	adds	r5,#8
go_on2:
	str 	r5, [r0, #4]	/* store rp */
	subs	r3, r3, #1	/* decrement dword count */
	cmp	r3, #0
	beq	exit
	b	wait_fifo
error:
	movs	r1, #0
	str	r1, [r0, #4]	/* set rp = 0 on error */
exit:
	mov	r0, r6		/* return status in r0 */
	bkpt	#0x00

	.pool
