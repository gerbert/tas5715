/*
 * ASoC Driver for TAS5715
 *
 * Author:	Andrei Andreyanau <a.andreyanau@sam-solutions.com>
 * Based on ASoC driver for TAS5713 by Sebastian Eickhoff
 *				<basti.eickhoff@googlemail.com>
 * and TAS5086 ASoC codec driver by Daniel Mack
 *				<zonque@gmail.com>
 * This program is free software; you can redistribute it andr/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#ifndef _TAS5715_H
#define _TAS5715_H

// TAS5715 I2C-bus register addresses

#define TAS5715_CLOCK_CTRL				0x00
#define TAS5715_DEVICE_ID				0x01
#define TAS5715_ERROR_STATUS				0x02
#define TAS5715_SYSTEM_CTRL1				0x03
#define TAS5715_SERIAL_DATA_INTERFACE			0x04
#define TAS5715_SYSTEM_CTRL2				0x05
#define TAS5715_SOFT_MUTE				0x06
#define TAS5715_VOL_MASTER				0x07
#define TAS5715_VOL_CH1					0x08
#define TAS5715_VOL_CH2					0x09
#define TAS5715_VOL_CH3					0x0A
#define TAS5715_VOL_CONFIG				0x0E
#define TAS5715_MAX_DUTY_CYCLE				0x0F
#define TAS5715_MODULATION_LIMIT			0x10
#define TAS5715_IC_DLY_CH1				0x11
#define TAS5715_IC_DLY_CH2				0x12
#define TAS5715_IC_DLY_CH3				0x13
#define TAS5715_IC_DLY_CH4				0x14

#define TAS5715_START_STOP_PERIOD			0x1A
#define TAS5715_OSC_TRIM				0x1B
#define TAS5715_BKND_ERR				0x1C
#define TAS5715_INPUT_MUX				0x20
#define TAS5715_PWM_MUX					0x25
#define TAS5715_SOFT_RESET				0xC8
#define TAS5715_MAX_REGISTER				TAS5715_PWM_MUX

// Bitmasks for registers
#define TAS5715_SOFT_MUTE_ALL				0x07

struct tas5715_init_command {
	const int size;
	const char *const data;
};

static const struct tas5715_init_command tas5715_init_sequence[] = {
	// Trim oscillator (See Datasheet, p.41)
	{ .size = 2, .data = "\x1B\x00" },
	// System control register 1 (0x03): block DC
	{ .size = 2, .data = "\x03\x80" },
	// Mute everything
	{ .size = 2, .data = "\x05\x40" },
	// Modulation limit register (0x10): 97.7%
	{ .size = 2, .data = "\x10\x02" },
	// Interchannel delay registers
	// (0x11, 0x12, 0x13 and 0x14): BD mode
	{ .size = 2, .data = "\x11\xB8" },
	{ .size = 2, .data = "\x12\x60" },
	{ .size = 2, .data = "\x13\xA0" },
	{ .size = 2, .data = "\x14\x48" },
	// PWM shutdown group register (0x19): no shutdown
	{ .size = 2, .data = "\x19\x00" },
	// Start/stop period
	{ .size = 2, .data = "\x1A\x48" },
	// Input multiplexer register (0x20): BD mode
	{ .size = 5, .data = "\x20\x00\x89\x77\x72" },
	// PWM output mux register (0x25)
	// Channel 1 --> OUTA, OUTB
	// Channel 2 neg --> OUTC, OUTD
	//{ .size = 5, .data = "\x25\x01\x02\x13\x45" },
	{ .size = 5, .data = "\x25\x01\x00\x33\x45" },
	// DRC control (0x46): DRC1/2 on
	{ .size = 5, .data = "\x46\x00\x00\x00\x03" },
	// BKND_ERR register (0x1C): 299ms reset period
	{ .size = 2, .data = "\x1C\x32" },
	// Volume configuration register (0x0E): volume slew 512 steps
	{ .size = 2, .data = "\x0E\x90" },
	// Bank switch and eq control (0x50): no bank switching
	{ .size = 5, .data = "\x50\x00\x00\x00\x00" },
	// Volume registers (0x07, 0x08, 0x09, 0x0C)
	// 0x07 - Master volume
	// 0x08 - Channel-1 volume
	// 0x08 - Channel-2 volume
	// 0x0C - Headphone volume
	{ .size = 2, .data = "\x07\x20" },
	{ .size = 2, .data = "\x08\x30" },
	{ .size = 2, .data = "\x09\x30" },
	{ .size = 2, .data = "\x0C\xFF" },
	// Channel-1 DRC cross-over mixer (0x70): no mix
	{ .size = 5, .data = "\x70\x00\x80\x00\x00" },
	// Channel-1 input scaler (0x73): no mix
	{ .size = 5, .data = "\x73\x00\x80\x00\x00" },
	// Channel-2 DRC cross-over mixer (0x74): no mix
	{ .size = 5, .data = "\x74\x00\x80\x00\x00" },
	// Channel-2 input scaler (0x77): no mix
	{ .size = 5, .data = "\x77\x00\x80\x00\x00" },
	// Output post-/pre-scale (0x56, 0x57)
	{ .size = 5, .data = "\x56\x00\x80\x00\x00" },
	{ .size = 5, .data = "\x57\x00\x02\x00\x00" },
	// DRC1 softening filter alpha/omega (0x3B)
	{ .size = 9, .data = "\x3B\x00\x08\x00\x00\x00\x78\x00\x00" },
	// DRC1 attack/release rate (0x3C)
	{ .size = 9, .data = "\x3C\x00\x00\x01\x00\xFF\xFF\xFF\x00" },
	// DRC2 softening filter alpha/omega (0x3E)
	{ .size = 9, .data = "\x3E\x00\x08\x00\x00\xFF\xF8\x00\x00" },
	// DRC2 attack/release rate (0x3F)
	{ .size = 9, .data = "\x3F\x00\x08\x00\x00\xFF\xF8\x00\x00" },
	// DRC1 attack/release threshold (0x40)
	{ .size = 9, .data = "\x40\x08\x00\x00\x00\x07\xFF\xFF\xFF" },
	// DRC2 attach/release threshold (0x43)
	{ .size = 9, .data = "\x43\x00\x74\x00\x00\x00\x73\xFF\xFF" },
	// 0x51, 0x52: output mixer
	{ .size = 9, .data = "\x51\x00\x80\x00\x00\x00\x00\x00\x00" },
	{ .size = 9, .data = "\x52\x00\x80\x00\x00\x00\x00\x00\x00" },
	// PEQ defaults
	{ .size = 21, .data = "\x29\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2A\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2B\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2C\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2D\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2E\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2F\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x30\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x31\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x32\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x33\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x34\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x35\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x36\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x59\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x5D\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
};

#endif /* _TAS5715_H */
