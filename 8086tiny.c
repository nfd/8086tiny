// 8086tiny: a tiny, highly functional, highly portable PC emulator/VM
// Copyright 2013-14, Adrian Cable (adrian.cable@gmail.com) - http://www.megalith.co.uk/8086tiny
//
// Revision 1.25
//
// This work is licensed under the MIT License. See included LICENSE.TXT.

#define __USE_POSIX199309
#define _POSIX_C_SOURCE 199309L

#include <time.h>
#include <sys/timeb.h>
#include <memory.h>
#include <stdint.h>
#include <stdlib.h>

#include "x86.h"

// #define XMS_DEBUG 1
// #define XMS_FAIL_FIRST_ALLOC 1
/* FreeCOM with XMS Swap seems to break when it loads
 * into our XMS. Various errors occur with different versions.
 * As a workaround, this fails FreeCOM's XMS allocation.
 * After that, for example, lDebug symbolic can use XMS.
 *
 * Update: I found the FreeCOM bug and reported it at
 * https://github.com/FDOS/freecom/issues/15
 *
 * With the support for the push imm16 instruction,
 * we can work around the FreeCOM bug.
 */

#ifndef GRAPHICS_UPDATE_DELAY
#define GRAPHICS_UPDATE_DELAY 360000
#endif

#ifdef XMS_DEBUG
#include <stdio.h>
#endif
#ifdef INT6_DEBUG
#include <stdio.h>
#endif
#ifdef IMUL_DEBUG
#include <stdio.h>
#endif

#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#endif

#ifndef NO_GRAPHICS
#include <SDL/SDL.h>
#endif

extern unsigned char mem[RAM_SIZE + 16];
extern unsigned char *opcode_stream;
extern unsigned short *regs16;
extern unsigned short reg_ip;

// Emulator entry point
int main(int argc, char **argv)
{
	// If the HD image file is prefixed with @, then boot from the HD
	int boot_from_hdd = 0;
	if ((argc > 3) && (*argv[3] == '@')) {
		boot_from_hdd = 1;
		argv[3]++;
	}

	struct x86_state *s = x86_init(boot_from_hdd, argv[1], argv[2], argv[3], NULL, NULL);

	// Instruction execution loop. Terminates if CS:IP = 0:0
	for (; opcode_stream = mem + 16 * regs16[REG_CS] + reg_ip, opcode_stream != s->mem;)
	{
		x86_step(s);
	}

#ifndef NO_GRAPHICS
	SDL_Quit();
#endif
	return 0;
}
