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

// Keyboard driver for console. This may need changing for UNIX/non-UNIX platforms
#ifdef _WIN32
void keyboard_driver(struct x86_state *s) {
	kbhit() && (s->mem[0x4A6] = getch(), pc_interrupt(7));
}
#else
void keyboard_driver(struct x86_state *s) {
	if(read(0, s->mem + 0x4A6, 1)) { 
		/* if esc was pressed, check shortly afterwards to see if it should be enqueued separately or if it's
		 * part of an escape sequence. */
		s->int8_asap = (s->mem[0x4A6] == 0x1B);
		pc_interrupt(s, 7);
	}
}
#endif


#ifdef NO_GRAPHICS
// Keyboard driver for SDL
void sdl_keyboard_driver(struct x86_state *s) {
	keyboard_driver(s);
}

void sdl_pause_audio(int pause) { }

#else

SDL_Surface *sdl_screen;
unsigned short vid_addr_lookup[VIDEO_RAM_SIZE], cga_colors[4] = {0 /* Black */, 0x1F1F /* Cyan */, 0xE3E3 /* Magenta */, 0xFFFF /* White */};
SDL_AudioSpec sdl_audio = {44100, AUDIO_U8, 1, 0, 128};

void sdl_keyboard_driver(struct x86_state *s) {
	unsigned int scratch_uint, scratch2_uint;
	SDL_Event sdl_event;

	if(sdl_screen && SDL_PollEvent(&sdl_event) && (sdl_event.type == SDL_KEYDOWN || sdl_event.type == SDL_KEYUP)) {
		unsigned int keysym_unicode = sdl_event.key.keysym.unicode;
		unsigned int keysym_mod = sdl_event.key.keysym.mod;

		*(short *)&s->mem[0x4A6] = 0x400
			+ ((!keysym_unicode || keysym_unicode > 0x7F) ? sdl_event.key.keysym.sym : keysym_unicode)
			+ 0x800*!!(keysym_mod & KMOD_ALT)
			+ 0x1000*!!(keysym_mod & KMOD_SHIFT)
			+ 0x2000*!!(keysym_mod & KMOD_CTRL)
			+ 0x4000*(sdl_event.type == SDL_KEYUP);

		pc_interrupt(s, 7);
	} else {
		keyboard_driver(s);
	}
}

void sdl_redraw_display(struct x86_state *s) {
	s->graphics_inst_counter = 0;
	s->hlt_this_time = 0;
	// Video card in graphics mode?
	if (s->io_ports[0x3B8] & 2)
	{
		// If we don't already have an SDL window open, set it up and compute color and video memory translation tables
		if (!sdl_screen)
		{
			for (int i = 0; i < 16; i++)
				s->pixel_colors[i] = s->mem[0x4AC] ? // CGA?
					cga_colors[(i & 12) >> 2] + (cga_colors[i & 3] << 16) // CGA -> RGB332
					: 0xFF*(((i & 1) << 24) + ((i & 2) << 15) + ((i & 4) << 6) + ((i & 8) >> 3)); // Hercules -> RGB332

			for (int i = 0; i < s->GRAPHICS_X * s->GRAPHICS_Y / 4; i++)
				vid_addr_lookup[i] = i / s->GRAPHICS_X * (s->GRAPHICS_X / 8) + (i / 2) % (s->GRAPHICS_X / 8) + 0x2000*(s->mem[0x4AC] ? (2 * i / s->GRAPHICS_X) % 2 : (4 * i / s->GRAPHICS_X) % 4);

			SDL_Init(SDL_INIT_VIDEO);
			sdl_screen = SDL_SetVideoMode(s->GRAPHICS_X, s->GRAPHICS_Y, 8, 0);
			SDL_EnableUNICODE(1);
			SDL_EnableKeyRepeat(500, 30);
		}

		// Refresh SDL display from emulated graphics card video RAM
		s->vid_mem_base = s->mem + 0xB0000 + 0x8000*(s->mem[0x4AC] ? 1 : s->io_ports[0x3B8] >> 7); // B800:0 for CGA/Hercules bank 2, B000:0 for Hercules bank 1
		for (int i = 0; i < s->GRAPHICS_X * s->GRAPHICS_Y / 4; i++)
			((unsigned *)sdl_screen->pixels)[i] = s->pixel_colors[15 & (s->vid_mem_base[vid_addr_lookup[i]] >> 4*!(i & 1))];

		SDL_Flip(sdl_screen);
	}
	else if (sdl_screen) // Application has gone back to text mode, so close the SDL window
	{
		SDL_QuitSubSystem(SDL_INIT_VIDEO);
		sdl_screen = 0;
	}
	SDL_PumpEvents();
}

void audio_callback(void *data, unsigned char *stream, int len)
{
	struct x86_state *s = data;
	for (int i = 0; i < len; i++)
		stream[i] = (s->spkr_en == 3) && *(unsigned short*)&s->mem[0x4AA] ? -((54 * s->wave_counter++ / *(unsigned short*)&s->mem[0x4AA]) & 1) : sdl_audio.silence;

	s->spkr_en = s->io_ports[0x61] & 3;
}

void sdl_pause_audio(int pause) {
	SDL_PauseAudio(pause);
}
#endif

// Emulator entry point
int main(int argc, char **argv)
{
#ifndef NO_GRAPHICS
	// Initialise SDL
	SDL_Init(SDL_INIT_AUDIO);
	sdl_audio.callback = audio_callback;
#ifdef _WIN32
	sdl_audio.samples = 512;
#endif
#endif

	// If the HD image file is prefixed with @, then boot from the HD
	int boot_from_hdd = 0;
	if ((argc > 3) && (*argv[3] == '@')) {
		boot_from_hdd = 1;
		argv[3]++;
	}

	struct x86_state *s = x86_init(boot_from_hdd, argv[1], argv[2], argv[3], sdl_redraw_display, sdl_keyboard_driver, sdl_pause_audio);
#ifndef NO_GRAPHICS
	sdl_audio.userdata = s;
	SDL_OpenAudio(&sdl_audio, 0);
#endif

	// Instruction execution loop. Terminates if CS:IP = 0:0
	for (; s->opcode_stream = s->mem + 16 * s->regs16[REG_CS] + s->reg_ip, s->opcode_stream != s->mem;)
	{
		x86_step(s);
		x86_handle_hlt(s);
		x86_handle_irqs(s);
	}

#ifndef NO_GRAPHICS
	SDL_Quit();
#endif
	return 0;
}
