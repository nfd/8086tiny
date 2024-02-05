// 8086tiny: a tiny, highly functional, highly portable PC emulator/VM
// Copyright 2013-14, Adrian Cable (adrian.cable@gmail.com) - http://www.megalith.co.uk/8086tiny
//
// Revision 1.25
//
// This work is licensed under the MIT License. See included LICENSE.TXT.

#define __USE_POSIX199309
#define _POSIX_C_SOURCE 199309L

#define USE_SDL_FOR_TEXT 1

#define MIN_MS_PER_FRAME (1000 / 60)

#include <time.h>
#include <sys/timeb.h>
#include <memory.h>
#include <stdint.h>
#include <stdlib.h>

#include "x86.h"
#include "hercplus.h"

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
	kbhit() && (s->mem[0x4A6] = getch(), pc_interrupt(s, 7));
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

void sdl_redraw_display(struct x86_state *s) { }

#else

SDL_Surface *sdl_screen;

unsigned short vid_addr_lookup[VIDEO_RAM_SIZE], cga_colors[4] = {0 /* Black */, 0x1F1F /* Cyan */, 0xE3E3 /* Magenta */, 0xFFFF /* White */};
unsigned char text_colours[16] = {0b00000000, 0b00000010, 0b00010000, 0b00010010, 0b10000000, 0b10000010, 0b10001000, 0b10010010, 0b00100101, 0b00000011, 0b00011100, 0b00011111, 0b11100000, 0b11100011, 0b11111100, 0b11111111};
SDL_AudioSpec sdl_audio = {44100, AUDIO_U8, 1, 0, 128};
uint32_t last_redraw_time_ms = 0;

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

void sdl_create_screen(struct x86_state *s) {
	// If we don't already have an SDL window open, set it up and compute color and video memory translation tables
	if (!sdl_screen) {
		if (!(s->io_ports[0x3B8] & 2)) {
			/* In text mode, the HGA has a resolution of 720x350, which can't actually be expressed as GRAPHICS_X
			 * and GRAPHICS_Y, so we override that here. */
			s->GRAPHICS_X = 720;
			s->GRAPHICS_Y = 350;
			for(int i = 0; i < 16; i++) {
				s->pixel_colors[i] = text_colours[i];
			}
		} else {
			for (int i = 0; i < 16; i++)
				s->pixel_colors[i] = s->mem[0x4AC] ? // CGA?
					cga_colors[(i & 12) >> 2] + (cga_colors[i & 3] << 16) // CGA -> RGB332
					: 0xFF*(((i & 1) << 24) + ((i & 2) << 15) + ((i & 4) << 6) + ((i & 8) >> 3)); // Hercules -> RGB332
		}

		for (int i = 0; i < s->GRAPHICS_X * s->GRAPHICS_Y / 4; i++)
			vid_addr_lookup[i] = i / s->GRAPHICS_X * (s->GRAPHICS_X / 8) + (i / 2) % (s->GRAPHICS_X / 8) + 0x2000*(s->mem[0x4AC] ? (2 * i / s->GRAPHICS_X) % 2 : (4 * i / s->GRAPHICS_X) % 4);

		SDL_Init(SDL_INIT_VIDEO);
		sdl_screen = SDL_SetVideoMode(s->GRAPHICS_X, s->GRAPHICS_Y, 8, 0);
		SDL_EnableUNICODE(1);
		SDL_EnableKeyRepeat(500, 30);
	}
}

void sdl_redraw_graphics(struct x86_state *s) {
	sdl_create_screen(s);

	// Refresh SDL display from emulated graphics card video RAM
	unsigned char *vid_mem_base = s->mem + 0xB0000 + 0x8000*(s->mem[0x4AC] ? 1 : s->io_ports[0x3B8] >> 7); // B800:0 for CGA/Hercules bank 2, B000:0 for Hercules bank 1
	for (int i = 0; i < s->GRAPHICS_X * s->GRAPHICS_Y / 4; i++)
		((unsigned *)sdl_screen->pixels)[i] = s->pixel_colors[15 & (vid_mem_base[vid_addr_lookup[i]] >> 4*!(i & 1))];

	SDL_Flip(sdl_screen);
}

void sdl_redraw_text(struct x86_state *s) {
	/* The way text output works in this emulator is a bit nuts.
	 * - characters written using int 10 are directly output to stdout using ANSI codes, but are also stored at c000
	 *   so that the 'get character at position' function works.
	 * - every time a timer tick happens, memory at b800 is scanned. If it's nonempty (indicating that a program is
	 *   bypassing int 10 to write characters to the screen); code to generate ANSI is written to stdout (this is the
	 *   vmem driver in the BIOS)
	 * - So as not continuously to redraw the entire screen in the above case, the text is double buffered
	 *   at c800 and only changed characters are written to stdout.
	 * 
	 * This means that we never have a complete buffer; stuff written using int 10 is at c000, stuff written
	 * directly is at b800 and c800.
	 *
	 * We should just write everything to b800 and let the vmem driver sort it out. It seems that this driver
	 * may have been written later.
	*/
	uint32_t now = SDL_GetTicks();
	if(now - last_redraw_time_ms < MIN_MS_PER_FRAME) {
		return;
	}

	last_redraw_time_ms = now;

	sdl_create_screen(s);

	unsigned char *text_mem = s->text_vid_mem;
	unsigned char *text_mem_double_buffer = s->text_vid_mem_double_buffer;

	static int hits = 0;

	for(int y = 0; y < 25; y++) {
		for(int x = 0; x < 80; x++) {
			uint16_t ch_attr = *(uint16_t *)&text_mem[y*160 + x*2];
			uint16_t ch_attr_db = *(uint16_t *)&text_mem_double_buffer[y*160 + x*2];

			if(ch_attr == ch_attr_db) {
				continue;
			}

			unsigned char ch = ch_attr & 0xFF;
			unsigned char attrib = ch_attr >> 8;

			// TODO seems pixel_colors are wrong for text mode. We need to convert to an '8x8x4 color cube'.
			unsigned int figure = s->pixel_colors[attrib & 0x0F];
			unsigned int ground = s->pixel_colors[attrib >> 4];

			const unsigned char *glyph = &hercplus[ch * 14];


			for(int i = 0; i < 14; i++) {
				unsigned char *dst = (unsigned char *)sdl_screen->pixels + (y*14 + i)*sdl_screen->pitch + x*9;

				unsigned char bits = glyph[i];

				*dst++ = bits & 0x80 ? figure : ground;
				*dst++ = bits & 0x40 ? figure : ground;
				*dst++ = bits & 0x20 ? figure : ground;
				*dst++ = bits & 0x10 ? figure : ground;
				*dst++ = bits & 0x08 ? figure : ground;
				*dst++ = bits & 0x04 ? figure : ground;
				*dst++ = bits & 0x02 ? figure : ground;
				*dst++ = bits & 0x01 ? figure : ground;

				/* MDA / Herc fonts are 9 columns wide, but the 9th pixel is alway clear unless it's in the range
				 * 0xc0-0xdf (top 3 bits are 110), in which case it's the same as the 8th pixel. */
				*dst++ = ((ch & 0xe0) == 0xc0) ? (bits & 0x01 ? figure : ground) : ground;
			}

			*(uint16_t *)&text_mem_double_buffer[y*160 + x*2] = ch_attr;
		}
	}

	// Draw cursor
	unsigned char cursor_x = s->mem[0x450];
	unsigned char cursor_y = s->mem[0x451];

	if(cursor_x < 80 && cursor_y < 25) {
		for(int y = 11; y < 14; y++) {
			unsigned char *dst = (unsigned char *)sdl_screen->pixels + (cursor_y*14 + y)*sdl_screen->pitch + cursor_x*9;
			for(int i = 0; i < 9; i++) {
				*dst++ ^= 0xFF;
			}
		}

		// Update the double buffer
		*(uint16_t *)&text_mem_double_buffer[cursor_y*160 + cursor_x*2] ^= 0x0F00;
	}

	SDL_Flip(sdl_screen);
}

void sdl_redraw_display(struct x86_state *s) {
	s->graphics_inst_counter = 0;
	s->hlt_this_time = 0;
#ifdef USE_SDL_FOR_TEXT
	if (s->io_ports[0x3B8] & 2) {
		sdl_redraw_graphics(s);
	} else {
		sdl_redraw_text(s);
	}
#else
	// Video card in graphics mode?
	if (s->io_ports[0x3B8] & 2)
	{
		sdl_redraw_graphics(s);
	}
	else if (sdl_screen) // Application has gone back to text mode, so close the SDL window
	{
		SDL_QuitSubSystem(SDL_INIT_VIDEO);
		sdl_screen = 0;
	}
#endif
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

ssize_t emu_read(int fd, void *buf, size_t count) {
	return read(fd, buf, count);
}

ssize_t emu_write(int fd, const void *buf, size_t count) {
#ifdef USE_SDL_FOR_TEXT
	/* Don't write to stdout as we will just redraw text next time sdl_redraw_display is called. This works because
	 * the BIOS writes text to a buffer at b800 too. */
	if(fd == 1) return count;
#endif

	return write(fd, buf, count);
}

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

	struct x86_state *s = x86_init(boot_from_hdd, argv[1], argv[2], argv[3], sdl_redraw_display, sdl_keyboard_driver, sdl_pause_audio, emu_read, emu_write);
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
