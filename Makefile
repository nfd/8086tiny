# 8086tiny: a tiny, highly functional, highly portable PC emulator/VM
# Copyright 2013-14, Adrian Cable (adrian.cable@gmail.com) - http://www.megalith.co.uk/8086tiny
#
# This work is licensed under the MIT License. See included LICENSE.TXT.

# 8086tiny builds with graphics and sound support
# 8086tiny_slowcpu improves graphics performance on slow platforms (e.g. Raspberry Pi)
# no_graphics compiles without SDL graphics/sound

# OPTS_ALL=-O3 -fsigned-char -std=c99
OPTS_ALL=-g -fsigned-char -std=c99
OPTS_SDL=`sdl-config --cflags --libs`
OPTS_NOGFX=-DNO_GRAPHICS
OPTS_SLOWCPU=-DGRAPHICS_UPDATE_DELAY=25000

bios.bin: bios.asm
	nasm -f bin -l bios.lst -o bios.bin bios.asm

tinyxms.sys: tinyxms.asm
	nasm -f bin -I ../lmacros/ -I lmacros/ -l tinyxms.lst -o tinyxms.sys tinyxms.asm

8086tiny: 8086tiny.c bios.bin tinyxms.sys x86.c x86.h hercplus.h
	${CC} 8086tiny.c x86.c ${OPTS_SDL} ${OPTS_ALL} -o 8086tiny
	strip 8086tiny

8086tiny_slowcpu: 8086tiny.c bios.bin tinyxms.sys x86.c x86.h hercplus.h
	${CC} 8086tiny.c x86.c ${OPTS_SDL} ${OPTS_ALL} ${OPTS_SLOWCPU} -o 8086tiny
	strip 8086tiny

no_graphics: 8086tiny.c bios.bin tinyxms.sys
	${CC} 8086tiny.c x86.c ${OPTS_NOGFX} ${OPTS_ALL} -std=gnu99 -o 8086tiny
	strip 8086tiny

clean:
	rm -f 8086tiny bios.bin bios.lst tinyxms.sys tinyxms.lst
