#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#ifndef NO_GRAPHICS
#include <SDL/SDL.h>
#else
#include <time.h>
#endif

#include "x86.h"
#include "cp437.h"

#define MASK_SHIFT_COUNT(count) (31 & (count))
// change this to just (count) to be detected as an 8086

// Graphics/timer/keyboard update delays (explained later)
#define KEYBOARD_TIMER_UPDATE_DELAY 20000
#define HALT_TIME_MICROSECONDS 100
#define KEYBOARD_TIMER_DELAY_FROM_HALT 1000
#define GRAPHICS_DELAY_FROM_HALT 1000

#define AMOUNT_XMS_HANDLES 32
#define XMS_REPORTED_FREE (16 * 1024 * 1024)
#ifndef GRAPHICS_UPDATE_DELAY
#define GRAPHICS_UPDATE_DELAY 360000
#endif


// Lookup tables in the BIOS binary
#define TABLE_XLAT_OPCODE 8
#define TABLE_XLAT_SUBFUNCTION 9
#define TABLE_STD_FLAGS 10
#define TABLE_PARITY_FLAG 11
#define TABLE_BASE_INST_SIZE 12
#define TABLE_I_W_SIZE 13
#define TABLE_I_MOD_SIZE 14
#define TABLE_COND_JUMP_DECODE_A 15
#define TABLE_COND_JUMP_DECODE_B 16
#define TABLE_COND_JUMP_DECODE_C 17
#define TABLE_COND_JUMP_DECODE_D 18
#define TABLE_FLAGS_BITFIELDS 19

// Bitfields for TABLE_STD_FLAGS values
#define FLAGS_UPDATE_SZP 1
#define FLAGS_UPDATE_AO_ARITH 2
#define FLAGS_UPDATE_OC_LOGIC 4

// Helper macros

// Decode mod, r_m and reg fields in instruction
#define DECODE_RM_REG s->scratch2_uint = 4 * !s->i_mod, \
	s->op_to_addr = s->rm_addr = s->i_mod < 3 ? SEGREG(s->seg_override_en ? s->seg_override : bios_table_lookup[s->scratch2_uint + 3][s->i_rm], bios_table_lookup[s->scratch2_uint][s->i_rm], s->regs16[bios_table_lookup[s->scratch2_uint + 1][s->i_rm]] + bios_table_lookup[s->scratch2_uint + 2][s->i_rm] * s->i_data1+) : GET_REG_ADDR(s->i_rm), \
	s->op_from_addr = GET_REG_ADDR(s->i_reg), \
	s->i_d && (s->scratch_uint = s->op_from_addr, s->op_from_addr = s->rm_addr, s->op_to_addr = s->scratch_uint)

// Return memory-mapped register location (offset into mem array) for register #reg_id
#define GET_REG_ADDR(reg_id) (REGS_BASE + (s->i_w ? 2 * reg_id : 2 * reg_id + reg_id / 4 & 7))

// Returns number of top bit in operand (i.e. 8 for 8-bit operands, 16 for 16-bit operands)
#define TOP_BIT 8*(s->i_w + 1)

// Opcode execution unit helpers
#define OPCODE ;break; case
#define OPCODE_CHAIN ; case

// [I]MUL/[I]DIV/DAA/DAS/ADC/SBB helpers
#define MUL_MACRO(op_data_type,out_regs) (set_opcode(s, 0x10), \
	out_regs[s->i_w + 1] = (s->op_result = CAST(op_data_type)s->mem[s->rm_addr] * (op_data_type)*out_regs) >> 16, \
	s->regs16[REG_AX] = s->op_result, \
	set_OF(s, set_CF(s, s->op_result - (op_data_type)s->op_result)))
#define DIV_MACRO(out_data_type,in_data_type,out_regs) (s->scratch_int = CAST(out_data_type)s->mem[s->rm_addr]) && !(s->scratch2_uint = (in_data_type)(s->scratch_uint = (out_regs[s->i_w+1] << 16) + s->regs16[REG_AX]) / s->scratch_int, s->scratch2_uint - (out_data_type)s->scratch2_uint) ? out_regs[s->i_w+1] = s->scratch_uint - s->scratch_int * (*out_regs = s->scratch2_uint) : pc_interrupt_reset(s, 0)
#define DAA_DAS(op1,op2) \
	set_AF(s, (((s->scratch_uchar = s->regs8[REG_AL]) & 0x0F) > 9) || s->regs8[FLAG_AF]) && (s->op_result = (s->regs8[REG_AL] op1 6), set_CF(s, s->regs8[FLAG_CF] || (s->regs8[REG_AL] op2 s->scratch_uchar))), \
	set_CF(s, (s->regs8[REG_AL] > 0x9f) || s->regs8[FLAG_CF]) && (s->op_result = (s->regs8[REG_AL] op1 0x60))
#define ADC_SBB_MACRO(a) OP(a##= s->regs8[FLAG_CF] +), \
						 set_CF(s, s->regs8[FLAG_CF] && (s->op_result == s->op_dest) || (a s->op_result < a(int)s->op_dest)), \
						 set_AF_OF_arith(s)

// Execute arithmetic/logic operations in emulator memory/registers
#define R_M_OP(dest,op,src) (s->i_w ? s->op_dest = CAST(unsigned short)dest, s->op_result = CAST(unsigned short)dest op (s->op_source = CAST(unsigned short)src) \
								 : (s->op_dest = dest, s->op_result = dest op (s->op_source = CAST(unsigned char)src)))
#define R_M_OP_64_EQUALS(dest,op,src) (s->i_w ? s->op_dest = CAST(unsigned short)dest, s->op_result = CAST(unsigned short)dest = (uint64_t)CAST(unsigned short)dest op (uint64_t)(s->op_source = CAST(unsigned short)src) \
								 : (s->op_dest = dest, s->op_result = dest = (uint64_t)dest op (uint64_t)(s->op_source = CAST(unsigned char)src)))
#define MEM_OP(dest,op,src) R_M_OP(s->mem[dest],op,s->mem[src])
#define OP(op) MEM_OP(s->op_to_addr,op,s->op_from_addr)

// Increment or decrement a register #reg_id (usually SI or DI), depending on direction flag and operand size (given by i_w)
#define INDEX_INC(reg_id) (s->regs16[reg_id] -= (2 * s->regs8[FLAG_DF] - 1)*(s->i_w + 1))

// Helpers for stack operations
#define R_M_PUSH(a) (s->i_w = 1, R_M_OP(s->mem[SEGREG(REG_SS, REG_SP, --)], =, a))
#define R_M_POP(a) (s->i_w = 1, s->regs16[REG_SP] += 2, R_M_OP(a, =, s->mem[SEGREG(REG_SS, REG_SP, -2+)]))

// Convert segment:offset to linear address in emulator memory space
#define SEGREG(reg_seg,reg_ofs,op) 16 * s->regs16[reg_seg] + (unsigned short)(op s->regs16[reg_ofs])

// Returns sign bit of an 8-bit or 16-bit operand
#define SIGN_OF(a) (1 & (s->i_w ? CAST(short)a : a) >> (TOP_BIT - 1))

// Reinterpretation cast
#define CAST(a) *(a*)&

// Global variable definitions
unsigned char bios_table_lookup[20][256];

#ifndef NO_GRAPHICS
SDL_AudioSpec sdl_audio = {44100, AUDIO_U8, 1, 0, 128};
SDL_Surface *sdl_screen;
SDL_Event sdl_event;
unsigned short vid_addr_lookup[VIDEO_RAM_SIZE], cga_colors[4] = {0 /* Black */, 0x1F1F /* Cyan */, 0xE3E3 /* Magenta */, 0xFFFF /* White */};
#endif


// Helper functions

void callxms(struct x86_state *);

// Set carry flag
char set_CF(struct x86_state *s, int new_CF)
{
	return s->regs8[FLAG_CF] = !!new_CF;
}

// Set auxiliary flag
char set_AF(struct x86_state *s, int new_AF)
{
	return s->regs8[FLAG_AF] = !!new_AF;
}

// Set overflow flag
char set_OF(struct x86_state *s, int new_OF)
{
	return s->regs8[FLAG_OF] = !!new_OF;
}

// Set auxiliary and overflow flag after arithmetic operations
char set_AF_OF_arith(struct x86_state *s)
{
	set_AF(s, (s->op_source ^= s->op_dest ^ s->op_result) & 0x10);
	if (s->op_result == s->op_dest)
		return set_OF(s, 0);
	else
		return set_OF(s, 1 & (s->regs8[FLAG_CF] ^ s->op_source >> (TOP_BIT - 1)));
}

// Assemble and return emulated CPU FLAGS register in scratch_uint
void make_flags(struct x86_state *s)
{
	s->scratch_uint = 0xF002; // 8086 has reserved and unused flags set to 1
	for (int i = 9; i--;)
		s->scratch_uint += s->regs8[FLAG_CF + i] << bios_table_lookup[TABLE_FLAGS_BITFIELDS][i];
}

// Set emulated CPU FLAGS register from regs8[FLAG_xx] values
void set_flags(struct x86_state *s, int new_flags)
{
	for (int i = 9; i--;)
		s->regs8[FLAG_CF + i] = !!(1 << bios_table_lookup[TABLE_FLAGS_BITFIELDS][i] & new_flags);
}

// Convert raw opcode to translated opcode index. This condenses a large number of different encodings of similar
// instructions into a much smaller number of distinct functions, which we then execute
void set_opcode(struct x86_state *s, unsigned char opcode)
{
	s->xlat_opcode_id = bios_table_lookup[TABLE_XLAT_OPCODE][s->raw_opcode_id = opcode];
	s->extra = bios_table_lookup[TABLE_XLAT_SUBFUNCTION][opcode];
	s->i_mod_size = bios_table_lookup[TABLE_I_MOD_SIZE][opcode];
	s->set_flags_type = bios_table_lookup[TABLE_STD_FLAGS][opcode];
}

// Execute INT #interrupt_num on the emulated machine
char pc_interrupt(struct x86_state *s, unsigned char interrupt_num)
{
	set_opcode(s, 0xCD); // Decode like INT

	make_flags(s);
	R_M_PUSH(s->scratch_uint);
	R_M_PUSH(s->regs16[REG_CS]);
	R_M_PUSH(s->reg_ip);
	MEM_OP(REGS_BASE + 2 * REG_CS, =, 4 * interrupt_num + 2);
	R_M_OP(s->reg_ip, =, s->mem[4 * interrupt_num]);

	s->trap_flag = 0;
	return s->regs8[FLAG_TF] = s->regs8[FLAG_IF] = 0;
}

// Execute interrupt but reset ip
char pc_interrupt_reset(struct x86_state *s, unsigned char interrupt_num)
{
	s->reg_ip = s->reg_ip_before_rep_trace;
	return pc_interrupt(s, interrupt_num);
}


// AAA and AAS instructions - which_operation is +1 for AAA, and -1 for AAS
int AAA_AAS(struct x86_state *s, char which_operation)
{
	return (s->regs16[REG_AX] += 262 * which_operation*set_AF(s, set_CF(s, ((s->regs8[REG_AL] & 0x0F) > 9) || s->regs8[FLAG_AF])), s->regs8[REG_AL] &= 0x0F);
}

#ifndef NO_GRAPHICS
void audio_callback(void *data, unsigned char *stream, int len)
{
	struct x86_state *s = data;
	for (int i = 0; i < len; i++)
		stream[i] = (s->spkr_en == 3) && CAST(unsigned short)s->mem[0x4AA] ? -((54 * s->wave_counter++ / CAST(unsigned short)s->mem[0x4AA]) & 1) : sdl_audio.silence;

	s->spkr_en = s->io_ports[0x61] & 3;
}
#endif

void callin(struct x86_state *s, uint8_t* dest, uint16_t port) {
	s->io_ports[0x20] = 0; // PIC EOI
	s->io_ports[0x42] = --s->io_ports[0x40]; // PIT channel 0/2 read placeholder
	s->io_ports[0x3DA] ^= 9; // CGA refresh
	port == 0x60 && (s->io_ports[0x64] = 0); // Scancode read flag
	port == 0x3D5 && (s->io_ports[0x3D4] >> 1 == 7) && (s->io_ports[0x3D5] = ((s->mem[0x49E]*80 + s->mem[0x49D] + CAST(short)s->mem[0x4AD]) & (s->io_ports[0x3D4] & 1 ? 0xFF : 0xFF00)) >> (s->io_ports[0x3D4] & 1 ? 0 : 8)); // CRT cursor position
	R_M_OP(*dest, =, s->io_ports[port]);
}

void callout(struct x86_state *s, uint8_t* source, uint16_t port) {
	R_M_OP(s->io_ports[port], =, *source);
	port == 0x61 && (s->io_hi_lo = 0, s->spkr_en |= *source & 3); // Speaker control
	(port == 0x40 || port == 0x42) && (s->io_ports[0x43] & 6) && (s->mem[0x469 + port - (s->io_hi_lo ^= 1)] = *source); // PIT rate programming
#ifndef NO_GRAPHICS
	port == 0x43 && (s->io_hi_lo = 0, *source >> 6 == 2) && (SDL_PauseAudio((*source & 0xF7) != 0xB6), 0); // Speaker enable
#endif
	port == 0x3D5 && (s->io_ports[0x3D4] >> 1 == 6) && (s->mem[0x4AD + !(s->io_ports[0x3D4] & 1)] = *source); // CRT video RAM start offset
	port == 0x3D5 && (s->io_ports[0x3D4] >> 1 == 7) && (s->scratch2_uint = ((s->mem[0x49E]*80 + s->mem[0x49D] + CAST(short)s->mem[0x4AD]) & (s->io_ports[0x3D4] & 1 ? 0xFF00 : 0xFF)) + (*source << (s->io_ports[0x3D4] & 1 ? 0 : 8)) - CAST(short)s->mem[0x4AD], s->mem[0x49D] = s->scratch2_uint % 80, s->mem[0x49E] = s->scratch2_uint / 80); // CRT cursor position
	port == 0x3B5 && s->io_ports[0x3B4] == 1 && (s->GRAPHICS_X = *source * 16); // Hercules resolution reprogramming. Defaults are set in the BIOS
	port == 0x3B5 && s->io_ports[0x3B4] == 6 && (s->GRAPHICS_Y = *source * 4);
}

static size_t
code_to_utf8(unsigned char *const buffer,
             const unsigned short code)
{
	if (code <= 0x7F) {
		buffer[0] = code;
		return 1;
	}
	if (code <= 0x7FF) {
		buffer[0] = 0xC0 | (code >> 6);    /* 110xxxxx */
		buffer[1] = 0x80 | (code & 0x3F);  /* 10xxxxxx */
		return 2;
	}

	buffer[0] = 0xE0 | (code >> 12);           /* 1110xxxx */
	buffer[1] = 0x80 | ((code >> 6) & 0x3F);   /* 10xxxxxx */
	buffer[2] = 0x80 | (code & 0x3F);          /* 10xxxxxx */
	return 3;
}

// Keyboard driver for console. This may need changing for UNIX/non-UNIX platforms
#ifdef _WIN32
void keyboard_driver(struct x86_state *s) {
	kbhit() && (s->mem[0x4A6] = getch(), pc_interrupt(7));
}
#else
void keyboard_driver(struct x86_state *s) {
	read(0, s->mem + 0x4A6, 1) && (s->int8_asap = (s->mem[0x4A6] == 0x1B), pc_interrupt(s, 7));
}
#endif


// Keyboard driver for SDL
#ifdef NO_GRAPHICS
void sdl_keyboard_driver(struct x86_state *s) {
	keyboard_driver(s);
}
#else
void sdl_keyboard_driver(struct x86_state *s) {
	sdl_screen ? SDL_PollEvent(&sdl_event) && (sdl_event.type == SDL_KEYDOWN || sdl_event.type == SDL_KEYUP) && (s->scratch_uint = sdl_event.key.keysym.unicode, s->scratch2_uint = sdl_event.key.keysym.mod, CAST(short)s->mem[0x4A6] = 0x400 + 0x800*!!(s->scratch2_uint & KMOD_ALT) + 0x1000*!!(s->scratch2_uint & KMOD_SHIFT) + 0x2000*!!(s->scratch2_uint & KMOD_CTRL) + 0x4000*(sdl_event.type == SDL_KEYUP) + ((!s->scratch_uint || s->scratch_uint > 0x7F) ? sdl_event.key.keysym.sym : s->scratch_uint), pc_interrupt(s, 7)) : (keyboard_driver(s));
}
#endif



struct x86_state *x86_init(int boot_from_hdd, char *bios_filename, char *fdd_filename, char *hdd_filename, void(*redraw_display)(struct x86_state *), void(*keyboard_driver)(struct x86_state *)) {
	struct x86_state *s = calloc(1, sizeof(struct x86_state));

#ifndef NO_GRAPHICS
	// Initialise SDL
	SDL_Init(SDL_INIT_AUDIO);
	sdl_audio.callback = audio_callback;
	sdl_audio.userdata = s;
#ifdef _WIN32
	sdl_audio.samples = 512;
#endif
	SDL_OpenAudio(&sdl_audio, 0);
#endif

	// regs16 and reg8 point to F000:0, the start of memory-mapped registers. CS is initialised to F000
	s->regs16 = (unsigned short *)(s->regs8 = s->mem + REGS_BASE);
	s->regs16[REG_CS] = 0xF000;

	// Trap flag off
	s->regs8[FLAG_TF] = 0;

	// Set DL equal to the boot device: 0 for the FD, or 0x80 for the HD. Normally, boot from the FD.
	// But, if the HD image file is prefixed with @, then boot from the HD
	s->regs8[REG_DL] = boot_from_hdd ? 0x80 : 0;

	// Open BIOS (file id disk[2]), floppy disk image (disk[1]), and hard disk image (disk[0]) if specified
	if(bios_filename) s->disk_bios = open(bios_filename, 32898);
	if(fdd_filename) s->disk_fdd = open(fdd_filename, 32898);
	if(hdd_filename) s->disk_hdd = open(hdd_filename, 32898);

	// Set CX:AX equal to the hard disk image size, if present
	CAST(unsigned)s->regs16[REG_AX] = s->disk_hdd ? lseek(s->disk_hdd, 0, 2) >> 9 : 0;

	// Load BIOS image into F000:0100, and set IP to 0100
	read(s->disk_bios, s->regs8 + (s->reg_ip = s->reg_ip_before_rep_trace = 0x100), 0xFF00);

	// Load instruction decoding helper table
	for (int i = 0; i < 20; i++)
		for (int j = 0; j < 256; j++)
			bios_table_lookup[i][j] = s->regs8[s->regs16[0x81 + i] + j];

	return s;
}

// Emulator entry point
void x86_step(struct x86_state *s)
{
#ifdef XMS_DEBUG
	if (s->opcode_stream[0] == 0xEB &&
			s->opcode_stream[1] == 0x3 &&
			s->opcode_stream[2] == 0x90 &&
			s->opcode_stream[3] == 0x90 &&
			s->opcode_stream[4] == 0x90) {
		printf("xms call ax=%04Xh dx=%04Xh bx=%04Xh\r\n",
				s->regs16[REG_AX], s->regs16[REG_DX], s->regs16[REG_BX]);
	}
#endif
	s->setting_ss = 0;

	// Set up variables to prepare for decoding an opcode
	set_opcode(s, *s->opcode_stream);

	// Extract i_w and i_d fields from instruction
	s->i_w = (s->i_reg4bit = s->raw_opcode_id & 7) & 1;
	s->i_d = s->i_reg4bit / 2 & 1;

	// Extract instruction data fields
	s->i_data0 = CAST(short)s->opcode_stream[1];
	s->shift_count = s->i_data1 = CAST(short)s->opcode_stream[2];
	s->i_data2 = CAST(short)s->opcode_stream[3];

	// seg_override_en and rep_override_en contain number of instructions to hold segment override and REP prefix respectively
	if (s->seg_override_en)
		s->seg_override_en--;
	if (s->rep_override_en)
		s->rep_override_en--;

	// i_mod_size > 0 indicates that opcode uses i_mod/i_rm/i_reg, so decode them
	if (s->i_mod_size)
	{
		s->i_mod = (s->i_data0 & 0xFF) >> 6;
		s->i_rm = s->i_data0 & 7;
		s->i_reg = s->i_data0 / 8 & 7;

		if ((!s->i_mod && s->i_rm == 6) || (s->i_mod == 2)) {
			s->i_data2 = CAST(short)s->opcode_stream[4];
			s->shift_count = s->opcode_stream[4];
		} else if (s->i_mod != 1) {
			s->i_data2 = s->i_data1;
		} else { // If i_mod is 1, operand is (usually) 8 bits rather than 16 bits
			s->i_data1 = (char)s->i_data1;
			s->shift_count = s->opcode_stream[3];
		}

		DECODE_RM_REG;
	}

	// Instruction execution unit
	switch (s->xlat_opcode_id)
	{
		OPCODE_CHAIN 0: // Conditional jump (JAE, JNAE, etc.)
						// i_w is the invert flag, e.g. i_w == 1 means JNAE, whereas i_w == 0 means JAE
			s->scratch_uchar = s->raw_opcode_id / 2 & 7;
		s->reg_ip += (char)s->i_data0 * (s->i_w ^ (s->regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_A][s->scratch_uchar]] || s->regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_B][s->scratch_uchar]] || s->regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_C][s->scratch_uchar]] ^ s->regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_D][s->scratch_uchar]]))
			OPCODE 1: // MOV reg, imm
			s->i_w = !!(s->raw_opcode_id & 8);
		R_M_OP(s->mem[GET_REG_ADDR(s->i_reg4bit)], =, s->i_data0)
			OPCODE 3: // PUSH regs16
			R_M_PUSH(s->regs16[s->i_reg4bit])
			OPCODE 4: // POP regs16
			R_M_POP(s->regs16[s->i_reg4bit])
			OPCODE 2: // INC|DEC regs16
			s->i_w = 1;
		s->i_d = 0;
		s->i_reg = s->i_reg4bit;
		DECODE_RM_REG;
		s->i_reg = s->extra
			OPCODE_CHAIN 5: // INC|DEC|JMP|CALL|PUSH
			if (s->i_reg < 2) // INC|DEC
				MEM_OP(s->op_from_addr, += 1 - 2 * s->i_reg +, REGS_BASE + 2 * REG_ZERO),
					s->op_source = 1,
					set_AF_OF_arith(s),
					set_OF(s, s->op_dest + 1 - s->i_reg == 1 << (TOP_BIT - 1)),
					(s->xlat_opcode_id == 5) && (set_opcode(s, 0x10), 0); // Decode like ADC
			else if (s->i_reg != 6) // JMP|CALL
				s->i_reg - 3 || R_M_PUSH(s->regs16[REG_CS]), // CALL (far)
					  s->i_reg & 2 && R_M_PUSH(s->reg_ip + 2 + s->i_mod*(s->i_mod != 3) + 2*(!s->i_mod && s->i_rm == 6)), // CALL (near or far)
					  s->i_reg & 1 && (s->regs16[REG_CS] = CAST(short)s->mem[s->op_from_addr + 2]), // JMP|CALL (far)
					  R_M_OP(s->reg_ip, =, s->mem[s->op_from_addr]),
					  set_opcode(s, 0x9A); // Decode like CALL
			else // PUSH
				R_M_PUSH(s->mem[s->rm_addr])
					OPCODE 6: // TEST r/m, imm16 / NOT|NEG|MUL|IMUL|DIV|IDIV reg
					s->op_to_addr = s->op_from_addr;

		switch (s->i_reg)
		{
			OPCODE_CHAIN 0: // TEST
				set_opcode(s, 0x20); // Decode like AND
			s->reg_ip += s->i_w + 1;
			R_M_OP(s->mem[s->op_to_addr], &, s->i_data2)
				OPCODE 2: // NOT
				OP(=~)
				OPCODE 3: // NEG
				OP(=-);
			s->op_dest = 0;
			set_opcode(s, 0x28); // Decode like SUB
			set_CF(s, s->op_result > s->op_dest)
				OPCODE 4: // MUL
				s->i_w ? MUL_MACRO(unsigned short, s->regs16) : MUL_MACRO(unsigned char, s->regs8)
				OPCODE 5: // IMUL
				s->i_w ? MUL_MACRO(short, s->regs16) : MUL_MACRO(char, s->regs8)
				OPCODE 6: // DIV
				s->i_w ? DIV_MACRO(unsigned short, unsigned, s->regs16) : DIV_MACRO(unsigned char, unsigned short, s->regs8)
				OPCODE 7: // IDIV
				s->i_w ? DIV_MACRO(short, int, s->regs16) : DIV_MACRO(char, short, s->regs8);
		}
		OPCODE 7: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP AL/AX, immed
			s->rm_addr = REGS_BASE;
		s->i_data2 = s->i_data0;
		s->i_mod = 3;
		s->i_reg = s->extra;
		s->reg_ip--;
		OPCODE_CHAIN 8: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP reg, immed
			s->op_to_addr = s->rm_addr;
		s->regs16[REG_SCRATCH] = (s->i_d |= !s->i_w) ? (char)s->i_data2 : s->i_data2;
		s->op_from_addr = REGS_BASE + 2 * REG_SCRATCH;
		s->reg_ip += !s->i_d + 1;
		set_opcode(s, 0x08 * (s->extra = s->i_reg));
		OPCODE_CHAIN 9: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP|MOV reg, r/m
			switch (s->extra)
			{
				OPCODE_CHAIN 0: // ADD
					OP(+=),
					set_CF(s, s->op_result < s->op_dest)
						OPCODE 1: // OR
						OP(|=)
						OPCODE 2: // ADC
						ADC_SBB_MACRO(+)
						OPCODE 3: // SBB
						ADC_SBB_MACRO(-)
						OPCODE 4: // AND
						OP(&=)
						OPCODE 5: // SUB
						OP(-=),
					set_CF(s, s->op_result > s->op_dest)
						OPCODE 6: // XOR
						OP(^=)
						OPCODE 7: // CMP
						OP(-),
					set_CF(s, s->op_result > s->op_dest)
						OPCODE 8: // MOV
						OP(=);
			}
		OPCODE 10: // MOV sreg, r/m | POP r/m | LEA reg, r/m
			if (!s->i_w) { // MOV
				s->i_w = 1;
				s->i_reg += 8;
				DECODE_RM_REG;
				if (s->i_d && (s->op_to_addr == REGS_BASE + 2 * REG_CS
							|| s->op_to_addr > REGS_BASE + 2 * REG_DS)) {
					pc_interrupt_reset(s, 6);
					break;
				}
				if (!s->i_d && (s->op_from_addr > REGS_BASE + 2 * REG_DS)) {
					pc_interrupt_reset(s, 6);
					break;
				}
				OP(=);
				if (s->i_d && s->op_to_addr == REGS_BASE + 2 * REG_SS)
					s->setting_ss = 1;
			} else if (!s->i_d) // LEA
				s->seg_override_en = 1,
								s->seg_override = REG_ZERO,
								DECODE_RM_REG,
								R_M_OP(s->mem[s->op_from_addr], =, s->rm_addr);
			else // POP
				R_M_POP(s->mem[s->rm_addr])
					OPCODE 11: // MOV AL/AX, [loc]
					s->i_mod = s->i_reg = 0;
		s->i_rm = 6;
		s->i_data1 = s->i_data0;
		DECODE_RM_REG;
		MEM_OP(s->op_from_addr, =, s->op_to_addr)
			OPCODE 12: // ROL|ROR|RCL|RCR|SHL|SHR|???|SAR reg/mem, 1/CL/imm (80186)
			s->scratch2_uint = SIGN_OF(s->mem[s->rm_addr]),
						  s->scratch_uint = s->extra ? // xxx reg/mem, imm
							  MASK_SHIFT_COUNT(s->shift_count)
							  : // xxx reg/mem, CL
							  s->i_d
							  ? MASK_SHIFT_COUNT(s->regs8[REG_CL])
							  : // xxx reg/mem, 1
							  1;
		if (s->scratch_uint)
		{
			if (s->i_reg < 4) // Rotate operations
				s->scratch_uint %= s->i_reg / 2 + TOP_BIT,
							 R_M_OP(s->scratch2_uint, =, s->mem[s->rm_addr]);
			if (s->i_reg & 1) // Rotate/shift right operations
				R_M_OP_64_EQUALS(s->mem[s->rm_addr], >>, s->scratch_uint);
			else // Rotate/shift left operations
				R_M_OP_64_EQUALS(s->mem[s->rm_addr], <<, s->scratch_uint);
			if (s->i_reg > 3) // Shift operations
				s->set_flags_type = FLAGS_UPDATE_SZP; // Shift instructions affect SZP
			if (s->i_reg > 4) // SHR or SAR
				set_CF(s, (uint64_t)s->op_dest >> (s->scratch_uint - 1) & 1);
		}

		switch (s->i_reg)
		{
			OPCODE_CHAIN 0: // ROL
				R_M_OP_64_EQUALS(s->mem[s->rm_addr], + , s->scratch2_uint >> (TOP_BIT - s->scratch_uint));
			set_OF(s, SIGN_OF(s->op_result) ^ set_CF(s, s->op_result & 1))
				OPCODE 1: // ROR
				s->scratch2_uint &= (1 << s->scratch_uint) - 1,
							  R_M_OP_64_EQUALS(s->mem[s->rm_addr], + , s->scratch2_uint << (TOP_BIT - s->scratch_uint));
			set_OF(s, SIGN_OF(s->op_result * 2) ^ set_CF(s, SIGN_OF(s->op_result)))
				OPCODE 2: // RCL
				R_M_OP_64_EQUALS(s->mem[s->rm_addr], + ((uint64_t)s->regs8[FLAG_CF] << (s->scratch_uint - 1)) + , s->scratch2_uint >> (1 + TOP_BIT - s->scratch_uint));
			set_OF(s, SIGN_OF(s->op_result) ^ set_CF(s, s->scratch2_uint & (uint64_t)1 << (TOP_BIT - s->scratch_uint)))
				OPCODE 3: // RCR
				R_M_OP_64_EQUALS(s->mem[s->rm_addr], + ((uint64_t)s->regs8[FLAG_CF] << (TOP_BIT - s->scratch_uint)) + , s->scratch2_uint << (1 + TOP_BIT - s->scratch_uint));
			set_CF(s, s->scratch2_uint & (uint64_t)1 << (s->scratch_uint - 1));
			set_OF(s, SIGN_OF(s->op_result) ^ SIGN_OF(s->op_result * 2))
				OPCODE 4: // SHL
				set_OF(s, SIGN_OF(s->op_result) ^ set_CF(s, SIGN_OF(s->op_dest << (s->scratch_uint - 1))))
				OPCODE 5: // SHR
				set_OF(s, SIGN_OF(s->op_dest))
				OPCODE 7: // SAR
				s->scratch_uint < TOP_BIT || set_CF(s, s->scratch2_uint);
			set_OF(s, 0);
			R_M_OP_64_EQUALS(s->mem[s->rm_addr], +, s->scratch2_uint *= ~((uint64_t)(((uint64_t)1 << TOP_BIT) - 1) >> s->scratch_uint));
		}
		OPCODE 13: // LOOPxx|JCZX
			s->scratch_uint = !!--s->regs16[REG_CX];

		switch(s->i_reg4bit)
		{
			OPCODE_CHAIN 0: // LOOPNZ
				s->scratch_uint &= !s->regs8[FLAG_ZF]
				OPCODE 1: // LOOPZ
				s->scratch_uint &= s->regs8[FLAG_ZF]
				OPCODE 3: // JCXXZ
				s->scratch_uint = !++s->regs16[REG_CX];
		}
		s->reg_ip += s->scratch_uint*(char)s->i_data0
			OPCODE 14: // JMP | CALL short/near
			s->reg_ip += 3 - s->i_d;
		if (!s->i_w)
		{
			if (s->i_d) // JMP far
				s->reg_ip = 0,
					   s->regs16[REG_CS] = s->i_data2;
			else // CALL
				R_M_PUSH(s->reg_ip);
		}
		s->reg_ip += s->i_d && s->i_w ? (char)s->i_data0 : s->i_data0
			OPCODE 15: // TEST reg, r/m
			MEM_OP(s->op_from_addr, &, s->op_to_addr)
			OPCODE 16: // XCHG AX, regs16
			s->i_w = 1;
		s->op_to_addr = REGS_BASE;
		s->op_from_addr = GET_REG_ADDR(s->i_reg4bit);
		OPCODE_CHAIN 24: // NOP|XCHG reg, r/m
			if (s->op_to_addr != s->op_from_addr)
				OP(^=),
					MEM_OP(s->op_from_addr, ^=, s->op_to_addr),
					OP(^=)
						OPCODE 17: // MOVSx (extra=0)|STOSx (extra=1)|LODSx (extra=2)
						s->scratch2_uint = s->seg_override_en ? s->seg_override : REG_DS;

		s->scratch_uint = s->rep_override_en ? s->regs16[REG_CX] : 1;
		if (s->trap_flag && s->scratch_uint > 1) {
			s->reset_ip_after_rep_trace = 1;
			s->scratch_uint = 1;
		}
		for (; s->scratch_uint; s->scratch_uint--)
		{
			MEM_OP(s->extra < 2 ? SEGREG(REG_ES, REG_DI,) : REGS_BASE, =, s->extra & 1 ? REGS_BASE : SEGREG(s->scratch2_uint, REG_SI,)),
				s->extra & 1 || INDEX_INC(REG_SI),
				s->extra & 2 || INDEX_INC(REG_DI);
			s->rep_override_en && s->regs16[REG_CX]--;
		}
		OPCODE 18: // CMPSx (extra=0)|SCASx (extra=1)
			s->scratch2_uint = s->seg_override_en ? s->seg_override : REG_DS;

		s->scratch_uint = s->rep_override_en ? s->regs16[REG_CX] : 1;
		if (s->trap_flag && s->scratch_uint > 1) {
			s->reset_ip_after_rep_trace = 1;
			s->scratch_uint = 1;
		}
		if (s->scratch_uint)
		{
			for (; s->scratch_uint; s->rep_override_en || s->scratch_uint--)
			{
				MEM_OP(s->extra ? REGS_BASE : SEGREG(s->scratch2_uint, REG_SI,), -, SEGREG(REG_ES, REG_DI,));
				s->extra || INDEX_INC(REG_SI);
				INDEX_INC(REG_DI);
				if (s->rep_override_en) {
					if (!(--s->regs16[REG_CX] && (!s->op_result == s->rep_mode))) {
						(s->scratch_uint = 0, s->reset_ip_after_rep_trace = 0);
					} else if (s->trap_flag) {
						s->scratch_uint = 0;
					}
				}
			}

			s->set_flags_type = FLAGS_UPDATE_SZP | FLAGS_UPDATE_AO_ARITH; // Funge to set SZP/AO flags
			set_CF(s, s->op_result > s->op_dest);
		}
		OPCODE 19: // RET|RETF|IRET
			s->i_d = s->i_w;
		R_M_POP(s->reg_ip);
		if (s->extra) // IRET|RETF|RETF imm16
			R_M_POP(s->regs16[REG_CS]);
		if (s->extra & 2) // IRET
			set_flags(s, R_M_POP(s->scratch_uint));
		else if (!s->i_d) // RET|RETF imm16
			s->regs16[REG_SP] += s->i_data0
				OPCODE 20: // MOV r/m, immed
				R_M_OP(s->mem[s->op_from_addr], =, s->i_data2)
				OPCODE 21: // IN AL/AX, DX/imm8
				s->scratch_uint = s->extra ? s->regs16[REG_DX] : (uint8_t)s->i_data0;
		callin(s, &s->regs8[REG_AL], s->scratch_uint);
		OPCODE 22: // OUT DX/imm8, AL/AX
			s->scratch_uint = s->extra ? s->regs16[REG_DX] : (uint8_t)s->i_data0;
		callout(s, &s->regs8[REG_AL], s->scratch_uint);
		OPCODE 23: // REPxx
			s->rep_override_en = 2;
		s->rep_mode = s->i_w;
		s->seg_override_en && s->seg_override_en++
			OPCODE 25: // PUSH reg
			R_M_PUSH(s->regs16[s->extra])
			OPCODE 26: // POP reg
			R_M_POP(s->regs16[s->extra]);
		if ((void*)&s->regs16[s->extra] == (void*)&s->mem[REGS_BASE + 2 * REG_SS])
			s->setting_ss = 1;
		OPCODE 27: // xS: segment overrides
			s->seg_override_en = 2;
		s->seg_override = s->extra;
		s->rep_override_en && s->rep_override_en++
			OPCODE 28: // DAA/DAS
			s->i_w = 0;
		if (s->extra) DAA_DAS(-=, >); else DAA_DAS(+=, <) // extra = 0 for DAA, 1 for DAS
			OPCODE 29: // AAA/AAS
				s->op_result = AAA_AAS(s, s->extra - 1)
				OPCODE 30: // CBW
				s->regs8[REG_AH] = -SIGN_OF(s->regs8[REG_AL])
				OPCODE 31: // CWD
				s->regs16[REG_DX] = -SIGN_OF(s->regs16[REG_AX])
				OPCODE 32: // CALL FAR imm16:imm16
				R_M_PUSH(s->regs16[REG_CS]);
		R_M_PUSH(s->reg_ip + 5);
		s->regs16[REG_CS] = s->i_data2;
		s->reg_ip = s->i_data0
			OPCODE 33: // PUSHF
			make_flags(s);
		R_M_PUSH(s->scratch_uint)
			OPCODE 34: // POPF
			set_flags(s, R_M_POP(s->scratch_uint))
			OPCODE 35: // SAHF
			make_flags(s);
		set_flags(s, (s->scratch_uint & 0xFF00) + s->regs8[REG_AH])
			OPCODE 36: // LAHF
			make_flags(s),
			s->regs8[REG_AH] = s->scratch_uint
				OPCODE 37: // LES|LDS reg, r/m
				if (s->i_mod == 3) {
					pc_interrupt_reset(s, 6);
					break;
				}
		s->i_w = s->i_d = 1;
		DECODE_RM_REG;
		OP(=);
		MEM_OP(REGS_BASE + s->extra, =, s->rm_addr + 2)
			OPCODE 38: // INT 3
			++s->reg_ip;
		pc_interrupt(s, 3)
			OPCODE 39: // INT imm8
			s->reg_ip += 2;
		pc_interrupt(s, s->i_data0)
			OPCODE 40: // INTO
			++s->reg_ip;
		s->regs8[FLAG_OF] && pc_interrupt(s, 4)
			OPCODE 41: // AAM
			if (s->i_data0 &= 0xFF)
				s->regs8[REG_AH] = s->regs8[REG_AL] / s->i_data0,
					s->op_result = s->regs8[REG_AL] %= s->i_data0;
			else // Divide by zero
				pc_interrupt_reset(s, 0)
					OPCODE 42: // AAD
					s->i_w = 0;
		s->regs16[REG_AX] = s->op_result = 0xFF & s->regs8[REG_AL] + s->i_data0 * s->regs8[REG_AH]
			OPCODE 43: // SALC
			s->regs8[REG_AL] = -s->regs8[FLAG_CF]
			OPCODE 44: // XLAT
			s->regs8[REG_AL] = s->mem[SEGREG(s->seg_override_en ? s->seg_override : REG_DS, REG_BX, s->regs8[REG_AL] +)]
			OPCODE 45: // CMC
			s->regs8[FLAG_CF] ^= 1
			OPCODE 46: // CLC|STC|CLI|STI|CLD|STD
			s->regs8[s->extra / 2] = s->extra & 1
			OPCODE 47: // TEST AL/AX, immed
			R_M_OP(s->regs8[REG_AL], &, s->i_data0)
			OPCODE 48: // Emulator-specific 0F xx opcodes
			switch ((char)s->i_data0)
			{
				OPCODE_CHAIN 0: { // PUTCHAR_AL
					unsigned char buf[3];
					unsigned len;

					len = code_to_utf8(buf, cp437_to_utf16[*s->regs8]);
					write(1, buf, len);
				}
				OPCODE 1: // GET_RTC
					time(&s->clock_buf);
				ftime(&s->ms_clock);
				memcpy(s->mem + SEGREG(REG_ES, REG_BX,), localtime(&s->clock_buf), sizeof(struct tm));
				CAST(short)s->mem[SEGREG(REG_ES, REG_BX, 36+)] = s->ms_clock.millitm;
				OPCODE 2: // DISK_READ
				OPCODE_CHAIN 3: // DISK_WRITE
				{
					int disk = 0;
					switch(s->regs8[REG_DL]) {
						case 2: disk = s->disk_bios; break;
						case 1: disk = s->disk_fdd; break;
						case 0: disk = s->disk_hdd; break;
					}

					s->regs8[REG_AL] = ~lseek(disk, CAST(unsigned)s->regs16[REG_BP] << 9, 0)
					? ((char)s->i_data0 == 3 ? (int(*)())write : (int(*)())read)(disk, s->mem + SEGREG(REG_ES, REG_BX,), s->regs16[REG_AX])
					: 0;
				}
				OPCODE 4:	// XMS
					callxms(s);
			}
		if ((uint8_t)s->i_data0 == 11	// ud2
				|| (uint8_t)s->i_data0 >= 32)
			pc_interrupt_reset(s, 6);
		OPCODE 100: // HLT
			s->hlt_this_time = 1;
		OPCODE 101: // PUSH imm16
			R_M_PUSH(s->i_data0);
		OPCODE 102: // PUSH imm8
			s->scratch_uint = (int16_t)(int8_t)s->i_data0;
		R_M_PUSH(s->scratch_uint);
		OPCODE 103: // FPU instructions - no ops
			OPCODE 104: // PUSHA
			s->scratch_uint = s->regs16[REG_SP];
		R_M_PUSH(s->regs16[REG_AX]);
		R_M_PUSH(s->regs16[REG_CX]);
		R_M_PUSH(s->regs16[REG_DX]);
		R_M_PUSH(s->regs16[REG_BX]);
		R_M_PUSH(s->scratch_uint);
		R_M_PUSH(s->regs16[REG_BP]);
		R_M_PUSH(s->regs16[REG_SI]);
		R_M_PUSH(s->regs16[REG_DI]);
		OPCODE 105: // POPA
			R_M_POP(s->regs16[REG_DI]);
		R_M_POP(s->regs16[REG_SI]);
		R_M_POP(s->regs16[REG_BP]);
		R_M_POP(s->scratch_uint);
		R_M_POP(s->regs16[REG_BX]);
		R_M_POP(s->regs16[REG_DX]);
		R_M_POP(s->regs16[REG_CX]);
		R_M_POP(s->regs16[REG_AX]);
		OPCODE 106: // ENTER
			R_M_PUSH(s->regs16[REG_BP]);
		s->scratch_uint = s->regs16[REG_SP];
		s->scratch_int = (uint8_t)s->i_data2 & 31;
		if (s->scratch_int > 0) {
			for (; s->scratch_int > 1; --s->scratch_int) {
				s->regs16[REG_BP] -= 2;
				R_M_PUSH(CAST(uint16_t)s->mem[SEGREG(REG_SS, REG_BP,)]);
			}
			R_M_PUSH(s->scratch_uint);
		}
		s->regs16[REG_BP] = s->scratch_uint;
		s->regs16[REG_SP] -= s->i_data0;
		OPCODE 107: // LEAVE
			s->regs16[REG_SP] = s->regs16[REG_BP];
		R_M_POP(s->regs16[REG_BP]);
		OPCODE 108: // IMUL r,r/m,imm16 / imm8
			s->i_w = 1;
		s->i_d = 1;
		DECODE_RM_REG;
		s->op_to_addr = s->op_from_addr;
#ifdef IMUL_DEBUG
		printf("imul extra=%u op_to_addr=%08Xh "
				"i_data2=%04Xh i_reg=%u ",
				s->extra, s->op_to_addr, s->i_data2, s->i_reg);
#endif
		if (s->extra) {
			;
#ifdef IMUL_DEBUG
			printf("w ");
#endif
		} else {
			s->i_data2 = (int16_t)(int8_t)s->i_data2;
#ifdef IMUL_DEBUG
			printf("b ");
#endif
		}
		R_M_OP(s->mem[s->op_to_addr], *, s->i_data2);
#ifdef IMUL_DEBUG
		printf("i_data2(cast)=%04X op_result=%08Xh\r\n",
				s->i_data2, s->op_result);
#endif
		s->regs16[s->i_reg] = s->op_result;
		set_OF(s, set_CF(s, (int32_t)s->op_result
					- (int32_t)(int16_t)s->op_result));
		OPCODE 109: // BOUND
			if (s->i_mod == 3) {
				pc_interrupt_reset(s, 6);
				break;
			}
		s->i_w = 1;
		s->i_d = 0;
		DECODE_RM_REG;
		// op_to_addr = mem operand
		// op_from_addr = reg operand
		if ((CAST(int16_t)s->mem[s->op_from_addr]
					< CAST(int16_t)s->mem[s->op_to_addr])
				|| (CAST(int16_t)s->mem[s->op_from_addr]
					> CAST(int16_t)s->mem[s->op_to_addr + 2]))
			pc_interrupt_reset(s, 5);
		OPCODE 110: // INSx (extra=1), OUTSx (extra=2)
			s->scratch2_uint = s->seg_override_en ? s->seg_override : REG_DS;

		s->scratch_uint = s->rep_override_en ? s->regs16[REG_CX] : 1;
		if (s->trap_flag && s->scratch_uint > 1) {
			s->reset_ip_after_rep_trace = 1;
			s->scratch_uint = 1;
		}
		for (; s->scratch_uint; s->scratch_uint--)
		{
			if (s->extra == 1)
				callin(s, &s->mem[SEGREG(REG_ES, REG_DI,)], s->regs16[REG_DX]);
			else
				callout(s, &s->mem[SEGREG(s->scratch2_uint, REG_SI,)], s->regs16[REG_DX]);
			s->extra & 1 || INDEX_INC(REG_SI),
				  s->extra & 2 || INDEX_INC(REG_DI);
			s->rep_override_en && s->regs16[REG_CX]--;
		}
		OPCODE 111: // LOCK prefix
			s->seg_override_en && s->seg_override_en++;
		s->rep_override_en && s->rep_override_en++;
		OPCODE 112: // INT1
			++s->reg_ip;
		pc_interrupt(s, 1);
		break; default:
#ifdef INT6_DEBUG
			printf("Interrupt 6 at %04X:%04X = %02X %02X %02X %02X\r\n",
					s->regs16[REG_CS], s->reg_ip,
					s->opcode_stream[0],
					s->opcode_stream[1],
					s->opcode_stream[2],
					s->opcode_stream[3]);
#endif
		pc_interrupt_reset(s, 6);
	}
	if (s->xlat_opcode_id != 23
			&& s->xlat_opcode_id != 27
			&& s->xlat_opcode_id != 111) {
		s->rep_override_en = s->seg_override_en = 0;
		if (s->prior_setting_ss)
			s->setting_ss = 0;
		s->prior_setting_ss = s->setting_ss;
	}


	// Increment instruction pointer by computed instruction length. Tables in the BIOS binary
	// help us here.
	s->reg_ip += (s->i_mod*(s->i_mod != 3) + 2*(!s->i_mod && s->i_rm == 6))*s->i_mod_size + bios_table_lookup[TABLE_BASE_INST_SIZE][s->raw_opcode_id] + bios_table_lookup[TABLE_I_W_SIZE][s->raw_opcode_id]*(s->i_w + 1);

	if (s->reset_ip_after_rep_trace) {
		s->reg_ip = s->reg_ip_before_rep_trace;
		s->reset_ip_after_rep_trace = 0;
	}

	// If instruction needs to update SF, ZF and PF, set them as appropriate
	if (s->set_flags_type & FLAGS_UPDATE_SZP)
	{
		s->regs8[FLAG_SF] = SIGN_OF(s->op_result);
		s->regs8[FLAG_ZF] = !s->op_result;
		s->regs8[FLAG_PF] = bios_table_lookup[TABLE_PARITY_FLAG][(unsigned char)s->op_result];

		// If instruction is an arithmetic or logic operation, also set AF/OF/CF as appropriate.
		if (s->set_flags_type & FLAGS_UPDATE_AO_ARITH)
			set_AF_OF_arith(s);
		if (s->set_flags_type & FLAGS_UPDATE_OC_LOGIC)
			set_CF(s, 0), set_OF(s, 0);
	}

	// Poll timer/keyboard every KEYBOARD_TIMER_UPDATE_DELAY instructions
	if (!s->setting_ss &&
			(++s->keyboard_timer_inst_counter >= KEYBOARD_TIMER_UPDATE_DELAY)) {
		s->keyboard_timer_inst_counter = 0;
		s->hlt_this_time = 0;
		s->int8_asap = 1;
	}

#ifndef NO_GRAPHICS
	// Update the video graphics display every GRAPHICS_UPDATE_DELAY instructions
	if (!s->setting_ss &&
			(++s->graphics_inst_counter >= GRAPHICS_UPDATE_DELAY))
	{
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
#endif
}

void x86_handle_hlt(struct x86_state *s)
{
	if (s->hlt_this_time) {
		struct timespec ts;
		ts.tv_sec = 0;
		ts.tv_nsec = HALT_TIME_MICROSECONDS * 1000;
		nanosleep(&ts, NULL);
		s->hlt_this_time = 0;
		s->keyboard_timer_inst_counter += KEYBOARD_TIMER_DELAY_FROM_HALT;
		s->graphics_inst_counter += GRAPHICS_DELAY_FROM_HALT;
	}
}

void x86_handle_irqs(struct x86_state *s)
{
	if (!s->seg_override_en
			&& !s->rep_override_en
			&& s->xlat_opcode_id != 111) {
		s->reg_ip_before_rep_trace = s->reg_ip;

		// Application has set trap flag, so fire INT 1
		if (!s->setting_ss && s->trap_flag)
			pc_interrupt(s, 1);

		s->trap_flag = s->regs8[FLAG_TF];

		// If a timer tick is pending, interrupts are enabled, and no overrides/REP are active,
		// then process the tick and check for new keystrokes
		if (!s->setting_ss && s->int8_asap && s->regs8[FLAG_IF] && !s->regs8[FLAG_TF]) {
			pc_interrupt(s, 0xA);
			s->int8_asap = 0;
			sdl_keyboard_driver(s);
		}
	}
}


/*

8086tiny XMS 2.00 implementation (extended memory functions 8 to 15)
 2019 by C. Masloch

This work is licensed under the MIT License. See included LICENSE.TXT.

Alternatively, this work may be used under these conditions:

Usage of the works is permitted provided that this
instrument is retained with the works, so that any entity
that uses the works is notified of this instrument.

DISCLAIMER: THE WORKS ARE WITHOUT WARRANTY.

*/


struct xmshandle {
	void * allocation;
	uint32_t size;
	uint32_t lockcount;
};
#pragma pack(push, 1)
struct __attribute__((__packed__)) xmsmove {
        uint32_t count;
        uint16_t sourcehandle;
        union {
                uint32_t sourceaddress;
                struct {
                        uint16_t sourceoffset;
                        uint16_t sourcesegment;
                };
        };
        uint16_t desthandle;
        union {
                uint32_t destaddress;
                struct {
                        uint16_t destoffset;
                        uint16_t destsegment;
                };
        };
};
#pragma pack(pop)


struct xmshandle xmshandles[AMOUNT_XMS_HANDLES];
/* Handle 0 in the XMS interface is invalid
 * because it refers to 86 Mode memory for
 * the XMS move handling. Therefore, we check
 * that handles are nonzero and then index
 * into this array with handle number less one
 * so that the first array entry corresponds
 * to handle number 1 from the interface.
 */

uint8_t freexms(unsigned int ii, uint8_t force) {
    	if (ii == 0 || ii > AMOUNT_XMS_HANDLES)
    		return 0xA2;
	if (xmshandles[ii - 1].allocation == NULL)
		return 0xA2;
	if (!force && xmshandles[ii - 1].lockcount != 0)
		return 0xAB;
	free(xmshandles[ii - 1].allocation);
	xmshandles[ii - 1].allocation = NULL;
	xmshandles[ii - 1].size = 0;
	xmshandles[ii - 1].lockcount = 0;
	return 0;
}

void returnxms(struct x86_state *s, uint8_t bl) {
	uint16_t ax = 1;
	if (bl != 0)
		ax = 0;
	s->regs16[REG_AX] = ax;
	s->regs8[REG_BL] = bl;
}

uint8_t checkhandle(struct x86_state *s, uint16_t handle, uint8_t errorcode_handle) {
	if (handle == 0 || handle > AMOUNT_XMS_HANDLES) {
    		returnxms(s, errorcode_handle);
		return 0;
	}
	if (xmshandles[handle - 1].allocation == NULL) {
    		returnxms(s, errorcode_handle);
		return 0;
	}
	return 1;
}

void * checkmoveaddress(
	struct x86_state *s,
	uint16_t handle, uint16_t segment, uint16_t offset,
	uint32_t address, uint32_t count,
	uint8_t errorcode_handle, uint8_t errorcode_address) {
	void * pp;
	if (handle) {
		if (checkhandle(s, handle, errorcode_handle) == 0)
			return NULL;
		if (address + count
			> xmshandles[handle - 1].size) {
			returnxms(s, errorcode_address);
			return NULL;
		}
		return xmshandles[handle - 1].allocation
			+ address;
	} else {
		pp = s->mem + 16 * segment + (unsigned short)(offset);
		if (pp + count
			> (void*)s->mem + RAM_SIZE) {
			returnxms(s, errorcode_address);
			return NULL;
		}
		return pp;
	}
}

uint32_t getfreexms() {
	uint32_t ii, upper, lower;
	void* pp;
	/* Now we use XMS_REPORTED_FREE as a maximum limit for
	 * a binary search, which repeatedly tries to allocate
	 * memory until it has found the maximum available.
	 * That mmeans the reported value is no longer entirely
	 * fabricated. However, it also avoids large allocations.
	 */
	upper = XMS_REPORTED_FREE;	// upper = upper limit, not yet tested
	lower = 0;			// lower = lower limit, tested already
	ii = upper;			// on first iteration, test upper
	for (;;) {
		pp = malloc(ii);	// check whether we can allocate
		free(pp);		// immediately free the allocation
		if (pp)			// pointer still non-NULL if was allocated
			lower = ii;	// valid: is the new lower (tested)
		else
			upper = ii - 1;	// invalid: one less is the new upper (untested)
		if (upper <= lower)	// hardened compare, upper == lower should suffice
			break;		// binary search ended
		ii = lower + (upper - lower + 1) / 2;
		/* upper-lower is at least 1 (upper must be higher, else break)
		 * upper-lower+1 is at least 2
		 * /2 it is at least 1
		 * ii is set to more than lower
		 */
	}
	return lower;
}


void callxms(struct x86_state *s) {
  uint32_t ii, free;
  uint64_t uu64;
  void* pp;
  if (s->regs8[FLAG_CF]) {		// CY, this is an XMS entrypoint call
    set_CF(s, 0);			// signal that we support this
#ifdef XMS_DEBUG
	printf("xms call ax=%04Xh dx=%04Xh bx=%04Xh\r\n",
		s->regs16[REG_AX], s->regs16[REG_DX], s->regs16[REG_BX]);
#endif
    switch(s->regs8[REG_AH]) {
    OPCODE_CHAIN 8:
	free = getfreexms();
	uu64 = 0;
	if (0)
		for (ii = 0; ii < AMOUNT_XMS_HANDLES; ++ii) {
		  uu64 += xmshandles[ii].size;
		}
	uu64 += free;
	uu64 >>= 10;		// rounding down
	if (uu64 > 0xFFFF) {
	  uu64 = 0xFFFF;	// maximum reportable
	}
	s->regs16[REG_DX] = uu64;
	uu64 = free;
	uu64 >>= 10;		// rounding down
	s->regs16[REG_AX] = uu64;
	s->regs8[REG_BL] = 0;
    OPCODE 9:
#ifdef XMS_FAIL_FIRST_ALLOC
	{
		static uint8_t enabled = 0;
		if (! enabled) {
			enabled = 1;
			set_CF(s, 1); return;
			// signal failure to tinyxms
		}
	}
#endif
	uu64 = s->regs16[REG_DX];
	uu64 <<= 10;
	for (ii = 0; ii < AMOUNT_XMS_HANDLES; ++ii)
		if (xmshandles[ii].allocation == NULL)
			break;
	if (ii == AMOUNT_XMS_HANDLES) {
		returnxms(s, 0xA1);
		break;
	}
	++ii;
	pp = calloc(1, uu64);
	if (!pp) {
		returnxms(s, 0xA0);
		break;
	}
	xmshandles[ii - 1].allocation = pp;
	xmshandles[ii - 1].size = uu64;
	xmshandles[ii - 1].lockcount = 0;
	s->regs16[REG_DX] = ii;
	returnxms(s, 0);
    OPCODE 10:
	returnxms(s, freexms(s->regs16[REG_DX], 0));
    OPCODE 11: {
	struct xmsmove *mm = (void*)s->mem + SEGREG(REG_DS, REG_SI,);
	void* psource;
	void* pdest;
	// Odd lengths may be rejected with
	// error code A7h but we support them.
#ifdef XMS_DEBUG
	printf("xmsmove count=%u sourcehandle=%u sourceaddress=%08Xh"
		" desthandle=%u destaddress=%08Xh\r\n",
		mm->count, mm->sourcehandle, mm->sourceaddress,
		mm->desthandle, mm->destaddress);
#endif
	psource = checkmoveaddress(
		s,
		mm->sourcehandle, mm->sourcesegment, mm->sourceoffset,
		mm->sourceaddress, mm->count, 0xA3, 0xA4);
	if (!psource)
		break;
	pdest = checkmoveaddress(
		s,
		mm->desthandle, mm->destsegment, mm->destoffset,
		mm->destaddress, mm->count, 0xA5, 0xA6);
	if (!pdest)
		break;
#ifdef XMS_DEBUG
	printf("xmsmove psource=%08Xh pdest=%08Xh mem=%08Xh\r\n",
		psource, pdest, s->mem);
#endif
	// memmove acts correctly even when memory overlaps.
	// We are allowed to return error A8h if a backwards
	// move is needed, but we may support it too.
	memmove(pdest, psource, mm->count);
	returnxms(s, 0);
	}
    OPCODE 12:
	if (checkhandle(s, s->regs16[REG_DX], 0xA2) == 0)
		break;
	returnxms(s, 0xAD);
    OPCODE 13:
	if (checkhandle(s, s->regs16[REG_DX], 0xA2) == 0)
		break;
	returnxms(s, 0xAA);
    OPCODE 14:
	if (checkhandle(s, s->regs16[REG_DX], 0xA2) == 0)
		break;
	for (ii = 0, uu64 = 0; ii < AMOUNT_XMS_HANDLES; ++ii)
		if (xmshandles[ii].allocation == NULL)
			++uu64;
	s->regs16[REG_AX] = 1;
	if (uu64 > 0xFF)
		uu64 = 0xFF;
	s->regs8[REG_BL] = uu64;
	ii = xmshandles[s->regs16[REG_DX] - 1].lockcount;
	if (ii > 0xFF)
		ii = 0xFF;
	s->regs8[REG_BH] = ii;
	uu64 = xmshandles[s->regs16[REG_DX] - 1].size;
	uu64 >>= 10;
	if (uu64 > 0xFFFF)
		uu64 = 0xFFFF;
	s->regs16[REG_DX] = uu64;
    OPCODE 15:
	if (checkhandle(s, s->regs16[REG_DX], 0xA2) == 0)
		break;
	if (xmshandles[ii - 1].lockcount != 0) {
		returnxms(s, 0xAB);
		break;
	}
	uu64 = s->regs16[REG_BX];
	uu64 <<= 10;
	pp = realloc(xmshandles[s->regs16[REG_DX] - 1].allocation, uu64);
	if (!pp) {
		// original pointer in the handle still valid
		returnxms(s, 0xA0);
		break;
	}
	xmshandles[s->regs16[REG_DX] - 1].allocation = pp;
	ii = xmshandles[s->regs16[REG_DX] - 1].size;
	if (ii < uu64) {
		for (; ii < uu64; ++ii)
			((uint8_t*)pp)[ii] = 0;
	}
	xmshandles[s->regs16[REG_DX] - 1].size = uu64;
	returnxms(s, 0);
	break;
    default:
	returnxms(s, 0x80);	// function not implemented
	break;
    }
  } else {			// NC, this is a different dispatcher
    switch(s->regs16[REG_AX]) {
    OPCODE_CHAIN 0:
      for (ii = 0; ii < AMOUNT_XMS_HANDLES; ++ii) {
	freexms(ii + 1, 1);
      }
      s->regs16[REG_AX] = -1;
    OPCODE 1:
	free = getfreexms();
	uu64 = 0;
	if (1)
		for (ii = 0; ii < AMOUNT_XMS_HANDLES; ++ii) {
		  uu64 += xmshandles[ii].size;
		}
	uu64 += free;
	uu64 >>= 10;		// rounding down
	if (uu64 > 0xFFFF) {
	  uu64 = 0xFFFF;	// maximum reportable
	}
	s->regs16[REG_AX] = uu64;
	set_CF(s, 1);		// this means no error here
    }
  }
}
