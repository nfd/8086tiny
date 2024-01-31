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
#define DECODE_RM_REG scratch2_uint = 4 * !i_mod, \
	op_to_addr = rm_addr = i_mod < 3 ? SEGREG(seg_override_en ? seg_override : bios_table_lookup[scratch2_uint + 3][i_rm], bios_table_lookup[scratch2_uint][i_rm], regs16[bios_table_lookup[scratch2_uint + 1][i_rm]] + bios_table_lookup[scratch2_uint + 2][i_rm] * i_data1+) : GET_REG_ADDR(i_rm), \
	op_from_addr = GET_REG_ADDR(i_reg), \
	i_d && (scratch_uint = op_from_addr, op_from_addr = rm_addr, op_to_addr = scratch_uint)

// Return memory-mapped register location (offset into mem array) for register #reg_id
#define GET_REG_ADDR(reg_id) (REGS_BASE + (i_w ? 2 * reg_id : 2 * reg_id + reg_id / 4 & 7))

// Returns number of top bit in operand (i.e. 8 for 8-bit operands, 16 for 16-bit operands)
#define TOP_BIT 8*(i_w + 1)

// Opcode execution unit helpers
#define OPCODE ;break; case
#define OPCODE_CHAIN ; case

// [I]MUL/[I]DIV/DAA/DAS/ADC/SBB helpers
#define MUL_MACRO(op_data_type,out_regs) (set_opcode(0x10), \
	out_regs[i_w + 1] = (op_result = CAST(op_data_type)mem[rm_addr] * (op_data_type)*out_regs) >> 16, \
	regs16[REG_AX] = op_result, \
	set_OF(set_CF(op_result - (op_data_type)op_result)))
#define DIV_MACRO(out_data_type,in_data_type,out_regs) (scratch_int = CAST(out_data_type)mem[rm_addr]) && !(scratch2_uint = (in_data_type)(scratch_uint = (out_regs[i_w+1] << 16) + regs16[REG_AX]) / scratch_int, scratch2_uint - (out_data_type)scratch2_uint) ? out_regs[i_w+1] = scratch_uint - scratch_int * (*out_regs = scratch2_uint) : pc_interrupt_reset(0)
#define DAA_DAS(op1,op2) \
	set_AF((((scratch_uchar = regs8[REG_AL]) & 0x0F) > 9) || regs8[FLAG_AF]) && (op_result = (regs8[REG_AL] op1 6), set_CF(regs8[FLAG_CF] || (regs8[REG_AL] op2 scratch_uchar))), \
	set_CF((regs8[REG_AL] > 0x9f) || regs8[FLAG_CF]) && (op_result = (regs8[REG_AL] op1 0x60))
#define ADC_SBB_MACRO(a) OP(a##= regs8[FLAG_CF] +), \
						 set_CF(regs8[FLAG_CF] && (op_result == op_dest) || (a op_result < a(int)op_dest)), \
						 set_AF_OF_arith()

// Execute arithmetic/logic operations in emulator memory/registers
#define R_M_OP(dest,op,src) (i_w ? op_dest = CAST(unsigned short)dest, op_result = CAST(unsigned short)dest op (op_source = CAST(unsigned short)src) \
								 : (op_dest = dest, op_result = dest op (op_source = CAST(unsigned char)src)))
#define R_M_OP_64_EQUALS(dest,op,src) (i_w ? op_dest = CAST(unsigned short)dest, op_result = CAST(unsigned short)dest = (uint64_t)CAST(unsigned short)dest op (uint64_t)(op_source = CAST(unsigned short)src) \
								 : (op_dest = dest, op_result = dest = (uint64_t)dest op (uint64_t)(op_source = CAST(unsigned char)src)))
#define MEM_OP(dest,op,src) R_M_OP(mem[dest],op,mem[src])
#define OP(op) MEM_OP(op_to_addr,op,op_from_addr)

// Increment or decrement a register #reg_id (usually SI or DI), depending on direction flag and operand size (given by i_w)
#define INDEX_INC(reg_id) (regs16[reg_id] -= (2 * regs8[FLAG_DF] - 1)*(i_w + 1))

// Helpers for stack operations
#define R_M_PUSH(a) (i_w = 1, R_M_OP(mem[SEGREG(REG_SS, REG_SP, --)], =, a))
#define R_M_POP(a) (i_w = 1, regs16[REG_SP] += 2, R_M_OP(a, =, mem[SEGREG(REG_SS, REG_SP, -2+)]))

// Convert segment:offset to linear address in emulator memory space
#define SEGREG(reg_seg,reg_ofs,op) 16 * regs16[reg_seg] + (unsigned short)(op regs16[reg_ofs])

// Returns sign bit of an 8-bit or 16-bit operand
#define SIGN_OF(a) (1 & (i_w ? CAST(short)a : a) >> (TOP_BIT - 1))

// Reinterpretation cast
#define CAST(a) *(a*)&

// Keyboard driver for console. This may need changing for UNIX/non-UNIX platforms
#ifdef _WIN32
#define KEYBOARD_DRIVER kbhit() && (mem[0x4A6] = getch(), pc_interrupt(7))
#else
#define KEYBOARD_DRIVER read(0, mem + 0x4A6, 1) && (int8_asap = (mem[0x4A6] == 0x1B), pc_interrupt(7))
#endif

// Keyboard driver for SDL
#ifdef NO_GRAPHICS
#define SDL_KEYBOARD_DRIVER KEYBOARD_DRIVER
#else
#define SDL_KEYBOARD_DRIVER sdl_screen ? SDL_PollEvent(&sdl_event) && (sdl_event.type == SDL_KEYDOWN || sdl_event.type == SDL_KEYUP) && (scratch_uint = sdl_event.key.keysym.unicode, scratch2_uint = sdl_event.key.keysym.mod, CAST(short)mem[0x4A6] = 0x400 + 0x800*!!(scratch2_uint & KMOD_ALT) + 0x1000*!!(scratch2_uint & KMOD_SHIFT) + 0x2000*!!(scratch2_uint & KMOD_CTRL) + 0x4000*(sdl_event.type == SDL_KEYUP) + ((!scratch_uint || scratch_uint > 0x7F) ? sdl_event.key.keysym.sym : scratch_uint), pc_interrupt(7)) : (KEYBOARD_DRIVER)
#endif

// Global variable definitions
unsigned char mem[RAM_SIZE + 16], io_ports[IO_PORT_COUNT + 16],
	*opcode_stream, *regs8, i_rm, i_w, i_reg, i_mod, i_mod_size, i_d, i_reg4bit,
	raw_opcode_id, xlat_opcode_id, extra, rep_mode, seg_override_en, rep_override_en,
	trap_flag, int8_asap, scratch_uchar, io_hi_lo, *vid_mem_base, spkr_en, bios_table_lookup[20][256],
	hlt_this_time, setting_ss, prior_setting_ss, reset_ip_after_rep_trace,
	shift_count;
unsigned short *regs16, reg_ip, seg_override, file_index, wave_counter, reg_ip_before_rep_trace;
unsigned int op_source, op_dest, rm_addr, op_to_addr, op_from_addr, i_data0, i_data1, i_data2, scratch_uint, scratch2_uint, keyboard_timer_inst_counter, graphics_inst_counter, set_flags_type, GRAPHICS_X, GRAPHICS_Y, pixel_colors[16], vmem_ctr;
int op_result, scratch_int;
time_t clock_buf;
struct timeb ms_clock;

#ifndef NO_GRAPHICS
SDL_AudioSpec sdl_audio = {44100, AUDIO_U8, 1, 0, 128};
SDL_Surface *sdl_screen;
SDL_Event sdl_event;
unsigned short vid_addr_lookup[VIDEO_RAM_SIZE], cga_colors[4] = {0 /* Black */, 0x1F1F /* Cyan */, 0xE3E3 /* Magenta */, 0xFFFF /* White */};
#endif

// Helper functions

void callxms();

// Set carry flag
char set_CF(int new_CF)
{
	return regs8[FLAG_CF] = !!new_CF;
}

// Set auxiliary flag
char set_AF(int new_AF)
{
	return regs8[FLAG_AF] = !!new_AF;
}

// Set overflow flag
char set_OF(int new_OF)
{
	return regs8[FLAG_OF] = !!new_OF;
}

// Set auxiliary and overflow flag after arithmetic operations
char set_AF_OF_arith()
{
	set_AF((op_source ^= op_dest ^ op_result) & 0x10);
	if (op_result == op_dest)
		return set_OF(0);
	else
		return set_OF(1 & (regs8[FLAG_CF] ^ op_source >> (TOP_BIT - 1)));
}

// Assemble and return emulated CPU FLAGS register in scratch_uint
void make_flags()
{
	scratch_uint = 0xF002; // 8086 has reserved and unused flags set to 1
	for (int i = 9; i--;)
		scratch_uint += regs8[FLAG_CF + i] << bios_table_lookup[TABLE_FLAGS_BITFIELDS][i];
}

// Set emulated CPU FLAGS register from regs8[FLAG_xx] values
void set_flags(int new_flags)
{
	for (int i = 9; i--;)
		regs8[FLAG_CF + i] = !!(1 << bios_table_lookup[TABLE_FLAGS_BITFIELDS][i] & new_flags);
}

// Convert raw opcode to translated opcode index. This condenses a large number of different encodings of similar
// instructions into a much smaller number of distinct functions, which we then execute
void set_opcode(unsigned char opcode)
{
	xlat_opcode_id = bios_table_lookup[TABLE_XLAT_OPCODE][raw_opcode_id = opcode];
	extra = bios_table_lookup[TABLE_XLAT_SUBFUNCTION][opcode];
	i_mod_size = bios_table_lookup[TABLE_I_MOD_SIZE][opcode];
	set_flags_type = bios_table_lookup[TABLE_STD_FLAGS][opcode];
}

// Execute INT #interrupt_num on the emulated machine
char pc_interrupt(unsigned char interrupt_num)
{
	set_opcode(0xCD); // Decode like INT

	make_flags();
	R_M_PUSH(scratch_uint);
	R_M_PUSH(regs16[REG_CS]);
	R_M_PUSH(reg_ip);
	MEM_OP(REGS_BASE + 2 * REG_CS, =, 4 * interrupt_num + 2);
	R_M_OP(reg_ip, =, mem[4 * interrupt_num]);

	trap_flag = 0;
	return regs8[FLAG_TF] = regs8[FLAG_IF] = 0;
}

// Execute interrupt but reset ip
char pc_interrupt_reset(unsigned char interrupt_num)
{
	reg_ip = reg_ip_before_rep_trace;
	return pc_interrupt(interrupt_num);
}


// AAA and AAS instructions - which_operation is +1 for AAA, and -1 for AAS
int AAA_AAS(char which_operation)
{
	return (regs16[REG_AX] += 262 * which_operation*set_AF(set_CF(((regs8[REG_AL] & 0x0F) > 9) || regs8[FLAG_AF])), regs8[REG_AL] &= 0x0F);
}

#ifndef NO_GRAPHICS
void audio_callback(void *data, unsigned char *stream, int len)
{
	for (int i = 0; i < len; i++)
		stream[i] = (spkr_en == 3) && CAST(unsigned short)mem[0x4AA] ? -((54 * wave_counter++ / CAST(unsigned short)mem[0x4AA]) & 1) : sdl_audio.silence;

	spkr_en = io_ports[0x61] & 3;
}
#endif

void callin(uint8_t* dest, uint16_t port) {
	io_ports[0x20] = 0; // PIC EOI
	io_ports[0x42] = --io_ports[0x40]; // PIT channel 0/2 read placeholder
	io_ports[0x3DA] ^= 9; // CGA refresh
	port == 0x60 && (io_ports[0x64] = 0); // Scancode read flag
	port == 0x3D5 && (io_ports[0x3D4] >> 1 == 7) && (io_ports[0x3D5] = ((mem[0x49E]*80 + mem[0x49D] + CAST(short)mem[0x4AD]) & (io_ports[0x3D4] & 1 ? 0xFF : 0xFF00)) >> (io_ports[0x3D4] & 1 ? 0 : 8)); // CRT cursor position
	R_M_OP(*dest, =, io_ports[port]);
}

void callout(uint8_t* source, uint16_t port) {
	R_M_OP(io_ports[port], =, *source);
	port == 0x61 && (io_hi_lo = 0, spkr_en |= *source & 3); // Speaker control
	(port == 0x40 || port == 0x42) && (io_ports[0x43] & 6) && (mem[0x469 + port - (io_hi_lo ^= 1)] = *source); // PIT rate programming
#ifndef NO_GRAPHICS
	port == 0x43 && (io_hi_lo = 0, *source >> 6 == 2) && (SDL_PauseAudio((*source & 0xF7) != 0xB6), 0); // Speaker enable
#endif
	port == 0x3D5 && (io_ports[0x3D4] >> 1 == 6) && (mem[0x4AD + !(io_ports[0x3D4] & 1)] = *source); // CRT video RAM start offset
	port == 0x3D5 && (io_ports[0x3D4] >> 1 == 7) && (scratch2_uint = ((mem[0x49E]*80 + mem[0x49D] + CAST(short)mem[0x4AD]) & (io_ports[0x3D4] & 1 ? 0xFF00 : 0xFF)) + (*source << (io_ports[0x3D4] & 1 ? 0 : 8)) - CAST(short)mem[0x4AD], mem[0x49D] = scratch2_uint % 80, mem[0x49E] = scratch2_uint / 80); // CRT cursor position
	port == 0x3B5 && io_ports[0x3B4] == 1 && (GRAPHICS_X = *source * 16); // Hercules resolution reprogramming. Defaults are set in the BIOS
	port == 0x3B5 && io_ports[0x3B4] == 6 && (GRAPHICS_Y = *source * 4);
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

struct x86_state *x86_init(int boot_from_hdd, char *bios_filename, char *fdd_filename, char *hdd_filename, void(*redraw_display)(struct x86_state *), void(*keyboard_driver)(struct x86_state *)) {
#ifndef NO_GRAPHICS
	// Initialise SDL
	SDL_Init(SDL_INIT_AUDIO);
	sdl_audio.callback = audio_callback;
#ifdef _WIN32
	sdl_audio.samples = 512;
#endif
	SDL_OpenAudio(&sdl_audio, 0);
#endif

	struct x86_state *s = calloc(1, sizeof(struct x86_state));

	// regs16 and reg8 point to F000:0, the start of memory-mapped registers. CS is initialised to F000
	regs16 = (unsigned short *)(regs8 = mem + REGS_BASE);
	regs16[REG_CS] = 0xF000;

	// Trap flag off
	regs8[FLAG_TF] = 0;

	// Set DL equal to the boot device: 0 for the FD, or 0x80 for the HD. Normally, boot from the FD.
	// But, if the HD image file is prefixed with @, then boot from the HD
	regs8[REG_DL] = boot_from_hdd ? 0x80 : 0;

	// Open BIOS (file id disk[2]), floppy disk image (disk[1]), and hard disk image (disk[0]) if specified
	if(bios_filename) s->disk_bios = open(bios_filename, 32898);
	if(fdd_filename) s->disk_fdd = open(fdd_filename, 32898);
	if(hdd_filename) s->disk_hdd = open(hdd_filename, 32898);

	// Set CX:AX equal to the hard disk image size, if present
	CAST(unsigned)regs16[REG_AX] = s->disk_hdd ? lseek(s->disk_hdd, 0, 2) >> 9 : 0;

	// Load BIOS image into F000:0100, and set IP to 0100
	read(s->disk_bios, regs8 + (reg_ip = reg_ip_before_rep_trace = 0x100), 0xFF00);

	// Load instruction decoding helper table
	for (int i = 0; i < 20; i++)
		for (int j = 0; j < 256; j++)
			bios_table_lookup[i][j] = regs8[regs16[0x81 + i] + j];

	return s;
}

// Emulator entry point
void x86_step(struct x86_state *s)
{
#ifdef XMS_DEBUG
	if (opcode_stream[0] == 0xEB &&
			opcode_stream[1] == 0x3 &&
			opcode_stream[2] == 0x90 &&
			opcode_stream[3] == 0x90 &&
			opcode_stream[4] == 0x90) {
		printf("xms call ax=%04Xh dx=%04Xh bx=%04Xh\r\n",
				regs16[REG_AX], regs16[REG_DX], regs16[REG_BX]);
	}
#endif
	setting_ss = 0;

	// Set up variables to prepare for decoding an opcode
	set_opcode(*opcode_stream);

	// Extract i_w and i_d fields from instruction
	i_w = (i_reg4bit = raw_opcode_id & 7) & 1;
	i_d = i_reg4bit / 2 & 1;

	// Extract instruction data fields
	i_data0 = CAST(short)opcode_stream[1];
	shift_count = i_data1 = CAST(short)opcode_stream[2];
	i_data2 = CAST(short)opcode_stream[3];

	// seg_override_en and rep_override_en contain number of instructions to hold segment override and REP prefix respectively
	if (seg_override_en)
		seg_override_en--;
	if (rep_override_en)
		rep_override_en--;

	// i_mod_size > 0 indicates that opcode uses i_mod/i_rm/i_reg, so decode them
	if (i_mod_size)
	{
		i_mod = (i_data0 & 0xFF) >> 6;
		i_rm = i_data0 & 7;
		i_reg = i_data0 / 8 & 7;

		if ((!i_mod && i_rm == 6) || (i_mod == 2)) {
			i_data2 = CAST(short)opcode_stream[4];
			shift_count = opcode_stream[4];
		} else if (i_mod != 1) {
			i_data2 = i_data1;
		} else { // If i_mod is 1, operand is (usually) 8 bits rather than 16 bits
			i_data1 = (char)i_data1;
			shift_count = opcode_stream[3];
		}

		DECODE_RM_REG;
	}

	// Instruction execution unit
	switch (xlat_opcode_id)
	{
		OPCODE_CHAIN 0: // Conditional jump (JAE, JNAE, etc.)
						// i_w is the invert flag, e.g. i_w == 1 means JNAE, whereas i_w == 0 means JAE
			scratch_uchar = raw_opcode_id / 2 & 7;
		reg_ip += (char)i_data0 * (i_w ^ (regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_A][scratch_uchar]] || regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_B][scratch_uchar]] || regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_C][scratch_uchar]] ^ regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_D][scratch_uchar]]))
			OPCODE 1: // MOV reg, imm
			i_w = !!(raw_opcode_id & 8);
		R_M_OP(mem[GET_REG_ADDR(i_reg4bit)], =, i_data0)
			OPCODE 3: // PUSH regs16
			R_M_PUSH(regs16[i_reg4bit])
			OPCODE 4: // POP regs16
			R_M_POP(regs16[i_reg4bit])
			OPCODE 2: // INC|DEC regs16
			i_w = 1;
		i_d = 0;
		i_reg = i_reg4bit;
		DECODE_RM_REG;
		i_reg = extra
			OPCODE_CHAIN 5: // INC|DEC|JMP|CALL|PUSH
			if (i_reg < 2) // INC|DEC
				MEM_OP(op_from_addr, += 1 - 2 * i_reg +, REGS_BASE + 2 * REG_ZERO),
					op_source = 1,
					set_AF_OF_arith(),
					set_OF(op_dest + 1 - i_reg == 1 << (TOP_BIT - 1)),
					(xlat_opcode_id == 5) && (set_opcode(0x10), 0); // Decode like ADC
			else if (i_reg != 6) // JMP|CALL
				i_reg - 3 || R_M_PUSH(regs16[REG_CS]), // CALL (far)
					  i_reg & 2 && R_M_PUSH(reg_ip + 2 + i_mod*(i_mod != 3) + 2*(!i_mod && i_rm == 6)), // CALL (near or far)
					  i_reg & 1 && (regs16[REG_CS] = CAST(short)mem[op_from_addr + 2]), // JMP|CALL (far)
					  R_M_OP(reg_ip, =, mem[op_from_addr]),
					  set_opcode(0x9A); // Decode like CALL
			else // PUSH
				R_M_PUSH(mem[rm_addr])
					OPCODE 6: // TEST r/m, imm16 / NOT|NEG|MUL|IMUL|DIV|IDIV reg
					op_to_addr = op_from_addr;

		switch (i_reg)
		{
			OPCODE_CHAIN 0: // TEST
				set_opcode(0x20); // Decode like AND
			reg_ip += i_w + 1;
			R_M_OP(mem[op_to_addr], &, i_data2)
				OPCODE 2: // NOT
				OP(=~)
				OPCODE 3: // NEG
				OP(=-);
			op_dest = 0;
			set_opcode(0x28); // Decode like SUB
			set_CF(op_result > op_dest)
				OPCODE 4: // MUL
				i_w ? MUL_MACRO(unsigned short, regs16) : MUL_MACRO(unsigned char, regs8)
				OPCODE 5: // IMUL
				i_w ? MUL_MACRO(short, regs16) : MUL_MACRO(char, regs8)
				OPCODE 6: // DIV
				i_w ? DIV_MACRO(unsigned short, unsigned, regs16) : DIV_MACRO(unsigned char, unsigned short, regs8)
				OPCODE 7: // IDIV
				i_w ? DIV_MACRO(short, int, regs16) : DIV_MACRO(char, short, regs8);
		}
		OPCODE 7: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP AL/AX, immed
			rm_addr = REGS_BASE;
		i_data2 = i_data0;
		i_mod = 3;
		i_reg = extra;
		reg_ip--;
		OPCODE_CHAIN 8: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP reg, immed
			op_to_addr = rm_addr;
		regs16[REG_SCRATCH] = (i_d |= !i_w) ? (char)i_data2 : i_data2;
		op_from_addr = REGS_BASE + 2 * REG_SCRATCH;
		reg_ip += !i_d + 1;
		set_opcode(0x08 * (extra = i_reg));
		OPCODE_CHAIN 9: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP|MOV reg, r/m
			switch (extra)
			{
				OPCODE_CHAIN 0: // ADD
					OP(+=),
					set_CF(op_result < op_dest)
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
					set_CF(op_result > op_dest)
						OPCODE 6: // XOR
						OP(^=)
						OPCODE 7: // CMP
						OP(-),
					set_CF(op_result > op_dest)
						OPCODE 8: // MOV
						OP(=);
			}
		OPCODE 10: // MOV sreg, r/m | POP r/m | LEA reg, r/m
			if (!i_w) { // MOV
				i_w = 1;
				i_reg += 8;
				DECODE_RM_REG;
				if (i_d && (op_to_addr == REGS_BASE + 2 * REG_CS
							|| op_to_addr > REGS_BASE + 2 * REG_DS)) {
					pc_interrupt_reset(6);
					break;
				}
				if (!i_d && (op_from_addr > REGS_BASE + 2 * REG_DS)) {
					pc_interrupt_reset(6);
					break;
				}
				OP(=);
				if (i_d && op_to_addr == REGS_BASE + 2 * REG_SS)
					setting_ss = 1;
			} else if (!i_d) // LEA
				seg_override_en = 1,
								seg_override = REG_ZERO,
								DECODE_RM_REG,
								R_M_OP(mem[op_from_addr], =, rm_addr);
			else // POP
				R_M_POP(mem[rm_addr])
					OPCODE 11: // MOV AL/AX, [loc]
					i_mod = i_reg = 0;
		i_rm = 6;
		i_data1 = i_data0;
		DECODE_RM_REG;
		MEM_OP(op_from_addr, =, op_to_addr)
			OPCODE 12: // ROL|ROR|RCL|RCR|SHL|SHR|???|SAR reg/mem, 1/CL/imm (80186)
			scratch2_uint = SIGN_OF(mem[rm_addr]),
						  scratch_uint = extra ? // xxx reg/mem, imm
							  MASK_SHIFT_COUNT(shift_count)
							  : // xxx reg/mem, CL
							  i_d
							  ? MASK_SHIFT_COUNT(regs8[REG_CL])
							  : // xxx reg/mem, 1
							  1;
		if (scratch_uint)
		{
			if (i_reg < 4) // Rotate operations
				scratch_uint %= i_reg / 2 + TOP_BIT,
							 R_M_OP(scratch2_uint, =, mem[rm_addr]);
			if (i_reg & 1) // Rotate/shift right operations
				R_M_OP_64_EQUALS(mem[rm_addr], >>, scratch_uint);
			else // Rotate/shift left operations
				R_M_OP_64_EQUALS(mem[rm_addr], <<, scratch_uint);
			if (i_reg > 3) // Shift operations
				set_flags_type = FLAGS_UPDATE_SZP; // Shift instructions affect SZP
			if (i_reg > 4) // SHR or SAR
				set_CF((uint64_t)op_dest >> (scratch_uint - 1) & 1);
		}

		switch (i_reg)
		{
			OPCODE_CHAIN 0: // ROL
				R_M_OP_64_EQUALS(mem[rm_addr], + , scratch2_uint >> (TOP_BIT - scratch_uint));
			set_OF(SIGN_OF(op_result) ^ set_CF(op_result & 1))
				OPCODE 1: // ROR
				scratch2_uint &= (1 << scratch_uint) - 1,
							  R_M_OP_64_EQUALS(mem[rm_addr], + , scratch2_uint << (TOP_BIT - scratch_uint));
			set_OF(SIGN_OF(op_result * 2) ^ set_CF(SIGN_OF(op_result)))
				OPCODE 2: // RCL
				R_M_OP_64_EQUALS(mem[rm_addr], + ((uint64_t)regs8[FLAG_CF] << (scratch_uint - 1)) + , scratch2_uint >> (1 + TOP_BIT - scratch_uint));
			set_OF(SIGN_OF(op_result) ^ set_CF(scratch2_uint & (uint64_t)1 << (TOP_BIT - scratch_uint)))
				OPCODE 3: // RCR
				R_M_OP_64_EQUALS(mem[rm_addr], + ((uint64_t)regs8[FLAG_CF] << (TOP_BIT - scratch_uint)) + , scratch2_uint << (1 + TOP_BIT - scratch_uint));
			set_CF(scratch2_uint & (uint64_t)1 << (scratch_uint - 1));
			set_OF(SIGN_OF(op_result) ^ SIGN_OF(op_result * 2))
				OPCODE 4: // SHL
				set_OF(SIGN_OF(op_result) ^ set_CF(SIGN_OF(op_dest << (scratch_uint - 1))))
				OPCODE 5: // SHR
				set_OF(SIGN_OF(op_dest))
				OPCODE 7: // SAR
				scratch_uint < TOP_BIT || set_CF(scratch2_uint);
			set_OF(0);
			R_M_OP_64_EQUALS(mem[rm_addr], +, scratch2_uint *= ~((uint64_t)(((uint64_t)1 << TOP_BIT) - 1) >> scratch_uint));
		}
		OPCODE 13: // LOOPxx|JCZX
			scratch_uint = !!--regs16[REG_CX];

		switch(i_reg4bit)
		{
			OPCODE_CHAIN 0: // LOOPNZ
				scratch_uint &= !regs8[FLAG_ZF]
				OPCODE 1: // LOOPZ
				scratch_uint &= regs8[FLAG_ZF]
				OPCODE 3: // JCXXZ
				scratch_uint = !++regs16[REG_CX];
		}
		reg_ip += scratch_uint*(char)i_data0
			OPCODE 14: // JMP | CALL short/near
			reg_ip += 3 - i_d;
		if (!i_w)
		{
			if (i_d) // JMP far
				reg_ip = 0,
					   regs16[REG_CS] = i_data2;
			else // CALL
				R_M_PUSH(reg_ip);
		}
		reg_ip += i_d && i_w ? (char)i_data0 : i_data0
			OPCODE 15: // TEST reg, r/m
			MEM_OP(op_from_addr, &, op_to_addr)
			OPCODE 16: // XCHG AX, regs16
			i_w = 1;
		op_to_addr = REGS_BASE;
		op_from_addr = GET_REG_ADDR(i_reg4bit);
		OPCODE_CHAIN 24: // NOP|XCHG reg, r/m
			if (op_to_addr != op_from_addr)
				OP(^=),
					MEM_OP(op_from_addr, ^=, op_to_addr),
					OP(^=)
						OPCODE 17: // MOVSx (extra=0)|STOSx (extra=1)|LODSx (extra=2)
						scratch2_uint = seg_override_en ? seg_override : REG_DS;

		scratch_uint = rep_override_en ? regs16[REG_CX] : 1;
		if (trap_flag && scratch_uint > 1) {
			reset_ip_after_rep_trace = 1;
			scratch_uint = 1;
		}
		for (; scratch_uint; scratch_uint--)
		{
			MEM_OP(extra < 2 ? SEGREG(REG_ES, REG_DI,) : REGS_BASE, =, extra & 1 ? REGS_BASE : SEGREG(scratch2_uint, REG_SI,)),
				extra & 1 || INDEX_INC(REG_SI),
				extra & 2 || INDEX_INC(REG_DI);
			rep_override_en && regs16[REG_CX]--;
		}
		OPCODE 18: // CMPSx (extra=0)|SCASx (extra=1)
			scratch2_uint = seg_override_en ? seg_override : REG_DS;

		scratch_uint = rep_override_en ? regs16[REG_CX] : 1;
		if (trap_flag && scratch_uint > 1) {
			reset_ip_after_rep_trace = 1;
			scratch_uint = 1;
		}
		if (scratch_uint)
		{
			for (; scratch_uint; rep_override_en || scratch_uint--)
			{
				MEM_OP(extra ? REGS_BASE : SEGREG(scratch2_uint, REG_SI,), -, SEGREG(REG_ES, REG_DI,));
				extra || INDEX_INC(REG_SI);
				INDEX_INC(REG_DI);
				if (rep_override_en) {
					if (!(--regs16[REG_CX] && (!op_result == rep_mode))) {
						(scratch_uint = 0, reset_ip_after_rep_trace = 0);
					} else if (trap_flag) {
						scratch_uint = 0;
					}
				}
			}

			set_flags_type = FLAGS_UPDATE_SZP | FLAGS_UPDATE_AO_ARITH; // Funge to set SZP/AO flags
			set_CF(op_result > op_dest);
		}
		OPCODE 19: // RET|RETF|IRET
			i_d = i_w;
		R_M_POP(reg_ip);
		if (extra) // IRET|RETF|RETF imm16
			R_M_POP(regs16[REG_CS]);
		if (extra & 2) // IRET
			set_flags(R_M_POP(scratch_uint));
		else if (!i_d) // RET|RETF imm16
			regs16[REG_SP] += i_data0
				OPCODE 20: // MOV r/m, immed
				R_M_OP(mem[op_from_addr], =, i_data2)
				OPCODE 21: // IN AL/AX, DX/imm8
				scratch_uint = extra ? regs16[REG_DX] : (uint8_t)i_data0;
		callin(&regs8[REG_AL], scratch_uint);
		OPCODE 22: // OUT DX/imm8, AL/AX
			scratch_uint = extra ? regs16[REG_DX] : (uint8_t)i_data0;
		callout(&regs8[REG_AL], scratch_uint);
		OPCODE 23: // REPxx
			rep_override_en = 2;
		rep_mode = i_w;
		seg_override_en && seg_override_en++
			OPCODE 25: // PUSH reg
			R_M_PUSH(regs16[extra])
			OPCODE 26: // POP reg
			R_M_POP(regs16[extra]);
		if ((void*)&regs16[extra] == (void*)&mem[REGS_BASE + 2 * REG_SS])
			setting_ss = 1;
		OPCODE 27: // xS: segment overrides
			seg_override_en = 2;
		seg_override = extra;
		rep_override_en && rep_override_en++
			OPCODE 28: // DAA/DAS
			i_w = 0;
		if (extra) DAA_DAS(-=, >); else DAA_DAS(+=, <) // extra = 0 for DAA, 1 for DAS
			OPCODE 29: // AAA/AAS
				op_result = AAA_AAS(extra - 1)
				OPCODE 30: // CBW
				regs8[REG_AH] = -SIGN_OF(regs8[REG_AL])
				OPCODE 31: // CWD
				regs16[REG_DX] = -SIGN_OF(regs16[REG_AX])
				OPCODE 32: // CALL FAR imm16:imm16
				R_M_PUSH(regs16[REG_CS]);
		R_M_PUSH(reg_ip + 5);
		regs16[REG_CS] = i_data2;
		reg_ip = i_data0
			OPCODE 33: // PUSHF
			make_flags();
		R_M_PUSH(scratch_uint)
			OPCODE 34: // POPF
			set_flags(R_M_POP(scratch_uint))
			OPCODE 35: // SAHF
			make_flags();
		set_flags((scratch_uint & 0xFF00) + regs8[REG_AH])
			OPCODE 36: // LAHF
			make_flags(),
			regs8[REG_AH] = scratch_uint
				OPCODE 37: // LES|LDS reg, r/m
				if (i_mod == 3) {
					pc_interrupt_reset(6);
					break;
				}
		i_w = i_d = 1;
		DECODE_RM_REG;
		OP(=);
		MEM_OP(REGS_BASE + extra, =, rm_addr + 2)
			OPCODE 38: // INT 3
			++reg_ip;
		pc_interrupt(3)
			OPCODE 39: // INT imm8
			reg_ip += 2;
		pc_interrupt(i_data0)
			OPCODE 40: // INTO
			++reg_ip;
		regs8[FLAG_OF] && pc_interrupt(4)
			OPCODE 41: // AAM
			if (i_data0 &= 0xFF)
				regs8[REG_AH] = regs8[REG_AL] / i_data0,
					op_result = regs8[REG_AL] %= i_data0;
			else // Divide by zero
				pc_interrupt_reset(0)
					OPCODE 42: // AAD
					i_w = 0;
		regs16[REG_AX] = op_result = 0xFF & regs8[REG_AL] + i_data0 * regs8[REG_AH]
			OPCODE 43: // SALC
			regs8[REG_AL] = -regs8[FLAG_CF]
			OPCODE 44: // XLAT
			regs8[REG_AL] = mem[SEGREG(seg_override_en ? seg_override : REG_DS, REG_BX, regs8[REG_AL] +)]
			OPCODE 45: // CMC
			regs8[FLAG_CF] ^= 1
			OPCODE 46: // CLC|STC|CLI|STI|CLD|STD
			regs8[extra / 2] = extra & 1
			OPCODE 47: // TEST AL/AX, immed
			R_M_OP(regs8[REG_AL], &, i_data0)
			OPCODE 48: // Emulator-specific 0F xx opcodes
			switch ((char)i_data0)
			{
				OPCODE_CHAIN 0: { // PUTCHAR_AL
					unsigned char buf[3];
					unsigned len;

					len = code_to_utf8(buf, cp437_to_utf16[*regs8]);
					write(1, buf, len);
				}
				OPCODE 1: // GET_RTC
					time(&clock_buf);
				ftime(&ms_clock);
				memcpy(mem + SEGREG(REG_ES, REG_BX,), localtime(&clock_buf), sizeof(struct tm));
				CAST(short)mem[SEGREG(REG_ES, REG_BX, 36+)] = ms_clock.millitm;
				OPCODE 2: // DISK_READ
				OPCODE_CHAIN 3: // DISK_WRITE
				{
					int disk = 0;
					switch(regs8[REG_DL]) {
						case 2: disk = s->disk_bios; break;
						case 1: disk = s->disk_fdd; break;
						case 0: disk = s->disk_hdd; break;
					}

					regs8[REG_AL] = ~lseek(disk, CAST(unsigned)regs16[REG_BP] << 9, 0)
					? ((char)i_data0 == 3 ? (int(*)())write : (int(*)())read)(disk, mem + SEGREG(REG_ES, REG_BX,), regs16[REG_AX])
					: 0;
				}
				OPCODE 4:	// XMS
					callxms();
			}
		if ((uint8_t)i_data0 == 11	// ud2
				|| (uint8_t)i_data0 >= 32)
			pc_interrupt_reset(6);
		OPCODE 100: // HLT
			hlt_this_time = 1;
		OPCODE 101: // PUSH imm16
			R_M_PUSH(i_data0);
		OPCODE 102: // PUSH imm8
			scratch_uint = (int16_t)(int8_t)i_data0;
		R_M_PUSH(scratch_uint);
		OPCODE 103: // FPU instructions - no ops
			OPCODE 104: // PUSHA
			scratch_uint = regs16[REG_SP];
		R_M_PUSH(regs16[REG_AX]);
		R_M_PUSH(regs16[REG_CX]);
		R_M_PUSH(regs16[REG_DX]);
		R_M_PUSH(regs16[REG_BX]);
		R_M_PUSH(scratch_uint);
		R_M_PUSH(regs16[REG_BP]);
		R_M_PUSH(regs16[REG_SI]);
		R_M_PUSH(regs16[REG_DI]);
		OPCODE 105: // POPA
			R_M_POP(regs16[REG_DI]);
		R_M_POP(regs16[REG_SI]);
		R_M_POP(regs16[REG_BP]);
		R_M_POP(scratch_uint);
		R_M_POP(regs16[REG_BX]);
		R_M_POP(regs16[REG_DX]);
		R_M_POP(regs16[REG_CX]);
		R_M_POP(regs16[REG_AX]);
		OPCODE 106: // ENTER
			R_M_PUSH(regs16[REG_BP]);
		scratch_uint = regs16[REG_SP];
		scratch_int = (uint8_t)i_data2 & 31;
		if (scratch_int > 0) {
			for (; scratch_int > 1; --scratch_int) {
				regs16[REG_BP] -= 2;
				R_M_PUSH(CAST(uint16_t)mem[SEGREG(REG_SS, REG_BP,)]);
			}
			R_M_PUSH(scratch_uint);
		}
		regs16[REG_BP] = scratch_uint;
		regs16[REG_SP] -= i_data0;
		OPCODE 107: // LEAVE
			regs16[REG_SP] = regs16[REG_BP];
		R_M_POP(regs16[REG_BP]);
		OPCODE 108: // IMUL r,r/m,imm16 / imm8
			i_w = 1;
		i_d = 1;
		DECODE_RM_REG;
		op_to_addr = op_from_addr;
#ifdef IMUL_DEBUG
		printf("imul extra=%u op_to_addr=%08Xh "
				"i_data2=%04Xh i_reg=%u ",
				extra, op_to_addr, i_data2, i_reg);
#endif
		if (extra) {
			;
#ifdef IMUL_DEBUG
			printf("w ");
#endif
		} else {
			i_data2 = (int16_t)(int8_t)i_data2;
#ifdef IMUL_DEBUG
			printf("b ");
#endif
		}
		R_M_OP(mem[op_to_addr], *, i_data2);
#ifdef IMUL_DEBUG
		printf("i_data2(cast)=%04X op_result=%08Xh\r\n",
				i_data2, op_result);
#endif
		regs16[i_reg] = op_result;
		set_OF(set_CF((int32_t)op_result
					- (int32_t)(int16_t)op_result));
		OPCODE 109: // BOUND
			if (i_mod == 3) {
				pc_interrupt_reset(6);
				break;
			}
		i_w = 1;
		i_d = 0;
		DECODE_RM_REG;
		// op_to_addr = mem operand
		// op_from_addr = reg operand
		if ((CAST(int16_t)mem[op_from_addr]
					< CAST(int16_t)mem[op_to_addr])
				|| (CAST(int16_t)mem[op_from_addr]
					> CAST(int16_t)mem[op_to_addr + 2]))
			pc_interrupt_reset(5);
		OPCODE 110: // INSx (extra=1), OUTSx (extra=2)
			scratch2_uint = seg_override_en ? seg_override : REG_DS;

		scratch_uint = rep_override_en ? regs16[REG_CX] : 1;
		if (trap_flag && scratch_uint > 1) {
			reset_ip_after_rep_trace = 1;
			scratch_uint = 1;
		}
		for (; scratch_uint; scratch_uint--)
		{
			if (extra == 1)
				callin(&mem[SEGREG(REG_ES, REG_DI,)], regs16[REG_DX]);
			else
				callout(&mem[SEGREG(scratch2_uint, REG_SI,)], regs16[REG_DX]);
			extra & 1 || INDEX_INC(REG_SI),
				  extra & 2 || INDEX_INC(REG_DI);
			rep_override_en && regs16[REG_CX]--;
		}
		OPCODE 111: // LOCK prefix
			seg_override_en && seg_override_en++;
		rep_override_en && rep_override_en++;
		OPCODE 112: // INT1
			++reg_ip;
		pc_interrupt(1);
		break; default:
#ifdef INT6_DEBUG
			printf("Interrupt 6 at %04X:%04X = %02X %02X %02X %02X\r\n",
					regs16[REG_CS], reg_ip,
					opcode_stream[0],
					opcode_stream[1],
					opcode_stream[2],
					opcode_stream[3]);
#endif
		pc_interrupt_reset(6);
	}
	if (xlat_opcode_id != 23
			&& xlat_opcode_id != 27
			&& xlat_opcode_id != 111) {
		rep_override_en = seg_override_en = 0;
		if (prior_setting_ss)
			setting_ss = 0;
		prior_setting_ss = setting_ss;
	}


	// Increment instruction pointer by computed instruction length. Tables in the BIOS binary
	// help us here.
	reg_ip += (i_mod*(i_mod != 3) + 2*(!i_mod && i_rm == 6))*i_mod_size + bios_table_lookup[TABLE_BASE_INST_SIZE][raw_opcode_id] + bios_table_lookup[TABLE_I_W_SIZE][raw_opcode_id]*(i_w + 1);

	if (reset_ip_after_rep_trace) {
		reg_ip = reg_ip_before_rep_trace;
		reset_ip_after_rep_trace = 0;
	}

	// If instruction needs to update SF, ZF and PF, set them as appropriate
	if (set_flags_type & FLAGS_UPDATE_SZP)
	{
		regs8[FLAG_SF] = SIGN_OF(op_result);
		regs8[FLAG_ZF] = !op_result;
		regs8[FLAG_PF] = bios_table_lookup[TABLE_PARITY_FLAG][(unsigned char)op_result];

		// If instruction is an arithmetic or logic operation, also set AF/OF/CF as appropriate.
		if (set_flags_type & FLAGS_UPDATE_AO_ARITH)
			set_AF_OF_arith();
		if (set_flags_type & FLAGS_UPDATE_OC_LOGIC)
			set_CF(0), set_OF(0);
	}

	// Poll timer/keyboard every KEYBOARD_TIMER_UPDATE_DELAY instructions
	if (!setting_ss &&
			(++keyboard_timer_inst_counter >= KEYBOARD_TIMER_UPDATE_DELAY)) {
		keyboard_timer_inst_counter = 0;
		hlt_this_time = 0;
		int8_asap = 1;
	}

#ifndef NO_GRAPHICS
	// Update the video graphics display every GRAPHICS_UPDATE_DELAY instructions
	if (!setting_ss &&
			(++graphics_inst_counter >= GRAPHICS_UPDATE_DELAY))
	{
		graphics_inst_counter = 0;
		hlt_this_time = 0;
		// Video card in graphics mode?
		if (io_ports[0x3B8] & 2)
		{
			// If we don't already have an SDL window open, set it up and compute color and video memory translation tables
			if (!sdl_screen)
			{
				for (int i = 0; i < 16; i++)
					pixel_colors[i] = mem[0x4AC] ? // CGA?
						cga_colors[(i & 12) >> 2] + (cga_colors[i & 3] << 16) // CGA -> RGB332
						: 0xFF*(((i & 1) << 24) + ((i & 2) << 15) + ((i & 4) << 6) + ((i & 8) >> 3)); // Hercules -> RGB332

				for (int i = 0; i < GRAPHICS_X * GRAPHICS_Y / 4; i++)
					vid_addr_lookup[i] = i / GRAPHICS_X * (GRAPHICS_X / 8) + (i / 2) % (GRAPHICS_X / 8) + 0x2000*(mem[0x4AC] ? (2 * i / GRAPHICS_X) % 2 : (4 * i / GRAPHICS_X) % 4);

				SDL_Init(SDL_INIT_VIDEO);
				sdl_screen = SDL_SetVideoMode(GRAPHICS_X, GRAPHICS_Y, 8, 0);
				SDL_EnableUNICODE(1);
				SDL_EnableKeyRepeat(500, 30);
			}

			// Refresh SDL display from emulated graphics card video RAM
			vid_mem_base = mem + 0xB0000 + 0x8000*(mem[0x4AC] ? 1 : io_ports[0x3B8] >> 7); // B800:0 for CGA/Hercules bank 2, B000:0 for Hercules bank 1
			for (int i = 0; i < GRAPHICS_X * GRAPHICS_Y / 4; i++)
				((unsigned *)sdl_screen->pixels)[i] = pixel_colors[15 & (vid_mem_base[vid_addr_lookup[i]] >> 4*!(i & 1))];

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
	if (hlt_this_time) {
		struct timespec ts;
		ts.tv_sec = 0;
		ts.tv_nsec = HALT_TIME_MICROSECONDS * 1000;
		nanosleep(&ts, NULL);
		hlt_this_time = 0;
		keyboard_timer_inst_counter += KEYBOARD_TIMER_DELAY_FROM_HALT;
		graphics_inst_counter += GRAPHICS_DELAY_FROM_HALT;
	}

	if (!seg_override_en
			&& !rep_override_en
			&& xlat_opcode_id != 111) {
		reg_ip_before_rep_trace = reg_ip;

		// Application has set trap flag, so fire INT 1
		if (!setting_ss && trap_flag)
			pc_interrupt(1);

		trap_flag = regs8[FLAG_TF];

		// If a timer tick is pending, interrupts are enabled, and no overrides/REP are active,
		// then process the tick and check for new keystrokes
		if (!setting_ss && int8_asap && regs8[FLAG_IF] && !regs8[FLAG_TF])
			pc_interrupt(0xA), int8_asap = 0, SDL_KEYBOARD_DRIVER;
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

void returnxms(uint8_t bl) {
	uint16_t ax = 1;
	if (bl != 0)
		ax = 0;
	regs16[REG_AX] = ax;
	regs8[REG_BL] = bl;
}

uint8_t checkhandle(uint16_t handle, uint8_t errorcode_handle) {
	if (handle == 0 || handle > AMOUNT_XMS_HANDLES) {
    		returnxms(errorcode_handle);
		return 0;
	}
	if (xmshandles[handle - 1].allocation == NULL) {
    		returnxms(errorcode_handle);
		return 0;
	}
	return 1;
}

void * checkmoveaddress(
	uint16_t handle, uint16_t segment, uint16_t offset,
	uint32_t address, uint32_t count,
	uint8_t errorcode_handle, uint8_t errorcode_address) {
	void * pp;
	if (handle) {
		if (checkhandle(handle, errorcode_handle) == 0)
			return NULL;
		if (address + count
			> xmshandles[handle - 1].size) {
			returnxms(errorcode_address);
			return NULL;
		}
		return xmshandles[handle - 1].allocation
			+ address;
	} else {
		pp = mem + 16 * segment + (unsigned short)(offset);
		if (pp + count
			> (void*)mem + RAM_SIZE) {
			returnxms(errorcode_address);
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


void callxms() {
  uint32_t ii, free;
  uint64_t uu64;
  void* pp;
  if (regs8[FLAG_CF]) {		// CY, this is an XMS entrypoint call
    set_CF(0);			// signal that we support this
#ifdef XMS_DEBUG
	printf("xms call ax=%04Xh dx=%04Xh bx=%04Xh\r\n",
		regs16[REG_AX], regs16[REG_DX], regs16[REG_BX]);
#endif
    switch(regs8[REG_AH]) {
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
	regs16[REG_DX] = uu64;
	uu64 = free;
	uu64 >>= 10;		// rounding down
	regs16[REG_AX] = uu64;
	regs8[REG_BL] = 0;
    OPCODE 9:
#ifdef XMS_FAIL_FIRST_ALLOC
	{
		static uint8_t enabled = 0;
		if (! enabled) {
			enabled = 1;
			set_CF(1); return;
			// signal failure to tinyxms
		}
	}
#endif
	uu64 = regs16[REG_DX];
	uu64 <<= 10;
	for (ii = 0; ii < AMOUNT_XMS_HANDLES; ++ii)
		if (xmshandles[ii].allocation == NULL)
			break;
	if (ii == AMOUNT_XMS_HANDLES) {
		returnxms(0xA1);
		break;
	}
	++ii;
	pp = calloc(1, uu64);
	if (!pp) {
		returnxms(0xA0);
		break;
	}
	xmshandles[ii - 1].allocation = pp;
	xmshandles[ii - 1].size = uu64;
	xmshandles[ii - 1].lockcount = 0;
	regs16[REG_DX] = ii;
	returnxms(0);
    OPCODE 10:
	returnxms(freexms(regs16[REG_DX], 0));
    OPCODE 11: {
	struct xmsmove *mm = (void*)mem + SEGREG(REG_DS, REG_SI,);
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
		mm->sourcehandle, mm->sourcesegment, mm->sourceoffset,
		mm->sourceaddress, mm->count, 0xA3, 0xA4);
	if (!psource)
		break;
	pdest = checkmoveaddress(
		mm->desthandle, mm->destsegment, mm->destoffset,
		mm->destaddress, mm->count, 0xA5, 0xA6);
	if (!pdest)
		break;
#ifdef XMS_DEBUG
	printf("xmsmove psource=%08Xh pdest=%08Xh mem=%08Xh\r\n",
		psource, pdest, mem);
#endif
	// memmove acts correctly even when memory overlaps.
	// We are allowed to return error A8h if a backwards
	// move is needed, but we may support it too.
	memmove(pdest, psource, mm->count);
	returnxms(0);
	}
    OPCODE 12:
	if (checkhandle(regs16[REG_DX], 0xA2) == 0)
		break;
	returnxms(0xAD);
    OPCODE 13:
	if (checkhandle(regs16[REG_DX], 0xA2) == 0)
		break;
	returnxms(0xAA);
    OPCODE 14:
	if (checkhandle(regs16[REG_DX], 0xA2) == 0)
		break;
	for (ii = 0, uu64 = 0; ii < AMOUNT_XMS_HANDLES; ++ii)
		if (xmshandles[ii].allocation == NULL)
			++uu64;
	regs16[REG_AX] = 1;
	if (uu64 > 0xFF)
		uu64 = 0xFF;
	regs8[REG_BL] = uu64;
	ii = xmshandles[regs16[REG_DX] - 1].lockcount;
	if (ii > 0xFF)
		ii = 0xFF;
	regs8[REG_BH] = ii;
	uu64 = xmshandles[regs16[REG_DX] - 1].size;
	uu64 >>= 10;
	if (uu64 > 0xFFFF)
		uu64 = 0xFFFF;
	regs16[REG_DX] = uu64;
    OPCODE 15:
	if (checkhandle(regs16[REG_DX], 0xA2) == 0)
		break;
	if (xmshandles[ii - 1].lockcount != 0) {
		returnxms(0xAB);
		break;
	}
	uu64 = regs16[REG_BX];
	uu64 <<= 10;
	pp = realloc(xmshandles[regs16[REG_DX] - 1].allocation, uu64);
	if (!pp) {
		// original pointer in the handle still valid
		returnxms(0xA0);
		break;
	}
	xmshandles[regs16[REG_DX] - 1].allocation = pp;
	ii = xmshandles[regs16[REG_DX] - 1].size;
	if (ii < uu64) {
		for (; ii < uu64; ++ii)
			((uint8_t*)pp)[ii] = 0;
	}
	xmshandles[regs16[REG_DX] - 1].size = uu64;
	returnxms(0);
	break;
    default:
	returnxms(0x80);	// function not implemented
	break;
    }
  } else {			// NC, this is a different dispatcher
    switch(regs16[REG_AX]) {
    OPCODE_CHAIN 0:
      for (ii = 0; ii < AMOUNT_XMS_HANDLES; ++ii) {
	freexms(ii + 1, 1);
      }
      regs16[REG_AX] = -1;
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
	regs16[REG_AX] = uu64;
	set_CF(1);		// this means no error here
    }
  }
}
