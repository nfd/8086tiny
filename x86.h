#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>

// Emulator system constants
#define IO_PORT_COUNT 0x10000
#define RAM_SIZE 0x10FFF0
#define REGS_BASE 0xF0000
#define VIDEO_RAM_SIZE 0x10000

// 16-bit register decodes
#define REG_AX 0
#define REG_CX 1
#define REG_DX 2
#define REG_BX 3
#define REG_SP 4
#define REG_BP 5
#define REG_SI 6
#define REG_DI 7

#define REG_ES 8
#define REG_CS 9
#define REG_SS 10
#define REG_DS 11

#define REG_ZERO 12
#define REG_SCRATCH 13

// 8-bit register decodes
#define REG_AL 0
#define REG_AH 1
#define REG_CL 2
#define REG_CH 3
#define REG_DL 4
#define REG_DH 5
#define REG_BL 6
#define REG_BH 7

// FLAGS register decodes
#define FLAG_CF 40
#define FLAG_PF 41
#define FLAG_AF 42
#define FLAG_ZF 43
#define FLAG_SF 44
#define FLAG_TF 45
#define FLAG_IF 46
#define FLAG_DF 47
#define FLAG_OF 48

struct x86_state {
	void (*redraw_display)(struct x86_state *);
	void (*keyboard_driver)(struct x86_state *);
	void (*pause_audio)(int pause);
	ssize_t (*read)(int fd, void *buf, size_t count);
	ssize_t (*write)(int fd, const void *buf, size_t count);
	int op_result, disk_bios, disk_fdd, disk_hdd;
	time_t clock_buf;
	unsigned int op_source, op_dest, rm_addr, op_to_addr, op_from_addr, i_data0, i_data1, i_data2, scratch_int, scratch_uint, scratch2_uint, keyboard_timer_inst_counter, graphics_inst_counter, set_flags_type, GRAPHICS_X, GRAPHICS_Y, pixel_colors[16];
	unsigned short *regs16, reg_ip, seg_override, wave_counter, reg_ip_before_rep_trace;
	unsigned char *opcode_stream, *regs8, i_rm, i_w, i_reg, i_mod, i_mod_size, i_d, i_reg4bit, raw_opcode_id, xlat_opcode_id, extra, rep_mode, seg_override_en, rep_override_en, trap_flag, int8_asap, scratch_uchar, io_hi_lo, spkr_en, hlt_this_time, setting_ss, prior_setting_ss, reset_ip_after_rep_trace, shift_count;
	unsigned char *text_vid_mem, *text_vid_mem_double_buffer;
	unsigned char io_ports[IO_PORT_COUNT + 16];
	unsigned char mem[RAM_SIZE + 16];
};

struct x86_state *x86_init(int boot_from_hdd, char *bios_filename, char *fdd_filename, char *hdd_filename, void(*redraw_display)(struct x86_state *), void(*keyboard_driver)(struct x86_state *), void(*pause_audio)(int pause), ssize_t(*read)(int, void *, size_t), ssize_t(*write)(int, const void *, size_t));
void x86_free(struct x86_state *);
void x86_step(struct x86_state *);
uint32_t x86_handle_hlt(struct x86_state *s);
void x86_handle_irqs(struct x86_state *s);
//char set_CF(struct x86_state *s, int new_CF);
char pc_interrupt(struct x86_state *s, unsigned char interrupt_num);
