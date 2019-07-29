
%if 0

8086tiny XMS driver (UMBs, HMA)
 2019 by C. Masloch

Usage of the works is permitted provided that this
instrument is retained with the works, so that any entity
that uses the works is notified of this instrument.

DISCLAIMER: THE WORKS ARE WITHOUT WARRANTY.

%endif


%include "lmacros3.mac"
%include "AMIS.MAC"

	defaulting

	numdef DEBUG4
	numdef CHECK_8086TINY, 1
	numdef USE_8086TINY_CALL, 1
	gendef UMBLIST, {0D000h, 2000h}


	cpu 8086
	org 0
deviceheader:
	dd -1
	dw 8000h
.strategy:
	dw strategy_init
	dw deviceretf
	db "XMSXXXX0"

i2F.hwreset:
deviceretf:
	retf


options:
	db 0
opt_hma_in_use equ 1


	align 4
umbtable:
	dw _UMBLIST
	dw 0, 0


	align 2
iispentry i2F, 0, i2F
	cmp ax, 4300h
	je .check
	cmp ax, 4310h
	je .entry
	jmp far [cs:.next]

.check:
	mov al, 80h
	iret

.entry:
	mov bx, xmsentry
	push cs
	pop es
	iret

xmsentry:
	jmp strict short .entry
	times 3 nop
.entry:
	cmp ah, 1
	jb .version		; version
	je .allochma		; request HMA
	cmp ah, 3
	jb .freehma		; free HMA
	je .success		; global enable A20
	cmp ah, 5
	jb .disable		; global disable A20
	je .success		; local enable A20
	cmp ah, 7
	jb .disable		; local disable A20
	je .success		; get A20 status (1 = enabled)
	cmp ah, 10h
	jb .call
	je .requestumb
	cmp ah, 12h
	jb .releaseumb
.invalid:
	mov bl, 80h		; function not implemented
	jmp .failure


.call:
%if _USE_8086TINY_CALL
	stc
	db 0Fh, 04h		; call into emulator
		; The emulator handles functions 8 to 0Fh.
		; These need access to extended memory.
	jnc .retf
%endif
	jmp .invalid


.version:
	mov ax, 200h
	mov bx, 100h
	mov dx, 1
.retf:
	retf


.allochma:
	testopt [cs:options], opt_hma_in_use
	jz @F
	mov bl, 91h
.failure:
	xor ax, ax
	retf

@@:
	setopt [cs:options], opt_hma_in_use
.success:
	mov bl, 0
	mov ax, 1
	retf


.freehma:
	testopt [cs:options], opt_hma_in_use
	jnz @F
	mov bl, 93h
	jmp .failure

@@:
	clropt [cs:options], opt_hma_in_use
	jmp .success


.disable:
	mov bl, 94h
	jmp .failure


.requestumb:
	push si
	push ds
	push cx

d4	mov si, debugmsg.umb_request_1
d4	call disp_msg
d4	mov ax, dx
d4	call disp_ax_hex
d4	mov si, debugmsg.umb_request_2
d4	call disp_msg

	 push cs
	 pop ds
	xor cx, cx
	mov si, umbtable
@@:
	lodsw
	lodsw
	test ax, ax
	js @B
	jz .requestumb_end
	cmp ax, cx
	jbe @F
	mov cx, ax
@@:
	cmp ax, dx
	jne @BB
	mov dx, [si - 2]
	or byte [si - 1], 80h
	mov bx, word [si - 4]

d4	mov si, debugmsg.umb_response_1
d4	call disp_msg
d4	mov ax, dx
d4	call disp_ax_hex
d4	mov si, debugmsg.umb_response_2_yes
d4	call disp_msg
d4	mov ax, bx
d4	call disp_ax_hex
d4	mov si, debugmsg.umb_response_3_yes
d4	call disp_msg

.success_pop:
	pop cx
	pop ds
	pop si
	jmp .success

.requestumb_end:
	mov bl, 0B0h		; only smaller UMB available
	mov dx, cx
	test cx, cx
	jnz @F
	mov bl, 0B1h		; no UMBs available
@@:

d4	mov si, debugmsg.umb_response_1
d4	call disp_msg
d4	mov ax, dx
d4	call disp_ax_hex
d4	mov si, debugmsg.umb_response_2_no
d4	call disp_msg
d4	mov al, bl
d4	call disp_al_hex
d4	mov si, debugmsg.umb_response_3_no
d4	call disp_msg

.failure_pop:
	pop cx
	pop ds
	pop si
	jmp .failure


.releaseumb:
	push si
	push ds
	push cx
	 push cs
	 pop ds
	mov si, umbtable
@@:
	lodsw
	cmp ax, dx
	lodsw
	je .releaseumb_free
	test ax, ax
	jnz @B
@@:
	mov bl, 0B2h
	jmp .failure_pop

.releaseumb_free:
	test ax, ax
	jns @B
	and byte [si - 1], ~80h
	jmp .success_pop


%if _DEBUG4
disp_ax_hex:
	xchg al, ah
	call disp_al_hex
	xchg al, ah
disp_al_hex:
	push cx
	mov cl, 4
	rol al, cl
	call disp_al_nybble_hex
	rol al, cl
	pop cx
disp_al_nybble_hex:
	push ax
	and al, 0Fh
	cmp al, 9
	jbe @F
	add al, 'A' - ('0' + 10)
@@:
	add al, '0'
	call disp_al
	pop ax
	retn

disp_al:
	push ax
	push bx
	push bp
	mov ah, 0Eh
	mov bx, 7
	int 10h
	pop bp
	pop bx
	pop ax
	retn


disp_msg.disp:
	call disp_al
disp_msg:
	cs lodsb
	test al, al
	jnz .disp
	retn

debugmsg:
.umb_request_1:		asciz "Requesting UMB size="
.umb_request_2:		asciz "h",13,10
.umb_response_1:	asciz "Responding with UMB size="
.umb_response_2_yes:	asciz "h segment="
.umb_response_3_yes:	asciz "h",13,10
.umb_response_2_no:	asciz "h error code="
.umb_response_3_no:	asciz "h",13,10
%endif


	align 16
resident_end:


strategy_init:
	push ds
	push si
	push di
	push dx
	push cx
	push ax
	push es
	push bx
	 push cs
	 pop ds

	mov word [es:bx + 3], 8103h	; error, done, unknown command

	cmp byte [es:bx + 2], 0		; INIT command ?
	jne .return

	xor ax, ax
	mov byte [es:bx + 0Dh], al	; no units
	mov word [es:bx + 0Eh], ax
	mov word [es:bx + 0Eh + 2], cs	; to remove driver
	mov word [es:bx + 12h], ax
	mov word [es:bx + 12h + 2], ax	; BPB array (none)

%if _CHECK_8086TINY
	mov dx, msg.error_not_8086tiny
	mov ax, 0F000h
	mov es, ax
	xor di, di
	mov cx, -1
@@:
	mov al, '8'
	repne scasb
	jne .error
	push di
	push cx
	dec di
	mov si, msg.signature
	mov cx, msg.signature.length
	repe cmpsb
	pop cx
	pop di
	jne @B
%endif

	mov ax, 4300h
	int 2Fh
	cmp al, 80h
	mov dx, msg.error_already
	jne @F
.error:
	mov ah, 09h
	int 21h
	jmp .return

@@:
%if _USE_8086TINY_CALL
	xor ax, ax
	clc			; NC, ax=0: free all allocations (after reboot)
	db 0Fh, 04h
%endif

	mov ax, 352Fh
	int 21h
	mov word [i2F.next], bx
	mov word [i2F.next + 2], es
	mov dx, i2F
	mov ax, 252Fh
	int 21h

	pop bx
	pop es
	push es
	push bx
	mov word [es:bx + 0Eh], resident_end
	mov word [es:bx + 3], 0100h	; no error, done
	mov word [deviceheader.strategy], deviceretf

.return:
	pop bx
	pop es
	pop ax
	pop cx
	pop dx
	pop di
	pop si
	pop ds
	retf

msg:
.error_already:	ascic "tinyxms: An XMS handler is already installed. Unloading.",13,10
%if _CHECK_8086TINY
.error_not_8086tiny:	ascic "tinyxms: Not running in 8086tiny. Unloading.",13,10
.signature:		db "8086tiny BIOS"
.signature.length: equ $ - .signature
%endif
