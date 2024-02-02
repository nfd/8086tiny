8086tiny
========

8086tiny is a completely free (MIT License) open source
PC XT-compatible emulator/virtual machine written in C.
It is, we believe, the smallest of its kind (the
fully-commented source is under 25K). Despite its size,
8086tiny provides a highly accurate 8086 CPU emulation,
together with support for PC peripherals including XT-style
keyboard, floppy/hard disk, clock, audio, and Hercules/CGA
graphics. 8086tiny is powerful enough to run software like
AutoCAD, Windows 3.0, and legacy PC games: the 8086tiny
distribution includes Alley Cat, the author's favorite PC
game of all time.

8086tiny is highly portable and runs on practically any
little endian machine, from simple 32-bit MCUs upwards.
8086tiny has successfully been deployed on 32-bit/64-bit
Intel machines (Windows, Mac OS X and Linux), Nexus 4/ARM
(Android), iPad 3 and iPhone 5S (iOS), and Raspberry Pi (Linux).

The philosophy of 8086tiny is to keep the code base as
small as possible, and through the open source license
encourage individual developers to tune and extend it as
per their specific requirements, adding support, for example,
for more complex instruction sets (e.g. Pentium) or
peripherals (e.g. mouse). Forking this repository is highly
encouraged!

Any questions, comments or suggestions are very welcome
in our forum at 8086tiny.freeforums.net.

nfd's fork
==========
I forked ecm's fork (see below) to support running more than
one instance of the emulator in one address space, which means
removing all the global variables and consolidating the IO.

ecm's fork
==========

In this fork, I implemented proper Trace Flag handling
for debuggers, interrupt-lockout for mov or pop to ss,
and idling with the HLT instruction. I also fixed the
shift/rotate count handling so that the machine is
always detected as an 8086, as most 186 instructions
are still missing. (This is still an option in the
source but now defaults to being detected as an 186.)
All 186 instructions were added. The invalid opcode
condition now invokes interrupt 6, except for the
0Fh-prefixed emulator extension instructions with a
second byte below 20h. (Codes 00h to 04h are in use.)

Further, the BIOS's interrupt handlers were modified
to be more compatible. Data has been aligned, and several
corner cases are handled more properly. The two forms of
186+ push instructions with immediates were implemented,
which are used by FreeCOM (a bug). (Now all of the 186
instructions were added.) Three bugs in the emulator
were fixed, two of which affected some of the shift and
rotate instructions, one the DAA and DAS instructions.

A DOS driver and coupled emulator interface add XMS 2.00
handling to the machine, including HMA, UMB, and XMS
extended memory. This driver is limited: A20 is always
on, HMAMIN is assumed as zero, and XMS blocks cannot be
locked. XMS memory can only be accessed by the driver.
UMBs can only be allocated if the requested size
exactly matches a block's full size (86-DOS systems
compatible to MS-DOS 5 generally do it that way).
Further, the UMB area is assumed by the DOS driver.
(This is 128 KiB (2000h paragraphs) between segments
D000h and EFFFh.) When the system is rebooted, the
DOS driver's initialisation frees all prior allocations.
