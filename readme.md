# OpenKlave

{:toc}

## Introduction

OpenKlave is an open source operating system for the Akai MPK2 keyboard series, including the MPK 225, MPK249, and the MPK261.

The launch goal of the project is to permit keyboard users to load and execute arbitrary code onto the keyboard using only a usb cable to the keyboard and stock software.
It is intended to work around and augment the stock bootloader and operating system, instead of replacing them.

If the launch goal is reached and there's community interest, we will work towards a from-scratch rewrite of both the bootloader and operating system.

## Project purpose and copyright note

This project was started because the MPK249 is a beautiful device, full of promise, with terrible software, and almost no support from Akai. It has fun light pads and nice-feeling keys.
But the MPK249 barely interacts with Ableton. One can fix many issues using sysex commands run from a laptop, but certain issues require modifying the system software.
For example, one cannot currently write arbitrary messages to the LCD screen without completely freezing the system.

This project does not copy or redistribute any copyright-protectible software from Akai or any other source. The MPK2 series has no encryption mechanism
which prevents modification. As device owners, it is our legal right to arbitrarily modify our devices without restriction, and to write and share original software
to advance this purpose.

If you receive any notifications from any party to the contrary, let us know in the Github Issues. 

## Required background-knowledge and tools

This document assumes almost no knowledge except a little python. The goal is to learn everything by playing and experimentation, diving deeper and deeper each iteration by trying to accomplish progressively more difficult tasks.

The two essential tools are a) an ST-LINK/V2 debugger, which you can get for $30 on Amazon (link below), and a b) screw driver to open the keyboard up.

## Diving in

A whole lot of modification can be achieved just by sending sysex commands to the MPK2 device, for example by using rtmidi to send midi messages from your laptop to the device.
Doing this, you can change the colors on the keypads, and even intercept and modify midi messages between the device and your software.

But our goal is to permit keyboard users to execute arbitrary code on the keyboard with only a USB cable. To do this, we're going to have to deeply investigate the guts of the keyboard and learn a lot about it.

### Investigating the firmware

Unscrew the screws at the bottom of your keyboard, and investigate the primary printed circuit board. 

<p align="center">
  <img width="460" height="300" src="/pics/mpk249pcb.jpg">
</p>

Notice two things. First, its brain is a microcontroller called an STM32F103. Second, it has a 16 pin JTAG connector so you can debug it.

[You'll want to buy the $30 ST-Link/V2 debugger from Amazon.](https://www.amazon.com/ST-LINK-V2-circuit-debugger-programmer/dp/B00SDTEM1I/ref=sr_1_3?dchild=1&keywords=st-link%2Fv2+debugger&qid=1618172788&sr=8-3) This thing is essential. It will allow you to debug firmware on the STM32F103, set memory values, and even step through program execution instruction by instruction.

#### A note on the STM32F103

The STM32F103 runs an instruction set called ARM Cortex M3. The chip is 32-bit and little-endian, which will be important for setting up Ghidra and sending values to it later.

#### Accompanying software

We assume you have installed:

1. Ghidra, for decompiling the system firmware and annotating it.
2. OpenOCD, for connecting your laptop debugger to the microcontroller and setting up a gdb instance.
3. GDB, for debugging.
4. A python installation with rtmidi (pip install), so you can send midi and sysex messages to the keyboard.

Here's what it should look like once you have the ST-Link/V2 plugged in to the MPK2. Note that this requires 2 usb connections to work. So you'll have the ST-LINK/v2 connected to the JTAG port and plugged into your computer's USB, and you'll have a second USB connection to the back of the keyboard, to power and send messages to it, like you would usually.

<p align="center">
  
  <img width="460" height="300" src="/pics/debugger-setup.jpg">
</p>

### Warm-up exercise: entering debug mode on the keyboard.

Let's verify everything is properly connected. Open up a few terminal windows. In one, you will run OpenOCD to launch a GDB server.

```
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg
```

If everything is working properly you should see something like
```
...
Info : STLINK V2J29S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.217323
Info : stm32f1x.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : starting gdb server for stm32f1x.cpu on 3333
Info : Listening on port 3333 for gdb connections
...
```

Now, in a second terminal, start gdb and connect to this local server.

```
gdb

(in gdb)
target remote localhost:3333
```
which should display
```
Remote debugging using localhost:3333
```

#### Your first GDB lesson

1. Press CTRL-C to stop the keyboard executing its current program, and enter debug mode.
2. Enter 'c' to continue execution. PC means 'program counter'. It is the current instruction that is executing.
3. Enter 'stepi' to step forward one instruction on the keyboard.
4. Reset the keyboard by entering 'monitor reset halt'. You will need to enter c to start it running like normal.

#### Task: Demonstrate you can put your MPK249 in debug mode. 
If you did everything right and sucessfully connected gdb to the MPK2, the unit will display cool debug stripes in the light pads indicating the keyboard is in debug mode. Notice how the MPK249 debug mode alters the pad colors between columns of blue and green.

<p align="center">
  <img width="460" height="300" src="/pics/debugstripes.jpg">
</p>

When tinkering with the keyboard, it is more comfortable to adjust the plastic hood mostly back over the keys, so you can play notes and test things out as you debug.

### Task: Understand the memory layout of the STM32F103 and rip the MPK2 bootloader to your computer.

Review the memory layout of the STM32.
<p align="center">
  <img width="460" height="300" src="https://i.stack.imgur.com/ugTmg.png">
</p>

Memory addresses are represented in hexidecimal, so each digit goes 1 to 9, then A, B, C, D, E, F.

There are two pieces of software running on the device, the bootloader, and the operating system. 
The bootloader is located from memory address 0x0 to 0x6000, and it is mirrored at 0x08000000 to 0x08006000.

How much software is that? Let's review hexidecimal. 0x6000 means no ones, no 16s, no 256s, and 6 4096s.

So the bootloader is 24576 bytes. That's not too bad! We have a reasonable chance of reading and understanding what it does.

Each memory address stores a byte, which is made of up of two hex digits, like 0xA5, or 0x01, for example, up to 0xFF.

The addressess typically run in sequences of 2 or 4 bytes. 2 byte sequences, like 0x04 0x06 (move the value of register 0 to register 4) are usually Cortex M3 instructions. 4 byte sequences are typically either "vectors," which means they store an address on the chip itself, or 4 byte Cortex M3 instructions.

It can be really hard to read through sequences of bytes to try to figure out which instructions are encoded, forget about whole functions. Decompilers like Ghidra or IDA load these byte sequences and display them in human-readable code so you can understand what is happening.

The operating system is stored from 0x6000 to 0x33fff. That is 0x2DFFF bytes, or just about one fifth of a megabyte.

### Ripping the firmware, the OS, and both.

GDB can copy sectors of memory from your MPK2 to your local filesystem using the [dump command](https://stuff.mit.edu/afs/athena/project/rhel-doc/3/rhel-gdb-en-3/dump-restore-files.html).

```
 dump binary memory bootloader.bin 0 0x6000
```
 Will dump the bootloader to a bin file. Note you can also dump it to an ihex file format, which we'll discuss more later.
 Make sure you keep this backup file in a safe place you never modify. 
 I personally went through several chips before I learned to back up and restore firmware properly.
 
 ```
 dump binary memory mpk2os.bin 0x08006000 0x08033fff
 ```
 
 Let's be safe and also dump the os in the ihex format so we can reload it using the bootloader
 
 ```
dump ihex memory mpk2os.ihex 0x08006000 0x08033fff
 ```

Now shockingly enough, this bare ihex dump is sufficient to completely restore the operating system if you bork it during development.
The bootloader has a built-in recovery mode which receives ihex as a sysex message and rerwrites the flash over usb.

Rename mpk2os.ihex to mpk2os.syx, and add the following hex bytes to the beginning of the file. 

#### Make sure you have a good hex editor handy
https://hexed.it/ is excellent and it works in your browser.

```
F0 47 00 24 70
```

and this byte to the end of the file

```
F7
```

And you can play mpk2os.syx to the system, for example by using software like Sysex Librarian, to restore it.
 
 ### Minimal rtmidi program to send sysex to wipe the OS
 
 ```
import rtmidi
midiout = rtmidi.MidiOut()
midiout.open_port(0)
header = [0xF0, 0x47, 0x00, 0x24]
footer = [0xF7]
msg = [0x72, 0, 0] 
midiout.send_message(header + message + footer)
```

### Task: Dump the OS, wipe the OS, play the dump back as sysex to restore the system.

Dump the OS as instructed above into an ihex file.
Make the modifications to the .syx file.
Use rtmidi over usb to send a [0x72, 0, 0] wipe message.
Play your dumped OS back to the system and restore it.

Do not move on until you are able to complete this task, as it requires basic proficiency with sysex, rtmidi, and gdb.
If you're getting stuck at any parts, submit an issue and let us know.

*Warning* Be extremely careful that you are backing up your firmware correctly before you attempt a wipe and replace.
Read the dump instructions a few times, and make sure you enter them exactly.
We will not be able to provide you a working copy of the operating system if something goes wrong, and you will have to contact Akai.

*Hint*
The first few bytes of the backed up OS should look like this:
```
0xF0, 0x47, 0x00, 0x24, 0x70, 0x3A, 0x30, 0x32, 0x30, 0x30, 0x30, 0x30,
0x30, 0x34, 0x30, 0x38, 0x30, 0x30, 0x46, 0x32, 0x0D, 0x0A...
```
 ## Active Research
 
The goal is to enable a new device owner to overwrite an arbitrary address in memory, using only the usb connection and their laptop.
This is likely possible in one of two ways: either a stack overflow, or using a facility in the current bootloader or OS.
 
We know that once the system is put in [0x72, 0, 0] mode, the OS is wiped at 0x6000 and after, and it is possible to use the ihex backup to rewrite the system.
Here is a small test message you can use to overwrite the first four bytes of the OS with the following values:

```
0x6000:	0x12345678	0x080061ab	0x0800fdef	0x0800e5f3
```
You can track these addresses at every iteration in gdb via
```
display x/4xw 0x6000
```
```
testmsg = [0xF0, 0x47, 0x00, 0x24, 0x70, 0x3A, 0x30, 0x32, 0x30, 0x30, 0x30, 0x30,
	0x30, 0x34, 0x30, 0x38, 0x30, 0x30, 0x46, 0x32, 0x0D, 0x0A, 0x3A, 0x31,
	0x30, 0x36, 0x30, 0x30, 0x30, 0x30, 0x30, 0x37, 0x38, 0x35, 0x36, 0x33,
	0x34, 0x31, 0x32, 0x41, 0x42, 0x36, 0x31, 0x30, 0x30, 0x30, 0x38, 0x45,
	0x46, 0x46, 0x44, 0x30, 0x30, 0x30, 0x38, 0x46, 0x33, 0x45, 0x35, 0x30,
	0x30, 0x30, 0x38, 0x38, 0x34, 0x0D, 0x0A]
```
 
the question is, is it possible to overwrite specific addresses without completely wiping the OS beforehand? Need to investigate DMA.

If you restart the device and hold the "Push to Enter" wheel down as it powers on you will see the message
```
Emeregency Updater
Use Updater Utility
to perform update
```
Once the knob is released, the system will boot into normal mode, demonstrating no part of the OS was erased.

In this mode, if you send the testmsg message above over USB using rtmidi, 
we find that a routine was run that converted the ihex message characters into the exact four words we're looking for, and stored them at 0x200001495 in flash.
```
 x/4xw 0x20001495
```

When the system is in reset mode, these words are further overwritten to 0x6000, suggesting that there's some memory map between these flash addresses and the operating system, or a routine that otherwise copies these addresses to the OS.

It appears after much examination, that 0x08000f88 is the function that does the copying from the flash to the device. We notice 1) this function moves 0x1 to 0x40022010 (the flash controller on STM32f1) before it writes, and that it 
```
strh r6, [r5, #0]
```
where r6 is the half-word being written, and r5 holds the value 0x8006000 or similar.

### How do presets work

Presets stored with the [0x10] command are written to flash pages starting at 0x8034040

## Ghidra Basics

In this section we discuss analyzing the OS and bootloader you ripped from the device earlier, in Ghidra. We discuss memory maps and loaders to make debugging easier.

Please refer to the [reference card](https://www.ic.unicamp.br/~ranido/mc404/docs/ARMv7-cheat-sheet.pdf) as you start to make your way through Arm M3 asesembly.

## Finding an exploit to inject arbitrary code

A significant amount of time was spent looking for areas where the last element of the stack frame 
could be overwritten to force the return to an arbitrary address.

Here are some of the potential areas.

### ldrb and strb in loops

[Start here.](https://medium.com/techmaker/stack-buffer-overflow-in-stm32-b73fa3f0bf23). There are a few ways to allocate stack. 
One is looking for sub sp, add sp instructions at the start and end of a function.

Observe the function in the OS defined at 0x0800e5d0:

```
        0800e5d0 00 b5           push       { lr }
        0800e5d2 91 b0           sub        sp,#0x44
	...
        0800e5ec 11 b0           add        sp,#0x44
```
A stack 0x44 bytes long (68 bytes) is being allocated. The operations that take place between the elipses 
will almost certainly write to the stack frame. If you can find a subroutine, for example a loop,
that checks for a terminator, or uses some other means besides the stack size to determine
how many times to allocate to the stack, there's a possibility of finding an exploit.

For example

```
0x80061f6   ldrb.w      r3, [r1], #1    (load value in address r1 into r3, increment by 1)                                                        
0x80061fa   strb.w      r3, [r0], #1    (store value in r3 into address r0, increment by 1)                                                      
0x80061fe   subs        r2, r2, #1      (r2--)       
0x8006200   bcs.n       0x80061f6       (loop to the top if r2 > 0)
```
Here's a similar loop, but this one waits for a string terminator to stop copying between two addresses in RAM

```
0x800626c   ldrb.w     r2,[r1],#0x1     (load value in address r1 into r2, increment by 1)
0x8006270   strb.w     r2,[r0],#0x1     (store value in r2 into address in r0, increment by 1)
0x8006274   cmp        r2,#0x0          (Is r2 == 0?)
0x8006276   bne        LAB_0800626c     (keep looping if r2 isn't 0)
```
Here, if there was a way to write an arbitrary length stream of data starting at r1, this routine
could help us copy into the address at r0, possibly corrupting the stack frame and allowing 
us to inject arbitrary code.

Here's yet another variant of this pattern, this time using ldmia and stmia.
```
0x80061ea   ldmia       r1!, {r3}       (load value in address stored in r1 to r3, increment r1)                                                           
0x80061ec   subs        r2, r2, #4      (r2 -= 4)                                                  
0x80061ee   stmia       r0!, {r3}       (Take the value in r3 and store it at the address in r0, increment r0)                                                
0x80061f0   cmp r2, #4                  (is r2 == 4 yet?)                                                        
0x80061f2   bcs.n       0x80061ea       (if not, keep looping)
```
## Learning more about the display (to drive it)

The LCD Display on the MPK249 is a Winstar model number
WH2004a.

<p align="center">
  <img width="460" height="300" src="/pics/winstarback.jpg">
</p>

[Here is the full specification](https://www.winstar.com.tw/products/character-lcd-display-module/wh2004a.html) distributed by the manufacturer.

There are a couple of different ways to drive it, but our brand uses '6800 interface (ST7066 IC)'. We're going to have to 
initialize it, send some clock pulses to it from our STM32, and come up with a loop to update the screen.

[Here is the data sheet for the ST7066] (https://www.sparkfun.com/datasheets/LCD/st7066.pdf), which is pin compatible with the HD44780.


We don't get to decide arbitrarily whether we'll use GPIO, IC2, or something else to drive the LCD. The STM32F103 is slotted into 
the MPK249 PCB, and there are already dedicated connected to the LCD ribbon.

<p align="center">
  <img width="460" height="300" src="/pics/lcdpinout.jpg">
</p>

Thankfully, we can learn a lot from the text, and it matches our winstar LCD spec. Specifically, we see
* 8 pins [D0-D7] for data
* LCD RW
* LCD RS
* LCD E(nable) and
* LCD Contrast DC (which appears to be the voltage potentiometer to alter the screen contrast).

and some other stuff that doesn't belong, like "Load LEDS". So this pinout is probably doing double duty.

Now, the trick is to figure out which GPIO pins are mapped to which of the labelled outlets on the PCB.

So we set some breakpoints in GDB when characters are being written to the screen and hopefully we can figure out what's going on.

### Go understand how GPIO works before you read further.

STM32F103 memory-maps its perpherials. Which means everything, clocks, output voltages driving LCDs, USB,
is just another address.

Here's a handy [cheatsheet](https://gist.github.com/iwalpola/6c36c9573fd322a268ce890a118571ca) on how GPIO works.

Every GPIO 'port' (a-e) has 16 pins it drives, which are physical pieces of metal sticking out of the CPU
which are wired via ribbon cable to the MPK2 device.

Mostly, the pins are written via the 0x10 address extension (BSRR).

Critically, you need to watch (and set) two seperate things, the GPIO configuration (is the pin input or output, what is its speed etc)., and the data coming out of the pin (0 or 1). 

The below 4 gdb instructions will show you config status (0x00) and data read (0x08) address values for GPIOB and GPIOC.
Theoretically, if just GPIOB and GPIOC are driving the LCD, these four values are all we need to track over time to
see how the LCD is being driven. Ideally, it will map to the ST7066 spec sheet linked above, and not some
random nonsense reflecting hidden knowledge we can't obtain public documentation for.

Our operating system depends on using publicly documented and non-creative, obvious mechanisms
to interface with peripherals like the screen or buttons.

```
display/1xw 0x40010c08
display/1xw 0x40010c00
display/1xw 0x40011008
display/1xw 0x40011000
```
Port B Config

Port B Data

Port C Config

Port C Data

Note also, the BSRR function just *overwrites* certain pin values, it 
does not change them all at once. So you really need to track GPIO values over time 
to understand what's going on; it's not enough to see a given value is stored in a GPIO register
that corresponds to BSRR, because you're missing the current values its amending, and the port's 
current configuration.

## We wrote some characters.

It appears GPIOB1 and GPIOB2 are driving the R/S and R/W bits of the LCD,
and that GPIOC0-GPIO-7 are driving the data bits. At this point this is 
just a hypothesis.
```
0x40010c00 : 0x70000
0x40010c00 : 0x1000
0x40010c00 : 0x20000000
0x40010c00 : 0x80000000
0x40010c00 : 0x200000

0x40011008 (idr) : 0x980067
and then a g is written to the screen.
```

So breaking this down, what happened?

0x980067 translates to 0b100110000000000001100111, and matching
that last byte to to the ST7066 data sheet, [0110][0111] corresponds
to the letter g. So we have confirmation that GPIOC0-7 are most likely
wired to the LCD data.

Now through experiment it appears GPIOB1 and GPIOB2 are actually driving 
R/S and RW. But what does GPIOB0 do? It sends the enable clock pulse
[0 - 1 - 0] to the LCD which drives one iteration of the circuit.

Don't take my word for it, we can prove all of this is true within GDB.

Boot up an unmodified MPK249 and pause it in GDB (ctrl-z). Define a
'clock pulse' function. 

```
define lcdpulse
  set {int} 0x40010c10 = 0x1
  shell sleep .1
  set {int} 0x40010c10 = 0x10000
end
```

where this is proven to work on Mac and LINUX, but the 'shell sleep .1' needs an equivalent
Windows command. Isn't this cool? We can use GDB to test out certain ideas and write
fragments of code to verify those ideas before we even code them.

While the debugger is paused, make sure the lower 8 GPIOC pins are set to write
```
set {int} 0x40011000 = 0x33333333
```

Write in the character 'g' into the LCD data:

```
set {int} 0x40011010 = 0x980067
```

and then 

```
lcdpulse
```

Wow! 'g' has been written to the screen.

## A small matter of contrast

At this point, we have everything we need to write a quick driver in C
to drive the LCD. Except hmm. We know a potentiometer is supposed to change the 
LCD contrast, and it's not exactly apparent how this digital chip can
send precisely tuned voltage. And which pin is doing it?

By experimenting with all the GPIO pins, we can figure it out.
Watch the GPIOB configuration and data.
```
display/1xw 0x40010c08
display/1xw 0x40010c00
```
Ok, this is weird. If we pause the debugger and just stepi through the instructions, we notice
x/xw 0x40010c08  0x40010c08:	0x0000ffd4
x/xw 0x40010c08  0x40010c08:	0x0000fff4

and back and forth. It looks like pin B5 is flipping back and forth between 1 and 0 on its own. 
Learning about pulse width modulation here: link. This seems like an interesting candidate.

And look at Pin 5's configuration. 
```
(gdb) x 0x40010c00
0x40010c00:	0x44b84222
```
What the heck does b mean? 'Alternative function?' That seems new. If we set that pin to read
```
set {int} 0x40010c00 = 0x44484222
```
the screen contrast goes to 0. Ok. And if we change it from b to write:
```
set {int} 0x40010c00 = 0x44184222
```
the screen goes full contrast. Ok we're getting somewhere.

### Shortcutting through pulse width modulation and timers.

So Pin B5 is flickering alot, by itself apparently. And its voltage
seems to drive the potentiometer, somehow. We also see its configuration
is in this new 'alternative function'. What is that?

<p align="center">
  <img width="460" height="300" src="/pics/LQFP100.png">
</p>

There's a pin diagram for our chip, a high density, LQFP, 100 pin STM32F103 C series.

We see from the spec sheet that PIN B5 corresponds to TIM3, which is a clock ticky peripheral.

At this point, we could go learn about that, but let's see how much we can get away with just 
by reading TIM3 values in gdb! It's a memory-mapped peripheral system after all, and there should
be no hidden variables. If TIM3 does stuff, let's see if we can learn just by watching it.

Cool. So boot up the stock OS and go into the Contrast change mode.
Let's spy on TIM3

```
display/32xw 0x40000400
```
When we start with the contrast value at 50
```
Contrast 50:
TIM3
                    0                 4              8               c
0x40000400:	0x00000081	0x00000000	0x00000000	0x00000000
0x40000410:	0x0000001f	0x00000000	0x00006800	0x00000000
0x40000420:	0x00000010	[0x00000141]	0x00000000	0x000001df
0x40000430:	0x00000000	0x00000000	0x00000155	0x00000000
0x40000440:	0x00000000	0x00000000	0x00000000	0x00000081
```
where [] seems to go up and down as we stepi, 
no matter what else is happening in the system.
```
Contrast 99:
TIM3
0x40000400:	0x00000081	0x00000000	0x00000000	0x00000000
0x40000410:	0x0000001f	0x00000000	0x00006800	0x00000000
0x40000420:	0x00000010	[0x000000df]	0x00000000	0x000001df
0x40000430:	0x00000000	0x00000000	<0x000001de>	0x00000000
0x40000440:	0x00000000	0x00000000	0x00000000	0x00000081
```
Where [] keep changing, and <> seems permanently changed, and associated
with the changed contrast.

Contrast 00: makes that <> value = 0x000000c8, but everything else behaves the same.

Ok interesting. So some values are set in TIM3, one value seems to change directly
with when we spin the contrast knob, and one value just ticks up and down in a circle
regardless of what we do. Let's go check the address maps in Ghidra and see if we can make sense of
it.
Consulting the SVD file, w.r.t. TIM3 ->
```
40000400 = CR1, this is set to 0x81
40000410 = SR , this is set to 0x1f
40000418 = CCMR_Input1, this is set to 0x00006800
40000420 = CCER, this is set to 0x10
40000424 = CNTR (counter), this is going all over the place, even during debug mode. Not sure we set it.
40000428 = Prescaler. Not set to anything. But we note it because it's mentioned alot in PWM docs.
4000042c = ARR, this is set to 0x000001df
40000438 = CCR2, this is what seems to go between 0x000000c8 - 0x000001de, with 0x155 the center.
```
CCR2 the configurable parameter.

Cool, just by watching and skipping learning absolutely everything about pulse width modulation,
we have learned 95% of what it takes to drive the contrast. In fact, this is our basic contrast function in the operating system!
We skips tons of HALs and system libraries and confusion, and just cut straight to the heart of the matter.
```
void contrast(){
  TIM3 -> CR1 = 0x81;
  TIM3 -> SR  = 0x1f;
  TIM3 -> CCMR1 = 0x00006800;
  TIM3 -> CCER = 0x10;
  TIM3 -> ARR  = 0x000001df;
  TIM3 -> CCR2 = 0x00000155; // This is the contrast value.
  TIM3 -> DMAR = 0x00000081;
}
```
7 lines is cool.

When you boot up an operating system with that function called, things seem pretty good.
Our TIM3 values look just like the TIM3 values in the stock operating system, except
Pin B5 is not flickering the way we'd expect. Contrast is 0. How do we hook TIM3
to Pin B5? 

A little internet investigation reveals
```
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
 ```
 
 That plus our contrast function, we're in business! We now have all the ingredients to write a full LCD driver.
 
 Now, we skipped past a lot of learning and understanding by just watching the TIM3 values and trying to clone them.
 
 Let's spend a moment understanding what just happened. We set 7 pointers in TIM3
 CR1, SR, CCMR1, CCER, ARR, CCR2, and DMAR.
 
 Let's consult [Page 35 of the STM32 Timer Cookbook] (https://www.st.com/resource/en/application_note/dm00236305-generalpurpose-timer-cookbook-for-stm32-microcontrollers-stmicroelectronics.pdf).
