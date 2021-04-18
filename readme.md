# OpenKlave

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

