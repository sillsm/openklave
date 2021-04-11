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

The STM32F103 runs an instruction called ARM Cortex M3. The chip is 32-bit and little-endian, which will be important for setting up Ghidra and sending values to it later.

#### Accompanying software

We assume you have installed:

1. Ghidra, for decompiling the system firmware and annotating it.
2. OpenOCD, for connecting your laptop debugger to the microcontroller and setting up a gdb instance.
3. GDB, for debugging.

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
