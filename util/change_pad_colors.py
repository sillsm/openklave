# Change pad colors on an MPK249
# Erases operating system, preserves factory bootloader
# at 0-0x6000.

import time
import rtmidi

bridge = rtmidi.MidiOut()
bridge.open_virtual_port("My virtual output")
bridge_back = rtmidi.MidiIn()
bridge_back.open_virtual_port("My virtual output")

# Connect to Akai249
midiin = rtmidi.MidiIn()
midiin.open_port(0)

midiout = rtmidi.MidiOut()
midiout.open_port(0)

header = [0xF0, 0x47, 0x00, 0x24]
footer = [0xF7]

base_color = 0x2
pad_address= [0x0a, 0x7c]

midiout.send_message(header + [0x31,0,43,40] +  pad_address+ [base_color]*64 + footer)
# End
