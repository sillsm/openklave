# Python script to wipe keyboard via a Sysex command.
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
note_on = [0x91, 60, 112] # channel 1, middle C, velocity 112
note_off = [0x81, 60, 0]

print("Wiping OS; preserving MPK bootloader.")
header = [0xF0, 0x47, 0x00, 0x24]
footer = [0xF7]
midiout.send_message(header + [0x72,0,0] + footer)
# End
