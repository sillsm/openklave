#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#
# This file is a verbatim copy of sendsysex.py, from the RTMIDI project examples.
# Released under a modified MIT license and available here:
# https://github.com/khamaileon/python-rtmidi/blob/master/examples/sendsysex.py
#
"""Send all system exclusive files given on the command line.

The paths given on the command line can also contain directories and all files
with a *.syx extension in them will be sent (in alphabetical order).

All consecutive sysex messages in each file will be sent to the chosen MIDI
ouput, after confirmation (which can be turned off).

"""

__program__ = 'sendsysex.py'
__version__ = '1.1 ($Rev$)'
__author__  = 'Christopher Arndt'
__date__    = '$Date$'


import argparse
import logging
import os
import sys
import time

from os.path import basename, exists, isdir, join

import rtmidi

try:
    raw_input
except NameError:
    # Python 3
    raw_input = input


log = logging.getLogger("sendsysex")

SYSTEM_EXCLUSIVE = b'\xF0'
END_OF_EXCLUSIVE = b'\xF7'

starter = [0xF0, 0x47, 0x00, 0x24, 0x70]
m = [ 0x3A, 0x30, 0x32, 0x30, 0x30, 0x30, 0x30,
	0x30, 0x34, 0x30, 0x38, 0x30, 0x30, 0x46, 0x32, 0x0D, 0x0A, 0x3A, 0x31,
	0x30, 0x36, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x34, 0x30,
	0x30, 0x32, 0x30, 0x41, 0x42, 0x36, 0x31, 0x30, 0x30, 0x30, 0x38, 0x45,
	0x46, 0x46, 0x44, 0x30, 0x30, 0x30, 0x38, 0x46, 0x33, 0x45, 0x35, 0x30,
	0x30, 0x30, 0x38, 0x38, 0x34, 0x0D, 0x0A, 0xf7]


def send_sysex_file(filename, midiout, portname, prompt=True, delay=50):
    """Read file given by filename and send all consecutive sysex messages
    found in it to given midiout after prompt.

    """

    with open(filename, 'rb') as sysex_file:
        data = sysex_file.read()
        print("File Length: " + str(len(data)) + " bytes. ")

        try:
            if prompt:
                yn = raw_input("OK?")
        except (EOFError, KeyboardInterrupt):
            print('')
            raise StopIteration

        if not prompt or yn.lower() in ('y', 'yes'):
            chunkSize = 10000
            sysex_msg = data
            # Python 2: convert data into list of integers
            if isinstance(sysex_msg, str):
                print("NOOO")
                sysex_msg = [ord(c) for c in sysex_msg]

            #Now sysex_msg is an array.
            start = 0
            while True:
               if len(sysex_msg) <= start + chunkSize:
                   print("Sending last chunk ")
                   midiout.send_message(bytes(bytearray(starter)) + sysex_msg[start:]+ bytes(bytearray(0xf7)))
                   break
               # normal case
               print("Sending chunk " + str(start))
               #print("First Char " + str(sysex_msg[start]))
               # find an 0A end of ihex piece nearby
               stride = start+chunkSize
               while sysex_msg[stride] != 0x0a:
                   stride = stride - 1
               #print(type(sysex_msg[start:stride]) )
               midiout.send_message(bytes(bytearray(starter)) + sysex_msg[start:stride] + bytes(bytearray(0xf7)))
               start = stride
               time.sleep(.05)


def main(args=None):
    """Main program function.

    Parses command line (parsed via ``args`` or from ``sys.argv``), detects
    and optionally lists MIDI output ports, opens given MIDI output port,
    assembles list of sysex files and calls ``send_sysex_file`` on each of them.

    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(dest='sysexfiles', nargs="*", metavar="SYSEX",
        help='MIDI system exclusive files or directories to send.')
    parser.add_argument('-l',  '--list-ports', action="store_true",
        help='list available MIDI output ports')
    parser.add_argument('-p',  '--port', dest='port', default=0, type=int,
        help='MIDI output port number (default: %(default)s)')
    parser.add_argument('-d',  '--delay', default="50", metavar="MS", type=int,
        help='delay between sending each Sysex message in milliseconds '
        '(default: %(default)s)')
    parser.add_argument('-y',  '--no-prompt', dest='prompt',
        action="store_false", help='do not ask for confirmation before sending')
    parser.add_argument('-v',  '--verbose', action="store_true",
        help='verbose output')

    args = parser.parse_args(args if args is not None else sys.argv[1:])

    logging.basicConfig(format="%(name)s: %(levelname)s - %(message)s",
        level=logging.DEBUG if args.verbose else logging.WARNING)

    try:
        midiout = rtmidi.MidiOut()

        ports = midiout.get_ports()

        if ports:
            if args.list_ports:
                for i, port in enumerate(ports):
                    print("%i: %s" % (i, port))

                return 0

            if args.port < len(ports):
                midiout.open_port(args.port)
                portname = midiout.get_port_name(args.port)
            else:
                log.error("MIDI port number out of range.")
                log.error("Use '-l' option to list MIDI ports.")
                return 2
        else:
            log.error("No MIDI output ports found.")
            return 1

        files = []
        for path in args.sysexfiles or [os.curdir]:
            if isdir(path):
                files.extend(sorted([join(path, fn) for fn in os.listdir(path)
                    if fn.lower().endswith('.syx')]))
            elif exists(path):
                files.append(path)
            else:
                log.error("File '%s' not found.")

        if not files:
            log.warning("No sysex (.syx) files found in given directories or "
                "working directory.")

        for filename in files:
            try:
                send_sysex_file(
                    filename, midiout, portname, args.prompt, args.delay)
            except StopIteration:
                break
            except Exception as exc:
                log.error("Error while sending file '%s': %s", (filename, exc))
    finally:
        midiout.close_port()
        del midiout

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]) or 0)
