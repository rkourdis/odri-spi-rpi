#!/usr/bin/env python3

#Thomas Flayols - feb 2022
#https://github.com/thomasfla/odri-spi-rpi
#https://github.com/open-dynamic-robot-initiative/master-board/blob/master/documentation/BLMC_%C2%B5Driver_SPI_interface.md

# Modified for linear control and multiple SPIs - Rafael Kourdis

import math
import copy
import time
import struct
import spidev
import statistics

from math import pi
from termcolor import colored
import multiprocessing as multiproc

def crc32(buf):
    crc=0xffffffff

    for val in buf:
        crc ^= val << 24
        for _ in range(8):
            crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7

    return crc

def checkcrc(buf):
    crc = crc32(buf[:-4])
    return (crc & 0xffff == buf[-4] * 256 + buf[-3] and (crc & 0xffff0000) >> 16 == buf[-2] * 256 + buf[-1])

class SPIuDriver:
    def __init__(self, connected_boards = 1, waitForInit = True, absolutePositionMode = False, spi_bus = 0):
        self.connected_boards = connected_boards

        # On RPi5, we have to use https://github.com/waveform80/rpi-lgpio
        # instead of RPi.GPIO.

        # We rely on the hardware for CS. It seems that the delay for the uDriver DMA
        # to transfer the sensor packet is enough.

        #Initialise SPI
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, 0)
        self.spi.mode=0
        self.spi.max_speed_hz = 16000000

        # Allocate all variables
        self.is_system_enabled = 0
        self.error_code = 0

        self.position    = [0, 0]
        self.velocity    = [0, 0]
        self.current     = [0, 0]
        self.is_enabled  = [0, 0]
        self.is_ready    = [0, 0]
        self.iSatCurrent = [5.0, 5.0]

        self.has_index_been_detected = [0, 0]
        self.index_toggle_bit        = [0, 0]
        self.EIOC                    = [1] * 2 if absolutePositionMode else [0] * 2

        self.alpha = [0.] * (2 * 4 * self.connected_boards)
        self.beta  = [0., 0.]

        self.timeout = 0
        self.error   = -1

        # Wait for system enable:
        if waitForInit:
            print(">> Calibrating motor, please wait")
            while(not self.is_ready[0]):
                self.transfer()
                time.sleep(0.001)

        if absolutePositionMode:
            if (not self.has_index_been_detected[0] or not self.has_index_been_detected[1]):
                print(">> Waiting for index pulse to have absolute position reference, please move the motors manualy")
                displayedIndex0 = False
                displayedIndex1 = False

                while(not self.has_index_been_detected[0] or not self.has_index_been_detected[1]):
                    self.transfer()
                    if self.has_index_been_detected[0] == True and displayedIndex0 == False:
                        print (" >> Index 0 detected!")
                        displayedIndex0 = True

                    if self.has_index_been_detected[1] == True and displayedIndex1 == False:
                        print (" >> Index 1 detected!")
                        displayedIndex1 = True

                    time.sleep(0.001)

        print ("Ready!")

    def transfer(self):
        # Generate command packet
        ES      = 1
        EM1     = 1
        EM2	    = 1
        EPRE    = 1
        EI1OC   = self.EIOC[0]
        EI2OC	= self.EIOC[1]
        mode    = (ES << 7) | (EM1 << 6) | (EM2<<5)|(EPRE<<4)|(EI1OC<<3)|(EI2OC<<2)
        timeout = self.timeout

        rawIsat0 = int(self.iSatCurrent[0] * (1 << 3))
        rawIsat1 = int(self.iSatCurrent[1] * (1 << 3))

        header_values = [
                # Mode + Timeout 16 bits:
                mode,
                timeout,

                # Saturation current 0 + 1 uint16_t:
                rawIsat0,
                rawIsat1,

                # Index (uint16_t):
                0,
         ]

        # We pack these values in big-endian *byte* mode. The C2000
        # is little-endian, but for _words_ of 16 bits. The header does
        # not contain any 32 bit values.
        header_bytes = struct.pack("> BB BB H", *header_values)

        # Floats should be packed in big-endian byte mode,
        # but we'll need to manually flip the words so that we get
        # them in little-endian word mode:
        floats = self.alpha + self.beta

        floats_bytes = struct.pack(f"> {2 * 4 * self.connected_boards + 2}f", *floats)
        floats_word_swapped = b''.join(
            floats_bytes[f_idx*4 + 2 : f_idx*4 + 4] + floats_bytes[f_idx*4 + 0 : f_idx*4 + 2]
            for f_idx in range(len(floats))
        )

        commandPacket = bytearray(
            header_bytes +
            b'\x00' * (len(header_bytes) % 4) +    # 32-bit values are aligned on 32-bit boundaries
            floats_word_swapped +
            struct.pack(f"I", 0)                   # Temporary CRC (uint32_t)
        )

        crc = crc32(commandPacket[:-4])
        commandPacket[-4] =(crc>>24) & 0xff
        commandPacket[-3] =(crc>>16) & 0xff
        commandPacket[-2] =(crc>>8) & 0xff
        commandPacket[-1] =(crc) & 0xff

        # Trim sensor packet to 17 words due to the fact that the SPI TX buffer
        # is limited and bytes sent after (because the command is larger) will be garbage:
        sensorPacket = bytearray(self.spi.xfer(commandPacket))[:34]

        # print("Command: ", " ".join(format(x, "02x") for x in commandPacket))
        # print("Sensor:    ", " ".join(format(x, "02x") for x in sensorPacket))
        # print(checkcrc(sensorPacket))
        # print()

        if not checkcrc(sensorPacket):
            raise Exception(f"Error: Corrupted sensor frame - is uDriver powered on?")

        # Decode received sensor packet
        data = struct.unpack(">H H i i h h h h xxxxxxxxxxxxxx", sensorPacket)
        self.is_system_enabled          = data[0]&0b1000000000000000 != 0
        self.is_enabled[0]              = data[0]&0b0100000000000000 != 0
        self.is_ready[0]                = data[0]&0b0010000000000000 != 0
        self.is_enabled[1]              = data[0]&0b0001000000000000 != 0
        self.is_ready[1]                = data[0]&0b0000100000000000 != 0
        self.has_index_been_detected[0] = data[0]&0b0000010000000000 != 0
        self.has_index_been_detected[1] = data[0]&0b0000001000000000 != 0

        self.error       = data[0]&0b0000000000001111
        self.position[0] = data[2] / (1<<24) * 2.0 * pi
        self.position[1] = data[3] / (1<<24) * 2.0 * pi
        self.velocity[0] = data[4] / (1<<11) * 2000*pi/60.0
        self.velocity[1] = data[5] / (1<<11) * 2000*pi/60.0
        self.current[0]  = data[6] / (1<<10)
        self.current[1]  = data[7] / (1<<10)

        if self.error!=0:
            raise Exception(f"Error from motor driver: Error {self.error}")

    def stop(self):
        self.EIOC[0] = self.EIOC[1] = 0

        self.alpha = [0.] * (2 * 4 * self.connected_boards)
        self.beta  = [0., 0.]

        self.iSatCurrent[0] = self.iSatCurrent[1] = 0
        self.timeout = 0

        dt=0.001
        t = time.perf_counter()

        for _ in range(2):
            self.transfer()

            t += dt
            while(time.perf_counter() < t):
                pass

class SharedCommand:
    def __init__(self, connected_boards: int):
        self.alpha       = multiproc.RawArray("f",  2 * 4 * connected_boards)
        self.beta        = multiproc.RawArray("f",  2)
        self.iSatCurrent = multiproc.RawArray("f",  2)
        
class ParalleluDriver:
    @classmethod
    def _subprocess_loop(_, flags, state, command, spi_args,):
        ud = SPIuDriver(**spi_args)
        flags["ready"].value = True

        _outer_quit = False

        try:
            while True:
                if flags["quit"].value:
                    _outer_quit = True
                    raise Exception()

                if not flags["transfer"].value:
                    continue

                ud.alpha       = command.alpha[:]
                ud.beta        = command.beta[:]
                ud.iSatCurrent = command.iSatCurrent[:]

                ud.transfer()
                x_current = ud.position + ud.velocity

                state[:] = x_current
                flags["transfer"].value = False

        except (KeyboardInterrupt, Exception) as ex:
            ud.stop()
            print(colored(f"Quit bus {spi_args['spi_bus']}.", "yellow"))

            if not _outer_quit and not isinstance(ex, KeyboardInterrupt):
                raise

    def __init__(self, **kwargs):
        self.synched_flags = {
            "quit":     multiproc.Value('b', False),     # Stop SPI subprocess
            "transfer": multiproc.Value('b', False),     # Trigger SPI transfer, False when done
            "ready":    multiproc.Value('b', False),     # Board ready
        }

        self.state   = multiproc.RawArray('f', 4)        # NOTE: These aren't locked
        self.command = SharedCommand(kwargs["connected_boards"])

        self.proc = multiproc.Process(
            target = self._subprocess_loop,
            kwargs = {
                "spi_args": kwargs,
                "flags": self.synched_flags,
                "command": self.command,
                "state": self.state,
            }
        )

        self.proc.start()

        # Block until the board is ready:
        while True:
            if not self.alive:
                raise Exception("Could not initialize board!")

            if self.synched_flags["ready"].value:
                break

    @property
    def alive(self):
        return self.proc.is_alive()

    def stop(self):
        self.synched_flags["quit"].value = True