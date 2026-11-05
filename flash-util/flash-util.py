#!/usr/bin/env python3

import time
import serial
import sys, getopt
import os.path

from os import path

import binascii


# Number of 1 KB flash blocks (default: uPD78F0534 with 48 KB flash).
# Can be changed with -n/--blocks, e.g. 32 for uPD78F0513.
DEFAULT_N_BLOCKS = 48
nBlocks = DEFAULT_N_BLOCKS

# Flash mode entry timing (seconds)
RESET_HOLD_TIME = 0.5            # RESET low (FLMD0 already high) before releasing RESET
T_RFCF = 0.002                   # RESET high -> first FLMD0 pulse (1..10 ms)
FLMD0_PULSES = 3                 # 3 pulses = UART6 (TxD6/RxD6), verified on uPD78F0534; FLMD0 must be high at RESET release
FLMD0_PULSE_WIDTH = 0.000150     # pulse high/low width (10..500 us)
FLASH_MODE_SETTLE_TIME = 0.4     # FLMD0 high -> first 00h sync byte

StatusList = {
    0x00: "No data",
    0x04: "Command number error",
    0x05: "Parameter error",
    0x06: "Normal acknowledgment (ACK)",
    0x07: "Checksum error",
    0x0F: "Verify error",
    0x10: "Protect error", 
    0x15: "Negative acknowledgment (NACK)", 
    0x1A: "MRG10 error",
    0x1B: "MRG11 error",
    0x1C: "Write error",
    0x20: "Read error",
    0xFF: "Processing in progress (BUSY)",
}


BLOCK_SIZE = 1024
FRAME_SIZE = 256                 # max. payload of one data frame
DATA_TIMEOUT = 5                 # seconds to wait for the status of a data frame


def LoadImage(file):
    # Read a binary image and pad it with 0xFF to a multiple of the block
    # size (0xFF = erased flash, so padding does not change anything).
    with open(file, 'rb') as f:
        data = f.read()
    if len(data) % BLOCK_SIZE:
        data += b'\xff' * (BLOCK_SIZE - len(data) % BLOCK_SIZE)
    return data


def StatusText(status):
    return StatusList.get(status, "Unknown status 0x%02X" % status)


def add_crc(b, size):
        crc = 0x00 # initial value

        for x in range(size + 1):
            crc = (crc - b[x+1]) % 256
        return crc

class Programmer(object):

    def __init__(self):
        print("NEC Programmer v1.0")
        print("written by Martin Jansson\n\n")

    def SerialInit(self, serial_port):
        print("Requesting serial port and resetting chip...")
        # Configure the port before opening it, so DTR/RTS already have the
        # desired level when the port is opened (no glitch on RESET/FLMD0).
        # rtscts/dsrdtr must be off to keep manual control over the lines.
        self.ser = serial.Serial()
        self.ser.port = serial_port
        self.ser.baudrate = 9600
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.timeout = 1
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.dtr = True  # RESET low
        self.ser.rts = True  # FLMD0 low
        self.ser.open()

        self.EnterFlashMode()

        self.ser.write(bytearray([0x00]))
        time.sleep(11/1000)
        self.ser.write(bytearray([0x00]))
        time.sleep(11/1000)

        return

    def EnterFlashMode(self):
        # Line mapping (inverted by the RS232/USB-UART adapter):
        #   dtr = True  -> RESET = 0V  (reset active)
        #   dtr = False -> RESET = VDD (reset released)
        #   rts = True  -> FLMD0 = 0V
        #   rts = False -> FLMD0 = VDD
        print("Entering flash programming mode (FLMD0 pulses)...")

        # Step 1: RESET low, FLMD0 high. FLMD0 is sampled when RESET is
        # released: 0V -> normal operation (application), VDD -> flash
        # programming mode. So FLMD0 must already be high here.
        self.ser.dtr = True
        self.ser.rts = False
        time.sleep(RESET_HOLD_TIME)

        # Step 2: release RESET with FLMD0 high -> flash programming mode
        self.ser.dtr = False

        # Step 3: wait tRFCF (1..10 ms), then send FLMD0 pulses
        time.sleep(T_RFCF)
        for _ in range(FLMD0_PULSES):
            self.ser.rts = False  # FLMD0 high
            time.sleep(FLMD0_PULSE_WIDTH)
            self.ser.rts = True   # FLMD0 low
            time.sleep(FLMD0_PULSE_WIDTH)

        # Step 4: keep FLMD0 high for the rest of the session
        self.ser.rts = False

        time.sleep(FLASH_MODE_SETTLE_TIME)
        return

    def ReceiveFrame(self):
        # Status frame (sent by the chip as a data frame):
        #   STX, LEN, DATA[LEN], SUM, ETX
        #   SUM = 00h - LEN - DATA[0] - ... (mod 256)
        # Returns the data bytes, or None on timeout / malformed frame.
        b = self.ser.read(2) # read 2 bytes to determine length

        if len(b) < 2:
            print("error: Length less than 2")
            return None
        length = b[1] if b[1] != 0 else 256
        b += self.ser.read(length + 2)
        if len(b) < length + 4:
            print("error: Incomplete response:", b.hex())
            return None
        if b[0] != 0x02 or b[-1] != 0x03:
            print("error: Invalid frame (STX/ETX):", b.hex())
            return None
        if add_crc(b, length) != b[-2]:
            print("error: Checksum error in response:", b.hex())
            return None

        return bytes(b[2:2 + length])

    def Receive(self):
        # Receive a status frame and return its first status byte (ST1).
        data = self.ReceiveFrame()
        if data is None:
            return None
        print("Response: ", StatusText(data[0]))
        return data[0]

    def ExitFlashMode(self):
        # Leave flash programming mode: FLMD0 low, then pulse RESET so the
        # chip restarts into the normal application.
        print("Leaving flash mode (FLMD0 low, RESET pulse)...")
        self.ser.rts = True   # FLMD0 low
        time.sleep(0.01)
        self.ser.dtr = True   # RESET low
        time.sleep(0.1)
        self.ser.dtr = False  # RESET released with FLMD0 low -> application
        time.sleep(0.01)
        return
    
    def Reset(self):
        b = bytearray()

        b.append(0x01)      # Start byte
        b.append(0x01)      # Length
        b.append(0x00)      # Command
        b.append(add_crc(b, b[1])) # crc
        b.append(0x03)      # Stop byte

        self.ser.write(b)
        return self.Receive()

    def BlankCheckBlock(self, block):

        start_address = block * 1024
        end_address = ((block + 1) * 1024) -1
        b = bytearray()

        b.append(0x01)   # Start byte
        b.append(0x07)   # Length
        b.append(0x32)   # Command

        b.append(start_address >> 16 & 0xFF)
        b.append(start_address >> 8 & 0xFF)
        b.append(start_address & 0xFF)
        b.append(end_address >> 16 & 0xFF)
        b.append(end_address >> 8 & 0xFF)
        b.append(end_address & 0xFF)

        b.append(add_crc(b, b[1])) # crc
        b.append(0x03) # stop byte
        self.ser.write(b)

        # Returns "empty", "not empty" or "error (<status>)"
        status = self.Receive()
        if status == 0x06:
            print("Block empty")
            return "empty"
        if status == 0x1B:  # MRG11 error = block is not blank
            print("Block not empty")
            return "not empty"
        if status is None:
            result = "error (no valid response)"
        else:
            result = "error (%s)" % StatusText(status)
        print("Block", block, result)
        return result
    
    def ProgramBlock(self, startBlock, data):
        # Program data (multiple of BLOCK_SIZE) starting at startBlock
        start_address = startBlock * BLOCK_SIZE
        end_address = start_address + len(data) - 1
        b = bytearray()

        b.append(0x01)   # Start byte
        b.append(0x07)   # Length
        b.append(0x40)   # Command (programming)

        b.append(start_address >> 16 & 0xFF)
        b.append(start_address >> 8 & 0xFF)
        b.append(start_address & 0xFF)
        b.append(end_address >> 16 & 0xFF)
        b.append(end_address >> 8 & 0xFF)
        b.append(end_address & 0xFF)

        b.append(add_crc(b, b[1])) # crc
        b.append(0x03) # stop byte

        #print(b.hex())
        print("Sending programming command (0x%05X-0x%05X)..." % (start_address, end_address))
        self.ser.write(b)

        status = self.Receive()
        if status != 0x06:
            print("error: Did not receive ACK, returning...")
            return False

        if not self.SendData(start_address, data, internalVerify=True):
            print("error: Programming failed!")
            return False
        print("Programming succeeded!")
        return True

    def SendData(self, start_address, data, internalVerify=False):
        # Send data as frames of up to 256 bytes:
        #   STX, LEN (00h = 256), DATA, SUM, ETB (more frames follow) / ETX (last)
        # Each frame is answered with a status frame ST1 (reception), ST2
        # (write/verify result). For verify, the uPD78F0534 reports a mismatch
        # only in the status of the last frame (tested), so a verify error
        # applies to the whole range. After the last programming frame the
        # chip sends one more status frame with the internal verify result.
        nFrames = (len(data) + FRAME_SIZE - 1) // FRAME_SIZE
        oldTimeout = self.ser.timeout
        self.ser.timeout = DATA_TIMEOUT
        try:
            for i in range(nFrames):
                chunk = data[i * FRAME_SIZE:(i + 1) * FRAME_SIZE]
                last = (i == nFrames - 1)
                b = bytearray([0x02, len(chunk) & 0xFF])
                b.extend(chunk)
                b.append(add_crc(b, len(chunk)))
                b.append(0x03 if last else 0x17)
                addr = start_address + i * FRAME_SIZE
                print("Sending data frame %d/%d (0x%05X)..." % (i + 1, nFrames, addr))
                self.ser.write(b)
                self.ser.flush()

                st = self.ReceiveFrame()
                if st is None:
                    print("error: No valid status for frame at 0x%05X" % addr)
                    return False
                if st[0] != 0x06:
                    print("error: Frame at 0x%05X not accepted: %s" % (addr, StatusText(st[0])))
                    return False
                if len(st) > 1 and st[1] != 0x06:
                    if last:
                        print("error: %s in range 0x%05X-0x%05X" % (StatusText(st[1]), start_address, start_address + len(data) - 1))
                    else:
                        print("error: Frame at 0x%05X failed: %s" % (addr, StatusText(st[1])))
                    return False

            if internalVerify:
                print("Waiting for internal verify...")
                st = self.ReceiveFrame()
                if st is None:
                    print("error: No internal verify status received (data may still be written, check with -v)")
                    return False
                if st[0] != 0x06:
                    print("error: Internal verify failed: %s" % StatusText(st[0]))
                    return False
            return True
        finally:
            self.ser.timeout = oldTimeout

    def BlockErase(self, block):
        start_address = block * 1024
        end_address = ((block + 1) * 1024) -1
        b = bytearray()

        b.append(0x01)   # Start byte
        b.append(0x07)   # Length
        b.append(0x22)   # Command (block erase)

        b.append(start_address >> 16 & 0xFF)
        b.append(start_address >> 8 & 0xFF)
        b.append(start_address & 0xFF)
        b.append(end_address >> 16 & 0xFF)
        b.append(end_address >> 8 & 0xFF)
        b.append(end_address & 0xFF)

        b.append(add_crc(b, b[1])) # crc
        b.append(0x03) # stop byte

        print("Sending erase block command..")
        self.ser.write(b)

        status = self.Receive()
        if status != 0x06:
            print("error: Did not receive ACK, returning...")
            return False
        print("Block erased.") 
        return True


    def ChipErase(self):
        b = bytearray()

        b.append(0x01)   # Start byte
        b.append(0x01)   # Length
        b.append(0x20)   # Command (chip erase)

        b.append(add_crc(b, b[1])) # crc
        b.append(0x03) # stop byte

        print("Sending erase chip command..")
        self.ser.write(b)

        status = self.Receive()
        if status != 0x06:
            print("error: Did not receive ACK, returning...")
            return False
        print("Chip erased.") 
        return True
    
    def VerifyBlock(self, startBlock, data):
        # Verify data (multiple of BLOCK_SIZE) starting at startBlock
        start_address = startBlock * BLOCK_SIZE
        end_address = start_address + len(data) - 1
        b = bytearray()

        b.append(0x01)   # Start byte
        b.append(0x07)   # Length
        b.append(0x13)   # Command (verify)

        b.append(start_address >> 16 & 0xFF)
        b.append(start_address >> 8 & 0xFF)
        b.append(start_address & 0xFF)
        b.append(end_address >> 16 & 0xFF)
        b.append(end_address >> 8 & 0xFF)
        b.append(end_address & 0xFF)

        b.append(add_crc(b, b[1])) # crc
        b.append(0x03) # stop byte

        #print(b.hex())
        print("Sending verify command (0x%05X-0x%05X).." % (start_address, end_address))
        self.ser.write(b)

        status = self.Receive()
        if status != 0x06:
            print("error: Did not receive ACK, returning...")
            return False

        if not self.SendData(start_address, data):
            print("error: Verification failed!")
            return False
        print("Verification OK!")
        return True


def usage():
    print("usage: -h, --help                    Print this help")
    print("       -p, --port        <port>      Specify serial port")
    print("       -b, --block       <block>     Block to erase, or start block for flash/verify")
    print("       -f, --flash       <file>      Flash binary file (requires -b, length = file size)")
    print("       -e, --erase                   Erase chip, or only the block given with -b")
    print("       -v, --verify      <file>      Verify binary file (requires -b, length = file size)")
    print("       -c, --checkempty              Check if chip/block is empty")
    print("       -n, --blocks      <count>     Number of 1 KB flash blocks (default: %d)" % DEFAULT_N_BLOCKS)
    return

def main():
    global nBlocks
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hp:f:ecb:v:n:", ["help", "port=", "flash=", "erase", "checkempty", "block=", "verify=", "blocks="])
    except getopt.GetoptError as err:
        # print help information and exit:
        print(err)  # will print something like "option -a not recognized"
        usage()
        sys.exit(2)
    port = "COM1"
    flashFile = None
    verifyFile = None
    block = 0
    blockSpecified = False

    flash = False
    erase = False
    verify = False
    checkEmpty = False
    for o, a in opts:
        if o in ("-v", "--verify"):
            verify = True
            verifyFile = a
        elif o in ("-h", "--help"):
            usage()
            sys.exit()
        elif o in ("-p", "--port"):
            port = a
            print("setting port to: ", port)
        elif o in ("-b", "--block"):
            blockSpecified = True
            try:
                block = int(a)
            except ValueError:
                print("invalid block: ", a)
                sys.exit(2)
            print("specified block: ", block)
        elif o in ("-n", "--blocks"):
            try:
                count = int(a)
            except ValueError:
                count = 0
            if count <= 0:
                print("invalid block count: ", a)
                usage()
                sys.exit(2)
            nBlocks = count
            print("number of blocks: ", nBlocks)
        elif o in ("-f", "--flash"):
            flash = True
            flashFile = a
        elif o in ("-e", "--erase"):
            erase = True
        elif o in ("-c", "--checkempty"):
            checkEmpty = True
        else:
            assert False, "unhandled option"
    
    if flash or erase or verify or checkEmpty:
        if blockSpecified and not 0 <= block < nBlocks:
            print("block %d out of range (0..%d)" % (block, nBlocks - 1))
            sys.exit(2)
        if (flash or verify) and not blockSpecified:
            print("flash/verify require the start block (-b)")
            sys.exit(2)

        images = {}
        for f in (flashFile, verifyFile):
            if f is None:
                continue
            if not path.isfile(f):
                print("missing input file: ", f)
                sys.exit(2)
            images[f] = LoadImage(f)
            count = len(images[f]) // BLOCK_SIZE
            if count == 0:
                print("input file is empty: ", f)
                sys.exit(2)
            if block + count > nBlocks:
                print("%s needs blocks %d..%d, but flash has only %d blocks" % (f, block, block + count - 1, nBlocks))
                sys.exit(2)

        foo = Programmer()
        foo.SerialInit(port)
        ok = True
        crashed = False
        try:
            time.sleep(500/1000) # 500 ms sleep
            if foo.Reset() != 0x06:
                print("error: No ACK from bootloader, aborting.")
                ok = False
                return
            time.sleep(500/1000) # 500 ms sleep

            if erase:
                if not blockSpecified:
                    print("\nErasing chip...\n")
                    if not foo.ChipErase():
                        ok = False
                        return
                else:
                    print("\nErasing block", block, "...\n")
                    if not foo.BlockErase(block):
                        ok = False
                        return
            if flash:
                print("\nFlashing chip...\n")
                if not foo.ProgramBlock(block, images[flashFile]):
                    ok = False
                    return

            if verify:
                print("\nVerifying chip...\n")
                if not foo.VerifyBlock(block, images[verifyFile]):
                    ok = False
                    return

            if checkEmpty:
                print("\nGetting empty blocks...\n")
                blockStates = []
                for x in range(nBlocks):
                    blockStates.append(foo.BlankCheckBlock(x))

                for x in range(nBlocks):
                    print("Block", x, blockStates[x])

                if any(s.startswith("error") for s in blockStates):
                    print("error: Blank check failed for some blocks.")
                    ok = False
                    return
        except BaseException:
            crashed = True
            raise
        finally:
            # Always leave the chip running its application again
            foo.ExitFlashMode()
            foo.ser.close()
            if not ok and not crashed:
                sys.exit(2)

    
    

if __name__ == "__main__":
   main()

