#!/usr/bin/env python3
'''
  Ported to Python 3
  Based on Bastille Networks Logitech USB Flash tool
'''

import sys
import logging
import struct
from unifying import unifying_dongle

# Setup logging
logging.basicConfig(level=logging.INFO, format='[%(asctime)s.%(msecs)03d]  %(message)s', datefmt="%Y-%m-%d %H:%M:%S")

# Compute CRC-CCITT over 1 byte
def crc_update(crc, data):
    crc ^= (data << 8)
    for x in range(8):
        if (crc & 0x8000) == 0x8000:
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF
        else:
            crc <<= 1
    crc &= 0xFFFF
    return crc

# Make sure a firmware image path was passed in
if len(sys.argv) < 3:
    print("Usage: sudo ./logitech-usb-flash.py [firmware-image.bin] [firmware-image.ihx]")
    sys.exit(1)

# Compute the CRC of the firmware image
logging.info("Computing the CRC of the firmware image")
path = sys.argv[1]
with open(path, 'rb') as f:
    data = f.read()

crc = 0xFFFF
for byte in data:
    crc = crc_update(crc, byte)

# Read in the firmware hex file
logging.info("Preparing USB payloads")
path_hex = sys.argv[2]
payloads = []

with open(path_hex, 'r') as f:
    lines = f.readlines()
    for line in lines:
        line = line.strip()
        if not line.startswith(':'): continue
        
        # Parse hex line: skip ':' and checksum at the end
        content = line[1:]
        
        # Original logic: line[2:6] + line[0:2] + line[8:-2]
        # In IHX: [count(2)][addr(4)][type(2)][data(N)][checksum(2)]
        transformed = content[2:6] + content[0:2] + content[8:-2]
        transformed = "20" + transformed
        transformed = transformed.ljust(64, '0') # 32 bytes = 64 hex chars
        
        # Convert hex string to bytes
        payloads.append(bytes.fromhex(transformed))

# Special adjustment for the first payload (as in original script)
if payloads:
    first = list(payloads[0])
    first[2] = (first[2] + 1) & 0xFF
    first[3] = (first[3] - 1) & 0xFF
    payloads[0] = bytes(first)

# Add the firmware CRC payload
crc_payload = b'\x20\x67\xFE\x02' + struct.pack('!H', crc) + b'\x00'*26
payloads.append(crc_payload)

# Instantiate the dongle
try:
    dongle = unifying_dongle()
except Exception as e:
    logging.error(f"Error initializing dongle: {e}")
    sys.exit(1)

# Init command
logging.info("Initializing firmware update")
dongle.send_command(0x21, 0x09, 0x0200, 0x0000, b"\x80" + b"\x00"*31)

# Clear existing flash memory
logging.info("Clearing existing flash memory up to bootloader")
for x in range(0, 0x70, 2):
    # chr(x) changed to bytes([x])
    clear_cmd = b"\x30" + bytes([x]) + b"\x00\x01" + b"\x00"*28
    dongle.send_command(0x21, 0x09, 0x0200, 0x0000, clear_cmd)

# Send the data
logging.info("Transferring the new firmware")
for payload in payloads:
    dongle.send_command(0x21, 0x09, 0x0200, 0x0000, payload)

# Re-send first payload (as per original tool logic)
dongle.send_command(0x21, 0x09, 0x0200, 0x0000, payloads[0])

# Completed command
logging.info("Mark firmware update as completed")
dongle.send_command(0x21, 0x09, 0x0200, 0x0000, b"\x20\x00\x00\x01\x02" + b"\x00"*27)

# Restart the dongle
logging.info("Restarting dongle into research firmware mode")
dongle.send_command(0x21, 0x09, 0x0200, 0x0000, b"\x70" + b"\x00"*31)
