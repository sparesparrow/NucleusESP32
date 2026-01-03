#!/usr/bin/env python3
'''
  Copyright (C) 2016 Bastille Networks
  Ported to Python 3 for sparrow-ai-tech
'''

import usb.core
import usb.util
import time
import sys
import array
import logging

# Nastavení logování
logging.basicConfig(level=logging.INFO, format='[%(asctime)s.%(msecs)03d]  %(message)s', datefmt="%Y-%m-%d %H:%M:%S")

# USB timeout
usb_timeout = 2500

# Kontrola argumentů
if len(sys.argv) < 2:
    print('Usage: sudo python3 usb-flash.py path-to-firmware.bin')
    sys.exit(1)

# Načtení firmwaru
try:
    with open(sys.argv[1], 'rb') as f:
        data = bytearray(f.read())
except FileNotFoundError:
    logging.error(f"Soubor {sys.argv[1]} nebyl nalezen.")
    sys.exit(1)

# Zarovnání dat na násobky 512 bajtů (velikost stránky)
padding_size = (512 - len(data) % 512) % 512
data += b'\x00' * padding_size

# 1. Pokus o nalezení zařízení a přepnutí do Nordic bootloaderu
logging.info("Looking for a compatible device that can jump to the Nordic bootloader")
product_ids = [0x0102, 0x7777]
dongle = None

for product_id in product_ids:
    dongle = usb.core.find(idVendor=0x1915, idProduct=product_id)
    if dongle:
        try:
            dongle.set_configuration()
            logging.info(f"Device found (0x{product_id:04x}), jumping to the Nordic bootloader")

            if product_id == 0x0102:
                # Příkaz pro skok do bootloaderu u research firmwaru
                dongle.write(0x01, [0xFF], timeout=usb_timeout)
            else:
                # Příkaz pro RFStorm
                dongle.ctrl_transfer(0x40, 0xFF, 0, 0, (), timeout=usb_timeout)

            try:
                dongle.reset()
            except:
                pass
            break
        except usb.core.USBError as e:
            logging.error(f"Could not communicate with device 0x{product_id:04x}: {e}")

# 2. Hledání samotného Nordic bootloaderu (ID 0101)
logging.info("Looking for a device running the Nordic bootloader")
dongle = None
start = time.time()
while time.time() - start < 2: # Prodloužen timeout na 2s pro stabilitu
    dongle = usb.core.find(idVendor=0x1915, idProduct=0x0101)
    if dongle:
        try:
            dongle.set_configuration()
            break
        except:
            continue
    time.sleep(0.1)

if not dongle:
    logging.error("No compatible device found in bootloader mode.")
    sys.exit(1)

# 3. Zápis dat po stránkách
logging.info("Writing image to flash")
page_count = len(data) // 512

for page in range(page_count):
    # Inicializace zápisu stránky
    dongle.write(0x01, [0x02, page], timeout=usb_timeout)
    dongle.read(0x81, 64, timeout=usb_timeout)

    # Zápis stránky v 8 blocích po 64 bajtech
    for block in range(8):
        offset = page * 512 + block * 64
        block_write = data[offset : offset + 64]
        dongle.write(0x01, block_write, timeout=usb_timeout)
        dongle.read(0x81, 64, timeout=usb_timeout)

# 4. Verifikace zapsaných dat
logging.info("Verifying write")
block_number = 0
for page in range(page_count):
    # Příkaz pro čtení z dolních 16KB flash
    dongle.write(0x01, [0x06, 0], timeout=usb_timeout)
    dongle.read(0x81, 64, timeout=usb_timeout)

    for block in range(8):
        # Čtení bloku
        dongle.write(0x01, [0x03, block_number], timeout=usb_timeout)
        # V Pythonu 3 vrací .read() přímo array/bytes
        block_read = dongle.read(0x81, 64, timeout=usb_timeout)

        expected = data[block_number * 64 : block_number * 64 + 64]
        if bytes(block_read) != bytes(expected):
            raise Exception(f'Verification failed on page {page}, block {block}')
        block_number += 1

logging.info("Firmware programming completed successfully")
logging.info("\033[92m\033[1mPlease unplug your dongle and plug it back in.\033[0m")
