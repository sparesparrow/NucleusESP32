#!/usr/bin/env python3
'''
  Copyright (C) 2016 Bastille Networks
  Ported to Python 3 for sparrow-ai-tech
'''

import time, logging, sys
from lib import common

# Inicializace argumentů a rádia
common.init_args('./nrf24-scanner.py')
common.parser.add_argument('-p', '--prefix', type=str, help='Promiscuous mode address prefix', default='')
common.parser.add_argument('-d', '--dwell', type=float, help='Dwell time per channel, in milliseconds', default=100)
common.parse_and_init()

# Parsování prefixu adresy (Python 3 bytes management)
try:
    prefix_address = bytes.fromhex(common.args.prefix.replace(':', ''))
except ValueError:
    print(f"Error: Invalid hex prefix format: {common.args.prefix}")
    sys.exit(1)

if len(prefix_address) > 5:
    raise Exception('Invalid prefix address: {0}'.format(common.args.prefix))

# Přepnutí rádia do promiskuitního módu
common.radio.enter_promiscuous_mode(prefix_address)

# Převod dwell time z milisekund na sekundy
dwell_time = float(common.args.dwell) / 1000.0

# Nastavení počátečního kanálu
common.radio.set_channel(common.channels[0])

# Skenování kanálů a dekódování ESB paketů v pseudo-promiskuitním módu
last_tune = time.time()
channel_index = 0

logging.info("Starting nRF24 channel scanner...")

try:
    while True:
        # Přepnutí na další kanál po uplynutí dwell_time
        if len(common.channels) > 1 and time.time() - last_tune > dwell_time:
            channel_index = (channel_index + 1) % len(common.channels)
            common.radio.set_channel(common.channels[channel_index])
            last_tune = time.time()

        # Příjem payloadu
        value = common.radio.receive_payload()

        # V Pythonu 3 vrací receive_payload objekt typu bytes nebo list integerů
        if value and len(value) >= 5:
            # Rozdělení na adresu (prvních 5 bajtů) a payload (zbytek)
            address, payload = value[0:5], value[5:]

            # Výpis zachyceného paketu
            logging.info('{0: >2}  {1: >2}  {2}  {3}'.format(
                common.channels[channel_index],
                len(payload),
                ':'.join('{:02X}'.format(b) for b in address),
                ':'.join('{:02X}'.format(b) for b in payload)
            ))

except KeyboardInterrupt:
    print("\nScanner stopped by user.")
