#!/usr/bin/env python3
'''
  Copyright (C) 2016 Bastille Networks
  Ported to Python 3
'''

import time, logging, sys
from lib import common

# Parse command line arguments and initialize the radio
common.init_args('./nrf24-sniffer.py')
common.parser.add_argument('-a', '--address', type=str, help='Address to sniff, following as it changes channels', required=True)
common.parser.add_argument('-t', '--timeout', type=float, help='Channel timeout, in milliseconds', default=100)
common.parser.add_argument('-k', '--ack_timeout', type=int, help='ACK timeout in microseconds, accepts [250,4000], step 250', default=250)
common.parser.add_argument('-r', '--retries', type=int, help='Auto retry limit, accepts [0,15]', default=1, choices=range(0, 16), metavar='RETRIES')
common.parser.add_argument('-p', '--ping_payload', type=str, help='Ping payload, ex 0F:0F:0F:0F', default='0F:0F:0F:0F', metavar='PING_PAYLOAD')
common.parse_and_init()

# Parse the address (Python 3 bytes management)
try:
    hex_addr = common.args.address.replace(':', '')
    address = bytes.fromhex(hex_addr)[::-1][:5]
except ValueError:
    print(f"Error: Invalid hex address format: {common.args.address}")
    sys.exit(1)

address_string = ':'.join('{:02X}'.format(b) for b in address[::-1])
if len(address) < 2:
    raise Exception('Invalid address: {0}'.format(common.args.address))

# Put the radio in sniffer mode (ESB w/o auto ACKs)
common.radio.enter_sniffer_mode(address)

# Convert channel timeout from milliseconds to seconds
timeout = float(common.args.timeout) / float(1000)

# Parse the ping payload
try:
    ping_payload = bytes.fromhex(common.args.ping_payload.replace(':', ''))
except ValueError:
    print(f"Error: Invalid hex payload format: {common.args.ping_payload}")
    sys.exit(1)

# Format the ACK timeout and auto retry values
ack_timeout = int(common.args.ack_timeout / 250) - 1
ack_timeout = max(0, min(ack_timeout, 15))
retries = max(0, min(common.args.retries, 15))

# Sweep through the channels and decode ESB packets in pseudo-promiscuous mode
last_ping = time.time()
channel_index = 0

logging.info(f"Sniffing on address {address_string}...")

try:
    while True:
        # Follow the target device if it changes channels
        if time.time() - last_ping > timeout:

            # First try pinging on the active channel
            if not common.radio.transmit_payload(ping_payload, ack_timeout, retries):

                # Ping failed on the active channel, so sweep through all available channels
                success = False
                for idx in range(len(common.channels)):
                    common.radio.set_channel(common.channels[idx])
                    if common.radio.transmit_payload(ping_payload, ack_timeout, retries):
                        # Ping successful, exit out of the ping sweep
                        channel_index = idx
                        last_ping = time.time()
                        logging.debug('Ping success on channel {0}'.format(common.channels[channel_index]))
                        success = True
                        break

                # Ping sweep failed
                if not success: 
                    logging.debug('Unable to ping {0}'.format(address_string))

            # Ping succeeded on the active channel
            else:
                logging.debug('Ping success on channel {0}'.format(common.channels[channel_index]))
                last_ping = time.time()

        # Receive payloads
        value = common.radio.receive_payload()
        if value and value[0] == 0:
            # Reset the channel timer
            last_ping = time.time()

            # Split the payload from the status byte
            payload = value[1:]

            # Log the packet
            logging.info('{0: >2}  {1: >2}  {2}  {3}'.format(
                        common.channels[channel_index],
                        len(payload),
                        address_string,
                        ':'.join('{:02X}'.format(b) for b in payload)))
except KeyboardInterrupt:
    print("\nSniffing stopped by user.")
