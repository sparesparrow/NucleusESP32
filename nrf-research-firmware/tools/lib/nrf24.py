#!/usr/bin/env python3
'''
  Copyright (C) 2016 Bastille Networks
  Ported to Python 3 for sparrow-ai-tech
'''

import usb, logging, sys

try:
    from usb import core as _usb_core
except ImportError:
    print('PyUSB not found. Install with: sudo pip3 install pyusb')
    sys.exit(1)

# USB commands
TRANSMIT_PAYLOAD               = 0x04
ENTER_SNIFFER_MODE             = 0x05
ENTER_PROMISCUOUS_MODE         = 0x06
ENTER_TONE_TEST_MODE           = 0x07
TRANSMIT_ACK_PAYLOAD           = 0x08
SET_CHANNEL                    = 0x09
GET_CHANNEL                    = 0x0A
ENABLE_LNA_PA                  = 0x0B
TRANSMIT_PAYLOAD_GENERIC       = 0x0C
ENTER_PROMISCUOUS_MODE_GENERIC = 0x0D
RECEIVE_PAYLOAD                = 0x12

RF_RATE_250K = 0
RF_RATE_1M   = 1
RF_RATE_2M   = 2

class nrf24:
    usb_timeout = 2500

    def __init__(self, index=0):
        try:
            devices = list(usb.core.find(idVendor=0x1915, idProduct=0x0102, find_all=True))
            if not devices:
                raise Exception('Cannot find USB dongle.')
            self.dongle = devices[index]
            self.dongle.set_configuration()
        except Exception as e:
            raise Exception(f'Error initializing dongle: {e}')

    def send_usb_command(self, request, data):
        usb_data = [request] + list(data)
        self.dongle.write(0x01, usb_data, timeout=self.usb_timeout)

    def enter_sniffer_mode(self, address):
        data = [len(address)] + list(address)
        self.send_usb_command(ENTER_SNIFFER_MODE, data)
        self.dongle.read(0x81, 64, timeout=self.usb_timeout)
        addr_str = ':'.join('{:02X}'.format(b) for b in address[::-1])
        logging.debug(f'Entered sniffer mode with address {addr_str}')

    def receive_payload(self):
        self.send_usb_command(RECEIVE_PAYLOAD, [])
        try:
            return self.dongle.read(0x81, 64, timeout=self.usb_timeout)
        except usb.core.USBError:
            return None

    def transmit_payload(self, payload, timeout=4, retransmits=15):
        data = [len(payload), timeout, retransmits] + list(payload)
        self.send_usb_command(TRANSMIT_PAYLOAD, data)
        try:
            res = self.dongle.read(0x81, 64, timeout=self.usb_timeout)
            return bool(res and res[0] > 0)
        except:
            return False

    def transmit_payload_generic(self, payload, address=b"\x33\x33\x33\x33\x33"):
        data = [len(payload), len(address)] + list(payload) + list(address)
        self.send_usb_command(TRANSMIT_PAYLOAD_GENERIC, data)
        try:
            res = self.dongle.read(0x81, 64, timeout=self.usb_timeout)
            return bool(res and res[0] > 0)
        except:
            return False

    def set_channel(self, channel):
        chan = int(channel)
        if chan > 125: chan = 125
        self.send_usb_command(SET_CHANNEL, [chan])
        self.dongle.read(0x81, 64, timeout=self.usb_timeout)
        logging.debug(f'Tuned to {chan}')

    def enable_lna(self):
        self.send_usb_command(ENABLE_LNA_PA, [])
        self.dongle.read(0x81, 64, timeout=self.usb_timeout)
