#!/usr/bin/env python3
'''
  Copyright (C) 2016 Bastille Networks
  Ported to Python 3 for sparrow-ai-tech
'''

import logging, argparse, sys
# Importujeme třídu nrf24 z lokálního modulu nrf24.py
try:
    from nrf24 import nrf24
except ImportError:
    # Pokud spouštíš skript z nadřazeného adresáře, zkusíme relativní import
    try:
        from .nrf24 import nrf24
    except ImportError:
        print("Error: nrf24.py not found in the library path.")
        sys.exit(1)

# Globální proměnné pro přístup z ostatních skriptů
channels = []
args = None
parser = None
radio = None

def init_args(description):
    """Inicializuje argument parser se standardními volbami pro nRF24 nástroje."""
    global parser
    parser = argparse.ArgumentParser(
        description=description,
        formatter_class=lambda prog: argparse.HelpFormatter(prog, max_help_position=50, width=120)
    )

    # Definice parametrů
    parser.add_argument('-c', '--channels', type=int, nargs='+',
                        help='RF channels (2-83)', default=list(range(2, 84)), metavar='N')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Enable verbose debug output', default=False)
    parser.add_argument('-l', '--lna', action='store_true',
                        help='Enable the LNA (for CrazyRadio PA dongles)', default=False)
    parser.add_argument('-i', '--index', type=int,
                        help='USB dongle index (if multiple present)', default=0)

def parse_and_init():
    """Zpracuje argumenty a inicializuje rádio."""
    global parser, args, channels, radio

    # Parsování argumentů z příkazové řádky
    args = parser.parse_args()

    # Nastavení logování podle parametru verbose
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='[%(asctime)s.%(msecs)03d]  %(message)s',
        datefmt="%Y-%m-%d %H:%M:%S"
    )

    # Nastavení kanálů
    channels = args.channels
    logging.debug(f"Using channels: {', '.join(map(str, channels))}")

    # Inicializace rádio hardwaru (třída nrf24 z nrf24.py)
    try:
        radio = nrf24(args.index)

        # Aktivace LNA (zesilovače), pokud je vyžadováno
        if args.lna:
            radio.enable_lna()
            logging.debug("LNA/PA enabled")

    except Exception as e:
        logging.error(f"Failed to initialize radio: {e}")
        sys.exit(1)
