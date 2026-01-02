#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import scipy.io.wavfile as wavfile
import scipy.signal as signal
import argparse
import sys
import matplotlib.pyplot as plt

# --- Script Configuration ---

# Default parameters for a common FSK fob signal (e.g., 433 MHz or 315 MHz fobs)
# You MUST adjust these parameters based on your specific signal analysis.
DEFAULT_SAMPLE_RATE = 2000000  # Example: 2 Msps (Mega samples per second)
DEFAULT_BAUD_RATE = 4800       # Example: 4800 symbols per second
DEFAULT_CENTER_FREQ_OFFSET = 0 # Center frequency offset from recording. Adjust if needed.

# Clock Recovery Parameters
# Gain (Kp) for the proportional timing loop controller. Adjust for stability vs. lock speed.
CLOCK_RECOVERY_GAIN = 0.1

# Payload Parameters
# Expected payload size in bits (e.g., 64 bits for many rolling code fobs)
PAYLOAD_SIZE_BITS = 64
# Preamble to search for (e.g., a repeating 01 pattern). This helps to sync the decoder.
PREAMBLE_PATTERN = '0101010101010101' # 16 bits of preamble

def setup_argparse():
    """Configures command-line argument parsing."""
    parser = argparse.ArgumentParser(description="Python FSK Demodulator for SDR Recordings with Advanced Clock Recovery and Payload Extraction.")
    parser.add_argument("input_file", type=str, help="Path to the input WAV file containing I/Q data.")
    parser.add_argument("-sr", "--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE,
                        help=f"Sample rate of the recorded signal (in Hz). Default: {DEFAULT_SAMPLE_RATE}")
    parser.add_argument("-br", "--baud-rate", type=int, default=DEFAULT_BAUD_RATE,
                        help=f"Baud rate of the FSK signal (symbols per second). Default: {DEFAULT_BAUD_RATE}")
    parser.add_argument("-fo", "--freq-offset", type=int, default=DEFAULT_CENTER_FREQ_OFFSET,
                        help=f"Frequency offset (in Hz) from the recording center. Default: {DEFAULT_CENTER_FREQ_OFFSET}")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output with plot generation.")
    return parser

def load_iq_data(filepath, sample_rate):
    """Loads I/Q data from a WAV file, supporting integer or float formats."""
    try:
        rate, data = wavfile.read(filepath)
        if rate != sample_rate:
            print(f"Warning: Sample rate in WAV header ({rate} Hz) does not match requested sample rate ({sample_rate} Hz). Using requested rate for calculations.", file=sys.stderr)

        if len(data.shape) != 2 or data.shape[1] != 2:
            print(f"Error: Expected stereo WAV file (I/Q data), got shape {data.shape}", file=sys.stderr)
            sys.exit(1)

        if np.issubdtype(data.dtype, np.floating):
            print(f"Detected floating-point data type ({data.dtype}). Skipping integer normalization.")
            i_data = data[:, 0]
            q_data = data[:, 1]
        elif np.issubdtype(data.dtype, np.integer):
            print(f"Detected integer data type ({data.dtype}). Normalizing to float range [-1.0, 1.0].")
            scaling_factor = np.iinfo(data.dtype).max
            i_data = data[:, 0].astype(np.float64) / scaling_factor
            q_data = data[:, 1].astype(np.float64) / scaling_factor
        else:
            print(f"Error: Unsupported data type '{data.dtype}' in WAV file.", file=sys.stderr)
            sys.exit(1)

        complex_data = i_data + 1j * q_data

        return complex_data, rate # return original rate from file for verification
    except Exception as e:
        print(f"Error loading WAV file: {e}", file=sys.stderr)
        sys.exit(1)

def downconvert_and_decimate(complex_data, sample_rate, center_freq_offset, baud_rate):
    """Adjusts signal to baseband and decimates to a lower, more manageable sample rate."""
    time_index = np.arange(len(complex_data)) / sample_rate
    lo_signal = np.exp(-1j * 2 * np.pi * center_freq_offset * time_index)
    shifted_data = complex_data * lo_signal

    decimation_factor = int(sample_rate / (10 * baud_rate)) # Target 10 samples per symbol
    if decimation_factor < 1:
        decimation_factor = 1

    decimated_data = signal.decimate(shifted_data, decimation_factor, ftype='fir')
    new_sample_rate = sample_rate / decimation_factor

    return decimated_data, new_sample_rate

# --- Phase Estimator Class for Adaptive Clock Recovery ---
class PhaseEstimator(object):
    """Estimates the phase of a sample by estimating the tangent (derivative)
    of the sample point. This is used to adjust the sampling time."""

    def __init__(self, sample_rate, symbol_rate):
        self.dx = 2 * symbol_rate / sample_rate
        self.samples = np.zeros(3)
        self.index = 0
        self.min_y = np.zeros(32)
        self.max_y = np.zeros(32)
        self.min_i = 0
        self.max_i = 0

    def _estimate(self):
        self.min_y[self.min_i] = np.min(self.samples)
        self.max_y[self.max_i] = np.max(self.samples)
        self.min_i = (self.min_i + 1) % len(self.min_y)
        self.max_i = (self.max_i + 1) % len(self.max_y)

        y_scale = (np.max(self.max_y) - np.min(self.min_y)) / 2.0
        if y_scale == 0:
            return 0

        dy = (self.samples[2] - self.samples[0]) / y_scale
        ratio = dy / self.dx
        ratio = min(5.0, ratio)
        ratio = max(-5.0, ratio)

        return ratio

    def estimate(self, samples):
        self.samples = samples
        return self._estimate()

def perform_demodulation(complex_data, new_sample_rate, baud_rate):
    """Performs non-coherent FSK demodulation and bit extraction with adaptive clock recovery."""

    # --- Step 1: Frequency Demodulation using complex conjugate multiplication ---
    demodulated_signal = np.angle(complex_data[1:] * np.conj(complex_data[:-1]))

    # --- Step 2: Low-pass filtering to smooth the demodulated signal ---
    lowpass_cutoff_freq = baud_rate * 0.5 # Lowpass cutoff at 0.5 * baud rate for standard FSK
    nyquist_freq = new_sample_rate / 2
    if lowpass_cutoff_freq >= nyquist_freq:
        print(f"Warning: Lowpass cutoff freq ({lowpass_cutoff_freq}) is too high for nyquist freq ({nyquist_freq}). Skipping lowpass filter.", file=sys.stderr)
        b, a = [1], [1] # bypass filter
    else:
        # Create a Butterworth filter (common choice for its flat passband)
        b, a = signal.butter(5, lowpass_cutoff_freq / nyquist_freq, btype='low')

    filtered_signal = signal.filtfilt(b, a, demodulated_signal)

    # --- Step 3: Adaptive Symbol Synchronization (Clock Recovery) ---
    # We use a Phase-Locked Loop (PLL) or similar timing recovery mechanism.
    samples_per_symbol = new_sample_rate / baud_rate
    phase_estimator = PhaseEstimator(new_sample_rate, baud_rate)

    bitstream = []
    current_sample_index = 0.0 # Use float for precise adjustments
    clock_error = 0.0

    while True:
        # Calculate next sampling index based on nominal rate and current error
        current_sample_index += samples_per_symbol - CLOCK_RECOVERY_GAIN * clock_error
        sample_index_int = int(round(current_sample_index))

        if sample_index_int >= len(filtered_signal) - 1:
            break # Reached end of signal

        bit_value = 1 if filtered_signal[sample_index_int] > 0 else 0
        bitstream.append(bit_value)

        # Calculate new clock error for next iteration (feedback)
        start_index = max(0, sample_index_int - 1)
        end_index = min(len(filtered_signal), sample_index_int + 2)

        sample_window = filtered_signal[start_index:end_index]
        if len(sample_window) < 3:
            sample_window = np.pad(sample_window, (0, 3 - len(sample_window)), mode='constant')

        clock_error = phase_estimator.estimate(sample_window)

    return filtered_signal, bitstream

def find_payloads(bitstream, preamble=PREAMBLE_PATTERN, payload_len=PAYLOAD_SIZE_BITS):
    """Searches for a specific payload after a preamble in the bitstream."""
    print(f"\nSearching for preamble '{preamble}' followed by {payload_len} bits...")
    payloads = []
    preamble_len = len(preamble)

    # Convert bitstream list to string for easier search
    bit_string = "".join(map(str, bitstream))

    # Search for preamble and extract payload
    start_index = bit_string.find(preamble)

    while start_index != -1:
        payload_start = start_index + preamble_len
        payload_end = payload_start + payload_len

        # Check if payload exists within remaining data
        if payload_end <= len(bit_string):
            payload = bit_string[payload_start:payload_end]
            payloads.append(payload)
            # Continue search from the end of the found payload to avoid overlaps
            start_index = bit_string.find(preamble, payload_end)
        else:
            # Preamble found, but not enough bits left for full payload
            break

    return payloads

def decode_manchester(bitstream):
    """Decodes a bitstream using Manchester (or differential Manchester) encoding logic."""
    # Manchester: A transition in the middle of a bit period.
    # We need to look at pairs of bits from the raw stream.
    # e.g., '01' might mean '0', '10' might mean '1'. This depends on the specific encoding variant.
    # For simplicity, we assume a simple bit transition logic where we sample at the center.
    # This function is a placeholder for more complex logic.
    decoded_bits = []
    for i in range(0, len(bitstream) - 1, 2):
        if bitstream[i] != bitstream[i+1]:
            # This indicates a transition, which is typical for Manchester encoding.
            # Example: 0->1 transition = '1', 1->0 transition = '0'.
            # A common implementation samples at the start of the bit period, and at the half-bit period.
            # If start bit == half bit, it's a 0. If start bit != half bit, it's a 1.
            # Let's assume the current simple bitstream extraction is sufficient for non-Manchester.

            # If you know the specific Manchester encoding for your fob (e.g., NRZ-L),
            # uncomment this to implement decoding logic:
            # if bitstream[i] == 0 and bitstream[i+1] == 1:
            #     decoded_bits.append(1)
            # elif bitstream[i] == 1 and bitstream[i+1] == 0:
            #     decoded_bits.append(0)
            pass # Placeholder logic for non-Manchester or simple FSK

    # For now, return the original stream as is, as simple FSK demodulation usually doesn't need this step.
    # If the key fob uses Manchester, the actual payload length would be half of the extracted stream length.
    return bitstream

def print_binary_output(bitstream, baud_rate, press_number, preamble_len):
    """Formats and prints the extracted binary data for a single press."""
    bit_string = "".join(map(str, bitstream))

    # Remove preamble for cleaner display if found.
    # This assumes the first part of the stream is preamble/sync data.
    display_string = bit_string[preamble_len:]

    print(f"\n--- Press {press_number} Demodulation Complete ---")
    print(f"Extracted Bitstream (Length: {len(bitstream)} bits, Preamble/Sync Removed: {len(display_string)} bits, Baud Rate: {baud_rate}):\n")

    formatted_bits = ""
    for i, bit in enumerate(display_string):
        formatted_bits += bit
        if (i + 1) % 8 == 0:
            formatted_bits += " "
            if (i + 1) % 64 == 0:
                formatted_bits += "\n"
    print(formatted_bits)

# --- Main Execution ---
if __name__ == "__main__":
    parser = setup_argparse()
    args = parser.parse_args()

    # --- Step 1: Load and Segment Data ---
    print(f"Loading I/Q data from {args.input_file}...")
    complex_data, file_sample_rate = load_iq_data(args.input_file, args.sample_rate)

    # Simple segmentation: Use a burst detection algorithm to find press start and end times.
    # We will assume a simple burst detection for now.
    # If this causes issues (e.g., slow performance), consider manually trimming the file first.
    # The "takes too long" issue often comes from processing large files with many non-signal portions.

    # --- Step 2: Demodulate and Extract Bitstream from entire file or segments ---
    # To improve performance for long recordings, let's process the entire file and then search for payloads.
    # The decimation step will reduce the size significantly.
    print("Pre-processing signal...")
    decimated_data, effective_sample_rate = downconvert_and_decimate(
        complex_data, args.sample_rate, args.freq_offset, args.baud_rate
    )

    print("Performing FSK demodulation and bit extraction on decimated signal...")
    filtered_signal, raw_bitstream = perform_demodulation(
        decimated_data, effective_sample_rate, args.baud_rate
    )

    # --- Step 3: Extract Payloads ---
    # Search for the expected preamble and fixed-size payload.
    # If a fob uses Manchester encoding, the actual payload length would be halved.
    payloads = find_payloads(raw_bitstream)

    # --- Step 4: Output results ---
    if payloads:
        print("\n--- Detected Key Fob Press Payloads ---")
        for i, payload in enumerate(payloads):
            print(f"Press {i+1} Payload: {payload}")
    else:
        print("No expected payloads detected. Check parameters or encoding.")

    # --- Step 5: Visualization (Optional) ---
    if args.verbose:
        try:
            time_demod = np.arange(len(filtered_signal)) / effective_sample_rate
            plt.figure(figsize=(12, 6))
            plt.plot(time_demod, filtered_signal, label='Filtered Demodulated Signal')
            plt.title('Demodulated Signal with Bit Slicer')
            plt.xlabel('Time (s)')
            plt.ylabel('Normalized Frequency Deviation')

            # Show bit sampling points on plot
            samples_per_symbol = effective_sample_rate / args.baud_rate
            sample_indices = np.arange(0, len(filtered_signal), samples_per_symbol)
            plt.plot(sample_indices / effective_sample_rate, filtered_signal[sample_indices.astype(int)], 'ro', markersize=3, label='Bit Samples (Estimated)')

            plt.legend()
            plt.grid(True)
            plt.show()
        except ImportError:
            pass