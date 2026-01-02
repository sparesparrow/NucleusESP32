#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# --- Python Library Imports ---
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
LOWPASS_CUTOFF_RATIO = 0.5     # Low-pass filter cutoff relative to baud rate.

# Clock Recovery Parameters
CLOCK_RECOVERY_GAIN = 0.1

# Segmentation Parameters (Burst Detection)
# Noise floor threshold factor: Signal must be this many times louder than noise to be considered a burst.
BURST_THRESHOLD_FACTOR = 5.0
# Minimum length (in samples) for a detected burst to be considered valid (prevents short noise spikes).
MIN_BURST_SAMPLES = 1000      # Note: Adjust dynamically or based on expected payload length.

# Flipper Output Parameters
DEFAULT_FLIPPER_FREQUENCY_HZ = 433920000 # 433.92 MHz in Hz. Set this to your fob's frequency.
DEFAULT_FLIPPER_PRESET = "FuriHalSubGhzPresetASK_650" # Default Flipper preset.


# --- Command-Line Argument Setup ---
def setup_argparse():
    """Configures command-line argument parsing for the combined script."""
    parser = argparse.ArgumentParser(description="End-to-end FSK demodulator and protocol analyzer. Loads baseband data, segments bursts, analyzes rolling code patterns, and generates Flipper Zero file.")
    parser.add_argument("input_file", type=str, help="Path to the input WAV file containing I/Q data.")
    parser.add_argument("output_file", type=str, help="Name of the output .sub file (e.g., keyfob.sub).")
    parser.add_argument("-sr", "--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE,
                        help=f"Sample rate of the recorded signal (in Hz). Default: {DEFAULT_SAMPLE_RATE}")
    parser.add_argument("-br", "--baud-rate", type=int, default=DEFAULT_BAUD_RATE,
                        help=f"Baud rate of the signal (symbols per second). Default: {DEFAULT_BAUD_RATE}")
    parser.add_argument("-fo", "--freq-offset", type=int, default=DEFAULT_CENTER_FREQ_OFFSET,
                        help=f"Frequency offset (in Hz) from the recording center. Default: {DEFAULT_CENTER_FREQ_OFFSET}")
    parser.add_argument("-freq", "--flipper-frequency", type=int, default=DEFAULT_FLIPPER_FREQUENCY_HZ,
                        help=f"RF frequency of the key fob in Hz for Flipper file. Default: {DEFAULT_FLIPPER_FREQUENCY_HZ}")
    parser.add_argument("-p", "--preset", type=str, default=DEFAULT_FLIPPER_PRESET,
                        help=f"Flipper SubGhz preset. Default: {DEFAULT_FLIPPER_PRESET}")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output with plot generation.")
    return parser


# --- Data Loading and Preprocessing ---
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


# --- Signal Segmentation ---
def segment_signal_bursts(complex_data, threshold_factor=BURST_THRESHOLD_FACTOR, min_burst_samples=MIN_BURST_SAMPLES):
    """Segments the complex data into bursts based on signal energy detection."""
    print("Analyzing signal energy to find bursts...")
    envelope = np.abs(complex_data)
    hist, bin_edges = np.histogram(envelope, bins=100)
    noise_floor_amplitude_index = np.argmax(hist)
    noise_floor = bin_edges[noise_floor_amplitude_index]
    threshold = noise_floor * threshold_factor

    segments = []
    in_burst = False
    start_index = 0
    end_of_data = len(complex_data)

    for i in range(end_of_data):
        if not in_burst and envelope[i] > threshold:
            start_index = i
            in_burst = True
        elif in_burst and envelope[i] < threshold:
            end_index = i
            duration = end_index - start_index
            if duration > min_burst_samples:
                segments.append((start_index, end_index))
            in_burst = False

    if in_burst:
        duration = end_of_data - start_index
        if duration > min_burst_samples:
            segments.append((start_index, end_of_data))

    return segments


# --- FSK Demodulation and Timings Extraction ---
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
    demodulated_signal = np.angle(complex_data[1:] * np.conj(complex_data[:-1]))
    lowpass_cutoff_freq = baud_rate * LOWPASS_CUTOFF_RATIO
    nyquist_freq = new_sample_rate / 2
    if lowpass_cutoff_freq >= nyquist_freq:
        b, a = [1], [1] # bypass filter
    else:
        b, a = signal.butter(5, lowpass_cutoff_freq / nyquist_freq, btype='low')

    filtered_signal = signal.filtfilt(b, a, demodulated_signal)
    samples_per_symbol = new_sample_rate / baud_rate
    phase_estimator = PhaseEstimator(new_sample_rate, baud_rate)

    bitstream = []
    current_sample_index = 0.0 # Use float for precise adjustments
    clock_error = 0.0

    while True:
        current_sample_index += samples_per_symbol - CLOCK_RECOVERY_GAIN * clock_error
        sample_index_int = int(round(current_sample_index))

        if sample_index_int >= len(filtered_signal) - 1:
            break

        bit_value = 1 if filtered_signal[sample_index_int] > 0 else 0
        bitstream.append(bit_value)

        start_index = max(0, sample_index_int - 1)
        end_index = min(len(filtered_signal), sample_index_int + 2)
        sample_window = filtered_signal[start_index:end_index]
        if len(sample_window) < 3:
            sample_window = np.pad(sample_window, (0, 3 - len(sample_window)), mode='constant')
        clock_error = phase_estimator.estimate(sample_window)

    return filtered_signal, bitstream

def bitstream_to_timings(bitstream, baud_rate):
    """Converts a binary bitstream into a list of Flipper-compatible pulse timings (microseconds)."""
    pulse_length_us = int(round(1 / baud_rate * 1e6))
    timings = []
    for bit in bitstream:
        if bit == '1':
            timings.append(pulse_length_us)
        else:
            timings.append(-pulse_length_us)
    return timings


# --- Protocol Analysis ---
def analyze_payload_structure(payloads):
    """Compares multiple payloads to identify fixed and rolling sections."""
    print("\n--- Fixed vs. Rolling Code Analysis ---")
    if len(payloads) < 2:
        print("Need at least 2 payloads to compare fixed vs. rolling sections.")
        return

    payload_length = len(payloads[0])
    int_payloads = [int(p, 2) for p in payloads]

    # Calculate bitmask for changing bits:
    xor_result = int_payloads[0]
    for p in int_payloads[1:]:
        xor_result ^= p

    fixed_mask = ~xor_result & int(payloads[0], 2)
    changing_mask = xor_result

    fixed_bits_str = format(fixed_mask, '0' + str(payload_length) + 'b')
    changing_bits_str = format(changing_mask, '0' + str(payload_length) + 'b')

    print(f"Fixed (Static) Bits Mask: {fixed_bits_str}")
    print(f"Changing (Rolling) Bits Mask: {changing_bits_str}")
    print("Interpretation: '1's in the changing mask represent the rolling code portion of the signal.")


# --- Flipper Output Generation ---
def write_flipper_sub_file(filename, frequency, preset, timings):
    """Writes the timings data to a .sub file in Flipper Zero format."""
    try:
        with open(filename, 'w') as f:
            f.write("Filetype: Flipper SubGhz Key File\n")
            f.write("Protocol: RAW\n")
            f.write(f"Frequency: {frequency}\n")
            f.write(f"Preset: {preset}\n")
            f.write("Data: ")
            f.write(" ".join(map(str, timings)) + "\n")

        print(f"\nSuccessfully created Flipper file: '{filename}'")
        print(f"File size (approx. bitstream length): {len(timings)} timings.")

    except IOError as e:
        print(f"Error writing file '{filename}': {e}", file=sys.stderr)


# --- Main Execution Loop ---
if __name__ == "__main__":
    parser = setup_argparse()
    args = parser.parse_args()

    # --- Step 1: Load and Segment Data ---
    print(f"Loading I/Q data from {args.input_file}...")
    complex_data, file_sample_rate = load_iq_data(args.input_file, args.sample_rate)

    print(f"Analyzing signal energy to find bursts (sample rate: {file_sample_rate} Hz)...")
    segments = segment_signal_bursts(complex_data)

    if not segments:
        print("No button presses detected for analysis. Exiting.", file=sys.stderr)
        sys.exit(1)

    all_press_bitstreams = []

    # --- Step 2: Demodulate each detected segment ---
    for press_index, (start_idx, end_idx) in enumerate(segments):
        print(f"\n--- Processing Press {press_index + 1} (Samples {start_idx}-{end_idx}) ---")

        # Extract segment data with buffer for stability
        buffer_samples = int(args.sample_rate / 100) # 10ms buffer
        start_buffered = max(0, start_idx - buffer_samples)
        end_buffered = min(len(complex_data), end_idx + buffer_samples)
        segment_data = complex_data[start_buffered:end_buffered]

        # Downconvert and decimate a single segment
        decimated_segment, effective_sample_rate = downconvert_and_decimate(
            segment_data, args.sample_rate, args.freq_offset, args.baud_rate
        )

        # Demodulate and extract bitstream for this segment
        filtered_signal, bitstream_list = perform_demodulation(
            decimated_segment, effective_sample_rate, args.baud_rate
        )
        bitstream_string = "".join(map(str, bitstream_list))
        all_press_bitstreams.append(bitstream_string)

        # Print raw bitstream for manual inspection
        print(f"Extracted Bitstream (Length: {len(bitstream_string)} bits):\n{bitstream_string}")

    # --- Step 3: Protocol Analysis ---
    if len(all_press_bitstreams) > 1:
        analyze_payload_structure(all_press_bitstreams)

    # --- Step 4: Generate Flipper File for the first press ---
    print("\n--- Generating Flipper File ---")
    flipper_timings = bitstream_to_timings(all_press_bitstreams[0], args.baud_rate)
    write_flipper_sub_file(args.output_file, args.flipper_frequency, args.preset, flipper_timings)
