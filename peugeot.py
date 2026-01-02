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
DEFAULT_SAMPLE_RATE = 2000000  # Example: 2 Msps (Mega samples per second)
DEFAULT_BAUD_RATE = 4800       # Example: 4800 symbols per second
DEFAULT_CENTER_FREQ_OFFSET = 0 # Center frequency offset from recording. Adjust if needed.
LOWPASS_CUTOFF_RATIO = 0.5     # Low-pass filter cutoff relative to baud rate.

# Segmentation Parameters (Burst Detection)
BURST_THRESHOLD_FACTOR = 5.0
MIN_BURST_SAMPLES = 20000      # Note: Adjust dynamically or based on expected payload length.

# Flipper Output Parameters
DEFAULT_FLIPPER_FREQUENCY_HZ = 433920000 # 433.92 MHz in Hz. Set this to your fob's frequency.
DEFAULT_FLIPPER_PRESET = "FuriHalSubGhzPresetASK_650" # Default Flipper preset.

# Protocol Configuration (Keeloq Heuristic)
KEELOQ_FIXED_ID_LEN = 28 # Common fixed ID length for Keeloq (e.g., HCS301)
KEELOQ_ROLLING_CODE_LEN = 16 # Common rolling counter length
KEELOQ_STATUS_BITS_LEN = 4 # Common status/button bits length (last 4 bits before checksum)


# --- Command-Line Argument Setup ---
def setup_argparse():
    """Configures command-line argument parsing for the combined script."""
    parser = argparse.ArgumentParser(description="End-to-end FSK demodulator and dynamic protocol decoder.")
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
def perform_demodulation(complex_data, new_sample_rate, baud_rate):
    """Performs non-coherent FSK demodulation to get the baseband signal."""
    # Step 1: Frequency Demodulation using complex conjugate multiplication
    demodulated_signal = np.angle(complex_data[1:] * np.conj(complex_data[:-1]))

    # Step 2: Low-pass filtering to smooth the demodulated signal
    lowpass_cutoff_freq = baud_rate * LOWPASS_CUTOFF_RATIO
    nyquist_freq = new_sample_rate / 2
    if lowpass_cutoff_freq >= nyquist_freq:
        b, a = [1], [1] # bypass filter
    else:
        b, a = signal.butter(5, lowpass_cutoff_freq / nyquist_freq, btype='low')

    filtered_signal = signal.filtfilt(b, a, demodulated_signal)
    return filtered_signal

def extract_pulse_timings(filtered_signal, new_sample_rate, threshold=0.0):
    """Extracts high/low pulse durations from the filtered signal by detecting zero-crossings."""
    timings = []
    last_crossing_index = 0
    last_level = 1 if filtered_signal[0] > threshold else -1

    crossings = np.where(np.diff(np.sign(filtered_signal - threshold)))[0] + 1

    for crossing_index in crossings:
        pulse_duration_samples = crossing_index - last_crossing_index
        pulse_duration_us = int(round(pulse_duration_samples / new_sample_rate * 1e6))
        timings.append(pulse_duration_us * last_level)

        last_crossing_index = crossing_index
        last_level *= -1

    final_pulse_duration_samples = len(filtered_signal) - last_crossing_index
    final_pulse_duration_us = int(round(final_pulse_duration_samples / new_sample_rate * 1e6))
    if final_pulse_duration_us > 0:
        timings.append(final_pulse_duration_us * last_level)

    return timings


# --- Protocol Decoding Functions (Dynamic PWM/Manchester) ---

def compare_pulse_duration(duration_us, reference, tolerance):
    """Checks if a pulse duration matches a reference with a given tolerance."""
    return abs(duration_us - reference) < tolerance

def analyze_pulse_timings(timings, te_delta=150):
    """Analyzes pulse durations to determine short/long pulse lengths for decoding."""
    abs_timings = [abs(t) for t in timings if abs(t) > 10]

    if not abs_timings:
        return None, None

    hist, bin_edges = np.histogram(abs_timings, bins=30)
    dominant_bins = np.argsort(hist)[::-1]

    dominant_lengths = []
    for i in range(len(dominant_bins)):
        candidate_length = (bin_edges[dominant_bins[i]] + bin_edges[dominant_bins[i]+1]) / 2
        is_distinct = True
        for existing_length in dominant_lengths:
            if abs(candidate_length - existing_length) < te_delta:
                is_distinct = False
                break
        if is_distinct:
            dominant_lengths.append(candidate_length)
        if len(dominant_lengths) >= 2:
            break

    if len(dominant_lengths) == 2:
        return min(dominant_lengths), max(dominant_lengths)
    else:
        print("Warning: Failed to identify two distinct short/long pulse lengths automatically. This may indicate NRZ or a complex encoding scheme.")
        return None, None

def decode_generic_pwm(timings, te_short, te_long, te_delta=150):
    """Decodes a list of pulse timings using a generic PWM scheme (short-long=0, long-short=1)."""
    decoded_bits = []
    i = 0
    while i < len(timings) - 1:
        high_pulse = timings[i]
        low_pulse = timings[i+1]

        if high_pulse > 0 and low_pulse < 0:
            high_duration = high_pulse
            low_duration = abs(low_pulse)

            if compare_pulse_duration(high_duration, te_short, te_delta) and \
               compare_pulse_duration(low_duration, te_long, te_delta):
                decoded_bits.append('0')
            elif compare_pulse_duration(high_duration, te_long, te_delta) and \
                 compare_pulse_duration(low_duration, te_short, te_delta):
                decoded_bits.append('1')
            else:
                pass

        i += 2

    return "".join(decoded_bits)

def decode_manchester(bitstream_string):
    """Performs Manchester decoding on the raw bitstream.
    A common convention: '01' = 1, '10' = 0.
    """
    decoded_bits = []
    i = 0
    while i < len(bitstream_string) - 1:
        pair = bitstream_string[i:i+2]
        if pair == '01':
            decoded_bits.append('1')
        elif pair == '10':
            decoded_bits.append('0')
        else:
            # Skip non-standard transitions (e.g., '00' or '11')
            pass
        i += 2

    return "".join(decoded_bits)

# --- Keeloq Data Parsing ---

def parse_keeloq_data(bitstream_string):
    """Parses a 64-bit Keeloq data structure based on the provided C code logic."""
    if len(bitstream_string) < 64:
        return None

    fixed_id = bitstream_string[0:28]
    rolling_counter = bitstream_string[28:44]
    status_bits = bitstream_string[44:48]

    return {
        "fixed_id": fixed_id,
        "rolling_counter": rolling_counter,
        "status_bits": status_bits,
        "full_payload": bitstream_string
    }

def analyze_protocol_and_compare(all_decoded_payloads, pulse_short, pulse_long, decoding_method):
    """Compares multiple decoded payloads to identify fixed and rolling sections,
    and structures the data based on the identified protocol heuristic."""

    print(f"\n--- Protocol Analysis ---")

    if len(all_decoded_payloads) < 2:
        print("Need at least 2 press payloads to compare rolling code sections.")
        return

    payload_length = len(all_decoded_payloads[0])
    if payload_length < 64:
        print(f"Warning: Decoded payload length ({payload_length} bits) is less than 64 bits. Analysis may be inaccurate.")

    if not all(len(p) == payload_length for p in all_decoded_payloads):
        print("Error: Decoded payloads have inconsistent lengths. Cannot perform analysis.")
        return

    int_payloads = [int(p, 2) for p in all_decoded_payloads]
    xor_result = int_payloads[0]
    for p in int_payloads[1:]:
        xor_result ^= p

    fixed_mask = ~xor_result & int(all_decoded_payloads[0], 2)
    changing_mask = xor_result

    fixed_bits_str = format(fixed_mask, '0' + str(payload_length) + 'b')
    changing_bits_str = format(changing_mask, '0' + str(payload_length) + 'b')

    print(f"Pulse Timings Used for Decoding: Short={pulse_short} µs, Long={pulse_long} µs")
    print(f"Decoding Method: {decoding_method}")
    print(f"Payload Length: {payload_length} bits")
    print(f"Fixed (Static) Bits Mask: {fixed_bits_str}")
    print(f"Changing (Rolling) Bits Mask: {changing_bits_str}")
    print("Interpretation: '1's in the changing mask represent the rolling code portion of the signal.")

    if payload_length >= 64:
        first_press_data = parse_keeloq_data(all_decoded_payloads[0])
        print("\n--- Detailed Keeloq Structure (Press 1) ---")
        if first_press_data:
            print(f"Fixed ID: {first_press_data['fixed_id']} (Decimal: {int(first_press_data['fixed_id'], 2)})")
            print(f"Rolling Counter: {first_press_data['rolling_counter']} (Decimal: {int(first_press_data['rolling_counter'], 2)})")
            print(f"Button/Status: {first_press_data['status_bits']}")

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

    MIN_BURST_SAMPLES = int(args.sample_rate / args.baud_rate * 64 * 0.8)
    print(f"Analyzing signal energy to find bursts (sample rate: {file_sample_rate} Hz, minimum burst samples: {MIN_BURST_SAMPLES})...")
    segments = segment_signal_bursts(complex_data, min_burst_samples=MIN_BURST_SAMPLES)

    if not segments:
        print("No button presses detected for analysis. Exiting.", file=sys.stderr)
        sys.exit(1)

    all_decoded_payloads = []
    flipper_timings_data = []

    # --- Step 2: Demodulate each detected segment ---
    for press_index, (start_idx, end_idx) in enumerate(segments):
        print(f"\n--- Processing Press {press_index + 1} (Samples {start_idx}-{end_idx}) ---")

        buffer_samples = int(args.sample_rate / 100)
        start_buffered = max(0, start_idx - buffer_samples)
        end_buffered = min(len(complex_data), end_idx + buffer_samples)
        segment_data = complex_data[start_buffered:end_buffered]

        decimated_segment, effective_sample_rate = downconvert_and_decimate(
            segment_data, args.sample_rate, args.freq_offset, args.baud_rate
        )

        filtered_signal = perform_demodulation(decimated_segment, effective_sample_rate, args.baud_rate)
        raw_pulse_timings = extract_pulse_timings(filtered_signal, effective_sample_rate)

        # Store timings from the first press to generate Flipper file later.
        if press_index == 0:
            flipper_timings_data = raw_pulse_timings

        # --- Step 3: Decode raw pulse timings using specific protocol logic ---
        # 3a. Analyze raw pulse durations to identify short/long pulses for THIS specific signal.
        short_pulse, long_pulse = analyze_pulse_timings(raw_pulse_timings)

        # 3b. Decode using generic PWM if short/long pulses found.
        if short_pulse is not None and long_pulse is not None:
            # Try PWM decoding first (based on Peugeot example)
            decoded_pwm = decode_generic_pwm(raw_pulse_timings, short_pulse, long_pulse)
            # Try Manchester decoding (assuming simple 01=1, 10=0 logic on decoded PWM)
            decoded_manchester = decode_manchester(decoded_pwm)

            # --- Heuristic: Select best decoding based on consistency/length ---
            # If PWM decoding gives a consistent length and Manchester gives half, choose Manchester.
            # If Manchester decoding fails, use PWM result (if consistent).

            # For now, let's just use PWM and check for consistency.
            decoded_bitstream_string = decoded_pwm

            print(f"Decoded bitstream (Length: {len(decoded_bitstream_string)} bits):\n{decoded_bitstream_string}")
        else:
            print("Warning: Failed to identify short/long pulses automatically for this press. Decoding skipped.")
            decoded_bitstream_string = "" # Empty string if decoding failed

        all_decoded_payloads.append(decoded_bitstream_string)

    # --- Step 4: Protocol Analysis (using decoded payloads) ---
    short_pulse, long_pulse = analyze_pulse_timings(flipper_timings_data)
    analyze_protocol_and_compare(all_decoded_payloads, short_pulse, long_pulse, "dynamic PWM")

    # --- Step 5: Generate Flipper File for the first press ---
    print("\n--- Generating Flipper File ---")
    write_flipper_sub_file(args.output_file, args.flipper_frequency, args.preset, flipper_timings_data)
