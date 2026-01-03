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
LOWPASS_CUTOFF_RATIO = 0.5     # Low-pass filter cutoff relative to baud rate.

# Clock Recovery Parameters (Used for general signal processing, but we extract timings directly)
CLOCK_RECOVERY_GAIN = 0.1

# Segmentation Parameters (Burst Detection)
BURST_THRESHOLD_FACTOR = 5.0  # Threshold = noise_floor * factor
MIN_BURST_SAMPLES = 1000      # Minimum length of a press to avoid noise detection

# Flipper Output Parameters
DEFAULT_FLIPPER_FREQUENCY_HZ = 433920000 # 433.92 MHz in Hz. Set this to your fob's frequency.
DEFAULT_FLIPPER_PRESET = "FuriHalSubGhzPresetASK_650" # Default Flipper preset.


def setup_argparse():
    """Configures command-line argument parsing."""
    parser = argparse.ArgumentParser(description="Python FSK Demodulator and Flipper Raw Timings Converter.")
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

def segment_signal_bursts(complex_data, threshold_factor=BURST_THRESHOLD_FACTOR, min_burst_samples=MIN_BURST_SAMPLES):
    """Segments the complex data into bursts based on signal energy detection."""
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

# --- New Function: Convert Demodulated Signal to Raw Pulse Timings ---
def convert_demodulated_to_timings(filtered_signal, new_sample_rate, min_pulse_samples=10):
    """Converts a filtered demodulated signal (representing frequency deviation)
    into a sequence of pulse durations using zero-crossing detection.
    """
    timings = []
    last_crossing_index = 0
    last_level = 1 if filtered_signal[0] > 0 else -1

    # Find zero crossings by checking sign changes
    crossings = np.where(np.diff(np.sign(filtered_signal)))[0] + 1

    for crossing_index in crossings:
        # Calculate pulse length since last crossing
        pulse_duration_samples = crossing_index - last_crossing_index

        # Convert to microseconds
        pulse_duration_us = int(round(pulse_duration_samples / new_sample_rate * 1e6))

        # Check for noise (pulses too short to be meaningful)
        if pulse_duration_samples < min_pulse_samples:
            continue

        # Append duration based on polarity (positive for high, negative for low)
        # The sign of the duration is based on the level *before* the crossing.
        timings.append(pulse_duration_us * last_level)

        # Update last state for next pulse calculation
        last_crossing_index = crossing_index
        last_level *= -1 # Invert level for next pulse segment

    # Add final pulse segment from last crossing to end of signal
    final_pulse_duration_samples = len(filtered_signal) - last_crossing_index
    final_pulse_duration_us = int(round(final_pulse_duration_samples / new_sample_rate * 1e6))
    if final_pulse_duration_samples >= min_pulse_samples:
        timings.append(final_pulse_duration_us * last_level)

    return timings

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

def plot_signals(envelope, segments, sample_rate):
    """Generates a plot of the signal envelope and detected segments."""
    try:
        time = np.arange(len(envelope)) / sample_rate
        plt.figure(figsize=(12, 6))
        plt.plot(time, envelope, label='Signal Envelope')
        plt.title('Signal Bursts Detection')
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude')

        for start, end in segments:
            start_time = start / sample_rate
            end_time = end / sample_rate
            plt.axvspan(start_time, end_time, color='red', alpha=0.3)

        plt.legend()
        plt.grid(True)
        plt.show()

    except ImportError:
        print("Warning: matplotlib not installed. Cannot generate plots. Install with 'pip install matplotlib'.", file=sys.stderr)

# --- Main Execution ---
if __name__ == "__main__":
    parser = setup_argparse()
    args = parser.parse_args()

    # --- Step 1: Load and Segment Data ---
    print(f"Loading I/Q data from {args.input_file}...")
    complex_data, file_sample_rate = load_iq_data(args.input_file, args.sample_rate)

    # Segment the data based on energy bursts
    segments = segment_signal_bursts(complex_data)

    if not segments:
        print("No button presses detected for analysis. Exiting.", file=sys.stderr)
        sys.exit(1)

    # --- Step 2: Demodulate and convert first detected segment ---
    # We will only process the first detected press to avoid confusing multiple presses.
    start_idx, end_idx = segments[0]
    print(f"\n--- Processing First Detected Press (Samples {start_idx}-{end_idx}) ---")

    # Extract segment data with buffer for stability
    buffer_samples = int(args.sample_rate / 100) # 10ms buffer
    start_buffered = max(0, start_idx - buffer_samples)
    end_buffered = min(len(complex_data), end_idx + buffer_samples)
    segment_data = complex_data[start_buffered:end_buffered]

    # Downconvert and decimate a single segment
    print("Pre-processing signal segment...")
    decimated_segment, effective_sample_rate = downconvert_and_decimate(
        segment_data, args.sample_rate, args.freq_offset, args.baud_rate
    )

    # --- Step 3: Frequency Demodulation and Filtering ---
    # We perform demodulation to get the baseband signal for zero-crossing detection.
    print("Performing FSK demodulation...")
    demodulated_signal = np.angle(decimated_segment[1:] * np.conj(decimated_segment[:-1]))

    # Low-pass filtering to smooth the demodulated signal
    lowpass_cutoff_freq = args.baud_rate * LOWPASS_CUTOFF_RATIO
    nyquist_freq = effective_sample_rate / 2
    b, a = signal.butter(5, lowpass_cutoff_freq / nyquist_freq, btype='low')
    filtered_signal = signal.filtfilt(b, a, demodulated_signal)

    # --- Step 4: Convert filtered signal to raw timings ---
    print("Converting filtered signal to Flipper-compatible pulse timings...")
    # Calculate minimum pulse width based on baud rate to filter out noise.
    # A pulse duration should be at least (1 / baud_rate) seconds.
    min_pulse_duration_samples = int(effective_sample_rate / args.baud_rate * 0.5) # Minimum half a bit period
    flipper_timings = convert_demodulated_to_timings(filtered_signal, effective_sample_rate, min_pulse_samples=min_pulse_duration_samples)

    # --- Step 5: Write Flipper file ---
    write_flipper_sub_file(args.output_file, args.flipper_frequency, args.preset, flipper_timings)

    # --- Step 6: Visualization (Optional) ---
    if args.verbose:
        # Plot the segments over the full signal envelope
        envelope = np.abs(complex_data)
        plot_signals(envelope, segments, args.sample_rate)