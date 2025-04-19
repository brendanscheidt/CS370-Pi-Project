#!/usr/bin/env python3
"""
sound_server.py: continuous tone with smooth pitch‐bend based on incoming frequency data
"""

import socket
import threading
import time

import numpy as np
import sounddevice as sd

# ─── Configuration ─────────────────────────────────────────────────────────────
LISTEN_PORT = 8080
SAMPLE_RATE = 44100  # Hz
CHANNELS = 2         # stereo
SMOOTHING = 0.2      # smoothing factor per block (0 < SMOOTHING ≤ 1)

# ─── Shared state ──────────────────────────────────────────────────────────────
current_freq = 440.0  # actual frequency used for this block
target_freq  = 440.0  # set by network thread
_phase        = 0.0   # phase accumulator (radians)

# ─── Audio callback ────────────────────────────────────────────────────────────
def audio_callback(outdata, frames, time_info, status):
    global _phase, current_freq, target_freq

    # 1) Smoothly move current_freq toward target_freq
    f0 = current_freq
    f1 = f0 + SMOOTHING * (target_freq - f0)
    current_freq = f1

    # 2) Generate a linear‐frequency‐ramp sine wave over this block
    T = frames / SAMPLE_RATE
    t = np.arange(frames) / SAMPLE_RATE
    k = f1 - f0
    # phase for each sample: φ(t) = 2π (f0·t + (k/(2T))·t²)
    phi = 2 * np.pi * (f0 * t + (k / (2 * T)) * t ** 2)
    wave = 0.2 * np.sin(phi + _phase)

    # 3) Write stereo data
    stereo = np.repeat(wave[:, np.newaxis], CHANNELS, axis=1)
    outdata[:] = stereo.astype(np.float32)

    # 4) Update phase accumulator: Δφ = 2π·T·(f0 + f1)/2
    phase_inc = 2 * np.pi * T * (f0 + f1) / 2
    _phase = (_phase + phase_inc) % (2 * np.pi)

# ─── Network thread ────────────────────────────────────────────────────────────
def network_thread():
    global target_freq
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', LISTEN_PORT))
    sock.listen(1)
    print(f"Listening for freq data on port {LISTEN_PORT}…")
    conn, addr = sock.accept()
    print("Client connected from", addr)
    with conn:
        buf = b''
        while True:
            data = conn.recv(1024)
            if not data:
                print("Sender disconnected")
                break
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                try:
                    f = float(line.strip())
                    target_freq = f
                except ValueError:
                    pass

# ─── Main ─────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    # Start continuous‐output audio stream
    stream = sd.OutputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        callback=audio_callback,
        blocksize=1024
    )
    stream.start()

    # Launch the TCP listener thread
    th = threading.Thread(target=network_thread, daemon=True)
    th.start()

    print("Continuous smooth pitch‐bend tone running. Press Ctrl+C to quit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting…")
    finally:
        stream.stop()
        stream.close()
        print("Clean exit.")
