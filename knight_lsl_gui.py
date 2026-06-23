"""
NeuroPawn Knight Board -> LSL streamer (barebones GUI).

Reads the raw serial packet from a NeuroPawn Knight board, parses the 8 EXG
channels, optionally applies notch / bandpass filtering, and publishes the data
as an LSL stream.

Packet format (from brainflow knight.cpp), 21 bytes total:
    [0]      0xA0 start byte
    [1]      sample number
    [2-3]    EXG channel 1   (16-bit big-endian signed)
    [4-5]    EXG channel 2
    ...
    [16-17]  EXG channel 8
    [18]     LOFF STATP
    [19]     LOFF STATN
    [20]     0xC0 end byte

After the start byte is found, the firmware sends 20 more bytes (b[0..19]):
    b[0]     sample number
    b[1..16] 8 EXG channels (2 bytes each, big-endian signed)
    b[17]    LOFF STATP
    b[18]    LOFF STATN
    b[19]    0xC0 end byte

Config commands (sent as ASCII over the same serial port, 115200 baud):
    chon_{ch}_{gain}   enable channel ch (1-indexed) with gain
    choff_{ch}         disable channel ch
    rldadd_{ch}        enable right-leg-drive for channel ch
    rldremove_{ch}     disable right-leg-drive for channel ch
"""

import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
import serial
import serial.tools.list_ports
from pylsl import StreamInfo, StreamOutlet
from scipy.signal import butter, iirnotch, lfilter, lfilter_zi

# ---------------------------------------------------------------------------
# Board constants
# ---------------------------------------------------------------------------
START_BYTE = 0xA0
END_BYTE = 0xC0
BAUD_RATE = 115200
NUM_CHANNELS = 8
SAMPLING_RATE = 125  # Hz (Knight / KnightIMU)

# Frame payload sizes (bytes read AFTER the start byte, end byte included).
EEG_PAYLOAD_LEN = 20   # non-IMU board: counter + 8*2 EXG + 2 LOFF + end
IMU_PAYLOAD_LEN = 56   # IMU board:    counter + 8*2 EXG + 2 LOFF + 9*4 IMU + end

# IMU channel labels (9x float32, little-endian) appended after the 8 EXG ch.
IMU_LABELS = [
    "AccelX", "AccelY", "AccelZ",
    "GyroX", "GyroY", "GyroZ",
    "MagX", "MagY", "MagZ",
]

START_STREAM_SENTINEL = "__START_STREAM__"

GAIN_VALUES = [1, 2, 3, 4, 6, 8, 12]
DEFAULT_GAIN = 12
CMD_PAUSE = 2      # seconds to pause after every command


def eeg_scale(gain):
    """microvolts-per-count scaling factor for a given gain."""
    return 4.0 / (pow(2, 15) - 1) / gain * 1_000_000.0


# ---------------------------------------------------------------------------
# Streaming filter (per-channel state, processes one sample at a time)
# ---------------------------------------------------------------------------
class StreamFilter:
    """Optional notch + bandpass applied sample-by-sample, per channel."""

    def __init__(self, fs, n_channels):
        self.fs = fs
        self.n_channels = n_channels
        self.notch_b = None
        self.notch_a = None
        self.notch_zi = None
        self.bp_b = None
        self.bp_a = None
        self.bp_zi = None

    def set_notch(self, freq):
        """freq in Hz (50 or 60), or None to disable."""
        if freq is None:
            self.notch_b = self.notch_a = self.notch_zi = None
            return
        b, a = iirnotch(w0=freq, Q=30.0, fs=self.fs)
        self.notch_b, self.notch_a = b, a
        zi = lfilter_zi(b, a)
        self.notch_zi = np.tile(zi, (self.n_channels, 1))

    def set_bandpass(self, low, high):
        """low/high in Hz, or None to disable."""
        if low is None or high is None:
            self.bp_b = self.bp_a = self.bp_zi = None
            return
        b, a = butter(N=4, Wn=[low, high], btype="band", fs=self.fs)
        self.bp_b, self.bp_a = b, a
        zi = lfilter_zi(b, a)
        self.bp_zi = np.tile(zi, (self.n_channels, 1))

    def process(self, sample):
        """sample: 1D array of length n_channels. Returns filtered array."""
        out = np.asarray(sample, dtype=np.float64)
        if self.notch_b is not None:
            for ch in range(self.n_channels):
                y, self.notch_zi[ch] = lfilter(
                    self.notch_b, self.notch_a, [out[ch]], zi=self.notch_zi[ch]
                )
                out[ch] = y[0]
        if self.bp_b is not None:
            for ch in range(self.n_channels):
                y, self.bp_zi[ch] = lfilter(
                    self.bp_b, self.bp_a, [out[ch]], zi=self.bp_zi[ch]
                )
                out[ch] = y[0]
        return out


# ---------------------------------------------------------------------------
# Worker thread: owns the serial port (commands + acquisition + LSL push)
# ---------------------------------------------------------------------------
class KnightWorker(threading.Thread):
    def __init__(self, ser, gain, log_fn):
        super().__init__(daemon=True)
        self.ser = ser
        self.gain = gain
        self.scale = eeg_scale(gain)
        self.log = log_fn

        self.cmd_queue = queue.Queue()
        self.connected = True
        self.streaming = False
        self.outlet = None
        self.filter = StreamFilter(SAMPLING_RATE, NUM_CHANNELS)

        # set once the board type is auto-detected at stream start
        self.board_type = None          # 'eeg' or 'imu'
        self.payload_len = EEG_PAYLOAD_LEN
        self.n_lsl_channels = NUM_CHANNELS

    # -- public API (called from GUI thread) -------------------------------
    def send_command(self, cmd):
        self.cmd_queue.put(cmd)

    def request_stream_start(self):
        """Queue a sentinel so detection happens AFTER all config commands."""
        self.cmd_queue.put(START_STREAM_SENTINEL)

    def stop_streaming(self):
        self.streaming = False
        self.outlet = None
        self.board_type = None
        self.log("LSL stream stopped.")

    def stop(self):
        self.connected = False

    # -- board detection ---------------------------------------------------
    @staticmethod
    def _scan_frame_size(buf):
        """Return 'eeg' / 'imu' / None by locating two consecutive frames.

        Non-IMU frames are 21 bytes (0xA0 + 20), IMU frames are 57 bytes
        (0xA0 + 56). Requiring two consecutive start/end boundaries at the
        expected stride makes the detection robust against random matches.
        """
        n = len(buf)
        for i in range(n):
            if buf[i] != START_BYTE:
                continue
            # non-IMU: stride 21
            if (i + 42 < n and buf[i + 20] == END_BYTE
                    and buf[i + 21] == START_BYTE
                    and buf[i + 41] == END_BYTE
                    and buf[i + 42] == START_BYTE):
                return "eeg"
            # IMU: stride 57
            if (i + 114 < n and buf[i + 56] == END_BYTE
                    and buf[i + 57] == START_BYTE
                    and buf[i + 113] == END_BYTE
                    and buf[i + 114] == START_BYTE):
                return "imu"
        return None

    def _detect_board_type(self, timeout=4.0):
        self.ser.reset_input_buffer()
        buf = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline and self.connected:
            chunk = self.ser.read(256)
            if chunk:
                buf.extend(chunk)
            board = self._scan_frame_size(buf)
            if board is not None:
                return board
            if len(buf) > 8192:
                del buf[:-1024]
        return None

    def _begin_stream(self):
        self.log("Detecting board type from incoming packets...")
        board = self._detect_board_type()
        if board is None:
            self.log(
                "Detection failed: no valid packets. Enable at least one "
                "channel, then press Start again."
            )
            return

        self.board_type = board
        if board == "imu":
            self.payload_len = IMU_PAYLOAD_LEN
            labels = [f"EXG{i + 1}" for i in range(NUM_CHANNELS)] + IMU_LABELS
        else:
            self.payload_len = EEG_PAYLOAD_LEN
            labels = [f"EXG{i + 1}" for i in range(NUM_CHANNELS)]
        self.n_lsl_channels = len(labels)

        info = StreamInfo(
            name="NeuroPawnKnight",
            type="EEG",
            channel_count=self.n_lsl_channels,
            nominal_srate=SAMPLING_RATE,
            channel_format="float32",
            source_id="neuropawn_knight",
        )
        chns = info.desc().append_child("channels")
        for lbl in labels:
            ch = chns.append_child("channel")
            ch.append_child_value("label", lbl)
            if lbl.startswith("EXG"):
                ch.append_child_value("unit", "microvolts")
                ch.append_child_value("type", "EEG")
            else:
                ch.append_child_value("unit", "arbitrary")
                ch.append_child_value("type", "IMU")
        self.outlet = StreamOutlet(info)
        self.ser.reset_input_buffer()
        self.streaming = True
        kind = "IMU" if board == "imu" else "non-IMU"
        self.log(
            f"Detected {kind} board. LSL stream started "
            f"({self.n_lsl_channels} ch @ {SAMPLING_RATE} Hz)."
        )

    # -- internal ----------------------------------------------------------
    def _drain_one_command(self):
        try:
            cmd = self.cmd_queue.get_nowait()
        except queue.Empty:
            return False
        if cmd == START_STREAM_SENTINEL:
            self._begin_stream()
            return True
        try:
            self.ser.write(cmd.encode("ascii"))
            self.log(f"sent: {cmd}")
        except Exception as exc:  # noqa: BLE001
            self.log(f"command error: {exc}")
        time.sleep(CMD_PAUSE)
        return True

    def _read_packet(self):
        # find start byte
        b = self.ser.read(1)
        if len(b) != 1 or b[0] != START_BYTE:
            return None
        payload = bytearray()
        while len(payload) < self.payload_len:
            chunk = self.ser.read(self.payload_len - len(payload))
            if not chunk:
                return None
            payload.extend(chunk)
        if payload[self.payload_len - 1] != END_BYTE:
            return None

        sample = np.zeros(self.n_lsl_channels, dtype=np.float64)
        # 8 EXG channels: 16-bit big-endian signed, starting at payload[1]
        for i in range(NUM_CHANNELS):
            hi = payload[1 + 2 * i]
            lo = payload[2 + 2 * i]
            raw = (hi << 8) | lo
            if raw & 0x8000:
                raw -= 0x10000
            sample[i] = raw * self.scale
        # 9 IMU channels: 32-bit little-endian float, starting at payload[19]
        if self.board_type == "imu":
            imu_offset = 19
            for i in range(len(IMU_LABELS)):
                o = imu_offset + 4 * i
                sample[NUM_CHANNELS + i] = struct.unpack(
                    "<f", bytes(payload[o:o + 4])
                )[0]
        return sample

    def run(self):
        while self.connected:
            # commands take priority; the 1 s pause happens here
            if self._drain_one_command():
                continue
            if not self.streaming:
                # discard any stray bytes so the buffer does not grow
                if self.ser.in_waiting:
                    self.ser.reset_input_buffer()
                time.sleep(0.01)
                continue
            sample = self._read_packet()
            if sample is None:
                continue
            # filter the 8 EXG channels only; IMU channels pass through
            sample[:NUM_CHANNELS] = self.filter.process(sample[:NUM_CHANNELS])
            if self.outlet is not None:
                self.outlet.push_sample(sample.astype(np.float32).tolist())


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("NeuroPawn Knight -> LSL")
        self.resizable(False, False)

        self.ser = None
        self.worker = None

        self.notch_freq = None      # None / 50 / 60
        self.bandpass_on = False

        self._build_ui()

    # -- UI construction ---------------------------------------------------
    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}

        # Connection row
        conn = ttk.LabelFrame(self, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", **pad)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, **pad)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            conn, textvariable=self.port_var, width=14, state="readonly"
        )
        self.port_combo.grid(row=0, column=1, **pad)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, **pad
        )

        ttk.Label(conn, text="Gain:").grid(row=0, column=3, **pad)
        self.gain_var = tk.IntVar(value=DEFAULT_GAIN)
        self.gain_combo = ttk.Combobox(
            conn,
            textvariable=self.gain_var,
            width=5,
            state="readonly",
            values=GAIN_VALUES,
        )
        self.gain_combo.grid(row=0, column=4, **pad)

        self.connect_btn = ttk.Button(
            conn, text="Connect", command=self.toggle_connect
        )
        self.connect_btn.grid(row=0, column=5, **pad)

        # Channels frame
        chan = ttk.LabelFrame(self, text="Channels / RLD")
        chan.grid(row=1, column=0, sticky="ew", **pad)

        ttk.Label(chan, text="Ch").grid(row=0, column=0, **pad)
        ttk.Label(chan, text="On").grid(row=0, column=1, **pad)
        ttk.Label(chan, text="RLD").grid(row=0, column=2, **pad)

        self.chan_vars = []
        self.rld_vars = []
        for i in range(NUM_CHANNELS):
            ch_num = i + 1
            ttk.Label(chan, text=str(ch_num)).grid(row=i + 1, column=0, **pad)

            cv = tk.BooleanVar(value=True)
            self.chan_vars.append(cv)
            ttk.Checkbutton(
                chan,
                variable=cv,
                command=lambda c=ch_num, v=cv: self.on_channel_toggle(c, v),
            ).grid(row=i + 1, column=1, **pad)

            rv = tk.BooleanVar(value=False)
            self.rld_vars.append(rv)
            ttk.Checkbutton(
                chan,
                variable=rv,
                command=lambda c=ch_num, v=rv: self.on_rld_toggle(c, v),
            ).grid(row=i + 1, column=2, **pad)

        # Filters frame
        filt = ttk.LabelFrame(self, text="Filters (software, applied before LSL)")
        filt.grid(row=2, column=0, sticky="ew", **pad)

        self.notch_btn_off = ttk.Button(
            filt, text="Notch Off", command=lambda: self.set_notch(None)
        )
        self.notch_btn_off.grid(row=0, column=0, **pad)
        self.notch_btn_50 = ttk.Button(
            filt, text="Notch 50 Hz", command=lambda: self.set_notch(50)
        )
        self.notch_btn_50.grid(row=0, column=1, **pad)
        self.notch_btn_60 = ttk.Button(
            filt, text="Notch 60 Hz", command=lambda: self.set_notch(60)
        )
        self.notch_btn_60.grid(row=0, column=2, **pad)

        self.bp_btn = ttk.Button(
            filt, text="Bandpass 1-40 Hz: OFF", command=self.toggle_bandpass
        )
        self.bp_btn.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)

        # Stream control
        ctrl = ttk.LabelFrame(self, text="Stream")
        ctrl.grid(row=3, column=0, sticky="ew", **pad)
        self.start_btn = ttk.Button(
            ctrl, text="Start Stream", command=self.start_stream, state="disabled"
        )
        self.start_btn.grid(row=0, column=0, **pad)
        self.stop_btn = ttk.Button(
            ctrl, text="Stop Stream", command=self.stop_stream, state="disabled"
        )
        self.stop_btn.grid(row=0, column=1, **pad)

        # Log
        logf = ttk.LabelFrame(self, text="Log")
        logf.grid(row=4, column=0, sticky="ew", **pad)
        self.log_text = tk.Text(logf, width=52, height=10, state="disabled")
        self.log_text.grid(row=0, column=0, **pad)

        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # -- helpers -----------------------------------------------------------
    def log(self, msg):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def is_connected(self):
        return self.worker is not None and self.worker.connected

    # -- connection --------------------------------------------------------
    def toggle_connect(self):
        if self.is_connected():
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "No serial port selected.")
            return
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Error", f"Could not open {port}:\n{exc}")
            return

        gain = self.gain_var.get()
        self.worker = KnightWorker(self.ser, gain, self.log)
        # apply current filter selection to the worker
        self.worker.filter.set_notch(self.notch_freq)
        if self.bandpass_on:
            self.worker.filter.set_bandpass(1, 40)
        self.worker.start()

        self.connect_btn.configure(text="Disconnect")
        self.start_btn.configure(state="normal")
        self.gain_combo.configure(state="disabled")
        self.port_combo.configure(state="disabled")
        self.log(f"Connected to {port} @ {BAUD_RATE} baud (gain {gain}).")

    def disconnect(self):
        self.stop_stream()
        if self.worker is not None:
            self.worker.stop()
            self.worker.join(timeout=2)
            self.worker = None
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
            self.ser = None
        self.connect_btn.configure(text="Connect")
        self.start_btn.configure(state="disabled")
        self.stop_btn.configure(state="disabled")
        self.gain_combo.configure(state="readonly")
        self.port_combo.configure(state="readonly")
        self.log("Disconnected.")

    # -- streaming ---------------------------------------------------------
    def start_stream(self):
        if not self.is_connected():
            return
        gain = self.gain_var.get()
        # queue the configuration commands (each followed by a 1 s pause)
        for i in range(NUM_CHANNELS):
            ch = i + 1
            if self.chan_vars[i].get():
                self.worker.send_command(f"chon_{ch}_{gain}")
            else:
                self.worker.send_command(f"choff_{ch}")
        for i in range(NUM_CHANNELS):
            ch = i + 1
            if self.rld_vars[i].get():
                self.worker.send_command(f"rldadd_{ch}")
        self.worker.request_stream_start()
        self.start_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        self.log("Configuration queued; board type auto-detected after commands.")

    def stop_stream(self):
        if self.worker is not None and self.worker.streaming:
            self.worker.stop_streaming()
        self.start_btn.configure(
            state="normal" if self.is_connected() else "disabled"
        )
        self.stop_btn.configure(state="disabled")

    # -- live toggles ------------------------------------------------------
    def on_channel_toggle(self, ch, var):
        if not self.is_connected():
            return
        gain = self.gain_var.get()
        if var.get():
            self.worker.send_command(f"chon_{ch}_{gain}")
        else:
            self.worker.send_command(f"choff_{ch}")

    def on_rld_toggle(self, ch, var):
        if not self.is_connected():
            return
        if var.get():
            self.worker.send_command(f"rldadd_{ch}")
        else:
            self.worker.send_command(f"rldremove_{ch}")

    # -- filters -----------------------------------------------------------
    def set_notch(self, freq):
        self.notch_freq = freq
        if self.worker is not None:
            self.worker.filter.set_notch(freq)
        label = "Off" if freq is None else f"{freq} Hz"
        self.log(f"Notch filter: {label}")

    def toggle_bandpass(self):
        self.bandpass_on = not self.bandpass_on
        if self.worker is not None:
            self.worker.filter.set_bandpass(1, 40) if self.bandpass_on else \
                self.worker.filter.set_bandpass(None, None)
        state = "ON" if self.bandpass_on else "OFF"
        self.bp_btn.configure(text=f"Bandpass 1-40 Hz: {state}")
        self.log(f"Bandpass 1-40 Hz: {state}")

    # -- shutdown ----------------------------------------------------------
    def on_close(self):
        try:
            self.disconnect()
        finally:
            self.destroy()


if __name__ == "__main__":
    App().mainloop()
