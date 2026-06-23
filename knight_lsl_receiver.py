"""
NeuroPawn Knight LSL receiver + visualizer (barebones GUI).

Resolves an LSL stream (default name "NeuroPawnKnight"), pulls samples, and
plots a scrolling multi-channel time series. The channel count and labels are
read from the stream metadata, so it automatically adapts to the non-IMU
(8 channels) or IMU (17 channels) version produced by knight_lsl_gui.py.
"""

import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from pylsl import StreamInlet, resolve_streams

WINDOW_SECONDS = 5.0     # how much history to show
REDRAW_MS = 40           # ~25 FPS


class ReceiverApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("NeuroPawn Knight - LSL Receiver")
        self.geometry("900x650")

        self.inlet = None
        self.srate = 0.0
        self.n_channels = 0
        self.labels = []
        self.buffer = None       # shape (n_channels, n_samples)
        self.lines = []
        self.axes = []
        self.streams = []
        self._after_id = None

        self._build_ui()

    # -- UI ----------------------------------------------------------------
    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(side="top", fill="x", padx=6, pady=6)

        ttk.Label(top, text="Stream:").pack(side="left", padx=4)
        self.stream_var = tk.StringVar()
        self.stream_combo = ttk.Combobox(
            top, textvariable=self.stream_var, width=30, state="readonly"
        )
        self.stream_combo.pack(side="left", padx=4)

        ttk.Button(top, text="Refresh", command=self.refresh_streams).pack(
            side="left", padx=4
        )
        self.connect_btn = ttk.Button(
            top, text="Connect", command=self.toggle_connect
        )
        self.connect_btn.pack(side="left", padx=4)

        ttk.Label(top, text="Scale (uV):").pack(side="left", padx=(16, 4))
        self.scale_var = tk.DoubleVar(value=200.0)
        ttk.Spinbox(
            top, from_=1, to=100000, increment=50, width=8,
            textvariable=self.scale_var,
        ).pack(side="left", padx=4)

        self.status_var = tk.StringVar(value="Not connected.")
        ttk.Label(self, textvariable=self.status_var).pack(
            side="bottom", fill="x", padx=6, pady=2
        )

        self.fig = Figure(figsize=(8, 5), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(
            side="top", fill="both", expand=True, padx=6, pady=6
        )

        self.refresh_streams()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # -- stream discovery --------------------------------------------------
    def refresh_streams(self):
        self.streams = resolve_streams(wait_time=1.0)
        names = [
            f"{s.name()} ({s.type()}, {s.channel_count()}ch)"
            for s in self.streams
        ]
        self.stream_combo["values"] = names
        if names:
            # prefer the NeuroPawn stream if present
            idx = next(
                (i for i, s in enumerate(self.streams)
                 if s.name() == "NeuroPawnKnight"),
                0,
            )
            self.stream_combo.current(idx)
        self.status_var.set(f"Found {len(names)} stream(s).")

    # -- connection --------------------------------------------------------
    def toggle_connect(self):
        if self.inlet is not None:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        idx = self.stream_combo.current()
        if idx < 0 or idx >= len(self.streams):
            messagebox.showerror("Error", "No stream selected.")
            return
        info = self.streams[idx]
        self.inlet = StreamInlet(info, max_buflen=int(WINDOW_SECONDS) + 2)
        full = self.inlet.info()

        self.n_channels = full.channel_count()
        self.srate = full.nominal_srate() or 125.0
        self.labels = self._read_labels(full, self.n_channels)

        n_samples = max(1, int(self.srate * WINDOW_SECONDS))
        self.buffer = np.zeros((self.n_channels, n_samples), dtype=np.float64)

        self._build_axes()
        self.connect_btn.configure(text="Disconnect")
        self.status_var.set(
            f"Connected to {info.name()} - {self.n_channels} ch @ "
            f"{self.srate:.0f} Hz."
        )
        self._after_id = self.after(REDRAW_MS, self._update)

    def disconnect(self):
        if self._after_id is not None:
            self.after_cancel(self._after_id)
            self._after_id = None
        if self.inlet is not None:
            try:
                self.inlet.close_stream()
            except Exception:  # noqa: BLE001
                pass
            self.inlet = None
        self.connect_btn.configure(text="Connect")
        self.status_var.set("Disconnected.")

    @staticmethod
    def _read_labels(info, n_channels):
        labels = []
        try:
            ch = info.desc().child("channels").child("channel")
            for _ in range(n_channels):
                label = ch.child_value("label")
                labels.append(label if label else "?")
                ch = ch.next_sibling()
        except Exception:  # noqa: BLE001
            labels = []
        if len(labels) != n_channels or any(l == "?" for l in labels):
            labels = [f"ch{i + 1}" for i in range(n_channels)]
        return labels

    # -- plotting ----------------------------------------------------------
    def _build_axes(self):
        self.fig.clear()
        self.axes = []
        self.lines = []
        n = self.n_channels
        n_samples = self.buffer.shape[1]
        t = np.linspace(-WINDOW_SECONDS, 0, n_samples)
        for i in range(n):
            ax = self.fig.add_subplot(n, 1, i + 1)
            (line,) = ax.plot(t, self.buffer[i], linewidth=0.8)
            ax.set_ylabel(self.labels[i], rotation=0, ha="right",
                          va="center", fontsize=8)
            ax.set_xlim(-WINDOW_SECONDS, 0)
            ax.set_yticks([])
            if i < n - 1:
                ax.set_xticks([])
            else:
                ax.set_xlabel("Time (s)")
            self.axes.append(ax)
            self.lines.append(line)
        self.fig.subplots_adjust(
            left=0.12, right=0.99, top=0.99, bottom=0.06, hspace=0.1
        )
        self.canvas.draw()

    def _update(self):
        if self.inlet is None:
            return
        chunk, _ = self.inlet.pull_chunk(timeout=0.0)
        if chunk:
            data = np.asarray(chunk, dtype=np.float64).T  # (n_channels, n)
            k = data.shape[1]
            if k >= self.buffer.shape[1]:
                self.buffer = data[:, -self.buffer.shape[1]:]
            else:
                self.buffer = np.roll(self.buffer, -k, axis=1)
                self.buffer[:, -k:] = data

            scale = self.scale_var.get()
            for i, line in enumerate(self.lines):
                line.set_ydata(self.buffer[i])
                # EXG channels: fixed uV scale; IMU channels: autoscale
                if self.labels[i].startswith("EXG"):
                    self.axes[i].set_ylim(-scale, scale)
                else:
                    lo = float(np.min(self.buffer[i]))
                    hi = float(np.max(self.buffer[i]))
                    if hi - lo < 1e-6:
                        lo, hi = lo - 1, hi + 1
                    self.axes[i].set_ylim(lo, hi)
            self.canvas.draw_idle()

        self._after_id = self.after(REDRAW_MS, self._update)

    # -- shutdown ----------------------------------------------------------
    def on_close(self):
        try:
            self.disconnect()
        finally:
            self.destroy()


if __name__ == "__main__":
    ReceiverApp().mainloop()
