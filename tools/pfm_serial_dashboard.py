import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD_RATE = 115200
PREDICTION_RE = re.compile(r"^\s+(Correct|Push|Rest):\s+([0-9.]+)")
TOP_RE = re.compile(r"^Top:\s+(\w+)\s+\(([0-9.]+)\)")
ANOMALY_RE = re.compile(r"^Anomaly score:\s+([0-9.]+)")
LEDS_RE = re.compile(r"^LEDs - Green:(ON|OFF) Red:(ON|OFF) Blue:(ON|OFF)")
SAMPLE_RE = re.compile(r"^Sample\s+\d+:\s+(.+)")
WINDOW_RE = re.compile(r"^Window end:\s+(\d+)\s+ms")


class SerialReader(threading.Thread):
    def __init__(self, port, output_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.handle = None

    def run(self):
        try:
            self.handle = serial.Serial(self.port, BAUD_RATE, timeout=0.2)
            self.output_queue.put(("status", f"Connected to {self.port} at {BAUD_RATE}"))
            while not self.stop_event.is_set():
                raw = self.handle.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.output_queue.put(("line", line))
        except Exception as exc:
            self.output_queue.put(("status", f"Serial error: {exc}"))
        finally:
            if self.handle and self.handle.is_open:
                self.handle.close()
            self.output_queue.put(("closed", "Disconnected"))


class Dashboard:
    COLORS = {
        "Correct": "#2e7d32",
        "Push": "#c62828",
        "Rest": "#1565c0",
        "Unknown": "#616161",
    }

    def __init__(self, root):
        self.root = root
        self.root.title("PFM Rehab Monitor")
        self.root.geometry("820x620")
        self.root.minsize(720, 560)

        self.output_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.reader = None

        self.predictions = {"Correct": 0.0, "Push": 0.0, "Rest": 0.0}
        self.top_label = "Unknown"
        self.top_value = 0.0
        self.last_update = None

        self.build_ui()
        self.refresh_ports()
        self.poll_queue()
        self.update_age()

    def build_ui(self):
        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)

        controls = ttk.Frame(root, padding=12)
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(1, weight=1)

        ttk.Label(controls, text="Port").grid(row=0, column=0, padx=(0, 8))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(controls, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(0, 8))
        ttk.Button(controls, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8))
        self.connect_button = ttk.Button(controls, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3)

        self.status_var = tk.StringVar(value="Choose a serial port and connect.")
        ttk.Label(root, textvariable=self.status_var, padding=(12, 0)).grid(row=1, column=0, sticky="ew")

        summary = ttk.Frame(root, padding=12)
        summary.grid(row=2, column=0, sticky="ew")
        summary.columnconfigure(0, weight=1)
        summary.columnconfigure(1, weight=1)

        self.top_card = tk.Frame(summary, bg=self.COLORS["Unknown"], height=110)
        self.top_card.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        self.top_card.grid_propagate(False)
        self.top_card.columnconfigure(0, weight=1)
        self.top_text = tk.StringVar(value="Waiting")
        self.conf_text = tk.StringVar(value="No prediction yet")
        tk.Label(self.top_card, textvariable=self.top_text, bg=self.COLORS["Unknown"], fg="white",
                 font=("Segoe UI", 30, "bold")).grid(row=0, column=0, sticky="s", pady=(14, 0))
        tk.Label(self.top_card, textvariable=self.conf_text, bg=self.COLORS["Unknown"], fg="white",
                 font=("Segoe UI", 12)).grid(row=1, column=0, sticky="n")

        metrics = ttk.Frame(summary)
        metrics.grid(row=0, column=1, sticky="nsew")
        metrics.columnconfigure(1, weight=1)

        self.age_var = tk.StringVar(value="Last update: never")
        self.anomaly_var = tk.StringVar(value="Anomaly: --")
        self.leds_var = tk.StringVar(value="LEDs: --")
        self.window_var = tk.StringVar(value="Window: --")
        ttk.Label(metrics, textvariable=self.age_var).grid(row=0, column=0, columnspan=2, sticky="w", pady=2)
        ttk.Label(metrics, textvariable=self.anomaly_var).grid(row=1, column=0, columnspan=2, sticky="w", pady=2)
        ttk.Label(metrics, textvariable=self.leds_var).grid(row=2, column=0, columnspan=2, sticky="w", pady=2)
        ttk.Label(metrics, textvariable=self.window_var).grid(row=3, column=0, columnspan=2, sticky="w", pady=2)

        bars = ttk.Frame(root, padding=(12, 0, 12, 12))
        bars.grid(row=3, column=0, sticky="nsew")
        bars.columnconfigure(1, weight=1)

        self.bar_vars = {}
        self.bar_labels = {}
        for row, label in enumerate(("Correct", "Push", "Rest")):
            ttk.Label(bars, text=label, width=8).grid(row=row, column=0, sticky="w", pady=4)
            var = tk.DoubleVar(value=0.0)
            self.bar_vars[label] = var
            ttk.Progressbar(bars, variable=var, maximum=1.0).grid(row=row, column=1, sticky="ew", pady=4)
            value = tk.StringVar(value="0.000")
            self.bar_labels[label] = value
            ttk.Label(bars, textvariable=value, width=8).grid(row=row, column=2, sticky="e", pady=4)

        bottom = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        bottom.grid(row=4, column=0, sticky="nsew", padx=12, pady=(0, 12))
        root.rowconfigure(4, weight=2)

        sample_frame = ttk.Labelframe(bottom, text="Latest Samples", padding=8)
        log_frame = ttk.Labelframe(bottom, text="Serial Log", padding=8)
        bottom.add(sample_frame, weight=1)
        bottom.add(log_frame, weight=2)

        self.samples = tk.Listbox(sample_frame, height=8)
        self.samples.pack(fill=tk.BOTH, expand=True)
        self.log = tk.Text(log_frame, height=8, wrap="none", state="disabled")
        self.log.pack(fill=tk.BOTH, expand=True)

    def refresh_ports(self):
        if list_ports is None:
            self.status_var.set("pyserial is not installed in this Python environment.")
            return
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if not ports:
            self.status_var.set("No serial ports found. Plug in the board and press Refresh.")

    def toggle_connection(self):
        if self.reader and self.reader.is_alive():
            self.stop_event.set()
            self.connect_button.configure(text="Connect")
            return

        port = self.port_var.get()
        if not port:
            self.status_var.set("Choose a serial port first.")
            return

        self.stop_event = threading.Event()
        self.reader = SerialReader(port, self.output_queue, self.stop_event)
        self.reader.start()
        self.connect_button.configure(text="Disconnect")

    def poll_queue(self):
        while True:
            try:
                kind, payload = self.output_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "line":
                self.handle_line(payload)
                self.append_log(payload)
            elif kind == "status":
                self.status_var.set(payload)
            elif kind == "closed":
                self.status_var.set(payload)
                self.connect_button.configure(text="Connect")

        self.root.after(50, self.poll_queue)

    def handle_line(self, line):
        prediction = PREDICTION_RE.match(line)
        if prediction:
            label, value = prediction.groups()
            self.predictions[label] = float(value)
            self.update_bars()
            return

        top = TOP_RE.match(line)
        if top:
            self.top_label, value = top.groups()
            self.top_value = float(value)
            self.last_update = time.time()
            self.update_top_card()
            return

        anomaly = ANOMALY_RE.match(line)
        if anomaly:
            self.anomaly_var.set(f"Anomaly: {float(anomaly.group(1)):.3f}")
            return

        leds = LEDS_RE.match(line)
        if leds:
            green, red, blue = leds.groups()
            self.leds_var.set(f"LEDs: Green {green} | Red {red} | Blue {blue}")
            return

        sample = SAMPLE_RE.match(line)
        if sample:
            self.samples.insert(0, sample.group(1))
            while self.samples.size() > 12:
                self.samples.delete(tk.END)
            return

        window = WINDOW_RE.match(line)
        if window:
            self.window_var.set(f"Window end: {window.group(1)} ms")

    def update_bars(self):
        for label, value in self.predictions.items():
            self.bar_vars[label].set(value)
            self.bar_labels[label].set(f"{value:.3f}")

    def update_top_card(self):
        color = self.COLORS.get(self.top_label, self.COLORS["Unknown"])
        self.top_card.configure(bg=color)
        for child in self.top_card.winfo_children():
            child.configure(bg=color)
        self.top_text.set(self.top_label)
        self.conf_text.set(f"Confidence {self.top_value:.3f}")

    def update_age(self):
        if self.last_update is None:
            self.age_var.set("Last update: never")
        else:
            age = time.time() - self.last_update
            self.age_var.set(f"Last update: {age:.1f}s ago")
        self.root.after(250, self.update_age)

    def append_log(self, line):
        self.log.configure(state="normal")
        self.log.insert(tk.END, line + "\n")
        self.log.see(tk.END)
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > 300:
            self.log.delete("1.0", "80.0")
        self.log.configure(state="disabled")

    def close(self):
        self.stop_event.set()
        self.root.destroy()


def main():
    root = tk.Tk()
    dashboard = Dashboard(root)
    root.protocol("WM_DELETE_WINDOW", dashboard.close)
    root.mainloop()


if __name__ == "__main__":
    main()
