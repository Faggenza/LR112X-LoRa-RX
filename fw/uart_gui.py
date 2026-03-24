#!/usr/bin/env python3

import queue
import threading
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


class SerialClient:
    def __init__(self):
        self.ser = None
        self._rx_thread = None
        self._rx_running = False
        self.rx_queue = queue.Queue()

    def connect(self, port: str, baud: int = 115200):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")

        self.disconnect()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._rx_thread.start()

    def disconnect(self):
        self._rx_running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send_line(self, line: str):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial port is not connected")
        payload = (line.strip() + "\r\n").encode("ascii", errors="ignore")
        self.ser.write(payload)

    def _reader_loop(self):
        while self._rx_running and self.ser is not None and self.ser.is_open:
            try:
                data = self.ser.read(256)
                if data:
                    text = data.decode("utf-8", errors="replace")
                    self.rx_queue.put(text)
            except Exception as exc:
                self.rx_queue.put(f"\n[Serial read error] {exc}\n")
                break


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("LR112X RX Runtime Control")
        self.geometry("760x520")

        self.client = SerialClient()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")

        self.freq_var = tk.StringVar(value="868030000")
        self.sf_var = tk.StringVar(value="12")
        self.cr_var = tk.StringVar(value="45")
        self.bw_var = tk.StringVar(value="125")
        self.preamble_var = tk.StringVar(value="8")
        self.payload_var = tk.StringVar(value="16")
        self.raw_cmd_var = tk.StringVar()
        self._pending_cr = False

        self._build_ui()
        self._refresh_ports()
        self.after(120, self._drain_rx_queue)

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        conn = ttk.LabelFrame(root, text="Serial")
        conn.pack(fill=tk.X, padx=2, pady=4)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, padx=4, pady=6, sticky="w")
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=4, pady=6, sticky="w")

        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=4, pady=6)

        ttk.Label(conn, text="Baud:").grid(row=0, column=3, padx=4, pady=6, sticky="e")
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=4, padx=4, pady=6, sticky="w")

        ttk.Button(conn, text="Connect", command=self._connect).grid(row=0, column=5, padx=4, pady=6)
        ttk.Button(conn, text="Disconnect", command=self._disconnect).grid(row=0, column=6, padx=4, pady=6)

        params = ttk.LabelFrame(root, text="Parameters")
        params.pack(fill=tk.X, padx=2, pady=4)

        self._param_row(params, 0, "Frequency (Hz)", self.freq_var, "SET FREQ")
        self._param_row(params, 1, "SF", self.sf_var, "SET SF", choices=[str(x) for x in range(5, 13)])
        self._param_row(params, 2, "CR", self.cr_var, "SET CR", choices=["45", "46", "47", "48"])
        self._param_row(params, 3, "BW (kHz)", self.bw_var, "SET BW", choices=["125", "250", "500", "200", "400", "800"])
        self._param_row(params, 4, "Preamble", self.preamble_var, "SET PREAMBLE")
        self._param_row(params, 5, "Payload Len", self.payload_var, "SET PAYLOAD")

        actions = ttk.Frame(root)
        actions.pack(fill=tk.X, padx=2, pady=4)
        ttk.Button(actions, text="Apply All", command=self._apply_all).pack(side=tk.LEFT, padx=2)
        ttk.Button(actions, text="RX START", command=lambda: self._send("RX START")).pack(side=tk.LEFT, padx=2)
        ttk.Button(actions, text="RX STOP", command=lambda: self._send("RX STOP")).pack(side=tk.LEFT, padx=2)
        ttk.Button(actions, text="SHOW", command=lambda: self._send("SHOW")).pack(side=tk.LEFT, padx=2)
        ttk.Button(actions, text="HELP", command=lambda: self._send("HELP")).pack(side=tk.LEFT, padx=2)

        raw = ttk.LabelFrame(root, text="Raw Command")
        raw.pack(fill=tk.X, padx=2, pady=4)
        ttk.Entry(raw, textvariable=self.raw_cmd_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4, pady=6)
        ttk.Button(raw, text="Send", command=self._send_raw).pack(side=tk.LEFT, padx=4, pady=6)

        log_frame = ttk.LabelFrame(root, text="Device Log")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=4)

        self.log = tk.Text(log_frame, height=16, wrap="word")
        self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log.configure(yscrollcommand=scroll.set)

    def _param_row(self, parent, row, label, var, command_prefix, choices=None):
        ttk.Label(parent, text=label).grid(row=row, column=0, padx=4, pady=4, sticky="w")
        if choices is None:
            ttk.Entry(parent, textvariable=var, width=18).grid(row=row, column=1, padx=4, pady=4, sticky="w")
        else:
            cb = ttk.Combobox(parent, textvariable=var, values=choices, width=16, state="readonly")
            cb.grid(row=row, column=1, padx=4, pady=4, sticky="w")

        ttk.Button(
            parent,
            text="Set",
            command=lambda: self._send(f"{command_prefix} {var.get().strip()}"),
        ).grid(row=row, column=2, padx=4, pady=4)

    def _refresh_ports(self):
        if serial is None:
            self.port_combo["values"] = []
            return

        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Select a serial port first")
            return

        try:
            baud = int(self.baud_var.get().strip())
            self.client.connect(port, baud)
            self._append_log(f"[Connected] {port} @ {baud}\n")
            self._send("PING")
            self._send("SHOW")
        except Exception as exc:
            messagebox.showerror("Connect error", str(exc))

    def _disconnect(self):
        self.client.disconnect()
        self._append_log("[Disconnected]\n")

    def _send(self, cmd: str):
        try:
            self.client.send_line(cmd)
            self._append_log(f"> {cmd}\n")
        except Exception as exc:
            messagebox.showerror("Send error", str(exc))

    def _apply_all(self):
        commands = [
            f"SET FREQ {self.freq_var.get().strip()}",
            f"SET SF {self.sf_var.get().strip()}",
            f"SET CR {self.cr_var.get().strip()}",
            f"SET BW {self.bw_var.get().strip()}",
            f"SET PREAMBLE {self.preamble_var.get().strip()}",
            f"SET PAYLOAD {self.payload_var.get().strip()}",
            "SHOW",
        ]

        for cmd in commands:
            self._send(cmd)

    def _send_raw(self):
        cmd = self.raw_cmd_var.get().strip()
        if not cmd:
            return
        self._send(cmd)
        self.raw_cmd_var.set("")

    def _drain_rx_queue(self):
        try:
            while True:
                text = self.client.rx_queue.get_nowait()
                self._append_log(text)
        except queue.Empty:
            pass
        self.after(120, self._drain_rx_queue)

    def _append_log(self, text: str):
        normalized = text

        normalized = normalized.replace("\\r\\n", "\n").replace("\\n", "\n").replace("\\r", "\n")

        if self._pending_cr:
            normalized = "\r" + normalized
            self._pending_cr = False

        if normalized.endswith("\r"):
            self._pending_cr = True
            normalized = normalized[:-1]

        normalized = normalized.replace("\r\n", "\n").replace("\r", "\n")
        self.log.insert(tk.END, normalized)
        self.log.see(tk.END)


if __name__ == "__main__":
    app = App()
    app.mainloop()
