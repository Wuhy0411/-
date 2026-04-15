import serial
import time
import threading
import queue
import csv
import bisect
import tkinter as tk
from tkinter import font, filedialog, messagebox
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# =========================
# User config
# =========================
ENCODER_PORT = "COM4"
ENCODER_BAUD = 9600
ENCODER_RESOLUTION = 32768          # 编码器每圈计数

TTL_PORT = "COM3"                   # Arduino 串口
TTL_BAUD = 115200

ENCODER_TARGET_HZ = 30.0

WHEEL_DIAMETER_M = 0.1              # 轮子直径（米），请根据实际修改


# =========================
# Modbus CRC
# =========================
def modbus_crc(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


# =========================
# Encoder serial thread
# =========================
class EncoderThread(threading.Thread):
    def __init__(self, port, baudrate, resolution, data_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.resolution = resolution
        self.data_queue = data_queue
        self.running = False
        self.ser = None
        self.fail_count = 0

    def run(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.04
            )
            self.running = True

            self.data_queue.put({
                "type": "status",
                "source": "encoder",
                "message": f"CONNECTED {self.port}"
            })

            cmd = bytes([0x01, 0x03, 0x00, 0x03, 0x00, 0x01])
            crc = modbus_crc(cmd)
            frame = cmd + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

            target_period = 1.0 / ENCODER_TARGET_HZ

            while self.running:
                loop_start = time.perf_counter()

                try:
                    self.ser.reset_input_buffer()
                    self.ser.write(frame)
                    resp = self.ser.read(7)
                    cpu_ns = time.perf_counter_ns()

                    if len(resp) == 7 and resp[0] == 0x01 and resp[1] == 0x03 and resp[2] == 0x02:
                        raw = (resp[3] << 8) | resp[4]
                        if raw >= 32768:
                            raw -= 65536

                        rpm = raw * 600.0 / self.resolution
                        self.fail_count = 0

                        self.data_queue.put({
                            "type": "data",
                            "source": "encoder",
                            "cpu_time_ns": cpu_ns,
                            "raw": raw,
                            "rpm": rpm,
                            "resp_hex": resp.hex(" ")
                        })
                    else:
                        self.fail_count += 1
                        if self.fail_count % 30 == 1:
                            self.data_queue.put({
                                "type": "status",
                                "source": "encoder",
                                "message": f"READ_FAIL {resp.hex(' ')}"
                            })

                except Exception as e:
                    self.data_queue.put({
                        "type": "status",
                        "source": "encoder",
                        "message": f"ERROR {e}"
                    })

                elapsed = time.perf_counter() - loop_start
                sleep_time = target_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            self.data_queue.put({
                "type": "status",
                "source": "encoder",
                "message": f"OPEN_FAIL {e}"
            })
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.data_queue.put({
                "type": "status",
                "source": "encoder",
                "message": "STOPPED"
            })

    def stop(self):
        self.running = False


# =========================
# TTL serial thread
# Expected line:
# TTL,<index>,<arduino_us>
# =========================
class TTLThread(threading.Thread):
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.data_queue = data_queue
        self.running = False
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.1
            )
            self.running = True

            self.data_queue.put({
                "type": "status",
                "source": "ttl",
                "message": f"CONNECTED {self.port}"
            })

            self.ser.reset_input_buffer()

            while self.running:
                try:
                    line_bytes = self.ser.readline()
                    if not line_bytes:
                        continue

                    cpu_ns = time.perf_counter_ns()
                    line = line_bytes.decode("ascii", errors="ignore").strip()
                    if not line:
                        continue

                    parts = line.split(",")

                    if len(parts) == 3 and parts[0] == "TTL":
                        try:
                            ttl_index = int(parts[1])
                            arduino_us = int(parts[2])

                            self.data_queue.put({
                                "type": "data",
                                "source": "ttl",
                                "cpu_time_ns": cpu_ns,
                                "ttl_index": ttl_index,
                                "ttl_arduino_us": arduino_us,
                                "line": line
                            })
                        except ValueError:
                            self.data_queue.put({
                                "type": "status",
                                "source": "ttl",
                                "message": f"PARSE_FAIL {line}"
                            })
                    else:
                        self.data_queue.put({
                            "type": "status",
                            "source": "ttl",
                            "message": f"INFO {line}"
                        })

                except Exception as e:
                    self.data_queue.put({
                        "type": "status",
                        "source": "ttl",
                        "message": f"ERROR {e}"
                    })

        except Exception as e:
            self.data_queue.put({
                "type": "status",
                "source": "ttl",
                "message": f"OPEN_FAIL {e}"
            })
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.data_queue.put({
                "type": "status",
                "source": "ttl",
                "message": "STOPPED"
            })

    def stop(self):
        self.running = False


# =========================
# GUI
# =========================
class DualSerialGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Encoder + TTL Recorder (with distance)")
        self.root.geometry("1100x680")
        self.root.configure(bg="#1e1e1e")

        self.base_cpu_ns = time.perf_counter_ns()

        # plot buffers, display only
        self.max_points = 300
        self.encoder_x_plot = deque(maxlen=self.max_points)
        self.encoder_y_plot = deque(maxlen=self.max_points)
        self.ttl_x_plot = deque(maxlen=self.max_points)

        self.data_queue = queue.Queue()
        self.encoder_thread = None
        self.ttl_thread = None

        # saved data, controlled by record button
        self.recording_started = False
        self.record_start_cpu_ns = 0
        self.encoder_records = []   # each: cpu_time_ns, record_rel_s, raw, rpm, distance_m
        self.ttl_records = []       # each: cpu_time_ns, record_rel_s, ttl_index, ttl_arduino_us

        # 路程累计相关
        self.wheel_diameter = WHEEL_DIAMETER_M
        self.circumference = 3.141592653589793 * self.wheel_diameter
        self.prev_raw = None
        self.total_distance_m = 0.0

        self.encoder_frame_count = 0
        self.ttl_frame_count = 0
        self.last_rate_time = time.perf_counter()

        self.latest_rpm = 0.0
        self.latest_raw = 0
        self.latest_ttl_index = 0
        self.latest_ttl_arduino_us = 0
        self.latest_ttl_rel_s = 0.0

        self.ttl_vlines = None

        self.setup_ui()
        self.start_threads()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.update_gui()

    def setup_ui(self):
        title_font = font.Font(family="Helvetica", size=13, weight="bold")
        info_font = font.Font(family="Consolas", size=11)

        top_frame = tk.Frame(self.root, bg="#1e1e1e")
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=16, pady=10)

        tk.Label(
            top_frame,
            text="Encoder RPM with TTL Markers (with distance)",
            font=title_font,
            bg="#1e1e1e",
            fg="#ffffff"
        ).pack(side=tk.LEFT)

        # Buttons for record control
        button_frame = tk.Frame(top_frame, bg="#1e1e1e")
        button_frame.pack(side=tk.RIGHT)

        self.start_record_btn = tk.Button(
            button_frame,
            text="Start Record",
            command=self.on_start_recording,
            bg="#2ecc71",
            fg="white",
            font=info_font,
            padx=10
        )
        self.start_record_btn.pack(side=tk.LEFT, padx=5)

        self.stop_record_btn = tk.Button(
            button_frame,
            text="Stop Record",
            command=self.on_stop_recording,
            bg="#e74c3c",
            fg="white",
            font=info_font,
            padx=10,
            state=tk.DISABLED
        )
        self.stop_record_btn.pack(side=tk.LEFT, padx=5)

        info_frame = tk.Frame(self.root, bg="#1e1e1e")
        info_frame.pack(side=tk.TOP, fill=tk.X, padx=16, pady=4)

        self.encoder_status_label = tk.Label(
            info_frame, text="ENC: connecting...",
            font=info_font, bg="#1e1e1e", fg="#f39c12"
        )
        self.encoder_status_label.grid(row=0, column=0, padx=10, pady=4, sticky="w")

        self.ttl_status_label = tk.Label(
            info_frame, text="TTL: connecting...",
            font=info_font, bg="#1e1e1e", fg="#f39c12"
        )
        self.ttl_status_label.grid(row=0, column=1, padx=10, pady=4, sticky="w")

        self.record_status_label = tk.Label(
            info_frame, text="REC: idle (press Start)",
            font=info_font, bg="#1e1e1e", fg="#f1c40f"
        )
        self.record_status_label.grid(row=0, column=2, padx=10, pady=4, sticky="w")

        self.encoder_rate_label = tk.Label(
            info_frame, text="ENC rate: 0.0 Hz",
            font=info_font, bg="#1e1e1e", fg="#00d2ff"
        )
        self.encoder_rate_label.grid(row=0, column=3, padx=10, pady=4, sticky="w")

        self.ttl_rate_label = tk.Label(
            info_frame, text="TTL rate: 0.0 Hz",
            font=info_font, bg="#1e1e1e", fg="#ffb347"
        )
        self.ttl_rate_label.grid(row=0, column=4, padx=10, pady=4, sticky="w")

        self.rpm_label = tk.Label(
            info_frame, text="RPM: 0.00",
            font=info_font, bg="#1e1e1e", fg="#2ecc71"
        )
        self.rpm_label.grid(row=1, column=0, padx=10, pady=4, sticky="w")

        self.raw_label = tk.Label(
            info_frame, text="RAW: 0",
            font=info_font, bg="#1e1e1e", fg="#2ecc71"
        )
        self.raw_label.grid(row=1, column=1, padx=10, pady=4, sticky="w")

        self.ttl_count_label = tk.Label(
            info_frame, text="TTL count: 0",
            font=info_font, bg="#1e1e1e", fg="#ffb347"
        )
        self.ttl_count_label.grid(row=1, column=2, padx=10, pady=4, sticky="w")

        self.ttl_last_label = tk.Label(
            info_frame, text="Last TTL: 0.000000 s",
            font=info_font, bg="#1e1e1e", fg="#ffb347"
        )
        self.ttl_last_label.grid(row=1, column=3, padx=10, pady=4, sticky="w")

        self.ttl_index_label = tk.Label(
            info_frame, text="TTL index: 0",
            font=info_font, bg="#1e1e1e", fg="#ffb347"
        )
        self.ttl_index_label.grid(row=2, column=0, padx=10, pady=4, sticky="w")

        self.ttl_arduino_label = tk.Label(
            info_frame, text="Arduino us: 0",
            font=info_font, bg="#1e1e1e", fg="#ffb347"
        )
        self.ttl_arduino_label.grid(row=2, column=1, padx=10, pady=4, sticky="w")

        self.saved_encoder_label = tk.Label(
            info_frame, text="Saved ENC: 0",
            font=info_font, bg="#1e1e1e", fg="#9b59b6"
        )
        self.saved_encoder_label.grid(row=2, column=2, padx=10, pady=4, sticky="w")

        self.saved_ttl_label = tk.Label(
            info_frame, text="Saved TTL: 0",
            font=info_font, bg="#1e1e1e", fg="#9b59b6"
        )
        self.saved_ttl_label.grid(row=2, column=3, padx=10, pady=4, sticky="w")

        plt.style.use("dark_background")
        self.fig, self.ax = plt.subplots(figsize=(9.5, 4.9), dpi=100)
        self.fig.patch.set_facecolor("#1e1e1e")
        self.ax.set_facecolor("#1e1e1e")

        for spine in self.ax.spines.values():
            spine.set_visible(False)

        self.ax.tick_params(colors="#888888")
        self.ax.grid(True, linestyle="--", color="#333333", alpha=0.7)
        self.ax.set_xlabel("CPU time since app start (s)", color="#888888")
        self.ax.set_ylabel("Speed (RPM)", color="#888888")
        self.ax.set_title("Real-time RPM with TTL vertical markers", color="#ffffff")

        self.line, = self.ax.plot([], [], color="#00d2ff", linewidth=2, label="RPM")
        self.ax.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        bottom_frame = tk.Frame(self.root, bg="#1e1e1e")
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=16, pady=8)

        self.hint_label = tk.Label(
            bottom_frame,
            text=f"Encoder: {ENCODER_PORT} @ {ENCODER_BAUD} | TTL: {TTL_PORT} @ {TTL_BAUD} | Wheel diam: {WHEEL_DIAMETER_M} m",
            font=info_font,
            bg="#1e1e1e",
            fg="#aaaaaa"
        )
        self.hint_label.pack(side=tk.LEFT)

    def start_threads(self):
        self.encoder_thread = EncoderThread(
            ENCODER_PORT, ENCODER_BAUD, ENCODER_RESOLUTION, self.data_queue
        )
        self.ttl_thread = TTLThread(
            TTL_PORT, TTL_BAUD, self.data_queue
        )

        self.encoder_thread.start()
        self.ttl_thread.start()

    def redraw_plot(self):
        if len(self.encoder_x_plot) == 0:
            return

        xs = list(self.encoder_x_plot)
        ys = list(self.encoder_y_plot)

        self.line.set_data(xs, ys)

        xmin = xs[0]
        xmax = xs[-1]
        if xmax <= xmin:
            xmax = xmin + 1.0
        self.ax.set_xlim(xmin, xmax)

        ymin = min(ys)
        ymax = max(ys)
        if ymax == ymin:
            pad = 10.0
        else:
            pad = max(5.0, (ymax - ymin) * 0.2)

        ylow = ymin - pad
        yhigh = ymax + pad
        self.ax.set_ylim(ylow, yhigh)

        if self.ttl_vlines is not None:
            try:
                self.ttl_vlines.remove()
            except Exception:
                pass
            self.ttl_vlines = None

        visible_ttl_x = [x for x in self.ttl_x_plot if xmin <= x <= xmax]
        if visible_ttl_x:
            self.ttl_vlines = self.ax.vlines(
                visible_ttl_x,
                ylow,
                yhigh,
                colors="#ffb347",
                linewidth=1.2,
                alpha=0.85,
                label="_nolegend_"
            )

        self.canvas.draw_idle()

    def on_start_recording(self):
        """Start recording data (clear previous records)."""
        if self.recording_started:
            messagebox.showinfo("Info", "Already recording. Please stop first if you want to restart.")
            return

        # Reset record state
        self.recording_started = True
        self.record_start_cpu_ns = time.perf_counter_ns()
        self.encoder_records.clear()
        self.ttl_records.clear()
        # Reset distance accumulator
        self.prev_raw = None
        self.total_distance_m = 0.0

        # Update GUI
        self.record_status_label.config(text="REC: recording...", fg="#2ecc71")
        self.saved_encoder_label.config(text="Saved ENC: 0")
        self.saved_ttl_label.config(text="Saved TTL: 0")
        self.start_record_btn.config(state=tk.DISABLED)
        self.stop_record_btn.config(state=tk.NORMAL)

    def on_stop_recording(self):
        """Stop recording, keep existing data."""
        if not self.recording_started:
            return
        self.recording_started = False
        self.record_status_label.config(text="REC: stopped", fg="#e67e22")
        self.start_record_btn.config(state=tk.NORMAL)
        self.stop_record_btn.config(state=tk.DISABLED)

    def update_gui(self):
        plot_updated = False

        while not self.data_queue.empty():
            evt = self.data_queue.get_nowait()

            if evt["type"] == "status":
                if evt["source"] == "encoder":
                    msg = evt["message"]
                    color = "#2ecc71" if msg.startswith("CONNECTED") else "#e67e22"
                    if msg.startswith("OPEN_FAIL") or msg.startswith("ERROR"):
                        color = "#e74c3c"
                    self.encoder_status_label.config(text=f"ENC: {msg}", fg=color)

                elif evt["source"] == "ttl":
                    msg = evt["message"]
                    color = "#2ecc71" if msg.startswith("CONNECTED") else "#e67e22"
                    if msg.startswith("OPEN_FAIL") or msg.startswith("ERROR"):
                        color = "#e74c3c"
                    self.ttl_status_label.config(text=f"TTL: {msg}", fg=color)

            elif evt["type"] == "data":
                display_rel_time_s = (evt["cpu_time_ns"] - self.base_cpu_ns) / 1e9

                # display path
                if evt["source"] == "encoder":
                    self.encoder_x_plot.append(display_rel_time_s)
                    self.encoder_y_plot.append(evt["rpm"])

                    self.latest_rpm = evt["rpm"]
                    self.latest_raw = evt["raw"]

                    self.rpm_label.config(text=f"RPM: {evt['rpm']:.2f}")
                    self.raw_label.config(text=f"RAW: {evt['raw']}")
                    self.encoder_frame_count += 1
                    plot_updated = True

                    # save if recording active
                    if self.recording_started and evt["cpu_time_ns"] >= self.record_start_cpu_ns:
                        raw = evt["raw"]
                        # 计算路程增量（基于 raw 的变化）
                        if self.prev_raw is None:
                            delta_raw_abs = 0
                        else:
                            delta = raw - self.prev_raw
                            # 处理 wrap-around（32768 模）
                            if delta > ENCODER_RESOLUTION // 2:
                                delta -= ENCODER_RESOLUTION
                            elif delta < -ENCODER_RESOLUTION // 2:
                                delta += ENCODER_RESOLUTION
                            delta_raw_abs = abs(delta)
                        delta_distance = delta_raw_abs / ENCODER_RESOLUTION * self.circumference
                        self.total_distance_m += delta_distance
                        self.prev_raw = raw

                        rec = {
                            "cpu_time_ns": evt["cpu_time_ns"],
                            "record_rel_s": (evt["cpu_time_ns"] - self.record_start_cpu_ns) / 1e9,
                            "raw": raw,
                            "rpm": evt["rpm"],
                            "distance_m": self.total_distance_m
                        }
                        self.encoder_records.append(rec)
                        self.saved_encoder_label.config(text=f"Saved ENC: {len(self.encoder_records)}")

                elif evt["source"] == "ttl":
                    self.ttl_x_plot.append(display_rel_time_s)

                    self.latest_ttl_rel_s = display_rel_time_s
                    self.latest_ttl_index = evt["ttl_index"]
                    self.latest_ttl_arduino_us = evt["ttl_arduino_us"]

                    self.ttl_count_label.config(text=f"TTL count: {len(self.ttl_x_plot)}")
                    self.ttl_last_label.config(text=f"Last TTL: {display_rel_time_s:.6f} s")
                    self.ttl_index_label.config(text=f"TTL index: {evt['ttl_index']}")
                    self.ttl_arduino_label.config(text=f"Arduino us: {evt['ttl_arduino_us']}")
                    self.ttl_frame_count += 1
                    plot_updated = True

                    # save TTL if recording active
                    if self.recording_started and evt["cpu_time_ns"] >= self.record_start_cpu_ns:
                        rec = {
                            "cpu_time_ns": evt["cpu_time_ns"],
                            "record_rel_s": (evt["cpu_time_ns"] - self.record_start_cpu_ns) / 1e9,
                            "ttl_index": evt["ttl_index"],
                            "ttl_arduino_us": evt["ttl_arduino_us"]
                        }
                        self.ttl_records.append(rec)
                        self.saved_ttl_label.config(text=f"Saved TTL: {len(self.ttl_records)}")

        now = time.perf_counter()
        elapsed = now - self.last_rate_time
        if elapsed >= 1.0:
            enc_rate = self.encoder_frame_count / elapsed
            ttl_rate = self.ttl_frame_count / elapsed

            self.encoder_rate_label.config(text=f"ENC rate: {enc_rate:.1f} Hz")
            self.ttl_rate_label.config(text=f"TTL rate: {ttl_rate:.1f} Hz")

            self.encoder_frame_count = 0
            self.ttl_frame_count = 0
            self.last_rate_time = now

        if plot_updated:
            self.redraw_plot()

        self.root.after(30, self.update_gui)

    def build_encoder_rows_with_ttl(self):
        """
        将 TTL 事件匹配到最近的编码器样本上。
        返回列表，每个元素是一个编码器样本，增加 ttl_event 和 ttl_index 字段。
        """
        if not self.encoder_records:
            return []

        # 复制并排序
        enc_sorted = sorted(self.encoder_records, key=lambda x: x["cpu_time_ns"])
        ttl_sorted = sorted(self.ttl_records, key=lambda x: x["cpu_time_ns"])

        # 为每个编码器样本初始化 ttl_event = 0, ttl_index = None
        for enc in enc_sorted:
            enc["ttl_event"] = 0
            enc["ttl_index"] = None

        # 为每个 TTL 找到最近的编码器样本
        for ttl in ttl_sorted:
            ttl_time = ttl["cpu_time_ns"]
            enc_times = [e["cpu_time_ns"] for e in enc_sorted]
            idx = bisect.bisect_left(enc_times, ttl_time)
            best_enc = None
            best_dt = None
            if idx < len(enc_sorted):
                dt = abs(enc_sorted[idx]["cpu_time_ns"] - ttl_time)
                best_enc = enc_sorted[idx]
                best_dt = dt
            if idx > 0:
                dt = abs(enc_sorted[idx-1]["cpu_time_ns"] - ttl_time)
                if best_dt is None or dt < best_dt:
                    best_enc = enc_sorted[idx-1]
                    best_dt = dt
            if best_enc is not None:
                # 如果该编码器样本已经被之前的TTL匹配过，保留距离更近的
                if best_enc["ttl_event"] == 0 or best_dt < best_enc.get("ttl_dt", float('inf')):
                    best_enc["ttl_event"] = 1
                    best_enc["ttl_index"] = ttl["ttl_index"]
                    best_enc["ttl_dt"] = best_dt
        # 移除临时字段 ttl_dt
        for enc in enc_sorted:
            enc.pop("ttl_dt", None)
        return enc_sorted

    def save_marked_encoder_csv(self, filepath):
        """保存每个编码器样本，包含累计路程和 TTL 标记"""
        rows = self.build_encoder_rows_with_ttl()
        if not rows:
            messagebox.showwarning("No data", "No encoder data to save.")
            return False

        try:
            with open(filepath, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "record_rel_s", "cpu_time_ns", "raw", "rpm", "distance_m", "ttl_event", "ttl_index"
                ])
                for rec in rows:
                    writer.writerow([
                        f"{rec['record_rel_s']:.9f}",
                        rec["cpu_time_ns"],
                        rec["raw"],
                        f"{rec['rpm']:.6f}",
                        f"{rec['distance_m']:.6f}",
                        rec["ttl_event"],
                        rec["ttl_index"] if rec["ttl_index"] is not None else ""
                    ])
            return True
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save CSV:\n{e}")
            return False

    def on_closing(self):
        if self.encoder_thread:
            self.encoder_thread.stop()
            self.encoder_thread.join(timeout=1.0)

        if self.ttl_thread:
            self.ttl_thread.stop()
            self.ttl_thread.join(timeout=1.0)

        if self.encoder_records:
            answer = messagebox.askyesno("Save Data", f"Recorded {len(self.encoder_records)} encoder samples. Save CSV with distance and TTL markers?")
            if answer:
                filepath = filedialog.asksaveasfilename(
                    title="Save Encoder CSV with distance and TTL",
                    defaultextension=".csv",
                    filetypes=[("CSV files", "*.csv")],
                    initialfile="encoder_data.csv"
                )
                if filepath:
                    self.save_marked_encoder_csv(filepath)
        elif self.ttl_records:
            messagebox.showinfo("No encoder data", "Only TTL events recorded, no encoder data to save.")
        else:
            pass

        plt.close(self.fig)
        self.root.destroy()


# =========================
# Main
# =========================
if __name__ == "__main__":
    root = tk.Tk()
    app = DualSerialGUI(root)
    root.mainloop()