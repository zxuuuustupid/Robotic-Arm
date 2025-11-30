import tkinter as tk
from tkinter import messagebox, ttk, scrolledtext
import numpy as np
import math
import serial
import serial.tools.list_ports
import struct
import time
import threading

# =========================================================
# 1. 核心算法库 (保持不变)
# =========================================================

class RobotAlgorithm:
    def __init__(self):
        # ========== 机械臂尺寸参数 (单位 mm) ==========
        self.L1 = 185.0
        self.L2 = 210.0
        self.L3 = 60.0

        # 机械臂原点偏差
        self.X_bias = -35.89
        self.Y_bias = 70.0
        self.Z_bias = 0.0

        # ========== 书写范围映射参数 ==========
        self.physical_width_mm = 45.0
        self.physical_height_mm = 135.0
        self.canvas_w_ref = 600.0
        self.canvas_h_ref = 600.0

        # ========== 高度参数 ==========
        self.z_write = 355.0
        self.z_lift = 335.0
        self.tilt_angle_deg = -12

    def inverse_3link_robot(self, X, Y, Z):
        x_move = X
        Ye = Y - self.L3
        Ze = Z
        D = math.sqrt(Ye**2 + Ze**2)
        try:
            cos_a2 = (D**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            cos_a2 = max(min(cos_a2, 1.0), -1.0)
            a2_rad = math.acos(cos_a2)
        except ValueError:
            return 0, 0, 0, 0

        phi = math.atan2(Ze, Ye)
        psi = math.atan2(self.L2 * math.sin(a2_rad), self.L1 - self.L2 * math.cos(a2_rad))
        a1_rad = phi + psi

        a1 = 180 - math.degrees(a1_rad)
        a2 = 180 - math.degrees(a2_rad)
        a3 = 180 - (a1 + a2) - 20

        return x_move, a1, a2, a3

    def generate_stroke_trajectory(self, raw_stroke_pixels):
        stroke_np = np.array(raw_stroke_pixels)
        if stroke_np.shape[0] < 2:
            return np.zeros((0, 3))

        diff_points = np.diff(stroke_np, axis=0)
        segment_lengths = np.sqrt(np.sum(diff_points**2, axis=1))
        total_length = np.sum(segment_lengths)

        # 插值密度: 每 2 像素一个点
        num_new_points = max(2, int(round(total_length / 2.0)))

        new_t = np.linspace(0, 1, num_new_points)
        dist_cumulative = np.insert(np.cumsum(segment_lengths), 0, 0)
        if total_length == 0: total_length = 1e-9
        t = dist_cumulative / total_length

        new_x = np.interp(new_t, t, stroke_np[:, 0])
        new_y = np.interp(new_t, t, stroke_np[:, 1])
        resampled_stroke = np.column_stack((new_x, new_y))

        scale_x = self.physical_width_mm / self.canvas_w_ref
        scale_y = self.physical_height_mm / self.canvas_h_ref

        scaled_stroke = resampled_stroke.copy()
        scaled_stroke[:, 0] = scaled_stroke[:, 0] * scale_x
        scaled_stroke[:, 1] = (self.canvas_h_ref - scaled_stroke[:, 1]) * scale_y

        full_trajectory = []
        if len(scaled_stroke) > 0:
            start_pos = scaled_stroke[0]
            last_pos = scaled_stroke[-1]
            # 抬笔移过去 -> 落笔 -> 写 -> 抬笔
            full_trajectory.append([start_pos[0], start_pos[1], self.z_lift])
            full_trajectory.append([start_pos[0], start_pos[1], self.z_write])
            for i in range(len(scaled_stroke)):
                full_trajectory.append([scaled_stroke[i][0], scaled_stroke[i][1], self.z_write])
            full_trajectory.append([last_pos[0], last_pos[1], self.z_lift])

        final_points = np.array(full_trajectory)
        if final_points.shape[0] > 0:
            tilt_rad = np.radians(self.tilt_angle_deg)
            y_pivot = np.max(final_points[:, 1])
            z_tilt_offset = (final_points[:, 1] - y_pivot) * np.tan(tilt_rad)
            final_points[:, 2] += z_tilt_offset
            final_points[:, 0] += self.X_bias
            final_points[:, 1] += self.Y_bias
            final_points[:, 2] += self.Z_bias

        return final_points

    def solve_ik_batch(self, points_3d):
        if points_3d.shape[0] == 0:
            return []
        n_points = points_3d.shape[0]
        result_int = []
        for i in range(n_points):
            X, Y, Z = points_3d[i, :]
            xm, a1, a2, a3 = self.inverse_3link_robot(X, Y, Z)
            val1 = int(round(xm * 100))
            val2 = int(round(a1 * 10))
            val3 = int(round(a2 * 10))
            val4 = int(round(a3 * 10))
            result_int.append((val1, val2, val3, val4))
        return result_int


# =========================================================
# 2. GUI 应用程序
# =========================================================

class HandwritingApp:
    def __init__(self, root, serial_port='COM3', baud_rate=115200):
        self.root = root
        self.root.title("机械臂书写系统 (带数据监视)")
        self.root.geometry("650x850") # 增加高度以容纳日志窗口

        self.algo = RobotAlgorithm()
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.current_stroke_pixels = []

        self.setup_ui()
        self.connect_serial()

    def setup_ui(self):
        # --- 顶部：连接控制 ---
        control_frame = tk.Frame(self.root)
        control_frame.pack(fill=tk.X, pady=5, padx=10)

        tk.Label(control_frame, text="串口:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(control_frame, width=10)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if self.serial_port in ports:
            self.port_combo.set(self.serial_port)
        elif ports:
            self.port_combo.current(0)

        tk.Button(control_frame, text="连接/重连", command=self.connect_serial).pack(side=tk.LEFT, padx=5)
        self.status_label = tk.Label(control_frame, text="未连接", fg="red")
        self.status_label.pack(side=tk.LEFT, padx=10)

        self.send_info = tk.Label(control_frame, text="等待书写...", fg="blue")
        self.send_info.pack(side=tk.RIGHT, padx=10)

        # --- 中部：画布 ---
        self.canvas_width = 600
        self.canvas_height = 500 # 稍微调小一点画布高度，留位置给日志
        self.algo.canvas_w_ref = float(self.canvas_width)
        self.algo.canvas_h_ref = float(self.canvas_height)

        self.canvas = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height,
                                bg='white', cursor="pencil", relief="sunken", borderwidth=2)
        self.canvas.pack(pady=5)

        # --- 按钮栏 ---
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(fill=tk.X, pady=5)
        tk.Button(btn_frame, text="清空画布 & 日志", command=self.clear_all, width=20, bg="#f0f0f0").pack()

        # --- 底部：数据监视器 ---
        monitor_frame = tk.LabelFrame(self.root, text="发送数据监视 (Packet Monitor)", padx=5, pady=5)
        monitor_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # 监视器控制
        chk_frame = tk.Frame(monitor_frame)
        chk_frame.pack(fill=tk.X)
        self.show_log_var = tk.BooleanVar(value=True) # 默认开启
        tk.Checkbutton(chk_frame, text="启用实时日志 (Debug)", variable=self.show_log_var).pack(side=tk.LEFT)
        tk.Label(chk_frame, text="格式: <Header AA 55> <Data 8 bytes> <Checksum>", fg="gray").pack(side=tk.RIGHT)

        # 滚动文本框
        self.log_text = scrolledtext.ScrolledText(monitor_frame, height=8, font=("Consolas", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # 绑定事件
        self.canvas.bind("<Button-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_move)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)

    def connect_serial(self):
        port = self.port_combo.get()
        if not port: port = self.serial_port
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(port, self.baud_rate, timeout=1)
            self.status_label.config(text=f"已连接 {port}", fg="green")
            self.append_log(f"[System] 串口 {port} 已连接")
        except Exception as e:
            self.status_label.config(text="连接失败", fg="red")
            self.append_log(f"[Error] 串口错误: {e}")
            self.ser = None

    # ========== 鼠标操作 ==========
    def on_mouse_down(self, event):
        self.current_stroke_pixels = []
        self.last_x, self.last_y = event.x, event.y
        self.current_stroke_pixels.append([event.x, event.y])

    def on_mouse_move(self, event):
        x, y = event.x, event.y
        if len(self.current_stroke_pixels) > 0:
            last_pt = self.current_stroke_pixels[-1]
            if (x - last_pt[0])**2 + (y - last_pt[1])**2 < 9: return

        self.current_stroke_pixels.append([x, y])
        self.canvas.create_line(self.last_x, self.last_y, x, y, width=3, fill='black', capstyle=tk.ROUND)
        self.last_x, self.last_y = x, y

    def on_mouse_up(self, event):
        if len(self.current_stroke_pixels) > 1:
            stroke_data = list(self.current_stroke_pixels)
            # 启动发送线程
            threading.Thread(target=self.process_and_send_batch, args=(stroke_data,)).start()
        self.current_stroke_pixels = []

    def clear_all(self):
        self.canvas.delete("all")
        self.log_text.delete(1.0, tk.END)
        self.send_info.config(text="已清空")

    def append_log(self, msg):
        """线程安全地向日志框追加文本"""
        def _update():
            self.log_text.insert(tk.END, msg + "\n")
            self.log_text.see(tk.END) # 自动滚动到底部
        self.root.after(0, _update)

    # ========== 发送与日志逻辑 ==========
    def process_and_send_batch(self, stroke_pixels):
        is_connected = self.ser and self.ser.is_open

        try:
            self.root.after(0, lambda: self.send_info.config(text="计算中...", fg="orange"))

            points_3d = self.algo.generate_stroke_trajectory(stroke_pixels)
            ik_data = self.algo.solve_ik_batch(points_3d)

            count = len(ik_data)
            self.root.after(0, lambda: self.send_info.config(text=f"发送 {count} 点...", fg="blue"))

            if self.show_log_var.get():
                self.append_log(f"--- 新笔画: {count} 个点 ---")

            for i, row in enumerate(ik_data):
                # 1. 准备数据
                # row = (X_rail, Theta1, Theta2, Theta3)
                payload = struct.pack('<hhhh', row[0], row[1], row[2], row[3])
                header = b'\xAA\x55'
                checksum_val = sum(payload) & 0xFF
                checksum = struct.pack('B', checksum_val)
                packet = header + payload + checksum

                # 2. 如果开启日志，格式化显示
                # 为了防止日志拖慢速度，只有勾选时才生成字符串
                if self.show_log_var.get():
                    hex_str = ' '.join(f'{b:02X}' for b in packet)
                    # 显示物理含义以便调试
                    debug_msg = f"[{i:03d}] X:{row[0]:4d} A1:{row[1]:4d} A2:{row[2]:4d} A3:{row[3]:4d} | Hex: {hex_str}"
                    self.append_log(debug_msg)

                # 3. 硬件发送
                if is_connected:
                    self.ser.write(packet)
                    time.sleep(0.002) # 2ms 间隔

            self.root.after(0, lambda: self.send_info.config(text="发送完成", fg="green"))

            if not is_connected:
                self.append_log("[Warn] 串口未连接，数据仅模拟生成，未发送")

        except Exception as e:
            print(f"Error: {e}")
            self.append_log(f"[Error] {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = HandwritingApp(root, serial_port='COM3', baud_rate=115200)
    root.mainloop()