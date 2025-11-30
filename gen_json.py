import tkinter as tk
from tkinter import messagebox
import json
import numpy as np
import os

class HandwritingRecorder:
    def __init__(self, root):
        self.root = root
        self.root.title("汉字轨迹录制器 (一键保存为 write.json)")
        
        # ========== 参数设置 ==========
        self.canvas_width = 600
        self.canvas_height = 600
        self.target_scale = 1024  # 目标坐标系大小
        
        # 数据存储
        self.strokes_data = [] 
        self.current_stroke = [] 
        
        # ========== UI 布局 ==========
        self.canvas = tk.Canvas(root, width=self.canvas_width, height=self.canvas_height, 
                                bg='white', cursor="cross")
        self.canvas.pack(pady=10)
        
        btn_frame = tk.Frame(root)
        btn_frame.pack(fill=tk.X, pady=5)
        
        tk.Button(btn_frame, text="撤销上一笔", command=self.undo, width=15).pack(side=tk.LEFT, padx=20)
        tk.Button(btn_frame, text="清空画布", command=self.clear, width=15).pack(side=tk.LEFT, padx=20)
        
        # 修改按钮文字，表明是一键保存
        tk.Button(btn_frame, text="一键保存 (write.json)", command=self.save_json, bg="#ddffdd", width=25).pack(side=tk.RIGHT, padx=20)
        
        # ========== 事件绑定 ==========
        self.canvas.bind("<Button-1>", self.start_stroke) 
        self.canvas.bind("<B1-Motion>", self.record_point) 
        self.canvas.bind("<ButtonRelease-1>", self.end_stroke) 

    def start_stroke(self, event):
        self.current_stroke = []
        self.last_x, self.last_y = event.x, event.y
        self.current_stroke.append([event.x, event.y])

    def record_point(self, event):
        x, y = event.x, event.y
        if len(self.current_stroke) > 0:
            last_pt = self.current_stroke[-1]
            dist = np.sqrt((x - last_pt[0])**2 + (y - last_pt[1])**2)
            if dist < 3: return

        self.current_stroke.append([x, y])
        self.canvas.create_line(self.last_x, self.last_y, x, y, 
                                width=3, fill='black', capstyle=tk.ROUND, smooth=True)
        self.last_x, self.last_y = x, y

    def end_stroke(self, event):
        if len(self.current_stroke) > 1:
            self.strokes_data.append(self.current_stroke)
        self.current_stroke = []

    def undo(self):
        if not self.strokes_data: return
        self.strokes_data.pop()
        self.redraw_canvas()

    def clear(self):
        self.strokes_data = []
        self.canvas.delete("all")

    def redraw_canvas(self):
        self.canvas.delete("all")
        for stroke in self.strokes_data:
            if len(stroke) < 2: continue
            for i in range(len(stroke) - 1):
                p1 = stroke[i]
                p2 = stroke[i+1]
                self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], 
                                        width=3, fill='black', capstyle=tk.ROUND)

    def generate_svg_path(self, median_points):
        if not median_points: return ""
        path_str = f"M {median_points[0][0]} {median_points[0][1]}"
        for pt in median_points[1:]:
            path_str += f" L {pt[0]} {pt[1]}"
        return path_str

    def save_json(self):
        """直接保存为 write.json，无弹窗"""
        if not self.strokes_data:
            messagebox.showwarning("提示", "画布为空，无法保存")
            return

        # ========== 核心修改：直接指定文件名和路径 ==========
        filename = "write.json"
        output_dir = "track_data"  # 还是用 track_data 比较规范
        
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        file_path = os.path.join(output_dir, filename)

        # ========== 数据转换 ==========
        final_medians = []
        final_strokes_outline = []
        
        scale_ratio = self.target_scale / self.canvas_width
        
        for stroke in self.strokes_data:
            scaled_stroke = []
            for pt in stroke:
                raw_sx = int(pt[0] * scale_ratio)
                raw_sy = int(pt[1] * scale_ratio)
                
                # === 修正逻辑 (X正常，Y翻转) ===
                sx = raw_sx                    
                sy = self.target_scale - raw_sy 
                
                scaled_stroke.append([sx, sy])
            
            final_medians.append(scaled_stroke)
            final_strokes_outline.append(self.generate_svg_path(scaled_stroke))

        json_data = {
            "strokes": final_strokes_outline,
            "medians": final_medians
        }

        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(json_data, f, ensure_ascii=False)
            
            # 只有这一个提示框了，告诉你保存成功
            print(f"✅ 已保存: {file_path}")
            messagebox.showinfo("完成", "✅ 已保存 write.json")
            
        except Exception as e:
            messagebox.showerror("错误", f"保存失败: {str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = HandwritingRecorder(root)
    root.mainloop()