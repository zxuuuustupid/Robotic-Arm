import json
import os
import math
import numpy as np
import matplotlib.pyplot as plt

# =========================================================
# 0. 修复中文乱码配置 (Windows专用)
# =========================================================
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'SimSun'] 
plt.rcParams['axes.unicode_minus'] = False 

# =========================================================
# 1. 逆运动学算法 (保持不变)
# =========================================================
def inverse_3link_robot(X, Y, Z, L1, L2, L3):
    x_move = X
    Ye = Y - L3
    Ze = Z
    
    D = math.sqrt(Ye**2 + Ze**2)
    # 防止数值误差导致 acos 越界
    cos_a2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_a2 = max(min(cos_a2, 1.0), -1.0)
    a2_rad = math.acos(cos_a2)
    
    phi = math.atan2(Ze, Ye)
    psi = math.atan2(L2 * math.sin(a2_rad), L1 - L2 * math.cos(a2_rad))
    a1_rad = phi + psi
    
    a1 = 180 - math.degrees(a1_rad)
    a2 = 180 - math.degrees(a2_rad)
    a3 = 180 - (a1 + a2) + 20
    
    return x_move, a1, a2, a3

# =========================================================
# 2. 生成 3D 轨迹 (修改为只读取 json1.json)
# =========================================================
def generate_single_json_3D():
    # ========== 参数设置 ==========
    target_filename = 'write.json'  # <--- 指定读取的文件名
    
    physical_width_mm = 220.0  # 单个图案的物理宽度
    physical_height_mm = 220.0 # 单个图案的物理高度
    z_write = 150.0           # 落笔高度
    z_lift = 170.0            # 抬笔高度
    target_point_count = 400  # 总点数限制
    
    # 整体偏移量
    y_offset = 45.0
    x_offset = 0.0
    
    data_dir = 'track_data' # 确保你的json文件在这个目录下

    # ========== 初始化 ==========
    all_critical_points = []
    all_write_segments = []
    current_x = 0.0 # 单文件模式下，其实不需要累加，初始为0即可
    
    final_points = np.empty((0, 3))

    print(f"🚀 开始生成 3D 轨迹，目标文件: {target_filename} ...")

    # ========== 第一阶段：读取并处理单个文件 ==========
    json_path = os.path.join(data_dir, target_filename)
    
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"找不到文件: {json_path}，请检查路径。")
    
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
        
    strokes = data['medians'] # 假设json结构包含 'medians'
    
    # --- 1.1 重采样 (Resampling) ---
    resampled_strokes = []
    for stroke in strokes:
        stroke_np = np.array(stroke)
        if stroke_np.shape[0] < 2:
            resampled_strokes.append(stroke_np)
            continue
        
        diff_points = np.diff(stroke_np, axis=0)
        segment_lengths = np.sqrt(np.sum(diff_points**2, axis=1))
        total_length = np.sum(segment_lengths)
        
        # 每隔约3个单位像素采一个点
        num_new_points = max(2, int(round(total_length / 3.0)))
        new_t = np.linspace(0, 1, num_new_points)
        dist_cumulative = np.insert(np.cumsum(segment_lengths), 0, 0)
        t = dist_cumulative / total_length
        
        new_x = np.interp(new_t, t, stroke_np[:, 0])
        new_y = np.interp(new_t, t, stroke_np[:, 1])
        resampled_strokes.append(np.column_stack((new_x, new_y)))

    all_points_pixel = np.vstack(resampled_strokes) if resampled_strokes else np.empty((0, 2))
    
    if all_points_pixel.shape[0] == 0:
        print("警告: JSON文件中没有有效笔画数据。")
        return np.zeros((1, 3))
        
    # --- 1.2 缩放 (Scaling) ---
    min_x = np.min(all_points_pixel[:, 0])
    max_x = np.max(all_points_pixel[:, 0])
    min_y = np.min(all_points_pixel[:, 1])
    max_y = np.max(all_points_pixel[:, 1])
    
    scale_x = physical_width_mm / ((max_x - min_x) + 1e-9)
    scale_y = physical_height_mm / ((max_y - min_y) + 1e-9)
    
    scaled_strokes = []
    for stroke in resampled_strokes:
        s_copy = stroke.copy()
        s_copy[:, 0] = (s_copy[:, 0] - min_x) * scale_x + current_x
        # 注意：通常图像坐标系Y向下，机械臂坐标系Y可能需要反转，这里保持原逻辑
        s_copy[:, 1] = - (max_y - s_copy[:, 1]) * scale_y 
        scaled_strokes.append(s_copy)

    # --- 1.3 生成抬笔/落笔路径 (Lift/Write Logic) ---
    for i, stroke in enumerate(scaled_strokes):
        if len(stroke) == 0: continue
        
        # 第一笔的第一个点，直接生成“写”的数据（或者你需要先移过去再下笔，这里保持原逻辑）
        if i == 0:
            z_col = np.full((stroke.shape[0], 1), z_write)
            seg = np.hstack((stroke, z_col))
            all_write_segments.append(seg)
        else:
            # 获取上一个段的终点
            if not all_write_segments and not all_critical_points:
                last_xy = np.array([0.0, 0.0])
            else:
                if len(all_write_segments) > 0 and len(all_write_segments[-1]) > 0:
                    last_xy = all_write_segments[-1][-1, :2]
                elif len(all_critical_points) > 0:
                    last_xy = all_critical_points[-1][:2]
                else:
                    last_xy = np.array([0.0, 0.0])

            start_pos = stroke[0, :]
            
            # 1. 抬笔点 (在上一个位置抬起)
            crit_point = np.array([last_xy[0], last_xy[1], z_lift])
            all_critical_points.append(crit_point)
            
            # 2. 空中移动 (从上一个位置的空中 -> 当前笔画起点的空中)
            move_vec = start_pos - last_xy
            move_dist = np.linalg.norm(move_vec)
            
            full_segment_parts = []
            if move_dist > 1e-3:
                num_move_points = max(2, int(round(move_dist / 0.4))) # 空中移动分辨率
                t_move = np.linspace(0, 1, num_move_points)
                mx = last_xy[0] + move_vec[0] * t_move
                my = last_xy[1] + move_vec[1] * t_move
                mz = np.full_like(t_move, z_lift)
                full_segment_parts.append(np.column_stack((mx, my, mz)))
            
            # 3. 落笔动作 (当前位置空中 -> 当前位置写字高度)
            full_segment_parts.append(np.array([
                [start_pos[0], start_pos[1], z_lift],
                [start_pos[0], start_pos[1], z_write]
            ]))
            
            # 4. 写字路径
            if stroke.shape[0] > 1:
                w_xy = stroke[1:, :]
                w_z = np.full((w_xy.shape[0], 1), z_write)
                full_segment_parts.append(np.hstack((w_xy, w_z)))
            
            if full_segment_parts:
                all_write_segments.append(np.vstack(full_segment_parts))

    # ========== 第二阶段：智能分配点数 (保持不变) ==========
    num_critical = len(all_critical_points)
    num_write_segments = len(all_write_segments)
    
    segment_lengths = [seg.shape[0] for seg in all_write_segments]
    total_segment_length = sum(segment_lengths)
    available_write_points = max(0, target_point_count - num_critical)
    
    allocated_points = np.zeros(num_write_segments, dtype=int)
    if total_segment_length > 0 and available_write_points > 0:
        ratios = np.array(segment_lengths) / total_segment_length
        allocated_points = np.round(available_write_points * ratios).astype(int)
        allocated_points = np.maximum(allocated_points, 1)
        
        current_sum = np.sum(allocated_points)
        adjustment = available_write_points - current_sum
        sort_idx = np.argsort(segment_lengths)[::-1]
        
        if adjustment > 0:
            for j in range(min(adjustment, len(sort_idx))):
                allocated_points[sort_idx[j]] += 1
        elif adjustment < 0:
            for j in range(abs(adjustment)):
                valid_indices = np.where(allocated_points > 1)[0]
                if len(valid_indices) > 0:
                    allocated_points[valid_indices[0]] -= 1
        
        if np.sum(allocated_points) != available_write_points:
             allocated_points[-1] += (available_write_points - np.sum(allocated_points))

    # ========== 第三阶段：构建 finalPoints ==========
    final_points_list = []
    # 添加第一段
    if len(all_write_segments) > 0:
        first_seg = all_write_segments[0]
        n_alloc = allocated_points[0]
        if first_seg.shape[0] <= n_alloc or n_alloc <= 1:
            sampled = first_seg
        else:
            indices = np.round(np.linspace(0, first_seg.shape[0] - 1, n_alloc)).astype(int)
            sampled = first_seg[indices, :]
        final_points_list.append(sampled)
        
    write_idx = 1
    for i in range(len(all_critical_points)):
        final_points_list.append(all_critical_points[i].reshape(1, 3))
        if write_idx < num_write_segments:
            seg = all_write_segments[write_idx]
            if seg.shape[0] > 0:
                n_alloc = allocated_points[write_idx]
                if seg.shape[0] <= n_alloc or n_alloc <= 1:
                    sampled = seg
                else:
                    indices = np.round(np.linspace(0, seg.shape[0] - 1, n_alloc)).astype(int)
                    sampled = seg[indices, :]
                final_points_list.append(sampled)
            write_idx += 1
            
    if final_points_list:
        final_points = np.vstack(final_points_list)
    else:
        final_points = np.zeros((0, 3))

    # 应用全局偏移
    final_points[:, 0] += x_offset
    final_points[:, 1] += y_offset
    
    # ========== 可视化 (简化版) ==========
    try:
        fig = plt.figure("轨迹可视化")
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(f'文件: {target_filename} 三维轨迹')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        
        # 直接绘制所有点
        if final_points.shape[0] > 0:
            ax.scatter(final_points[:, 0], final_points[:, 1], final_points[:, 2],
                       s=10, c='b', label='Trajectory')
            ax.plot(final_points[:, 0], final_points[:, 1], final_points[:, 2],
                    c='gray', alpha=0.5, linewidth=0.5)
        
        ax.legend()
        
        # -------------------------------------------------------------------
        # [核心代码] 强制 X, Y, Z 轴刻度比例一致
        # -------------------------------------------------------------------
        if final_points.shape[0] > 0:
            x_limits = [np.min(final_points[:, 0]), np.max(final_points[:, 0])]
            y_limits = [np.min(final_points[:, 1]), np.max(final_points[:, 1])]
            z_limits = [np.min(final_points[:, 2]), np.max(final_points[:, 2])]

            x_range = x_limits[1] - x_limits[0]
            y_range = y_limits[1] - y_limits[0]
            z_range = z_limits[1] - z_limits[0]

            max_range = max([x_range, y_range, z_range])

            x_mid = np.mean(x_limits)
            y_mid = np.mean(y_limits)
            z_mid = np.mean(z_limits)

            ax.set_xlim(x_mid - max_range/2, x_mid + max_range/2)
            ax.set_ylim(y_mid - max_range/2, y_mid + max_range/2)
            ax.set_zlim(z_mid - max_range/2, z_mid + max_range/2)
        
        ax.view_init(elev=20, azim=-60)

    except Exception as e:
        print(f"可视化出错: {e}")
        import traceback
        traceback.print_exc()

    # ========== 输出头文件 (trajectory_all.h) ==========
    with open('trajectory_all.h', 'w', encoding='utf-8') as fid:
        fid.write('/* 自动生成的轨迹 (单文件模式) */\n')
        fid.write('#ifndef TRAJECTORY_ALL_H\n#define TRAJECTORY_ALL_H\n\n')
        fid.write(f'const float Z_WRITE = {z_write:.1f};  // 落笔\n')
        fid.write(f'const float Z_LIFT  = {z_lift:.1f};  // 抬笔\n\n')
        fid.write(f'const int NUM_POINTS = {final_points.shape[0]};\n')
        fid.write('const float TRAJ[][3] = {\n')
        for i in range(final_points.shape[0]):
            fid.write(f'  {{{final_points[i,0]:.2f}, {final_points[i,1]:.2f}, {final_points[i,2]:.2f}}},\n')
        fid.write('};\n\n#endif\n')
        
    print(f"\n✅ 已生成 trajectory_all.h！共 {final_points.shape[0]} 个点")
    return final_points

# =========================================================
# 3. 主控制逻辑
# =========================================================
def generate_robot_angles():
    # 机械臂参数
    L1 = 185.0
    L2 = 210.0
    L3 = 60.0
    
    # 机械臂基座相对于绘图原点的偏移
    X_bias = -35.89
    Y_bias = 100.0
    Z_bias = 0.0

    print('🚀 生成轨迹...')
    # 调用新的单文件生成函数
    final_points = generate_single_json_3D()
    
    # 加上偏移量，转换到机械臂坐标系
    final_points[:, 0] += X_bias
    final_points[:, 1] += Y_bias
    final_points[:, 2] += Z_bias

    n_points = final_points.shape[0]
    result_float = np.zeros((n_points, 4))
    
    # 计算逆运动学
    for i in range(n_points):
        X = final_points[i, 0]
        Y = final_points[i, 1]
        Z = final_points[i, 2]
        xm, a1, a2, a3 = inverse_3link_robot(X, Y, Z, L1, L2, L3)
        result_float[i, :] = [xm, a1, a2, a3]

    # 转为整数格式 (根据下位机协议)
    result_int = np.zeros((n_points, 4), dtype=np.int16)
    result_int[:, 0] = np.round(result_float[:, 0] * 100).astype(np.int16)
    result_int[:, 1] = np.round(result_float[:, 1] * 10).astype(np.int16)
    result_int[:, 2] = np.round(result_float[:, 2] * 10).astype(np.int16)
    result_int[:, 3] = np.round(result_float[:, 3] * 10).astype(np.int16)

    # 保存 C++ PROGMEM 头文件
    with open('angles_and_xmove.h', 'w', encoding='utf-8') as fid:
        fid.write('#ifndef ANGLES_AND_XMOVE_H\n#define ANGLES_AND_XMOVE_H\n\n')
        fid.write('#include <avr/pgmspace.h>\n\n')
        fid.write(f'#define TRAJ_NUM_POINTS {n_points}\n\n')
        fid.write('// Data format: [X*100 (0.01mm), a1*10 (0.1deg), a2*10, a3*10]\n')
        fid.write('const int16_t JOINT_TRAJ[][4] PROGMEM = {\n')
        for i in range(n_points):
            fid.write(f'  {{{result_int[i,0]}, {result_int[i,1]}, {result_int[i,2]}, {result_int[i,3]}}},\n')
        fid.write('};\n\n#endif\n')
        
    print('\n✅ 已生成 angles_and_xmove.h（整数 + PROGMEM）')
    plt.show()

if __name__ == "__main__":
    generate_robot_angles()