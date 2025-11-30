import json
import os
import math
import numpy as np
import matplotlib.pyplot as plt

# =========================================================
# 0. 基础配置
# =========================================================
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'SimSun'] 
plt.rcParams['axes.unicode_minus'] = False 

# =========================================================
# 1. 逆运动学算法 (保持不变，用于生成 angles 头文件)
# =========================================================
def inverse_3link_robot(X, Y, Z, L1, L2, L3):
    """
    简易 3轴机械臂逆解
    注意：如果 Z 值是机械臂原生脉冲值而非物理坐标，此函数可能需要根据实际硬件调整
    这里仅作示例保留。
    """
    x_move = X
    Ye = Y - L3
    Ze = Z
    
    D = math.sqrt(Ye**2 + Ze**2)
    # 防止数值误差导致 acos 越界
    val = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_a2 = max(min(val, 1.0), -1.0)
    a2_rad = math.acos(cos_a2)
    
    phi = math.atan2(Ze, Ye)
    psi = math.atan2(L2 * math.sin(a2_rad), L1 - L2 * math.cos(a2_rad))
    a1_rad = phi + psi
    
    a1 = 180 - math.degrees(a1_rad)
    a2 = 180 - math.degrees(a2_rad)
    a3 = 180 - (a1 + a2) - 20
    
    return x_move, a1, a2, a3

# =========================================================
# 2. 生成 3D 轨迹 (核心修改部分)
# =========================================================
def generate_buchuwangxin_3D():
    # ========== 参数设置 (同步 MATLAB 代码) ==========
    char_list = ['不', '忘', '初', '心']
    json_map = {
        '不': 'bu.json', '忘': 'wang.json',
        '初': 'chu.json', '心': 'xin.json'
    }
    
    # 机械臂原生 Z 轴定义
    z_write_mech = 300.0  # 落笔
    z_lift_mech  = 320.0  # 抬笔
    
    # 物理尺寸与偏移
    physical_width_mm = 45.0
    physical_height_mm = 80.0
    spacing = 5.0
    y_offset = 80.0
    x_offset = 0.0
    
    target_point_count = 400
    data_dir = 'hanzi_data'  # 请确保 json 文件在此目录下，或者改为 'json'

    # ========== 初始化 ==========
    all_critical_points = []   # 关键点
    all_write_segments = []    # 书写段
    current_x = 0.0
    
    print("🚀 开始生成 3D 轨迹 (Python 版 - 同步 MATLAB 逻辑)...")

    # Check directory
    if not os.path.isdir(data_dir):
        # Fallback for common folder names if 'hanzi_data' doesn't exist
        if os.path.isdir('json'): data_dir = 'json'
        elif os.path.isdir('data'): data_dir = 'data'

    # ========== 第一阶段：生成轨迹结构 ==========
    for k, ch in enumerate(char_list):
        filename = json_map.get(ch)
        json_path = os.path.join(data_dir, filename)
        
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"找不到文件: {json_path} (对应汉字: {ch})")
        
        print(f"正在处理第 {k+1} 个字: {ch}")
        
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            
        if 'medians' not in data:
            raise ValueError(f"文件 {filename} 不包含 'medians' 字段")
            
        strokes = data['medians']
        num_strokes = len(strokes) # MATLAB 中的 numStrokes
        
        # ---------- 重采样笔画 ----------
        resampled_strokes = []
        for stroke in strokes:
            stroke_np = np.array(stroke)
            if stroke_np.shape[0] < 2:
                resampled_strokes.append(stroke_np)
                continue
            
            # 计算笔画长度
            diff_points = np.diff(stroke_np, axis=0)
            segment_lengths = np.sqrt(np.sum(diff_points**2, axis=1))
            total_length = np.sum(segment_lengths)
            
            # 插值
            num_new_points = max(2, int(round(total_length / 3.0)))
            new_t = np.linspace(0, 1, num_new_points)
            
            # 累积距离作为插值依据
            cumulative_dist = np.insert(np.cumsum(segment_lengths), 0, 0)
            if total_length > 0:
                t = cumulative_dist / total_length
            else:
                t = np.linspace(0, 1, len(stroke_np))
            
            new_x = np.interp(new_t, t, stroke_np[:, 0])
            new_y = np.interp(new_t, t, stroke_np[:, 1])
            resampled_strokes.append(np.column_stack((new_x, new_y)))

        # ---------- 修正的物理映射 (同步 MATLAB) ----------
        all_points_pixel = np.vstack(resampled_strokes) if resampled_strokes else np.empty((0, 2))
        
        if all_points_pixel.shape[0] == 0:
            current_x += physical_width_mm + spacing
            continue
            
        # 获取原始像素范围
        min_x_pix = np.min(all_points_pixel[:, 0])
        max_x_pix = np.max(all_points_pixel[:, 0])
        min_y_pix = np.min(all_points_pixel[:, 1])
        max_y_pix = np.max(all_points_pixel[:, 1])
        
        # 计算缩放因子
        scale_x = physical_width_mm / ((max_x_pix - min_x_pix) + 1e-9)
        scale_y = physical_height_mm / ((max_y_pix - min_y_pix) + 1e-9)
        
        scaled_strokes = []
        for stroke in resampled_strokes:
            s_copy = stroke.copy()
            
            # X轴：标准缩放 + 水平定位
            s_copy[:, 0] = (s_copy[:, 0] - min_x_pix) * scale_x + current_x
            
            # Y轴：三步处理 (归一化 -> 翻转 -> 赋值)
            normalized_y = (s_copy[:, 1] - min_y_pix) * scale_y
            flipped_y = physical_height_mm - normalized_y  # 垂直翻转
            s_copy[:, 1] = flipped_y
            
            scaled_strokes.append(s_copy)

        # ========== 构建轨迹段 (同步 MATLAB) ==========
        for i, stroke in enumerate(scaled_strokes):
            if len(stroke) == 0: continue
            
            if k == 0 and i == 0:
                # 第一笔：直接落笔
                z_col = np.full((stroke.shape[0], 1), z_write_mech)
                seg = np.hstack((stroke, z_col))
                all_write_segments.append(seg)
            else:
                # 获取上一位置
                if not all_write_segments and not all_critical_points:
                    last_xy = np.array([0.0, 0.0])
                else:
                    if len(all_write_segments) > 0 and len(all_write_segments[-1]) > 0:
                        last_xy = all_write_segments[-1][-1, :2]
                    elif len(all_critical_points) > 0:
                        last_xy = all_critical_points[-1][:2]
                    else:
                        last_xy = np.array([0.0, 0.0])

                start_pos = stroke[0, :] # 新笔画起点
                
                # === 1. 抬笔点 ===
                crit_point = np.array([last_xy[0], last_xy[1], z_lift_mech])
                all_critical_points.append(crit_point)
                
                # === 2. 空中移动段 ===
                move_vec = start_pos - last_xy
                move_dist = np.linalg.norm(move_vec)
                
                full_segment_parts = []
                
                if move_dist > 1e-3:
                    num_move_points = max(2, int(round(move_dist / 0.4)))
                    t_move = np.linspace(0, 1, num_move_points)
                    mx = last_xy[0] + move_vec[0] * t_move
                    my = last_xy[1] + move_vec[1] * t_move
                    mz = np.full_like(t_move, z_lift_mech)
                    full_segment_parts.append(np.column_stack((mx, my, mz)))
                
                # === 3. 垂直下落段 ===
                # 从 [start, z_lift] -> [start, z_write]
                fall_segment = np.array([
                    [start_pos[0], start_pos[1], z_lift_mech],
                    [start_pos[0], start_pos[1], z_write_mech]
                ])
                full_segment_parts.append(fall_segment)
                
                # === 4. 书写段 ===
                if stroke.shape[0] > 1:
                    w_xy = stroke[1:, :]
                    w_z = np.full((w_xy.shape[0], 1), z_write_mech)
                    full_segment_parts.append(np.hstack((w_xy, w_z)))
                
                # 合并
                if full_segment_parts:
                    all_write_segments.append(np.vstack(full_segment_parts))

        current_x += physical_width_mm + spacing

    # ========== 第二阶段：智能分配点数 (算法保持不变) ==========
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
        
        # 调整点数误差
        if adjustment > 0:
            sort_idx = np.argsort(segment_lengths)[::-1]
            for j in range(min(adjustment, len(sort_idx))):
                allocated_points[sort_idx[j]] += 1
        elif adjustment < 0:
            for j in range(abs(adjustment)):
                # 寻找大于1的点减少
                valid_indices = np.where(allocated_points > 1)[0]
                if len(valid_indices) > 0:
                    allocated_points[valid_indices[0]] -= 1
        
        # 最后兜底检查
        if np.sum(allocated_points) != available_write_points:
             allocated_points[-1] += (available_write_points - np.sum(allocated_points))

    # ========== 第三阶段：构建 finalPoints ==========
    final_points_list = []
    
    # 第一笔
    if len(all_write_segments) > 0:
        first_seg = all_write_segments[0]
        n_alloc = allocated_points[0]
        if first_seg.shape[0] <= n_alloc or n_alloc <= 1:
            sampled = first_seg
        else:
            indices = np.round(np.linspace(0, first_seg.shape[0] - 1, n_alloc)).astype(int)
            sampled = first_seg[indices, :]
        final_points_list.append(sampled)
        
    # 后续笔画
    write_idx = 1
    for i in range(len(all_critical_points)):
        # 插入抬笔关键点
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
            
    final_points = np.vstack(final_points_list)
    
    # ========== 应用最终偏移 ==========
    final_points[:, 0] += x_offset
    final_points[:, 1] += y_offset
    
    # ========== 可视化 ==========
    try:
        fig = plt.figure("机械臂原生坐标系轨迹")
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(f'机械臂轨迹 (Z落={z_write_mech}, Z抬={z_lift_mech})')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (Native/mm)')
        
        colors = plt.cm.tab10(np.arange(4)) 
        x_start = x_offset
        
        for k in range(4):
            x_min = x_start if k == 0 else x_start - spacing / 2
            x_max = x_start + physical_width_mm + spacing / 2
            # 这里的筛选仅用于按字上色，实际可能会有一点重叠
            mask = (final_points[:, 0] >= x_min) & (final_points[:, 0] <= x_max + spacing)
            
            if np.any(mask):
                color_tuple = tuple(colors[k])
                ax.scatter(final_points[mask, 0], final_points[mask, 1], final_points[mask, 2],
                           s=10, c=[color_tuple], label=char_list[k])
            x_start += physical_width_mm + spacing
            
        ax.legend()
        
        # 强制等比例显示 (Python中最好的验证方式)
        x_limits = [np.min(final_points[:, 0]), np.max(final_points[:, 0])]
        y_limits = [np.min(final_points[:, 1]), np.max(final_points[:, 1])]
        z_limits = [min(z_write_mech, z_lift_mech) - 10, max(z_write_mech, z_lift_mech) + 10]

        max_range = max(x_limits[1]-x_limits[0], y_limits[1]-y_limits[0], z_limits[1]-z_limits[0])
        x_mid, y_mid, z_mid = np.mean(x_limits), np.mean(y_limits), np.mean(z_limits)

        ax.set_xlim(x_mid - max_range/2, x_mid + max_range/2)
        ax.set_ylim(y_mid - max_range/2, y_mid + max_range/2)
        ax.set_zlim(z_mid - max_range/2, z_mid + max_range/2)
        
        ax.view_init(elev=30, azim=-60)

    except Exception as e:
        print(f"可视化出错: {e}")

    # ========== 输出头文件 (trajectory_all.h) ==========
    header_filename = 'trajectory_all.h'
    with open(header_filename, 'w', encoding='utf-8') as fid:
        fid.write('/* 机械臂原生坐标系轨迹 - 由 Python 脚本生成 */\n')
        fid.write('#ifndef TRAJECTORY_ALL_H\n#define TRAJECTORY_ALL_H\n\n')
        fid.write('// 机械臂Z轴原生值\n')
        fid.write(f'const float Z_WRITE = {z_write_mech:.1f};  // 落笔\n')
        fid.write(f'const float Z_LIFT  = {z_lift_mech:.1f};  // 抬笔\n\n')
        fid.write(f'const int NUM_POINTS = {final_points.shape[0]};\n')
        fid.write('const float TRAJ[][3] = {\n')
        for i in range(final_points.shape[0]):
            fid.write(f'  {{{final_points[i,0]:.2f}, {final_points[i,1]:.2f}, {final_points[i,2]:.2f}}},\n')
        fid.write('};\n\n#endif\n')
        
    print(f"\n✅ 成功生成 {header_filename}！共 {final_points.shape[0]} 个点")
    print(f"💡 验证: Z_WRITE={z_write_mech} (落笔), Z_LIFT={z_lift_mech} (抬笔)")
    
    return final_points

# =========================================================
# 3. 主程序
# =========================================================
def generate_buchuwangxin_angles():
    # 机械臂参数 (如果需要生成角度，请根据实际 Z 值含义调整 Z_bias)
    L1 = 185.0
    L2 = 210.0
    L3 = 60.0
    
    # 这里的 bias 视情况而定：
    # 如果 Z=300 代表机械臂底座上方 300mm，则 Z_bias 可能为 0 或负值用于调整
    X_bias = -35.89
    Y_bias = 70.0 
    Z_bias = 0.0  

    # 生成轨迹
    final_points = generate_buchuwangxin_3D()
    
    # 注意：如果 Z=300 是舵机值，下面的逆解计算将无物理意义
    # 如果 Z=300 是物理毫米高度，则逆解有效
    print('🚀 正在计算逆运动学角度...')
    
    n_points = final_points.shape[0]
    result_float = np.zeros((n_points, 4))
    
    for i in range(n_points):
        X = final_points[i, 0] + X_bias
        Y = final_points[i, 1] + Y_bias
        Z = final_points[i, 2] + Z_bias
        
        xm, a1, a2, a3 = inverse_3link_robot(X, Y, Z, L1, L2, L3)
        result_float[i, :] = [xm, a1, a2, a3]

    # 转为整数格式 (x100, x10, x10, x10)
    result_int = np.zeros((n_points, 4), dtype=np.int16)
    result_int[:, 0] = np.round(result_float[:, 0] * 100).astype(np.int16)
    result_int[:, 1] = np.round(result_float[:, 1] * 10).astype(np.int16)
    result_int[:, 2] = np.round(result_float[:, 2] * 10).astype(np.int16)
    result_int[:, 3] = np.round(result_float[:, 3] * 10).astype(np.int16)

    # 输出角度文件
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
    generate_buchuwangxin_angles()