import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_3pt_raw(file_path):
    try:
        # 1. 读取数据
        # 假设 CSV 格式为: 时间戳, 平移误差, 旋转误差
        data = pd.read_csv(file_path, names=['time', 'trans_err', 'rot_err']).dropna()
        
        # 转换为 Numpy 数组以避免绘图时的版本兼容问题
        frames = np.arange(len(data))
        t_err = data['trans_err'].values
        r_err = data['rot_err'].values

    except Exception as e:
        print(f"读取文件失败: {e}")
        return

    # 2. 创建画布
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8))

    # --- 上图：平移误差 (Translation Error) ---
    ax1.plot(frames, t_err, color='blue', linewidth=1, label='Raw 3pt Translation Error')
    ax1.set_title('Arun 3-point Algorithm: Raw Relative Translation Error', fontsize=14)
    ax1.set_ylabel('Error (L2 Norm)')
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='upper right')

    # --- 下图：旋转误差 (Rotation Error) ---
    ax2.plot(frames, r_err, color='red', linewidth=1, label='Raw 3pt Rotation Error')
    ax2.set_title('Arun 3-point Algorithm: Raw Relative Rotation Error', fontsize=14)
    ax2.set_xlabel('Frame Index (Sequence)')
    ax2.set_ylabel('Error (Chordal Distance)')
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.legend(loc='upper right')

    # 3. 自动布局并保存
    plt.tight_layout()
    plt.savefig('3pt_raw_data_plot_option.png', dpi=300)
    print("原始数据图表已保存为: 3pt_raw_data_plot.png")
    plt.show()

# 执行绘图
plot_3pt_raw('3pt_option.csv')
