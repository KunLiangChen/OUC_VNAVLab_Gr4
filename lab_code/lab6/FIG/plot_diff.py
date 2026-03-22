import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_ransac_comparison_linear(with_ransac_file, no_ransac_file):
    try:
        # 加载数据并转换为 numpy 以避免之前的兼容性报错
        data_ransac = pd.read_csv(with_ransac_file, names=['time', 'trans_err', 'rot_err']).dropna()
        data_no_ransac = pd.read_csv(no_ransac_file, names=['time', 'trans_err', 'rot_err']).dropna()
    except Exception as e:
        print(f"Error: {e}")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    # 1. 平移误差对比 (线性坐标)
    ax1.plot(np.arange(len(data_no_ransac)), data_no_ransac['trans_err'].values, 
             label='5-point WITHOUT RANSAC', color='red', alpha=0.3, linestyle='-')
    ax1.plot(np.arange(len(data_ransac)), data_ransac['trans_err'].values, 
             label='5-point WITH RANSAC', color='green', linewidth=1.5, alpha=0.9)
    
    ax1.set_title('Impact of RANSAC: Relative Translation Error (Linear Scale)', fontsize=14)
    ax1.set_ylabel('L2 Norm Error', fontsize=12)
    ax1.set_ylim(0, 2.2) # 平移误差最大为 2（方向完全相反），固定刻度更利于对比
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.6)

    # 2. 旋转误差对比 (线性坐标)
    ax2.plot(np.arange(len(data_no_ransac)), data_no_ransac['rot_err'].values, 
             label='5-point WITHOUT RANSAC', color='red', alpha=0.3, linestyle='-')
    ax2.plot(np.arange(len(data_ransac)), data_ransac['rot_err'].values, 
             label='5-point WITH RANSAC', color='green', linewidth=1.5, alpha=0.9)
    
    ax2.set_title('Impact of RANSAC: Relative Rotation Error (Linear Scale)', fontsize=14)
    ax2.set_xlabel('Frame Index', fontsize=12)
    ax2.set_ylabel('Chordal Distance', fontsize=12)
    # 旋转误差如果没有 RANSAC 可能会非常高，如果图被撑得太严重，可以取消下面这行的注释
    # ax2.set_ylim(0, 1.0) 
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.savefig('ransac_impact_linear.png', dpi=300)
    print("Linear plot saved.")
    plt.show()

# 运行
plot_ransac_comparison_linear('5pt.csv', '5pt_noRAN.csv')
