import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
def generate_rpe_plots(csv_files, labels):
    """
    csv_files: 文件路径列表
    labels: 算法名称列表
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    colors = ['r', 'g', 'b']
    
    for file, label, color in zip(csv_files, labels, colors):
        data = pd.read_csv(file, names=['time', 'trans_err', 'rot_err'])
        
        # 1. 获取数据并转换为 numpy 数组以避免兼容性报错
        #t = (data['time'] - data['time'].iloc[0]).values
        t = np.arange(len(data))
        trans_err = data['trans_err'].values
        rot_err = data['rot_err'].values
        
        # 2. 子图 1: 平移误差
        ax1.plot(t, trans_err, label=label, color=color, alpha=0.8)
        
        # 3. 子图 2: 旋转误差
        ax2.plot(t, rot_err, label=label, color=color, alpha=0.8)

    # 格式化平移图
    ax1.set_title('Deliverable: Relative Translation Error')
    ax1.set_ylabel('L2 Norm Error (Normalized)')
    ax1.legend()
    ax1.grid(True, linestyle='--')

    # 格式化旋转图
    ax2.set_title('Deliverable: Relative Rotation Error')
    ax2.set_xlabel('Data index (Limited by computer performance trade-offs)')
    ax2.set_ylabel('Chordal Distance (Frobenius Norm)')
    ax2.legend()
    ax2.grid(True, linestyle='--')

    plt.tight_layout()
    plt.savefig('rpe_comparison_plot.png', dpi=300)
    plt.show()

# 运行绘图
generate_rpe_plots(['8pt.csv', '5pt.csv', 'essential.csv'], 
                   ['8-point RANSAC', '5-point RANSAC', 'Nister Essential RANSAC'])
