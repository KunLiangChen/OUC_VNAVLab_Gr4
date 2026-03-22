<div align="center">
  <h1>MIT 16.485: VNAV</h1>
  <p><b>Visual Navigation for Autonomous Vehicles | Fall 2025 Implementation</b></p>
</div>

<div align="center">
  <a href="https://vnav.mit.edu/"><img src="https://img.shields.io/badge/MIT-16.485%20VNAV-A31F34?style=for-the-badge&logo=massachusettsinstituteoftechnology&logoColor=white" alt="MIT VNAV"></a>
  <img src="https://img.shields.io/badge/ROS-1%20Noetic-orange?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 1 Noetic">
  <img src="https://img.shields.io/badge/OUC%20x%20MIT-green?style=for-the-badge" alt="School">
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="License">
</div>

---

# 📖 项目概述
VNAV 是 MIT 航空航天系的一门研究生核心课程，由 Luca Carlone 教授指导。实验内容专注于在资源受限的无人机和赛车平台上实现鲁棒的感知与定位算法。 本项目是中国海洋大学集成小组项目课程I的内容，我们完成了lab1-lab6 + lab9的所有内容。项目包括从linux系统设置，ROS下载和配置到无人机控制器，轨迹优化，CV关键点匹配算法等内容。

# 🛠️ 技术栈
- **系统**: Ubuntu 20.04 LTS
- **核心语言**: C++11/14, Python 3.10
- **机器人框架**: ROS
- **数学与视觉库**:  
    - Eigen: 线性代数运算
    - mav_trajectory_generation: 轨迹优化
    - OpenCV: 图像处理与视觉几何
    - OpenGV: 求解相机的位姿估计

# 📃 项目结构
| 文件夹 | 内容分类 | 关键描述 |
| :--- | :--- | :--- |
| **`lab_code/`** | **核心代码** | 包含所有实验要求实现的源代码。 |
| **`lab_report/`** | **理论分析** | 详细的实验报告。记录了算法推导、实验参数设置、误差分析（如 ATE/RPE 轨迹误差）以及对 VNAV 核心理论的思考。 |
| **`lab_video/`** | **演示视频** | 算法运行的直观展示。包含 Rviz 轨迹可视化、特征点追踪效等 |

---

# ⚖️ 许可说明
本项目代码仅供学习参考。请遵守 MIT 的学术诚信准则，不要直接复制用于课程作业提交。