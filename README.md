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

<p align="center">
  <b>English</b> | <a href="./README_zh.md">简体中文</a>
</p>

---

# 📖 Overview
VNAV (Visual Navigation for Autonomous Vehicles) is a core graduate course at MIT's Department of Aeronautics and Astronautics, instructed by Professor Luca Carlone. The laboratory exercises focus on implementing robust perception and localization algorithms on resource-constrained platforms, such as drones and racing cars.

This project is part of the **Integrated Group Project Course I at Ocean University of China (OUC)**. We have completed the contents of Lab 1-6 and Lab 9, covering everything from Linux system setup and ROS configuration to UAV controllers, trajectory optimization, and CV-based keypoint matching algorithms.

# 🛠️ Tech Stack
- **OS**: Ubuntu 20.04 LTS
- **Core Languages**: C++11/14, Python 3.10
- **Framework**: ROS (Robot Operating System)
- **Math & Vision Libraries**:  
    - **Eigen**: Linear algebra operations.
    - **mav_trajectory_generation**: Trajectory optimization.
    - **OpenCV**: Image processing and visual geometry.
    - **OpenGV**: Geometric vision for pose estimation.

# 📃 Project Structure
| Folder | Category | Description |
| :--- | :--- | :--- |
| **`lab_code/`** | **Source Code** | Contains the implementation of all lab requirements. |
| **`lab_report/`** | **Theoretical Analysis** | Detailed lab reports including algorithm derivations, parameter settings, error analysis (e.g., ATE/RPE), and reflections on VNAV core theories. |
| **`lab_video/`** | **Demo Videos** | Visual demonstrations of algorithm performance, including Rviz trajectory visualization and feature point tracking. |

---

# 🛠️ Maintenance & Troubleshooting

Our team utilized the **NVIDIA Jetson Orin NX** development board. During the project, we encountered and resolved several issues such as boot failures, firmware corruption, and system crashes. We have documented our recovery process and solutions for reference:

**[Troubleshooting Guide & Solutions](https://foremost-garage-64a.notion.site/Jetson-Orin-NX-32d3024d444380bea5e8e50f9d7ee834?pvs=143)**

---

# ⚖️ License & Academic Integrity
This project is for educational and reference purposes only. Please adhere to MIT’s academic integrity guidelines and do not copy the code directly for course submissions.
