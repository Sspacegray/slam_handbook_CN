# SLAM Handbook 中文译稿

原文件: `main.pdf`

说明:
- 本文件为当前会话完成的中文译稿。
- 专业名词尽量保留英文原词或缩写，例如 `SLAM`、`Structure from Motion`、`Factor Graph`、`LiDAR`、`Bundle Adjustment (BA)`、`IMU`。
- 当前内容包含封面信息、前言、目录以及第 1 到第 12 章正文。

## 封面信息

**SLAM Handbook（SLAM 手册）**

**从定位与建图到空间智能**
From Localization and Mapping to Spatial Intelligence

编订时间: 2025 年 11 月 25 日

主编:
- Luca Carlone
- Ayoung Kim
- Timothy Barfoot
- Daniel Cremers
- Frank Dellaert

发布说明:
- © Cambridge University Press。未经 Cambridge University Press 书面许可，不得复制本书任何部分。

## 前言

**Simultaneous Localization and Mapping**，也就是更广为人知的 **SLAM**，指的是这样一个基础问题: 在构建环境空间模型的同时，实时确定机器人在该环境中的位置。这个术语最早由 Hugh Durrant-Whyte 和 John Leonard 于 1995 年提出，标志着一个处于机器人学、几何学、控制理论以及概率推断交叉点上的问题被正式化。

SLAM 既优雅，又极具挑战性。它的核心在于处理高维、不确定且动态系统中的推理问题。这个过程要求精确的空间推断与稳健的概率建模，从而构建出一致的世界地图，而这些地图往往必须在噪声和歧义并存的条件下实时生成。

SLAM 尤其引人注目的地方，在于它的普适性。在计算机视觉中，它对应于 **Structure from Motion** 问题；在机器人学中，它支撑着从室内自主导航到行星探索，再到自动驾驶汽车等各种应用。自诞生以来，SLAM 激发了数以万计的研究论文，深度汲取了物理学、统计学、计算机视觉、几何学、控制理论以及机器学习等多个学科的成果。它的发展推动了自主系统能力的持续跃升，使其能够在复杂、开放世界环境中大规模运行。

本书汇集了该领域一批最重要的专家以及新生代研究者的贡献。各章代表了当今 SLAM 的最新进展，既体现了理论创新的深度，也展现了实际应用的广度。从早期以 **Kalman filter** 和 **Bayesian estimation** 为基础的形式化方法出发，SLAM 已发展为一套丰富的数学框架体系，涵盖 **graph-based optimization**、**factor graph**、**nonlinear least squares** 以及基于深度学习的技术。除了介绍 SLAM 的数学基础外，本书还通过讨论真实世界中的应用案例，为实践者提供了有价值的指导，内容覆盖基于视觉的 SLAM、基于 **LiDAR** 的 SLAM，以及足式运动等场景。

本书也涵盖了 **Spatial AI** 的最新发展，展示了深度学习、可微渲染（**differentiable rendering**）以及大规模视觉与语言模型（large vision and language models）的进步，如何推动能够为机器人提供丰富空间与语义理解的新型表征。

SLAM 的现实影响已经十分明确。无论是嵌入在扫地机器人中的系统，还是驱动自动驾驶车队的核心能力，SLAM 都已经成为使智能系统能够在物理世界中感知并行动的基石技术。它代表了迄今为止机器人学中最复杂、也最持久的研究方向之一，并持续塑造着机器如何理解空间、不确定性与运动。

SLAM 研究社区是机器人学中最具活力、技术要求也最严格的社区之一。今天许多领先的机器人学家，都是从解决 SLAM 问题开始其职业生涯的，随后又将其中的原则扩展到了状态估计（state estimation）、控制（control）和传感器融合（sensor fusion）等领域。SLAM 是概率机器人学（probabilistic robotics）的标志性问题之一，而正是通过 SLAM，统计推理才得以进入机器人感知与自主性的核心。

作为这一卓越社区的参与者与受益者，我们很自豪能推出这本 handbook。它全面呈现了 SLAM 当前所达到的前沿，既建立在三十年的基础性工作之上，也指向未来的挑战与机遇。这里汇聚的作者代表着下一代的引领者，他们的工作定义了严谨科学与现实复杂性相遇时所能达到的高度。

我们赞赏他们的贡献，并相信这本书将在未来多年中，成为研究人员、工程师和学生的重要参考资料与灵感来源。

Wolfram Burgard, Dieter Fox, Sebastian Thrun

## 目录

### 前言
- Preface: 前言
- Notation: 记号说明

### 第一部分 SLAM 基础
- I Prelude: 导论
- I.1 What is SLAM?: 什么是 SLAM？
- I.2 Anatomy of a Modern SLAM System: 现代 SLAM 系统的构成
- I.3 The Role of SLAM in the Autonomy Architecture: SLAM 在自主系统架构中的作用
- I.3.1 Do We Really Need SLAM for Robotics?: 机器人学真的需要 SLAM 吗？
- I.4 Past, Present, and Future of SLAM, and Scope of this Handbook: SLAM 的过去、现在与未来，以及本书范围
- I.4.1 Short History and Scope of this Handbook: 简史与本书覆盖范围
- I.4.2 From SLAM to Spatial AI: 从 SLAM 到 Spatial AI
- I.5 Handbook Structure: 本书结构

### 第 1 章 面向 SLAM 的 Factor Graphs（因子图）
- 1 Factor Graphs for SLAM: 面向 SLAM 的 Factor Graphs（因子图）
- 1.1 Visualizing SLAM With Factor Graphs: 用 Factor Graph 可视化 SLAM
- 1.1.1 A Toy Example: 一个玩具示例
- 1.1.2 A Factor-Graph View: 因子图视角
- 1.1.3 Factor Graphs as a Language: 将 Factor Graph 作为一种语言
- 1.2 From MAP Inference to Least Squares: 从 MAP Inference 到 Least Squares
- 1.2.1 Factor Graphs for MAP Inference: 用于 MAP Inference 的 Factor Graphs
- 1.2.2 Specifying Probability Densities: 概率密度的定义
- 1.2.3 Nonlinear Least Squares: 非线性最小二乘
- 1.3 Solving Linear Least Squares: 求解线性最小二乘
- 1.3.1 Linearization: 线性化
- 1.3.2 SLAM as Linear Least Squares: 将 SLAM 表述为线性最小二乘
- 1.3.3 Matrix Factorization for Least Squares: 用于最小二乘的矩阵分解
- 1.4 Nonlinear Optimization: 非线性优化
- 1.4.1 Steepest Descent: 最速下降
- 1.4.2 Gauss-Newton: Gauss-Newton 方法
- 1.4.3 Levenberg-Marquardt: Levenberg-Marquardt 方法
- 1.4.4 Dogleg Minimization: Dogleg 最小化
- 1.5 Factor Graphs and Sparsity: Factor Graph 与稀疏性
- 1.5.1 The Sparse Jacobian and its Factor Graph: 稀疏 Jacobian 及其 Factor Graph
- 1.5.2 The Sparse Information Matrix and its Graph: 稀疏信息矩阵及其图结构
- 1.5.3 Sparse Factorization: 稀疏分解
- 1.6 Elimination: 消元
- 1.6.1 Variable Elimination Algorithm: 变量消元算法
- 1.6.2 Linear-Gaussian Elimination: 线性高斯消元
- 1.6.3 Sparse Cholesky Factor as a Bayes Net: 作为 Bayes Net 的稀疏 Cholesky 因子
- 1.7 Incremental SLAM: 增量式 SLAM
- 1.7.1 The Bayes Tree: Bayes Tree
- 1.7.2 Updating the Bayes Tree: 更新 Bayes Tree
- 1.7.3 Incremental Smoothing and Mapping: 增量平滑与建图
- 1.8 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第 2 章 高级状态变量表示
- 2 Advanced State Variable Representations: 高级状态变量表示
- 2.1 Optimization on Manifolds: 流形上的优化
- 2.1.1 Rotations and Poses: 旋转与位姿
- 2.1.2 Matrix Lie Groups: 矩阵 Lie 群
- 2.1.3 Lie Group Optimization: Lie 群优化
- 2.1.4 Uncertainty and Lie Groups: 不确定性与 Lie 群
- 2.1.5 Lie Group Extras: Lie 群补充内容
- 2.2 Continuous-Time Trajectories: 连续时间轨迹
- 2.2.1 Splines: 样条
- 2.2.2 From Parametric to Nonparametric: 从参数化到非参数化
- 2.2.3 Gaussian Processes: Gaussian Processes（高斯过程）
- 2.2.4 Spline and GPs on Lie Groups: Lie 群上的样条与 GPs
- 2.3 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第 3 章 面向错误数据关联与离群点的鲁棒性
- 3 Robustness to Incorrect Data Association and Outliers: 面向错误数据关联与离群点的鲁棒性
- 3.1 What Causes Outliers and Why Are They a Problem?: 离群点为何产生，以及它们为何构成问题？
- 3.1.1 Data Association and Outliers: 数据关联与离群点
- 3.1.2 Least-Squares in the Presence of Outliers: 存在离群点时的最小二乘
- 3.2 Detecting and Rejecting Outliers in the SLAM Front-end: 在 SLAM Front-end 中检测并剔除离群点
- 3.2.1 RANdom SAmple Consensus (RANSAC): RANSAC
- 3.2.2 Graph-theoretic Outlier Rejection and Pairwise Consistency Maximization: 基于图论的离群点剔除与成对一致性最大化
- 3.3 Increasing Robustness to Outliers in the SLAM Back-end: 提升 SLAM Back-end 对离群点的鲁棒性
- 3.3.1 Iteratively Reweighted Least Squares: 迭代重加权最小二乘
- 3.3.2 Black-Rangarajan Duality: Black-Rangarajan 对偶
- 3.3.3 Alternating Minimization: 交替最小化
- 3.3.4 Graduated Non-Convexity: 渐进非凸化
- 3.4 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第 4 章 可微优化
- 4 Differentiable Optimization: 可微优化
- 4.1 Recap on Nonlinear Least Squares: 非线性最小二乘回顾
- 4.2 Differentiation Through Nonlinear Least Squares: 针对非线性最小二乘的微分
- 4.2.1 Unrolled Differentiation: 展开式微分
- 4.2.2 Truncated Unrolled Differentiation: 截断展开式微分
- 4.2.3 Implicit Differentiation: 隐式微分
- 4.3 Differentiation on Manifold: 流形上的微分
- 4.3.1 Lie Group Derivatives: Lie 群导数
- 4.3.2 Differentiation Operations on Manifold: 流形上的微分运算
- 4.4 Numerical Challenges of Automatic Differentiation and Modern Libraries: 自动微分与现代库的数值挑战
- 4.4.1 Example of Implementation of Differentiable Optimization: 可微优化实现示例
- 4.4.2 Related Open-source Libraries: 相关开源库
- 4.5 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第 5 章 稠密地图表示
- 5 Dense Map Representations: 稠密地图表示
- 5.1 Range Sensing Preliminaries: 距离传感预备知识
- 5.1.1 Sensor Measurement Model: 传感器测量模型
- 5.1.2 Conversion to Point Cloud: 转换为点云
- 5.2 Foundations of Dense Mapping: 稠密建图基础
- 5.2.1 Occupancy Maps: Occupancy Maps（占据栅格地图）
- 5.2.2 Implicit Surface and Distance Fields: 隐式曲面与距离场
- 5.2.3 Occupancy Maps or Distance Fields?: Occupancy Maps 还是 Distance Fields？
- 5.3 Map Representations: 地图表示
- 5.3.1 Explicitness of Target Spatial Structures: 目标空间结构的显式程度
- 5.3.2 Types of Spatial Abstractions: 空间抽象的类型
- 5.3.3 Data Structures and Storage: 数据结构与存储
- 5.4 Constructing Maps: Methods and Practices: 地图构建的方法与实践
- 5.4.1 Points: 点
- 5.4.2 Surfels: Surfels
- 5.4.3 Meshes: 网格
- 5.4.4 Voxels: 体素
- 5.4.5 Gaussian Processes: Gaussian Processes（高斯过程）
- 5.4.6 Hilbert Maps: Hilbert Maps
- 5.4.7 Deep Learning in Mapping: 建图中的深度学习
- 5.5 Usage Considerations: 使用层面的考虑
- 5.5.1 Environmental Aspects: 环境因素
- 5.5.2 Downstream Task Types: 下游任务类型
- 5.5.3 Summary of Mapping Methods: 建图方法总结

### 第 6 章 SLAM 的可认证最优求解器与理论性质
- 6 Certifiably Optimal Solvers and Theoretical Properties of SLAM: SLAM 的可认证最优求解器与理论性质
- 6.1 Certifiably Optimal Solvers for SLAM: 面向 SLAM 的可认证最优求解器
- 6.1.1 Shor’s Relaxation: Shor 松弛
- 6.1.2 SE-Sync: Certifiably Correct Pose-Graph Optimization: SE-Sync: 可认证正确的 Pose-Graph Optimization
- 6.1.3 Landmark-based SLAM: 基于地标的 SLAM
- 6.1.4 Extensions: Range Measurements, Anisotropic Noise, and Outliers: 扩展内容: 距离测量、各向异性噪声与离群点
- 6.2 How Accurate is the Optimal Solution of a SLAM Problem?: SLAM 问题最优解的精度如何？
- 6.2.1 Cramer-Rao Lower Bound and the Fisher Information Matrix: Cramer-Rao 下界与 Fisher 信息矩阵
- 6.2.2 Fisher Information Matrix and Graph Laplacian: Fisher 信息矩阵与 Graph Laplacian
- 6.3 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第二部分 SLAM 实践
- II Prelude: 导论
- II.1 Key Modules in the SLAM Front-End: SLAM Front-End 的关键模块
- II.1.1 Odometry: 里程计
- II.1.2 Loop Closure: 回环检测
- II.1.3 Priors and Unary Factors: 先验与 Unary Factors
- II.2 Sensors and Factor Graphs: 传感器与 Factor Graphs
- II.2.1 Selecting the Right Sensor for Your Application: 为应用选择合适的传感器
- II.2.2 Sensor Fusion: 传感器融合
- II.2.3 Calibration and Synchronization of Sensors: 传感器标定与同步
- II.3 Evaluation: 评估
- II.4 How to Read Part II?: 如何阅读第二部分？

### 第 7 章 Visual SLAM（视觉 SLAM）
- 7 Visual SLAM: 视觉 SLAM
- 7.1 Historical Background and Terminology: 历史背景与术语
- 7.1.1 From Photogrammetry to Bundle Adjustment and Visual SLAM: 从摄影测量到 Bundle Adjustment，再到 Visual SLAM
- 7.1.2 Terminology: 术语
- 7.2 The Processing Pipeline of a Visual SLAM System: 视觉 SLAM 系统的处理流水线
- 7.2.1 Visual Odometry Front-End: Visual Odometry Front-End
- 7.2.2 Mapping Back-End: Mapping Back-End
- 7.2.3 Visual Place Recognition and Relocalization: 视觉地点识别与重定位
- 7.2.4 Compute and Data Flow: 计算与数据流
- 7.3 Visual SLAM Fundamentals: 视觉 SLAM 基础
- 7.3.1 Camera Model: 相机模型
- 7.3.2 Keypoints: 关键点
- 7.3.3 Reprojection Error: 重投影误差
- 7.3.4 Keypoint-Based Visual SLAM: 基于关键点的视觉 SLAM
- 7.3.5 Photometric Error and Direct Methods: 光度误差与直接法
- 7.3.6 Visual Place Recognition and Global Localization: 视觉地点识别与全局定位
- 7.3.7 Initialization: 初始化
- 7.3.8 Map Representations: 地图表示
- 7.4 Further Considerations about Image Alignment and BA: 图像对齐与 BA 的进一步讨论
- 7.4.1 Keypoint-based Image Alignment: 基于关键点的图像对齐
- 7.4.2 Direct Image Alignment: 直接图像对齐
- 7.4.3 Solving BA: 求解 BA
- 7.4.4 Bundle Adjustment Revisited: 再谈 Bundle Adjustment
- 7.5 Examples of Full Visual SLAM Systems: 完整视觉 SLAM 系统示例
- 7.6 Real-time Dense Reconstruction: 实时稠密重建
- 7.7 SLAM with Depth-sensing Cameras: 基于深度相机的 SLAM
- 7.8 Combining Vision with Other Modalities: 视觉与其他模态的融合
- 7.8.1 Inertial Measurement Units (IMU): 惯性测量单元（IMU）
- 7.8.2 GPS and WiFi for Global Localization: 用于全局定位的 GPS 与 WiFi
- 7.9 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第 8 章 LiDAR SLAM
- 8 LiDAR SLAM: LiDAR SLAM
- 8.1 LiDAR Sensing Preliminaries and Categorization: LiDAR 感知基础与分类
- 8.2 LiDAR Odometry: LiDAR 里程计
- 8.2.1 Foundations of Scan Registration: 扫描配准基础
- 8.2.2 Common Components for LiDAR Odometry: LiDAR 里程计的常见组成
- 8.2.3 Summary of LiDAR Odometry: LiDAR 里程计总结
- 8.3 LiDAR Place Recognition: LiDAR 地点识别
- 8.3.1 Problem Definition: 问题定义
- 8.3.2 Methods for LiDAR Place Recognition: LiDAR 地点识别方法
- 8.3.3 Summary of LiDAR Place Recognition: LiDAR 地点识别总结
- 8.4 LiDAR SLAM: LiDAR SLAM
- 8.4.1 Structure of a LiDAR SLAM System: LiDAR SLAM 系统结构
- 8.4.2 Pose-graph Optimization and Map Update: Pose-graph Optimization 与地图更新
- 8.4.3 Multi-robot and Multi-session LiDAR SLAM: 多机器人与多会话 LiDAR SLAM
- 8.5 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第 9 章 Radar SLAM
- 9 Radar SLAM: Radar SLAM
- 9.1 Introduction to Radar: Radar 简介
- 9.1.1 Sensor Types: 传感器类型
- 9.1.2 Radar Sensing Principles: Radar 感知原理
- 9.1.3 Challenges to Radar Applications: Radar 应用面临的挑战
- 9.1.4 Radar Filtering: Radar 滤波
- 9.2 Radar Odometry: Radar 里程计
- 9.2.1 Doppler Odometry: Doppler 里程计
- 9.2.2 Direct Odometry: 直接里程计
- 9.2.3 Feature-based Odometry: 基于特征的里程计
- 9.2.4 Registration-based Odometry: 基于配准的里程计
- 9.2.5 Motion Compensation: 运动补偿
- 9.3 Radar Place Recognition: Radar 地点识别
- 9.3.1 Unique Challenges in Radar Place Recognition: Radar 地点识别中的独特挑战
- 9.3.2 Learning-based Radar Place Recognition: 基于学习的 Radar 地点识别
- 9.3.3 Descriptor-based Radar Place Recognition: 基于描述子的 Radar 地点识别
- 9.4 Radar SLAM: Radar SLAM
- 9.4.1 Map Representations: 地图表示
- 9.4.2 Radar SLAM Pipelines: Radar SLAM 流水线
- 9.4.3 Multi-modality in Radar SLAM: Radar SLAM 中的多模态
- 9.5 Radar Datasets: Radar 数据集
- 9.6 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第 10 章 Event-based SLAM（事件相机 SLAM）
- 10 Event-based SLAM: 事件相机 SLAM
- 10.1 Sensor Description: 传感器说明
- 10.1.1 Working principle: 工作原理
- 10.1.2 Advantages of Event Cameras: 事件相机的优势
- 10.1.3 Current Devices and Trends: 当前设备与趋势
- 10.2 Challenges and Applications: 挑战与应用
- 10.3 Overview and Taxonomy of Event-based SLAM Methods: 事件相机 SLAM 方法概览与分类
- 10.4 Front-end of an Event-based SLAM System: 事件相机 SLAM 系统的 Front-end
- 10.4.1 Pre-processing and Event Representations: 预处理与事件表示
- 10.4.2 Indirect Methods: 间接法
- 10.4.3 Direct Methods: 直接法
- 10.4.4 Model-based and Learning-based Methods: 基于模型与基于学习的方法
- 10.5 Back-end of an Event-based SLAM System: 事件相机 SLAM 系统的 Back-end
- 10.6 State-of-the-Art Systems: 最先进系统
- 10.7 Datasets, Simulators, and Benchmarks: 数据集、模拟器与基准
- 10.7.1 Simulators: 模拟器
- 10.7.2 Datasets and Benchmarks: 数据集与基准
- 10.7.3 Metrics: 指标
- 10.8 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第 11 章 面向 SLAM 的惯性里程计
- 11 Inertial Odometry for SLAM: 面向 SLAM 的惯性里程计
- 11.1 Basics of Inertial Sensing and Navigation: 惯性感知与导航基础
- 11.1.1 Sensing Principles and Measurement Models: 感知原理与测量模型
- 11.1.2 Initial Alignment: 初始对准
- 11.2 IMU Preintegration and Factor Graphs: IMU 预积分与 Factor Graphs
- 11.2.1 Motion Integration: 运动积分
- 11.2.2 IMU Preintegration on Manifold: 流形上的 IMU 预积分
- 11.2.3 Advanced Preintegration Techniques: 高级预积分技术
- 11.3 Observability of Aided Inertial Navigation: 辅助惯性导航的可观测性
- 11.3.1 Linearized Measurement Models: 线性化测量模型
- 11.3.2 Observability Analysis: 可观测性分析
- 11.3.3 Degenerate Motions: 退化运动
- 11.4 Visual-Inertial Odometry and Practical Considerations: 视觉惯性里程计与实践考虑
- 11.4.1 Visual-Inertial Odometry: 视觉惯性里程计
- 11.4.2 Extrinsic Calibration: 外参标定
- 11.4.3 Temporal Synchronization: 时间同步
- 11.5 Further Readings & Recent Trends: 延伸阅读与最新趋势

### 第 12 章 面向 SLAM 的腿式里程计
- 12 Leg Odometry for SLAM: 面向 SLAM 的腿式里程计
- 12.1 Historical Background and Preliminaries: 历史背景与预备知识
- 12.1.1 Historical Background: 历史背景
- 12.1.2 Reference Frames: 参考坐标系
- 12.1.3 State Definition: 状态定义
- 12.1.4 Legged Robot Kinematics: 腿式机器人运动学
- 12.1.5 Legged Robot Dynamics: 腿式机器人动力学
- 12.1.6 Joint Sensing: 关节感知
- 12.2 Motion Estimation: 运动估计
- 12.2.1 Relative Pose Estimation: 相对位姿估计
- 12.2.2 Velocity Estimation: 速度估计
- 12.3 Contact Estimation: 接触估计
- 12.3.1 With Contact Sensors: 基于接触传感器
- 12.3.2 With Force/Torque Sensors: 基于力/力矩传感器
- 12.3.3 With inertial measurement units (IMUs): 基于惯性测量单元（IMUs）
- 12.3.4 From Joint Torque Sensing: 基于关节力矩感知
- 12.4 Using Leg Odometry for State Estimation: 将腿式里程计用于状态估计
- 12.4.1 Encoder Noise Propagation: 编码器噪声传播
- 12.4.2 Factor Graph Smoothing: Factor Graph 平滑
- 12.4.3 Integration with Exteroceptive Sensors for SLAM: 与外感知传感器集成用于 SLAM
- 12.5 Open Challenges: 开放挑战
- 12.5.1 Leg Deformation: 腿部形变
- 12.5.2 Non-rigid Contacts and Slippage: 非刚性接触与打滑
- 12.6 Further Readings & Recent Trends: 延伸阅读与最新趋势

## 术语保留约定

- `SLAM` 保留缩写，不译作纯中文简称。
- `Spatial AI` 保留英文。
- `Factor Graph`, `Bayes Tree`, `Bundle Adjustment`, `Pose-graph Optimization` 等核心术语保留英文主词。
- `LiDAR`, `Radar`, `IMU`, `RANSAC`, `GPS`, `WiFi` 保留原缩写。

## 前言

经过两年多的努力，我们很自豪能够推出这本关于 `SLAM` 的 handbook。我们的愿景，是写出一本可以直接交给高年级本科生或新入学研究生的入门读物，对当前最先进的研究现状进行概括总结。我们努力将它写成一份统一的文档，使各章节能够彼此衔接、逐步展开。考虑到有 70 位作者同时撰写，这项工作极具挑战；但我们对最终成果非常满意: 这是一本“由社区写给社区”的书。

`SLAM` 这个术语本身大约已有 30 年历史，但“边定位边建图”这一思想的来源还要更早。本书在多个地方都会回顾这条历史脉络，不过我们的重点主要放在 `SLAM` 的现在与未来。因此，本 handbook 被分为三个部分:

- Part I: 我们介绍 `SLAM` 的基础，重点关注 back-end 的估计理论工具链以及它所生成的地图表示。内容包括 `Factor Graph`、流形状态表示（manifold state representation）、离群点处理、可微优化（differentiable optimization）、稠密建图以及高级求解器。
- Part II: 这一部分讨论实践中的 `SLAM`，重点是 `SLAM` 中各种传感模态的特性及其集成方式，包括 `RGB camera`、`LiDAR`、事件相机（event camera）、`Radar`、`IMU` 和机器人运动学。我们解释这些传感器如何为 odometry、loop closure 和其他 factors 提供信息，以及它们如何在 `Factor Graph` 框架下完成标定、同步、融合与评估。
- Part III: 我们面向未来，讨论 `SLAM` 与 `Spatial AI` 中的新兴趋势，包括深度学习集成、新型地图表示，以及在动态或可变形环境中的运行。我们也讨论语义推理（semantic reasoning）、语言 grounding，以及用于分布式空间感知的未来计算架构。

我们感谢所有作者作出的宝贵贡献。他们都是各自领域的专家，能与他们合作既是一种荣誉，也是一种愉快的经历。我们也特别感谢《Probabilistic Robotics》一书的作者 Wolfram Burgard、Dieter Fox 和 Sebastian Thrun 为本 handbook 撰写前言；毫无疑问，我们是站在巨人的肩膀上。我们感谢《Handbook of Robotics》的联合编辑 Oussama Khatib，就本书写作提供了成熟而宝贵的建议。我们尤其感谢 Frank Park 在两年前率先提出编写一本 `SLAM handbook` 的设想，并从一开始就持续推动此事。我们也非常感谢 Dongjae Lee，他在完成这样一部大型手稿的具体事务上提供了巨大帮助；同时感谢 Vendi Pavic，在准备最终材料以及协调投稿流程方面给予了极其重要的支持。我们还感谢 `IEEE RAS Technical Committee on Computer & Robot Vision` 对本项目的宣传与推广。最后，我们感谢 Navid Mahmoudian Bidgoli、Andrew Kramer、Chris Offner 和 Adam Pooley 对本 handbook 早期版本提出评论，也感谢 Cambridge University Press 对这一富有雄心且我们希望具有深远影响的项目提供支持。

将这本 handbook 从构想到最终完成，是一段非常美妙的旅程。我们希望你阅读这些页面时，能像我们组织并汇集它们时一样，感受到同样的收获与乐趣。

Luca Carlone, Ayoung Kim, Timothy Barfoot, Daniel Cremers, Frank Dellaert

## 记号说明

### 一般记号

- `a`: 实数标量使用该字体表示。
- `a`（列向量字体）: 实数列向量使用该字体表示。
- `A`: 实矩阵使用该字体表示。
- `A`（集合字体）: 集合使用该字体表示。
- `I`: 单位矩阵。
- `0`: 零矩阵。
- `A^T`: 矩阵 `A` 的转置。
- `R^(M×N)`: `M × N` 实矩阵所构成的向量空间。
- `p(a)`: `a` 的概率密度。
- `p(a|b)`: 在给定 `b` 条件下 `a` 的概率密度。
- `p(a; b)`: 由 `b` 参数化的 `a` 的概率密度。
- `N(µ, Σ)`: 均值为 `µ`、协方差为 `Σ` 的高斯概率密度。
- `GP(µ(t), K(t,t'))`: 均值函数为 `µ(t)`、协方差函数为 `K(t,t')` 的 Gaussian process。
- `^`: 带帽记号表示后验（估计）量。
- `ˇ`: 带抑扬符号表示先验量。
- `(·)_k`: 某个量在时刻 `k` 的取值。
- `(·)_{k1:k2}`: 某个量从时刻 `k1` 到 `k2`（含端点）的取值集合。
- `||·||_1`: `L1 norm`。
- `||·||_2`: `L2 norm`。

### 三维几何记号

- `F_a`: 三维空间中的一个参考坐标系（reference frame）。
- `^a v`: 向量在坐标系 `F_a` 下的坐标。
- `R_a^b`: 一个 `3×3` 旋转矩阵，属于 `SO(3)`，用于把在 `F_a` 中表达的点重新表达为纯旋转后的 `F_b` 坐标。
- `^b t_a`: 坐标系 `F_a` 原点在 `F_b` 中的三维位置。
- `~^a v`: 在 `F_a` 中表达的 `4×1` 齐次点（homogeneous point）。
- `T_a^b`: 一个 `4×4` 变换矩阵，属于 `SE(3)`，用于把在 `F_a` 中表达的齐次点变换到经过旋转和平移后的 `F_b` 中。
- `SO(3)`: special orthogonal group，用于表示三维旋转的矩阵 `Lie group`。
- `so(3)`: 与 `SO(3)` 相关联的 `Lie algebra`。
- `SE(3)`: special Euclidean group，用于表示三维位姿（pose）的矩阵 `Lie group`。
- `se(3)`: 与 `SE(3)` 相关联的 `Lie algebra`。
- `(·)^`: 一个算子，把 `R^3`（或 `R^6`）中的向量映射到三维旋转（或三维位姿）的 `Lie algebra` 元素；对三维量而言它实现叉积，例如对 `u, v ∈ R^3`，有 `u^ v = u × v`。
- `(·)∨`: 一个算子，把三维旋转（或三维位姿）的 `Lie algebra` 元素映射回 `R^3`（或 `R^6`）中的向量。

## 第一部分导论

本章介绍 `Simultaneous Localization and Mapping (SLAM)` 问题，说明一个典型 `SLAM` 系统由哪些模块组成，并解释 `SLAM` 在自主系统架构中的作用。本章也简要回顾该主题的历史，并讨论传统意义上的 `SLAM` 如何演化，以更充分地利用新的技术趋势和机会。本章的最终目标，是引入基础术语与核心动机，并说明本 handbook 的范围与结构。

## I.1 什么是 SLAM？

机器人若要在未知环境中安全而有效地运行，一个必要前提是形成对周围环境的内部表示（internal representation）。这种表示可以用来支持避障、底层控制、规划，以及更一般地，支持机器人完成既定任务所需的决策过程。对于简单任务，例如沿车道前进，或者与前方物体保持一定距离，机器人可能只需要在传感器数据流中跟踪感兴趣的实体；而对于复杂任务，例如大规模导航或移动操作（mobile manipulation），则需要建立并维护环境的持久性表示，也就是地图（map）。

这样的地图描述了障碍物、物体以及其他感兴趣实体的存在情况，并给出它们相对于机器人位姿（position and orientation）的相对位置。例如，地图可以用来指示机器人抵达某个目标位置、抓取某个特定物体，或者支持对最初未知环境的探索。

对于一个在初始未知环境中运行的机器人而言，一边构建环境地图、一边同时估计自身相对于该地图的位姿，这个问题就称为 `simultaneous localization and mapping (SLAM)`。如果地图已知，那么 `SLAM` 就退化为 `localization` 问题，此时机器人只需估计自己相对于地图的位姿。反过来，如果机器人的位姿已知，例如存在绝对定位系统，那么 `SLAM` 就退化为 `mapping` 问题，机器人只需依据传感器数据对周围环境建模。

`SLAM` 在机器人研究中之所以居于核心位置，是因为在实际应用里，机器人位姿通常并不是已知量。差分 `GPS` 和 motion capture 系统价格昂贵，而且只适用于小范围区域，因此不适合大规模机器人部署。消费级 `GPS` 更加普及，但它的精度通常只有米级误差，同时可用性也受限于室外且必须能看到卫星的场景，因此往往不足以单独承担定位任务。实际中，消费级 `GPS` 即便可用，也通常只是 `SLAM` 的附加信息源，而不是取代 `SLAM` 中定位部分的完整方案。

类似地，在许多机器人应用中，机器人通常也无法直接获得一份先验地图，因此它需要执行 `SLAM`，而不只是做 `localization`。事实上，在某些应用里，构建地图本身就是机器人部署的目标。例如在灾害响应和搜救场景中，机器人可能被部署去构建灾区地图，以帮助一线救援人员。在另一些场景中，已有地图可能已经过时，或者细节不足。比如家用机器人可能拿到公寓的平面图，但这种平面图通常不会描述环境中实际存在的家具和物体，也无法反映这些元素会随着每天的使用而重新摆放。类似地，火星探测车虽然能够获得低分辨率的火星表面卫星地图，但仍然需要进行局部建图，以支持避障和运动规划。

正因 `SLAM` 问题如此重要，它在研究社区和实践界都受到了广泛关注，应用范围从机器人学扩展到虚拟现实（VR）和增强现实（AR）等多个领域。与此同时，`SLAM` 仍然是一个充满活力的研究方向，存在许多尚未解决的问题，也蕴含着新的机会。

## I.2 现代 SLAM 系统的构成

`SLAM` 的最终目标，是从传感器数据中推断出地图表示和机器人位姿，也就是机器人的轨迹（trajectory）。这些传感器数据包括本体感知传感器（proprioceptive sensors），例如轮式 odometry 或惯性测量单元 `IMU`，以及外感知传感器（exteroceptive sensors），例如 camera、`LiDAR` 和 `Radar`。从数学上看，这可以理解为一个逆问题（inverse problem）: 给定一组测量，目标是找到一个世界模型（即地图）和一组机器人位姿（即轨迹），使它们能够产生这些观测。求解 `SLAM` 问题主要有两类策略: `indirect methods` 和 `direct methods`。

绝大多数 `SLAM` 方法都会先对原始传感器数据进行预处理，以提取更紧凑、也更便于数学描述的“中间表示”（intermediate representations）。例如在视觉 `SLAM` 中，`indirect methods` 不会直接使用图像中的每一个像素，而是提取少量具有代表性的二维点特征（或 `keypoints`），然后只建模这些关键点如何依赖于相机位姿以及场景几何。与之相对，`direct methods` 不先构建中间抽象，而是试图直接从原始传感数据中完成定位与建图。这个分类在视觉 `SLAM` 中最为常见，但并不限于视觉场景，本书在 Chapter 8 和 Chapter 9 中还会继续讨论。两类方法各有优点，也各有局限。

`Indirect methods` 通常更快，也更节省内存，因为它们只处理少量已知或待估计三维位置的关键点。因此，能够实时运行的 `indirect visual SLAM` 系统在 2000 年左右就已经出现。直到今天，在算力受限的平台上，`indirect methods` 仍然是实时机器人视觉的主流选择。此外，一旦中间表示被确定下来，后续计算往往在数学上更简单，使得推断问题更容易处理。以视觉 `SLAM` 为例，一旦在多张图像之间识别出一组对应点，那么后续的定位与建图问题就可归结为经典的 `bundle adjustment (BA)` 问题，而这一问题已经拥有大量强大的求解器和近似方法。

相对地，`direct methods` 的潜力在于，它们能够利用全部可用输入信息，因此有望获得更高精度。虽然处理全部输入信息，例如每张图像中的所有像素，在计算上会非常昂贵；而且从原始输入数据（例如每个像素的亮度）到目标量（定位与建图）之间的复杂关系，也会在估计所依赖的损失函数中引入更多非凸性，但研究者已经提出了高效的近似与推断策略，使得首批能够实时运行的 `direct visual SLAM` 方法在 2010 年代开始出现。正如本书第二部分和第三部分将展示的那样，对海量输入数据的高效处理，可以借助 `graphics processing units (GPUs)` 的并行计算能力来实现。

无论是 `direct methods` 还是 `indirect methods`，其核心都是利用观测去推断机器人位姿和地图表示。估计理论（estimation theory）中已经有一整套成熟文献，专门讨论如何从观测中推断感兴趣的未知量。在本书中，我们特别关注那些以概率推断为基础、并将估计问题改写为优化问题求解的估计理论工具；这些内容将在 Chapter 1 和 Chapter 2 中结合 `SLAM` 问题进行回顾和定制化介绍。

`Indirect methods` 还自然导致了现代 `SLAM` 系统中的一个典型模块划分: front-end 与 back-end。原始传感器数据会先进入一组算法模块，也就是 `SLAM front-end`，由它负责提取中间表示；这些中间表示随后被送入估计器，也就是 `SLAM back-end`，由它负责估计真正关心的量。`Front-end` 往往还负责提供一个初始猜测（initial guess），供 `back-end` 在迭代优化时使用，从而缓解由非凸性带来的收敛问题。下面通过几个例子说明 `front-end` 与 `back-end` 的区别。

在基于地标的 `SLAM` 模型中，`front-end` 产生的是针对三维地标（3D landmarks）的观测，而 `back-end` 则估计机器人的轨迹（作为一系列 poses）以及地标的位置。在基于 `pose graph` 的 `SLAM` 模型中，`front-end` 会把原始传感器数据抽象为 odometry 和 loop closure 测量，这些通常都是相对位姿测量；`back-end` 则据此估计整条机器人轨迹。

**Example I.1 (Visual SLAM: from pixels to landmarks)**  
视觉 `SLAM` 使用相机图像来估计机器人轨迹，并构建稀疏三维点云地图。典型的视觉 `SLAM front-end` 会在每张图像中提取二维 `keypoints`，并在多帧之间进行匹配，使每一组匹配结果（feature track）都对应于不同相机视角下对同一个三维点（landmark）的重复观测。`Front-end` 还会利用计算机视觉中称为 `minimal solvers` 的技术，对相机位姿和三维地标位置进行粗略估计。随后，`back-end` 会通过求解一个被称为 `bundle adjustment` 的优化问题，来估计或细化这些地标的三维位置，以及观测它们的机器人位姿。这个例子导向的是一种基于地标（landmark-based）或基于特征（feature-based）的 `SLAM` 模型，本书将在 Chapter 7 中详细讨论视觉 `SLAM`。

**Example I.2 (LiDAR SLAM: from scans to odometry and loop closures)**  
`LiDAR SLAM` 使用 `LiDAR scans` 来估计机器人轨迹和地图。一种常见的 `LiDAR SLAM front-end` 做法，是使用扫描匹配算法，例如 `Iterative Closest Point (ICP)`，来计算两帧 `LiDAR scans` 之间的相对位姿。更具体地说，`front-end` 会匹配相邻时刻采集的扫描，以估计机器人在这两时刻之间的相对运动，也就是所谓的 `odometry`；同时，它也会匹配机器人多次访问同一地点时得到的扫描，这就是所谓的 `loop closures`。这些 `odometry` 与 `loop closure` 测量随后会传递给 `back-end`，后者通过求解 `pose-graph optimization (PGO)` 问题来优化机器人轨迹。这个例子导向的是一种基于 `pose graph` 的 `SLAM` 模型，本书将在 Chapter 8 中讨论 `LiDAR SLAM`。

前面的例子展示了三类常见的“中间表示”或伪测量（pseudo-measurements），它们由 `front-end` 产生并交给 `back-end`: `landmark observations`、`odometry` 和 `loop closures`。在复杂的 `SLAM` 系统中，这些表示可以组合使用。例如，在某些视觉 `SLAM` 系统中，可以同时提取 `keypoints`、估计相对运动，并引入回环约束，以共同提升系统的鲁棒性与精度。

## I.3 SLAM 在自主系统架构中的作用

`SLAM` 的作用，是为下游任务（downstream tasks）提供支撑。例如，机器人的位姿估计可用于控制机器人跟随期望轨迹，而地图则可以结合当前机器人位姿用于运动规划（motion planning）。这里的运动规划是广义概念: `SLAM` 不仅可用于构建大尺度地图以支持导航任务，也可以支持构建局部三维地图，从而服务于操作（manipulation）与抓取（grasping）任务。

虽然人们很容易把 `SLAM` 想象成一个“黑盒式”的整体系统: 输入传感器数据，立即输出机器人位姿与地图，但在真实系统实现以及它与自主架构的集成中，情况要复杂得多。这是因为机器人必须闭合多个控制与决策回路，而这些回路对时延（latency）的要求并不相同。例如，机器人需要对自身轨迹闭合低层控制环路，这类环路通常需要相对较高的运行频率和极低的时延，才能保持稳定；一个高速飞行的 `UAV`，可能要求 `front-end` 在几毫秒内就给出 `odometry` 估计。相比之下，闭合运动规划环路通常可以容忍更大的时延，因为全局规划一般运行频率更低，因此 `back-end` 以秒级延迟给出全局轨迹和地图估计也可能是可以接受的。

正因如此，一个典型的 `SLAM` 系统实现通常包含多个并行运行的进程，并且需要确保较慢的进程，例如 `back-end` 中的全局位姿与地图优化，不会阻塞较快的进程，例如 `odometry` 估计。

我们还可以观察到，`SLAM` 系统内部各过程之间存在复杂的相互作用。比如，`front-end` 会把 `odometry` 送给 `back-end`，而 `back-end` 又会周期性地对里程计轨迹施加全局校正，并把修正后的结果交给运动控制器。类似地，`front-end` 会计算 `loop closures` 并送入 `back-end`，而 `back-end` 也可以反过来为回环检测提供信息，提示哪些回环候选更可信，哪些则不太可能成立。

尽管 `SLAM back-end` 的运行速度可能较慢，但必须强调，它依然需要保持在线（online）。理想情况下，整个 `SLAM` 系统的运行时间应当保持在合理范围内，不会随着时间无限增长，并且在数据持续采集的同时，能够在嵌入式机器人硬件上完成处理。这些实时性约束对于机器人在复杂环境中正确行动至关重要，尤其是对无人机等高速机器人而言更是如此。

从历史上看，这也是 `SLAM` 与计算机视觉中一些相关问题的重要区别之一，例如 `structure from motion (SFM)`。`SFM` 同样可以根据相机图像重建三维场景几何，但它往往依赖强大的计算资源，例如服务器集群，运行时间可能以小时计，而且通常作用于一组无序图像数据。与之相对，`SLAM` 处理的是机器人在探索环境过程中按时间因果顺序采集的数据，并且必须在严格的计算约束下，于秒级时间内完成运行。需要指出的是，视觉 `SLAM` 与 `SFM` 的边界如今已经越来越模糊，这得益于视觉领域中在线 `SFM` 的研究，以及使用 `SLAM` 方法对离线数据集进行后处理的实践。因此，很多研究者会把 visual `SLAM` 和 `SFM` 这两个术语交替使用。

## I.3.1 机器人学真的需要 SLAM 吗？

按照前面的描述，`SLAM` 似乎是一个既吸引人又极具挑战的问题: 它不仅实现复杂，还要求在资源受限的平台上具备快速运行能力。因此，一个合理的问题是: 我们能否构建不依赖 `SLAM` 的复杂自主机器人系统？原文将这个问题细化为三个子问题。

**Q1. Do we need SLAM for any robotics task?**  
我们在本节开头说过，`SLAM` 是为机器人任务服务的。那么一个自然的问题是: 所有机器人任务都需要 `SLAM` 吗？答案显然是否定的。更偏反应式（reactive）的任务，例如持续保持目标在视野中，可以通过更简单的控制策略来解决，例如 `visual servoing`。类似地，如果机器人只需要在较小范围内活动，那么依赖 `odometry` 和局部建图可能就已经足够。此外，如果机器人所处环境本身具备某种定位基础设施，那么我们也未必需要求解 `SLAM`。

不过，对于在非结构化、也就是没有基础设施支撑的环境中进行长期运行的机器人而言，`SLAM` 仍然看起来是不可或缺的组件。长期运行通常需要某种“记忆”能力，例如机器人需要回到之前见过的物体附近，或者寻找一条合适的无碰撞路径，而由 `SLAM` 构建出来的地图表示正好可以承担这种长期空间记忆（spatial memory）的角色。

**Q2. Do we need globally consistent geometric maps for navigation?**  
`SLAM` 的一个重要关注点，是对轨迹和地图表示进行优化，使其在度量意义上尽可能准确，也就是实现全局一致（globally consistent）。这正是 `SLAM back-end` 的职责之一。于是人们会问: 度量精确性真的有必要吗？

一个直观替代方案，是只依赖 `odometry` 来获得局部一致的轨迹和地图估计，这样就可以绕开 `loop closures` 和 `back-end optimization`。但很遗憾，由于漂移（drift）的存在，单纯依赖 `odometry` 并不适合支持长期运行。设想机器人先访问一栋楼中的 `Office 1`，然后在探索其他区域之后，又来到与其仅一墙之隔的 `Office 3`。如果只使用 `odometry`，机器人可能会因累计漂移而误以为 `Office 1` 和 `Office 3` 相距很远，从而无法意识到二者之间其实存在一条很短的连接路径。

另一个稍微复杂一些的替代方案，是构建拓扑地图（topological map）。拓扑地图可以理解为一张图（graph）: 节点表示机器人访问过的地点，边表示这些地点之间的可通行关系。与本 handbook 所采取的 metric `SLAM` 视角不同，拓扑地图中的节点和边并不携带距离、方位、位置等度量信息，因此也不需要进行优化。机器人只要在穿行于两个地点之间时添加一条边（对应 `odometry`），或者在 place recognition 模块判断两个地点重合时添加一条边（对应 `loop closures`），就能建立这样的图。

然而，这种做法的主要问题在于，place recognition 技术并不完美；更根本地说，不同地点本身也可能看起来十分相似，这一现象被称为 `perceptual aliasing`。因此回到前面的例子，如果 `Office 1` 和 `Office 3` 看起来极其相似，一个纯拓扑的方法就可能错误地把它们当成同一个办公室。相比之下，metric `SLAM` 方法能够利用几何信息判断这两个办公室实际上是不同房间，从而为系统提供更强大的工具，来决定 place recognition 的结果是否正确，以及两次观测是否真的对应同一个地点。这些工具将在 Chapter 3 中详细讨论。

`Q3. Do we need maps?` 这一部分在下一段继续。

**Q3. Do we need maps?**  
`SLAM` 会构建一张可被直接查询（queried）、检查（inspected）和可视化（visualized）的地图。正如 Chapter 5 将会介绍的那样，地图有许多表示方式，包括三维点云（3D point clouds）、体素（voxels）、网格（meshes）、神经辐射场（neural radiance fields）等等。另一方面，也可以采取一种完全不同的路线: 为了让机器人完成任务，我们可以训练机器人把原始传感器数据直接映射为动作，例如使用 `Reinforcement Learning`，从而绕过显式建图这一步。

在这种做法中，从传感器数据到动作的神经网络很可能也会形成某种内部表示（internal representation），但这种表示通常无法被直接查询、检查或可视化。关于“地图是否真的必需”这一问题，目前仍无定论；不过，已有一些初步证据表明，在许多视觉任务中，把地图作为中间表示，至少是有益的。

## I.4 SLAM 的过去、现在与未来，以及本书范围

空间推理（spatial reasoning）算法的设计，自机器人学和计算机视觉诞生以来，就一直处于这两个领域研究的核心位置。与此同时，`SLAM` 研究也在持续演化，并不断扩展到新的工具与问题。

## I.4.1 简史与本书覆盖范围

正如本书各章将反复展示的那样，`SLAM` 具有多重侧面。因此，它的历史也是多线并行的，其起源可以追溯到多个不同的科学共同体。

根据观测与测量来构建世界地图，是人类历史上最古老的挑战之一，它最终发展出大地测量学（geodesy）与测量学（surveying）等领域。许多先驱都对这一方向作出了贡献。Carl Friedrich Gauss 在 `1821-1825` 年间完成了对汉诺威王国的三角测量。Sir George Everest 在 `1830-1843` 年担任印度测量总长（Surveyor General of India），主持了大三角测量工程（Great Trigonometric Survey），后来世界最高峰也以他的名字命名。`1856` 年，Carl Maximilian von Bauernfeind 出版了《Elements of Surveying》标准教材，并于 `1868` 年创立慕尼黑工业大学，将大地测量学确立为一门核心科学学科。André-Louis Cholesky 则是在第一次世界大战前于克里特和北非从事测量工作期间，发展出了著名的 `Cholesky` 矩阵分解。

视觉 `SLAM` 问题也与摄影测量学（photogrammetry）以及 `Structure from Motion` 问题密切相关，而这些问题的起源同样可以追溯到 19 世纪。本书会在 Chapter 7 中进一步讨论这一脉络。

在机器人学领域，`SLAM` 的起源通常追溯到 Smith 和 Chessman、Durrant-Whyte，以及 Crowley、Chatila 和 Laumond 的开创性工作。`SLAM` 这一缩写则是在 `1995` 年的一篇综述论文中正式提出的。这些早期工作给出了两个关键洞见。第一，要想在未知环境中避免累计漂移，机器人必须同时估计自身位姿以及环境中固定外部实体的位置，例如 landmarks。第二，估计理论中的现有工具，尤其是著名的 `Extended Kalman Filter (EKF)`，可以用于在一个扩展状态上执行估计；该扩展状态同时包含机器人位姿和 landmark 位置，从而形成了一大类 `EKF-SLAM` 方法。

`EKF-SLAM` 曾经极为流行，但在实践中面临三个主要问题。第一个问题是它对离群点（outliers）和数据关联错误（data association errors）非常敏感。这些错误可能来自 place recognition 或 object detection 的失败，例如机器人误以为自己观测到的是某个已知物体或地点，实际上观测到的却是另一个外观相似但不同的对象。如果不能妥善处理这些伪测量，`EKF-SLAM` 就会产生严重错误的估计结果。

第二个问题在于，`EKF` 依赖于对机器人运动和传感器观测方程的线性化。在实际中，线性化点通常来自 `odometry`；而一旦 `odometry` 出现漂移，线性化后的系统就可能成为原始非线性系统的糟糕近似。这会导致 `EKF-SLAM` 在 `odometry` 积累较大漂移时发生发散。

第三个问题是计算复杂度。朴素实现的 `Kalman Filter` 由于需要维护和操作稠密协方差矩阵，其计算复杂度会随着状态变量数量呈平方增长。在基于 landmark 的 `SLAM` 问题中，拥有上千个 landmarks 并不少见，因此朴素方法很难实现实时运行。

作为对这些问题的回应，研究社区在 2000 年代早期开始转向基于粒子滤波（particle filter）的做法。这类方法使用一组假设或粒子（particles）来表示机器人轨迹，并建立在估计理论中的 particle filtering 框架之上。当它们与基于 landmark 的地图结合时，便可以处理大量 landmarks，从而突破 `EKF` 的平方复杂度瓶颈；同时，它们也更容易估计诸如二维占据栅格（2D occupancy grid maps）之类的稠密地图模型。此外，这些方法不依赖线性化，对离群点和错误数据关联的敏感性也更低。

不过，这类方法依旧面临计算量与精度之间的权衡。若要得到高精度轨迹和地图，通常需要成千上万个粒子；但粒子越多，计算负担也越大。特别是当粒子数量有限时，如果没有任何采样粒子足够接近真实机器人轨迹，particle filter 仍然可能发散，这个问题被称为 `particle depletion`。在三维问题中，这一问题会更加严重，因为系统需要大量粒子去覆盖潜在的三维机器人位姿空间。

在 `2005-2015` 年之间，一个关键洞见推动了另一类 `SLAM` 方法进入主流视野。这个洞见是: 尽管 `EKF` 中出现的协方差矩阵是稠密的，但它的逆矩阵，也就是所谓的 `Information Matrix`，实际上非常稀疏；而且当估计中保留过去机器人位姿时，其稀疏模式还具有相当可预测的结构。这使得人们可以设计出复杂度接近线性的滤波算法，而不是 `EKF` 那样的平方复杂度。

这一洞见最初被用于类似 `EKF` 的方法，例如 `EIF`，但它也为基于优化（optimization-based）的做法铺平了道路。事实上，基于优化的 `SLAM` 方法在早期就已经被提出，但当时常被认为速度太慢、不适合实际使用。上述稀疏结构的发现，使得研究者得以重新审视这些优化方法，并将其扩展为更具可伸缩性、且能够在线求解的系统。

这波新进展也可以理解为估计框架上的又一次转移，即转向 `maximum likelihood` 和 `maximum a posteriori (MAP)` 估计。这些框架把估计问题重写为优化问题，同时用概率图模型（probabilistic graphical model）来描述问题结构，更具体地说，就是使用 `factor graph`。由此形成的基于 `factor graph` 的 `SLAM` 方法，至今仍然是主导范式，并深刻影响了社区对 visual odometry 和 visual-inertial odometry 等相关问题的理解方式。

优化视角非常强大，它使得人们能够对 `SLAM` 进行比以往更深入的理论分析，这一点会在 Chapter 6 中进一步体现。此外，也不难证明，带有适当线性化点的 `EKF`，本质上可以被看成是一次非线性优化求解器的单步迭代，因此从表达能力上说，优化视角严格强于纯滤波视角。最后，基于优化的 perspective 也更适合 `SLAM` 的最新扩展，尤其是在本书下一节与 Part III 将讨论的场景中，系统需要同时估计连续变量（例如场景几何）和离散变量（例如场景的语义属性）。

上述简史基本停留在 `2015` 年左右，而本 handbook 的 Part III 则会讨论更新近的趋势，包括大约从 `2012` 年开始并逐渐渗透到机器人学中的“深度学习革命（deep learning revolution）”。同时也应看到，上面的历史回顾主要聚焦于我们所谓的 `SLAM back-end`，也就是估计引擎本身；至于 `SLAM front-end` 的发展，则与计算机视觉、信号处理和机器学习等多个社区的工作密切相关。

基于上述考虑，本 handbook 将主要聚焦于 `factor-graph-based` 的 `SLAM` 表述。这只是范围上的选择，并不意味着其他技术路线没有价值。例如，在本书撰写时，基于 `EKF` 的工具在视觉惯性里程计（visual-inertial odometry）中仍然广泛使用；与此同时，新的估计框架也不断出现，包括 `invariant filters`、`equivariant filters`，以及基于 `random finite sets` 的替代表述。

## I.4.2 从 SLAM 到 Spatial AI

`SLAM` 的核心，主要是估计环境以及机器人自身的几何属性。例如，`SLAM map` 会携带关于环境中障碍物、两个位置之间距离与可通行路径、以及显著 landmarks 几何坐标等信息。从这个角度看，`SLAM` 对于机器人理解并执行诸如“robot: go to position [x,y,z]”这样的指令是有用的，其中 `[x,y,z]` 表示机器人需要到达的地点或物体在地图坐标系中的几何坐标。

然而，以坐标形式来指定目标并不适合非专业人类用户，而且这显然也不是我们与人类交流任务时所采用的方式。因此，理想情况下，下一代机器人应该能够理解并执行用自然语言表达的高层指令，例如:“robot: pick up the clothes in the bathroom, and take them to the laundry room”。要解析这样的指令，机器人不仅需要理解几何信息，例如“bathroom 在哪里”，还必须理解语义信息，例如“什么是 bathroom 或 laundry room”“哪些物体属于 clothes”。

这一认识推动研究社区开始把 `SLAM` 看作一个更大范围空间感知系统（spatial perception system）中的集成组件。这个更广义的系统需要同时对场景的几何、语义，乃至可能的物理属性进行推理，从而构建一种多面向的地图表示，也就是所谓的“world model”，以支持机器人理解并执行复杂指令。

由此产生的 `Spatial AI` 算法与系统，在过去十年中取得了快速进展，并有潜力显著提升机器人自主性。直观地说，可以把 `Spatial AI` 看作包含 `SLAM` 这个子模块，负责几何推理部分，同时在其之上增加语义推理能力。这样一来，系统就能够把控制闭环从单纯的 motion planning 进一步扩展到 task planning；机器人接收的目标也不再只是几何坐标，而可以是更高层的语义目标。本书将在 Part III 中详细讨论 `Spatial AI`。

## I.5 本书结构

本 handbook 的章节被组织为三个部分。

Part I 介绍 `SLAM` 的基础，重点关注 `SLAM back-end` 中使用的估计理论工具，以及 `SLAM` 能够产生的不同地图表示。具体来说，Chapter 1 介绍 `SLAM` 的 `factor graph` 表述，以及如何通过迭代式非线性优化方法来求解它。Chapter 2 则进一步扩展这一表述，使其能够处理属于光滑流形（smooth manifolds）的变量，例如旋转和位姿。Chapter 3 讨论如何在 `SLAM back-end` 中建模并缓解离群点和错误数据关联的影响。Chapter 4 回顾如何让 `back-end optimization` 具备可微性，这也是将传统 `SLAM` 方法与较新的深度学习架构结合起来的重要一步。Chapter 5 则把焦点从 `back-end` 转向稠密地图表示（dense map representations），讨论 `SLAM` 中最重要的地图表达方式。最后，Chapter 6 讨论更高级的求解器以及 `SLAM back-end` 的理论性质。

Part II 讨论 `SLAM` 的“实践现状（state of practice）”，围绕不同传感模态下的关键方法与应用展开。这一部分会涉及 `SLAM front-end` 的设计，因为它高度依赖具体传感器，也会展示现代 `SLAM` 算法与系统已经能够做到什么。Chapter 7 回顾庞大的 visual `SLAM` 文献。Chapter 8 和 Chapter 9 分别讨论 `LiDAR SLAM` 和 `Radar SLAM`。Chapter 10 讨论基于事件相机（event-based cameras）的 `SLAM`。Chapter 11 回顾如何在 `factor-graph SLAM` 系统中建模惯性测量，并讨论其基础极限，例如可观测性（observability）。Chapter 12 讨论其他 odometry 信息源的建模，包括轮式里程计和腿式里程计。

Part III 提供对最新研究进展和未来趋势的前瞻性视角。内容涵盖从计算架构，到新问题与新表示，再到语言与 Foundation Models 在 `SLAM` 中的作用。Chapter 13 回顾通过引入深度学习模块并结合可微优化，近期在 `SLAM` 中取得的改进。Chapter 14 讨论新型地图表示的机会与挑战，包括 `neural radiance fields (NeRFs)` 和 `Gaussian Splatting`。Chapter 15 关注高度动态和可变形环境中的 `SLAM`，涉及从拥挤场景建图到手术机器人等真实应用。Chapter 16 讨论 `Spatial AI` 与 metric-semantic map representations 的最新进展。Chapter 17 考察由 Foundation Models，例如 Large Vision-Language Models 带来的新机会，以及它们如何帮助构建面向 `Spatial AI` 的新型地图表示，使系统能够理解并 grounding 以自然语言给出的“open-vocabulary”指令。最后，Chapter 18 关注未来的 `Spatial AI` 计算架构，这些架构有望利用更灵活、分布式的计算硬件，并更好地支持多种机器人平台上的空间感知。

