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

## 第 1 章 面向 SLAM 的 Factor Graphs（因子图）

### 本章概览

本章介绍 `factor graphs`，并说明它们如何与 `maximum a posteriori (MAP)` 推断以及最小二乘（least squares）建立联系，前提是假设先验为 Gaussian、测量噪声也服从 Gaussian。我们关注的是 `SLAM back-end`，也就是已经由 `front-end` 提取出测量并完成数据关联之后的阶段。

本章首先用 `factor graph` 来可视化 `SLAM` 问题（Section `1.1`），然后说明 `MAP inference` 如何导出最小二乘优化（Section `1.2`）。接着讨论线性问题（Section `1.3`）和非线性问题（Section `1.4`）的求解方法，并进一步阐明稀疏性（sparsity）、`factor graphs` 和 `Bayes nets` 之间的关系（Section `1.5`）。随后，本章介绍变量消元（variable elimination）算法及其图模型解释（Section `1.6`），最后据此发展出 `Bayes tree` 和 `incremental smoothing and mapping (iSAM)` 算法（Section `1.7`）。本章最后以延伸阅读和最新趋势收尾（Section `1.8`）。

### 历史说明

`SLAM` 中的 smoothing 思路，并不只估计机器人当前时刻的位置，而是估计截至当前时刻的整条机器人轨迹。有不少作者只考虑对机器人轨迹本身进行 smoothing，这类问题如今通常被称为 `pose-based SLAM`。这尤其适用于诸如激光测距仪（laser-range finders）之类的传感器，因为它们往往产生相邻机器人 poses 之间的成对约束（pairwise constraints）。

更一般地，人们也可以考虑完整的 `full SLAM problem`，也就是同时最优估计整组传感器 poses 以及环境中所有特征参数的问题。从计算角度看，这种基于优化的 smoothing 路线被认为是有益的，原因包括:

- 与基于滤波的协方差矩阵或信息矩阵不同，后者都会随着时间演化而趋于完全稠密（fully dense），而 smoothing 所关联的信息矩阵则保持稀疏。
- 在典型建图场景中，也就是机器人不是在一个小环境里反复来回走动时，这个矩阵对地图协方差结构的表达要紧凑得多。

这直接促成了 `2000-2005` 年间大量工作，把上述思想应用到 `SLAM` 语境中。

`Square-root smoothing and mapping (SAM)`，也就是常说的 “factor-graph approach”，是在相关工作中被提出的。其核心依据是: 信息矩阵或测量 `Jacobian` 可以分别通过稀疏 `Cholesky` 分解或 `QR factorization` 高效因式分解。这样便得到一个平方根信息矩阵（square-root information matrix），它可以用来直接求得最优的机器人轨迹和地图。对信息矩阵做这种分解，在序贯估计（sequential estimation）文献中被称为 `square-root information filtering (SRIF)`，其历史甚至可以追溯到 `1969` 年 JPL 的 Mariner 10 金星任务。使用平方根形式通常会带来更高的数值精度和稳定性。

本章下面会详细说明，为什么 `factor graphs` 是表达 `SLAM` 问题中固有稀疏性的自然方式，为什么把矩阵分解为“矩阵平方根”正是这类问题求解的核心，以及这些内容如何与更一般的变量消元（variable elimination）算法联系起来。

## 1.1 用 Factor Graph 可视化 SLAM

本节将 `factor graphs` 作为一种直观工具，用来可视化 `SLAM` 问题的稀疏结构。我们先从一个 toy example 开始，介绍它的 `factor graph` 表达，然后展示 `SLAM` 的多种不同形态都可以被表示为这样的图结构，甚至在更大规模的问题中，其稀疏性也能被直接看出来。

### 1.1.1 一个玩具示例

我们先看一个简单的 `SLAM` 场景，用来说明 `factor graph` 是如何构造的。Figure `1.1` 展示了一个 toy example: 机器人依次经过三个 poses，分别是 `p1`、`p2` 和 `p3`，并对两个 landmarks `ℓ1` 和 `ℓ2` 做 bearing observations。为了把整个解固定在全局空间中，书中还假设对第一个 pose `p1` 存在一个绝对位置/姿态测量。如果没有这一项，由于 bearing measurements 都是相对的，系统将缺乏关于绝对位置的信息。

由于测量存在不确定性，我们不可能恢复世界的真实状态，但我们可以得到一个关于“从这些观测中能推断出什么”的概率描述。在 Bayesian 概率框架中，我们使用概率密度函数（probability density functions, `PDFs`）`p(x)` 来对未知变量 `x` 赋予主观信念程度。`PDF` 是非负函数，并满足总概率公理:

`∫ p(x) dx = 1`

在 Figure `1.1` 的简单例子里，状态变量 `x` 可以写成把所有未知量堆叠起来的形式:

`x = [p1, p2, p3, ℓ1, ℓ2]^T`

这里原书还专门加了一条脚注提醒: 本章为了便于讲解，会先把 rotations 与 poses 暂时当作普通向量量来处理；但严格地说，这样的处理并不完全正确。下一章会为这些量建立更合适的数学基础，而本章暂时把相关 subtleties 延后。

在 `SLAM` 中，我们希望在给定一组已观测测量 `z` 的前提下，刻画自己对未知量 `x` 的知识。在这里，`x` 包括机器人 poses 和未知 landmark positions。用 Bayesian 概率的语言来说，这就是条件密度，也就是后验（posterior）:

`p(x|z)`

得到这样的描述，被称为 `probabilistic inference`。而要完成这件事，前提是先定义一个概率模型，描述感兴趣变量如何产生带不确定性的测量。这也正是 `probabilistic graphical models` 发挥作用的地方。

`Probabilistic graphical models` 提供了一种紧凑表达复杂概率密度的方法，它们通过利用问题中的结构来完成压缩。`Bayesian networks` 可能是最知名的一类图模型: 它由若干变量节点组成，每个节点都附带一个先验或条件概率密度。Figure `1.2` 展示了与 Figure `1.1` toy example 相对应的 `Bayesian network`。在这张图里，初始测量 `z1` 依赖于第一个 pose `p1`，而 `z2 ... z4` 则是 bearing measurements，它们分别与某个 pose 和某个 landmark 共同相关。按照惯例，已知量在 `Bayesian network` 中用方形节点表示。

不过，尽管 `Bayesian networks` 非常适合做建模，作者接下来引入了一种更面向优化（optimization）的图模型，它只聚焦于问题中的未知变量，这就是 `factor graph`。

### 1.1.2 因子图视角

由于局部性（locality），高维概率密度往往能够分解成许多因子的乘积，而每个因子只定义在一个更小的变量域上。`Factor graphs` 正是能够把任意密度写成因子乘积的一类 `probabilistic graphical models`。

为了引出 `factor graphs`，作者再次考虑前面的 toy `SLAM` 例子。由 Bayes 定律，后验 `p(x|z)` 可以改写为 `p(z|x)p(x)` 的形式，也就是由运动先验、landmark 先验以及多项测量似然共同相乘得到。这里的每一个因子，都表示关于未知量 `x` 的一条信息。

为了可视化这种分解，我们使用 `factor graph`。在 Figure `1.3` 中，所有未知状态 `x`，也就是 poses 和 landmarks，都对应着变量节点。测量本身并没有被显式表示出来，因为它们是已知量，并不是图中关注的求解对象。`Factor graph` 额外引入了一类节点，用来表示后验 `p(x|z)` 中的每一个 factor。图中的每个小黑点就是一个 factor 节点，而且关键在于，它只与那些它真正依赖的状态变量相连。

例如，factor `ϕ9(p3, ℓ2)` 只和变量节点 `p3` 与 `ℓ2` 相连。整个联合函数可以写成若干 factors 的乘积。这里每个 factor 的值只需要与相应概率密度成比例即可，那些与状态变量无关的归一化常数都可以省略。这个 toy example 中，所有 factors 要么来自先验，例如 `ϕ1(p1) ∝ p(p1)`，要么来自测量，例如 `ϕ9(p3, ℓ2) ∝ p(z4 | p3, ℓ2)`。

虽然测量变量 `z1 ... z4` 没有在 `factor graph` 中被显式画出来，但这些 measurement factors 实际上都隐式条件于它们。有时为了表达更明确，人们也会把 factor 写成类似 `ϕ(p3, ℓ2; z4)` 的形式。

### 1.1.3 将 Factor Graph 作为一种语言

除了为推断提供形式基础外，`factor graphs` 还有一个重要作用: 它们帮助我们把各种不同形式的 `SLAM` 问题放在统一语言下表达，从而看清问题结构，也让不同背景的研究者和工程人员更容易对齐理解。

在 `factor graph` 中，每个 factor 都可以看成是一条涉及其所连接变量的方程。通常，方程的数量会多于未知量的数量，因此我们必须对先验信息和测量中的不确定性进行量化。这最终会导向最小二乘（least squares）形式，用来把来自不同来源的信息恰当地融合起来。

许多不同 flavor 的 `SLAM` 问题，都可以自然地表示成 `factor graphs`。Figure `1.1` 是一个 `landmark-based SLAM` 例子，因为其中既有 pose variables，也有 landmark variables。Figure `1.4` 则展示了若干其他变体:

- `bundle adjustment (BA)`: 和 `landmark-based SLAM` 类似，但没有 motion model。
- `pose-graph optimization (PGO)`: 不包含 landmark variables，但包含 poses 之间的 `loop closure` measurements。
- `simultaneous trajectory estimation and mapping (STEAM)`: 与 `landmark-based SLAM` 相似，但 pose state 会被扩展，包含例如速度等导数项，并采用平滑的连续时间运动先验。

相比 toy example，一个更真实的 `landmark-based SLAM` 的 `factor graph` 可以像 Figure `1.5` 那样。该图来自一个模拟二维机器人场景: 机器人在平面内运动约 `100` 个时间步，并对 landmarks 进行观测。为了便于可视化，每个机器人 pose 和 landmark 都按其 ground-truth 位置画在二维平面上。由此可以明显看到，`odometry factors` 构成了一条显著的链式骨架，而连接到约 `20` 个 landmarks 上的二元似然因子（binary likelihood factors）则分布在侧边。除了 priors 之外，这类 `SLAM` 问题中的 factors 通常都是非线性的。

观察这样的 `factor graph`，能够直接获得大量关于该 `SLAM` 实例结构的洞见。例如，有些 landmarks 对应很多观测，因此我们预期它们会被较好地“钉住”；另一些 landmarks 与整张图的连接则非常弱，因此其位置会更不确定。比如图右下角那个孤立 landmark，就只有一次测量与之相关。

## 1.2 从 MAP Inference 到 Least Squares

`Maximum a posteriori (MAP) inference`，指的是寻找一组未知量 `x` 的取值，使其与不确定测量和先验信息最为一致。在真实问题中，我们并不知道 landmarks 的 ground-truth 位置，也不知道机器人随时间变化的真实 pose，不过在很多实际场景下，我们可能拥有一个不错的初始估计。作者接下来说明: 在 Gaussian 测量噪声模型和 Gaussian priors 的前提下，`MAP inference` 对应的优化问题，其实正是熟悉的 `nonlinear least squares` 问题。

### 1.2.1 用于 MAP Inference 的 Factor Graphs

我们关心的是未知状态变量 `x`，例如 poses 和 landmarks，并且已知测量 `z`。对这些未知量，最常用的估计器是 `maximum a posteriori (MAP) estimate`，也就是使后验密度 `p(x|z)` 最大的那组状态:

`x_MAP = argmax_x p(x|z)`

由 Bayes 定律可得:

`x_MAP = argmax_x p(z|x)p(x)/p(z) = argmax_x p(z|x)p(x)`

这里第二步只是应用了 Bayes 公式，而第三步则利用了 `p(z)` 与 `x` 无关、不会影响 `argmax` 的事实。因此，`MAP estimate` 本质上是在最大化测量似然 `p(z|x)` 与先验 `p(x)` 的乘积。

我们使用 `factor graphs` 来表达这个未归一化的后验 `p(z|x)p(x)`。形式上，`factor graph` 是一个二部图 `F = (U, V, E)`，其中包含两类节点:

- factors `ϕ_i ∈ U`
- variables `x_j ∈ V`

边 `e_ij ∈ E` 只存在于 factor 节点与 variable 节点之间。与某个 factor `ϕ_i` 相邻的变量节点集合记为 `X(ϕ_i)`，而 `x_i` 则表示对此变量集合的一组赋值。在这些定义下，`factor graph F` 规定了一个全局函数 `ϕ(x)` 的分解方式:

`ϕ(x) = ∏_i ϕ_i(x_i)`

换句话说，变量间的独立关系由图中的边编码；每个 factor `ϕ_i` 只依赖其邻接集合 `X(ϕ_i)` 中的变量。

因此，对任意一个 `factor graph`，`MAP inference` 就归结为最大化所有 factor potentials 的乘积:

`x_MAP = argmax_x ϕ(x) = argmax_x ∏_i ϕ_i(x_i)`

剩下的问题，就是如何具体写出每个 factor `ϕ_i(x_i)` 的形式，而这又取决于我们如何建模测量模型 `p(z|x)` 和先验密度 `p(x)`。

### 1.2.2 概率密度的定义

上面提到的 `p(z|x)` 与 `p(x)` 的具体形式，很大程度上取决于应用场景和传感器类型。最常见的形式是多元 Gaussian density:

`N(θ; µ, Σ)`

其中 `µ ∈ R^n` 是均值，`Σ` 是 `n×n` 协方差矩阵，而对应的平方 `Mahalanobis distance` 为:

`||θ - µ||^2_Σ = (θ - µ)^T Σ^{-1} (θ - µ)`

更精确地说，多元 Gaussian density 的显式形式为

`N(θ; µ, Σ) = 1 / sqrt(|2πΣ|) * exp(-1/2 ||θ - µ||^2_Σ)`  。`(1.9)`

其中

`||θ - µ||^2_Σ = (θ - µ)^T Σ^{-1} (θ - µ)`  。`(1.10)`

这里的归一化常数 `sqrt(|2πΣ|) = (2π)^(n/2) |Σ|^(1/2)`，其中 `|.|` 表示矩阵行列式；它保证多元 Gaussian density 在定义域上的积分为 `1`。

对未知量的 priors 往往可以用 Gaussian density 来表示；在很多场景里，把测量建模为受到零均值 Gaussian 噪声污染，既合理又方便。例如，从某个 pose `p` 到某个 landmark `ℓ` 的 bearing measurement，可以写成:

`z = h(p, ℓ) + η`  。`(1.11)`

其中 `h(·)` 是 measurement prediction function，而噪声 `η` 服从测量协方差为 `Σ_R` 的零均值 Gaussian density。这就导出了测量的条件密度 `p(z | p, ℓ)`:

`p(z | p, ℓ) = N(z; h(p, ℓ), Σ_R) = 1 / sqrt(|2πΣ_R|) * exp(-1/2 ||z - h(p, ℓ)||^2_{Σ_R})`  。`(1.12)`

在实际机器人问题中，测量函数 `h(·)` 往往是非线性的。不过尽管如此，它们通常并不难理解，也不难写出。例如，在二维 bearing measurement 中，测量函数就是:

`h(p, ℓ) = atan2(ℓ_y - p_y, ℓ_x - p_x)`  。`(1.13)`

其中 `ℓ_x, ℓ_y` 是 landmark 的 `x`、`y` 坐标，`p_x, p_y` 是 pose 的 `x`、`y` 坐标，而 `atan2` 是熟知的双参数反正切函数。因此最终的概率测量模型 `p(z | p, ℓ)`，也可写成

`p(z | p, ℓ) = 1 / sqrt(|2πΣ_R|) * exp(-1/2 ||z - atan2(ℓ_y - p_y, ℓ_x - p_x)||^2_{Σ_R})`  。`(1.14)`

当然，本书并不总是假设 Gaussian measurement noise。为了应对偶发的数据关联错误，不少工作提出了具有 heavier tails 的 robust measurement densities，这部分会在 Chapter `3` 里详细讨论。

并非所有概率密度都来自测量。例如，在 toy `SLAM` 中，轨迹上的先验 `p(x)` 由 `p(p1)` 以及条件密度 `p(p_{t+1} | p_t)` 组成，它们规定了机器人在已知控制输入 `u_t` 下遵循的概率运动模型（motion model）。实践中，人们通常采用条件 Gaussian 假设:

`p(p_{t+1} | p_t, u_t) = 1 / sqrt(|2πΣ_Q|) * exp(-1/2 ||p_{t+1} - g(p_t, u_t)||^2_{Σ_Q})`  。`(1.15)`

其中 `g(·)` 是 motion model，`Σ_Q` 是相应维度的协方差矩阵，例如二维平面机器人时常见的是 `3×3`。原书在这里也再次提醒: 若状态里包含旋转变量，还会涉及额外 subtleties，这些内容将在下一章更完整地处理。

很多时候我们没有已知控制输入 `u_t`，而是拥有关于机器人如何运动的观测，例如 odometry measurement `o_t`。如果我们假设 `odometry` 测量的是相邻 poses 的差，并带有协方差 `Σ_S` 的 Gaussian 噪声，就能得到相应的概率模型 `p(o_t | p_{t+1}, p_t)`。如果同时拥有控制输入 `u_t` 和 odometry measurements `o_t`，那么这两类信息还可以进一步组合。

需要注意的是，当机器人在三维空间中运动时，我们需要更复杂的数学工具来在诸如 `SE(3)` 这样的非线性流形上定义密度；这将在下一章讨论。

### 1.2.3 非线性最小二乘

接下来，作者说明: 当 `SLAM` 问题中的所有 factors 都与某个 multivariate Gaussian 成比例时，`MAP inference` 就等价于求解一个 `nonlinear least squares` 问题。换言之，如果每个 factor 都具有类似

`ϕ_i(x_i) ∝ exp(-1/2 ||z_i - h_i(x_i)||^2_{Σ_i})`

这样的形式，那么将它代入 `MAP` 目标，并取负对数、去掉常数项后，就得到一个最小化若干非线性最小二乘项之和的问题:

`x_MAP = argmin_x Σ_i ||z_i - h_i(x_i)||^2_{Σ_i}`

最小化这个目标函数，本质上就是通过多个 measurement-derived factors 和可能存在的 priors 来执行 `sensor fusion`，从而得到未知量的 `MAP solution`。

一个重要但不太显然的观察是，式中的这些 factors 往往对应的是“欠定”的密度。也就是说，除了一些简单 prior factors 之外，测量 `z_i` 的维度通常低于未知量 `x_i` 的维度，因此单独一个 factor 往往会对 `x_i` 的某个无限子集赋予相同的 likelihood。书中给的例子是: 图像中的一个二维观测，与一整条能够投影到同一像素位置的三维射线上的点都一致。

尽管 `h_i` 通常是非线性的，只要我们拥有一个足够好的初始猜测，本章讨论的非线性优化方法通常就能收敛到该问题的全局最小值。不过作者也提醒，由于这个目标函数本身是非凸（non-convex）的，如果初始值不好，优化就可能陷入局部极小值。这也正是后面 Chapter `6` 所讨论的 `certifiably optimal solvers` 之所以重要的原因。

## 1.3 求解线性最小二乘

### 1.3.1 线性化

为了解非线性最小二乘问题，我们首先要对各个 measurement functions `h_i(·)` 做线性化。做法很直接，就是在某个线性化点 `x_i^0` 处对 `h_i` 做一阶 Taylor 展开:

`h_i(x_i^0 + δ_i) ≈ h_i(x_i^0) + H_i δ_i`

其中 `H_i` 是 measurement Jacobian，也就是 `h_i(·)` 对状态变量的偏导；`δ_i = x_i - x_i^0` 则是状态更新向量。这里默认 `x_i` 处在一个向量空间中，或者至少可以用向量表示。对于三维旋转等流形变量，这一假设并不总是成立，这也是 Chapter `2` 要处理的问题。

把这个 Taylor 展开代回 `nonlinear least squares` 目标后，就得到一个关于更新量 `δ` 的 `linear least squares` 问题。此时，项 `z_i - h_i(x_i^0)` 表示在线性化点上的 prediction error，也就是实际测量与预测测量之间的差。

接下来，作者通过一个简单的变量变换把协方差矩阵 `Σ_i` 消掉。做法是利用 `Σ_i` 的矩阵平方根，把每个 Jacobian 和 prediction error 都左乘 `Σ_i^{-1/2}`:

- `A_i = Σ_i^{-1/2} H_i`
- `b_i = Σ_i^{-1/2} (z_i - h_i(x_i^0))`

这一过程就是 `whitening`。对标量测量来说，它等价于用测量标准差去除对应项。经过 whitening 之后，来自不同单位的测量，例如距离和角度，就都能被纳入同一个统一的 cost function。

### 1.3.2 将 SLAM 表述为线性最小二乘

完成线性化后，`SLAM` 的 inner loop 就变成了一个标准的 least squares 问题:

`δ* = argmin_δ Σ_i ||A_i δ_i - b_i||^2`  。`(1.24a)`

`δ* = argmin_δ ||Aδ - b||^2`  。`(1.24b)`

这里，大矩阵 `A` 和右端项 `b` 都是把所有 whitened Jacobian matrices `A_i` 与 whitened prediction errors `b_i` 分别收集并拼接起来得到的。`A` 往往是一个“大而稀疏”的矩阵，其块结构直接反映底层 `factor graph` 的结构。稍后我们会更详细讨论这种稀疏性；在此之前，作者先回顾求解这类 least squares 问题的经典线性代数方法。

### 1.3.3 用于最小二乘的矩阵分解

对于一个满秩的 `m × n` 矩阵 `A`，且 `m ≥ n`，其 least squares 解可以通过求解 normal equations 得到:

`(A^T A) δ* = A^T b`

通常会先构造 `information matrix`，也常被称为 `Hessian`:

`Λ = A^T A = R^T R`

这里的 `R` 是上三角 `Cholesky factor`。求得 `R` 后，就可以先解:

- `R^T y = A^T b`
- `R δ* = y`

分别通过前代和回代得到 `δ*`。对于稠密矩阵，`Cholesky factorization` 的成本大致是 `n^3 / 3` 量级。

另一种更数值稳定、也更精确的方法，是直接对 `A` 做 `QR-factorization`，而不显式构造 `Λ = A^T A`。在 QR 方法中:

- `A = Q [R; 0]`
- `[d; e] = Q^T b`

由于 `Q` 是正交矩阵，因此最小二乘问题可转化为:

`||Aδ - b||^2 = ||Rδ - d||^2 + ||e||^2`

于是 least squares 解由上三角系统

`Rδ* = d`

直接得到。作者指出，QR 得到的上三角因子 `R` 与 Cholesky 版本的 `R` 是一致的，只是可能在对角符号上不同。两者的计算复杂度同属 `O(mn^2)`，但在 `m ≫ n` 时，QR 往往比 Cholesky 慢一个约 `2` 倍的常数因子。

总结来说，线性化后的 `SLAM` 问题，可以概括为: 把 `information matrix Λ` 或 measurement Jacobian `A` 做平方根形式的因式分解。这也是 `square-root SAM` 这个名字的来源。

## 1.4 非线性优化

本节讨论若干经典的非线性优化方法，它们都用于求解 `SLAM` 中的 `nonlinear least squares` 问题。一般形式的目标函数可以写成:

`J(x) = Σ_i ||z_i - h_i(x_i)||^2_{Σ_i}`

这类问题通常不能直接一次性解出，而需要从一个合适的初始估计出发，迭代地构建局部线性近似并不断更新状态。所有这些方法的共同结构是:

- 从初始值 `x^0` 出发
- 在第 `t` 次迭代中计算一个更新量 `δ`
- 用 `x^{t+1} = x^t + δ` 形成新估计
- 当 `δ` 的范数足够小等收敛条件满足时停止

### 1.4.1 最速下降

`Steepest Descent (SD)` 沿当前点的最速下降方向前进，其更新形式为:

`δ_sd = -α ∇J(x)|_{x=x^t}`

其中 `x^t` 是当前对 `x` 的估计，负梯度用于给出 cost function 最速下降的方向。对于 `nonlinear least squares` 目标函数 `(1.34)`，若在当前线性化点附近把目标局部近似为二次型

`J(x) ≈ ‖A(x - x^t) - b‖^2`

则可在 `x = x^t` 处得到精确梯度

`∇J(x)|_{x=x^t} = -2A^T b` 。

步长 `α` 需要仔细选择，以在“更新安全性”和“收敛速度”之间取得平衡；常见做法是显式执行一次 line search，在当前下降方向上寻找更合适的极小值。`SD` 算法形式简单，但在接近最小值时通常收敛较慢。

### 1.4.2 Gauss-Newton 方法

`Gauss-Newton (GN)` 利用 `nonlinear least squares` 的特殊结构，通过 `A^T A` 来近似 Hessian，从而使用二阶更新:

`A^T A δ_gn = A^T b`

这个更新量可以通过第 `1.3.3` 节介绍的任意一种方法来求解相应的 normal equations。当目标函数足够“规整”，也就是局部确实接近二次函数，且初始估计较好时，`Gauss-Newton` 往往能表现出接近二次的收敛速度；但若局部二次近似较差，则一次 `GN` 步也可能把估计推离极小值，并导致后续发散。

### 1.4.3 Levenberg-Marquardt 方法

`Levenberg-Marquardt (LM)` 通过在 `GN` 的 normal equations 对角线上加入阻尼项 `λ` 来控制信任区域（trust region）:

`(A^T A + λI) δ_lm = A^T b`

当 `λ = 0` 时，它退化为 `Gauss-Newton`；而当 `λ` 很大时，它又近似于沿负梯度方向更新，因此自然实现了 `GN` 与 `SD` 之间的平滑过渡。Marquardt 后来又提出按对角元素缩放的版本:

`(A^T A + λ diag(A^T A)) δ_lm = A^T b`

这种改进使得在平坦方向上可以走更大的步，在陡峭方向上则更保守。与 `GN` 不同，`LM` 会拒绝那些导致残差平方和增大的更新；一旦更新被拒绝，就增大 `λ` 并重新求解，直到找到可接受的步长。

### 1.4.4 Dogleg 最小化

`Powell’s dogleg (PDL)` 是 `LM` 的一种更高效替代方案。`LM` 的一个主要缺点是，一旦某一步被拒绝，就需要重新分解修改后的信息矩阵，而这正是最昂贵的部分。`PDL` 的关键思想是分别计算 `GN` 步和 `SD` 步，然后在两者之间构造一条折线型路径，也就是所谓的 `dogleg`。

如 Figure `1.6` 所示，这条组合步长会先沿 `SD` 更新方向前进，然后突然折向 `GN` 更新方向，但始终在 trust region 边界处停止。与 `LM` 不同，`PDL` 显式维护一个我们愿意信任线性近似的 trust region。线性化是否合适，通过 gain ratio

`ρ = (J(x^t) - J(x^t + δ)) / (L(0) - L(δ))`  。`(1.39)`

来判断，其中

`L(δ) = A^T Aδ - A^T b`

表示在当前估计 `x^t` 处，对原非线性二次 cost `J` 的线性化。若 `ρ` 很小，也就是例如 `ρ < 0.25`，说明实际 cost 的下降没有达到线性化所预测的程度，于是 trust region 会被缩小；反过来，若下降与预测一致甚至更好，例如 `ρ > 0.75`，则会根据更新向量的大小增大 trust region，并接受这一步。因此，`PDL` 的一个关键优点是: 每次状态更新通常只需一次矩阵分解，而不是像 `LM` 那样在被拒步时可能需要重复多次分解。

## 1.5 Factor Graph 与稀疏性

前面介绍的求解器都默认矩阵可能是稠密的，但这种方法无法扩展到真实规模的 `SLAM` 问题。真实问题中，未知量数量常常达到几千、几百万，而之所以还能求解，根本原因就在于 `sparsity`。

从 `factor graph` 上就能直接看出这种稀疏性。以较大的模拟例子为例，`100` 个 pose 之间的 odometry chain 只形成一条链，而不是一个完全连接的图；`20` 个 landmarks 也并不会和所有 poses 都相连；landmarks 之间甚至完全没有因子相连，因为我们根本没有给出它们之间相对位置的信息。这正是大多数 `SLAM` 问题的典型结构。

### 1.5.1 稀疏 Jacobian 及其因子图

现代 `SLAM` 算法的关键，在于利用稀疏性，而 `factor graph` 的一个重要性质，就是它能直接表达 resulting sparse Jacobian matrix `A` 的块稀疏结构。

在线性 least squares 问题中，每个 factor 对应 `A` 中的一块 block-row，每个变量则对应一块 block-column。因此，`A` 的 block sparsity pattern 与 `factor graph` 完全一致。换言之，图结构本身就编码了 Jacobian 的稀疏模式。

作者接下来正是利用这一点，把 `factor graph` 与后续的稀疏矩阵计算直接联系起来。

### 1.5.2 稀疏信息矩阵及其图结构

如果使用 `Cholesky factorization` 去解 normal equations，就需要先构造 `information matrix`:

`Λ = A^T A`

严格来说，这并不是真正的 Hessian，而是 `Gauss-Newton` 近似得到的版本。不过，由于 `A` 是 block-sparse，`Λ` 通常也会保持稀疏。`Λ` 还天然定义了一张无向图 `G`: 当且仅当两个变量曾在同一个 factor 中同时出现时，这两个变量之间就在 `G` 中有边。对偶数因子之外的更高元因子来说，这种关系会在相关变量间诱导出一个 clique。

因此，`Λ = A^T A` 的 block sparsity pattern，实际上就对应于这张无向图的邻接结构。这张图在后面讨论稀疏性和 `fill-in` 时会反复出现。

### 1.5.3 稀疏分解

在 `nonlinear least squares` 中，我们需要不断重复求解这样的线性系统。既然 `A` 和 `A^T A` 都具有由 `factor graph` 决定的稀疏性，我们就可以借此大幅加速 `Cholesky` 或 `QR-factorization`。实际工程里常用的高效实现包括 `CHOLMOD` 和 `SuiteSparseQR`。

对于稀疏矩阵，factorization 的 flop 数远低于稠密情形，但一个非常关键的因素是: 变量的列排序（column ordering）会显著影响总计算成本。虽然任何变量顺序最终都会得到相同的 `MAP estimate`，但不同顺序会决定 factorization 中产生多少 `fill-in`，也就是那些原本不存在、但在分解过程中新出现的非零项。

最小化 `fill-in` 的最优排序本身是一个 `NP-hard` 问题，因此只能依赖好的启发式方法。书中示例表明:

- 如果按“poses 在前、landmarks 在后”的自然顺序排列，得到的稀疏 `R` 因子有 `9399` 个非零项。
- 如果使用 `COLAMD` 这类启发式重新排序，则 `R` 中非零项可降到 `4168` 个。

两者通过回代得到的解完全一致，但计算成本会大不相同。作者也提到，诸如 pre-conditioned conjugate gradient 之类的方法也可以迭代地求解 normal equations；不过对大多数 `SLAM` 问题而言，稀疏 factorization 仍然是首选，而且它还拥有非常自然的图模型解释。

## 1.6 消元

到目前为止，前面的解释主要建立在线性代数视角之上。本节把视角扩展到更一般的图模型推断框架，直接从 `graphical models` 来理解 `SLAM` 推断。这将导向下一节要讲的 `Bayes tree`，也就是现代增量式 `SLAM` 求解器背后的核心概念。

### 1.6.1 变量消元算法

`Variable elimination` 是一个非常通用的算法。给定任意一个最好是稀疏的 `factor graph`，它能够把未知变量的后验 `p(x|z)` 转换为一种更方便恢复 `MAP solution` 的形式。前面我们已经看到，`factor graph` 表示的是未归一化的后验 `ϕ(x) ∝ p(x|z)`；而 `variable elimination` 的作用，就是把这个 `factor graph` 转换成另一种图模型，即 `Bayes net`。

形式上，它把如下形式的因子图:

`ϕ(x) = ϕ(x1, ..., xn)`

分解成如下的 `Bayes net` 概率密度:

`p(x) = ∏_j p(x_j | s_j)`

其中 `s_j` 是与变量 `x_j` 对应的 separator，也就是在某一给定变量消元顺序下，`x_j` 最终所依赖的那组变量。由于 `SLAM` 的 `factor graph` 往往本身很稀疏，所以消元后得到的 separators 通常也比较小。

算法的过程是逐个消去变量。对当前要消去的变量 `x_j`:

1. 找到所有与 `x_j` 相邻的 factors。
2. 把它们相乘形成一个中间 product factor `ψ(x_j, s_j)`。
3. 将 `ψ(x_j, s_j)` 再分解成两部分:
   `ψ(x_j, s_j) = p(x_j | s_j) τ(s_j)`

这里，`p(x_j | s_j)` 是关于被消去变量的 conditional density，而 `τ(s_j)` 是只定义在 separator 上的新 factor。重复这一过程，直到所有变量都被消去，就得到一个完整的 `Bayes net`。

在 toy `SLAM` 例子中，如果按 `ℓ1, ℓ2, p1, p2, p3` 的顺序消元，就可以把原始 `factor graph` 转换成一个 `Bayes net`，其联合分解形式类似:

`p(ℓ1, ℓ2, p1, p2, p3) = p(ℓ1 | p1, p2) p(ℓ2 | p3) p(p1 | p2) p(p2 | p3) p(p3)`

这清楚地展示了 `Bayes net` 中的条件依赖结构。

### 1.6.2 线性高斯消元

当测量函数是线性的、噪声是加性 Gaussian 时，`elimination algorithm` 与稀疏矩阵 factorization 就完全等价。无论是稀疏 `Cholesky` 还是稀疏 `QR-factorization`，都可以看作这个一般算法的特例。

对某个变量 `x_j`，我们把所有相邻 factors 提取出来，合并成一个更大的块矩阵与 RHS，形成 product factor `ψ(x_j, s_j)`。随后对其做局部 `QR-factorization`，就能把这个 product factor 分解成:

- 一个关于被消去变量的条件项 `p(x_j | s_j)`
- 一个只依赖 separator 的新 factor `τ(s_j)`

在 toy example 中，消去 `ℓ1` 时，它相邻的 factors 是 `ϕ4`、`ϕ7` 和 `ϕ8`，其 separator 是 `[p1, p2]`。在稀疏 Jacobian 视角下，这等价于把第一列中所有非零块所在的 block-rows 抽取出来，做一次局部稀疏因式分解。

完成全部消元后，所得的条件密度 `p(x_j | s_j)` 都是 `linear-Gaussian` 的。求解 `MAP estimate` 时，只需按消元的逆顺序做 `back-substitution` 即可: 最后消去的变量不依赖其他变量，能直接从 `Bayes net` 中读出；然后依次代回，就能恢复所有变量的 `MAP` 值。

作者还指出，对多维变量做这样的逐块消元，本质上就是 `multi-frontal QR factorization`。在推断语义上，自然的变量分组和稀疏线性代数里为了计算效率而做的分组，很多时候是高度一致的。

### 1.6.3 作为 Bayes Net 的稀疏 Cholesky 因子

变量消元与稀疏矩阵 factorization 的等价性揭示了一个很漂亮的事实: 与一个上三角矩阵对应的图模型，本质上就是一个 `Bayes net`。正如 `factor graph` 是稀疏 Jacobian 的图形表达，`Bayes net` 则揭示了 `Cholesky factor` 的稀疏结构。

更进一步，`Cholesky factor` 对应的还是一个 `Gaussian Bayes net`，也就是由 `linear-Gaussian conditionals` 组成的 `Bayes net`。当因子图仅包含线性测量函数和 Gaussian 加性噪声时，消元之后得到的 `Bayes net` 具有非常具体的形式:

`p(x) = ∏_j p(x_j | s_j)`

其中每个条件项都可以写成线性高斯形式，其均值是 separator 的线性函数，而协方差则由对应三角块 `R_j` 决定。

消元完成之后，`MAP estimate` 的恢复也非常直接。只要按照逆消元顺序做 back-substitution，每一步都把当前变量设为对应 conditional mean，就能得到整组 `MAP` 解。

## 1.7 增量式 SLAM

在增量式 `SLAM` 场景中，机器人边走边收集新测量，我们希望每当有新测量到来时，就更新当前的最优轨迹和地图，或者至少定期更新。在线性情形中，可以通过增量式矩阵 factorization 重用已有计算结果；但实际 `SLAM` 多半是非线性的，因此如何在不重做完整分解的前提下完成增量式 re-linearization，并不是显然的事情。

为了解决这个问题，作者再次诉诸图模型，并引入一种新的图结构: `Bayes tree`。随后说明如何随着新测量与新状态的加入，对 `Bayes tree` 做增量更新，并由此得到 `incremental smoothing and mapping (iSAM)` 算法。

### 1.7.1 Bayes Tree

众所周知，在树结构图上做推断是高效的；但 `SLAM` 的 `factor graphs` 往往包含大量 loops。不过可以通过两个步骤构造出一个树状图模型:

1. 对因子图执行 `variable elimination`，得到一个满足特殊性质的 `Bayes net`
2. 利用这一性质，在该 `Bayes net` 的 cliques 之上建立树结构

这里的关键性质是 `chordal`，也就是任何长度大于 `3` 的无向环都带有 chord。在 AI 和机器学习文献里，这通常也叫 `triangulated`。虽然这个 `Bayes net` 仍然在变量层面上不是一棵树，但它已经具备能够重组成 clique tree 的结构。

`Bayes tree` 可以看作是这种 `clique tree` 的有向版本，它保留了变量消元顺序信息。更形式化地说，`Bayes tree` 的每个节点对应底层 chordal `Bayes net` 的一个 clique `c_k`，并定义一个条件密度 `p(f_k | s_k)`，其中:

- `s_k` 是该 clique 与其父 clique 的交集，也就是 separator
- `f_k = c_k \ s_k` 是 frontal variables

于是整组变量的联合密度可以写成:

`p(x) = ∏_k p(f_k | s_k)`

在 toy `SLAM` 例子中，根 clique 包含 `p2, p3`，它与其他两个 cliques 分别通过 `p2` 和 `p3` 相连。`Bayes tree` 也清晰揭示了平方根信息矩阵 `R` 的行块如何对应到不同 cliques 上。

### 1.7.2 更新 Bayes Tree

增量推断可以被看成是对 `Bayes tree` 的一次简单编辑。这个视角比“增量矩阵分解”那种更抽象的说法要直观得多，也更好地解释了为什么 `Bayes tree` 本身就是平方根信息矩阵的一种非常有意义的稀疏存储结构。

要对 `Bayes tree` 做增量更新，核心做法是: 只把树中受影响的那一部分临时转换回 `factor-graph` 形式。当加入一个新测量时，本质上就是增加了一个新 factor。例如，一个涉及两个变量的二元测量会引入新 factor `ϕ(x_j, x_j')`。在这种情况下，真正受到影响的，只是 `Bayes tree` 中“包含 `x_j` 和 `x_j'` 的 cliques 到根节点之间的路径”。这些 cliques 下面的子树，以及所有不包含这两个变量的其他子树，都不会被影响。

因此，增量更新过程是:

1. 只把受影响的那部分 `Bayes tree` 转回 `factor graph`
2. 在这个临时因子图中加入新的测量 factor
3. 用合适的 elimination ordering 重新消元
4. 得到新的局部 `Bayes tree`
5. 把未受影响的子树重新挂回去

作者接着解释，为什么通常只有树的“上半部分”会被影响。这直接来自 `Bayes tree` 编码的信息流结构。`Bayes tree` 是由 chordal `Bayes net` 按逆消元顺序构造出来的，因此每个 clique 中的变量，都会在其子 clique 被消去之后，从这些子 clique 收集信息。也就是说，信息在树中只会沿着“从子到父、最终到根”的方向传播。

第二个关键性质是: 一个 factor 所携带的信息，只有在与它相连的第一个变量被消去时，才真正进入消元过程。把这两点放在一起看，就能明白: 一个新 factor 不可能影响那些并非其变量后继（successors）的变量；但如果一个 factor 连接了两条原本独立的、通向根的路径上的变量，那么为了表达这条新的依赖关系，这两条路径就必须被重新消元。

Figure `1.14` 用书中的 canonical `SLAM` 例子直观展示了这一过程。作者在原有 `Bayes tree` 上加入一条连接 `p1` 与 `p3` 的新 factor，因此真正受影响的只有树的左支，也就是图中红色虚线圈出的部分。接着，系统把该部分树重新转换成 factor graph: 其中为 clique density `p(p2, p3)` 与 `p(ℓ1, p1 | p2)` 分别生成 factor，再额外插入新的 `f(p1, p3)`。然后按照 `ℓ1, p1, p2, p3` 的顺序重新消元，得到新的 chordal `Bayes net`，再由它重建新的局部 `Bayes tree`。最后，原树中那个没有改动的右侧绿色子树会被重新接回。

Figure `1.15` 则展示了一个更真实的小规模 `SLAM` 例子，即著名 `Manhattan world` 模拟序列在 step `400` 时的 `Bayes tree`。这里可以清楚看到，随着机器人探索环境，新测量通常只会影响树中的很小一部分，因此真正需要重新计算的也只是那一小块局部结构。这也正是 `iSAM2` 能够高效实现增量非线性最小二乘估计的核心原因。

### 1.7.3 增量平滑与建图

把以上内容和 re-linearization 的实际工程处理结合起来，就得到一个面向机器人 `MAP estimation` 的先进增量式非线性方法: `iSAM`。第一版 `iSAM1` 使用了经典的增量矩阵 factorization 方法；但它在线性化处理上并不理想，因为往往需要周期性地对整张因子图重新线性化，或者在 matrix fill-in 太严重时整体重做。

第二版 `iSAM2` 则改用 `Bayes tree` 来表示后验密度，并在每次新测量到来时执行前面描述的 `Bayes tree` 增量更新。因此，机器人探索环境时，新测量通常只会影响树中很小的一部分，只有这些局部部分需要重新计算。

在重新消元受影响 cliques 时，还涉及变量排序的选择。一个简单策略是对受影响变量局部应用 `COLAMD`；但更好的办法，是把最近访问的变量强制放到排序的末尾，也就是推入根 clique。为此可以使用 `constrained COLAMD`。这种策略通常会使后续更新继续局限在树的较小部分，从而保持高效，只有大型 `loop closures` 才可能显著扩大影响范围。

树更新完成后，还需要更新解。`Bayes tree` 上的 back-substitution 从根开始向叶子传播，但通常没有必要为所有变量都重新求解。因为局部树修改往往不会影响远处变量，所以可以在每个 clique 处检查向下传播的变量变化量，一旦变化低于阈值，就提前停止。

为了在非线性问题中真正做到增量式求解，`iSAM2` 还会选择性地重新线性化那些偏离当前 linearization point 超过阈值的 factors。与单纯的树结构编辑不同，这一步不仅需要重做那些把受影响变量当作 frontal variables 的 cliques，也需要重做那些把它们作为 separator variables 的 cliques。尽管如此，这通常仍然远比从头重建整棵树便宜得多。

`iSAM1` 和 `iSAM2` 已经成功应用于许多大型机器人估计问题，变量数量可达数百万级。两者都实现于 `GTSAM` 库中。

## 1.8 延伸阅读与最新趋势

如果读者希望更深入了解 `factor graphs`，作者推荐阅读 Dellaert 等人的更长文章。由于本章主要面向入门，因此不得不压缩许多更高级的内容，以尽量降低阅读门槛。但无论是 `factor graphs` 本身，还是更一般的 elimination algorithm，实际上都非常强大，值得系统掌握。

进一步地，如果想真正理解 `Bayes tree` 这一概念，以及它如何支撑 `GTSAM` 等现代求解器，推荐继续阅读 Kaess 等人的工作。

从今天的视角看，`factor graph` 范式已经能够:

- 处理定义在 manifolds 上的状态变量
- 集成许多不同类型的传感器
- 处理离群点测量
- 甚至融合深度学习方法的输出

本 handbook 后续大部分内容，其实都在围绕这些趋势展开。换句话说，第一章只是起点，后面的章节会不断扩展 `factor graph` 这一核心范式。

## 第 2 章 高级状态变量表示

### 本章概览

上一章详细说明了如何使用 `factor-graph` 范式建立并求解一个 `SLAM` 问题，但当时刻意略过了状态变量本身的一些细节。本章重新审视这些待估计状态的性质，并引入现代 `SLAM` 中非常常见的两个主题。第一，是如何处理带有约束的状态变量。这类约束通常定义了一个 manifold，因此在优化时不能再把变量简单当作普通向量处理。`SLAM` 中最常见的例子，就是与机器人旋转相关的状态，尤其是三维旋转与位姿。第二，是时间建模方式本身。上一章默认机器人按离散时间步在世界中运动，而本章进一步介绍平滑的连续时间轨迹表示，并说明它们如何与 `factor graph` 框架自然兼容。

本章前半部分聚焦 `optimization on manifolds`，重点讨论旋转、位姿、`matrix Lie groups`、`Lie algebra` 以及在这些结构上进行 `MAP optimization` 的方式（Section `2.1`）。后半部分转向 `continuous-time trajectories`，介绍参数化样条（splines）、由 `kernel trick` 引出的非参数方法，以及 `Gaussian Processes (GPs)` 在轨迹表示中的作用，并最后把这些方法推广到 `Lie groups` 上（Section `2.2`）。本章最后以延伸阅读与研究趋势收尾（Section `2.3`）。

## 2.1 流形上的优化

在某些机器人问题里，我们可以直接把未知量表示成普通向量；但在大多数实际场景中，都必须处理三维旋转以及其他不属于向量空间的 manifold。粗略地说，manifold 可以理解成一个拓扑上闭合的曲面，例如圆周边界或球面。它的关键性质是: 在每个点的局部邻域内，它都近似于 Euclidean space。因此，manifold 上的优化需要比普通向量空间更细致的工具链。

本节讨论如何在 manifold 上做优化，并把上一章面向向量空间的优化框架推广过来。书中用球面作为示意例子: manifold 本体记作 `M`，而在某个点 `χ ∈ M` 附近，可以用切空间 `TχM` 作为局部坐标系来开展优化。

### 2.1.1 旋转与位姿

在 `SLAM` 中，最常见的两个 manifold 是用于表示 rotations 和 poses 的 manifold。旋转通常发生在二维平面或三维空间，因此旋转 manifold 一般写作特殊正交群 `SO(d)`，其中 `d = 2` 或 `3`。

二维平面上的 rotation matrix `Rb_a ∈ SO(2)` 可以写成:

`Rb_a = [[cosθ, -sinθ], [sinθ, cosθ]]`

这里 `θ ∈ R` 是唯一的自由度，也就是旋转角。它可以把在参考系 `Fa` 中表示的二维向量 `ℓa` 旋转到 `Fb` 中，得到 `ℓb = Rb_a ℓa`。

三维旋转矩阵 `Rb_a ∈ SO(3)` 的作用与此类似，只不过是把三维向量从一个坐标系变换到另一个坐标系。虽然三维 rotation matrix 有 `9` 个矩阵元素，但只有 `3` 个自由度，例如 `roll`、`pitch` 和 `yaw`。无论二维还是三维，rotation matrix 都必须满足

`Rb_a^T Rb_a = I`，且 `det(Rb_a) = 1`

这些约束正是它无法被当作普通向量随意加减的原因。

机器人的 pose 同时包含旋转 `Rb_a ∈ SO(d)` 和平移 `t^b_a ∈ R^d`，总共有 `3(d - 1)` 个自由度。有时我们把这两部分分开表示，写作 `{R, t} ∈ SO(d) × R^d`；另一种更紧凑的表示方式，是把它们组装成 `(d + 1) × (d + 1)` 的 transformation matrix:

`T^b_a = [[R^b_a, t^b_a], [0, 1]]`

所有这类 transformation matrices 组成的 manifold 被称为特殊欧氏群 `SE(d)`，其中 `d = 2` 表示平面运动，`d = 3` 表示三维运动。使用 `SE(d)` 的一个直接好处，是对 landmarks 的平移和旋转都能统一写成一次矩阵乘法，并且可以在 homogeneous coordinates 下方便表达。

不过，无论是 `SO(d)` 还是 `SE(d)`，由于它们都受结构约束，因此都不是向量。比如，把两个 rotation matrices 直接相加，并不会得到另一个合法的 rotation matrix。幸运的是，`SO(d)` 和 `SE(d)` 同时还是 `matrix Lie groups`。正是这种额外结构，使我们能够在不显式引入约束的前提下，继续完成 `factor-graph SLAM` 中的 `MAP optimization`。

### 2.1.2 矩阵 Lie 群

要在 `SO(d)` 和 `SE(d)` 上做优化，关键是利用它们作为 group 所具有的代数结构。首先，`matrix Lie groups` 在矩阵乘法下是封闭的。也就是说，如果 `Rc_b, Rb_a ∈ SO(d)`，那么它们的乘积 `Rc_a = Rc_b Rb_a` 仍然属于 `SO(d)`。

更重要的是，每个 `matrix Lie group` 都伴随着一个非常有用的 companion structure，称为 `Lie algebra`。`Lie algebra` 同时也是该 `Lie group` 在单位元（identity element）处的切空间（tangent space）。从本章的角度看，`Lie algebra` 最重要的两点是:

- 它本身是一个向量空间，其维度正好等于对应 `Lie group` 的自由度数目。
- 从 `Lie algebra` 到 `Lie group` 之间，存在成熟而标准的映射，也就是 `matrix exponential`。

以 `SO(2)` 为例，我们可以从一个标量角度 `θ` 出发，先通过 `wedge operator` 得到 `Lie algebra` 元素 `θ∧`，再通过 `Exp(θ) = exp(θ∧)` 构造 rotation matrix。反过来，也可以通过 `matrix logarithm` 回到 `Lie algebra`，再用 `vee operator` 取得对应向量表示，即 `θ = Log(R) = (log(R))∨`。

对于 `SO(3)`，`wedge operator` 对应的是把 `R^3` 中的向量写成一个 skew-symmetric matrix；对于 `SE(d)`，`Lie algebra` 元素 `ξ` 则把旋转部分 `θ` 与平移部分 `ρ` 一起编码进 `ξ∧ ∈ se(d)`，并通过

`T = Exp(ξ) = exp(ξ∧) ∈ SE(d)`

生成位姿变换。对这些常见 `Lie groups`，从 `Lie algebra` 到 `Lie group` 的映射都有成熟的 closed-form expressions，因此实践中通常不需要真的去使用无穷级数展开。

### 2.1.3 Lie 群优化

有了 `matrix Lie groups` 之后，我们就可以像在线性最小二乘中做线性化那样，对含 `Lie group` 变量的测量模型进行局部近似，从而继续开展 `MAP inference`。

书中给出的例子是一个相机观测模型。假设 `hi(·)` 接收在相机坐标系下表示的 homogeneous landmark `ℓ̃^c_i`，输出其图像像素坐标 `zi ∈ R^2`。于是生成式传感器模型可以写为:

`zi = hi(T^c_w ℓ̃^w_i) + ηi`

其中 `T^c_w ∈ SE(3)` 是世界坐标系相对于相机坐标系的 pose，`ℓ̃^w_i` 是世界坐标系下的 homogeneous landmark，`ηi` 是传感器噪声。若世界中的 landmark 已知，那么我们就希望通过最小化重投影误差来估计 `T^c_w`，这对应经典的 `perspective-n-point (PNP)` 问题。

在线性化时，关键是不要直接在线性空间里“加一个 pose”，而是通过 `Lie algebra` 构造一个小扰动:

`T^c_w = T^{c0}_w Exp(ξ)`

这里 `ξ ∈ R^6` 表示相对于初始猜测 `T^{c0}_w` 的一个小位姿增量。这种写法常被简写成

`T^c_w = T^{c0}_w ⊕ ξ`

由于 `SE(3)` 在乘法下封闭，更新后的结果仍然保证是合法 pose。更重要的是，`ξ` 的维度刚好等于位姿真实自由度，因此我们不需要在优化中额外处理约束。

若只保留 `Exp(ξ)` 的一阶项，就有近似

`T^c_w ≈ T^{c0}_w (I + ξ∧)`

把它代回测量函数，再结合相机模型自身的一阶 Taylor 展开，就能得到与上一章完全平行的线性化误差形式。也就是说，原本的非线性 measurement factor，最终仍然会被写成一个对扰动变量 `ξ` 线性的 least-squares term。

求得最优扰动 `ξ*` 之后，还必须按照之前选定的扰动方式更新初值:

`T^{c0}_w ← T^{c0}_w ⊕ ξ*`

这样才能保证更新后的解继续落在 `SE(3)` 上，而不是偏离 manifold。整个优化过程和上一章一样，仍然是迭代进行，直到包括 `ξ` 在内的全部状态增量都足够小为止。

这种方法不仅适用于测量函数的输入在 manifold 上的情况，也适用于测量本身位于 manifold 上的情况。例如，若传感器直接给出一个带噪 pose measurement `Zi ∈ SE(3)`，则误差通常在 `Lie algebra` 中定义，通过 `Log(·)` 把 group 中的偏差映射回切空间，然后再进行 least-squares 优化。

从更高层看，这其实是 `Riemannian optimization` 的一个实例。我们利用 `Lie algebra` 这个切空间，把优化约束在 manifold 的切向方向上；而通过 `⊕` 更新或 `retraction`，又把结果重新投回 manifold。除了 `matrix exponential` 之外，也存在其他可用的 `retractions`，整个框架并不局限于 `Lie groups`。

### 2.1.4 不确定性与 Lie 群

在估计问题中，我们通常把状态看成随机变量，并用分布来表示其不确定性。对普通向量变量 `x`，最常见的写法是

`x = μ + δ,   δ ~ N(0, Σ)`

其中 `μ` 是均值，`Σ` 是协方差，`δ` 是零均值 Gaussian noise。

但对 `Lie groups`，不能再直接把噪声加到群元素上。比如对一个 rotation matrix `R ∈ SO(d)`，若直接写成 `R + δ`，结果一般不再是合法 rotation matrix。更合理的做法，是先在 tangent space 中定义 Gaussian noise，再通过 `matrix exponential` 把它映射到 group 上:

`R = R̄ ⊕ δ = R̄ Exp(δ),   δ ~ N(0, Σ)`

对 `SE(d)` 也完全类似:

`T = T̄ ⊕ δ = T̄ Exp(δ),   δ ~ N(0, Σ)`

这样定义之后，`R` 保证仍在 `SO(d)`，`T` 保证仍在 `SE(d)`。换句话说，我们并不是直接在旋转或位姿空间里定义 Gaussian，而是在其 tangent space 中定义 Gaussian，再映射回 manifold。这种做法在本章讨论的“局部不确定性”场景下非常自然。当然，也可以直接在 rotations 上定义分布，某些情况下那样会带来额外的计算优势，本书第六章还会再回到这一点。

### 2.1.5 Lie 群补充内容

关于 `Lie groups`，其实还有许多可以展开的内容。为了不打断主线，本节把后续章节会频繁用到的若干补充工具集中放在一起说明。更细致的导数计算会留到 Section `4.3`。

### 2.1.5.1 `⊕` 与 `⊖` 运算符

前面已经见过 `⊕` 运算符，它把一个 `Lie algebra` 向量与一个 `Lie group` 元素组合起来。例如对 `SE(d)`:

`T = T0 ⊕ ξ = T0 Exp(ξ) = T0 exp(ξ∧)`

与此同时，我们也经常需要比较两个 `Lie group` 元素之间的“差值”。这时可以定义 `⊖` 运算符。仍以 `SE(d)` 为例:

`ξ = T ⊖ T0 = Log(T0^{-1} T) = (log(T0^{-1} T))∨`

`⊕` 和 `⊖` 的好处，在于它们把底层 `exp/log` 与左右乘法的细节统一封装掉了，使公式表达更干净。

### 2.1.5.2 逆运算

在处理 perturbation 时，经常会遇到对一个逆变换做扰动的情况。对 `SE(d)`，如果

`T = T0 ⊕ ξ`

那么它的逆满足

`(T0 ⊕ ξ)^{-1} = (-ξ) ⊕ T0^{-1}`

这表明当对逆矩阵做扰动时，扰动会从右侧移到左侧，并带上负号。

### 2.1.5.3 伴随映射（Adjoint）

`adjoint` 提供了一种把 `Lie group` 元素看成其 `Lie algebra` 上线性变换的方法。对 `SO(d)`，adjoint representation 与 group 本身一致，因此细节较少；对 `SE(d)`，adjoint 则是状态估计里非常重要的工具。

对给定 pose `T`，`SE(d)` 的 adjoint map 可以把局部坐标系中的 `Lie algebra` 元素 `ξ∧`，转换成全局坐标系中的另一个 `Lie algebra` 元素 `ϵ∧`，满足

`ϵ∧ ⊕ T = T ⊕ ξ∧`

并且可以写成所谓的 `inner automorphism` 或 `conjugation`:

`ϵ∧ = Ad_T ξ∧ = T ξ∧ T^{-1}`

如果把它写成线性变换矩阵，那么 `Ad(T)` 就是把 `ξ ∈ R^6` 直接映射到另一个 `R^6` 向量的矩阵表达。对 `SE(d)`，这个 adjoint representation 可以直接由 homogeneous transformation matrix 的分块构造出来。

adjoint 的一个重要用途，是在不做近似的前提下，把 perturbation 从已知变换的一侧搬到另一侧:

`T Exp(ξ) = Exp(Ad(T) ξ) T`

在状态估计推导里，这类换边操作十分常见。

### 2.1.5.4 Jacobian（雅可比）

每个 `Lie group` 都对应自己的 `Jacobian`，用于把 group 元素的变化与 `Lie algebra` 元素联系起来。以 `SO(d)` 为例，经典运动学方程，也就是 `Poisson's equation`，把旋转矩阵 `R ∈ SO(d)` 与角速度 `ω` 联系起来，可写为

`Ṙ = ω^∧ R`

这里 `ω` 表示相对于某一参考系、并在局部坐标中表达的角速度。若进一步把旋转参数化为 `R = Exp(θ)`，则等价地可以写成

`θ̇ = J^{-1}(θ) ω`

其中 `J(θ)` 就是 `SO(d)` 的左 `Jacobian`。它在处理多个 `matrix exponentials` 的组合时特别有用。例如，当 `θ1` 足够小时，有近似关系

`Exp(θ1) Exp(θ2) ≈ Exp(θ2 + J(θ2)^{-1} θ1)`

书中还给出了 `J(θ)` 的级数表达式：

`J(θ) = Σ_(n=0)^∞ 1/(n+1)! (θ^∧)^n`

闭式表达式可参见 `Barfoot` [47]。作者也说明，在后文中常会重载记号，把 `SE(d)` 的左 `Jacobian` 同样记作 `J(ξ)`，具体含义由上下文判断。总体而言，`Lie group Jacobians` 不仅对经典状态估计至关重要，也构成了现代可微神经网络 pose estimation 方法的基础。

## 2.2 连续时间轨迹

`continuous-time trajectories` 提供了一种描述平滑机器人运动的方式。到目前为止，我们默认需要估计的是一串离散时刻的 poses；但实际机器人运动往往是平滑的，因此直接引入连续时间轨迹表示会更加自然。

这类方法主要有两大类。第一类是参数化方法，用一组已知 temporal basis functions 线性组合出一条平滑轨迹。常见选择包括具有局部支撑的样条（splines），例如 `B-splines` 或 cubic Hermite polynomials。局部支撑的好处，是 factor graph 仍然保持稀疏。第二类是非参数方法，其表达能力更强，通常通过 `kernel functions` 建模；特别是把时间作为自变量时，一维 `Gaussian Process (GP)` 可以直接用来表示轨迹。如果核函数选取得当，由此得到的 factor graph 同样可以非常稀疏。

除了轨迹平滑性之外，连续时间表示在处理高频传感器和异步传感器时尤其有价值。若对每一条测量都单独引入一个 pose 变量，那么在 `IMU` 这类高频传感器、或多传感器不同步采样的场景下，factor graph 会迅速膨胀。连续时间轨迹则允许我们用远少于测量数目的状态变量描述整条运动，同时仍然保留每个 measurement 的精确时间戳。

这对存在 motion distortion 的传感器尤为重要，例如 spinning `LiDAR`、`Radar`，甚至 rolling-shutter camera。连续时间方法允许我们把每个点或像素都与其真实采样时刻的轨迹状态对齐。

最后，在 `MAP inference` 完成之后，连续时间轨迹还能支持对任意时刻的轨迹进行高效查询，而不局限于测量发生的时刻。也就是说，`measurement times`、`estimation variables` 和 `query times` 可以彼此分离，这正是连续时间方法的重要优势之一。

### 2.2.1 样条

参数化连续时间轨迹的基本思路，是把 pose 写成若干已知 temporal basis functions `Ψk(t)` 的加权和:

`p(t) = Σ_k Ψk(t) ck`

这里 `ck` 是待估计系数。为了先把思路讲清楚，书中先回到向量空间情形，再在后面讨论如何推广到 `Lie groups`。

常见的 basis functions 是 splines，也就是分段多项式。样条的关键优势，在于它们通常具有 `local support`，也就是在自己影响范围之外函数值为零。这样一来，在任意时刻 `t`，真正起作用的 basis functions 只占很小一部分，因此对应的 factor graph 依然稀疏。

若机器人在时间 `ti` 观测到一个 landmark `ℓ`，则传感器模型可以写作

`zi = hi(p(ti), ℓ) + ηi`

把 spline 表达代入，就得到一个关于活动系数变量和 landmark 变量的 measurement factor。由于在 `ti` 时刻只有少量 basis functions 非零，我们仍然可以把它写成标准的 `zi = hi(xi) + ηi` 形式，于是上一章的非线性 least-squares 框架就可以原样套用。

如果 basis functions 本身足够可微，那么轨迹的导数同样易于计算:

`ṗ(t) = Σ_k Ψ̇k(t) ck`

因此，哪怕传感器输出与速度甚至更高阶导数有关，也仍然可以围绕同一批 spline coefficients 做统一优化。

当 `MAP inference` 求得最优系数之后，我们就能在任意时刻用 spline 表达式去查询轨迹及其导数。若推断过程中同时得到系数的协方差，那么由于 spline 表达式对系数是线性的，查询时刻的 pose 协方差也可以方便地传播出来；而局部支撑性质意味着我们只需相关系数的局部边缘协方差。

### 2.2.2 从参数化到非参数化

基础参数化方法的主要难点，在于我们必须事先决定使用什么类型、多少个 basis functions。若 basis functions 太多，就容易对 measurement data 过拟合；若太少，轨迹表达能力又不够，最终会得到过于平滑的解。这正是转向非参数方法的动机之一。

为了简化讨论，书中暂时只考虑 pose 变量，不讨论 landmarks。在线性化之后，参数化方法的 least-squares term 会呈现出一个关于 spline coefficient updates `δc` 的优化问题。为了抑制过拟合，还会显式加入一个 regularizer `||δc||^2`，相当于偏好较小的 spline coefficients。

不过，实际关心的通常不是这些 coefficients 本身，而是各测量时刻的 pose 更新 `δ = Ψ δc`。经过代数整理后，优化问题可以改写成一个直接以 pose updates 为未知量的线性系统:

`(A^T A + K^{-1}) δ* = A^T b`

其中 `K = ΨΨ^T` 被称为 `kernel matrix`，起到 regularization 或 smoothing 的作用。

到这里，作者引入著名的 `kernel trick`。它的核心思想是: 不再显式表示 basis functions，而只使用它们之间的内积。于是我们可以直接用一个选定的 kernel function `K(t, t')` 来填充 kernel matrix，而不再估计 spline 的显式系数。这样一来，方法就从参数化转向了非参数化。

当然，代价是我们需要调节核函数的 hyperparameters，例如 squared-exponential kernel 的 length scale，以获得想要的轨迹平滑程度。另一个问题是，若直接这么做，`K^{-1}` 往往会是稠密的，从而导致推断代价很高。下一节要解决的，正是如何选取一种既是 GP kernel、又能保证逆核矩阵稀疏的构造方式。

### 2.2.3 Gaussian Processes（高斯过程）

本节构造一类特殊的 kernel functions，使得对应的 inverse kernel matrix 天然稀疏，因此 factor graph 也保持稀疏。与其从显式 basis functions 出发再套 `kernel trick`，作者改从一个 `linear, time-invariant stochastic differential equation (SDE)` 出发:

`ẋ(t) = A x(t) + L w(t)`

其中 `w(t) = GP(0, Q δ(t - t'))` 是零均值白噪声 `Gaussian Process`，`Q` 是 `power-spectral density matrix`，`δ(·)` 是 Dirac delta function。这里的想法，是把这个 SDE 作为 trajectory 的 motion prior。

这个 SDE 可以解析积分，得到状态 `x(t)` 关于初始时刻状态和过程噪声的表达。由于系统是线性且由 Gaussian noise 驱动，因此 `x(t)` 本身也是一个 `Gaussian Process`。在假设初始状态均值为零的简化情形下，可以进一步写出它的 covariance function，也就是 kernel function `K(t, t')`。

虽然这个 covariance 看上去比较复杂，但在所有 measurement times 上求值得到的 kernel matrix 可以整理成一个非常整洁的形式:

`K = Φ Q Φ^T`

更关键的是，我们真正需要的是 `K^{-1}`。由于 `Q` 是 block-diagonal，而 `Φ^{-1}` 只有主对角和次对角上有非零块，所以

`K^{-1} = Φ^{-T} Q^{-1} Φ^{-1}`

最终会变成 `block-tridiagonal` 结构。根据上一章关于稀疏信息矩阵与 factor graph 的讨论，这立刻意味着: 整条轨迹的 motion prior 虽然作用于所有时刻，但它完全可以由一张非常稀疏的 factor graph 表达出来，通常只包含一个起点 unary factor 和一串相邻状态之间的 binary factors。

这种稀疏性的根本原因，在于我们所选 SDE 的状态 `x(t)` 是 `Markovian`。从实践角度讲，不同的 motion prior 可能要求把状态扩充到更高阶。例如，如果希望使用 `constant-velocity prior`，那么状态就不仅包含 pose `p(t)`，还包含其导数 `v(t) = ṗ(t)`。这类做法常被称作 `simultaneous trajectory estimation and mapping (STEAM)`，可以看作 `SLAM` 的一种连续时间变体。

从统计学习的角度看，这套连续时间轨迹建模其实就是 `Gaussian Process regression`。因此，在 measurement times 上完成估计后，我们还可以利用 `GP interpolation` 在任意查询时刻高效恢复轨迹的均值与协方差；若采用本节这种稀疏 kernel 构造，每次查询的代价相对于测量数 `M` 是常数级的，因为只需要查询时刻两侧最近的已估计状态。

更进一步，GP interpolation 还能帮助我们减少 control points 的数量。比如，对一整帧 `LiDAR` 扫描，只放一个控制点，但仍然利用该扫描中每一个点的真实时间戳。于是，连续时间框架下的 `measurement times`、`estimation times` 和 `query times` 可以完全分离。与参数化 spline 方法不同的是，在 GP 框架里，即便设置较多 estimation times，也不会像样条那样直接导致过拟合，因为 kernel 本身已经承担了正则化作用；当然，控制点仍需要足够密，才能表达轨迹细节。

### 2.2.4 Lie 群上的样条与 GPs

无论是 splines 还是 GP continuous-time methods，都可以推广到状态位于 manifold 上的情形。若这些 manifolds 是 `Lie groups`，两类方法都会借助 `Lie algebra` 完成推广，只是切入方式不同。作者先解释 splines，再讨论 GPs。

### 2.2.4.1 Lie 群上的样条

让 splines 适用于 `Lie groups` 的关键，是采用 `cumulative formulation`。在线性向量空间中，如果把所有自由度都用同一组标量 basis functions `ψk(t)` 参数化，那么原始 spline 表达

`p(t) = Σ_k ψk(t) pk`

可以改写成累计形式:

`p(t) = ψ1^c(t) p1 + Σ_{k=2}^K ψk^c(t) (pk - p_{k-1})`

其中 `ψk^c(t)` 是 cumulative basis functions，也就是从第 `k` 个 basis 往后的累加。对于均匀时间间隔下的线性样条，这套 cumulative basis functions 在任意时刻只会激活极少数项。比如在线性样条中，如果 `(k - 1)T ≤ t < kT`，就可以写成

`p(t) = p_{k-1} + ψk^c(t) (pk - p_{k-1})`

此时只需评估一个 basis function。

把这个思想搬到 `Lie groups` 上时，向量加法不再可用，因此要用 group 运算，也就是矩阵乘法，来替代求和。对 `SE(d)` 的线性样条，插值可以写成

`T(t) = Exp(ψk^c(t) Log(Tk T_{k-1}^{-1})) · T_{k-1}`

也就是说，我们不是在线性空间里插值，而是在相邻 control-point poses 之间，沿 `Lie algebra` 中定义的“相对位姿”去做插值。

一旦有了 `T(t)` 的表达，就可以把某个 measurement time `ti` 上的插值 pose 直接代入测量模型，并对 control points `Tk` 做线性化。由于 cumulative basis functions 依然具有稀疏激活性，因此每条测量只会关联极少数控制点。

对线性样条而言，上式还能改写成

`T(t) = (Tk T_{k-1}^{-1})^{αk(t)} T_{k-1}`

当对涉及 `T(t)` 的表达做线性化时，可以沿用 Section `2.1.3` 中的优化方法。这里作者选择在左侧进行 perturbation，并指出右侧 perturbation 也可以采用类似构造。于是有

`Exp(ξ(t)) T^0(t) = Exp(ξk) T_k^0 T_{k-1}^{0,-1} Exp(-ξ_{k-1})^{αk(t)} Exp(ξ_{k-1}) T_{k-1}^0`

作者的目标，是把插值 pose 的 perturbation `ξ(t)` 与两侧 control-point poses 的 perturbations `ξ_{k-1}` 和 `ξk` 联系起来。其一阶近似结果为

`ξ(t) ≈ (I - A(αk(t))) ξ_{k-1} + A(αk(t)) ξk`

其中

`A(αk(t)) = αk(t) J(αk(t) Log(T_k^0 T_{k-1}^{0,-1})) J(Log(T_k^0 T_{k-1}^{0,-1}))^{-1}`

这里 `J(·)` 是 `SE(d)` 的左 `Jacobian`。有了这个关系，就能把测量时刻的 pose 变化映射到两侧 bracketing control-point poses 上。

例如，对式 `(2.16)` 的线性化 measurement model，若把它改写为关于 pose 与 perturbation 的误差形式，则有

`e_i(t) ≈ z_i - h(T_i^0(t) l_i) - H_i ξ(t)`

再把上式中的 `ξ(t)` 用 `(2.64)` 代入，就得到一个直接关于相邻两个 control-point poses 的线性化误差项：

`e_i(t) ≈ z_i - h(T_i^0(t) l_i) - H_i (I - A(αk(t))) ξ_{k-1} - H_i A(αk(t)) ξk`

作者特别提醒，此时还需要把 nominal pose 也替换为 `T^0(t) = (T_k^0 T_{k-1}^{0,-1})^{αk(t)} T_{k-1}^0` 后再带入 `h` 和 `H_i`。本质上，这一步就是把 spline 插值的导数链式传到相邻两个 control points 上。高阶样条也可以按同样思路推广，只是活跃控制点数量会更多一些。

### 2.2.4.2 Lie 群上的 Gaussian Processes

要把 `Gaussian Processes (GPs)` 用到 `Lie group` 上，我们同样要借助它的 `Lie algebra`。图 `2.6` 先给出了本节思想的一个直观预告: 由这些构造得到的 `GP motion-prior factors` 会直接进入连续时间估计的图模型中。作者也提醒，与向量空间情形一样，具体的 control-point state 还可能包含额外的轨迹导数，这取决于一开始选定的 motion prior。

在 `Lie group` 上使用 `GP` 的做法，是像 spline 一样，在一组 control-point states 之间定义局部变量。图 `2.7` 用 `SE(d)` 给出了示意。设相邻控制点为 `T_k` 与 `T_{k+1}`，则可定义局部变量

```text
ξ_k(t_k) = 0
ξ_k(t) = Log(T(t) T_k^(-1))
ξ_k(t_{k+1}) = Log(T_{k+1} T_k^(-1))
```

也就是说，用来导出 kernel function 的 `SDE` 不再直接作用于全局位姿，而是作用于这些局部变量。以 `SE(d)` 上的 `random-walk prior` 为例，可选取如下 `SDE`：

```text
ξ̇_k(t) = w(t),   w(t) = GP(0, Q δ(t - t'))
```

这里需要注意，`SDE` 是用 control points `T_k` 与 `T_{k+1}` 之间的局部变量来定义的。对这个 `SDE`，其状态转移函数就是 `Φ(t, s) = I`，因此做随机积分后有

```text
ξ_k(t) = ξ_k(t_k) + ∫_(t_k)^t w(s) ds
```

进一步取均值与协方差后，就得到对应的 motion prior

```text
ξ_k(t) ~ GP(0, min(t, t') Q)
```

若将 control-point poses 按每隔 `T` 秒均匀放置，则对应的 inverse kernel matrix 可写成

```text
K^(-1) = Φ^(-1) Q^(-1) Φ^(-T)
```

其中

```text
Φ^(-1) = [ I           ]
         [ -I   I      ]
         [      ..  .. ]
         [         -I I]

Q = diag(K(t_1, t_1), TQ, ..., TQ)
```

这说明即使使用 `GP`，其逆核结构依然非常稀疏。作者随后给出了每一段误差项在局部变量中的写法：

```text
e_k = Log(T̄_1 T_1^(-1)),                 k = 1
e_k = ξ_(k-1)(t_k) - ξ_(k-1)(t_(k-1)),   k > 1
```

其中 `T̄_1` 表示某个先验初始位姿。在全局变量中，同样的误差可写为

```text
e_k = Log(T̄_1 T_1^(-1)),   k = 1
e_k = Log(T_k T_(k-1)^(-1)), k > 1
```

图 `2.6` 展示了这种 `random-walk GP motion prior` 在 factor graph 中的样子。与上一节讨论线性 spline 时一样，如果我们想在其他时刻查询轨迹，也可以使用 `GP interpolation`。对于 `random-walk prior`，最终结果再次是线性插值 [47]：

```text
T(t) = (T_k T_(k-1)^(-1))^(α_k(t)) T_(k-1)
```

其中

```text
α_k(t) = (t - (k - 1)T) / T,   (k - 1)T ≤ t < kT
```

这里与 spline 方法的一个关键区别是，这种线性插值并不是通过显式选择某个 spline basis 人为指定出来的，而是由一开始所选的 `SDE` 间接导出的。若最初采用更高阶的 `SDE`，那么插值就会自然对应更高阶的 splines。

最后一个需要解决的问题，是如何把这些误差项线性化，以用于 `MAP estimation`。作者再次使用前面介绍过的 `Lie group perturbation` 方法。以式 `(2.73)` 的第二种情况为例，可写成

```text
e_k = Log( Exp(ξ_k) T_k^0 T_(k-1)^0^(-1) Exp(-ξ_(k-1)) )
    ≈ Log(T_k^0 T_(k-1)^0^(-1)) + ξ_k - Ad_(T_k^0 T_(k-1)^0^(-1)) ξ_(k-1)
```

其中 `T_k^0` 与 `T_(k-1)^0` 是当前估计，`ξ_k` 和 `ξ_(k-1)` 是待求的 perturbations，`Ad(·)` 则是 `SE(d)` 上的 adjoint。这个线性化形式就能在每次迭代中直接插入标准的 `MAP` 估计框架。

另外，如果在这个 `random-walk` 例子中，我们希望用式 `(2.74)` 减少 control points 数量，那么也可以沿用前面线性 spline 的同一套做法，因为这两种方法最终都退化成 `SE(d)` control points 之间的线性插值。作者最后总结说，spline 与 `GP` 在 `Lie group` 上做连续时间估计时，最大的区别在于: `GP` 方法会显式引入 motion-prior terms 来正则化问题，而 spline 方法本身并不会自动提供这种正则项。

## 2.3 延伸阅读与最新趋势

关于 manifold，尤其是 `Lie groups` 上的状态估计，已经有非常丰富的文献。机器人学里最具代表性的基础参考之一，是 Chirikjian 的系列工作。这些书在理论上扎实而系统，讨论了如何在 manifolds 上处理不确定性，甚至包括全局层面的建模。若想更深入理解 manifold optimization，Boumal 的著作是非常好的参考，且特别照顾到机器人应用语境。对于本章这种“局部不确定性 + `Lie group` 优化”的处理方式，Barfoot 的书则给出了更贴近工程实践的系统阐述。

连续时间估计方面，Talbot 等人的综述系统梳理了参数化与非参数化方法；Barfoot 的材料也对非参数方法做了较为详细的讨论。

与上一章类似，本章中的许多主题其实都会在后续章节继续出现。例如，如何在 manifold 上处理离群点、如何把算法变成可微的、以及如何把这些状态表示工具用于不同类型的传感器，都会在后文中被进一步展开。因此，本章可以视为现代 `SLAM` 状态表示与轨迹建模工具箱的基础准备。

## 第 3 章 面向错误数据关联与离群点的鲁棒性

### 本章概览

第一章已经说明，`factor graphs` 与 `maximum a posteriori (MAP)` estimation 为 `SLAM` 提供了统一而扎实的推断框架。在最理想的设置下，若测量噪声满足加性零均值 Gaussian 假设，那么 `MAP` 就会落到标准的非线性最小二乘问题上。

本章讨论一个现实里几乎不可避免的问题: `outliers`。在实际 `SLAM` 中，许多测量可能由于错误数据关联（incorrect data association）、感知混淆（perceptual aliasing）、动态物体、传感器退化等原因而偏离 Gaussian 假设，这会严重破坏最小二乘估计。于是，本章一方面讨论如何在 `SLAM front-end` 中识别并剔除 gross outliers；另一方面讨论在仍有残留 outliers 的前提下，如何增强 `SLAM back-end` 的鲁棒性。

## 3.1 离群点为何产生，以及它们为何构成问题？

本节的核心观点是: 在大多数 `SLAM` 应用里，outliers 几乎不可完全避免；若不做恰当处理，它们会导致估计结果出现灾难性偏差。

### 3.1.1 数据关联与离群点

理解 outliers 的最好方式，是先看数据关联问题。在 landmark-based `SLAM` 中，机器人需要根据 odometry 与对 landmarks 的相对观测，同时恢复自身轨迹和外部 landmarks 的位置。如果这些观测被建模为零均值 Gaussian，那么每一条观测都会对应一个标准的误差项。

但在真实系统里，这些观测通常不是“直接给定”的，而是由 `SLAM front-end` 从原始传感器数据中提取出来的。例如在视觉系统中，某个 landmark bearing measurement 往往来自目标或特征检测、图像匹配以及像素反投影。问题在于，检测与匹配并不完美，本该对应 landmark `ℓj` 的观测，可能实际匹配到了别的 landmark。这样一来，该测量就不再满足原先的几何模型，从而成为 outlier。把观测错误地关联到某个 landmark 上，这就是典型的 `data association problem`。

同样的问题也出现在 `pose-graph optimization` 中。此时系统关注的重点是机器人轨迹，测量通常包括相邻 poses 之间的 odometry，以及非相邻 poses 之间的 `loop closures`。在实践中，loop closure 依赖 place recognition 方法检测“两个位姿是否看到同一地点”。但 place recognition 很容易受限于方法本身的不足，也会受到 `perceptual aliasing` 的影响，也就是两个不同地点看起来很像，例如相似的走廊、教室或办公室隔间。于是，本不该相连的两个 poses 被错误地添加 loop closure，也就形成了 outlier。

除了错误数据关联，outliers 也可能来自建模假设本身被破坏。例如许多 `SLAM` 方法默认 landmarks 是静态的，因此即便对动态物体的观测在识别上是“正确”的，它也可能在后端里表现为大残差 outlier。再比如，轮编码器故障、镜头被灰尘污染、激光雷达退化等传感器失效或退化，也都会制造 outliers。

### 3.1.2 存在离群点时的最小二乘

在存在 outliers 时，标准 least-squares formulation 往往会给出极其错误的解。理论上，Gaussian noise 是一种 `light-tailed` 噪声模型，它基本排除了“测量误差非常大”的情况；但 outliers 恰恰对应这类极大误差。

从优化角度看，outlier 会导致某些 residual `ri(x)` 在真实解附近也非常大。由于 least-squares objective 是把 residual 平方再求和，大残差项会被平方放大，于是优化器会把大量注意力放在“解释 outliers”上，而不是更好利用其余 inlier measurements。结果是，少量坏测量就足以把整条轨迹或整张地图拉偏。

书中的实验例子表明，无论是模拟的 `M3500` pose-graph benchmark、真实的 `SubT` 地下隧道数据集，还是 `Victoria Park` landmark-SLAM 数据集，只要混入一定比例的 outliers，普通 least squares 都会产生严重错误的轨迹与地图。甚至环境中的感知混淆还会通过这些错误测量被“固化”为错误地图结构。

## 3.2 在 SLAM 前端中检测并剔除离群点

`SLAM front-end` 的主要职责，是把原始传感器数据处理成可供后端使用的中间表示或伪测量。典型 front-end 通常先生成一批候选 measurements，其中往往含有不少 outliers；然后再通过额外的筛选步骤，把 gross outliers 在进入 `back-end` 之前剔除掉。

本节介绍两类经典 front-end outlier rejection 思路: `RANSAC`，以及基于图论的 `Pairwise Consistency Maximization (PCM)`。

### 3.2.1 RANdom SAmple Consensus（RANSAC）

`RANSAC` 是最成熟、最常见的 outlier rejection 工具之一，也是许多 landmark-based `SLAM` 系统前端中的关键组成部分。为了理解它在 `SLAM` 里的作用，书中以 feature-based visual `SLAM` 为例: 系统在每一帧图像中提取 `2D` 特征点，再通过 optical-flow tracking 或 descriptor matching，把当前帧与前一帧中的像素配对，形成 `2D-2D correspondences`。由于跟踪和匹配本身会出错，这一初始对应集里天然会混入 outliers，因此在把这些 correspondences 送入后端之前，必须先尽量剔除明显错误的匹配。

`RANSAC` 依赖两个核心观察。第一个观察是，在 `SLAM` 问题里，正确的 inlier correspondences 必须满足几何约束。以上述视觉例子为例，若匹配像素来自同一个静态 `3D` 点，那么它们在两帧中的运动不可能任意，而必须满足 `epipolar constraint`。对标定好的相机而言，若 `z_i(k-1)` 和 `z_i(k)` 表示地标 `i` 在时刻 `k-1` 与 `k` 的像素坐标，则有

`z_i(k-1)^T [t_k^{k-1}]_x R_k^{k-1} z_i(k) = 0`

其中 `t_k^{k-1}` 与 `R_k^{k-1}` 是两帧相机之间未知的相对平移与相对旋转。更一般地，这类约束可写成

`C(z_i, x) ≤ γ`

这里 `x` 是局部状态变量，`γ` 是为噪声预留的容忍阈值。也就是说，inliers 必须对同一个状态 `x` 保持一致。

第二个观察是，只要 outliers 的比例没有高到失控，我们就可以把 inliers 理解成“在同一个状态 `x` 下满足几何约束的最大 measurement subset”。这就对应计算机视觉中的 `consensus maximization` 问题: 在所有候选 correspondences `M` 中，寻找最大的子集 `S`，使得其中所有观测都与同一个 `x` 相容。该问题本质上是组合优化，直接精确求解代价太高，因此前端通常不会真的去穷举所有子集。

`RANSAC` 的关键假设是，状态 `x` 的维度相对较低，并且可以由一个很小的 measurement subset 决定，也就是所谓的 `minimal set`；同时还存在能从这个最小子集快速恢复状态的 `minimal solver`。在前面的视觉例子里，两帧标定相机之间的相对运动可以仅由 `5` 个像素对应通过 Nister 的 `5-point method` 估计出来。因此，`RANSAC` 不去穷举所有 `S ⊂ M`，而是反复随机采样 minimal sets 来寻找 inliers。

标准 `RANSAC` 的执行流程是:

1. 随机采样 `n` 个 correspondences，其中 `n` 是该问题 minimal set 的大小。
2. 通过 `minimal solver` 从这 `n` 个样本中计算一个状态估计 `x̂`。
3. 检查全部候选 measurements 中有哪些满足 `C(z_i, x̂) ≤ γ`，把它们组成 `consensus set` `S`。
4. 若当前 `S` 比历史最好结果更大，则保存该 `S`。
5. 当找到足够大的 `consensus set`，或达到最大迭代次数时停止。

`RANSAC` 的本质，就是试图在采样时恰好抽到全由 inliers 构成的 minimal set；一旦采到这样的子集，估计出的 `x̂` 往往就会与大量其他 inliers 一致，从而形成较大的 `consensus set`。这也是它在很多 `SLAM` 前端里如此有效的原因。

它的效率高度依赖两个条件:

- inlier 比例较高
- minimal set 足够小

若随机采到一个 measurement 为 inlier 的概率是 `ω`，那么抽到一个全为 inliers 的 minimal set 的概率就是 `ω^n`，相应所需的期望迭代次数约为 `1 / ω^n`。书中举例指出，当 `n = 5` 且 `ω = 0.7` 时，期望迭代次数不到 `10` 次；再加上许多 `minimal solvers` 在实践中极快，因此 `RANSAC` 往往能在毫秒级给出可用解，并同时提供一个有价值的后端初值。

但它并不适合所有问题。若 inlier 比例很低，或者 minimal set 很大，则所需采样次数会迅速失控。例如当 `n = 10`、`ω = 0.1` 时，期望迭代次数达到 `10^10` 量级，此时在远少于该次数时提前终止，往往会返回错误的状态估计与错误对应。对某些 `SLAM` 问题，例如节点数为 `N` 的 `pose-graph SLAM`，最小测量集至少要包含 `N-1` 条边才能形成 spanning tree，而 `N` 通常以千计，这时 `RANSAC` 就不再现实了。

### 3.2.2 基于图论的离群点剔除与成对一致性最大化

上一节已经说明，`RANSAC` 在 outlier 比例适中且 minimal set 很小时非常有效；但在 perceptual aliasing 很严重的环境中，outliers 可能远超 `70%`，而且很多 `SLAM` 问题并不存在“小而快”的 minimal solver。以 `pose-graph SLAM` 为例，若图中有 `N` 个节点，则 minimal set 至少要包含 `N-1` 条测量形成 spanning tree，而 `N` 往往是成千上万。针对这类情形，书中引入了另一条路线: `Pairwise Consistency Maximization (PCM)`。

`PCM` 的出发点不是随机采样最小子集，而是寻找“彼此内部一致”的最大 measurement 集合。其关键在于，很多问题都能定义一种 `pairwise consistency function`，用来判断两条 measurements 是否彼此兼容。与 `RANSAC` 使用的几何约束不同，这类一致性函数通常不依赖未知状态变量，因此可以只看测量本身就完成判断。

书中给出两个例子。第一个例子来自带 `RGB-D` 相机的 landmark-based visual `SLAM`。若两组 `3D-3D correspondences` 都是正确的，那么在两帧中，对应点对之间的距离应该保持不变，因此对两个对应 `i`、`j` 应满足

`| ||z_i(k-1) - z_j(k-1)|| - ||z_i(k) - z_j(k)|| | ≤ γ`

这个约束不需要先求出相机运动，就能直接判断两条 correspondences 是否相容。第二个例子来自 `pose-graph SLAM`。若两条 loop closures 都正确，那么它们与中间的 odometry 链围成闭环后，整体复合应接近单位位姿，也就是说，闭环变换与 `identity` 之间的距离应小于某个阈值。

更一般地，对任意两条 measurements `z_i` 与 `z_j`，一致性约束可写成

`F(z_i, z_j) ≤ γ`

其中 `F` 是 pairwise consistency function，`γ` 用于吸收噪声影响。基于这一约束，outlier rejection 可以被重写为: 在候选测量集 `M` 中，找一个最大的子集 `S`，使得其中任意两条 measurements 都满足 pairwise consistency。书中把这个问题写成

`S*_PCM = argmax |S|, s.t. F(z_i, z_j) ≤ γ, ∀ i, j ∈ S`

这就是 `Pairwise Consistency Maximization (PCM)`。

与 `RANSAC` 相比，`PCM` 的好处在于，它不再需要处理未知状态 `x`，也不需要 `minimal solver`；所有成对一致性关系都可以预先计算出来。更重要的是，它有一个非常自然的图论解释。我们构建一个 `consistency graph`:

- 每个 putative measurement 对应一个节点。
- 若两条 measurements 满足 `F(z_i, z_j) ≤ γ`，就在节点 `i` 与 `j` 之间连一条边。

这样一来，`PCM` 就等价于寻找图中最大的 `clique`，也就是一个节点子集，其中任意两点之间都有边相连。换句话说，`PCM` 的解恰好就是 consistency graph 的 `maximum clique`。这使得离群点剔除问题可以借助成熟的图论工具来处理。

基于 maximum clique 的 `PCM` 流程通常包括:

1. 针对当前问题选择合适的一致性函数 `F`。
2. 对每一对候选 measurements `(i, j)` 评估 `F(z_i, z_j)`，并据此建立 consistency graph。
3. 使用精确或近似的 maximum clique 算法求解该图。
4. 返回得到的 clique 中的 measurements 作为 inliers。

书中特别提醒，一致性函数的设计高度依赖问题本身，而且会显著影响最终效果。若 `F` 设计得过弱，例如无论输入都返回一致，那么 `PCM` 就无法剔除任何 outliers；若 `F` 设计得过严，又可能误删本该保留的 inliers。此外，尽管 `PCM` 比带状态变量的 `consensus maximization` 更容易处理，`maximum clique` 本身仍然是 `NP-hard` 的，逼近也很困难。实践中既有精确算法，也有利用图稀疏结构、并行化或启发式规则的近似算法。因此，`PCM` 并不是“免费午餐”，但在高 outlier 比例和大 minimal-set 问题下，它往往比 `RANSAC` 更合适。

## 3.3 提升 SLAM Back-end 对离群点的鲁棒性

即便 front-end 使用了 `RANSAC` 或 `PCM`，它们也未必能把所有 outliers 都完全剔除。少量残留 outliers 进入后端后，仍然可能把标准 least-squares 解严重拖偏。因此，`SLAM back-end` 也必须具备一定的 outlier robustness。

正如第 `3.1.2` 节所示，平方残差会“放大”离群测量对代价函数的影响。本节因此沿着 robust statistics 中标准的 `M-estimation` 理论，讨论如何稍微修改 `SLAM` 优化目标，使其重新获得对 outliers 的鲁棒性。

`M-estimation`（`Maximum-likelihood-type Estimation`）的核心做法，是把式 `(3.1)` 中的平方损失替换为某个合适的 robust loss `ρ`:

`x_MLE^MAP = argmin_x Σ_i r_i^2(x)   =>   x_MEST = argmin_x Σ_i ρ(r_i(x))`  。`(3.9)`

对 robust loss `ρ` 的关键要求，是它非负，并且在残差较大时增长速度低于二次函数。换言之，当 `r_i` 变大时，需要有 `∂ρ(r_i)/∂r_i << ∂||r_i||^2/∂r_i = 2r_i`；在很多情形下，最好 `∂ρ(r_i)/∂r_i` 还能逐渐趋近于 `0`。若我们用 gradient descent 去求解 `min_x Σ_i ρ(r_i(x))`，根据链式法则，其目标函数 `f(x) = Σ_i ρ(r_i(x))` 的梯度为

`∂f/∂x = Σ_i (∂ρ(r_i)/∂r_i) · (∂r_i(x)/∂x)`  。`(3.10)`

从式 `(3.10)` 可以直接看出: 只要初始化足够接近真实解，outlier 往往会产生很大的 residual，因此对应的 `∂ρ(r_i)/∂r_i` 会很小，从而对整体下降方向只有很弱的影响。函数 `ψ(r_i) := ∂ρ(r_i)/∂r_i` 也因此被称为 `influence function`。

原书还特别给出了一组常见 robust losses 的“菜单”，Figure `3.5` 中包括 `Huber`、`Geman-McClure`、`Tukey's biweight`、`truncated quadratic loss`，以及更激进的 `maximum consensus loss`。后者虽然通常不被列为经典 robust loss，但书中把它一并列出，是为了和前面讨论的 consensus maximization `(3.4)` 建立联系。不同 robust loss 的选择高度依赖具体问题。例如，当已知 inlier 最大误差阈值时，像 `truncated quadratic loss` 这种带 hard cut-off 的损失就很自然；而在 `BA` 问题中，`Huber` 由于保持凸性、优化行为更稳定，因此也很常用，尽管它对 outliers 仍保留非零 influence。相反，`truncated quadratic` 和 `maximum consensus` 对 outliers 更不敏感，但通常需要更 ad-hoc 的求解器。

Figure `3.6(g)-(l)` 则展示了把 gradient descent 应用于 `Huber loss` 与 `truncated quadratic loss` 时，在 `M3500`、`SubT` 和 `Victoria Park` 数据集上的结果。与非鲁棒 least squares（Figure `3.1(d)-(f)`）相比，robust losses 在 `M3500` 和 `SubT` 上很快就恢复了对 outliers 的鲁棒性；但对高度非凸的 `truncated quadratic cost`，简单的 gradient descent 在 `Victoria Park` 上仍可能失败。书中也指出，虽然 gradient descent 已经能在很多实例上提升性能，但它往往需要数千次迭代才收敛，存在明显的 slow convergence tail。因此，后文会转向质量和速度都更好的求解器。

在进入更高级求解器前，作者还补充了一点: 使用 robust losses 并不意味着放弃 probabilistic framework。事实上，若对测量噪声采用 heavy-tailed noise distributions 做 `MAP estimation`，就能推导出若干经典 robust losses。例如，`truncated quadratic loss` 可以由一种在 Gaussian inlier density 与 uniform outlier density 之间的 `max-mixture distribution` 推出。

### 3.3.1 迭代重加权最小二乘

`M-estimation` 的核心思想，是把 least-squares objective 中的平方损失替换为某个增长速度低于二次的 `robust loss` `ρ`。它的目的，是让极大的 residual 不再对总损失产生不成比例的支配作用。对于一个好的 robust loss，它的导数，也就是 `influence function`，在 residual 很大时应当显著减弱，理想情况下甚至趋近于 `0`。

这会带来两项代价。第一，我们失去了原本已为 least-squares formulations 发展成熟的高效求解器，例如 `Gauss-Newton` 和 `Levenberg-Marquardt` 都是围绕 least squares 设计的；第二，由于 `M-estimation` 往往具有非凸地形，基于 gradient descent 的迭代求解对初始化十分敏感，也常常收敛到不理想的次优解。

`Iteratively Reweighted Least Squares (IRLS)` 的目标，就是在保留鲁棒目标的同时，重新利用高效 least-squares solver。其基本思想是在每次迭代中求解一个 weighted least-squares 子问题:

`x^(t+1) = argmin_x Σ_i w_i^(t)(x^(t)) r_i^2(x)`  。`(3.11)`

其中各个权重 `w_i` 依赖于上一轮估计 `x^(t)`。我们希望由此产生的迭代序列 `x^(t)` 最终收敛到 `M-estimation (3.9)` 的最优解。为实现这一点，需要让 robust loss `(3.10)` 的梯度，与加权 least squares `(3.11)` 的梯度相匹配。将 `(3.11)` 的梯度写出并与 `(3.10)` 对比，可得 `IRLS` 的权重更新规则:

`w_i^(t)(x^(t)) = [1 / (2 r_i(x^(t)))] · [∂ρ(r_i(x^(t))) / ∂r_i(x^(t))] = ψ(r_i(x^(t))) / (2 r_i(x^(t)))`  。`(3.12)`

这里仍记 `ψ(r_i) := ∂ρ(r_i) / ∂r_i` 为 influence function。因此，`IRLS` 实际上是在两步之间交替进行:

- residual 小，权重大
- residual 大，权重小

也就是先按当前权重做一次 `Gauss-Newton` 或 `Levenberg-Marquardt` 的 weighted least-squares 更新，再根据新的 residual 重新计算权重。这样既能继续使用成熟 least-squares solver，又能逐步逼近 robust objective 的最优解。

书中的 Figure `3.7` 展示了 `IRLS` 在 `M3500`、`SubT` 和 `Victoria Park` 数据集上的性能。与 gradient descent 相比，`IRLS` 往往只需几十次迭代即可收敛，速度通常显著更快。例如，在书中的 `M3500` 实验里，用 gradient descent 优化 `Huber loss` 大约需要 `5` 秒，而 `IRLS` 不到 `1.5` 秒。不过，这种更快的收敛有时会以精度略降为代价；同时，式 `(3.12)` 的收敛性质仍然与初始化质量密切相关。

### 3.3.2 Black-Rangarajan 对偶

前面给出的 `IRLS` 权重更新规则 `(3.12)` 在实践中很常用，但它的推导多少带有启发式色彩，而且在 `ρ` 的非光滑点上并不总是定义良好，例如 `truncated quadratic loss` 的截断点。为此，作者引入了一个更有原则的框架，即 `Black-Rangarajan (B-R) duality` [85]，用来从更系统的角度理解并求解 `M-estimation`。

作者先用 `truncated quadratic loss` 来说明其直觉。对第 `i` 个残差，定义

`ρ(r_i(x)) := min{ r_i^2(x), β_i^2 }`                                            `(3.13)`

其中 `β_i^2` 是第 `i` 个残差的上界: 当 `r_i^2(x) ≤ β_i^2` 时，测量被视为 inlier；否则视为 outlier。关键观察是，这个 robust cost 可以通过引入一个新权重变量 `w_i ∈ [0, 1]`，重写成两项之和:

`ρ(r_i(x)) := min_(w_i ∈ [0,1])  w_i r_i^2(x) + (1 - w_i) β_i^2`                 `(3.14)`

这里第一项正是 weighted least squares，第二项则只依赖于 `w_i`，与状态变量 `x` 无关。于是，带 `truncated quadratic loss` 的 `M-estimation` 问题 `(3.9)` 就可以改写为

`min_(x, w_i ∈ [0,1]) Σ_i [ w_i r_i^2(x) + (1 - w_i) β_i^2 ]`                    `(3.15)`

这个形式很容易解释: 当 `w_i = 1` 时，意味着 `r_i^2(x) ≤ β_i^2`，该测量被视为 inlier；当 `w_i = 0` 时，意味着 `r_i^2(x) > β_i^2`，该测量被视为 outlier，并在优化中被有效丢弃。由此，robust estimation 不再只是“换一种损失函数”，而是变成了“同时估计状态变量 `x` 与各测量是否可信”。

`B-R duality` 的价值在于，它可以把上述思路推广到更一般的 robust losses。书中给出如下定理。

`Theorem 3.4 (Black-Rangarajan Duality [85])`:
给定 robust loss `ρ(·)`，定义 `ϕ(z) := ρ(√z)`。若 `ϕ(z)` 满足

- `lim_(z→0) ϕ'(z) = 1`
- `lim_(z→∞) ϕ'(z) = 0`
- `ϕ''(z) < 0`

则 `M-estimation` 问题 `(3.9)` 等价于

`min_(x, w_i ∈ [0,1]) Σ_i [ w_i r_i^2(x) + Φ_ρ(w_i) ]`                           `(3.16)`

其中 `w_i ∈ [0,1]` 是与每个残差 `r_i` 关联的权重变量，而 `Φ_ρ(w_i)` 被称为 `outlier process`，它是作用在权重 `w_i` 上的一个惩罚项，其具体形式取决于所选 robust loss `ρ`。

对 `truncated quadratic loss`，从 `(3.14)` 立刻就能看出 `Φ_ρ(w_i) = (1 - w_i) β_i^2`。而对其他 robust loss，则需要按 [85] 提供的步骤推导 `Φ_ρ(w_i)`。作者接着给出 `Geman-McClure (G-M)` loss 的例子。

`Example 3.5 (B-R Duality for G-M Loss)`:
考虑 `Geman-McClure` robust loss

`ρ(r_i(x)) = β_i^2 r_i^2(x) / (β_i^2 + r_i^2(x))`                                 `(3.17)`

其中 `β_i^2` 与前面一样，是第 `i` 个残差的噪声界。与它对应的 `outlier process` 为

`Φ_ρ(w_i) = β_i^2 ( √w_i - 1 )^2`                                                  `(3.18)`

为验证 `(3.18)`，考虑子问题

`min_(w_i ∈ [0,1])  w_i r_i^2(x) + Φ_ρ(w_i)`                                       `(3.19)`

对 `(3.19)` 求梯度并令其为零，可得最优解

`w_i* = ( β_i^2 / (r_i^2(x) + β_i^2) )^2`                                          `(3.20)`

把 `(3.20)` 代回 `(3.19)` 的目标函数，就能恢复 `G-M` robust loss `(3.17)`。这说明 `Geman-McClure` 同样可以被写成“weighted least squares + outlier process”的形式。

因此，`Black-Rangarajan duality` 提供了一个统一框架: 各种 robust losses 都可以通过显式引入权重变量和相应的 `outlier process`，被转化为一类具有共同结构的优化问题。这也为下一节把 `IRLS` 理解成 `alternating minimization` 奠定了基础。

### 3.3.3 交替最小化

有了 `Black-Rangarajan duality` 之后，`IRLS` 自然可以被理解为一种 `alternating minimization`。

直观地说，联合优化状态变量 `x` 和所有权重 `wi` 往往困难；但如果固定其中一部分，优化另一部分就容易得多:

- 固定 `wi` 时，问题退化成 weighted least squares
- 固定 `x` 时，每个 `wi` 都只需独立解决一个标量优化问题

因此，算法每一轮在两步之间交替:

1. 用当前权重求解状态变量
2. 用当前状态更新各 measurement 的权重

这种理解也解释了为何在某些 `SLAM` 问题中，用 `Geman-McClure` loss 的 `IRLS` 会与已有的动态协方差缩放（dynamic covariance scaling）方法高度一致。

### 3.3.4 渐进非凸化

尽管 `IRLS` 比朴素 gradient descent 更高效，但 robust losses 的非凸性依然让它对初始化敏感。哪怕 outlier 比例不高，`IRLS` 也可能掉入坏的局部极小值。`Graduated Non-Convexity (GNC)` 正是用来缓解这个问题的。

`GNC` 的思想是，不要一开始就直接优化原始的强非凸 robust loss，而是先构造一个“更平滑、更接近凸”的版本 `ρμ`，由一个控制参数 `μ` 决定其非凸程度。优化开始时用平滑版本，让问题更容易收敛；随着迭代推进，再逐步调整 `μ`，让 `ρμ` 慢慢逼近原始的 robust loss。

对于 `truncated quadratic` 和 `Geman-McClure`，书中都给出了相应的 `GNC` 版本。重要的是，这些平滑后的损失同样可以套用 `Black-Rangarajan duality`，因此 `GNC` 实际上仍能以“状态更新 + 权重更新”的 `IRLS` 风格执行，只是额外多了一步去更新控制参数 `μ`。

实验显示，`GNC` 在 `pose-graph optimization` 这类问题上通常显著优于直接 `IRLS` 或 gradient descent，更不容易卡在坏局部极小值，对高比例 loop closure outliers 也更稳健。不过，`GNC` 并没有一般性的全局收敛保证，其表现依然与具体问题结构密切相关。

本节最后还强调了一个现实点: 当前端和后端都可能受污染时，例如 odometry 自身也含 outliers，问题会变得极其困难。此时，单用 `GNC`、单用 `PCM`，甚至两者组合，都可能在某些数据集上失败。这说明 outlier-robust `SLAM` 依然是一个没有被彻底解决的问题。

## 3.4 延伸阅读与最新趋势

作者首先回顾 `consensus maximization` 的后续发展。虽然本章主要讨论的是最基础的 `RANSAC` 形式，也就是最初提案中的经典版本，但文献中其实已经发展出大量变体。例如，有些方法会在 `RANSAC` 迭代内部进一步做局部优化；有些方法使用比 `consensus set` 大小更复杂的评分准则，例如 `MLESAC`；也有方法在采样阶段引入偏置，例如 `PROSAC`。近年的研究还包括可微 `RANSAC`，以及在阈值 `γ` 未知时主动估计 inliers 的方法。若想系统性了解这些变体，作者建议参考相应的综述与基准评测工作。

除了 `RANSAC`，文献中还出现了精确求解 `consensus maximization` 的方法，通常基于 `branch-and-bound`。它们能够提供全局最优保证，但最坏情况下复杂度仍是指数级，因此不适用于高维大规模问题。

接着，作者回顾 `Pairwise Consistency Maximization (PCM)` 的后续工作。`PCM` 最初是在多机器人 `SLAM` 背景下提出的，在那里显示出了很大潜力。类似的 graph-theoretic outlier rejection 思想，在计算机视觉中也有丰富发展。例如，有方法通过构造 association graph 并求 maximum clique 来做 `2D image feature matching`；也有工作用 clique 表述做刚体运动分割，或者通过 correspondence graph 中的强连通聚类来建立图像匹配；还有研究针对 `3D-3D` 和 `2D-3D` 配准，使用 approximate vertex cover 或 maximum clique 来做离群点剔除。书中还提到，基于 measurement clustering 并用 `Chi-squared` 一致性检验做 loop closure rejection 的工作，与这里“在 measurement 子集上检查一致性”的思想也是相通的。

在 `PCM` 自身的发展上，较新的工作已把两两一致性的图结构，推广成 `group-k consistency`，也就是把一致性从“边”推广到包含 `k` 个节点的 `hyper-edge`，从而形成 consistency hypergraph。相关研究还考虑了 maximum clique 的软化版本，也就是把原本二值的边一致性条件放松成连续权重，形成所谓的“软最大团”思路。这些 graph-theoretic 方法已经在地下探索、`LiDAR point-cloud localization`、多机器人 metric-semantic mapping，以及无结构环境中的 global localization 等实际应用中得到使用。

之后，作者转向 `Alternating Minimization` 与 `Graduated Non-Convexity (GNC)` 的后续工作。`M-estimation` 早已是机器人和计算机视觉里鲁棒估计的主流工具之一，许多论文都研究了不同 robust loss 的行为。有趣的是，不少工作其实已经使用了带辅助变量的 formulation，形式上和式 `(3.16)` 很接近，只是当时并没有意识到它与 `Black-Rangarajan duality` 的联系。例如，有些方法显式引入 latent binary variables 去“关闭” outliers；有些方法用 expectation maximization；也有工作用 `max-mixture distribution` 近似多峰测量噪声。更近的研究还试图用单一参数化函数去统一多类 robust loss，或者在估计未知量 `x` 的同时自动选择合适的 robust loss `ρ`。

`GNC` 本身最早是为早期视觉中的离群点剔除提出的，近些年则被成功应用于点云配准、`SLAM` 以及其他多个鲁棒估计任务。还有研究提出了与 `GNC + IRLS` 十分类似、但从平滑 majorization 角度出发的算法，并进一步给出了 `GNC` 的全局与局部收敛保证。

最后，作者介绍了一类近年来非常重要的新方向: `certifiable algorithms`。此前介绍的算法大致可以分为两类: 一类是 `RANSAC` 或局部 `M-estimation` 这类高效启发式方法，速度快但性能保证弱；另一类是 `branch-and-bound` 这类全局方法，最优性有保证，但几乎无法扩展。`Certifiable algorithms` 试图在 tractability 与 optimality 之间取得平衡。它们把非凸的鲁棒估计问题放松成凸的 semidefinite program (`SDP`)，因而可以在多项式时间内求解，并给出可直接检查的 a posteriori 全局最优性证书。书中提到，这类方法已被用于 rotation estimation、`3D-3D registration` 和 `pose-graph optimization` 等问题；也有比较一般的方法总结了如何为含 outlier 的问题推导这类可认证求解器。不过除了少数特例，这些方法虽然理论上是多项式时间，实际计算代价通常仍远高于启发式方法。好消息是，在某些场景中，相关 insight 还能被用来“验证”一个快速启发式求得的解是否已经达到全局最优，从而兼顾效率与最优性。

总体来看，本章后的研究主线非常清楚: 一方面，前端继续围绕几何约束、图结构和一致性关系，尽可能早地滤除 gross outliers；另一方面，后端则通过 robust loss、`IRLS`、`GNC` 乃至可认证优化，去抑制剩余 outliers 的影响。真正稳健的现代 `SLAM` 系统，通常需要把这些路线组合起来使用。

## 第 4 章 可微优化

### 本章概览

深度学习与几何优化的融合，已经成为现代 `SLAM` 的重要趋势。本章讨论的核心问题是: 当系统里既有 neural networks，又有 `nonlinear least squares (NLS)` 优化模块时，如何把后者也变成可微的，使整个系统能够端到端训练。

更具体地说，本章说明如何对 `SLAM` 中常见的非线性最小二乘问题求导；这使得上层学习模块能够通过优化器返回的几何解，反向更新其参数，从而利用几何先验改进学习效果。

## 4.1 非线性最小二乘回顾

`Nonlinear least squares (NLS)` 的目标，是通过最小化模型预测与观测之间的平方误差，估计一组参数。与线性最小二乘不同，这里的残差一般是变量的非线性函数。在 `SLAM` 中，这类问题无处不在，例如重投影误差、位姿图误差、图像对齐误差等。

本章把目标写成一个由多个向量 residual 组成的求和形式。每个 residual 可以进一步写成 `wi ci(xi)`，其中 `wi` 是权重，`ci` 是向量代价项。上一章已经介绍过，`NLS` 通常通过迭代线性化来解:

- 在当前估计附近线性化残差
- 得到线性系统
- 解出增量 `δx`
- 更新变量
- 重复直到收敛

对应的经典算法包括 `Gauss-Newton (GN)`、`Levenberg-Marquardt (LM)` 以及带 line search 的方法如 `Dogleg`。若变量定义在 manifold 上，则更新操作更一般地应理解为 `retraction`，而非简单加法。

## 4.2 针对非线性最小二乘的微分

若要把深度学习与 `NLS` 无缝结合，就必须让优化器对上层参数可微。典型场景是 `bilevel optimization (BLO)`:

- 下层问题负责几何优化，求出 `x*`
- 上层问题把 `x*` 当作中间结果或监督信号，更新网络参数 `y`

例如:

- 网络负责提特征，`BA` 负责位姿与 landmark 优化
- 网络负责前端相对位姿预测，`pose-graph optimization` 负责消除漂移

更形式化地说，这里需要求解一个 `Bilevel Optimization (BLO)`:

`y* = argmin_(y ∈ Θ) U(y, x*)`  。`(4.2a)`

`s.t.  x* = argmin_(x ∈ Ψ) L(y, x)`  。`(4.2b)`

其中 `L : R^m × R^n -> R` 是 lower-level (`LL`) cost，`U : R^m × R^n -> R` 是 upper-level (`UL`) cost，`x ∈ Ψ` 与 `y ∈ Θ` 分别是可行域。实践中，`x` 往往对应具有明确物理意义的变量，例如 camera poses；而 `y` 往往是没有直接物理意义的参数，例如 neural network 的权重。

在用 gradient descent 优化上层变量时，关键是得到 `U` 对 `y` 的梯度。这个梯度可以写成

`∇_y U = ∂U(y, x*)/∂y + [∂U(y, x*)/∂x*] [∂x*(y)/∂y]`  。`(4.3)`

其中真正困难的部分，不是上层损失 `U(y, x*)` 本身，而是如何求间接梯度 `∂x*(y)/∂y`。围绕这一项，文献发展出了两条主要路线: 一条是显式地穿过下层迭代过程，也就是 `unrolled differentiation`；另一条是利用下层最优性条件，也就是 `implicit differentiation`。本节后续就围绕这两条路线展开。

### 4.2.1 展开式微分

`Unrolled differentiation` 通过让 `automatic differentiation (AutoDiff)` 直接穿过下层优化过程，来求解 `BLO` 问题。具体地说，若在 `t = 0` 时给定初始化

`x0 = Φ0(y)`

那么下层问题的迭代过程可写成

`x_t = Φ_t(x_{t-1}; y),  t = 1, ..., T`                      `(4.4)`

其中 `Φ_t` 表示第 `t` 步下层优化更新。一个典型例子就是 gradient descent:

```text
Φ_t(x_{t-1}; y)
= x_{t-1} - η_t · ∂L(x_{t-1}, y) / ∂x_{t-1}                 (4.5)
```

这里 `η_t` 是学习率，而 `∂L(x_{t-1}, y) / ∂x_{t-1}` 可以用 `AutoDiff` 计算。于是，我们就能把 `x_T` 当成对 `x*` 的近似，即

`x* ≈ x_T = Φ(y) = (Φ_T ◦ ... ◦ Φ_1 ◦ Φ_0)(y)`              `(4.6)`

这样一来，原本的双层优化 `(4.2)` 就被改写成一个显式的单层问题:

`min_{y ∈ Θ} U(y, Φ(y))`                                     `(4.7)`

此时不再需要显式求式 `(4.3)` 中的隐式梯度，而只需要对 `Φ(y)` 关于 `y` 做自动求导即可。

作者还给出了算法 1 的总体流程。先初始化 `y0` 与 `x0`；在每一轮外层迭代中，先用一个通用优化器 `O` 运行 `T` 步求解下层问题，得到 `x_T`；然后再用更高效的方式估计上层梯度 `(4.3)`，这里既可以用 `unrolled differentiation`，也可以用下一节的 `implicit differentiation`；最后再用求得的梯度更新 `y`。

对 `unrolled differentiation` 而言，上层梯度是通过 `AutoDiff` 穿过整个展开后的优化轨迹直接得到的。作者指出，递归梯度的计算通常有两种模式: 一种对应于 `reverse-mode` 的反向传播，另一种对应于 `forward-mode`。本章不展开这两种 `AutoDiff` 机制的细节，而是把实现层面留给 `PyTorch`、`PyPose`、`Theseus` 等库。

这种方法最大的优点，是概念非常直接，而且不局限于 gradient descent；同样的思想也能推广到 `Gauss-Newton` 等其他迭代优化器。缺点则在于计算量和内存占用都可能很高，因为系统必须显式保存整条优化轨迹，才能对其做端到端求导。

### 4.2.2 截断展开式微分

无论是 `reverse-mode` 还是 `forward-mode`，它们本质上都在精确地计算递归梯度；但如果把下层优化完整展开到 `T` 步，那么它们通常都非常耗时。这是因为上层问题会通过所有中间变量 `x_t, t = 0, 1, ..., T` 与下层过程发生复杂的长期依赖；而当 `y` 和 `x` 都是高维向量时，这种困难会进一步加剧。

为了解决这一问题，研究者提出了 `truncated unrolled differentiation`，其核心思路是: 不再保留完整历史，而只保存最近 `M` 次迭代，也就是 `t = T, T - 1, ..., T - M` 这一小段历史。通过忽略更早期的长期依赖，就能显著降低时间和空间复杂度。作者还引用 `Shaban` 等人的结论，指出即便回传步数更少，得到的近似梯度在很多情况下仍能达到与精确梯度接近的优化效果，同时占用更少内存、耗费更少算力。

但在一些计算与内存约束极其严格的机器人应用中，哪怕是截断式 unrolling，依旧可能成为瓶颈。因此，还有工作进一步把它简化为只执行一步迭代，从而彻底去掉递归结构。此时，上层梯度可写成

```text
∇y U
= ∂U(y, x1(y)) / ∂y
+ ∂U(y, x1(y)) / ∂x1 · ∂x1(y) / ∂y                        (4.8)
```

其中 `∂x1(y) / ∂y` 可以由式 `(4.5)` 导出:

`∂x1(y) / ∂y = - ∂²L(x0, y) / (∂x0 ∂y)`                   `(4.9)`

也就是说，这一项本质上是一个 Hessian。由于某些应用中显式计算 Hessian 依然代价很大，作者又给出一个数值近似做法: 直接对变量 `x0` 施加一个很小的扰动，用有限差分的方式近似式 `(4.8)` 中的第二项整体，而不是单独显式求 `∂x1(y) / ∂y`:

```text
∂U(y, x1(y)) / ∂x1 · ∂x1(y) / ∂y
≈ [∂L(x0⁺, y) / ∂y - ∂L(x0⁻, y) / ∂y] / (2ε)             (4.10)
```

这里 `ε` 是一个很小的标量，`x0^± = x0 ± ε ∂U(y, x1(y)) / ∂x1` 是施加了小扰动后的变量。这样做的好处，是可以绕开 `∂x1(y) / ∂y` 的显式计算。

不过，作者也特别提醒，如果变量不是 Euclidean 变量，而是属于 `Lie groups` 这类非欧几里得空间，那么扰动模型必须谨慎设计。好在现代库，例如 `PyPose`，已经支持 `Lie group` 上的 `Hessian-vector` 和 `Jacobian-vector` 自动求导，因此这类近似在机器人系统里已经有越来越可操作的实现基础。

### 4.2.3 隐式微分

从式 `(4.3)` 可以直观看出，其中的 `∂x*(y) / ∂y` 取决于下层问题 `LL cost` `(4.2b)` 的最优性条件，因此完全可以借助 `implicit differentiation` 来求梯度。

在常规微积分中，隐式微分指的是: 当变量 `y(x)` 并不是以显式形式给出，而是由某个约束方程 `R(x, y) = 0` 隐式定义时，我们通常无法先把 `y` 明确解出来再求导。这时可以直接对 `R(x, y) = 0` 关于 `x` 做全微分，然后把得到的线性方程整理成 `dy/dx` 的表达式。例如，对隐函数

`x + y + 5 = 0`

两边同时对 `x` 求导，可得 `dy/dx + 1 + 0 = 0`，因此 `dy/dx = -1`。

作者把这一思路用于双层优化。设下层代价 `L` 对 `x` 和 `y` 至少二次可微，并且 `x*(y)` 是下层问题的一个驻点。由最优性条件可知:

`∂L(x*(y), y) / ∂x*(y) = 0`

对上式关于 `y` 再求导，就得到:

```text
∂²L(x*(y), y) / (∂x*(y) ∂y)
+ ∂²L(x*(y), y) / (∂x*(y) ∂x*(y)) · ∂x*(y)/∂y = 0    (4.11)
```

于是可解出间接梯度:

```text
∂x*(y)/∂y =
- [∂²L(x*(y), y) / (∂x*(y) ∂x*(y))]⁻¹
  [∂²L(x*(y), y) / (∂x*(y) ∂y)]                       (4.12)
```

式 `(4.12)` 的意义在于，它把变量 `y` 与 `x*` 之间本来难以直接求得的间接梯度，转化成了对下层目标 `L` 的直接二阶导数；代价是需要处理一个 Hessian 矩阵的逆。作者随即指出，这在真实系统里通常并不现实。即便上层和下层只各自包含一个一百万参数的网络，存储参数本身只需要约 `4 MB`，但显式存储对应 Hessian 却需要 `4 TB` 量级内存，因而无法直接保存，更不可能显式求逆。

接下来，把 `(4.12)` 代回上层梯度 `(4.3)`，可得:

```text
∇y U
= ∂U(y, x*)/∂y
- ∂U(y, x*)/∂x* ·
  [∂²L(x*(y), y) / (∂x*(y) ∂x*(y))]⁻¹
  [∂²L(x*(y), y) / (∂x*(y) ∂y)]                      (4.13)
```

作者把其中两部分记成 `v^T` 与 `H^{-1}`，然后把问题转写为线性系统 `Hq = v`。这样就不需要显式求 Hessian 逆，而是可以通过求解下面的二次优化来获得 `q`:

```text
q* = argmin_q Q(q) = argmin_q 1/2 qᵀ H q - qᵀ v       (4.14)
```

这一问题可以用简单的 gradient descent 或 conjugate gradient 来高效求解。对 gradient descent 而言，需要的梯度是:

`∂Q(q) / ∂q = Hq - v`

而 `Hq` 本身又可以用快速 `Hessian-vector product` 计算:

```text
Hq = (∂²L / ∂x ∂x) · q = ∂[(∂L/∂x) · q] / ∂x          (4.15)
```

这里 `(∂L/∂x) · q` 是一个标量，因此既不需要显式形成 Hessian，也不需要把 Hessian 存到内存中。算法 2 总结了这一流程: 给定当前上层变量 `y` 和下层最优解 `x*`，初始化 `q`，然后反复执行

`q_k = q_{k-1} - η (H q_{k-1} - v)`                     `(4.16)`

直到收敛，最终再用 `q` 计算:

```text
∇y U = ∂U(y, x*)/∂y - [∂²L(x*(y), y) / (∂y ∂x*(y)) · q]ᵀ   (4.17)
```

其中 `H_yx · q` 同样也可以通过 `Hessian-vector product` 高效计算。

作者最后还讨论了一个常见近似: 完全忽略隐式项，只保留上层目标对 `y` 的直接导数，也就是把下层求得的 `x*` 视为上层问题中的常量。这样做显然更高效，但会引入误差项:

```text
ε ~ (∂U/∂x*) (∂x*/∂y)                                  (4.18)
```

当隐式梯度中出现的是若干很小的二阶导数乘积时，这种近似往往仍然有实际价值。不过是否可行，仍取决于具体的 `NLS` 问题结构。

## 4.3 流形上的微分

当优化变量定义在 manifold 上时，求导不再只是普通 Euclidean calculus。`SLAM` 中的 pose、rotation 等变量常位于 `Lie groups` 上，因此需要把上一章关于 manifold optimization 的工具与本章的 differentiable optimization 结合起来。

### 4.3.1 Lie 群导数

第 2 章已经介绍过 `Lie group`、`Lie algebra` 及其基本运算，例如指数映射和对数映射。本节在此基础上做简要回顾，但重点放在“如何定义它们的导数”上，因为这正是可微优化在流形上成立的关键。

考虑一个 `Lie group` 的流形 `M`。流形上的每个点 `χ` 都有一个唯一的切空间 `T_χ M`，微积分的基本法则就在这个局部线性空间中成立。记 `Lie algebra` 为 `m`，它可以在局部写成 `m = T_χ M`。指数映射 `exp : m → M` 把 `Lie algebra` 中的元素投影回 `Lie group`，而对数映射 `log : M → m` 则作为它的逆，于是有

`χ = exp(τ^)  ⇔  τ^ = log(χ)`                                 `(4.19)`

其中 hat 运算 `^` 是一个线性可逆映射，且 `τ^ ∈ m`。如果把 `Lie algebra` 中的坐标直接写成向量 `τ ∈ R^n`，则可进一步定义向量与群元素之间的映射:

`χ = Exp(τ)  ⇔  τ = Log(χ)`                                  `(4.20)`

这里作者只是把指数与对数映射重新记成以向量为直接输入和输出的形式。

在 `Lie group` 上定义导数之前，首先要刻画两个流形元素之间的相对变化，例如 `χ1` 与 `χ2` 之间的变化。作者通过 `⊕` 与 `⊖` 运算符来描述这种关系。由于群的 composition 一般不满足交换律，因此 `⊕` 与 `⊖` 都分为“右版本”和“左版本”。

右侧运算定义为:

```text
right-⊕ : χ2 = χ1 ⊕ τ  ≜  χ1 ◦ Exp(τ)
right-⊖ : τ  = χ2 ⊖ χ1 ≜  Log(χ1⁻¹ ◦ χ2)                     (4.21)
```

这里 `τ` 出现在右侧，表示它是在 `χ1` 的局部坐标系中表达的扰动。相对地，左侧运算定义为:

```text
left-⊕  : χ2 = ε ⊕ χ1 ≜ Exp(ε) ◦ χ1
left-⊖  : ε  = χ2 ⊖ χ1 ≜ Log(χ2 ◦ χ1⁻¹)                     (4.22)
```

其中 `ε` 是在全局参考系中表达的扰动。无论是 `τ` 还是 `ε`，都可以被看成施加到流形元素上的一个无穷小增量；借助对应的 `⊕` 与 `⊖`，这些增量都能被表示为切空间中的向量。

在有了右 `⊕` / `⊖` 之后，就可以定义流形上的右 Jacobian。设 `f(χ)` 是从一个 `Lie group` 到另一个 `Lie group` 的映射，则其右 Jacobian 定义为

```text
∂f(χ)/∂χ
= lim_{τ→0} [f(χ ⊕ τ) ⊖ f(χ)] / τ
= lim_{τ→0} [f(χ ◦ Exp(τ)) ⊖ f(χ)] / τ
= lim_{τ→0} Log(f(χ)⁻¹ ◦ f(χ ◦ Exp(τ))) / τ                (4.23)
```

若记

`g(τ) = Log(f(χ)⁻¹ ◦ f(χ ◦ Exp(τ)))`

则右 Jacobian `J_R` 就是 `g(τ)` 在 `τ = 0` 处的导数:

`∂f(χ)/∂χ = J_R = [∂g(τ)/∂τ]_{τ=0}`                         `(4.24)`

因此，流形上关于 `χ` 的导数可以由一个 Jacobian 矩阵 `J_R ∈ R^{m×n}` 表示，它实现了从输入流形切空间 `m` 到输出流形切空间 `n = T_{f(χ)} N` 的线性映射。

同理，如果把无穷小扰动 `ε ∈ T_g M` 作用在 `χ` 的左侧，则可以定义左 Jacobian `J_L`:

```text
∂f(χ)/∂χ
= lim_{ε→0} [f(ε ⊕ χ) ⊖ f(χ)] / ε
= lim_{ε→0} [f(Exp(ε) ◦ χ) ⊖ f(χ)] / ε                     (4.25)
```

这里使用的是左 `plus/minus` 运算，最终得到的 `J_L ∈ R^{n×m}` 同样是一个线性映射，只不过定义在全局切空间之间。

为了刻画点 `χ1` 附近的局部扰动，作者再令 `τ = χ ⊖ χ1`，即把 `χ` 看成对 `χ1` 的一个扰动版本。于是定义在切空间上的协方差矩阵可以写成

`Σ_χ ≜ E[τ τ^T] = E[(χ ⊖ χ1)(χ ⊖ χ1)^T]`                   `(4.26)`

有了这样的协方差，就能在流形上建立 Gaussian 分布，写成 `χ ~ N(χ1, Σ_χ)`。关键点在于，`Σ_χ` 实际上定义在 `T_{χ1} M` 上，因此流形中的不确定性依旧可以用向量和协方差矩阵来表达与传播。

作者随后给出一个视觉-惯性旋转估计的例子。设机器人同时配备 `IMU` 和 camera，分别给出带噪声的旋转观测 `R_IMU` 与 `R_Cam`，则可在 `SO(3)` 上通过最小化二者与估计旋转 `R` 的不一致来恢复姿态:

`R^ = arg min_{R ∈ SO(3)} f(R, R_IMU, R_Cam)`                `(4.27)`

其中代价函数具体写成

`f(R) = ||Log(R_IMU^{-1} R)||² + ||Log(R_Cam^{-1} R)||²`     `(4.28)`

为了最小化 `f(R)`，需要计算它在 `SO(3)` 上关于 `R` 的梯度。利用右 Jacobian，可以写出

```text
∇f(R)
= 2 [∂Log(R_IMU⁻¹ R) / ∂R]^T Log(R_IMU⁻¹ R)
+ 2 [∂Log(R_Cam⁻¹ R) / ∂R]^T Log(R_Cam⁻¹ R)                 (4.29)
```

然后就能配合 gradient descent 等算法，在切空间中更新，再投影回流形:

`R_{k+1} = R_k Exp(-α ∇f(R))`                               `(4.30)`

其中 `α` 为步长。如此迭代，直到代价函数收敛到极小值，就能得到同时融合 `IMU` 与 camera 观测的姿态估计。

### 4.3.2 流形上的微分运算

对于常见的流形运算，诸如 inversion、composition 和 group actions，都可以推导出封闭形式的 Jacobian。这样一来，`SLAM` 中的优化就能够系统地在流形上应用链式法则:

`∂Z / ∂χ = (∂Z / ∂Y) (∂Y / ∂χ)`                            `(4.31)`

其中 `Z = g(Y)`，而 `Y = f(χ)`。

先看 inverse。若取 `f(χ) = χ^{-1}`，并将上一节定义的右 Jacobian `(4.23)` 应用到这个映射上，就可以写出

```text
∂χ⁻¹ / ∂χ
= lim_{τ→0} Log((χ⁻¹)⁻¹ (χ Exp(τ))⁻¹) / τ                (4.32)
```

作者借此说明，逆运算在流形上同样可以被转换成切空间中的局部线性变化，因此 Jacobian 仍可通过 `Log` 与极限形式求得。

再看 composition。若令 `f(χ) = χ ◦ χ1`，则相对于 `χ` 的 composition Jacobian 为

```text
∂(χ ◦ χ1) / ∂χ
= lim_{τ→0} Log((χχ1)⁻¹ (χ Exp(τ) χ1)) / τ              (4.33)
```

该式最终等价于把局部扰动 `τ` 通过共轭作用映射到新的切空间中。另一方面，若相对于第二个变量 `χ1` 求导，则有

```text
∂(χ ◦ χ1) / ∂χ1
= lim_{τ→0} Log((χχ1)⁻¹ (χχ1 Exp(τ))) / τ
= I                                                     (4.34)
```

也就是说，composition 关于第二个变量的导数在这种表示下可直接化为单位映射。

接下来，作者给出流形自身的 Jacobian，也就是由指数映射诱导的左右 Jacobian。若 `τ ∈ R^m`，则右 Jacobian 定义为

`J_r(τ) ≜ τ ∂Exp(τ) / ∂τ`                                `(4.35)`

它描述的是 `τ` 的微小变化如何映射到 `Exp(τ)` 所在点的局部切空间。类似地，左 Jacobian 定义为

`J_l(τ) ≜ ε ∂Exp(τ) / ∂τ`                                `(4.36)`

它把 `τ` 的变化映射到流形的全局切空间中。

这些结果的意义在于，流形上的求导并不只是抽象地“把变量看成 pose”那么简单，而是需要为 inversion、composition、`Exp` / `Log` 等基本运算都建立正确的 Jacobian 表达。只有这样，`SLAM` 中的 differentiable optimization 才能真正用链式法则在流形上稳定工作，而不是把位姿变量粗暴塞回普通 Euclidean 向量空间去处理。

## 4.4 自动微分与现代库的数值挑战

让优化器“理论上可微”是一回事，让它“数值上稳定地可微”是另一回事。本节讨论 differentiable optimization 在实现时的数值难点，以及现代相关库如何处理这些问题。

一个代表性例子是 `Lie groups` 上的 `Exp` / `Log` 映射。例如 quaternion 的指数映射中会出现诸如 `sin(||ν||)/||ν||` 这样的表达。若 `||ν||` 很小，直接数值计算会遇到精度问题甚至除零风险。因此，工程实现通常会在小角度区域切换到 `Taylor expansion`，用稳定近似替代直接公式。

本章以 `PyPose` 的 `LieTensor` 为例说明这些问题。它试图把 `Lie groups` 与 `Lie algebras` 作为一类可微数据结构统一支持，并兼容 `CPU`、`GPU` 等主流硬件平台。

### 4.4.1 可微优化实现示例

为了在端到端训练中真正使用 `bilevel optimization`，系统不仅需要标准深度学习优化器，还需要能处理 `SLAM` 常见几何问题的二阶或约束优化器。例如 `LM`、鲁棒核、`IRLS`、信赖域等都可能进入 differentiable optimization pipeline。

书中以一个加权 least-squares 问题为例，说明 `LM` 的核心计算包括:

`min_y Σ_i (h_i(x) - z_i)^T Σ_i (h_i(x) - z_i)`  。`(4.42)`

其中 `h(·)` 是回归模型（`Module`），`x ∈ R^n` 是待优化参数，`h_i` 表示对第 `i` 个输入样本的预测，`Σ_i ∈ R^(d×d)` 是信息矩阵。对 `LM` 而言，解是通过迭代更新 `x ← x_(t-1) + δ_t` 获得的，而更新步 `δ_t` 来自

`Σ_i (Λ_i + λ · diag(Λ_i)) δ_t = - Σ_i J_i^T Σ_i r_i`  。`(4.43)`

其中 `r_i = h_i(x_i) - z_i` 是第 `i` 个 residual，`J_i` 是在 `x_(t-1)` 处计算的 `h` 的 Jacobian，`Λ_i = J_i^T Σ_i J_i` 是近似 Hessian，`λ` 是 damping factor。若记

`A · δ_t = β`  。`(4.44)`

其中 `A = Σ_i (Λ_i + λ · diag(Λ_i))`，`β = - Σ_i J_i^T Σ_i r_i`，那么求步长 `δ_t` 的问题就化成了一个线性系统。实践中，若 `A` 正定，可以使用 `Cholesky`；若 Jacobian 大且稀疏，则也可使用 sparse `Cholesky` 或 `PCG` 等稀疏线性求解器。

若进一步引入 robust kernel functions `ρ : R -> R` 来削弱 outliers 的影响，则目标函数变为

`min_y Σ_i ρ(r_i^T Σ_i r_i)`  。`(4.45)`

此时必须相应修正 `(4.43)`。一种流行方式是采用 `IRLS` 和 `Triggs correction` [1106]，`Ceres` 就采用了这一路线。不过，`Triggs correction` 需要 kernel `ρ` 的二阶导数，而这个二阶导通常为负，可能导致包括 `LM` 在内的二阶优化器变得不稳定。作为替代，`PyPose` 引入了 `FastTriggs`，它只依赖一阶导数，因此更快也更稳定:

`r_i^ρ = sqrt(ρ'(c_i)) r_i,   J_i^ρ = sqrt(ρ'(c_i)) J_i`  。`(4.46)`

其中 `c_i = r_i^T Σ_i r_i`，而 `r_i^ρ` 与 `J_i^ρ` 分别是引入 kernel 后修正过的 residual 与 Jacobian。原书明确指出，`FastTriggs` 的更多细节与证明可见 [1142]。

此外，简单的 `LM` 可能因为初值不好而不收敛，因此实际库还往往支持:

- adaptive damping
- trust region
- dogleg
- 更灵活的 solver / kernel / strategy / corrector 接口

这说明 differentiable optimization 不只是“把优化器包一层 gradient”，而是需要系统级地兼顾求解器、鲁棒性、数值稳定性和接口设计。

### 4.4.2 相关开源库

与 differentiable optimization 相关的开源库，大致可以分成三类: `(1)` 线性代数库，`(2)` 机器学习库，以及 `(3)` 专用优化库。

第一类是 `linear algebra libraries`。它们是机器学习和机器人研究的基础设施。`NumPy` 为 Python 提供了完整的向量与矩阵运算能力，而且由于底层调用了经过高度优化的 `C` 代码，运行效率相当高。`Eigen` 是高性能 `C++` 线性代数库，被 `TensorFlow`、`Ceres`、`GTSAM`、`g2o` 等大量项目采用。`ArrayFire` 则面向 `C`、`C++`、`Fortran` 与 Python 提供 `GPU` 加速的张量和线性代数运算接口，API 简洁，并内置了针对 `GPU` 调优过的函数。

第二类是 `machine learning libraries`。这类库更聚焦于 tensor，也就是高维矩阵运算，以及 `automatic differentiation`。较早的机器学习框架，如 `Torch`、`OpenNN` 和 `MATLAB`，已经能够支持研究者搭建神经网络，但它们大多只支持 `CPU` 计算，而且 API 不够紧凑。之后，随着神经网络规模和复杂度迅速上升，`Chainer`、`Theano`、`Caffe` 等框架开始出现，提供更易用的接口，并支持多 `GPU` 训练。再往后，`TensorFlow`、`PyTorch`、`MXNet` 等现代框架进一步形成了完整且灵活的生态，包括多语言 API、分布式数据并行训练、benchmark 与部署工具等。

在几何学习和 differentiable optimization 方面，作者特别提到几个代表性项目。`Gvnn` 把可微变换层引入 `Torch` 框架，从而推动了端到端几何学习。`JAX` 则可以对原生 Python 和 `NumPy` 函数自动求导，同时支持可组合的函数变换。近年越来越多工作尝试把标准优化工具与深度学习结合，像 `Theseus` 与 `CvxpyLayer` 就展示了如何把 differentiable optimization 嵌入深度神经网络。`PyPose` 则进一步把 `Gauss-Newton`、`Levenberg-Marquardt` 等二阶优化器纳入统一框架，并支持 `Lie groups` 和 `Lie algebras` 的任意阶梯度计算，这对机器人学尤为关键。

第三类是 `specialized optimization libraries`。在机器人学中，大量系统依赖这类专用库。`Ceres` 是处理大规模非线性最小二乘问题的经典 `C++` 开源库，在 `SLAM` 中已被广泛采用。`Pyomo` 与 `JuMP` 则是高度灵活的优化建模框架，便于构建、求解和分析多类优化模型。`CasADi` 依靠高效的数值最优控制实现，被广泛用于机器人真实控制问题。

图优化在机器人学中也极其重要，因此 `g2o` 与 `GTSAM` 这样的 `C++` 图优化框架同样被重点提及。它们提供了简洁的 API 来构造新的 pose graph 或 factor graph 问题，并已被用于求解大量 `SLAM` 优化任务。

此外，优化库在机器人控制中也十分常见。`IPOPT` 是基于 interior-point method 的非线性规划求解器，被广泛用于机器人与控制问题。`OpenOCL` 支持连续时间、离散时间、有约束、无约束、多阶段以及轨迹优化等多种问题形式，适合实时 `MPC`。`CT` 面向大规模最优控制与估计，提供统一接口来调用不同的最优控制求解器，并能扩展到多种机器人动力系统。`Drake` 则同时提供常见控制问题求解器与仿真工具箱，其系统完整性使它在研究社区中尤其受欢迎。

第三类是 `specialized optimization libraries`，包括:

- `Ceres`
- `Theseus`
- `CvxpyLayer`
- `PyPose`
- `g2o`
- `GTSAM`
- `CasADi`
- `IPOPT`
- `Drake`

其中有些更偏 `SLAM` 与图优化，有些更偏控制、最优控制或一般非线性规划。整体趋势是，这些库正越来越多地与深度学习框架耦合，让“学习 + 几何优化”成为一个连续可微的系统。

## 4.5 延伸阅读与最新趋势

近年来，面向机器人视觉任务的深度学习方法发展极快 [1285]。作为典型的 data-driven approaches，人们普遍认为它们在 visual tracking 上相较传统的 handcrafted features 具有优势。该方向的大量工作采用 end-to-end 结构，既包括 `DeepVO` [1162]、`TartanVO` [1166] 这类 supervised methods，也包括 `UnDeepVO` [655]、`Unsupervised VIO` [1172] 等 unsupervised methods。总体来看，supervised 方法通常比 unsupervised 方法表现更好，因为它们能够利用 pose、optical flow、depth 等多种 ground truth 进行学习；但现实世界里获取这些标注本身就是高成本且劳动密集的工作 [1165]。

最近，hybrid methods 正受到越来越多关注，因为它们试图结合 geometry-based methods 与 deep learning 的长处。已有研究探索把 `Bundle Adjustment (BA)` 与 learning modules 结合起来，以在帧之间引入更强的拓扑一致性。例如，有方法把 `BA` layer 直接嵌入网络，如 `BA-Net` [1067] 与 `DROID-SLAM` [1081]；也有方法将图像特征压缩成 code 或 embedded features，并在推理时优化 pose-code graph，例如 `DeepFactors` [236]。此外，`DiffPoseNet` [841] 通过网络预测 poses 与 normal flows，再通过 `Cheirality layer` 对粗预测进行细化。

不过，书中也指出，这些工作里 learning-based methods 与 geometry-based optimization 往往仍是解耦的，分别位于不同子模块中使用。前端和后端缺乏深度集成，可能导致整体性能并非全局最优。与此同时，这些方法通常只是把 pose error 通过 `BA` 反向传播，因此监督信号仍主要来自 ground-truth poses；在这种意义下，`BA` 更像是网络中的一个特殊层。较新的 `iSLAM` [346] 则双向连接前端与后端，通过 `bilevel optimization` 框架让学习模型直接从几何优化过程中获益，并在无需外部监督的情况下带来性能提升。

作者最后指出，`bilevel optimization` 并不限于本文场景。除视觉与 `SLAM` 之外，reinforcement learning [1026]、local planning [1215]、global planning [186]、feature matching [1266] 和 multi-robot routing [419] 等问题，也都可以被建模为类似的双层优化结构。

## 第 5 章 稠密地图表示

### 本章概览

本章讨论 `dense mapping`。与以 landmarks 为中心的稀疏地图不同，稠密地图试图更完整地表示环境的几何结构，适用于重建、碰撞检测、路径规划、渲染、语义理解等多种任务。其核心问题不是“能否建图”，而是“该用什么 quantity、什么空间抽象、什么数据结构，以及为什么”。

本章首先回顾与 dense mapping 关系最密切的传感模态，也就是 `range sensors`；随后讨论 occupancy、distance field、implicit surface 等基本建图 quantity；再系统梳理点云、surfel、mesh、voxel、continuous functions 等不同 map abstractions 与数据结构；最后总结各种表示在具体应用中的使用考量。

## 5.1 距离传感预备知识

在讨论稠密地图表示前，作者先回顾最关键的一类输入传感器: `range sensors`。这一类传感器输出的是到环境物体的距离测量，包括 `LiDAR`、`time-of-flight (TOF)` cameras、`RGB-D` cameras 以及 stereo cameras 等。

本章主要聚焦两类最常用设备:

- 适合户外环境的 `LiDAR`
- 适合室内环境、也逐渐支持室外应用的 `RGB-D camera`

### 5.1.1 传感器测量模型

作者先对 range sensor 的感知机理和测量模型做了简要回顾。对 `LiDAR` 而言，range measurement 由激光束发射、环境反射和回波检测产生 [934]。若发射时间为 `t_emit`、检测时间为 `t_detect`，则根据光速 `c`，测得距离 `r` 满足

`r = c (t_detect - t_emit) / 2`

单条激光束的距离测量，可以通过一组按既定模式运动的 rays 扩展成二维或三维点集，例如做 `360` 度旋转或沿某个特定轨迹扫描。传感器产生的这组点被称为 `point cloud`，它是后续建图最基本的输入元素。`LiDAR` 的测量也常可表示为 `range image` `R ∈ R^(B×M)`，其中 `B` 是 beam 的数量，`M` 是一次完整扫描中水平方向的采样数。对一台机械旋转式 `LiDAR` 而言，可以理解为: 每根 beam 在一整圈 `360` 度转动中记录了一行距离值，因此形成 `B × M` 的量测阵列。

机器人中另一类常用的 range sensor 是 `RGB-D camera`，例如微软的 `Kinect`、`Azure Kinect DK`，以及英特尔的 `RealSense`。`RGB-D` 相机会同时提供彩色图像 `I_RGB ∈ R^(3×H×W)`，以及同分辨率的深度图 `I_D ∈ R^(H×W)`，其中每个像素都存储该位置的 depth 或 range。

为了生成深度图，早期 `RGB-D` 相机通常采用 structured infrared light：向环境投射已知的红外图案，再根据该图案在场景中的畸变恢复每个像素的距离。由于 sunlight 往往会干扰这种 sensing mechanism，因此依赖投影光的这类 `RGB-D` 相机主要用于室内环境。较新的传感器则引入了改进的红外纹理投射器或 `time-of-flight (TOF)` 方案，它们对外界干扰更不敏感，因此更适合室外应用。

### 5.1.2 转换为点云

有了 range image 或 depth map，再结合传感器内参，就可以把测量转换成 point cloud `P = {p1, ..., pN}`。点是 dense mapping 中最基本的构件。

对 `LiDAR`，传感器会为每条 beam 提供一组内参标定角 `(ϕ_ij, θ_ij)`，其中 `1 ≤ i < B`、`1 ≤ j < M`。这里 `ϕ_ij ∈ [0, 2π]` 是 azimuthal angle，`θ_ij ∈ [-π, π]` 是 polar / inclination angle。利用这些已知角度，可将 range measurement `r_ij` 转换为三维点 `p = (x, y, z)`:

`x = r_ij cos(θ_ij) cos(ϕ_ij)`  。`(5.2)`

`y = r_ij cos(θ_ij) sin(ϕ_ij)`  。`(5.3)`

`z = r_ij sin(θ_ij)`  。`(5.4)`

对 `RGB-D camera`，则通常使用 pinhole camera model。设像素位置 `(u, v)` 处的 range 为 `r_uv = R_uv`，并记相机内参为 `K ∈ R^(3×4)`，则可以先把图像坐标写成 homogeneous coordinate `x = (u, v, r_uv)`，再用 `K` 的 pseudo inverse 将其映射到相机坐标系中的三维点:

`p = K^† x`  。`(5.5)`

作者还特别强调了 `organized` 与 `unorganized point clouds` 的差异。由 depth map 得到的点云通常是 organized 的，邻域关系可由像素索引直接推得；而 `LiDAR` 点云在 motion distortion、固态雷达非重复扫描模式等情形下，则常常变得 unorganized。这时，若要得到几何上合理的点云，往往需要结合 per-beam timestamp、`IMU` 或 odometry 对点云做 `motion compensation`。

## 5.2 稠密建图基础

地图本质上是一种对环境的符号化表示。对于同一片空间，可能存在很多等价但风格不同的表示方式。不同表示的优劣高度依赖应用目标: 稀疏特征地图适合 localization，稠密高分辨率地图适合重建，而 path planning 往往更关心 occupancy 或距离信息。

本章把 dense mapping 的核心问题归纳成三个:

1. 应该估计什么 quantity
2. 应该采用什么空间抽象
3. 这些表示该如何存储

### 5.2.1 Occupancy Maps（占据栅格地图）

自 Elfes 和 Moravec 在三十多年前提出以来 [304, 773]，`occupancy grids` 一直是最广泛使用的地图表示之一。它们之所以长期流行，一个重要原因是简单且计算高效，尤其适合室内，甚至很多室外环境的建图任务。最基本的情形下，我们要估计的是每个 cell 被占据的概率；也就是说，把一个 cell 是否包含障碍物建模成一个概率量，其中 `1` 表示 occupied，`0` 表示 free。从这个角度看，occupancy mapping 本质上是对每个 cell 做 binary classification。

给定传感器 measurements `z_(1:t)` 和一组传感器 poses `x_(1:t)`，地图中每个 cell `m_k` 的占据概率可写成 `p(m_k | z_(1:t), x_(1:t))`。书中在图 `5.5` 中用简单的 `2D` ray-casting 例子说明了这一点: 当某个方向上测得一条 range return 时，就意味着该方向上某个位置存在障碍物。

在假设各个 cells 相互独立、且 measurements 在条件下独立时，occupancy 的更新可以高效写成著名的 `log-odds` 形式 [773]:

`l(m_k | z_(1:t)) = l(m_k | z_(1:t-1)) + l(m_k | z_t)`  `(5.6)`

其中 `l(·|·) = log(o(·|·))`，而 `o(·|·)` 是 odds 形式:

`o(m_k | z_(1:t)) = p(m_k | z_(1:t)) / (1 - p(m_k | z_(1:t)))`  `(5.7)`

这套写法的主要优点在于，更新时只需要前一时刻的 occupancy 值，以及 inverse sensor model `l(m_k | z_t)`，然后通过一次简单加法就能完成递推，因此实现非常直接，也非常适合在线增量建图。

但这种高效性建立在若干很强的假设之上。最关键的是，occupancy grids 默认一个 cell 的占据概率与其他 cells 无关，因此忽略了空间相关性；而这种相关性恰恰可能对推断附近未观测区域是否被占据非常重要。除此之外，传统 occupancy grids 还要求事先固定环境离散化方式，因此整张地图的空间分辨率在构建前就已确定，难以根据局部几何复杂度自适应调整。

### 5.2.2 隐式曲面与距离场

另一条路线不是直接估计 occupancy probability，而是估计“自由空间与占据空间的边界”。若存在一个连续函数在物体表面处取零值，其正负号还能区分 inside / outside，那么这个函数就定义了一个 `implicit surface`。

这类表示的好处是:

- 没有固定分辨率限制
- 可表示任意复杂形状
- 可以通过函数符号快速判断点是否在障碍物内部

其中最常见的一个 quantity 是 `Euclidean Signed Distance Function (ESDF)`，它在任意查询点返回到最近表面的 signed distance，因此不仅能给出碰撞信息，也能提供高质量 proximity gradients，对 optimization-based planning 尤其重要。

在实际重建中，直接估计精确 `ESDF` 往往不容易，因此更常见的是使用由 measurement rays 更容易计算出的 `projective signed distance`。经典做法是把多个 projective signed distances 通过加权平均融合起来，并将其截断在一个固定带宽内，从而形成 `Truncated Signed Distance Function (TSDF)`。`TSDF` 的零交叉仍对应估计表面，而 `ESDF` 则可进一步从 `TSDF` 或 occupancy map 推出。

### 5.2.3 Occupancy Maps 还是 Distance Fields？

`Occupancy maps` 与 `distance fields` 都是在体积空间里“处处估计某种 quantity”，但它们优先服务的目标不同。

第一个关键差异是 `directness`。对 occupancy 而言，measurement rays 本身就直接告诉我们哪些区域是 free、occupied 或 unobserved，因此更新过程较少依赖额外假设。相比之下，implicit surface 通常估计的是到表面的距离，而这个 quantity 对于“部分可见的表面”来说并不是直接可观测的，因此往往需要像 `TSDF` 这样的代理构造。

第二个关键差异是 `smoothness`。implicit surfaces 天生更平滑，因此:

- 可微
- 可插值
- 更容易得到亚像素精度与高质量梯度

但平滑性也意味着它们不擅长表达真正的几何不连续，尤其容易遗漏薄障碍物。occupancy maps 则在捕捉 thin obstacles 时往往更保守、更可靠。

## 5.3 地图表示

本节系统梳理 dense maps 的空间抽象形式。作者从“表示目标空间结构的显式程度”开始，再讨论具体抽象类型与对应数据结构。

### 5.3.1 目标空间结构的显式程度

如图 `5.4` 所总结，地图表示既可以按 target space 来分，也可以按 explicitness 来分。在 `3D` 建图中，仅仅表示 volume 当然很自然，但 surfaces 对机器人任务同样重要，因为许多 downstream tasks 都依赖于对物体边界的理解。综合这两个维度，作者把常见表示概括为四大类:

- `explicit surface`
- `implicit surface`
- `explicit volume`
- `implicit volume`

对于 surface，可以采用显式或隐式两种表达。若把 surface 看作一个 `2D manifold`，那么 `explicit surface` representation 的目标就是直接刻画场景中物体的边界。最简单的显式抽象，就是 range sensors 直接产生的 `point cloud`。另一种更一般的表面表示是 polygon `mesh`，它由 vertices、edges 与 faces 组成，通常以连接的闭合 polygons，尤其是 triangles，来编码体积的边界。`Surfels` 也是很流行的表面抽象，并已广泛用于建图。表面表示不仅对 visualization 很关键，也常用于 rendering、模拟环境构造、augmented reality、`Computer-aided design` 以及 `3D printing` 等任务。

在 `3D` volume 建模里，也存在类似的区分。最朴素的 point-based 表示在很多 `LiDAR SLAM` 系统中都很常见；此外，`occupancy voxels` 与 `distance-based voxels` 也是典型的 `explicit volume` representations。由于 volumetric maps 往往数据量很大，因此在存储时必须仔细权衡数据结构，以控制计算和内存代价。

对应地，`implicit volume` representations 通常通过函数来建模体积空间。`Gaussian Process (GP)`（见 `5.4.5`）和 `Hilbert maps`（见 `5.4.6`）就是两类经典例子。它们不直接存储每个离散单元，而是把空间属性编码进连续函数，通过查询函数值来恢复某个位置的环境信息。

### 5.3.2 空间抽象的类型

#### 5.3.2.1 点

给定 range measurements，一种最直接的稠密地图表示就是 point clouds。其基本做法，是把时刻 `t` 在局部坐标系 `F_t` 下记录的点云 `P_t`，利用估计得到的全局位姿累计到同一个全局地图点云 `P_M` 中。实际中也常把第一帧点云坐标系 `F_1` 直接作为全局参考系。这样做实现简单，而且几乎不额外引入几何假设。

但单纯累积 `P_1, ..., P_t` 无法扩展到大规模环境，因此现实系统通常要丢弃对同一空间位置的冗余测量。为此，很多方法会结合高效的 nearest neighbor search 结构，例如 voxel grids 或 hierarchical tree-based representations（见第 `5.3.3` 节），来做下采样与存储；典型策略包括每个 voxel 只保留有限数量的 points，或只保存满足某种准则的特定 points。另一条路线是仅保留少量 keyframes，对应只显式存储少数点云，但这又要求系统能判断何时应生成新的 keyframe 或 submap。

作为最基础的表示形式，point cloud map 可以再被转换成其他表示，例如通过 `Poisson surface reconstruction` [1130] 得到 mesh，或通过 `marching cubes` [1131] 得到 `Signed Distance Function (SDF)`。但这类转换通常需要额外信息，例如点的观测 viewpoint。遗憾的是，当多个测量被融合成一个 point cloud map 时，这类信息往往已经丢失，因此通常还需要对表面朝向作额外假设，才能区分 inside 与 outside 区域。

#### 5.3.2.2 Surfels（表面元）

与直接存 measurements 的 point cloud maps 不同，点本身既不包含 surface information，也不记录一个点是从哪个方向被观测到的。`Surfels` [869] 正是通过给 point 加入方向信息来编码这类内容。常见 surfel 形式包括 circular discs、elliptic discs，以及更一般的 ellipsoids；后者常通过 Gaussian 建模。`Splatting` 还允许把纹理信息融入 surfels，并在某个 viewpoint 下把重叠 surfels 混合成一致渲染结果。

一种常见的 circular surfel，也就是圆盘型 surfel，通常由位置 `p ∈ R^3`、法向 `n ∈ R^3` 和半径 `r ∈ R` 定义。这类几何原语可以借助现代 `graphics processing units (GPUs)` 高效渲染，因此既能加速 point-to-surfel association，也常带来显著的内存节省。

从表示效果看，密集 point cloud 虽能高细节地刻画环境，但代价是内存开销大；surfel map 则会把多个 measurements 聚合为单个 surfel，因而牺牲一部分 fine details，却能在保留主要大尺度 surface structure 的同时大幅降低内存占用。

与 surfels 紧密相关的还有 `normal distributions transform (NDT)` [82, 1034]。`NDT` 把空间划分为 voxels，并将每个 voxel 内的 points 近似为正态分布 `N(μ, Σ)`。通过协方差矩阵的特征值 `λ_1 < λ_2 < λ_3` 及对应特征向量 `v_1, v_2, v_3`，可以估计该 voxel 内部的表面性质；对于平面区域，若 `λ_1 << λ_2`，则最小特征值对应的特征向量 `v_1` 就近似是 surface normal。因而，对 planar surfaces 来说，`NDT` 可以被视为一种 surfel 表示；同时，它又能更准确地表达那些无法被单个 surfel 近似的点分布。从这个意义上说，`NDT` 是一种 hybrid representation: 它因 voxel grid 而 explicit，又因 voxel 内部用 normal distribution 连续表示空间而带有 implicit 成分。

#### 5.3.2.3 网格

虽然 point clouds 与 surfel maps 都能描述 local surface properties，但它们仍相对稀疏，因为并未显式建模表面的 connectivity。为了获得对表面的更完整理解，可以使用 meshes，它将表面描述为一组彼此连接的 polygons。这样不仅可以表示 watertight surfaces，还能支持 surface interpolation，以及沿着 connected surface 的高效遍历。

在 meshing 术语中，每个 polygon 称为一个 face，而每个角点称为一个 vertex。最常见的两类是 triangle meshes 与 quad meshes；其中 triangle mesh 既最简单，也最一般，因为任意 polygon 都可以被分解为一组等价的 triangles。

Mesh 是一种非常灵活且 memory-efÏcient 的表示，因为 faces 与 vertices 的数量都可以直接随 surface complexity 和所需细节水平调整。比如，一个任意大小的平面理论上只需两个三角形和四个顶点即可表示。除此之外，meshes 也很适合 parallel processing 与 rendering，因此在 rendering、surface analysis、manipulation、deformation，以及更广泛的数字建模、simulation 和 surface-based algorithms 中都十分常见，例如用于 ground robots 的 path planning。

#### 5.3.2.4 体素

Point clouds 与 meshes 很适合表示定义在 surfaces 上的环境属性；但像 occupancy 和 signed distance 这类 quantity，则定义在整个 volume 中。要存储和处理这类 volumetric properties，一种直接办法就是把它们离散化到 regular grid 上；于是离散占据图和 signed distance map 分别就成为 occupancy grids 与 signed distance fields。

可以把 voxel 看作二维 pixel 在三维中的推广。给定规则网格结构后，每个 voxel 都可分配唯一索引并存入某种 data structure。需要注意的是，voxel 本身只是一个 container，更形式化地说，是一种 space partition；它的具体含义完全取决于其中存储的内容。

例如，在经典 occupancy grid 中，一个 voxel 的值表示该 voxel 内任意点被占据的 likelihood，因此地图中任意查询点的 occupancy 直接等于包含它的那个 voxel 的值。而 voxel 的值并不一定对应“常值小立方体”。在 `Signed Distance Field` 中，voxel 通常存的是该 voxel 中心处的 signed distance；若要查询任意点的值，就需要访问其邻近 voxels，并通过 interpolation 得到该点的距离值。还有一些应用甚至会用稀疏 voxel grids 来存放非 volumetric properties，例如 points 或 surface colors。

#### 5.3.2.5 连续函数

Functions 是以连续方式进行 mapping 的一种关键抽象。在这里，mapping 问题被转化为拟合某个 parametric 或 non-parametric function，也就是求解一个 regression problem。前面很多空间抽象都要求预先离散化环境，因此整张地图的 spatial resolution 往往是固定的；而 continuous functions，无论参数化还是非参数化，都更灵活，因为分辨率可以按需重新计算，同时天然支持 interpolation 来填补数据空洞。

其中，一些 parametric functions，例如二维中的无限直线 [1105]、三维中的平面 [532, 374, 1105]，需要对环境作较强假设，因此会限制 scene representation，但它们在 memory consumption 和 computational complexity 上都很高效。另一方面，基于 control points 的函数，如 `B-splines` [931]，以及 non-parametric 的 `Gaussian Process (GP)` 表示，则能以更少环境假设连续建模场景。

从 occupancy [815]、implicit surface [1185, 635]、distance fields [1194] 到 surface 本身 [1121]，基于 `GP` 的表示都很流行。尽管它们的 computational complexity 较高，但 probabilistic nature 使它们能够对已观测与未观测区域同时执行 uncertainty quantification 与 inference [378]。连续函数的另一个核心优势是: 如果它们至少一阶可微，就能提供 gradients。梯度信息对 localization 中计算 surface normals [1195]、loop closure 中构造 terrain features [377]、data fusion [635] 与 planning [1195] 都非常关键。

#### 5.3.2.6 表示转换

上述抽象并不总是互斥使用，实际系统里常会在多种形式之间来回转换，或者同时保留多种表示。

例如，可将 points 或 meshes 这类 explicit geometry 转换为 implicit surface。一种灵活做法，是在空间任意查询点上通过 closest point lookup 计算 signed distance；这可以对任意 explicit geometry 按需执行，仅在需要的位置和时刻查询。另一种做法，则是在规则网格上的所有点统一计算 signed distance，这时可利用 wavefront propagation，并通过 `fast marching method` [990] 高效实现。

反过来，把 implicit surfaces 转成 mesh 也很常见。最经典的算法是 `Marching Cubes` [695]。它将隐式表面划分成固定大小的 cubes，并分别独立处理；每个 cube 根据其八个角点上的 implicit function values，映射到若干标准配置中的一种，再生成相应的 triangle elements，随后通过线性插值细化顶点位置。由 `BIM (Building Information Modeling)` 或 `CAD (Computer-Aided Design)` 模型得到的 meshes，还可以进一步采样成 points 或 surfels。

有时还必须把 discrete representation 转回 continuous representation。常见做法，是对连续表示的参数求解一个 optimization problem，使其相对于离散数据的拟合误差最小。

### 5.3.3 数据结构与存储

有了空间抽象之后，仍然必须把它们有效存储在内存中。本节讨论常见数据结构及其利弊。

#### 5.3.3.1 朴素数据存储

对很多地图表示来说，一个简单且可动态扩展的数组是很自然的起点。对于具有预定义空间划分的数据，只需要两样东西: 要存储的数据类型，以及一个把空间坐标转换成索引坐标的函数。这类方式常见于 occupancy 或 signed distance values 的存储。对于不规则数据，则只需给出元素类型本身，例如 point clouds 或 surfels。书中用图 `5.9` 说明了这两种情形: 一种是借助映射函数把有序数据存入数组，另一种是直接把无序 point cloud 数据放入数组结构。

这种 naive approach 的优点是实现简单，并且能提供很快的 random access。其代价则是内存需求可能非常大。此外，虽然 read 与 modify 操作都很快，但一旦需要改变表示的空间范围，代价就会很高，因为往往需要复制整个 data structure 的内容。

#### 5.3.3.2 Hash Map（哈希映射）

为了解决朴素存储法在大空间下的局限，一个自然扩展就是使用 hash table。其做法是先把地图切分成 shards，再用 hash function 把每个 shard 的坐标压缩为单个值，并把该 shard 数据存入以该 hash value 为索引的表中。shards 通常对应具有明确坐标的地图子区域，例如规则网格中的 cubes；它们既可以对应单个 voxels，也可以对应固定大小的 voxel blocks，还可以存放位于各自子区域中的 points、surfels 或 mesh fragments。

Hash table 保留了固定数组 `O(1)` look-up time 的优点，同时又允许地图在不重分配整块内存的情况下动态增长。书中特别指出，在用 hash table 存 dense map 时，有三个关键设计点必须考虑:

1. `Granularity of the sharding`：更小的 shards 能带来更好的 sparsity，因为只在必要位置分配数据；但如果 shard 数量过大，又会降低 insertion performance 和 memory efficiency。这个取舍在只沿 surfaces 存储属性时尤其重要。
2. `Hash function`：理想的 hash function 应当把 keys 尽量均匀地分散到整张表中，即便这些 keys 在空间上彼此相邻，而这恰恰是 mapping 场景最常见的情况。
3. `Collision resolution`：无论采用 linear chaining，还是 open addressing，处理 collisions 的方式都会显著影响 hash table 的性能。

总体上看，hash tables 往往能在 fast access 与高效 insertion / deletion 之间取得较好平衡，但通常仍需根据具体应用做一定的初始调参。

#### 5.3.3.3 基于树的数据结构

另一种只为环境中 relevant parts 分配内存、同时仍能高效访问 spatial data 的办法，是采用 hierarchical tree-based representations。像 hash tables 一样，trees 通常也能提供高效的 access 与 insertion；但它们的独特优势在于 hierarchical structure 本身，这使它们既能高效存储 multi-resolution data，也能加速 nearest neighbor search 之类的空间操作。最常见的树结构包括 `kD-trees` [72]、`bounding volume hierarchies (BVH)` [223] 和 `octrees` [751]。

在这些结构中，`octree` 尤其适合支持 novel measurements 的增量融合。它以所谓 octant 作为节点，每个 octant 对应一个子空间，由中心 `c ∈ R^3` 和 extent `e ∈ R` 定义，可视作一个 axis-aligned bounding box。每个 octant 最多拥有 `8` 个子 octants，且子节点的 extent 为父节点的一半。实际中，通常只在 leaf octants 中存储 points，而内部 octants 对应的点集则通过 tree traversal 来确定。

构建 octree 时，可以从包围 point cloud `P` 的 axis-aligned bounding box 出发，递归地把空间切分成 octants。每次划分都会把 `P` 分成 `P_1, ..., P_8` 八个子集，对应 `8` 个 child octants；对非空子集继续递归，直到 octant 尺寸达到预设阈值，或点数小于某个最小值为止。树构建好后，更新和插入都可以通过遍历结构并按需添加内部节点来高效完成。如果新数据落在现有根 octant 之外，还可以通过创建新的 root node，把旧树和新数据分别挂到新根的不同 children 下，以实现动态扩展。

与 voxel grids 相比，octree 只表示真正含数据的子空间，因此在 occupied space 的存储上更高效。但这种 memory efficiency 的代价，是访问某个具体 leaf octant 时需要 tree traversal，从而可能带来更高的定位开销；此外，树结构本身也必须显式编码，因此还会有额外 memory overhead。

#### 5.3.3.4 混合数据结构

为了在 memory requirements 与 data access runtime 之间取得更好平衡，很多方法会有针对性地组合不同数据结构的优点，从而形成 hybrid representations。这里的核心思路是: 接受略差一些的纯粹内存最优性，换取更高效的访问和更新。

例如，`hashed voxel grids` [807] 把 dense voxel grids 与 hash tables 结合起来: 先把环境划分为固定大小的稠密 blocks（例如 `8×8×8` voxels），再把这些 blocks 存入 hash table。这样一来，借助 hash table 的灵活性，只有包含有意义信息的位置，例如靠近 surface 的区域，才需要真正分配 blocks；与此同时，每个 block 内部仍使用普通 `3D array` 存储 voxels，因此具体操作保持简单、高效，并且很适合 `GPU` 加速。

另一条路线，是把 hash tables 与 trees 结合起来。与 `hashed voxel grids` 类似，`VDB` [791, 790] 也会先把空间切成 hashed blocks，但每个 block 内部再维护一棵 hierarchical tree。这样它既拥有 hierarchical trees 的全部好处，例如 multi-resolution representation 与高效 nearest neighbor lookups，又因为每个 block 的大小固定，使得最大树高与环境总体大小无关。于是，lookups 与 insertions 都可以在常数时间内完成，并且通常比纯树结构更快。

## 5.4 地图构建的方法与实践

前面讨论的是“可以怎么表示地图”，本节讨论的是“实际系统如何构建这些地图”。作者按照主要空间抽象类型，把方法大致分为 points、surfels、meshes、voxels、`Gaussian Processes`、`Hilbert maps` 与 deep-learning-based mapping。

### 5.4.1 点

point-based 方法通常不会无脑积累全部 raw points，而是结合 voxel grid 或 octree 进行 subsampling 与存储。对 `LiDAR SLAM` 而言，很多系统会额外做 feature extraction，只保留 corner points 与 surface points，从而大幅减小点云规模。

另一类方法则更强调保留原始 measurements，只是通过 voxelization 控制每个 voxel 中存点数量。这样做的关键价值，是在压缩数据量的同时保留真实测量，而不是把点平均掉。

总体而言，point-based maps 的核心配套结构往往就是 `(hashed) voxel grid + nearest neighbor search`。

### 5.4.2 Surfels

对 surfels，也可以采用与点云类似的空间分区策略。例如有些方法使用 octree，并在不同层级维护多分辨率 surfel 统计，从而支持更高效的数据关联。

另一类方法则直接把 surfels 当作显式图形原语，利用渲染管线做高效关联与更新。对于 loop closure 场景，一些系统还会直接对 surfel map 做 deformation，而不是只在 pose graph 层面修正，从而让 dense map 本身随回环而被拉回一致状态。

在大尺度室外场景中，surfels 常配合 submaps 使用，以便把局部 map 分块管理，并按需在 `GPU` 与主存之间迁移。

### 5.4.3 网格

mesh generation 大致有两大家族。

第一类直接从测得点集出发，例如基于 `Delaunay triangulation` 直接找出表面三角形子集。这类方法对高质量、稠密、规则采样数据效果好，但也更依赖输入点云质量。

第二类先重建 implicit surface，再从中抽取 mesh。常见做法包括:

- 在规则网格上估计 signed distance
- 用 `RBF` 拟合 implicit function
- 用 `moving least squares (MLS)` 做局部拟合
- 用 `Poisson Surface Reconstruction` 通过 PDE 求解平滑表面

在机器人在线建图中，`TSDF`-based meshing 特别流行，因为:

- `TSDF` 更新本身天然增量式
- 计算主要花在 implicit surface 更新上
- 最终 mesh 提取相对容易

### 5.4.4 体素

voxel-based 方法是三维重建和机器人稠密建图中最常见的 volumetric representation 之一。本节按四个角度讨论它们: 估计 quantity、数据结构、measurement integration、以及 scalability。

#### 5.4.4.1 按所估计 quantity 分类的方法

在 voxel framework 中，首先要决定估计什么 quantity。最常见的两类是 occupancy 与 distance metrics，例如 `TSDF` / `ESDF`。两者的关键差别，在于前者更偏保守的占据判断，后者则更强调连续几何与距离信息。

Occupancy estimation 的早期连续概率 measurement model 最初是为 sonar 提出的 [773]；后来为了降低计算代价，又发展出更简单的 piecewise-constant models [478]。这种变化与 `LiDAR` 的兴起以及从 `2D` 向 `3D` 地图的迁移密切相关。更近一些的工作，如 `Loop` 等人 [691]，提出了一个连续概率模型，使得 occupancy probability 会沿物体表面收敛到 `0.5`，而不是把障碍物人为“膨胀”开来。总体上，occupancy estimation 因 recall 更高而在 collision avoidance 中非常流行，但它受限于不连续的表示性质，因此相比 distance-based methods，梯度信息要贫乏得多。

对 distance metrics 而言，问题会更微妙。我们不仅要估计表面前方的正距离，还必须对表面后方的负距离做外推，因为隐式表面正是由 signed distance field 的 zero-crossings 来定义的。为限制把不完美的正负距离估计融合在一起所带来的精度影响，实践中通常会把更新限制在表面边界附近一个较小的 truncation band 内。即便如此，distance-based methods 仍容易抹掉几何。例如，当一个 thin object 的两侧都被观测到时，把真实正距离与“想象出来的”负距离做平均，可能会使 zero-crossings 翻转或干脆消失。一些工作专门分析了 truncation band 宽度与 weight drop-off 对最终重建质量的影响 [134]。从根本上说，这个问题只能缓解，无法彻底消除。总体而言，`TSDF` 在 smooth surfaces 上通常比 occupancy methods 生成更好的表面，但代价是对 thin objects 的 recall 更低。

`TSDF` 提供的距离信息本身很有价值，但它并不保守，反而会系统性高估 Euclidean distance。为解决这一安全问题，`voxblox` [821] 推广了增量式构建 `ESDF` 的做法：先把 sensor data 融合进 `TSDF`，再用 `brushfire algorithm` [628] 更新对应的 `ESDF`。随后，`FIESTA` [426] 又提出了一条 hybrid route：不是从 `TSDF`，而是从 occupancy map 增量更新 `ESDF`。

#### 5.4.4.2 按数据结构分类的方法

最简单的 volumetric mapping data structure 是静态 `3D` 数组。正如 `KinectFusion` [800] 所展示的，这种结构在小型且固定大小的场景中效果很好。但很多应用都要求地图在运行时能够动态扩展，同时又只在真正需要的位置分配 voxels 以节省内存。

为解决这一问题，`Nießner` 等人 [807] 提出了 `voxel-block hashing`。其思路是把 voxels 按 blocks 分组（例如 `8×8×8` voxels），并把这些 blocks 存入 hash map。该结构很快被 `TSDF` 系统广泛采用，因为它能同时提供常数时间 `O(1)` lookups 和动态插入；当然，它也同样可以用于存储 occupancy probabilities，例如 `FIESTA` [426]。与逐个 voxel 做 hashing 相比，按 block 分组还能在 hash table 大小与 voxel 分配粒度之间提供一个可调节的折中。

当然，也可以使用 tree structures 存储 voxels。`Octomap` [478] 首先把 octree 用于 occupancy probabilities，并在很长时间里成为 volumetric mapping 的事实标准。树结构最大的优势是天然支持 multi-resolution；但其主要限制在于，编码树结构本身会引入显著 memory overhead，而且 cell lookup time 与树高成正比。

很多较新的方法通过 hybrid data structures 缓解这一问题。例如 `Supereight` [1124] 建议在前几层使用标准动态 octree，而在更深几层使用 static octrees；后者可以理解为用固定大小数组存储的 octree。这样做去掉了用 pointers 编码 parent-child relationships 的内存开销，但代价是分配粒度变粗，因为 static octrees 总是作为整块分配。

`VDB` [791] 最初来自 visual effects（`VFX`）行业，后来被多个 volumetric mapping frameworks [720, 1131] 引入。正如第 `5.3.3.4` 节所述，它把 block-hashing 与 trees 结合起来，兼得较好的 memory efficiency、类似 hash 的常数时间 lookups / insertions，以及类似树的 multi-resolution 能力。

一个很实际的考虑是，下游任务往往要求在 occupancy probabilities 或 `TSDF` 之外，再额外存储 colors、semantics [398, 941] 或 `ESDF` [821, 426]。虽然理论上几乎任何 data structure 都能扩展出这些额外 channels，但实现代价会随着底层结构复杂度迅速增加。这也是为什么很多系统会偏好更简单的结构，例如 `voxel-block hashing`，或者直接依赖灵活的第三方库。

#### 5.4.4.3 按测量融合算法分类的方法

measurement integration 算法决定了如何根据深度观测更新 map。主要有两条路线:

- `ray tracing`
- `projection-based integration`

`Ray tracing` 对每个 measurement ray 沿射线更新所有穿过的 voxels，通用性强，只需要知道传感器原点；但容易出现多个 rays 命中同一 voxel，从而带来重复计算与并行写冲突。

`Projection-based` 方法则反过来直接遍历地图中的 observed voxels，把 voxel 投影到传感器坐标中寻找对应更新。这样天然避免 race conditions，因此在多线程和 `GPU` 系统中很受欢迎。不过它对传感器 pose 与投影模型要求更明确，也更不适合处理未组织或经运动去畸变后的点云。

#### 5.4.4.4 按可扩展性分类的方法

对 volumetric mapping 而言，memory 与 computation 是两个最核心的 bottlenecks。对于 fixed-resolution 方法，memory 和 computational complexities 会随地图总体积线性增长，并随所选分辨率呈立方增长。如何降低这些复杂度，以便构建超越小范围场景的高细节地图，一直是重要研究方向。

早期工作主要着眼于降低 memory usage。例如 `Octomap` [478] 会在 octree 的内部节点存储其 children 的 max 或 average occupancy；这样一来，当 leaf nodes 的估计值与父节点非常接近时，就可以递归地把这些 leaf 剪掉，从而用更少、更低分辨率的节点表示常值区域。由于真实环境中绝大部分都是 free space，这种对环境几何的自适应在实践中非常有效。与此同时，在内部节点保存 min / max / average 值，对下游任务也有帮助，因为这使得系统能在更低分辨率上做 map query，并支持用于 fast collision checking 或 exploration planning 的 hierarchical algorithms。`Octomap` 的核心限制在于，虽然内存得到了节省，但它仍然总是在最高分辨率上融合所有 measurements，因此 computational complexity 的立方级增长问题并没有消失。

Multi-resolution 还可以被用来降低 measurement updates 的计算成本。由于 measurement rays 是按固定角度发出的，远处几何体本来就会被更少的 rays 击中，因此让 update resolution 随距离增大而降低是很合理的。实现这一点的方式包括 `multi-resolution ray-tracing` [286] 与 `multi-resolution projective integration` [1124]。`Supereight2` [347] 则更进一步，把更新分辨率与 measurement updates 的 entropy 联系起来，以进一步降低计算复杂度。

这类方法虽然显著提升了更新效率，但仍有一个残留问题：地图中不同 resolution levels 之间往往必须显式同步。一个更彻底避免同步的方法，是不在每个 octree node 中存绝对值，而只编码相邻 resolution levels 之间的差异。这可以通过 `wavelet decomposition` 形式化实现。Wavelet-encoded maps 能在任意时刻、任意分辨率下被高效查询。利用这一性质，`wavemap` [922] 进一步采用 coarse-to-fine 的方式更新地图；除了根据 measurement entropy 调整更新分辨率之外，它还会跳过那些不具信息量的更新，例如某块区域的 occupancy 已经收敛为 free，且所有新 measurements 都与这一结论一致的情况。

### 5.4.5 Gaussian Processes（高斯过程）

把 mapping 看成回归问题时，`Gaussian Processes (GPs)` 是非常自然的工具。它们是 stochastic、non-parametric、non-linear regression 方法，能够在给定稀疏、带噪测量的前提下，对任意查询位置估计未知函数值。

在稠密建图里，`GPs` 的重要优势包括:

- 能显式建模空间相关性
- 可提供预测均值与不确定性
- 可在未观测区域做有 principled 的推断
- 导数仍然是 `GP`，因此 gradients 与 uncertainty 都能连续获取

其中 `Gaussian Process Occupancy Mapping (GPOM)` 就是把 occupancy mapping 变成 `GP` 回归问题的代表。它克服了传统 occupancy grids 中“cells 独立”的强假设，但经典 `GPOM` 的代价也很高，原始形式通常具有立方级训练复杂度，因此后续大量工作都在研究增量式或稀疏近似版本。

#### 5.4.5.1 Gaussian Process Implicit Surface（高斯过程隐式曲面）

隐式曲面同样可以用 `GP` 来表示。`Gaussian Process Implicit Surface (GPIS)` 方法 [1185, 735, 676, 505] 使用 `GP` 来根据带噪测量估计一个 probabilistic、continuous 的 implicit surface 表示。此外，`GPIS` 还可以不仅估计曲面本身，也连续地估计 distance field [575, 1033, 635, 1194]。

在 `GPIS` 表述中，可将待估计的 distance field `d`，以及由表面点与其对应梯度 `∇d_i` 通过线性算子得到的量，一起建模为一个零均值联合 `GP`。之所以可以采用零均值，是因为在表面上距离本来就等于零。书中写为

```text
[d; ∇d] ~ GP(0, K(X, X'))    (5.12)
```

`GPIS` 的优势在于，它能够连续地估计 implicit surface，并通过梯度估计 surface normal，而且这两者都带有 uncertainty。部分工作还进一步引入 parametric function priors，以便更准确地拟合某些给定形状 [735, 505]。

还有一些方法试图不仅恢复隐式曲面，还恢复完整的 distance field。原因在于，vanilla `GPIS` 形式下，distance 在测量附近，也就是靠近 surface 的区域，近似得较好；但在远离 surface 时，它会回退到均值，而这里的均值正好是零。为了在远离 surface 的区域也能持续地、概率化地估计完整 distance field，相关工作考虑在 `GPIS` 类模型上再施加一个 non-linear operation [1194, 1195, 633]。

这些方法都必须面对 `GP` 表述本身的计算复杂度，但作为交换，它们能够仅凭 point clouds 得到一种连续的、生成式的、概率化的环境表示。

### 5.4.6 Hilbert Maps

`Hilbert Maps` 与 `GPOM` 很像，都是连续概率地图，但目标是降低 `GPOM` 的高计算成本。其设计目标包括:

- 连续、在线处理数据
- 建模观测依赖关系
- 融合测量不确定性

做法上，`Hilbert Maps` 通常在一个投影后的高维特征空间中，用 logistic regression 配合 stochastic gradient descent 进行学习。这里的 feature projection 扮演了类似 kernel 的角色，但采用近似形式，例如:

- `Nystroem`
- `Random Fourier Features`
- `Sparse Kernel`

这样做的结果是，系统可以用一个相对简单的分类器去表达复杂空间结构，同时保持在线更新能力。训练时，常沿 measurement rays 采样 free-space points，并把回波点作为 occupied samples。

它的难点在于 kernel / feature projection 的表达能力与计算代价之间的权衡: kernel 太弱则重建细节不够，太强则代价升高。

### 5.4.7 建图中的深度学习

随着 `NeRF`、neural `SDF` 与各种 neural scene representations 的兴起，dense mapping 也越来越多地采用深度学习表示。其核心思想，是让神经网络在任意空间位置输出某种隐式 quantity，例如:

- `SDF`
- radiance
- density

这种表示天然连续，因此可以在任意分辨率下生成 mesh 或 render novel views，还可能填补测量缺失区域。

早期很多方法在离线设置下、使用已知 poses 训练；后续工作则开始尝试:

- 增量式更新 neural map
- 在线同时估计 pose 与 neural representation
- 把 voxel / octree / point features 与小型 neural decoder 结合

其优势是表达能力强、可与语义和新视角合成自然结合；代价则在于训练/推理复杂、工程系统更重。这个方向在 `NeRF` 与 `Gaussian splatting` 的推动下发展极快，后续章节还会更详细讨论。

## 5.5 使用层面的考虑

任何 map representation 都是在不同优点之间做权衡。因此，选择表示时不能只看“重建质量高不高”，还必须结合环境条件、传感器类型以及所有 downstream tasks。

### 5.5.1 环境因素

环境因素会直接影响地图表示的选择。

在高度结构化、受控的环境中，例如自动化工厂，针对特定对象与任务定制的表示往往比通用 dense representations 更高效、更准确。而在开放、变化、部分未知的环境中，通用稠密表示的优势更明显。

另一个关键点是，系统是否需要区分:

- 已观测自由空间
- 未观测空间

这对安全规划与探索非常重要。通常来说:

- explicit surface 表示很难天然区分 free 与 unobserved
- occupancy-based methods 天然具备这种区分能力
- 一些 implicit surface methods 也能保留这类信息，但很多重建导向系统会为了节省资源，把远离表面的部分信息丢掉

可扩展性也是环境相关的重要维度。explicit representations 通常只描述 surface 本身，因此在大场景中往往比 implicit volumetric representations 更省内存。而 multi-resolution methods 在大范围场景和 thin objects 上通常比固定分辨率 voxel maps 更有优势。

最后，动态环境也会深刻影响表示选择。现有方法大致分三类:

- 完全不建模 dynamics，直接把所有测量融合进地图
- 显式识别并剔除动态物体，只保留静态背景
- 同时建模背景与运动目标

第三类通常需要 hybrid representations，把本章介绍的基础几何表示与 object-level modeling 结合起来。

### 5.5.2 下游任务类型

除了环境，另一个决定性因素是地图将被用于哪些下游任务。

总体上，explicit 与 implicit representations 各有擅长:

- explicit representations 更适合沿 surface 做操作，例如渲染、表面分析、coverage planning、纹理处理
- implicit representations 更适合在 Cartesian space 中做查询，例如 collision checking、occupancy filtering、distance queries

这解释了为何:

- explicit maps 常能生成更细致、视觉上更漂亮的重建
- implicit maps 更适合处理 noisy depth data，并更常见于 planning 与 optimization 场景

在表面修改方面，explicit representations 也通常更容易做局部几何 deformation，例如在 loop closure 后直接拉回 dense map；而 implicit representations 则更便于处理影响 surface topology 的操作，例如合并、裁剪、布尔运算与简化。

很多系统会把两者结合起来，例如先用 `TSDF` 滤波和融合噪声测量，再用 `Marching Cubes` 抽出 mesh。这种 hybrid pipeline 往往能同时利用 implicit 与 explicit 的优势。

### 5.5.3 建图方法总结

本章最后对各种 dense mapping 方法做一个高层总结。

在 explicit representations 中:

- `points` 最简单、假设最少，但信息量也最低
- `surfels` 增加了局部 surface properties，在内存与几何细节之间平衡更好
- `meshes` 进一步显式表示 surface connectivity，最适合需要连通曲面的任务

在 implicit representations 中:

- `occupancy maps` 更新直接、对 thin obstacles 更保守
- `implicit surfaces` 与 `distance fields` 提供更平滑的几何信息和梯度，更适合 optimization-based planning
- 基于 `GPs` 与 `Hilbert Maps` 的方法，在稀疏或高噪声场景下往往能提供更好的不确定性估计与插值能力

而 learning-based methods，尤其是 `NeRF`、neural `SDF` 与 `Gaussian splatting`，正在迅速扩大 dense mapping 的能力边界，尤其在高保真重建、语义建模与空间推理方面展现出很大潜力。

## 第 6 章 SLAM 的可认证最优求解器与理论性质

### 本章概览

本章围绕两个核心问题展开。第一，典型 `SLAM` 问题是高维、非凸而且往往存在多个局部极小值的优化问题，为什么在大量实际系统中，它却经常能求得非常好的解？第二，即便我们得到了“最优解”，这个解本身在统计意义上能有多准确，它的误差下界由什么决定？

作者指出，这两个问题分别对应两条研究主线。第一条主线是 `certifiably correct optimization`，它试图用 `semidefinite relaxation` 等凸松弛工具，为原本非凸的 `SLAM` 问题构造可认证的最优求解器，使系统不仅“解出一个结果”，还能够判断这个结果是不是全局最优。第二条主线是从信息论和估计论角度研究误差下界，核心工具是 `Cramer-Rao Lower Bound (CRLB)` 与 `Fisher Information Matrix (FIM)`，并进一步把它们与 `factor graph` 或图拉普拉斯结构联系起来。

作者强调，这两条主线实际上是相互呼应的。前者揭示了 `SLAM` 问题的代数结构、几何结构和图结构如何让某些非凸问题在实践中仍可高效求解；后者则说明这些同样的结构如何决定一个问题在理论上“能够被估得多准”。这也是本章标题里“求解器”和“理论性质”并列出现的原因。

## 6.1 面向 SLAM 的可认证最优求解器

`SLAM` 通常被写成非凸优化问题。对一般非凸问题而言，全局最优求解在理论上非常困难，因为目标函数中往往存在多个局部极小值。`SLAM` 的很多重要特例，例如 angular synchronization、rotation averaging 和 `pose-graph optimization (PGO)`，都已经被证明是 `NP-hard`。这意味着，在最一般的情形下，我们不可能指望总有一个多项式时间算法能够保证求出全局最优解。

然而，实践经验却呈现出一个非常有趣的事实。早期 `SLAM` 研究表明，只要系统有一个还不错的初值，例如来自 wheel odometry 或其他里程计估计，局部优化往往能收敛到接近真实值的解。更进一步，后续研究发现，即便初始化质量并不理想，像 `Stochastic Gradient Descent`、`Levenberg-Marquardt` 和预条件共轭梯度这类方法，也常常能恢复出非常好的解。这说明 `SLAM` 虽然是非凸问题，但它内部存在某些特殊结构，使它比一般非凸优化更“可解”。

但作者同时提醒，这种经验规律并不能替代理论保证。局部方法仍可能卡在坏的局部极小值中，而且传统局部优化器通常没有能力告诉你“现在这组解到底离全局最优还差多远”。这直接关系到系统可信度，因为在很多机器人任务中，错误轨迹和错误地图会传递给规划与控制模块，造成下游失败。

因此，近年来一个非常重要的发展方向是构造 `certifiably correct` 的优化方法。这里的“可认证”并不是说算法在所有最坏情况下都能轻松求出全局最优，而是说它在大量实际问题上既能恢复全局最优解，又能输出一个最优性证书；即使在无法给出证书的情况下，算法也能给出一个明确的 suboptimality bound。换言之，这类方法最大的价值不仅在于“解问题”，还在于“知道自己解得对不对”。

### 6.1.1 Shor 松弛

本节首先介绍构造可认证估计器的基础工具之一: `Shor’s relaxation`。它本质上是针对 `Quadratically Constrained Quadratic Program (QCQP)` 的一种 convex relaxation 技术。所谓 `QCQP`，就是目标函数和约束函数都由二次形式构成的优化问题。作者指出，很多常见 `SLAM` 表述经过合适整理后都可以写成 `QCQP`。

为说明 `Shor` 松弛的核心思想，作者先考虑一个一般形式的 `QCQP`。设变量为 `x`，目标是最小化一个二次型 `x^T C x`，并满足一组二次约束 `x^T A_i x = b_i`。关键观察在于，任意二次型都可以通过迹运算重写成 `tr(M x x^T)` 的形式，因此整个问题中变量真正出现的方式，其实是外积矩阵 `X = x x^T`。

一旦把变量从向量 `x` 提升为矩阵 `X`，原问题就可以被改写成: 在满足若干线性迹约束、`X ⪰ 0` 以及 `rank(X)=1` 的条件下，最小化关于 `X` 的线性目标。这里面最难处理的部分，是那个非凸的秩一约束。`Shor’s relaxation` 的做法非常直接: 把这个秩约束删掉，只保留线性约束和半正定约束。这样得到的就是一个 `semidefinite program (SDP)`。

这个步骤带来了两个非常重要的后果。第一，`SDP` 是凸优化问题，因此理论上可以在多项式时间内求解。第二，由于我们通过“放宽可行域”的方式得到 `SDP`，所以 `SDP` 的最优值天然给出了原始 `QCQP` 最优值的下界。也就是说，如果你手头有一个候选解 `x̂`，并且通过 `SDP` 求得了一个最优值 `d*`，那么 `f(x̂) - d*` 就构成了该候选解 suboptimality 的实际可计算上界。

更理想的情况是，`SDP` 的最优解恰好是秩一的。如果发生这种情况，那么你就可以从这个秩一矩阵中直接恢复出向量形式的最优解 `x*`，而且这个 `x*` 就是原始非凸 `QCQP` 的全局最优解。作者强调，在许多机器人状态估计任务中，这种“松弛恰好精确”的情况远比最坏情形分析暗示的更常见。

当然，直接用通用 `SDP` 求解器处理大规模机器人问题仍然可能很慢、很耗内存。因此，在很多应用里，`Shor` 松弛只是一个理论起点。真正让它能用于大规模 `SLAM` 的，是后续围绕问题结构设计出来的专用求解器，例如下一节的 `SE-Sync`。

### 6.1.2 SE-Sync: 可认证正确的 Pose-Graph Optimization

这一节以 `pose-graph optimization (PGO)` 为例，展示如何从 `Shor` 松弛出发构造一个真正可用的 certifiable algorithm。作者选择 `PGO` 作为例子有两个原因。首先，`PGO` 是最经典、最常见的 `SLAM` 后端模型之一；其次，它也是最早被证明可以有效进行凸松弛的 `SLAM` 形式之一，因此非常适合作为展示整个思想链条的具体案例。

作者把 `SE-Sync` 的推导分成三个阶段。第一步，把 `PGO` 写成一个最大似然估计问题，并进一步化成 `QCQP`。第二步，对这个 `QCQP` 应用 `Shor` 松弛，构造出对应的 `SDP`，并分析在噪声较小条件下这个松弛为何是精确的。第三步，设计一个能够利用 `PGO` 结构、真正求解大规模松弛问题的高效算法。这三步合起来，才构成 `SE-Sync` 这个完整的可认证求解框架。

#### 6.1.2.1 Pose-Graph Optimization: QCQP 表述

`Pose-graph optimization (PGO)` 的任务，是在 `d` 维空间中估计一组未知位姿 `T_1, ..., T_n ∈ SE(d)`；在典型 `SLAM` 里，`d = 2` 或 `3`。给定的是这些位姿之间一组带噪声的相对位姿测量 `T̃_ij ≈ T_i^{-1} T_j`。在实际系统中，未知位姿 `T_1, ..., T_n` 描述的是机器人轨迹上离散时刻的 poses，而测量 `T̃_ij` 则来自 `SLAM front-end`，例如 `LiDAR scan matching`、轮式里程计或三维计算机视觉方法。作者在本小节的目标，是把这一估计问题形式化为最大似然估计；在合适的噪声假设下，最终得到的问题就是一个 `QCQP`。

为便于建模，作者先引入 `pose graph`。这里可将其看作 factor graph 的一个特殊情形：变量节点是 poses，factor 则关联成对 poses。具体地，令 `G = (V, E)` 为一个简单无向图，其中节点 `i ∈ V` 与未知位姿 `T_i` 一一对应，边 `{i, j} ∈ E` 与可用测量一一对应。作者进一步假设 `G` 是连通的；若图不连通，则 `PGO` 会分解为若干独立的连通分量子问题。接着，对每条无向边赋予一个方向，就得到 `pose graph` `G^→ = (V, E^→)`。按照约定，测量 `T̃_ij` 表示在位姿 `T_i` 的坐标系下对位姿 `T_j` 的带噪声观测，因此它对应一条从 `i` 指向 `j` 的有向边。

为了把 `PGO` 写成最大似然估计，作者为测量引入噪声模型。旋转部分使用 `isotropic Langevin distribution` `L(M, κ)`，它是在 `SO(d)` 上的一个指数族分布，其概率密度函数为

```text
p(R; M, κ) = 1 / c_d(κ) * exp(κ tr(M^T R))
```

其中 `M ∈ SO(d)` 和 `κ ≥ 0` 是参数，`c_d(κ)` 是归一化常数。这里 `M` 起到 location parameter（mode）的作用，`κ` 则是 scalar concentration parameter。作者指出，这个分布在二维和三维下还有一个特别直观的生成式描述：若 `R̃ ~ L(M, κ)`，则先采样旋转角 `θ ~ von Mises(0, 2κ)`；当 `d = 2` 时，令 `R̃ = M · R(θ)`；当 `d = 3` 时，令 `R̃ = M exp(θ v^)`，其中 `v ~ U(S^2)` 是均匀采样的旋转轴。直观上，可以把它理解为定义在旋转流形上的 Gaussian 类比。

在给定有向 pose graph `G^→ = (V, E^→)` 后，作者假设每个测量 `T̃_ij = (t̃_ij, R̃_ij) ∈ SE(d)` 都由如下概率生成模型得到：

```text
t̃_ij = t̄_ij + t^ε_ij,      t^ε_ij ~ N(0, τ_ij^(-1) I_d)
R̃_ij = R̄_ij R^ε_ij,       R^ε_ij ~ L(I_d, κ_ij)
```

其中 `T̄_ij = (t̄_ij, R̄_ij) ∈ SE(d)` 是位姿 `T_i` 与 `T_j` 之间真实但未知的相对位姿。也就是说，模型 `(6.8)` 假设平移测量 `t̃_ij` 受到零均值各向同性 Gaussian 加性噪声污染，其 concentration parameter 为 `τ_ij > 0`；旋转测量 `R̃_ij` 则受到以 `I_d` 为 mode、concentration parameter 为 `κ_ij ≥ 0` 的乘性各向同性 `Langevin` 噪声污染。作者也解释，之所以采用这一模型，而不是第 `2` 章中更一般的 Lie group 上 exponentiated-Gaussian 噪声模型，是因为它对应的最大似然估计具有特别简洁的代数形式。

对由上述模型采样得到的带噪测量 `T̃_ij`，直接计算可得其对应的最大似然估计为

```text
p*_MLE = min Σ_(i,j)∈E [ κ_ij ||R_j - R_i R̃_ij||_F^2 + τ_ij ||t_j - t_i - R_i t̃_ij||_2^2 ]
          t_i∈R^d
          R_i∈SO(d)
```

这就是书中的 `Problem 1 (Pose-Graph Optimization)`。特别要注意，式 `(6.9)` 中的目标函数本质上是一个简单的二次 least-squares loss；而在平面情形 `d = 2` 与三维情形 `d = 3` 下，约束 `R_i ∈ SO(d)` 也可以写成二次约束。因此，整个 `PGO` 问题可以被自然地表述成一个 `quadratically constrained quadratic program (QCQP)`。

#### 6.1.2.2 将 Shor 松弛应用于 Pose-Graph Optimization

作者在本小节展示如何对 `PGO Problem 1` 应用 `Shor` 松弛，并由此得到一个凸松弛。第一步是把最大似然估计整理成一个只含旋转变量的更紧凑形式，并明确暴露它与底层图 `G`、`G^→` 的对应关系。为此，作者引入了若干由图构造出来的矩阵，并专门回顾了代数图论中的 incidence matrix 与 Laplacian matrix。

具体而言，对一个有 `n` 个节点、`m` 条边的有向图，其 incidence matrix `A ∈ R^(m×n)` 每一行对应一条边，tail 节点处为 `-1`，head 节点处为 `+1`；而 Laplacian matrix 满足 `L = A^T A`，其对角元对应节点度数，非对角元在有边相连时为 `-1`。作者特别强调，Laplacian 的零特征值个数对应图的连通分量数，而连通图的第二小特征值还刻画图的 algebraic connectivity。

在这些基础上，作者定义了 translational weight graph `W^τ` 的加权 Laplacian `L(W^τ)`、由旋转测量和精度构造的 connection Laplacian `L(G̃^ρ)`，以及由平移观测构造的矩阵 `Ṽ`、`D̃` 与精度对角阵 `Ω`。然后把旋转状态聚合为 `R = [R_1 ... R_n] ∈ SO(d)^n`，平移状态聚合为 `t = [t_1^T ... t_n^T]^T`。

有了这些记号之后，作者观察到：一旦固定 `R_1, ..., R_n`，原问题 `(6.9)` 对平移变量就退化为一个线性最小二乘问题，因此可显式求出最优平移赋值

```text
t*(R) = - vec(R Ṽ^T L(W^τ)^†)                              (6.16)
```

把这个最优平移代回原问题，就得到只关于旋转矩阵的 `rotation-only PGO`：

```text
p*_MLE = min tr(Q̃ R^T R)                                   (6.17a)
```

其中数据矩阵 `Q̃` 虽然表面看起来可能稠密，但作者进一步指出，它可以利用由图结构诱导出的稀疏分解来高效处理。特别地，投影矩阵 `Π` 虽然 generically dense，却能通过式 `(6.18)` 写成只涉及稀疏下三角因子 `L` 的分解；这一点对后续实现高效求解器至关重要。

接着，作者开始构造松弛。首先把约束 `R ∈ SO(d)^n` 放宽到 `R ∈ O(d)^n`。由于正交矩阵可由一组二次正交归一约束定义，因此放宽后的问题是一个 homogeneous `QCQP`，可写成

```text
min tr(Q̃ R^T R)
s.t. BlockDiag_(d×d)(R^T R) = (I_d, ..., I_d)             (6.20)
```

然后就可以像一般 `QCQP` 一样，把 `R^T R` 替换成新的矩阵变量 `Z`，再去掉秩约束，得到 `PGO` 的 semidefinite relaxation：

```text
min tr(Q̃ Z)
s.t. BlockDiag_(d×d)(Z) = (I_d, ..., I_d),   Z ⪰ 0       (6.21)
```

作者指出，这个构造立刻蕴含两个结论。第一，松弛问题的最优值必然给出原始最大似然最优值的下界。第二，如果解出的 `Z*` 恰好 admit 一个 rank-`d` 分解 `Z* = R*^T R*`，并且 `R* ∈ SO(d)^n`，那么 `R*` 就是原始 `PGO Problem 2` 的全局最优解。

更关键的是，作者引用 `Theorem 6.1` 说明，只要噪声不太大，这个 favorable situation 在实践中确实会发生。也就是说，当 `Q̃` 与由 ground truth relative transforms 构造的 `Q̄` 足够接近时，`Problem 3` 的 `SDP` 松弛具有唯一解，并且该解可以写成 `Z* = R*^T R*`，其中 `R* ∈ SO(d)^n` 正是 `Problem 2` 的全局最优解。换言之，在噪声足够小时，解一个凸 `SDP` 就足以恢复原始非凸 `PGO` 的全局最优解。

#### 6.1.2.3 通过 Riemannian Staircase 高效求解松弛问题

虽然 `SDP` 在理论上很好，但直接对大规模 `PGO` 使用通用 interior-point solver 并不现实。问题在于，`Problem 3` 中的决策变量 `Z` 是一个高维稠密半正定矩阵，而机器人与视觉中的实例规模通常会比通用 `SDP` 求解器的有效范围大一到两个数量级。因此，作者在这里发展了一套专门利用问题结构的高效求解过程。

核心出发点，是 `Problem 3` 的解往往具有低秩结构。`Theorem 6.1` 告诉我们，在 exactness 成立时，目标解 `Z*` 可以写成非常紧凑的分解 `Z* = R*^T R*`；即便 exactness 不成立，`Problem 3` 的最优解在实践中也常常只有略高于 `d` 的秩。因此，可把 `Z` 直接写成对称低秩分解 `Z = Y^T Y`，其中 `Y ∈ R^(r×dn)`，这就是 `Burer-Monteiro` 因子化思路。

代入后，原 `SDP` 可改写为

```text
min tr(Q̃ Y^T Y)
s.t. BlockDiag_(d×d)(Y^T Y) = (I_d, ..., I_d)            (6.22)
```

与直接优化 `Z` 相比，若所选最大秩 `r` 足够小，即 `r << dn`，那么 `Y` 的搜索空间维度就远小于原始 `SDP` 的搜索空间。作者指出，这也是 `Burer-Monteiro` 方法能在工程上真正带来加速的根本原因。

更进一步地，把 `Y` 按列块写成 `Y = (Y_1, ..., Y_n)` 后，块对角约束就等价于 `Y_i^T Y_i = I_d`。这意味着每个 `Y_i` 的列向量都构成一个正交标架，而所有这样的正交标架集合正是 `Stiefel manifold`：

```text
St(k, p) = { Y ∈ R^(p×k) | Y^T Y = I_k }               (6.23)
```

因此，原来的等式约束非线性规划实际上等价于一个定义在 `St(d, r)^n` 上的无约束流形优化问题：

```text
min tr(Q̃ Y^T Y),   Y ∈ St(d, r)^n                      (6.24)
```

作者强调，这种几何重写带来重要计算收益，因为一旦意识到可行域是若干个 `Stiefel manifolds` 的乘积，就可以直接使用专门针对光滑流形设计的优化算法，而这些算法通常比一般等式约束的非线性规划更简单、更快、更准确。

接下来真正关键的问题是：既然 `Problem 4` 重新引入了非凸正交约束，那么我们是否只是把一个困难非凸问题换成了另一个？这里作者引用 `Theorem 6.2` 给出一个非常强的充分条件：如果某个 `Y ∈ St(d, r)^n` 是 `Problem 4` 的 row-rank-deficient 二阶临界点，那么它就是 `Problem 4` 的全局最优解，并且 `Z* = Y^T Y` 也是原始 `SDP Problem 3` 的解。

这个结果直接催生了 `Riemannian Staircase`。算法从较小的秩 `r ≥ d` 开始，对 `Problem 4` 运行二阶 `Riemannian` 优化以得到一个二阶临界点 `Y*`。如果 `Y*` 已经 rank-deficient，那么由 `Theorem 6.2` 可知它就是全局最优点，于是 `Z* = Y*^T Y*` 就是所需的 `SDP` 解；若 `Y*` 仍非 rank-deficient，则只需把 `r` 增大一级再试一次。由于当 `r ≥ dn + 1` 时任意 `Y ∈ R^(r×dn)` 都必然 row-rank-deficient，因此 `Riemannian Staircase` 一定会在有限步内终止；而在实践里，通常只需一两个“台阶”就足够。

作者最后指出，从工程角度看，`Riemannian Staircase` 本质上是一个包裹在快速局部 `SLAM` 求解器外层的轻量级 meta-algorithm。它保留了现代局部优化方法的速度，同时又为恢复全局最优解提供了理论保证。

#### 6.1.2.4 解的舍入

前面已经看到，`Riemannian Staircase` 可以高效恢复 `SDP` 解 `Z* = Y*^T Y*` 的一个低秩因子 `Y*`。但在 `PGO` 中，我们理想上真正需要的是原问题 `Problem 2` 的一个解 `R* ∈ SO(d)^n`。因此，当松弛精确时，我们希望从 `Z*` 中恢复全局最优的 `R*`；而当松弛不精确时，也至少希望从中得到一个可行的近似解 `R̂ ∈ SO(d)^n`。

作者强调，这个 rounding 过程可以直接在低秩因子 `Y*` 上完成，而不必显式构造稠密高维的 `Z*`。关键观察是：如果松弛 `(6.21)` 是精确的，那么一定有

`Y*^T Y* = Z* = R*^T R*` 。

于是 `Y* ∈ R^(r×dn)` 的秩实际上就是 `d`，因此可以通过对 `Y*` 做一个 thin `singular value decomposition (SVD)` 来恢复 `R*`。

更一般地，即便 `(6.21)` 不精确，仍然可以先对 `Y*` 做截断 `SVD`，得到它的最佳 rank-`d` 近似 `R̂ ∈ R^(d×dn)`；然后再把 `R̂` 的每个 `d×d` block 分别投影到 `SO(d)` 上，同样使用 `SVD` 完成。这样就能构造出一个满足原始旋转约束的可行近似解。

#### 6.1.2.5 SE-Sync: 完整算法

把第 `6.1.2.3` 节的高效 `SDP` 求解与第 `6.1.2.4` 节的 rounding 过程结合起来，就得到了 `SE-Sync (Algorithm 3)`，也就是作者提出的 certifiably correct 的 `PGO` 算法 [936]。

书中给出的算法形式非常直接。输入是某个初始点 `Y_0 ∈ St(d, r_0)^n`，其中 `r_0 ≥ d + 1`；输出则是原最大似然问题 `Problem 1` 的一个可行估计 `T̂ ∈ SE(d)^n`，以及原问题最优值的一个下界 `p*_SDP`。其主要步骤包括:

`1.` 用 `RiemannianStaircase(Y_0)` 求得低秩因子 `Y*`。
`2.` 由 `Y*` 计算松弛问题对应的最优值下界 `p*_SDP`。
`3.` 对 `Y*` 执行 `RoundSolution`，得到可行旋转估计 `R̂`。
`4.` 再利用式 `(6.16)` 恢复与 `R̂` 对应的最优平移 `t̂`。
`5.` 最终组合得到 `T̂ = (t̂, R̂)` 并返回。

当 `SE-Sync` 应用于某个具体 `PGO` 实例时，它返回的不只是一个可行点 `T̂ ∈ SE(d)^n`，还同时返回一个下界 `p*_SDP ≤ p*_MLE`。这个下界会立刻给出任意可行解 `T = (t, R)` 的 suboptimality 上界:

`F(Q̃ R^T R) - p*_SDP ≥ F(Q̃ R^T R) - p*_MLE` 。

更重要的是，如果 `Problem 3` 的松弛是精确的，那么 `SE-Sync` 返回的估计 `T̂ = (t̂, R̂)` 会恰好达到这个下界:

`F(Q̃ R̂^T R̂) = p*_SDP` 。

也就是说，只要在事后验证这个等式成立，就得到了一个关于 `T̂` 的计算性最优性证书。这正是 `SE-Sync` 被称为 certifiably correct algorithm 的原因。

作者最后提到，图 6.3 给出了若干 benchmark 数据集上的 certifiably optimal 结果，而文献 [936] 的 runtime 分析还表明，该算法在实践中甚至可以与传统局部求解器一样快，甚至更快。因此，`SE-Sync` 的价值不只是“能给出一个解”，而是“能高效地给出一个带全局最优性证书的解”。

### 6.1.3 基于地标的 SLAM

上一节已经说明如何为 `PGO` 构造一个快速且可认证的算法。本节进一步说明，同样的推导可以扩展到 landmark-based `SLAM`，特别是机器人对地标拥有 bearing 与 range，也就是相对位置测量的情形。

设 `m_(n+1), ..., m_(n+ℓ) ∈ R^d` 是除 `n` 个机器人 poses 之外还需估计的 `ℓ` 个 landmark 位置。一个关键观察是，这些新的地图点变量可以非常自然地并入 `SE-Sync`：只需把 map point 看成“只有平移、没有旋转的 pose 变量”即可。

作者假设我们拥有一组 landmark measurements `m̃_ik`，用于描述第 `i` 个机器人 pose 相对于 landmark `m_k` 的相对位置，并且这些测量与前面平移观测一样，受到加性、零均值、各向同性 Gaussian 噪声污染:

`m̃_ik = m̄_ik + ε^m_ik,   ε^m_ik ~ N(0, μ_ik^(-1) I_d)` 。

为追踪这些新测量，需要把原来的 pose graph 扩展成一个更大的 measurement graph：新增地标顶点集 `V_m` 与地标观测边集 `E_m`；同时把原来对应机器人相对位姿观测的边记为 `E_r`。在这种建模下，由 landmark measurements 与 relative pose measurements 共同定义的最大似然问题可以写成原书的 `Problem 5`:

`min Σ_(i,j)∈E_r [ κ_ij ||R_j - R_i R̃_ij||_F^2 + τ_ij ||t_j - t_i - R_i t̃_ij||_2^2 ] + Σ_(i,k)∈E_m μ_ik ||m_k - t_i - R_i m̃_ik||_2^2`  。`(6.29)`

这里前一项仍是 pose-pose 约束，后一项则是 pose-to-landmark 约束。

接下来的发展与前一节基本一致。最优旋转变量仍可通过运行 `SE-Sync` 获得，只不过代价矩阵 `Q̃` 必须纳入地图点带来的影响。具体而言，权重矩阵 `Ω` 与测量矩阵 `D̃` 需要被重新定义为:

`Ω = BlockDiag(Ω_τ, Ω_μ),   Ω_τ = Diag(τ_1,...,τ_Np),   Ω_μ = Diag(μ_1,...,μ_Nm)`  。`(6.30)`

同时，`D̃` 也需要按照测量图 `G = (V ∪ V_m, E_r ∪ E_m)` 的边类型进行扩展：对 `E_r` 中的边填入相对平移测量，对 `E_m` 中的边填入相应的 landmark measurement，而其余位置为零。投影矩阵 `Π` 也要改写，以适应新的 measurement graph。作者特别指出，连接 Laplacian `L̃(G^ρ)` 本身并不会因为新增地图点观测而改变。

一旦最优旋转求出，pose translations 与 map points 仍能像式 `(6.16)` 那样闭式恢复:

`[(t*)^T (m*)^T]^T = -vec(R* Ṽ^T L(W^τ)^†)`  。`(6.31)`

这里默认 pose translation 与 map point 已按合适顺序排列，且 `Ṽ` 与 `L(W^τ)^†` 也已按地标测量做相应扩展。

作者随后讨论了一个非常实际的问题: 在典型 `SLAM` 中，地图点数量往往远大于 pose 数量。此时，把 map variables 纳入 `SE-Sync` 后，算法瓶颈会转移到数据矩阵 `Q̃` 的构造上。已有工作表明，可以使用经典的 `Schur-complement trick`，把该矩阵的构造复杂度控制为对 landmark 数量线性增长 [465]。图 6.4(b) 也展示了这一点: 随着地标数量增加，真正随之增长的主要是 `Q̃` 的构造开销，而 `SE-Sync` 其他主要组件的计算代价并不会随地标数一起恶化。

最后，作者强调，引入 landmarks 不只是“多了一些约束”而已，它还会影响 `SDP relaxation` 的 exactness。更具体地说，增加地标数量，以及提升 pose-to-map measurement graph 的 connectivity，都会提高松弛保持精确的概率，也就是让一个问题在更高噪声水平下仍保持 exact relaxation。图 6.4(c) 展示了这种效应。因此，地图点不仅能提升估计精度，也会提高 certifiable solver 成功恢复并认证全局最优解的能力。

### 6.1.4 扩展内容: 距离测量、各向异性噪声与离群点

前面的讨论主要集中在 relative pose measurements 和 pose-to-landmark measurements 这类相对规范的观测模型上。本节作者进一步说明，类似思路还可以扩展到更复杂、更贴近真实系统的问题，比如仅有 range 的测量、带各向异性噪声的 landmark 观测，以及存在 outliers 的 robust estimation。

困难在于，这些问题往往不再自然地落在 `QCQP` 之中。换言之，`Shor` 松弛本身虽然强大，但它并不能直接覆盖所有 `SLAM` 模型。于是我们需要更一般的 convex relaxation 框架，尤其是针对 `Polynomial Optimization Problems (POPs)` 的 moment / `Lasserre` relaxation。

#### 6.1.4.1 面向 Range-Aided SLAM 的快速可认证算法

在 range-aided `SLAM` 中，系统除了相对位姿测量外，还拥有距离测量，例如机器人之间或机器人与地标之间的距离观测。典型硬件例子就是 `Ultra-wideband (UWB)` 无线电，它能很好地提供距离信息，但不给出方位或姿态信息。

这一问题可用于建模多种实际场景，从机器人对地标的距离测量，到多机器人之间的相对测距问题。难点在于，原始 range-only 目标中的项 `(‖t_j - t_i‖ - r̃_ij)^2` 并不是二次形式。将其展开后会出现未平方的范数项 `2 r̃_ij ‖t_j - t_i‖`，这正是它不能直接写成 `QCQP` 的根本原因。

为解决这一问题，作者介绍了一个非常巧妙的重参数化: 为每个距离测量引入一个辅助 unit vector `b_ij`，并令 `‖b_ij‖ = 1`。这样，原问题就可改写为

`‖t_j - t_i - r̃_ij b_ij‖^2` ，

也就是在优化中同时估计“距离对应的方向”。直观上，这等于把一条纯 `range` 观测转写成一个带有未知 bearing 的 `range-and-bearing` 观测。其优点是非常关键的: 新目标对未知量变成了二次形式，而额外加入的 `‖b_ij‖ = 1` 约束本身也是二次约束，于是整个问题重新落回 `QCQP` 框架。

经过这种变换后，问题又回到了适合 `Shor` 松弛的结构。进而，我们就可以像在 `PGO` 一样，对它进行半正定松弛，并使用 `Riemannian Staircase` 高效求解。作者提到代表算法 `CORA` 就是基于这一思路构造出来的快速可认证求解器。

不过，这里的几何结构比纯 pose-graph 情况更弱，因此 relaxation 的 exactness 也更加依赖图结构。特别是在 multi-robot 仅由 range measurements 连接、缺少机器人之间相对位姿观测的情况下，松弛往往不再总是精确。作者据此指出，图 connectivity 对可认证性和估计精度都有深刻影响，这一观察也自然过渡到了后面的第 6.2 节。

#### 6.1.4.2 超越 Shor 松弛的可认证算法: 各向异性噪声与离群点

到目前为止，我们回顾的可认证 `SLAM` 算法与快速求解器，都建立在“问题能够被改写为 `QCQP`”这一前提上。作者在这里转向一类更广泛的问题：带各向异性测量噪声的问题，以及含有 outliers 的问题。关键观察是，这些问题虽然严格来说已经不再是 `QCQP`，但往往仍可被写成 `Polynomial Optimization Problems (POPs)`，也就是目标函数与约束都是多项式函数的优化问题。正因为如此，`Shor` 松弛的一个更一般化版本，也就是 `moment relaxation` 或 `Lasserre relaxation`，就可以被用来为这些 `POP` 构造 `SDP` relaxation，从而把可认证算法推广到更广的一类 `SLAM` 问题上。

一个典型例子，是带各向异性噪声的 landmark-based `SLAM`。前面几节里，作者一直假设测量噪声是各向同性的；但现实中常见传感器，如 stereo camera、`LiDAR` 和 radar，通常都表现出明显的各向异性噪声。例如，stereo camera 对 landmark 位置的估计，往往在相机视线方向上的不确定性更大，这是 stereo matching 与 triangulation 误差共同造成的。更形式化地，landmark 测量模型变为

```text
m̃_ik = R_i^T (m_k - t_i) + ε_ik,   ε_ik ~ N(0, W_ik)
```

其中 `W_ik` 是一个各向异性协方差矩阵，也就是说它不能写成单位阵的标量倍数。在这种情形下，原先的 landmark-based `SLAM` `Problem 5` 需要推广为

```text
min Σ_(i,j)∈E [ κ_ij ||R_j - R_i R̃_ij||_F^2 + τ_ij ||t_j - t_i - R_i t̃_ij||_2^2 ]
    + Σ_(i,k)∈E_m ||R_i^T (m_k - t_i) - m̃_ik||^2_(W_ik)
```

这里 `E_m` 表示所有 landmark 测量对应的边集合，而对任意向量 `a` 与矩阵 `W`，记号 `||a||^2_W = a^T W a` 表示标准 `Mahalanobis` 平方范数。作者强调，这个看起来很小的改动，实际上影响非常大：如果 `W_ik` 是各向同性的，例如 `W_ik = μ_ik^(-1) I`，那么就可以像 `Problem 5` 那样利用 `ℓ2` 范数对旋转的不变性，把问题整理回 `QCQP`；但如果 `W_ik` 是各向异性的，问题就会变成 quartic，也就是包含四次多项式项。

另一类典型例子，是含有 outliers 的 `SLAM`。前面一直默认所有测量都只受零均值 Gaussian 噪声影响；但正如第 `3` 章所述，真实 `SLAM` 中某些测量会是 outliers，尤其是 loop closure 与 landmark 测量，错误的 place recognition 或 data association 都可能把错误观测送进后端。一个常见的应对办法，是在目标函数中加入 robust loss。比如，对 landmark-based `SLAM`，就可以把相应目标写成

```text
min Σ_(i,j)∈E_o [ κ_ij ||R_j - R_i R̃_ij||_F^2 + τ_ij ||t_j - t_i - R_i t̃_ij||_2^2 ]
    + Σ_(i,k)∈E_m ρ( sqrt(μ_ik) ||m_k - t_i - R_i m̃_ik||_2 )
```

其中 `ρ(·)` 是用来降低潜在 outliers 影响的 robust loss function。作者提醒，我们已经在第 `3` 章见过 `Black-Rangarajan duality`，它可以把这类 robust estimation 改写成带辅助变量的 least-squares 问题。以 truncated quadratic loss 为例，上式可被重写为

```text
min Σ_(i,j)∈E_o [ κ_ij ||R_j - R_i R̃_ij||_F^2 + τ_ij ||t_j - t_i - R_i t̃_ij||_2^2 ]
    + Σ_(i,k)∈E_m [ w_ik μ_ik ||m_k - t_i - R_i m̃_ik||_2^2 + (1 - w_ik) β^2 ]
subject to w_ik ∈ [0, 1]
```

其中 `β` 是 truncated quadratic loss 指定的最大 inlier error，而辅助变量 `w_ik` 则表明某条测量更像是 inlier 还是 outlier。作者指出，这个重写后的目标函数最高会出现三次多项式，而约束依然至多是二次函数。相关工作 [1220] 还表明，这一结论对多种 robust loss 都成立，也适用于 pose-graph optimization 与 multiple rotation averaging 等问题变体。

上面两个例子说明，大量 `SLAM` 问题都可被归入 `POP`。更一般地，它们可以写成如下标准形式：

```text
min p(x)
subject to h_i(x) = 0,  i = 1, ..., n_h
           g_i(x) <= 0, i = 1, ..., n_g
```

其中 `p`、`h_i`、`g_i` 都是关于变量 `x` 的实系数多项式。把问题写成 `POP` 并不会立刻带来计算上的便利，因为 `POP` 是一类非常一般、通常也很难求解的优化问题；但它的重要性在于，存在一套标准程序，也就是 `moment (Lasserre) relaxation`，能够系统地从 `POP` 构造出 `SDP` relaxation。而且，这套方法还能生成一个 relaxation hierarchy；在温和假设下，这个层级中的某一级会是精确的。

作者随后给出一个简化例子来说明其基本机制。设 `x = [x_1; x_2]`，并假设 `p(x)` 与 `h(x)` 的次数至多为 `4`，且这里为简化起见只考虑等式约束。若取 relaxation order `r = 2`，则先定义次数不超过 `2` 的 monomial 向量

```text
[x]_2 = [1; x_1; x_2; x_1^2; x_1 x_2; x_2^2]
```

再构造 moment matrix，即它的外积

```text
X_4 = [x]_2 [x]_2^T
```

关键在于，任何次数不超过 `4` 的多项式，都可以写成 `X_4` 各个条目的线性组合。因此，原问题就能被改写为一个关于矩阵变量 `X` 的线性迹优化问题：

```text
min tr(CX)
subject to tr(H_i X) = 0,  i = 1, ..., n_h
           X = [x]_2 [x]_2^T
```

这与 `Shor` 松弛完全类似：把约束 `X = [x]_2 [x]_2^T` 替换为 `X ⪰ 0` 与 `rank(X) = 1`，再去掉秩约束，就得到一个 `SDP relaxation`。不过，`moment relaxation` 还会额外加入一批冗余约束来提升松弛质量。这些约束一方面利用了 moment matrix 中重复条目必须相等这一事实，另一方面也利用了“若 `h_i(x) = 0`，则 `x_1 h_i(x) = 0`、`x_2 h_i(x) = 0` 也必须成立”这类关系。该方法还可以自然推广到不等式约束。更进一步地，对任意整数 `r ≥ 2` 都可以重复这一过程，得到一个规模更大但质量更高的 `SDP` 层级。经典结果表明，在温和假设下，某个有限阶的 moment relaxation 会变成精确松弛；而相关工作 [1220, 465] 还观察到，对带各向异性噪声和 outliers 的 `SLAM` 问题，这种松弛往往在较低阶就已经精确，甚至在只使用 monomial basis 的一个稀疏子集时仍能保持 tight，这进一步降低了所得 `SDP` 的规模。

不过，这里存在一个关键“代价”。与前面几节中出现的松弛不同，moment relaxation 得到的 `SDP` 通常包含大量冗余约束。这个差别看似细微，但会对求解器造成深刻影响。特别地，这类带冗余约束的 `SDP` 往往是 degenerate 的，而这会让 `Riemannian Staircase` 变得不再适用，原因有二：第一，这些问题通常不满足 constraint qualification 条件，而 `Riemannian Staircase` 的收敛分析正依赖这些条件；第二，degenerate `SDP` 往往存在无穷多个 dual solutions，这会给 `Riemannian Staircase` 的实现带来直接的计算障碍。换句话说，对于由 moment relaxation 典型产生的这类 degenerate `SDP`，`Riemannian Staircase` 已经不再是可行求解器。作者最后提到，近期文献已经开始针对 `POP` 的 moment relaxation 设计专门求解器 [1219]，但这些方法与局部求解器相比仍然偏慢。

## 6.2 SLAM 问题最优解的精度如何？

前一节讨论的是“如何求一个可验证的最优解”；本节讨论的是“即便得到了最优解，这个解理论上能有多准”。在很多机器人系统里，仅仅说“我的优化收敛了”是不够的。我们还希望知道，当前轨迹和地图与真实值之间的误差是否已经足够小，是否满足下游任务，比如 motion planning，对精度的要求。

作者从估计论角度回答这个问题，核心工具是 `Cramer-Rao Lower Bound (CRLB)` 与 `Fisher Information Matrix (FIM)`。更进一步地，本节还会说明，在一些典型 `SLAM` 问题中，`FIM` 与图的拉普拉斯结构之间存在直接关系，这使图 connectivity 成为预测估计精度的关键量。

### 6.2.1 Cramér-Rao 下界与 Fisher 信息矩阵

`Cramér-Rao Lower Bound (CRLB)` 给出了任何无偏估计器所能达到的最佳协方差下界。形式上，

`Cov(x̂) ⪰ I(x_true)^(-1)` 。

这里 `x̂` 是对真实参数 `x_true` 的任意无偏估计器，符号 `A ⪰ B` 表示 `A - B` 为半正定矩阵。右侧的 `I(x_true)` 就是 `Fisher Information Matrix (FIM)`，它定义为对 log-likelihood 梯度外积的期望:

`[I(x_true)]_(i,j) = E_z [ ∂ log p(z; x) / ∂x_i · ∂ log p(z; x) / ∂x_j ]` 。

在满足一定正则条件时，`FIM` 也可以写成 log-likelihood Hessian 的负期望:

`[I(x_true)]_(i,j) = - E_z [ ∂² log p(z; x) / (∂x_i ∂x_j) ]` 。

这个表达给出了一个很直观的解释: `CRLB` 实际上是在用真实参数附近、期望意义下 log-likelihood 的局部曲率，来刻画任何无偏估计器至少会有多大的不确定性。如果 log-likelihood 在真实值附近很平，那么任何无偏估计器都很难仅凭观测把参数准确“钉住”，于是方差会更大，`MSE` 也会更高。换言之，`FIM` 描述了观测对真实参数提供了多少信息。

作者还提醒，经典 `CRLB` 默认变量位于 Euclidean 空间；而 `SLAM` 常常需要估计 poses 与 rotations，因此其结果后来也被推广到了 `Riemannian manifolds` 与 matrix `Lie groups` [105, 95]。这时下界中通常会出现一个与参数空间曲率有关的附加项，不过在 `signal-to-noise ratio` 较高时，这个曲率项往往可以忽略。

在一定条件下，`maximum likelihood estimator` `x̂_mle` 是渐近无偏的，并且当测量数量趋于无穷时能达到 `CRLB`，也就是在所有无偏估计器中拥有最小方差。由于真实参数 `x_true` 未知，实践中往往用 `I(x̂_mle)` 近似 `FIM`，并进一步用 `I(x̂_mle)^(-1)` 近似 `x̂_mle` 的协方差。

一个很常见的情形是，测量来自某个光滑但可能非线性的函数，并叠加 Gaussian 加性噪声:

`z = h(x_true) + ε,   ε ~ N(0, Σ)` 。

此时，似然函数满足 `p(z; x) = N(h(x), Σ)`，代入上式后可得

`I(x) = J(x)^T Σ^(-1) J(x)` ，

其中 `J(x)` 是测量模型 `h` 在 `x` 处的 Jacobian。`SLAM` 文献中也经常直接把这个矩阵称作 `information matrix`。

作者随后指出，虽然 `FIM` 很有用，但它本身是矩阵；在系统设计、active `SLAM` 或测量选择问题中，我们通常希望把“不确定性”压缩成单个标量，以便比较与优化。为此，`Optimal Experimental Design` 中常使用若干标准标量准则: `D-optimality` 使用 `FIM` 的行列式衡量不确定性超体积，`A-optimality` 使用逆矩阵迹衡量平均方差，`E-optimality` 使用最小特征值对应最坏方向上的估计方差。这些谱函数分别对应“总体体积”“平均误差”和“最坏方向”三种不同的不确定性视角。

### 6.2.2 Fisher 信息矩阵与 Graph Laplacian

为了进一步理解结构与精度之间的关系，作者从一个简化的 `PGO` 设定入手。在这个设定里，机器人方向被视为已知，测量噪声是各向同性的。作者先解释一个总体直觉: 从 `FIM = J^T Σ^{-1} J` 可以直接看出，噪声协方差越大，`CRLB` 就越差；但 Jacobian 的结构如何影响误差下界，仅从这个公式本身并不容易看明白。于是，这一节的目标，就是把 Jacobian 和 `FIM` 的结构同底层图联系起来。

作者指出，所有 `SLAM` 问题天然都可以看成图问题: 顶点表示变量，边表示相对观测。因此，图的连通性本质上反映了测量冗余度。虽然根据“information never hurts”原则，额外加入测量总会让 `FIM` 在 `Loewner` 序下变大，但不同边带来的收益并不相同。直观上，闭合一个更大的 loop，往往比增加一条局部冗余边更能提升最终精度。

在具体推导里，作者考虑已知朝向的 `3D PGO`。第 `k` 条相对位移测量写成 `zk = Rik^T (ti - tj) + εk`，其中噪声满足 `εk ~ N(0, w_k^{-1} I3)`。把全部测量堆叠后，可得一个由 reduced incidence matrix `A`、边权矩阵 `W` 和 Kronecker 积构成的线性模型。此时 Jacobian 可以写成 `J = R^T (A ⊗ I3)^T`，而噪声信息矩阵则是 `Σ^{-1} = W ⊗ I3`。

代入 `FIM = J^T Σ^{-1} J` 后，作者一步步得到:

`I = (A^T W A) ⊗ I3 = Lw ⊗ I3`

其中 `Lw` 正是对应测量图的 reduced weighted Laplacian。经过这一推导，可以得到一个非常漂亮的结论: 在这种简化 `PGO` 中，`FIM` 完全由底层图的加权拉普拉斯矩阵刻画。也就是说，估计精度在本质上被图结构所决定。

作者进一步指出，这一关系并不只限于最简化的模型。在更一般的 `PGO` 和 landmark-based `SLAM` 中，`FIM` 虽然还会包含依赖于轨迹姿态的附加项，例如由 `Adjoint` 和单条边的 elementary Laplacian 组成的项，但 reduced weighted Laplacian 仍然扮演核心角色。实证和理论分析都表明，许多最优设计准则用图拉普拉斯来近似时，效果依然很好。

这种联系带来了非常实际的启发。例如，`D-optimality` 准则可以近似地与图中加权生成树数量联系起来，而 `E-optimality` 则与 algebraic connectivity 密切相关。直观地说，图越连通、冗余约束越多，系统可达到的估计精度通常越高。更重要的是，用 graph Laplacian 近似设计指标时，很多时候根本不需要真的解整个 `SLAM` 问题，也不需要拿到真实测量值，只要知道图结构就能做出有价值的精度预测。

这使得图拉普拉斯不仅是一个后验分析工具，也成为 active `SLAM`、measurement selection、measurement pruning 和 lifelong `SLAM` 中非常重要的规划工具。因为机器人可以在“还没做完整估计之前”，就大致评估哪些观测最有价值、哪些路径最能提升最终精度。

## 6.3 延伸阅读与最新趋势

在 certifiable algorithms 方面，作者回顾了过去十多年里这一方向的快速发展。最早的二维 `SLAM` 可认证算法可以追溯到 [149]，随后很快扩展到三维 `SLAM`、多种 `PGO` 变体以及其他几何估计问题。`SE-Sync` 不仅提供了一个高效求解器的 blueprint，也首次给出了在有界测量噪声下 relaxation 精确性的全局最优性保证。

之后，这一思路被推广到了 rotation averaging、multi-robot `SLAM`、range-aided `SLAM`、3D registration、multi-set registration、两视图几何、PnP、calibration 以及 pose-and-shape estimation 等问题。作者还特别提醒，其中有些优化问题其实是在 `SLAM` 前端中求解的，例如从传感器 measurements 恢复 relative poses。近年来还出现了处理 anisotropic noise 与 outliers 的 certifiable methods，说明这一方向的覆盖范围正在持续扩大。

但作者也指出，仍有三个非常重要的开放问题。第一，仍有不少 `SLAM` 问题很难直接纳入 certifiable pipeline。例如 visual `SLAM` 中的透视投影会带来有理函数目标，而不是简单多项式；虽然原则上可以通过引入额外变量把它们改写成 `POP`，但在每个 keypoint 都引入新变量通常是完全不现实的。同样，`IMU` 测量模型也尚未自然地导向高效的 certifiable algorithm。

第二，即便理论上有 `SDP` relaxation，可扩展性仍然是巨大挑战。某些问题可借助 `Riemannian Staircase` 获得高效求解，但更多情况下仍不得不依赖 interior-point methods 或专门设计的 ad hoc solvers。前者在前端小规模问题上很有效，但在大规模后端图优化上往往难以承受；后者虽然更 scalable，却通常仍比局部求解器慢得多。因此，近年来也有研究致力于通过压缩 monomial basis、减少约束数量等手段，让 `SDP` 更小、更快、更少退化。

第三，当前文献更多是在“给定一个具体实例后，为这一次实例计算最优性证书”，而不是从更根本的层面理解“到底什么时候 relaxation 会是精确的”。也就是说，我们还缺少一个统一理论，来预测 exactness 如何随着噪声大小、图结构甚至 outlier 比例而变化。

关于带 outliers 的问题，作者指出，虽然已有工作把 outlier-free certifiable algorithm 套在 `Graduated Non-Convexity` 的外循环里以获得经验鲁棒性，但近年来开始直接构造面向 robust estimation 的 certifiable methods。这些工作已经覆盖 `PGO`、rotation estimation、3D registration、absolute pose estimation 以及 pose-and-shape estimation 等问题。不过，带 outlier 的 moment relaxation 仍然很大、很慢，因此也有一些新工作转而尝试用局部方法求解，再利用 moment-relaxation 的洞见仅做“最优性检查”。

这里还存在一些尚未解决的挑战。一个问题是，目前很多 robust certifiable methods 默认 odometry backbone 是无异常值的，也就是说仅把 loop closure 或部分高风险观测包进 robust loss。这在很多实际系统中是可接受的，但在视觉 `SLAM` 中，当 feature tracking 崩溃时，odometry 自身也可能变成不可靠观测；在 multi-robot `SLAM` 中，不同机器人之间根本没有这样的干净 backbone。若把 odometry 本身也纳入 robust loss，当前 relaxations 往往会变松，精确性显著下降，而如何改进这些 relaxations 仍不清楚。作者也提示，关于这些挑战更深入的讨论可参考 [147]。

另一个更深的问题是: 即便 robust estimator 达到了“优化意义上的最优”，如果大多数测量本身都是 outliers，那最终得到的最优解仍然可能是严重错误的。作者据此提出，一个非常值得探索的方向，是设计能够恢复多个 hypothesis、同时保证至少有一个 hypothesis 正确的 certifiable algorithm，这与统计学中的 list-decodable regression 有很强关联。

最后，在 uncertainty quantification 方面，本章也指出了若干仍未解决的问题。传统协方差分析依赖已知且准确的 measurement covariance；一旦这些噪声协方差本身不准，`FIM` 和 `CRLB` 就会变得不可靠。近年来开始出现用学习方法估计 measurement covariance 乃至整个 measurement model 的研究，这与第 4 章讨论的 differentiable optimization 也有呼应。此外，在存在 outliers 的情况下，传统协方差分析往往无法反映实际的多峰或偏斜后验分布，因此如何为下游任务提供真正可信的不确定性量化，仍然是一个开放方向。

## 第 7 章 Visual SLAM（视觉 SLAM）

### 本章概览

从相机中同时重建世界和传感器运动，是一个被称为 `Visual Simultaneous Localization and Mapping` 的挑战，简称 `Visual SLAM` 或 `VSLAM`。由于 cameras 几乎无处不在、成本低廉且功耗高效，visual `SLAM` 在 autonomous robots、self-driving cars，以及 mixed / augmented reality 中都具有近乎无穷的应用潜力。

本章首先给出一些历史背景与术语（`7.1`），并概览一个典型 visual `SLAM` pipeline（`7.2`）。随后，作者回顾 visual `SLAM` 建模中的关键组成（`7.3`），再讨论 image alignment 与 `Bundle Adjustment (BA)` 的更高级主题（`7.4`）。之后，本章会介绍完整 visual `SLAM` 系统示例（`7.5`）、实时 `3D` 重建（`7.6`）、基于 depth cameras 的 `SLAM`（`7.7`），以及把视觉与 `IMU`、`GPS`、`WiFi` 等其他 sensing modalities 结合的优势（`7.8`）。最后在 `7.9` 中，作者会简要讨论新的研究趋势。

## 7.1 历史背景与术语

visual `SLAM` 的历史既建立在通用 `SLAM` 方法上，也深深扎根于 photogrammetry 与 computer vision 的长期发展之中。作者在本节并不是简单做一个年代列表，而是说明 visual `SLAM` 之所以会出现，正是因为离线重建、几何视觉与在线状态估计三条技术脉络逐渐汇合。

### 7.1.1 从摄影测量到 Bundle Adjustment，再到 Visual SLAM

现代摄影起源于 19 世纪初，而从照片重建世界的尝试也很快随之出现。法国军官 `Aime Laussedat` 常被视为 photogrammetry 的奠基人之一，他在 19 世纪中叶就开始用地面照片做地形测绘。随后 `Albrecht Meydenbauer` 推动了建筑测量中的摄影测量发展，`Sebastian Finsterwalder` 则在高山冰川重建中发展了航空摄影和 projective geometry 的应用。

在这一脉络中，`bundle adjustment` 的数学基础在 20 世纪初便已成形。`Otto von Gruber` 形式化了“从多张图像中同时恢复结构和运动”的 `BA` 框架，而后 `Hellmut H. Schmid` 与 `Duane C. Brown` 把这些方法真正搬到计算机上实现。也就是说，今天视觉 `SLAM` 里无比核心的 `BA`，其概念其实远早于计算机视觉这门学科本身。

与此同时，computer vision 社区中的 `Structure from Motion (SfM)` 研究不断成熟。早期工作常聚焦于双视图重建；到了 1990 年代和 2000 年代，研究重心转向多视图 `SfM`，并逐步形成一条经典 pipeline: feature extraction、correspondence estimation、minimal solvers 初始化相机构型、再用全局 `BA` 得到一致的重建结果。`SIFT`、`SURF` 等特征描述子以及 `RANSAC` 等鲁棒采样策略，都是在这一阶段逐步建立起来的关键组件。

visual `SLAM` 与 `SfM` 的区别，主要不在“几何模型”本身，而在“问题设置”。`SfM` 往往处理无序图像集合，目标是离线、高精度地恢复大规模场景；visual `SLAM` 则强调一台运动中的相机在在线、因果、实时条件下完成定位与建图。因此，能够只依赖过去图像做最优状态估计的 causal methods，是 visual `SLAM` 真正出现的先决条件。大约在 2000 年前后，第一批具备实时能力的 visual `SLAM` / online `SfM` 方法开始出现。

### 7.1.2 术语

作者随后区分了几个在文献中经常被交替使用、但其实各有侧重点的术语。

`Photogrammetry` 更强调从 2D photographs 中提取精确测量、空间信息和 3D 重建。它关注的是通过多视点重叠图像构造环境或物体的几何表示，因此长期用于测绘、测量和三维建模，也构成许多 visual odometry 与 `SLAM` 方法的基础。

`Bundle Adjustment (BA)` 是一种数学优化方法，用于细化 3D 重建结果。它会联合优化相机的位置、姿态、可能还包括相机内参，以及被观测三维点的位置，目标是最小化重投影误差，也就是图像中真实像素位置与 3D 模型投影位置之间的差异。它通常使用 `Gauss-Newton` 或 `Levenberg-Marquardt` 一类二阶方法来求解，因此对初始化质量和 outlier rejection 都很敏感。

`Structure from Motion (SfM)` 指的是从多张不同视角图像中恢复三维结构与相机运动的过程。与 photogrammetry 经常假设部分相机位姿已知不同，`SfM` 通常同时估计三维结构和相机运动，而且更常见于非因果、非实时的环境，比如互联网图片集合的离线重建。作者还提到像 `Building Rome in a Day` 这样的 landmark project，说明 `SfM` 在大规模场景上已具有强大可扩展性。

`Visual Odometry (VO)` 则更聚焦局部运动估计。它通过分析连续视频帧之间的视觉对应关系，估计相机在短时间内的相对运动。它通常工作在一个 recent observations 的滑动窗口中，不构建显式全局地图，也不处理长期回环约束。不过，实际系统常把 `VO` 与 loop closure detection 以及 mapping 组合起来，从而形成完整 visual `SLAM`。

`Visual SLAM` 则是这一切的综合体。它不仅要在未知环境中实时估计相机自身位置，还要实时构建地图，并具备识别曾到访地点、在跟踪失败后重新定位，以及通过 loop closure 对全局轨迹做校正的能力。因此，visual `SLAM` 是 robotics、autonomous driving、AR/VR 等系统中最关键的空间感知技术之一。

## 7.2 视觉 SLAM 系统的处理流水线

一个完整的 visual `SLAM` system 需要把多个组件组合成一个可以实时运行、可扩展且具有鲁棒性的整体。作者强调，这并不是简单地“先做 tracking、再做优化”，而是要在实时性、计算开销与长期一致性之间 carefully 地安排模块职责和数据流。

与 `LiDAR` `SLAM` 相比，visual `SLAM` 的一个根本困难是三维几何并不是直接测得的。相机看到的是场景辐照度在图像平面上的投影，因此同一张图像里没有直接的三维距离信息，所有深度都必须通过多帧几何关系间接恢复。这使 visual `SLAM` 的整体估计问题比 `LiDAR` 更间接，也更依赖模型、初始化与数据关联质量。

典型 visual `SLAM` 系统一般包含三个核心子功能: 一个 odometry front-end，一个 mapping back-end，以及一个 visual place recognition / relocalization 组件。下面作者逐一说明它们的作用。

### 7.2.1 视觉里程计前端

visual `SLAM` 的核心前端模块是 visual odometry，它负责估计相邻相机帧之间的相对运动，并为后续后端优化提供初始姿态估计。

作者把视觉里程计方法概括为两大类。第一类是 feature-based 方法。它通常分三步进行: 先在图像中检测并提取特征点，再在不同图像间建立 feature correspondence，最后通过最小化相机运动和三维点位置上的重投影误差来恢复相对位姿。最后一步在本质上与经典 `BA` 非常接近。

第二类是 direct 方法。它不显式地把问题拆成“特征提取 + 特征匹配 + 几何优化”三步，而是直接定义一个 photometric loss，对相机运动和 3D 结构同时优化。因此，direct 方法与 optical flow、photometric `BA` 等思想有很强联系。

从历史经验上看，feature-based 方法在最初形式下通常具有更大的 basin of convergence，因为它们先显式建立了数据关联，优化时不必直接面对原始亮度空间中的高度非凸残差。为了缓解这个问题，direct 方法常采用 coarse-to-fine strategy，比如先用下采样图像对齐，或者先对 dense learned features 对齐，而不是直接对亮度做优化。

### 7.2.2 建图后端

后端的任务是利用全局优化技术，例如 `BA` 或 `pose-graph optimization`，对轨迹和地图做整体一致化。前端提供的是局部运动估计和中间观测；后端则在更长时间尺度上整合这些信息，并吸收由 place recognition 带来的长程约束。

通过后端优化，可以显著减小长期漂移，改善大尺度结构一致性，并在重复访问区域时把局部估计重新拉回全局一致的地图之中。

### 7.2.3 视觉地点识别与重定位

单纯 visual odometry 会随着时间推移不断累积 drift。若没有像 `GPS` 这样的绝对定位传感器，系统就必须靠 place recognition 来重新建立当前帧与历史图像之间的联系，从而消除漂移并恢复全局一致性。

实现上，这通常意味着要在一个可能非常大的图像数据库中进行 correspondence search。早期方法依赖 `SIFT`、`SURF`、`BRIEF` 等传统 descriptor；近年来则越来越多地使用训练好的 neural retrieval network。这个模块的结果，一方面用来检测 loop closure，另一方面在 tracking 失败时可以把系统 relocalize 回既有地图。

### 7.2.4 计算与数据流

作者最后从系统角度强调，data flow 的设计对 visual `SLAM` 的性能同样至关重要。

首先是 `pipeline parallelism`。tracking、mapping 和 optimization 往往并行运行，以充分利用算力并降低整体时延。其次是 `data sharing`。诸如 keypoints、poses 和其他中间结果，通常需要在多个模块之间共享，避免重复计算。最后是 `adaptive scheduling`。像全局优化这种计算代价大的任务，往往不是每帧都运行，而是根据系统状态按需调度，以优先保证实时性。

## 7.3 视觉 SLAM 基础

在前面给出整体 pipeline 之后，作者开始逐步拆解 visual `SLAM` 的基本技术元素。这些要素决定了系统如何建模相机、如何从图像中提取信息、如何定义残差以及如何初始化和表示地图。

### 7.3.1 相机模型

一个用于 visual `SLAM` 的相机模型，至少需要包括两部分。第一部分是 geometric component，也就是 projection model，用来描述三维点如何映射到二维像素。第二部分是 photometric component，用来描述物理光强如何变成相机输出的像素值。

#### 7.3.1.1 几何相机模型

作者先从最一般的写法开始。设 `x^c = [x, y, z]^T ∈ R^3` 是以相机坐标系表示的三维点，`z = [u, v]^T ∈ Ω ⊂ R^2` 是图像中的二维投影像素，`ξ ∈ R^n` 则是相机的内参集合。那么 projection function 可以写为 `z = π(x^c, ξ)`。与之对应的 unprojection `x^c = π^{-1}(z, ξ)` 则从 `2D` 像素恢复一条 `3D` 射线，但其深度仍未知，因此只能恢复到尺度不定的方向向量。

在实践中，不同镜头几何和相机类型会对应不同的 projection functions。最基础的是 rectilinear model，也就是常说的 pinhole / perspective camera model。它适用于窄视场且畸变较小的镜头，可写为

```text
ξ_p = [f_u, f_v, u_0, v_0]^T
π_p(x^c, ξ) = [f_u x / z + u_0, f_v y / z + v_0]^T
π_p^(-1)(z, ξ) ~ [(u - u_0)/f_u, (v - v_0)/f_v, 1]^T
```

其中 `f_u` 与 `f_v` 分别是水平和垂直方向上的 focal length（以 pixel 为单位），`u_0` 与 `v_0` 是 principal point。若像素近似为方形，通常可认为 `f_u ≈ f_v`。

为了描述镜头畸变，最常用的扩展是 radial-tangential distortion model。书中给出其形式为先令

```text
[x', y']^T = [x/z, y/z]^T
r = sqrt(x'^2 + y'^2)
```

再写出畸变后的归一化坐标

```text
x'' = x'(1 + k_1 r^2 + k_2 r^4) + 2 p_1 x' y' + p_2 (r^2 + 2 x'^2)
y'' = y'(1 + k_1 r^2 + k_2 r^4) + p_1 (r^2 + 2 y'^2) + 2 p_2 x' y'
```

最后再通过 pinhole 形式投到像素平面。作者特别指出，对这种模型通常无法解析写出 unprojection `π_RT^(-1)(z, ξ)`，因为无法从 `x''`、`y''` 解析求回 `x'`、`y'`，也就是 undistortion 一般没有闭式解，因此往往需要像 `Newton-Raphson` 这样的迭代方法。

对于更大视场的 wide-angle 和 fisheye cameras，还会使用专门模型。对视场接近但不超过 `180°` 的 wide-angle lenses，带少量径向畸变系数的 pinhole 模型通常已经足够；`Brown-Conrady (BC)` 模型 [283] 就相当于去掉 tangential coefficients 的 radial-tangential 模型，即只保留径向项。尽管相对 `RT` 模型更简单，但它同样没有解析 undistortion。

图 `7.1` 说明，视觉 `SLAM` 系统的一个核心要求，是选择合适的镜头。图中展示了 `BF2M2020S23 (195°)`、`BF5M13720 (183°)`、`BM4018S118 (126°)`、`BM2820 (122°)` 以及一款 `GoPro` 替换镜头（`150°`）。鱼眼镜头与广角镜头能够提供更大的视场，但也需要配套的投影模型。常见选择包括 `Brown-Conrady (BC)` 模型 [283]、`Kannala-Brandt (KB)` 模型 [536] 以及 `Double Sphere (DS)` 模型 [1116]。六参数 `DS` 模型在重投影精度上可与八参数 `KB` 模型相当，但其投影函数计算速度大约快五倍。（©2018 IEEE）

对于视场超过 `180°` 的 fisheye lenses，常用的是 `Kannala-Brandt (KB)` 模型 [536]。它把入射光线参数化为角度 `θ` 与 `ψ`，并用四个畸变系数对径向项 `r(θ)` 建模，然后写成

```text
π_KB(x^c, ξ) = [f_u r(θ) cos ψ + u_0, f_v r(θ) sin ψ + v_0]^T
r(θ) = θ + Σ_(n=1)^4 k_n θ^(2n+1)
```

其中 `θ = arctan(sqrt(x^2 + y^2) / z)`，`ψ = arctan(y / x)`。

另一个在 fisheye 与 wide-angle 场景里非常流行的选择，是 `Double Sphere (DS)` 模型 [1116]。作者强调，它在 accuracy 与 runtime 之间提供了很好的折中。`DS` 的思想是先把点依次投到两个球面上，这两个球心之间相隔参数 `γ`，再通过一个由 `α/(1-α)` 控制的偏移 pinhole model 投到图像平面。书中也特别指出，`DS` 模型具有 closed-form unprojection，因此在实时系统中非常有吸引力。根据 [1116] 的实验，六参数 `DS` 模型在保持与八参数 `KB` 模型相近 reprojection error 的同时，projection function 的计算速度大约快五倍。

#### 7.3.1.2 光度模型

photometric calibration 描述的是 irradiance 如何变成像素值，即 `I = f(E, T)`。这里 `T` 可以包含曝光时间、模拟增益、数字增益、gamma correction、去 Bayer 处理以及 lens vignetting 等影响成像亮度的因素。

作者指出，若系统主要做稀疏几何追踪，这部分通常不需要非常精细；但对于 dense textured mapping 或依赖 photo-consistency 的 direct methods，它就会直接进入损失函数，因此变得非常重要。

#### 7.3.1.3 时间相关效应

如果相机在曝光期间处于运动状态，那么成像过程本身就会受到时间效应影响。多数消费级相机使用的是 rolling shutter，这意味着图像不同行是在不同时间顺序采样的；而 global shutter 会在同一时刻捕获所有像素行。

作者因此建议，在机器人感知场景中最好使用 global shutter camera；若必须使用 rolling shutter，就应在模型中显式考虑图像行级别的 pose 变化。visual-inertial system 在这里有额外优势，因为 `IMU` 可以在曝光期间提供局部运动信息，从而让 rolling shutter 建模更实用。

#### 7.3.1.4 实践考虑

从实践上讲，选择相机模型的首要原则，是让参数化模型足够准确地逼近真实传感器与镜头系统的行为。使用 fisheye lens 时，最好选择球面类模型；使用 rectilinear lens 时，则应选择合适的线性基模型。

更普遍地说，相机模型选择本质上是计算效率与建模精度之间的平衡。如果模型选得不合适，就会引入系统性偏差，严重削弱 visual `SLAM` 的准确性与鲁棒性。

### 7.3.2 关键点

对 feature-based visual `SLAM` 来说，系统能否稳定建图和定位，很大程度上取决于能否在图像中检测、描述并匹配稳定的关键区域，也就是常说的 keypoints 或 features。

图 `7.2` 给出了一条时间线，回顾若干塑造视觉 `SLAM` 与图像匹配中文献脉络的代表性关键点算法。

一个理想的 keypoint detector 应具备 repeatability 和 distinctiveness，也就是说它在不同视角、不同光照、不同运行时刻下仍能被重复检测到，并且其局部外观足够独特，不容易和其他不相关区域混淆。相应的 descriptor 则要把局部邻域编码成紧凑、稳定而有判别力的表示。

现实场景中的难点在于: 光照变化、低纹理表面、动态物体和重复纹理都会使 detection 与 matching 出错，而这些错误又会直接伤害 `SLAM` 后端。因此，关键点设计长期围绕 scale invariance、rotation invariance、repeatability、distinctiveness 与 computational efficiency 几个目标演化。

#### 7.3.2.1 经典关键点检测器与描述子

经典手工设计的 keypoint detection 与 description 技术，在 feature detection 的演进中起到了奠基作用。`Harris-Stephens` keypoint detector [432]，也就是广为人知的 `Harris corner detector`，通过分析图像 patch 内 second-moment matrix 的特征值来检测 corners: 当两个特征值都较大时，说明图像在两个正交方向上都存在强烈强度变化，因此该 patch 可被视为 keypoint。`Shi-Tomasi` corner detector [1003] 建立在同一原理上，但直接用较小特征值做 corner selection；相比之下，`Harris` 使用一个 `cornerness` 响应函数来近似这一过程，以获得更高效率。尽管 `Harris` detector 鲁棒且计算高效，但它缺乏 scale invariance，这正是后续 `SIFT`、`SURF` 等方法试图解决的问题。

一个里程碑式方法，是 `Scale-Invariant Feature Transform (SIFT)` [698]。它在精度与召回率上都树立了新标准，对尺度与旋转高度不变，并对光照变化部分不变。`SIFT` 采用一个结构化的三阶段流程: `1.` 在 scale-space `Difference of Gaussians (DoG)` 金字塔中寻找 extrema，并通过三维二次拟合精细化 keypoint localization；`2.` 根据局部图像梯度的主方向为 keypoint 指派 orientation；`3.` 由离散梯度方向直方图构造一个 `128` 维 descriptor。不过，这种高鲁棒性也带来了较高计算代价，因此在实时应用中通常偏重。

为了提升效率，`Features from Accelerated Segment Test (FAST)` [945] 被提出为一种高速 corner detector。它检查候选像素周围圆形邻域内的强度分布，并使用 early rejection strategy 来减少计算量；随后又通过 decision tree 与 non-maximum suppression 进一步提速。但和 `SIFT` 不同，`FAST` 缺乏尺度不变性，因此对较大尺度变化会更敏感。之后，`Speeded-Up Robust Features (SURF)` [63] 通过 integral images 与 box filters 来近似 `SIFT` 中的 Gaussian derivatives，从而在速度与鲁棒性之间取得了更好的平衡。

对“又快又稳”的 descriptor 的需求，又推动了 binary-based 方法的发展。`Binary Robust Independent Elementary Features (BRIEF)` [141] 用二值强度比较构造紧凑 descriptor，并借助 `Hamming distance` 完成极快的 descriptor matching。虽然 `BRIEF` 本身不具备 scale 与 rotation invariance，但它表明 `SIFT` 鲁棒性所依赖的局部梯度信息，实际上可以通过更简化的 binary tests 来有效捕获。受此启发，`ORB` [952] 建立在 `FAST` 检测与 `BRIEF` 描述之上，并通过 image pyramid 和 intensity centroid 方法补上了尺度与旋转不变性。几乎同时提出的 `BRISK` [642] 则在 scale-space pyramid 上使用 `FAST` 或 `Harris` corners，并像 `SIFT` 一样通过 dominant keypoint direction 把 rotation invariance 纳入 binary descriptor。

作者特别指出，`ORB`、`BRISK` 这类方法虽然为了 scale / rotation invariance 牺牲了一点 descriptor distinctiveness，但它们在 visual `SLAM` 与实时机器人应用中已经证明非常有效。相反，如果 computational cost 不是主要限制，例如更偏离线的 computer vision 任务，那么 `SIFT` 与 `SURF` 往往仍更受青睐，因为它们在极端光照变化或视角变化下仍然更稳健。总体来看，经典手工 keypoint methods 在 robustness、efficiency 与 invariance 三者之间做了长期而细致的平衡；但随着环境越来越复杂、动态性越来越强，对更自适应 keypoints 的需求最终推动了后来的 learning-based 路线。

#### 7.3.2.2 基于深度学习的关键点检测与描述

到 `2010` 年代后期，基于深度学习的 image keypoints 方法开始快速发展。它们利用大规模数据集与 `Convolutional Neural Networks (CNNs)`，直接从数据中学习稳健特征。与人工设计的 keypoints 不同，这类方法会自动发现并优化 feature representation，因此在许多以往很难处理的场景中表现出更强的适应性与精度。一个典型例子是极端光照变化下的稳定关键点检测，这正是传统手工方法较为薄弱的场景。因此，visual `SLAM` 社区也越来越多地从传统 feature engineering 转向 data-driven representation learning。

`Learned Invariant Feature Transform (LIFT)` [1244] 是最早的一批代表方法之一。它把 keypoint detection、orientation estimation 和 descriptor computation 整合进一个端到端可训练 pipeline 中。`LIFT` 使用 `CNN` 从小图像 patch 中提取特征，并通过“先训练 descriptor，再训练 orientation estimation，最后训练 keypoint detection”的顺序学习策略，获得了比经典方法更强的尺度、光照和旋转鲁棒性。

`SuperPoint` [267] 则提出了一个 self-supervised 框架，可以在单次前向传播中同时完成 keypoint detection 与 descriptor computation，因此更适合实时应用。与基于 patch 的网络不同，`SuperPoint` 直接在整张图像上工作，并使用 `homographic adaptation` 这种自监督技术来生成 pseudo-ground-truth keypoints。作者指出，这使它在 illumination changes 下的 keypoint repeatability 与 descriptor quality 都显著优于 `SIFT`、`ORB` 和 `FAST` 等传统方法。

在此基础上，`HF-Net` [969] 进一步提出了分层定位思路，把 global image retrieval 与精确的 local feature matching 结合起来。它在一个统一的 `CNN` 中同时集成 keypoint detection、local descriptors 和 global descriptors，从而在保持高鲁棒性的同时降低计算代价。由于能先通过 global retrieval 缩小匹配图像集合，`HF-Net` 在 large-scale `SLAM` 和实时定位中尤其有效，即便在夜间等极端 appearance variation 下仍有较好表现。书中还指出，`HF-Net` 学到的特征通常比 `SuperPoint` 更稀疏，但判别力更强，因此被 `DX-SLAM` [648] 等深度学习 `SLAM` 系统采用。

另一个有意思的方向是 `D2-Net` [295]。它采用的是 `describe-and-detect` 而不是传统的“先检测再描述”顺序：先用 `CNN` 计算 dense feature maps，再把这些特征图中的局部极大值当作 keypoints。这样做可以引入更高层的语义信息，因此在极端光照变化和弱纹理环境中更加稳健。与把 detection 与 description 分离的 `SuperPoint` 不同，`D2-Net` 会联合优化这两项任务，从而提高 descriptor consistency。不过，这种 dense approach 的代价是计算量明显大于经典稀疏方法。即便如此，它在 visual localization 与 `SfM` 任务中仍表现很强，也推动了 deep-learning-based feature extraction 的边界。

总体来看，学习式方法在 matching recall 和外观变化鲁棒性上通常优于传统手工特征，但代价是更依赖训练数据、更依赖推理算力，并且在跨域泛化上仍可能出现问题。因此，在实际 visual `SLAM` 系统中，是否使用 learned features 仍是精度、速度、资源占用和泛化能力之间的综合权衡。

### 7.3.3 重投影误差

视觉 reprojection error 用于度量观测图像点 `z_j ∈ R^2` 与重投影点 `π(x_j^c, ξ) ∈ R^2` 之间的差异:

```text
e_reproj = z_j - π(x_j^c, ξ)
```

其中 `z_j` 是观测到的二维图像点，`x_j^c ∈ R^3` 是相机坐标系中的三维点，`ξ` 是相机内参，`π` 是投影函数。若假设观测图像点 `z_j` 受到 Gaussian 噪声扰动，则似然函数可写为

```text
p(z_j | x_j^c, ξ) ~ N(π(x_j^c, ξ), Σ_j)
```

其中 `Σ_j` 是图像中该特征位置噪声的协方差；在最简单情况下，常取各向同性且常值的 `Σ_j = σ^2 I`。最大化一组观测的似然，等价于最小化其负对数似然:

```text
L = - Σ_j log p(z_j | x_j^c, ξ)
  = (1/2) Σ_j || z_j - π(x_j^c, ξ) ||^2_(Σ_j) + const
```

去掉常数项后，就得到一幅图像内全部观测点的加权平方 reprojection error:

```text
E_reproj = (1/2) Σ_j || z_j - π(x_j^c, ξ) ||^2_(Σ_j)    (7.1)
```

为处理 outliers，通常还会在其外层套用 `Huber` 或 `Tukey` 等 robust kernels（参见第 `3` 章），例如

```text
E_robust = (1/2) Σ_j ρ( || z_j - π(x_j^c, ξ) ||^2_(Σ_j) )    (7.2)
```

其中 `ρ(·)` 是削弱大残差影响的鲁棒核函数。作者随后说明，为了简化后续记号，下面常省略对相机内参 `ξ` 的显式书写。

### 7.3.4 基于关键点的视觉 SLAM

基于关键点的 visual `SLAM` 的核心，就是最小化 reprojection error。设环境点 `P_j ∈ P` 被一组移动相机采集到的图像 `C_i ∈ C` 所观测。为简化记号，只保留点索引 `j ∈ P` 与相机索引 `i ∈ C`。视觉 `SLAM` 的目标，是估计世界参考系 `F^w` 中的点坐标 `x_j^w ∈ R^3`，以及相机位姿 `T_i^w ∈ SE(3)`。在 monocular 情况下，每个点在每幅图像中的观测就是图像坐标 `z_ij ∈ Ω_i ⊂ R^2`。

图 `7.3` 展示了基于特征的视觉 `SLAM` 问题：给定每幅图像中匹配到的一组特征（左），估计与之对应的三维点位置，以及每幅图像采集时的相机位姿（右）。图像由 `ORB-SLAM` [786] 生成。

当把所有点和所有相机位姿的重投影误差同时最小化时，就得到 full `Bundle Adjustment (BA)`:

```text
{T_i^{w*}, x_j^{w*} | i ∈ C, j ∈ P}
  = argmin_(T_i^w, x_j^w) (1/2) Σ_(i,j)
      ρ( || z_ij - π(R_i^w x_j^w + t_i^w) ||^2_(Σ_ij) )    (7.3)
```

围绕这一优化问题，文献中主要形成了几类计算复杂度与鲁棒性权衡不同的方法:

- `Batch Optimization`：对全部观测、全部位姿和全部地标反复迭代求解，也就是直接求解式 `(7.3)`。常用最小化算法包括 `Gauss-Newton (GN)` 和 `Levenberg-Marquardt (LM)`。这是 `SfM` 中的金标准，但在实时 `SLAM` 中若对每一帧都做 full `BA`，代价通常过高。
- `Filtering-Based Approaches`：使用 `Extended Kalman Filter (EKF)` 或 `Information Filter` 这类顺序、因果方法做实时状态估计。它们通过只保留最近的相机位姿来简化问题，但代价是破坏了图的稀疏性，因此通常只能处理几百个 features；同时，滤波法不会对过去观测重新线性化，精度也会受限。
- `Keyframe-Based Approaches`：只保留少量图像作为 keyframes [582]，中间图像及其观测不再用于地图估计。在相同计算预算下，这类方法通常比滤波法能构建更长、更准确的地图 [1038]。
- `Factor Graphs`：上述方法都可以统一表述为 factor graph。图中的节点对应位姿、地标等变量，factors 对应 reprojection errors 以及相机位姿或标定参数上的先验，这一点在本书第一部分已有详细讨论。

图 `7.4` 给出了一个包含 `4` 个相机位姿和 `5` 个特征的视觉 `SLAM` 示例：Bayes 网络（上）及其对应的马尔可夫随机场（中）。`EKF SLAM` 会边缘化历史相机位姿，从而得到稠密图（左下）；关键帧 `SLAM` 则仅保留少量相机位姿，并丢弃中间图像的观测，从而保持问题的稀疏性（右下）[1038]。

### 7.3.5 光度误差与直接法

Photometric error 为 reprojection error 提供了一种替代思路: 不再只比较离散特征位置，而是直接最小化观测图像区域与投影图像区域之间的像素强度差。这种方法建立在 photometric consistency 原则之上，即假设连续帧中相互对应的像素在光照条件一致时来自同一个场景点。

### 7.3.6 视觉地点识别与全局定位

Visual Place Recognition 的任务，是给定一张 query image，在已注册图像数据库中找出来自同一地点的图像。其典型做法，是先为每张图像计算一个全局描述子 `d = f(I) ∈ R^d`，再在这个描述子空间中通过 `k-NN` 搜索最近邻。

`Galvez-Lopez` 与 `Tardos` [362] 引入了 `bag-of-words` 路线，本质上是将 `ORB` 或其他局部描述子量化到视觉聚类或“visual words”中，再聚合成整图表示。这类方法在较小的时间与空间变化范围内表现很好，但由于手工特征对视觉纹理变化的 invariant 性有限，在外观变化较大的场景中会受限。针对这类情况，后续研究提出并训练了多种深度网络，用于 feature extraction 与 aggregation，以获得对视觉 appearance changes 更强的鲁棒性 [35, 508]。

### 7.3.7 初始化

初始化在视觉系统中至关重要，尤其是 monocular 系统，因为尺度、深度和运动都必须从有限视图中联合恢复。若初始化阶段就落在错误的几何模型或错误的尺度上，后续 tracking 和优化往往会持续被拖偏。

稳定初始化通常需要充分视差、足够可靠的特征对应，以及与场景几何相匹配的初始模型。作者因此把 initialization 看作 visual `SLAM` 里最容易被低估、但实际影响极大的模块之一。

### 7.3.8 地图表示

visual `SLAM` 并不只对应一种地图形式。系统可以构建稀疏 landmarks，也可以构建 semi-dense depth map、dense surface、surfel map，乃至更现代的 neural scene representation。

地图表示的选择会直接影响 tracking residual 的形式、loop correction 的方式，以及最终系统更适合做定位、重建还是场景理解。

## 7.4 关于图像对齐与 BA 的进一步讨论

在掌握了 visual `SLAM` 的基本要素之后，作者进一步讨论 image alignment 和 `Bundle Adjustment` 的实现细节，因为这往往是区分“能工作”和“工作得很稳”的关键。

### 7.4.1 基于关键点的图像对齐

虽然 full `BA (7.3)` 是 `SfM` 中的 gold standard，但它的计算代价太高，无法在通常 `10` 到 `50 Hz` 的 frame rate 下运行。因此，大多数基于关键点的视觉 `SLAM` 管线都会采用两项核心思想: 并行的 tracking / mapping，以及局部化的 `BA`。

第一项思想是 `Parallel Tracking and Mapping`，也就是把 `SLAM` 拆成并行运行的两个 threads [582]:

- 一个 tracking thread，针对当前图像 `i ∈ C` 寻找 feature matches，并在不更新地图点估计的情况下，只通过 `pose-only BA` 来计算当前 camera pose:

`T_w^{i*} = argmin_(T_w^i) Σ_(j∈P) ρ( z_ij^i - π(R_w^i x_j^w + t_w^i) )`  。`(7.4)`

- 一个 mapping thread，只对一部分图像 `K ⊂ C` 进行 `BA`。这些图像被称为 keyframes，它们的 poses 才真正进入地图。这样，`BA` 就不必以 frame rate 运行，而只需以典型 `0.5` 到 `5 Hz` 的 keyframe rate 运行。关键帧可以按固定频率插入，但更合理的做法则是只在当前帧带来显著新信息时把它升级为 keyframe。

第二项思想是 `Locality`。对于大场景来说，相机当前观测对地图中远处部分的影响通常很弱，除了回环发生时才需要全局协调。因此很多 visual `SLAM` 系统采用 local `BA`，只在当前关键帧及其 covisibility neighborhood 上做优化，而不是每次都优化整个图。这个局部窗口既可以按时间准则定义，例如最近 `k` 帧或 keyframes，这在 visual odometry 与 visual-inertial `SLAM` 中很常见；也可以按 covisibility 准则定义，例如把与当前 keyframe 共享超过 `θ` 个观测点的 keyframes 纳入窗口 [1037, 786]。后者在 visual `SLAM` 系统中通常更合理，因为它更符合地图实际耦合结构。此时，`local BA` 只更新一组互相共视的 keyframes 以及被它们观测到的 points:

图 `7.5` 展示了 `ORB-SLAM` [786] 中 locality 的表示。共视图（左）连接那些至少共同观测到 `θ` 个点的关键帧（本例中 `θ = 15`），并用于局部 `BA`。基本图（右）是其更稀疏的版本，在本例中连接至少共同观测到 `θ = 100` 个点的关键帧，并在回环校正期间用于位姿图优化。（©2015 IEEE）

`{T_w^{k*}, x_j^{w*} | k ∈ K_1, j ∈ P_1} = argmin_(T_w^k, x_j^w) Σ_(i∈K_1∪K_2, j∈P_1) ρ( z_ij^i - π(R_w^i x_j^w + t_w^i) )`  。`(7.5)`

图 `7.6` 说明了 `ORB-SLAM` [786] 中局部 `BA` 的实现。局部地图由关键帧集合 `K` 构成，其中包含当前关键帧 `k` 及其在共视图中的邻居；`P_1` 是它们观测到的点集（红色）。`K_2` 是地图中其余那些观测到 `P_1` 中某些点的关键帧。

以 `ORB-SLAM` 为例，局部地图通常由“当前 keyframe + 在 covisibility graph 中与其相邻的 keyframes”组成，再加上这些 keyframes 共同观测到的 points。Figure `7.7` 进一步给出了完整视觉 `SLAM` pipeline 的例子，其中一共包含四个线程: 以 frame rate 运行的 tracking、以 keyframe rate 运行的 local mapping、为每个 keyframe 尝试检测回环并加以校正的 loop closing，以及在检测到 loop closure 后可选运行的 full `BA`。

图 `7.7` 展示了 `ORB-SLAM2` 系统 [784] 的结构，包括地图、地点识别数据库，以及由四个线程组成的完整处理流程：tracking、local mapping、loop closing 和 full `BA`。（©2017 IEEE）

### 7.4.2 直接图像对齐

像 `LSD-SLAM` 和 `DSO` 这样的 direct methods，一般也遵循“先做相邻帧 tracking / mapping，再逐步保证全局一致性”的总体流程。但与先提取、匹配、跟踪关键点，再去最小化几何重投影误差的做法不同，它们直接使用传感器的亮度信息，目标是从原始观测数据中联合恢复三维结构与相机运动的最大后验估计。

这对应于最小化如下 photometric loss:

```text
E_photo
= Σ_{i∈F} Σ_{z∈P_i} Σ_{j∈obs(z)}
  ρ(I_i(z) - I_j(ω(z, d_z^i, T_j^i)))                     (7.6)
```

其中，优化变量包括所有 camera parameters `T_i ∈ SO(3)` 以及所有深度值 `d_z^i ∈ R`。这个目标函数要求: 对于关键帧集合 `F` 中的每一帧 `i`，以及帧 `i` 中每个点 `z ∈ P_i`，只要该点在 `obs(z)` 对应的其他帧里可见，那么这些 frame pair 中对应位置的颜色就应当保持一致。这里的 warp `ω` 会取点 `z` 及其深度 `d_z^i`，再利用刚体运动 `T_j^i` 把它从帧 `i` 变换到帧 `j`，最后重新投影到图像 `I_j` 中。

正如 Figure `7.8` 所示，`LSD-SLAM` 通过交替优化 depth maps 与 camera motion 来完成 tracking 和 mapping，同时再运行 `pose-graph optimization (PGO)`，以保证估计出的相机运动在全局上与所有成对图像对齐结果保持一致。

图 `7.8` 给出了 `Large Scale Direct (LSD) SLAM` [307] 的示意图，展示三个交替运行的组件：直接相机跟踪、直接建图，以及用于全局一致性的 `PGO`。相比之下，`Direct Sparse Odometry (DSO)` [308] 在单个 `Gauss-Newton` 优化中同时优化结构与运动，以获得更高精度。像 `DSO` 这样的直接法已被证明比基于关键点的方法具有更高精度，因为它们不进行中间抽象，甚至可以利用非常细微的亮度变化来估计相机运动 [308]。（©2017 Springer）

相比之下，`DSO` 把结构与运动放进同一个 photometric `BA` 里联合优化。它的 robust loss 是在小 patch 上的加权平方差，同时还显式考虑自动曝光时间补偿，以处理曝光时间未知的情况。相应 residual 的依赖关系可以自然地写成 factor graph。为了在 `CPU` 上实现实时性能，`DSO` 只对滑动窗口中的一小部分关键帧做优化，并将更早的帧边缘化出去。作者强调，这正是 `DSO` 在精度上显著受益的原因之一，因为它本质上利用了给定全部亮度观测时，对结构和运动的统计最优估计。

对于实时可运行的 `SLAM` 系统，全局一致性通常分多步实现。第一步，可以像 `DSO` 那样，只在最近 `k` 个关键帧上做联合优化，也就是 sliding-window photometric `BA`。第二步，可以额外运行 `PGO`，根据所有已估计的成对图像对齐结果，重新计算一条全局更一致的相机轨迹。第三步，则可以使用一种称为 `Pose Graph Bundle Adjustment (PGBA)` 的自适应 `PGO` 版本，它在只更新 camera poses 的前提下，也把 `BA` 的完整光度不确定性纳入进来，因此保留了相近的计算效率。

图 `7.9` 说明，在直接视觉 `SLAM` 方法中，可以执行位姿图优化，以计算一条与此前估计得到的所有成对图像对齐结果全局一致的轨迹 [552]。（©2013 IEEE）

### 7.4.3 求解 BA

虽然 `BA` 理论上可以用一般的变量消元技术求解，但它的 Jacobian 和 Hessian 具有极其特殊的稀疏结构，因此存在专门更高效的求解方法。

每个图像观测只依赖一个 camera 和一个 point，因此 observation Jacobian 十分稀疏，对应 Hessian 也呈现典型 block structure: camera-camera 对角块、point-point 对角块，以及 camera-point 耦合块。由于三维点数量通常比相机数量多得多，一个非常自然的思路是先消掉 points，再求 cameras。

图 `7.10` 给出了一个包含 `4` 个相机和 `9` 个点的玩具示例。第一行展示因子图以及点观测的 Jacobian；第二行展示完整 Hessian，以及约化后的相机系统 Hessian。

作者回顾了 `Schur complement` 的经典做法。通过先消元 points，可以得到 reduced camera system；解出 cameras 后，再反求 points。由于 `H_pp` 是 block diagonal，这一过程可以非常高效地逐点进行。local `BA` 中 reduced camera system 往往较稠密，因此可以用 dense solver；full `BA` 中 keyframes 更多，但 covisibility 更稀疏，因此则更适合 sparse solver。

### 7.4.4 再谈 Bundle Adjustment

`Bundle Adjustment (BA)` 已经被研究了一个多世纪，Section `7.4.3` 中介绍的经典计算流程，也已在大量里程碑论文中被证明有效。但作者指出，这条传统 pipeline 仍有两个重要短板。第一，它通常需要一个足够好的初始化，尤其是 landmarks 与 camera poses 的初值。第二，在大规模问题中，其计算量和内存消耗可能会增长到难以承受的程度。近几年，一系列工作正是围绕这两个问题，开始重新审视传统 `BA` 的计算路线。

作者指出，核心计算瓶颈在于 reduced camera system `(7.7)` 的求解。传统做法往往借助 iterative conjugate gradient 去求解；而 `Power Bundle Adjustment` 则提出，用矩阵幂级数来近似 Schur 矩阵

`H_cc^red = H_cc - H_cp H_pp^{-1} H_cp^T`                    `(7.8)`

的逆:

```text
(H_cc^red)⁻¹ ≈ Σ_{i=0}^m (H_cc⁻¹ H_cp H_pp⁻¹ H_cp^T)^i H_cc⁻¹   (7.9)
```

随着截断参数 `m` 的增大，这一幂级数可以严格收敛到真实逆矩阵。它的优势在于，把原本代价高昂的矩阵求逆，替换成了更快、更省内存的矩阵乘法。

针对对初始化的依赖，作者又介绍了另一条路线，也就是回到 `variable projection` 的思想。其基本做法是把 `BA` 拆成两个阶段。第一阶段中，先把复杂的 perspective projection 替换成一个更一般的 projective matrix，于是 landmarks 可以被解析地表示为 camera parameters 的函数，从而打破 landmarks 与 camera poses 之间“鸡和蛋”的相互依赖关系。这会显著扩大 camera pose 优化的收敛域。第二阶段则进行 projective refinement，把第一阶段得到的结果当作原始 perspective reconstruction 的初始化。

单独来看，这条路线在超过 `100` 个 cameras 的大规模问题上仍然过于昂贵；但若把它与上面的 power-series 方法结合，就能得到一种无需依赖初始化、同时又适合大规模 `BA` 的可扩展求解方案。这也正是作者想强调的: 现代 `BA` 并不只是“把最小二乘写出来再交给 `Gauss-Newton`”，它本身仍在随着规模化和初始化鲁棒性问题不断演化。

## 7.5 完整视觉 SLAM 系统示例

作者在这里回顾若干完整 visual `SLAM` 系统，用来说明前面介绍的 tracking、mapping、place recognition 与 optimization 模块，究竟如何在真实系统中协同工作。Figure `7.11` 还展示了一类较新的大规模 `bundle adjustment` 研究，它们倡导使用 `variable projection methods` 与 `matrix power series`，以在运行时间和内存上都较高效的方式，从随机初始化出发求解相机位姿与地标。

图 `7.11` 展示了在一系列论文 [469, 470, 1170, 1171] 中倡导的方法：使用 `variable projection` 方法与矩阵幂级数，以在运行时间和内存上都较高效的方式，在无需初始化的情况下求解大规模 `bundle adjustment` 问题。图中说明，相机位姿和地标可以从随机初始化出发恢复得到。（©2024 Springer）

`LSD-SLAM (Large-Scale Direct SLAM)` 是典型的 direct `SLAM` 系统，强调稠密跟踪与半稠密建图。它依赖 photometric error minimization，而不是关键点匹配，因此在低纹理环境中尤为有效。系统能够实时运行，输出场景的半稠密重建，尤其适合室内或小尺度室外环境中的 monocular camera。

`ORB-SLAM` 则因鲁棒性和灵活性而成为最广泛采用的系统之一。它基于 `ORB (Oriented FAST and Rotated BRIEF)` 描述子的关键点跟踪，配合有效的 loop closure detection 与稀疏地图表示。`ORB-SLAM` 支持 monocular、stereo 与 `RGB-D` 相机，因此适用场景非常广。它尤其擅长需要高精度与强重定位能力的任务。到 `ORB-SLAM3` 时，系统又进一步扩展到 fisheye camera、multimap 与 visual-inertial `SLAM`。与此相关，`OKVIS` 虽最初被设计为 visual-inertial odometry，最新版 `OKVIS2` 已经是 visual-inertial `SLAM` 系统，同时也能在纯视觉的多相机模式下运行；和 `ORB-SLAM` 家族类似，它同样使用关键点与描述子，只是采用的是 `BRISK`。

`Direct Sparse Odometry (DSO)` 则是 direct 路线中的另一代表性系统，用于同时估计 `3D point cloud` 与相机轨迹。与 `LSD-SLAM` 不同，它把相机运动和地标点放在同一个 `Gauss-Newton` 优化中联合估计。为了满足实时性，只对最近的 `k` 个关键帧做更新，因此本质上是一个 sliding-window photometric `BA`。参数 `k` 同时决定了速度与精度之间的权衡。`DSO` 还使用完整的 photometric calibration，包括 camera response function 与 vignette，因此传统的 brightness-constancy 假设被更严格的 irradiance-constancy 假设所取代，也就是说，同一个三维点在不同时刻应发出相同的辐照度。此后还有面向 stereo 系统和 omni-directional camera 的扩展版本，也有加入 loop closure 来降低长序列漂移的版本。

不过，作者也强调了不同路线的现实权衡。就实时系统而言，像 `DSO` 这样的 direct 方法在若干实验中已经显示出比 keypoint-based 方法更高的精度和更强的鲁棒性，但它们往往要求良好的 photometric calibration 与 `global shutter` camera。`rolling shutter` 会带来几何畸变，虽然 direct 方法可以显式建模这些畸变，但由此得到的系统常常难以继续保持实时性。因此，这类畸变在 keypoint-based 方法中往往更容易处理，因为它们本来就更偏向抑制几何失真。

另一方面，direct 方法在低分辨率视频中常常更有优势，因为在强模糊和下采样条件下，可靠 feature point 往往更难识别。反过来，feature point 又对高效 relocalization 和 loop closing 非常重要。也正因如此，真正实用的 visual `SLAM` 系统往往会走向 hybrid 路线，把 direct 与 feature-based 两种思路结合起来。书中举的一个例子是，把 feature-based 的重定位信息紧密集成进 direct visual `SLAM` 中，从而进一步提升系统鲁棒性与精度。

## 7.6 实时稠密重建

传统 `BA` 与 `SLAM` 主要关注恢复 camera motion 和一组稀疏 landmark points；但对从 augmented reality 到 autonomous robots 的许多应用而言，人们更希望得到对已观测世界的稠密重建。为此，过去多年出现了多种来自 monocular camera 的 real-time dense reconstruction 方法 [1045, 799, 1179, 874]。

这类方法往往回到 variational methods，通过最小化一个损失函数来估计连续三维结构 [1045]:

```text
min_{h: Ω → R} (1/2) Σ_{i∈I(x)} ∫_Ω ρ_i(z, h) dz + λ ∫_Ω |∇h|^2 dz    (7.10)
```

其中，稠密地图 `h` 会为图像平面 `Ω ⊂ R^2` 上的每个像素 `z` 赋予一个深度值。对应残差项写为

```text
ρ_i(z, h) = | I_i(π(R_w^i x^w(z, h) + t_w^i)) - I_0(z) |    (7.11)
```

它要求参考图像 `I_0` 中像素 `z` 的亮度，与一组相邻图像 `I_i` 中对应像素的亮度保持一致。这里的对应像素，是把三维点 `x^w(z, h)` 从 world frame 通过旋转 `R_w^i ∈ SO(3)` 和平移 `t_w^i ∈ R^3` 变换到相机 `i`，再通过投影模型 `π(·)` 映射到图像 `I_i` 中得到的。

由参数 `λ` 加权的 total variation regularizer，则用于约束估计得到的深度图在空间上保持平滑，并会对未观测区域产生一种类似 soap-film 的填充效果，Figure `7.12` 就展示了这种结果。这类方法通常需要依赖 `GPU` 并行化，才能在手持 monocular camera 上实现实时稠密三维重建。

图 `7.12` 说明，可利用变分方法，从手持单目相机（下）实时计算三维世界的稠密重建（上），并在 `GPU` 上高效并行化实现 [1045]。相关方法还见 [799, 1179, 874]。（©2010 Springer）

## 7.7 基于深度相机的 SLAM

随着 Microsoft `Kinect` 的出现，depth-sensing cameras 开始成为大众可获得的传感器。典型 `RGB-D` cameras 通常基于 structured light 或 time-of-flight 原理工作，同时输出 depth image 与 color image stream。从传感特性上看，它们介于普通 camera 与 `LiDAR` 之间: 不像 `LiDAR` 那样逐点扫描，而是直接给出一个瞬时的二维深度阵列；但与普通相机相比，又天然提供了每个像素的深度信息。

配合合适算法，这类传感器对 `3D` 感知非常强大，但书中也明确指出了它们的典型局限: 一方面多用于 indoor 场景，因为红外主动感知容易受到 sunlight 干扰；另一方面量程通常有限，常见设备大致只适用于 `5` 米以内的近距离环境。

作者随后回顾了 `KinectFusion` 这一经典系统。它建立在更早期的 range image fusion 思路之上，主张从移动中的 `RGB-D` camera 同时恢复相机运动与三维结构。其核心思想，是把每一张 depth image 编码成一个 projective signed distance function `d_i(x)`，其中每个 voxel `x` 上都保存到最近表面的有符号距离；随后再通过加权平均将多帧深度融合成一个全局距离函数 `D(x)`:

`D(x) = [Σ_i ω_i(x) d_i(x)] / [Σ_i ω_i(x)]`  。`(7.12)`

在假设 depth 方向噪声近似 Gaussian 的情况下，这种加权平均实际上就是距离函数的最大似然估计。为了增强稳健性，工程上通常不会直接平均完整 `SDF`，而是平均 `truncated signed distance functions (TSDFs)`，使每个表面点只在局部区域内影响重建。

权重 `ωi(x)` 则用于表达各表面测量的置信度，它通常依赖具体传感器特性，并会随物体距离增长而衰减。对重建中的空洞问题，可以在后处理阶段填补，也可以直接修改加权策略，使重建结果更接近 watertight surface。

在相机跟踪方面，早期 `RGB-D SLAM` 往往通过 `ICP` 对齐连续 depth point clouds；后续方法则越来越倾向于直接最小化 color consistency 与 depth consistency 残差，而不是只做纯几何点云配准。对连续两帧 `RGB-D` 数据 `(I_1, d_1)` 与 `(I_2, d_2)`，这类直接跟踪常用如下残差:

`r_I(ξ) = I_2(τ_g(z)) - I_1(z),   r_d = d_2(τ_g(z)) - [ g π^(-1)(z, d_1(z)) ]_z`  。`(7.13)`

其中 `g ∈ SE(3)` 是待估计的刚体运动，`τ_g(z) = π g π^(-1)(z, d_1(z))` 是对应像素间的 warping，而 `[·]_z` 表示取点的 `z` 分量。随后可为所有残差 `r = (r_I, r_d)` 拟合一个合适分布，并通过 coarse-to-fine 的 `Gauss-Newton` 优化，在 `ξ ∈ se(3)` 上求得 `g = exp(ξ)` 的 `MAP` 估计。与经典 `ICP` 相比，这种 direct tracking 在已有 benchmark 上能把 root mean square tracking error 降低接近一个数量级，同时运行速度也明显更快。

对更大尺度的建图而言，均匀 voxel 表示又会过于耗内存，因此系统通常会退回到 `voxel hashing` 或 `octrees` 之类的自适应表示；后续工作如 `kintinuous` 还使用 moving fusion window，而 `ElasticFusion` 则通过 deformation graph 把 loop closures 也整合进 dense map correction。

图 `7.13` 展示了利用移动 `RGB-D` 相机 [1029] 对包含多个办公室的大尺度走廊场景进行稠密重建的例子。借助 octrees [1029] 或 voxel hashing [807]，并结合直接相机跟踪 [553]，这类重建已经被证明可在 `GPU` [1029] 甚至平板 `CPU` [1030] 上实时运行。（©IEEE）

不过作者也提醒，depth camera 只是把问题从“从几何中恢复深度”转成“如何处理深度噪声、深度量程和光照适应性”。它并没有自动消除所有困难。

## 7.8 视觉与其他模态的融合

视觉虽然信息量极大，但在高速运动、低纹理、强曝光变化和长期漂移场景中也很容易失败。因此，把视觉与其他 sensing modalities 结合，已经成为现代 high-performance `SLAM` 的重要方向。

### 7.8.1 惯性测量单元（IMU）

`IMU` 能够以很高频率提供角速度与线加速度测量，因此特别适合精确估计局部运动。最简单的多传感器融合方式是 `loosely coupled` 方案，也就是先分别把各个传感器的信息独立融合成位姿估计，再用 `Kalman filter` 对这些估计做二次融合。这类方法在实践中往往也能取得不错效果，例如用于四旋翼自主导航或微型飞行器导航，但从精度上说，它通常仍不如 `tightly coupled` 的感知信息融合。

最早的一批紧耦合方法多采用 `EKF`。它们在 prediction step 中积分 `IMU kinematics`，再把 visual keypoint measurements 作为更新量加入滤波器，代表性工作如经典的 `MSCKF`。另一条路线则是 sliding-window 或 batch optimizer，它们可以直接采用第 11 章将详细讨论的 `factor graph` 表述。Figure `7.14` 就展示了一个用于 stereo-inertial odometry 的因子图，它把 stereo `LSD-SLAM` 与 `IMU` 信息结合在一起；Figure `7.15` 则说明，紧耦合 `IMU` 之后，系统漂移明显减小，重建结果也更加清晰和精确。今天，`factor graph` 形式的紧耦合已经同样被 `ORB-SLAM3`、`OKVIS2` 等最先进的 keypoint-based 方法广泛采用。

图 `7.14` 说明，利用因子图可以优雅地以紧耦合方式集成多传感器。这是一个视觉与 `IMU` 紧耦合融合的因子图示例 [1115]。它显著提高了相机运动与三维结构估计的精度和鲁棒性，见图 `7.15`。（©2016 IEEE）

图 `7.15` 说明，将 `IMU` 测量与直接图像对齐结果紧耦合融合后（左），相较于仅依赖图像对齐的纯视觉里程计系统（右），能够得到更准确的位置跟踪。该融合通过图 `7.14` 所示的因子图实现。重建点云来自纯里程计结果，未施加回环约束 [1115]。（©2016 IEEE）

在实践中，很多最精确的 visual-inertial odometry 系统，往往都是先以惯性积分作为基础，再主要依靠视觉去纠正由于噪声和 bias 双重积分所带来的快速漂移。此外，`IMU` 还能帮助系统观测运动的 metric scale，以及传感器相对于重力方向的姿态。这样一来，系统的 gauge freedom 会从纯视觉情况下的 `7` 个未知量，降低到只剩 `4` 个未知量: 全局 `x, y, z` 位置以及 `yaw`。相比之下，纯视觉系统还存在 `roll`、`pitch` 和 scale 的额外不确定性。从这个意义上说，`IMU` 与视觉在统计上是高度互补的，因此特别适合结合成 visual-inertial `SLAM` 或 odometry 系统。

这些年里，visual-inertial odometry / `SLAM` 方法已经大量出现，而且很多都是在现有 visual `SLAM` 系统基础上的扩展。比较流行的代表包括 `ORB-SLAM` 的视觉惯性版本、`Direct Sparse Visual-Inertial Odometry`、`VINS-Mono`、`BASALT` 和 `DM-VIO`。其中 `DM-VIO` 是一个 mono-inertial 表述，它利用 `delayed marginalization` 的思想，更好地刻画了相关传感器运动的可观测性。

### 7.8.2 用于全局定位的 GPS 与 WiFi

尽管 `IMU` 能改善局部运动估计，但在大尺度环境中，`GPS` 与 `WiFi` 这类外部信号对全局定位依然至关重要。在室外环境中，`GPS` 提供绝对位置观测，可把 `SLAM` 地图锚定到全局坐标系中；这一点对自动驾驶车辆等应用尤其关键，见 Figure `7.16`。在室内环境中，`GPS` 通常不可用，而 `WiFi` 信号则可提供 coarse localization，并与 visual map-based localization 形成互补。

图 `7.16` 说明，通过将包括双目相机、`IMU` 和 `RTK-GPS` 在内的多种传感器信息（左）以因子图方式紧耦合融合，可以获得高度精确且鲁棒的轨迹与点云；无论室内还是室外都适用。右图展示了汽车穿行于多层停车场时的重建结果 [1180]。（©2020 Springer）

## 7.9 延伸阅读与最新趋势

作者首先强调，visual `SLAM` 是一个极其活跃且持续快速演化的研究领域。本章虽然尽量覆盖了 classical visual `SLAM` 的主要方法，但终究只涉及了其中一部分工作，因此作者鼓励读者进一步查阅正文与参考文献中引用的大量原始论文，以获得更完整的认识。

在具体趋势上，作者首先讨论 `keypoint detection and matching` 的演进。与传统 handcrafted keypoints 相比，learning-based keypoints 在特定场景中已经表现出前所未有的 invariance 与 adaptability，这一变化正在重塑整个领域。它不仅改进了 keypoint detection 与 description，也显著推动了 keypoint matching 的发展。与 classical methods 依赖手工设计启发式进行 descriptor matching 不同，新的学习式方法通过引入 global spatial awareness，以更鲁棒的方式建立 correspondences。作者特别点名 `SuperGlue` [970] 与 `MASt3R` [641]，指出它们利用 `Graph Neural Networks (GNNs)` 与 `Transformers` 来在更广泛的图像上下文中细化 matches，从而缓解传统局部描述子在极端视角变化、强光照变化和遮挡场景中的局限。

其中，`SuperGlue` 通过 self-attention 和 cross-attention 机制增强 feature matching，可以更好地处理 repetitive textures 与 occlusions 带来的歧义。与只看局部 descriptor 的传统匹配器不同，它显式纳入上下文信息，因此在 indoor 和 outdoor 环境中都能明显提升匹配精度。其内部的 optimal matching layer 还允许在必要时保留 unmatched keypoints，从而更适合真实场景。

`MASt3R` 则进一步把这一思路扩展到 `3D-aware feature matching`。它从两张图像中重建一个三维场景表示，以改善在 textureless regions 和极端 viewpoint changes 下的表现。基于这一能力，`MASt3R-SLAM` [788] 把相关进展集成进完整 `SLAM` pipeline，改进了 camera pose estimation、global map consistency 与 loop closure strategies。作者把这类方法概括为 visual `SLAM` 中的一次 paradigm shift: 从 handcrafted `2D` feature matching，走向 learning-based、context-aware、甚至 `3D`-informed 的匹配与理解框架。

作者同时指出，尽管发展很快，classical methods 依然高度相关，因为它们通常更高效、可解释性更强。然而它们在 textureless surfaces、极端视角变化和显著 illumination variations 场景中仍会吃亏。深度学习缓解了这些问题，但新的挑战也随之而来，主要包括:

- 如何在有限算力平台上同时兼顾 computational efficiency 与 real-time performance。
- 如何获得足够多样且无偏的数据集来训练这些模型。
- 如何让学习式方法在 large-scale real-world applications 中稳定泛化。

作者预计，后续研究会继续沿着“同时最大化 accuracy 和 efficiency”的方向推进，使学习式 keypoint detection 与 matching 更适合大规模真实部署。

除 keypoints 之外，作者还特别提醒，近年的 visual `SLAM` 还有很多其他令人兴奋的发展前沿。其中之一，是把 visual `SLAM` 推广到存在 moving objects 乃至 deformable objects 的 dynamic environments。另一个显著趋势，则是 learning-based visual `SLAM` 正在变得越来越普遍。越来越多 classical visual `SLAM` pipeline 中的组件，例如 feature extraction、correspondence estimation、image alignment、camera tracking、`BA` 以及 dense reconstruction，都在被 learning-based formulations 增强甚至替代。作者也说明，这些内容将在本 handbook 的 Part III 中展开更详细讨论。

## 第 8 章 LiDAR SLAM

### 本章概览

本章转向 `LiDAR SLAM`。与视觉不同，`LiDAR` 可以直接提供场景的 range information，因此几何约束通常更直接、更稳定。但这并不意味着问题更简单。`LiDAR SLAM` 仍需要处理 scan registration、motion distortion、place recognition、loop closure、backend `PGO`、map update，以及更大规模的 multi-session / multi-robot 扩展。

作者本章的组织方式与 visual `SLAM` 相似: 先介绍 `LiDAR` 感知基础与分类，再讨论 odometry、place recognition、完整系统结构、后端 pose-graph optimization、地图更新和多机器人扩展，最后总结鲁棒性、多传感器融合与长期地图管理等当前趋势。

与 camera 一样，`LiDAR` 也是机器人中最常见的 sensing modalities 之一。它通过主动发射激光脉冲，并测量脉冲从物体表面反射回探测器所需的时间，来直接获取这些表面的距离。因此，`LiDAR` 既能感知周围环境的结构，也能用于估计传感器自身的运动或 ego-location。

原书还在章首补充了一段 `LiDAR` 技术发展史。`LiDAR` 的发展始于 `1960s` 和 `1970s`，当时主要是固定式系统，用于大气研究、地形测绘和军事用途。这些早期系统体积大、价格高，不适合移动平台。到了 `1980s`，随着激光技术和计算能力进步，出现了更紧凑、更经济的 `LiDAR` 设备，但它们仍以固定应用为主，例如 terrain mapping 与 environmental monitoring。进入 `1990s` 后，`LiDAR` 与 `Global Positioning System (GPS)` 及 `IMU` 的结合，开始催生最早的移动式 `LiDAR` 建图系统，这些系统通常装在车辆或飞机上，用于构建大范围高精度 `3D` 地图。到了 `1990s` 末，研究者开始探索把 `LiDAR` 用于机器人中的 real-time `SLAM`。`2D LiDAR` 在 `2000s` 初期极大推动了 probabilistic mapping 与 `SLAM` 的进展，而 `3D LiDAR` 则随着 `2004` 年 `DARPA Grand Challenge` 和 `2007` 年 `DARPA Urban Challenge` 的出现，在机器人和自动驾驶领域变得更加重要。

## 8.1 LiDAR 感知基础与分类

通过在 `LiDAR` 传感器内部让激光发射器与探测器绕一个或两个轴旋转，就能够逐步构建出围绕传感器的环境点云。最基础的测距原理是 `time-of-flight (TOF)`，即通过测量发射光脉冲返回到探测器所需的时间来推断距离。`TOF LiDAR` 能获得高分辨率测量，但它对外界光照较为敏感，这会降低 `signal-to-noise ratio (SNR)`，进而影响测量精度与频率 [597]。除了 `TOF` 之外，最早为 radar 传感器发展的 `Amplitude Modulated Continuous Wave (AMCW)` 和 `Frequency Modulated Continuous Wave (FMCW)` 技术，如今也已被引入 `LiDAR`。

作者随后按 sensing mechanism 对 `LiDAR` 分类 [934]，包括 `mechanical LiDAR`、`scanning solid-state LiDAR`、`flash LiDAR`，以及使用宏观扫描 `Risley prisms` 的传感器。本章重点讨论其中两类最常用的设备: `2D/3D mechanical LiDAR` 与 `macroscopic scanning LiDAR`。

图 `8.1` 展示了常见 `LiDAR` 传感器类型及其光束图案。`(a)` 标准 `2D LiDAR` 传感器，带有旋转的发射镜（黄色）；编码盘（蓝色）用于测量镜面的旋转角度。`(b)` 一个机械式多线 `LiDAR` 示例：多个激光束由不同发射器（黄色）发出，再由探测器（蓝色）接收，整个传感器头旋转以生成 `360°` 的水平视场。`(c)` 一种采用宏观转向 `Risley prisms` 的 `LiDAR`，其扫描图案为螺旋形，只在镜头窗口（青色）指向的方向上测量；由于螺旋图案，随着时间推移它能够产生更稠密的点云。

`Mechanical LiDAR` 是目前最常见的一类。它利用旋转机构来引导激光束方向，但也因此会受到机械磨损和数据采集速率较低的限制。最简单的 `2D mechanical LiDAR` 使用一个旋转镜面来偏转单束激光，并同时测量距离，如图 8.1(a) 所示。通过编码盘 `encoder disk` 可以测得激光束的角度，再与距离观测配对，形成一个 `2D` profile measurement。由于其工作原理所限，这类设备一次只能扫描一个独立的二维平面。

对“单一传感器实现完整 `3D` 扫描”的需求，很大程度上来自 `2000` 年代的 `DARPA Grand Challenges` 自动驾驶竞赛，并推动了先驱性的 `Velodyne HDL-64E` 传感器出现。它曾被当时多数参赛队伍使用 [1114, 771, 535]。在 `3D mechanical LiDAR` 中，多束激光发射器安装在同一旋转机构上，各自对应不同的俯仰角；整个机构再沿方位角方向旋转 `360` 度，如图 8.1(b) 所示。由此得到的点云能够对目标与周围环境形成高细节的 `3D` 表示，如图 8.2 所示，这一点已在多项研究中得到讨论 [1177, 556, 934]。随后，该技术继续演进，`LiDAR` 的体积与价格都显著下降。

图 `8.2` 展示了单次多线 `LiDAR` 扫描及其对应图像的示例。该扫描来自一台 `64` 线 `Hesai QT64` 扫描仪，垂直视场为 `104°`。图中可以看到单独的扫描线，并且点云在设备附近（即 `3D` 视图中的彩色坐标轴处）非常稠密，而在更远处则明显稀疏。

近年来，基于更多物理机制的原型传感器也不断出现，包括 `solid-state LiDAR`、`Risley prisms` 与 `polygonal mirrors`。与传统扫描式激光不同，这些传感器中很多使用 `Micro-electromechanical Systems (MEMS)` 镜面技术 [467] 或 `optical phased arrays (OPA)` [446] 来避免或至少减少机械旋转。这一点很重要，因为主动驱动的机械部件更少，往往意味着更长的寿命和更高的环境建图可靠性。

其中一个值得注意的进展是 `Risley prisms` [667]。它们能够在极小物理运动量下实现快速、可控的 beam steering，因此传感器可以做得更紧凑，不过当前通常仍伴随更受限的 `field of view (FOV)`。

上述所有传感器都会输出一组单独的距离测量，并为每个测量附带一个 intensity 值，也常称为 `remission`，表示有多少 `LiDAR` 光束被反射回来。结合各束光的方向角信息，这些距离观测就可以转换为 `2D` 或 `3D` 点云。

不过，较新的 `LiDAR` 技术已经不再局限于默认的 range measurement。比如 `FMCW LiDAR` 会持续发射频率变化的光，并可通过检测频移来测量被照射物体的相对速度，这与 `FMCW radar` 的机制类似。这种能力在动态环境或困难场景中很有价值 [1200]，但系统通常也会更复杂、更昂贵。另一种创新机制是 `flash LiDAR`，它还可以提供类似 camera photometric measurements 的 ambient channels。不同技术在功耗、重量和成本上的特性差异，使得 `LiDAR odometry` 与 `SLAM` 在不同应用中拥有多样化的硬件选择。

## 8.2 LiDAR 里程计

`LiDAR odometry` 是 `LiDAR SLAM` 的第一个基础构件。它的目标，是在给定当前 `LiDAR` scan 与过去观测（既可以是一帧 scan，也可以是聚合成 local map 的多帧 scans）的情况下，实时估计机器人或车辆的增量 ego-motion。这里的 `scan` 指的是传感器完成的一次 sweep scan 或一次完整采样周期。更具体地说，`scan` 往往对应传感器的一整圈旋转或一次完整扫掠，因此它构成某一特定时刻对周围环境的一次上下文快照，也通常会带有时间戳，以便按顺序作为序列观测处理。

`LiDAR odometry` 的核心技术是 `scan registration`，也常称为 `scan matching`。它通过精细对齐一对 scans，估计它们之间的精确相对变换。由于 scan 本质上就是点集，也就是 point cloud，相关文献发展了多种点云配准算法，包括 `Horn's method` 与 `ICP`。`Horn's method` 虽然能给出闭式解，但要求事先知道对应关系，因此在真实场景中的适用性有限；相比之下，`ICP` 不需要预先已知对应关系，因此成为更基础也更流行的点云相对位姿估计方法，下一节会继续展开。

原书还回顾了 `LiDAR SLAM` 的早期发展。`LiDAR SLAM` 的源头可追溯到 `Lu and Milios` 的奠基性工作，他们率先提出了全局一致的 `2D` range scan registration，并引入了由一组 poses 组成的网络结构，这与现代 pose graph 的思想已经非常接近。这项工作也为 `LiDAR odometry` 奠定了 `2D` scan registration 的基础。随后，一些工作进一步发展了 scan-to-scan matching 的 probabilistic framing；在 `2D` 场景里，除 points 和 lines 外，也出现了使用 correlation techniques 的实时配准方法。更早期的 `3D LiDAR` 扩展则常通过主动摆动或旋转传感器，或者被动利用人或车辆运动来积累更稠密的 `3D` 点云。随着数据规模显著增大，实时 `3D` scan matching 的计算挑战也随之上升，直到 `LOAM` 展示出实时 scan matching 能力，才真正为后续大批 `LiDAR odometry` 与 `SLAM` 方法奠定了基础。

### 8.2.1 扫描配准基础

scan registration 是 `LiDAR` 里程计与建图系统中的基础组件。它的目标，是求出一个变换，也就是旋转 `R ∈ SO(3)` 与平移 `t ∈ R^3`，使一帧扫描（例如传感器刚采到的 recent scan）尽可能与另一帧扫描或局部地图对齐。完成这一过程的同时，系统也就得到了这两次扫描采集位置之间的相对位姿。围绕 scan registration，研究界已经发展出大量兼顾精度、鲁棒性与计算效率的方法。

最经典的核心方法是 `Iterative Closest Point (ICP)` 及其变体。按照第 `5` 章的定义，point cloud 可写为 `P = {p_i ∈ R^3 | i = 1, 2, ..., N}`，其中每个 `p_i = (x_i, y_i, z_i)` 表示一个点的三维坐标。对配准问题，设 `P` 是 source point cloud，`Q` 是 target point cloud，则 `ICP` 在第 `k` 次优化迭代中寻找变换 `(R^k, t^k)`，使总配准误差最小:

`R^k, t^k = argmin_(R,t) Σ_(p,q)∈C d(p_i, Rq_i + t)`  。`(8.1)`

其中，source 点云 `P` 与 target 点云 `Q` 之间的对应集合 `C` 写为

`C = {(p, q) | p ∈ P, q ∈ Q}`  。`(8.2)`

在 `ICP` 中，source 与 target 之间的变换并不是一次性求出，而是通过迭代不断重算。也就是说，系统会根据上一次迭代 `k - 1` 得到的旋转 `R^(k-1)` 与平移 `t^(k-1)`，重新计算新的对应集合 `C`，再据此更新当前变换。

为了最小化式 `(8.1)` 中的总配准误差，必须明确两个核心问题:

1. 用什么几何关系来定义距离度量 `d(·)`？若不先定义“两个点云对得有多紧”，就无法谈最小化配准误差。
2. 用于最小化的对应集合 `C` 如何确定？也就是，对每个 source 点 `p ∈ P`，如何找到与之对应的 target 点 `q ∈ Q`。

下面两个小节就分别讨论这两个核心设计选择。

#### 8.2.1.1 配准残差中的距离度量

如前所述，`ICP` 在第 `k` 次迭代中寻找一组变换 `(R^k, t^k)`，使 source point cloud `P` 与 target point cloud `Q` 之间的总配准误差最小:

`R^k, t^k = argmin Σ_(p,q)∈C d(p, Rq + t)` 。

这里的关键首先在于，如何选择距离函数 `d(·)`。书中指出，最常见的几何元素是 points、lines 与 planes，如图 8.3 所示。

图 `8.3` 展示了 `ICP` 中常用的距离度量。`(a)` 点到点距离，就是两个点之间最直接的欧氏距离。`(b)` 和 `(c)` 点到更高层特征（如直线或平面）的距离，则计算为该点到由目标点重建出的直线或平面的最短距离。

`Point-to-point ICP` 是最基础的形式，它直接最小化对应点之间的欧氏距离。早期 `ICP` 经典工作，如 `Zhang` [1280] 以及 `Besl and McKay` [79]，都把 curves 或 surfaces 的匹配转化为点集匹配问题。这种 point-to-point cost 形式最简单直接，但对噪声、离群点以及点云稀疏性会比较敏感。

除点到点之外，也经常使用 `point-to-line distance`。它测量一帧点云中的点到另一帧中由若干点构成的线的最短距离，在具有明显线性结构的环境中往往能得到更好结果。

进一步利用更高层几何特征时，还可以使用 `point-to-plane distance`，也就是计算一个点到另一点云局部表面的最短距离。在包含较多平面结构的环境里，这种方法通常对噪声更鲁棒，也更容易获得高精度配准结果。

在这些基本几何之外，许多 `ICP` 变体还引入了更复杂的距离度量。书中提到的例子包括使用 `multi-distance metrics` [838]、`continuous-time formulation` [256] 以及 `adaptive thresholds` [1132] 的方法。另一些工作 [984, 590] 则不再直接比较欧氏距离，而是比较局部邻域概率分布之间的差异。

一个典型的 distribution-based matching 方法是 `normal distributions transform (NDT)` [82]。`NDT` 会先把输入点云划分为一组 voxels，并在每个 voxel 内拟合一个正态分布。这样它就不必显式计算最近邻对应，而是借助 voxelization 进行 distribution-to-distribution matching。作者指出，这类方法往往能得到更平滑、也更鲁棒的 registration cost surface，尤其适合复杂环境。

#### 8.2.1.2 确定对应关系

`ICP` 的第二个核心设计选择，是 source `P` 与 target `Q` 之间的数据关联，也就是 correspondence search。

最基本的做法，是在当前迭代更新得到的变换 `(R^(k-1), t^(k-1))` 下，对 source 中每个点 `p ∈ P` 在 target `Q` 中寻找最近邻:

`C = {(p, q) | p ∈ P, q = argmin_(q'∈Q) ‖p - (R^(k-1) q' + t^(k-1))‖^2}` 。

但如果 target point cloud 包含上千乃至更多点，这个最近邻搜索通常会很昂贵。

为了让 `LiDAR odometry` 满足实时性要求，常见的加速策略大致分成两类。第一类是减少可作为 correspondence 候选的点集。很多流行的 `LiDAR odometry` 系统 [1268, 838, 998] 都会先构造一个经过筛选的 target set，例如只保留满足某些几何条件的点。典型做法包括只保留边缘点和表面点 [1268, 838]、去除描述性较弱的地面点 [997]，以及对 target scan 做下采样 [1132, 256]。这些方法能显著加速匹配，但代价是有可能把真实对应点也一起删掉。

第二类策略则是改变搜索结构本身，用近似但更快的查找来替代精确最近邻。常见方法包括在 `range image` 中做 `projective neighbor search` [66]，或利用 voxel grids 做 approximate neighbor search [1268, 256, 1132]。此外，虽然这里重点讲的是欧氏空间中的纯几何 correspondence search，但实际上也可以使用其他距离度量，或者把点投影到 feature space [838] 后再进行对应关系识别。

作者最后指出，完整的 `LiDAR odometry` 不会只停留在“配一个 `ICP` 内核”上，而是会把 correspondence search 与 motion compensation、pose estimation 以及 scan-to-scan / scan-to-map 的系统结构整合起来，形成图 8.4 所示的完整 pipeline。

图 `8.4` 展示了 `LiDAR` 里程计流水线的组成：给定当前扫描，`(1)` 运动补偿用于校正在扫描过程中传感器运动造成的影响，从而得到去畸变后的扫描；随后，`(2)` 确定该去畸变扫描与历史扫描之间的对应关系，历史扫描可以是单帧扫描，也可以是局部地图中的聚合扫描；最后，`(3)` 通过扫描配准求得相对位姿估计，从而估计当前扫描的相对位姿。这些步骤是迭代进行的，对应集合会依据中间相对位姿估计不断细化；收敛后，最终位姿估计即为 `LiDAR` 里程计系统的输出。

### 8.2.2 LiDAR 里程计的常见组成

成熟的 `LiDAR` odometry 不只是一个简单的 `ICP` loop，而是一组相互配合的模块，包括 motion distortion compensation、feature extraction 或 direct point-wise alignment、local mapping 以及 pose refinement。

#### 8.2.2.1 点云运动畸变补偿

许多 `LiDAR` 传感器通过在一段时间内完成扫描来形成一帧点云。这意味着一帧内不同点实际上对应不同时间。若平台在运动，整帧点云就会产生 motion distortion。在现代 `LiDAR SLAM` 中，`LiDAR` 往往安装在车辆、机器人或可穿戴设备上；若假设扫描频率是 `10 Hz`，那么单次扫描周期就是 `0.1 s`。

因此，很多系统需要在 registration 之前先做 deskew / undistortion。若不做补偿，一帧点云就不是单一时刻的静态快照，而是由不同采样时刻的点拼接而成，最终会显著损害配准精度与鲁棒性。这类畸变与相机成像中的 `rolling shutter effect` 具有相似性。

图 `8.5` 展示了叠加在相机图像上的 `LiDAR` 点云。`LiDAR` 扫掠开始时（`b`）的轻微失配，到 `LiDAR` 扫掠结束时（`a`）会显著恶化；下方显示的是经运动补偿后的结果：无论在开始时（`d`）还是结束时（`c`），叠加都更加一致。引自 [1071]。

书中将常见补偿思路分为几类。`Constant-velocity model` 假设机器人沿用上一时刻估计得到的平移和旋转速度；它不依赖额外传感器，因此适合较简单的 `LiDAR odometry` 系统，但在存在高频机动时精度有限。`Continuous-time trajectory optimization` 则利用 splines 或 `Gaussian Processes (GPs)` 构造连续时间轨迹，从而在任意时间戳查询 pose 并对每个点逐一去畸变；这种方法建模能力强，但传统实现往往计算代价较高，很多情况下更适合离线处理。

另一类主流方法是 `IMU-based motion compensation`。由于 `IMU` 能直接测量高频角速度和加速度，因此可以在扫描时间范围内对 `LiDAR` 位姿进行预积分，再利用该预测轨迹校正点云畸变。受益于 `IMU` 的高采样率，例如 `200 Hz`，这一做法对抖动较强的平台尤其有效，因此已成为很多机器人平台上的事实标准。不过，`IMU` 噪声、bias、估计误差以及时钟不同步，也可能让这类方法在某些情形下反而不如更简单的补偿策略。

作者还提到一种更彻底的路线: `point-wise registration`。与传统“先积满一帧再处理”的 scan-based 框架不同，它在每个 `LiDAR` 点到达时就更新状态，因此从设计上避免了帧内运动畸变问题。

#### 8.2.2.2 基于特征的 LiDAR 里程计

`LOAM` 代表的经典路线，是先从点云中提取对几何约束最有价值的 corners 和 surfaces，再只对这些 feature points 做配准。这种方法的优点是大幅减少待匹配点数，同时保留最有判别力的结构信息。

与 visual `SLAM` 类似，这条路线的核心在于用少量高信息密度特征表示整帧 scan，并在特征层面完成 correspondence matching 与 residual computation，从而显著降低整体计算开销。

在 low-level features 中，lines 和 planes 是最常见的选择。`LOAM` 的突破之一，就是用简单而高效的低层检测器去识别有助于 scan registration 的中层几何特征。具体来说，它通过分析点与邻域的差异来估计 curvature: 高曲率点被标记为 edge features，低曲率点被视为 planar features。算法并不会使用全部点，而只选取最显著的一部分 edge 和 plane points，以在维持精度的同时降低计算复杂度。

另一类方法会使用 `Principal Component Analysis (PCA)` 来分析点及其邻域协方差矩阵的特征值和特征向量，从而判断局部主方向。若某个点局部只有一个显著主特征值，通常表示该区域更接近边缘；若两个特征值相近，则往往来自平面区域。借助这种局部几何分析，系统可以更稳定地区分 edge 与 planar features。

除低层几何特征外，high-level features 也越来越重要。作者列举了三类代表:

- `Semantic features`：利用 machine learning 或 deep learning 将点云划分为车辆、行人、建筑、植被等对象类别，尤其有助于区分动态目标与静态地标，从而提升 odometry 的可靠性。
- `Surfel features`：将点云局部区域拟合为小的盘状或椭球状表面元（surfels），其主半轴可由邻域协方差矩阵特征值决定，再用于构造 point-to-plane 一类配准残差。
- `Intensity features`：利用 `LiDAR` 回波强度作为附加信息，帮助描述材料属性与表面特征，在低结构环境等困难场景中可提升 feature matching 的稳健性。

这条路线在工程上取得了巨大成功，也催生了大量后续工作。不过，其性能高度依赖特征提取规则和传感器特性，因此在跨场景、跨设备泛化上仍会遇到参数敏感问题。

#### 8.2.2.3 直接点级 LiDAR 里程计

基于特征的方法虽然高效，但它会丢弃那些既不明显属于边缘、也不明显属于平面的孤立点贡献。这在自然环境中尤其成问题，例如灌木、树枝等非规则结构往往很难被少数手工特征充分表达。此外，这类方法通常还需要针对不同传感器调节 feature detector，而待处理点数又会随 `LiDAR` beams 数量上升而快速增加；对现代 `64` 线或 `128` 线设备来说，后端计算代价可能变得很高。

另一条路线是减少手工特征工程，直接使用更多原始点进行 point-wise registration。现代 `ICP`、`GICP`、point-to-plane、scan-to-map alignment 等方法都属于这一范畴。它们与 visual `SLAM` 中的 direct methods 类似，试图直接对点进行对齐，而不显式依赖中层几何特征。

早期这类方法之所以不占主流，一个重要原因是 point-wise correspondence matching 的代价太高。后来的工作通过加速最近邻搜索逐步改变了这一局面。例如，Zhou 等人的工作借助 `GPU` 加速的 `KD-tree`，使 direct methods 在工程上变得可行。再之后，`Fast-LIO2` 使用所谓的 `incremental KD-tree (iKD-tree)`，通过按需重平衡来支持高效的插入、删除与查询，从而避免每加入一帧 scan 就重建整棵树。

因此，这类方法的优势是更直接、少依赖启发式 feature selection，但往往对最近邻搜索、局部地图结构、初始化质量和退化处理提出更高要求。近年来不少研究表明，在设计得当的系统中，这类方法完全可以达到非常高的精度和鲁棒性。

#### 8.2.2.4 局部建图与位姿估计

一旦获得了可靠对应关系，下一步就是让连续 scans 的配准保持一致。正如前文所述，`LiDAR` odometry 中的增量位姿估计，在多数情况下都是通过某种 `ICP` 变体完成 scan registration。

早期方法更多采用 `scan-to-scan odometry`，力求在传感器全帧率下完成相邻 scans 的独立配准；但由于单帧 `LiDAR` scan 往往较稀疏，直接把上一帧当作参考时，经常会出现大量不理想对应，从而让配准误差不断累积，并最终损害整体地图一致性。

现代 `LiDAR` odometry 很少只做 frame-to-frame alignment，而更常做 `scan-to-map odometry`，也就是 `frame-to-local-map alignment`。这样做的原因是，当前 scan 与一个更致密、更稳定的局部地图配准时可以利用更多上下文信息，因此通常比单纯与上一帧配准更稳定，也更不容易漂移。这一范式已广泛用于 `2D LiDAR SLAM`、`3D LiDAR odometry` 和完整 `3D LiDAR SLAM` 系统，并已被证明能够降低整体 drift rate。

在这类系统中，来自 `scan-to-scan odometry` 或 `IMU preintegration` 的运动预测，通常会先用于对 incoming scan 做粗对齐；随后再对其与持久化的 local map 做精配准。由于 local map 比单帧 scan 稠密得多，因此通常能提供更合适的 inlier correspondences。配准完成后，当前 scan 会被纳入 local map。

较早的系统往往因算力受限，不得不交替运行高频 `scan-to-scan` 和较低频的 `scan-to-local-map`；而更新一代方法则更多采用单阶段的 `scan-to-map alignment`，并结合 voxelized local map 支持高效的 direct point-wise correspondences。

作者还提到，另一条完全不同的方向是使用 deep learning 直接学习 ego-motion。早期工作多依赖带 ground-truth labels 的 supervised learning [654]，随后也逐渐扩展到 unsupervised learning 路线 [210]。虽然这类 deep `LiDAR` 方法已经表现出不少有前景的结果，但其 generalization capability 仍持续受到关注。

### 8.2.3 LiDAR 里程计总结

总的来说，常见的 `3D LiDAR` 里程计算法，能够通过把 incoming scans 迭代配准到不断维护的 local map 上，生成高精度且较鲁棒的运动估计；在很多系统中，这一过程还会借助 `IMU` 测量或 motion model 来校正扫描过程中的运动畸变。表现良好的系统，其 drift rate 可以达到每行驶 `1000 m` 约 `1 m` 的量级。

不过，这类性能高度依赖机器人周围环境、场景中的动态程度，以及传感器自身的运动特性。因此，如何进一步处理这部分残余小漂移，就是 `LiDAR SLAM` 的关键问题之一，而 place recognition 正是解决这一问题的核心组成部分。

## 8.3 LiDAR 地点识别

要从单纯 odometry 走向完整 `LiDAR SLAM`，必须让系统具备 place recognition 能力，即能够识别“当前所在位置是否和历史中某个地点相同”。这是 loop closure detection 和重定位的基础。

与 visual data 不同，`LiDAR` 能稳定提供周围环境的一致性度量 `3D` 信息，因此对光照变化通常不如相机敏感。但作者强调，`LiDAR place recognition` 依然面临一组非常独特的困难。

第一是 `sparse data`。与密集且规则排列的图像像素不同，传统 `LiDAR` 获取的点在空间中的分布稀疏而不规则，局部密度还会随传感器类型和量程变化。正因如此，`LiDAR place recognition` 往往不像视觉那样天然适合为每个点都构造一个局部 keypoint descriptor。为弥补结构缺失，很多方法转而使用具有语义意义的点云 segment，或者为整帧 scan 计算一个 global descriptor。随着深度学习发展，学习式 local keypoint descriptors 也重新活跃起来。

第二是 `structural aliasing`。在长走廊、高速公路或规则办公环境中，不同地点可能在 `LiDAR` 扫描中呈现非常相似的结构布局。视觉系统有时还能借助纹理、海报或装饰来区分这些地方，但单靠一帧 `LiDAR` scan，区分“同样形状但不同位置”的场景往往非常困难。像 `Scan Context` 这类 global descriptors 在这种情形下可能失效，而使用 object-level clusters 或更高层语义特征的方法，则往往更有机会缓解这种结构混淆。

图 `8.6` 说明，尽管 `RGB` 图像中的 `Place A` 与 `Place B` 在视觉上差异很大（俯视图和机器人前视图），它们对应的 `LiDAR` 扫描却呈现出结构上相似的模式。这会导致结构性的感知混淆，即由于共享的道路拓扑和周围结构，不同地点会在基于 `LiDAR` 的地点识别算法看来十分相似。该示例采自 `STheReO` 数据集 [1261] 的 `SNU Afternoon` 序列。

总之，研究者在设计 `LiDAR place recognition` 方法时，必须始终把稀疏性与结构混淆这两类挑战放在心里。

### 8.3.1 问题定义

从形式上看，place recognition 的任务是: 给定当前观测，也就是以 point cloud 表示的 query，在历史数据库中找出一个或多个最可能对应于同一地点的候选。数据库本身由此前访问过地点的 descriptors 构成，通常按时空顺序建立。通常这一模块输出的不是精确位姿，而是 loop closure 候选或 relocalization 候选。

这里一个核心问题，是检索方法对传感器类型、采集时间和机器人位姿变化的鲁棒性。例如，query 可能来自与建库时不同类型的 `LiDAR`；重访发生时，环境中还可能出现结构变化、动态物体变化以及显著的位姿偏移。尤其是机器人平移和旋转变化，会直接改变同一地点在传感器坐标系中的外观。

作者据此区分了两个概念。若方法虽然不能估计重访时的位姿差，但仍能正确识别候选地点，就称其具有 `invariance`；若方法还能显式估计这种重访位姿变化，则称其具备 `awareness`。后者之所以重要，一方面是因为它能为后续精配准提供初值，帮助建立 `SE(2)` 或 `SE(3)` 的精确 loop closure 约束；另一方面，朝着 `awareness` 方向努力，往往也会自然增强方法本身的 `invariance`。

### 8.3.2 LiDAR 地点识别方法

除了追求 `invariance` 与 `awareness` 之外，现有方法还会从不同粒度上对点云进行表示，以应对原始 `LiDAR` 数据的非结构化特性，并保证大规模机器人系统中实时 place retrieval 的需求。总体上，descriptor-based 的 `LiDAR place recognition` 方法大致可分为基于 local descriptors、基于 global descriptors，以及更高层或组合型描述子的路线；此外，也有一些工作不只使用 descriptor distance，而是直接学习一个 place similarity function。

#### 8.3.2.1 局部描述子

在 `LiDAR place recognition` 的早期阶段，local keypoint descriptors 是非常自然的起点，这与 visual place recognition 从 `SIFT`、`ORB`、`DBoW2` 等方法发展出来的历史相呼应。无论对 `2D` 还是 `3D LiDAR`，研究者都曾尝试为每个局部关键点构造描述子，再通过局部匹配建立地点相似性。

但作者指出，很多 originally 为 dense `RGB-D` 点云配准或物体识别设计的 `3D` local descriptors，很难直接迁移到稀疏、无规则的 `LiDAR` 数据上，尤其在室外环境更明显。为缓解这一点，有些方法改为建模局部 keypoints 的统计分布，例如直方图式描述子。

不过，这类方法仍然受制于一个根本问题: 它们只利用局部邻域信息，缺乏整帧 scan 的度量结构上下文，因此 descriptiveness 往往不够强。在大规模数据库中，逐点或逐 patch 匹配的代价也较高，所以纯 local descriptor 方法通常需要额外索引结构或 coarse-to-fine 检索策略。

#### 8.3.2.2 全局描述子

global descriptors 则把整个 scan 或 submap 压缩成一个向量或 signature，用于快速数据库检索。它们在大规模回环候选生成中非常有吸引力，因为速度快、存储紧凑。

不过，global descriptor 设计必须在判别力和不变性之间权衡。表示太简单会导致 aliasing，表示太复杂则会降低效率或泛化性。书中进一步给出两类常见的 coarse representations。

第一类是 `Bird's-eye-view (BEV)`。它把 `3D` 点云转换为自顶向下的结构化粗粒度图像，可采用极坐标表示，也可采用稀疏网格表示。像 `Scan Context++` 和 `RING++` 就属于这一类方法: 前者提出了 yaw alignment matching 以获得朝向不变性，后者则利用 `Radon transform` 从理论上分析其 `invariance` 与 `awareness`。

第二类是 `range images`。与直接使用 `3D` 点云不同，range image 为一帧 scan 提供了结构化、天然对齐的表示。随着高密度 multi-beam `LiDAR` 的发展，这种表示变得更加实用。它的一大优势，是可以直接借用 computer vision 中成熟的 `CNN` 或 `Vision Transformer` 工具来提取细致特征。`OverlapNet` 通过在 range image 上施加 overlap loss 来获得 yaw invariance，而更近期的 `FRAME` 则展示了基于 range images 的矿井 `LiDAR place recognition`。

#### 8.3.2.3 高层或组合描述子

为了为一帧 scan 构造描述子，无论它最终是 local 还是 global，部分方法都会采用 hybrid strategy，甚至直接学习一个 place similarity function 来完成地点识别。

第一类是 `segmentation-based approaches`。前文已经提到，若用单一描述子来表示整帧 scan，就容易受到 `structural aliasing` 的影响。为此，有研究尝试用一组具有实际意义的物体或分割片段来描述一个地点，从而提高唯一性与描述能力，并尽量避免 perceptual aliasing。代表方法包括 `SegMap` [285] 与 `InstaLoc` [1273]。

第二类是连接 local 与 global 的桥接式描述子。前面介绍的 global descriptors 往往只有在点云投影能稳定对齐到某个固定方向时才效果最好，例如固定俯视图或球面视图。这会限制方法在道路上沿较可预测方向行驶的车辆场景中使用。为缓解这一问题，`BTC` [1257, 1258] 同时利用了 local 与 global descriptors，试图在保留局部几何的同时也保留 scan 的整体结构，从而把两类表示的优点结合起来。

第三类是 `Direct 3D Data Processing`。较新的数据驱动方法不再依赖人工设计规则，而是直接在原始、非结构化的点集上完成检索。尤其是一些基于深度学习的方法 [1118, 156]，尝试提取对不同传感器稀疏度与局部表面分布都更鲁棒的 point-wise features。`Cattaneo` 等人 [156] 进一步表明，若在 pipeline 中联合使用用于 place discrimination 的 `triplet loss` 和可微的相对位姿估计，不仅能改善 place retrieval，还能提升后续配准，从整体上帮助 `LiDAR SLAM`。这也可以被解释为: 对 `awareness` 的强调，往往会同时提升 `invariance` 与判别能力。

### 8.3.3 LiDAR 地点识别总结

总的来说，`LiDAR place recognition` 与 visual place recognition 扮演着相同角色，也共享很多共同属性，其性能同样由类似指标来定义。

作者还特别指出，两种传感器模态及其相关技术在很多情况下是互补的: 许多 `LiDAR place recognition` 失败的位置，visual place recognition 反而可以成功；反之亦然。因此，在移动机器人导航系统中，往往会同时使用这两种模态，以获得更稳健、更可靠的结果。

下一节将进一步说明，如何把 `LiDAR place recognition` 与 `pose-graph optimization` 结合起来，校正 `LiDAR odometry` 不可避免会积累的漂移，从而构建一个一致、准确且可扩展的 `LiDAR SLAM` 系统。

## 8.4 LiDAR SLAM

第 `8.2` 节介绍的 `LiDAR` 里程计系统，目标是随着传感器在环境中运动，持续估计一个局部一致（locally consistent）的运动轨迹。但只要运动距离足够长，这种局部估计就不可避免地会逐步积累漂移。

为了抵消这种漂移，完整的 `LiDAR SLAM` 系统需要在整段建图历史上维护一个全局一致（globally consistent）的估计。实现这一点的关键，是识别传感器何时重新回到先前访问过的区域。这类识别事件称为 `loop closures`，它们不仅可用于修正当前位姿，还可用于整体回修此前的整条轨迹，以及与之对应的地图表示。

因此，`LiDAR SLAM` 的一个核心性质，就是持续维护一张全局一致的地图。要做到这一点，系统必须让当前观测与过去观测在同一地图表示中彼此一致；而在大尺度、甚至动态环境中，这本身就是一项相当困难的任务。

前面在第 `8.2` 节中，我们已经讨论了漂移可低至每公里约 `1 m` 的 `LiDAR` 里程计；第 `8.3` 节又回顾了用于判定回环的 `LiDAR place recognition` 方法。`LiDAR SLAM` 所做的，就是把这些组件整合起来，在机器人、车辆或感知平台板载实时运行的条件下，持续维护一致的轨迹与地图表示。

大多数当代 `LiDAR SLAM` 系统 [66, 845, 282, 265, 1252] 都由 Figure `8.7` 所示的几个组件构成。接下来的 `8.4.1` 节将更详细讨论这一系统结构，`8.4.2` 节则重点介绍 backend optimization 与 map update，并说明如何在后端优化过程中融合 loop closures 来校正机器人轨迹。更进一步的高级话题，还包括 multi-session 与 multi-robot mapping，它们关注的是如何把多次建图任务，或多个平台同时采集的数据，融合到同一个全局参考系中。

图 `8.7` 展示了典型 `LiDAR SLAM` 系统的概念结构，由多个组件组成：`(1)` 里程计估计机器人 / 传感器位姿；`(2)` 回环检测判断某个地点是否已被重访；`(3)` 位姿图优化利用回环约束，通过因子图优化来校正位姿轨迹；`(4)` 地图更新利用最新的位姿轨迹来修订地图表示。

### 8.4.1 LiDAR SLAM 系统结构

在实现一个 `LiDAR SLAM` 系统时 [66, 845, 282, 265, 1252]，通常会把系统拆成多个模块: 一个模块持续维护相对位姿估计，其他模块则负责发现和利用 loop closures 来重新估计整条轨迹，并据此更新地图表示。典型地，odometry estimation module 以 `LiDAR` 传感器帧率运行，例如 `10 Hz`；而其他模块则以更低频率工作，例如 `1 Hz`。

更具体地，结合图 `8.7`，作者把典型系统结构分成四步。第一步 `(1)` 是 odometry component。它通常以 `frame-to-map` 的方式，用当前 active local map 来估计传感器位姿。对这个 odometry module，最常见的做法是把新到达的 laser scan 注册到一个 rolling / active local map 上，也就是前文 `8.2` 节讨论的 scan registration 流程。这个模块通常只使用传感器附近的局部数据，因此也常说它只访问传感器直接邻域内的 active local map。

第二步 `(2)` 是基于 `LiDAR` 的 place recognition。它的作用是找出潜在 loop closure candidates，正如 `8.3` 节所讨论的那样。不过，place recognition 只能说明“两处地点可能相同”，并不能直接给出两个地点之间精确的相对位姿。要得到真正可加入图中的 loop constraint，还必须对相应 scans 做 fine registration，最典型的工具就是 `ICP`。

为了让精配准收敛，通常还需要一个足够好的相对位姿初值。对较小的 pose graph，可以直接利用现有图结构中的几何先验；若没有可用几何先验，近年的 global registration methods 则提供了另一种初始化手段，它们不依赖初值，也能在 scan overlap 较低时相对稳健地估计两帧之间的相对变换 [1222, 664]。

第三步是在候选 loop closure 进入图之前进行 validity check。系统往往会依据一些 heuristics 来判断该候选是否可信，例如两帧之间的 travel distance、time difference，或基于 `RANSAC` 对齐得到的几何一致性分数。其目的很明确: 尽量避免把错误的 loop closures 加入 pose graph。

第四步 `(3)` 才是 backend `pose-graph optimization (PGO)`。它综合所有 `SLAM` constraints，对整张图做优化，恢复修正后的 pose trajectory。作者说明，后文将更详细讨论 `PGO` 本身。完成轨迹修正后，还需要第五类动作 `(4)`，也就是 map update mechanism: 按照校正后的轨迹，把传感器 measurements 融入一个统一地图表示中。最常见的做法是直接使用 points [256]；其他工作则用 `surfels` [282, 845, 66] 或 implicit representations [1252, 265]，希望提升地图质量或获得更强的 probabilistic foundation。关于这些 dense map representations 的技术细节，作者把读者引向第 `5` 章。

总之，一个完整的 `LiDAR SLAM` 系统并不是单一配准器，而是由局部 odometry、place recognition、loop verification、backend optimization 与 map update 共同组成的分层系统。

### 8.4.2 Pose-graph Optimization 与地图更新

在 `LiDAR SLAM` 后端中，`pose-graph optimization` 是最核心的步骤之一。它的任务是综合 odometry constraints 和 loop closures，对整条轨迹进行全局一致化修正。由于图里通常只包含相对位姿约束，因此这种图也被称为 pose graph。

图 `8.8` 给出了一个用位姿图表示的 `SLAM` 问题。每个节点表示传感器的位姿，边则表示来自里程计（橙色）和回环闭合（洋红色）的约束。`Prior Factor` 用于固定图的原点；在可获得惯性传感时，还可使用可选的 `Attitude Factors` 来约束 `pitch` 和 `roll`。引自 [886]。

作者指出，这类问题可使用通用优化器，如 `g2o` 或 `GTSAM`。由于约束集通常非常稀疏，配合稀疏矩阵重排、重线性化和增量求解器，拥有上千节点的 pose graph 也可以在很短时间内完成更新。

但仅仅纠正轨迹还不够，因为地图本身也包含历史测量积分出来的结构。一个简单做法是根据修正后轨迹重新把所有历史观测融回地图；但这要求系统长期保存所有原始观测，在大规模环境中往往难以接受。因此另一条路线是 deform map，或者把 map elements，如 surfels 或 submaps，显式链接到 poses 上，使它们能随 pose graph 的修正一起被拉回一致状态。

图 `8.9` 说明，通过引入回环约束，`SLAM` 系统可以为重访地点生成全局一致的地图。左图显示仅依赖里程计时的重访轨迹：在该路口，首次访问（蓝色）与当前访问（紫色）之间存在明显失配；右图显示在融入回环后经过位姿图优化的结果，从而得到一致的道路路口地图。

作者还讨论了 scalability。实际系统不会在传感器频率上给 pose graph 每一帧都加节点，而往往只在每隔一定距离、一定信息量或者每个 submap 粒度上加入节点。这样才能把 `LiDAR SLAM` 扩展到城市级地图。

此外，backend 还必须考虑 odometry 自身的不确定性，以及 loop closure 约束中可能存在的错误。作者指出，可以通过 data-driven covariance 来更合理地分配误差，也可以采用 robust `PGO` 方法对可疑约束做 down-weight、禁用或剔除。

### 8.4.3 多机器人与多会话 LiDAR SLAM

随着单机器人单次任务 `LiDAR SLAM` 的成熟，研究自然延伸到了 multi-session 与 multi-robot 场景。这样做的好处很多: 可以把多个不同时刻的扫描结果合并起来延展地图，也可以让多台机器人共享一个共同地图用于协同探索，还可以对同一地点的多次测绘做时序对比以检测环境变化。

但作者特别指出，这里面存在一个常被低估的问题: 即便现代 `LiDAR SLAM` 精度已经很高，一张最终地图内部仍会保留小量误差。若把两次任务最终输出的 point cloud 直接做刚体配准，很容易产生 double-wall 等假变化现象。因此，更合理的做法通常是把每次任务的后端约束保留下来，在统一图里共同优化。

图 `8.10` 对比了 `(a)` 两张全局点云的朴素直接对齐，与 `(b)` 多任务位姿图松弛的结果。`(a)` 两张全局点云之间的点到点距离会表现出“双墙”现象，从而产生虚假的变化；`(b)` 多任务松弛可减小点到点距离，使结构重建更清晰。引自 [950]（©2024 IEEE）。

一种做法是把多个 session 的约束直接转移到单一全局图中；另一种做法则是保留每个 session 自己的局部图结构，再通过额外变量把各自坐标系联系起来。作者提到 `anchor node` 就是一个很有代表性的辅助变量，它可以显式表示不同任务之间的坐标系偏移，从而方便地实现多 session 地图的联合优化。

图 `8.11` 展示了一个施工现场的多会话 `SLAM` 地图。通过建立会话间回环约束（红色），并联合优化五次建图会话的轨迹（绿色），将五个不同的建图会话融合在一起。

作者还指出，多 session `SLAM` 与单 session `SLAM` 的一个根本差异在于: 在完全断开的任务之间，没有现成的几何先验来生成第一个跨任务约束。因此，place recognition 在这里几乎成为唯一的桥梁。

#### 8.4.3.1 多机器人 SLAM

real-time multi-robot `SLAM` 比多 session 更进一步，因为它要求多个机器人在现场同时采集数据，并在线融合成统一地图。这样做能够让机器人团队共享探索结果、避免重复劳动、联合选择 frontiers，并共同支持搜索救援等任务。若能做到这一点，机器人团队就可以真正协同工作，高效探索区域、识别可通行路线，并发现感兴趣的人或物体。这种能力既适用于 search and rescue，也适用于军事等场景。

作者区分了 centralized 与 decentralized / distributed 两类系统。前者由现场机器人把观测传回 base station，由中心节点统一构图和优化；后者则让每台机器人本地构建自己的 `SLAM` 地图，再把多个本地图在某种中心节点或分布式协议中融合。采用 centralized 架构时，移动平台本身的 sensing 与 compute 可以保持相对简单，例如仓储场景中的一些机器人系统就是如此。

为了说明该领域的演进，作者回顾了两个重要挑战赛。2010 年的 `MAGIC` 挑战中，获胜队伍使用多台带 `2D LiDAR` 的机器人，在基地站完成全局 `2D` 多机器人 pose-graph 地图构建。更具体地说，Team Michigan 部署了 `14` 台 `3D` 打印机器人，每台都配备 `2D LiDAR` 与 camera；机器人本体执行 `2D LiDAR odometry`，再把 pose-graph constraints 传回 base station，由后者拼装成全局 `2D` multi-robot map。

到 2021 年的 `DARPA Subterranean Challenge`，系统已经发展到在复杂 `3D` 地下环境中融合 `3D multi-beam LiDAR`、visual odometry、wheel / legged odometry，甚至 thermal odometry，以应对隧道等几何退化场景。参赛队伍后来发表了系统综述 [299]，比较了各家的 fielded systems。一个共同特点是，多数队伍采用 semi-decentralized 架构：每台机器人在机载计算机上维护 pose-graph-based `SLAM` 地图，再由中心 base station 汇总成 multi-robot `SLAM` map。

图 `8.12` 说明，多机器人 `SLAM` 从 `2010` 年 `MAGIC` 挑战赛到 `2021` 年 `DARPA SubT Challenge` 期间，已经从 `2D` 发展到完整 `3D`。图中展示的是这两项挑战赛的获胜队伍：密歇根大学队和 `Cerberus` 队。图片由 `Edwin Olson` 和 `Cerberus` 团队提供。

这些比赛也暴露了新的关键问题: compression、communication 与 consistency maintenance。由于机器人团队必须维护动态无线 mesh network，把数据持续回传给基站，因此地图表示和压缩方式直接影响系统成败。作者举例说，CSIRO 的 `WildCat SLAM` [600] 使用 compressed surfel representation 表示 local submaps，这种 surfel map 相比原始 point clouds 占用空间小得多，从而显著降低传输 map 与 pose-graph constraints 所需带宽；在决赛阶段，每台机器人完整地图仅需 `21.5 MB`。

如作者所言，最复杂的问题其实是 fully distributed `SLAM`：不再依赖单一中心，而是要求每台机器人都在通信与规模约束下，自己维护整体联合地图的表示。已有一些工作 [493, 1094] 开始探索这一方向，重点研究如何逐步在机器人之间共享约束集与 local submaps。这里最关键的问题之一，就是如何在持续异步通信下保持整张地图的一致性。

## 8.5 延伸阅读与最新趋势

作者指出，尽管自 `LOAM` [1268] 以来，`LiDAR SLAM` 在过去几十年里取得了显著进展，高精度 odometry [1210, 998, 1132, 256] 和高效 pose-graph `SLAM` 系统 [66, 265, 845, 902] 也不断涌现，但这个方向仍存在大量尚未解决的问题。

第一个重点方向是 `robust and resilient perception`。`Zhao` 等人 [1287] 的近期鲁棒性评估表明，当前 `LiDAR SLAM` 在 cluttered、unstructured environments 中仍难以稳定工作。`DARPA Subterranean Challenge` 参赛队伍的综述 [299] 也指出，structure-less corridors、地下矿井，以及雪、雾、粉尘等极端天气和低可见度条件，都是现有系统的薄弱环节。

作者进一步强调，许多 `LiDAR SLAM` 系统目前仍主要通过 feature engineering 和人工参数调节来获得最佳性能，而不是建立在严格的 formal robustness evaluation 之上。某些方法，例如 `KISS-ICP` [1132]，已经开始根据运行场景动态调整算法行为；未来改进的一个重要方向，就是让系统具备更强的 introspection ability，能够根据环境变化主动调节自身。

在 place recognition 方面，研究也非常活跃。作者特别提到两篇综述 [1006, 1248] 为该领域提供了良好基础。当前热点包括:

- 跨不同 `LiDAR` 传感器类型都能稳健泛化的 retrieval 方法 [528]。
- `LiDAR` 与其他模态之间的 heterogeneous place recognition，例如和 radar [1247] 或 `OpenStreetMap` [211] 的跨模态识别。
- 将 `LiDAR` 的 intensity information 与传统 `XYZ` 数据结合，提升识别性能 [999, 1150]。
- 面向 multiple mapping sessions 的 long-term place recognition [565]，以及与 change detection、lifelong map management 相关的问题 [564, 1250]。

第二个大方向是 `multi-sensor fusion`。将多种互补传感器结合，是构建更 robust、更 resilient 机器人系统的重要路径。例如，radar 在雨天、烟雾中往往优于 `LiDAR`，而视觉 tracking 在隧道等 `LiDAR` 易退化场景中又能补足几何信息 [1190, 1286]。但随着额外传感器加入，系统不可避免地面临数据冗余问题，因此开放问题就变成了: 如何在 redundancy 和 lightweight computation 之间取得平衡？如何高效挑选多传感器估计中最可靠的信息？

作者指出，当前解决方案横跨 early fusion 和 late fusion 两大路线。前者倾向于在同一 tightly coupled estimator 中直接融合多源观测，后者则让不同传感器各自完成 pose estimation，再在更高层进行组合。

最后，还有一项很实际但常被忽视的问题: 多传感器系统可能既没有足够精确的 calibration，也没有足够严格的 temporal synchronization。这会让 full, tight sensor fusion 在长时间运行中变得非常困难。围绕这些问题，仍然存在大量值得深入研究的空间。

作者最后从应用视角补充说，`LiDAR SLAM` 最终要服务的场景跨度极大，从穿越森林的轻型无人机，到扫描施工现场的手持设备，再到恶劣天气中的自动驾驶汽车。除了传统电子系统里的 `SWaP`（size, weight, power）约束外，今天的 `LiDAR SLAM` 还必须同时满足 accuracy、robustness、computation 和 latency 等指标。例如，对自动驾驶汽车而言，`LiDAR` 感知与 `SLAM` 系统的计算延迟会直接影响它对潜在碰撞的反应速度。因此，在某些应用里，最精确、最复杂的系统未必就是最优选择。

## 第 9 章 Radar SLAM

### 本章概览

本章讨论在 `SLAM` 中使用 radar（`RAdio Detection and Ranging`）。与 cameras 和 `LiDAR` 相比，radar 长期以来相对“采用不足 / 使用不足（undersubscribed）”；但由于它能够在低可见度条件下工作、具备远距离感知能力，并且原生就能产生速度信息，因此其受关注程度正在上升。

本章将依次讨论机器人中常见的 radar 传感器类型、它们独特的感知原理，以及伴随而来的若干挑战（`9.1`）；随后介绍 radar filtering、radar odometry（`9.2`）、place recognition（`9.3`）以及完整的 `Radar SLAM`（`9.4`）。Figure `9.1` 展示了这些模块如何组合在一起。最后，作者会讨论 radar `SLAM` 数据集（`9.5`）以及 radar 在未来的发展前景（`9.6`）。

Figure `9.1` 的核心含义是: radar-based `SLAM` 的信息流与其他 `SLAM` 系统总体模式相同，但每个处理环节都需要针对 radar 的特性进行调整。原始 radar measurements（来自 spinning radar 或 phased-array radar）先经过 radar filtering，形成 pseudo-measurements；前端利用这些量完成里程计估计，地点识别模块输出 loop-closure constraints；后端再通过基于 `factor graph optimization` 的 `Radar SLAM` 联合估计最终 pose 与 map。

## 9.1 Radar 简介

本节介绍机器人中最常见的两类 radar: 旋转式 radar（spinning radar）与 `system-on-a-chip (SoC) radar`，并回顾 `mmWave`、`FMCW`、`Radar Cross Section (RCS)`、Doppler 效应与角度估计等基础概念。

### 9.1.1 传感器类型

机器人里常见的两大 radar 类别，主要差异在于它们如何发射、接收并整理回波。旋转式 radar 通过机械方式转动单一天线；`SoC radar` 则通过多发射/多接收天线阵列配合片上处理来推断目标角度、距离和速度。两类传感器各有优劣，其输出数据形式也完全不同。

Figure `9.2` 在这里提供了一个很有信息量的对比。图中展示了机器人中最常见的两类 radar：spinning radar（左上）与 phased-array `SoC radar`（右上），以及它们各自生成的主要数据产品；这些原始数据之后通常还会经过 radar filtering，转成稀疏 point cloud（下方）。图右下角给出了一个“充满烟雾的隧道”例子，其中红色是 `LiDAR` 点云，由于烟雾影响而量程明显受限；蓝色是 `2D spinning radar` 点云，可以在远距离清晰看到隧道壁；绿色是 `SoC radar` 生成的 `3D` 点云，能够看到车辆前方的墙面和地面。图左下角则给出了同一场景下的 thermal image、可见光 / `RGB` 图像，以及从中提取蓝色点云的 polar radargram。

#### 9.1.1.1 旋转式 Radar

旋转式 radar 也常被称为 scanning radar 或 imaging radar。它借助转动 radar 传感器逐步扫描周围环境，从而形成精确的极坐标表示，典型输出就是 `polar radargram`。这类成像 radar 的一个突出优点，是量程通常很远，许多设备能够探测到 `100` 米以上的目标；在某些工作模式下，还可以利用 Doppler 效应估计这些目标的速度。

与 `LiDAR` 相比，旋转式 radar 的一个明显局限，是它通常只能在二维平面内提供数据，无法直接测量被探测目标的高度。即便回波实际上可能来自天线垂直波束宽度覆盖的不同高度，这一限制依然存在。此外，由于它常常在每个天线方位角只发射和接收一次脉冲，因此有些设备并不直接提供速度信息，因为速度估计需要多个脉冲；较新的工作则会利用多个相邻方位角的信号来恢复速度。

作者还指出，像当前广泛使用的 `3D LiDAR` 那样，构造带有多条垂直波束的机械旋转式 `3D radar` 在工程上并不可行，因为 radar 的天线和聚焦机构相对于激光二极管系统要大得多。

#### 9.1.1.2 SoC Radar

`System-on-a-chip (SoC) radar` 将处理单元集成在少量芯片上，这些芯片要么直接安装在贴片天线阵列上，要么集成在印刷电路板中。由于其结构紧凑且没有机械运动部件，`SoC radar` 通常更轻、更省电，也更适合车载和机器人部署。

这类传感器的性能在很大程度上取决于天线阵列设计，以及厂商如何将多个天线的测量进行专有信号处理。其主要输出通常是 `radar datacube`。在表示上，`SoC` 回波往往以方位角、俯仰角和距离为球坐标轴来组织；再加上径向速度这一维后，常被称为 `3+1D` 或 `4D radar`。若垂直方向分辨率有限，只能得到距离、方位与速度，则通常被归为 `2+1D` 阵列 radar。

### 9.1.2 Radar 感知原理

本节回顾 radar 在机器人中的一些基本工作方式、天线结构、数据类型和核心挑战，并说明 radar 相较于其他测距传感器的独特性。`MmWave (millimeter-wave) radar` 指电磁波波长约为 1 到 10 毫米、频率约在 30 到 300 GHz 之间的雷达系统。由于汽车高级驾驶辅助系统（`ADAS`）对 76 到 81 GHz 频段的产业推动，很多机器人 radar 也工作在这一频段。

#### 9.1.2.1 Radar Cross Section

`MmWave radar` 通过发射电磁波并接收目标反射回来的回波完成感知。目标反射这些电磁波的能力，通常由 `Radar Cross Section (RCS)` 描述。`RCS` 受目标材料、形状和尺寸影响，可以理解为“与目标具有相同反射能量的假想球体大小”。因此，大型、坚固的结构，如车辆或厚实混凝土墙壁，往往具有更高的 `RCS`；而小型物体或行人则更低。

所谓 radar intensity，是指 radar 从目标接收到的回波强度，它是 radar 发射功率与目标 `RCS` 的函数。从本质上说，这个强度量主要由 radar 的发射功率和所遇目标的 `RCS` 决定。文献中指出，这一强度量对于提取目标的语义细节或辅助导航十分重要，因为较强回波往往对应环境中更鲜明、更容易识别的结构特征 [1239]。此外，radar 信号强度还会受到天线类型、所发射电磁脉冲的特性，以及天线检测目标回波能力的影响。

#### 9.1.2.2 FMCW 测距

一个 radar 至少包含一个发射天线（`TX`）和一个接收天线（`RX`）。虽然有时 `TX` 与 `RX` 可以是同一个单元，但多数 `SoC` 传感器会把它们分开布置，以便集成多个 `TX` 和 `RX` 天线。在 `Frequency Modulated Continuous Wave (FMCW)` radar 中，`TX` 发射的是频率随时间稳定上升的射频脉冲，也就是 `chirp`。最常见的是 sawtooth modulation，后文也会讨论另一种基于 `chirp` 的 triangular modulation。`Chirp` 打到环境中的目标后产生反射，`RX` 接收这些回波，并将其与原始 `TX chirp` 混频或相减，生成 `Intermediate Frequency (IF)` 信号。

`Figure 9.3` 展示了 `Frequency Modulated Continuous-Wave (FMCW)` radar 的工作方式：系统发射 `chirps`，也就是频率递增或递减的波；这些波被目标反射后再由接收端接收。距离以及速度，都是通过分析反射信号相对于发射信号的频率偏移和相位偏移来确定的。常见的频率调制策略包括 sawtooth 和 triangular，其中后者的优势在于可以把 Doppler frequency shift 与 range shift 解耦。

由于发射与回波具有相同斜率，混频后的 `IF` 会变成频率近似恒定的正弦波 `sin(2π f0 t + ϕ0)`。其中 `f0` 只依赖目标距离，而 `ϕ0` 则表现为一个额外的相位偏移。因此，`FMCW` 的关键思想，是把时间信息编码到连续波上，从而可以据此执行测距计算。

作者特别指出，这一点与 amplitude modulation 很不同。后者的测量精度主要依赖频带宽度；而 frequency modulation 对 `signal-to-noise ratio` 和目标 `RCS` 变化通常更稳健。在 `FMCW radar` 中，一串带有明确时间间隔的 chirps 仍然保持相干性，这意味着每个 chirp 都能被准确地对应回其所属的发射-接收对以及它在序列中的位置。因此，系统不仅能在 amplitude 上关联信号，也能在 phase 上关联信号。这相比早期依赖非相干幅值测量、并常需人工从 clutter 中挑选有效信号的 time-of-flight pulsed radar，是一次很大的进步。

在公式上，若电磁波速度为 `c`，回波到达初始时刻为 `t0`，目标距离满足

```text
d = c t0 / 2
```

当发射 `chirp` 的带宽为 `B = fM - fm`、持续时间为 `T`、斜率为 `S = B / T` 时，中频 `f0` 与距离 `d` 之间可写成

```text
f0 = 2Bd / (Tc) = 2Sd / c  =>  d = c f0 / (2S)
```

虽然书中公式在频域写出，但真实系统中信号通常是以数字形式采集的。常见做法是使用高速 `Analog-to-Digital Converter (ADC)` 对回波进行高频采样，典型采样间隔可达到 `100` 纳秒甚至更短。随后，采样数据会经过一系列 `Fast Fourier Transform (FFT)` 运算，生成 frequency-amplitude 图，再从中识别信号峰值。对单个 `chirp` 及其回波执行 `FFT` 可以得到 range frequency；对多组 chirp 序列进一步做 `FFT`，则可用于提取 velocity frequency。

#### 9.1.2.3 用锯齿调制确定距离与速度

Sawtooth frequency modulation 常见于 `SoC radar` 和部分旋转式 radar。此时 `IF` 信号相位 `ϕ0` 可写为波长 `λ` 与距离 `d` 的函数:

```text
ϕ0 = 2π fm t0 = 4π d / λ
```

虽然 `f0` 和 `ϕ0` 都与距离有关，但相位只在距离变化很小且不发生相位包裹时更有用，因此它通常不直接用来做绝对测距，而是用来估计小的距离变化 `Δd`，也就是径向速度。

在 Doppler 速度估计中，至少需要连续两个 up-chirp。由于两个 chirp 之间时间间隔非常短，量级可到 40 微秒，因此它们的测距频率几乎相同，但相位会不同。相位差 `Δϕ` 对应目标在这段时间内的位移，从而得到

```text
Δϕ = 4π Δd / λ = 4π v T / λ
v = λ Δϕ / (4π T)
```

由此也能看出，最大无歧义速度受 `|Δϕ| < π` 限制，因此 `vmax = λ / (4T)`，它只由信号波长与 chirp 间隔决定。这解释了为何 radar 天然适合进行 Doppler-based 自运动估计。

`Figure 9.4` 说明 radar 可以利用 Doppler effect 测量设备与其正在成像的场景之间的径向速度。

#### 9.1.2.4 用三角调制确定距离与速度

Triangular frequency modulation（见图 `9.3`）也可以使用，例如它有时会用于旋转式 radar。实际上，当由一个 up chirp 导出中频时，其中包含两个部分：`(i)` 由信号传播时延产生的频移 `f0,t`，以及 `ii)` 当 radar 与目标之间存在相对速度时，由 Doppler 效应产生的频移 `f0,d`。因此有

```text
f0,up = f0,t + f0,d
```

然而，如果在一个 up chirp 之后再跟一个由同一目标反射回来的 down chirp，那么时间项对应的频移符号会翻转，而 Doppler 项的符号不会翻转：

```text
f0,down = -f0,t + f0,d
```

由这两个式子可以解出 `f0,t` 与 `f0,d`：

```text
f0,t = (f0,up - f0,down) / 2
f0,d = (f0,up + f0,down) / 2
```

最后，可据此计算距离和速度：

```text
d = c f0,t / (2S)
v = c f0,d / (2ST)
```

需要注意的是，前面式 `(9.2)` 中给出的距离计算并没有对 Doppler 效应进行校正，而这里的做法进行了校正。使用 triangular modulation 的代价，是距离和速度计算会引入更高延迟，因为此时需要用到稍旧的数据；但它的好处是我们不需要处理信号相位。

#### 9.1.2.5 SoC Radar 的角度估计

对旋转式 radar 而言，确定目标角度是很直接的，因为系统在每个时刻都只朝单一方位角聚焦波束。`SoC radar` 的角度估计则更复杂一些，但也可以通过与前面类似的相位差计算来完成。

若存在多个接收单元，它们之间相隔距离 `ℓ`，那么反射回波中就会出现传播距离差 `Δd`。根据几何关系 `Δd = ℓ sin θ`，第二个接收天线比第一个多走了一段 `ℓ sin θ` 的路径，如图 `9.5` 所示。这对应于两个 `RX` 天线接收到的信号之间具有如下相位差：

`Figure 9.5` 展示了 phased-array radar 的 angle-of-arrival estimation：`TX` 发射天线发出的信号到达两个相距 `ℓ` 的 `RX` 接收天线时，其相位差对应于目标的入射角 `θ`。

```text
Δϕ = (2π / λ) ℓ sin θ
```

给定测得的相位差 `Δϕ`，目标到达角 `θ` 可计算为

```text
θ = arcsin( λ Δϕ / (2π ℓ) )
```

原则上，一个 `TX` 和两个 `RX` 天线就足以确定目标角度；但如果需要更高分辨率，或者希望区分多个彼此接近的目标，就需要两个以上的 `RX` 天线。此时，返回信号在每个 `RX` 上都会额外累积一个 `Δϕ` 偏移。对这些 `RX` 通道上的信号序列进行采样，并对该序列执行 `FFT`，就可以较可靠地估计 `Δϕ`。

上面的例子实际上对应一种 `SIMO` phased-array radar，也就是 `single input, multiple output`。而用于 `SLAM` 的多数 radar 传感器则是 `MIMO`（`multiple input, multiple output`），具有多个 `TX` 和 `RX` 天线。只要 `RX` 天线能够区分来自不同 `TX` 的信号，就可以通过额外增加一个 `TX`，而不是把 `RX` 数量翻倍，来达到相同分辨率。为保证多个 `TX` 信号彼此不相关（orthogonal），可采用多种技术，例如 frequency division、code division，或 `time-division multiple access (TDMA)`，即让每个发射器使用不同时间槽。同样的原理还可以扩展到二维 `TX-RX` 阵列，同时测量 azimuth 与 elevation，从而产生 `3+1D` 数据。

### 9.1.3 Radar 应用面临的挑战

与任何传感器一样，radar 也有一系列需要认真对待的噪声与失真来源。相较视觉和 `LiDAR`，radar 特有的问题包括多种形式的 measurement noise、宽波束导致的距离偏置与稀疏读数、接收机饱和、`speckle noise` 和 `multipath` 等。这些问题都会直接影响后续的点云提取、配准、回环识别与建图质量。

`Figure 9.6` 展示了将 `(a)` 中的 polar radargram 变换到 `(b)` 中的 Euclidean coordinates 后的结果，并在 `(c)` 中放大显示了 radar 相比其他传感器更独特的几类噪声。最常见的是 speckle noise，图中绿色圆圈标出了具有歧义的 clutter；multipath reflections 则来自回波先撞击附近墙面或地面再返回天线，从而生成对真实目标的镜像反射，图中红圈标出了这类重复回波。原图取自 Mulran dataset [566]。

#### 9.1.3.1 Speckle 噪声

Radar 测量噪声来自多个来源，包括热噪声、电子缺陷以及目标 `RCS` 的变化。当 radar 发射一个电磁脉冲时，它会接收天线视场范围内所有物体反射回来的能量。这个脉冲与物体相互作用后会让 radar 波发生散射，并导致相长和相消干涉。这些作用既可能产生虚假信号，也可能抵消本来真实存在的合法回波，而不论这些信号最初来自哪里。

这些因素会使频域中的信号发生变化，其中最突出的峰值往往同时混合了真实目标和虚警。若缺少区分真实与虚假回波的机制，传感器最终就会产生一种散乱点状的输出模式，这就是所谓的 `speckle noise`。对于对位姿估计和特征匹配都很关键的 landmark 识别而言，通常必须跨多个 scans 去估计这些反射的不确定性。

#### 9.1.3.2 多路径效应

`Multipath` 是另一种典型误差来源。一个目标的回波并不一定沿最短路径返回，有些信号会先反射地面或墙面，再回到接收机。结果是，同一个真实目标可能对应多条不同传播路径，系统于是会感知到一些似乎位于墙后、地面下方或其他不真实位置的“鬼影目标（ghost objects）”。如果不在前端或后端中抑制这些离群观测，点云地图和定位结果都会受到显著影响。

#### 9.1.3.3 运动诱导畸变

和旋转式 `LiDAR` 一样，扫描式 radar 也会发生 `motion-induced distortion`。旋转式 radar 通常需要完成一整圈扫描才能形成一张极坐标图像，因此如果平台在这段时间内在运动，同一物体在一圈开始和结束时的测量位置就会不同。对于低频设备，这一问题尤其严重。例如常见的 Navtech 成像 radar 工作频率只有 4 Hz，在车辆平台上非常容易出现明显畸变；即便是更快的 10 Hz 或 50 Hz 设备，只要平台速度足够高，畸变仍不可忽视。

### 9.1.4 Radar 滤波

由于 radar 中虚假目标的分布会随时间变化，而且参数往往不可预测，因此静态阈值这类简单方法很难稳定工作。许多 radar 处理方法都要沿单个方位角分析一维信号，并判断哪些 range bins 对应真实目标，哪些只是噪声。

`Constant False Alarm Rate (CFAR)` 是最常见的一类方法。它将频域信号划分为若干 cell，以滑动窗口方式对“待测单元（CUT）”与其周围训练单元进行比较，从而动态估计背景干扰水平，并设置自适应阈值。经典版本如 `CA-CFAR` 使用均值统计量，`OS-CFAR` 则使用更鲁棒的排序统计量。`BFAR` 则是在 `CFAR` 的输出之上做简单仿射变换，以在自适应阈值和固定阈值之间取得更合适的折中。

不过，作者也指出，`Radar SLAM` 未必总是需要复杂滤波器。与传统目标检测相比，里程计更关心的是“哪些点能在不同时间、不同视角下稳定重复出现”。因此，很多系统采用更保守的策略，例如在每个方位角仅保留超过静态阈值的 `k` 个最强回波，其中 `k` 可从 `1` 开始逐步增加；也可以使用统计阈值，仅保留强度高于均值一个标准差的点。

`Figure 9.7` 给出了 radar filtering 的示例，对比了 `CFAR`、constant power threshold 和 `k-strongest` 三类策略。沿单个 azimuth 方向的 power/range 曲线以灰色绘出；真实目标回波会表现为尖峰，但其中也夹杂多个歧义峰值。`CFAR` 生成的自适应阈值用黑线表示，蓝点是 `CFAR` 报告出的 detections，在该例中包含了若干 false alarms；而 `k-strongest` 滤波器在这里取 `k = 12`，更保守，只返回主目标附近的点。图引自 Adolfsson 等人 [6]（©2021 IEEE）。

原文还补充了两类基于学习的代表做法。`Cheng` 等人 [195] 使用 `generative adversarial network (GAN)`，根据 `range-Doppler velocity matrices` 生成点云；`Xu` 等人 [1209] 则联合训练 regressor 和 classifier，其中 regressor 输出更高分辨率、质量更好的 depth readings，classifier 负责估计数据是否已经超出量程。它们的共同目标，都是从 radargram 中尽量只保留那些若换成 `LiDAR` 也会落在真实表面上的回波。

## 9.2 Radar 里程计

`Radar odometry` 的目标，是根据时间有序的 radar 观测估计传感器的自运动。典型方法会维护某种中间表示，如最近 `N` 帧读数、局部子地图或连续裁剪的局部地图，并关注局部尺度上的位姿估计精度。与其他模态一样，如果没有显式回环约束，再优秀的 radar 里程计也会随时间累积漂移。以旋转式 radar 为例，优秀方法的平移漂移量级通常仍在每 100 米约 1% 到 2% 左右。

`Figure 9.8` 给出了 Oxford Radar RobotCar dataset [127] 上的 open-loop radar odometry 示例，其中使用的是 `Navtech 2D spinning radar` 与 `CFEAR` 方法 [8]。蓝色曲线是真值轨迹，橙色曲线是里程计估计，灰色点是从 radargram 中提取的 point targets。

Radar 的一个独特之处，在于很多传感器能提供逐点 Doppler 速度估计，因此可构造几乎不依赖显式对应关系的 odometry。除此之外，作者还将现有方法概括为 direct、feature-based、registration-based，以及 motion compensation 几条路线。

### 9.2.1 Doppler 里程计

如果 radar 能直接测得目标的径向速度，那么就可以利用多个目标的 Doppler 约束来估计平台线速度。相关方法通常通过 `RANSAC` 和最小二乘来联合多个回波，得到对传感器速度的稳健估计。这种方法的优点，是物理意义明确，且不依赖传统 scan matching 中繁琐的点对应过程。

不过，Doppler 只能测量速度在视线方向上的投影，因此单个 radar 一般无法仅凭这些约束恢复完整角速度，除非利用额外运动学假设。例如，若已知 radar 相对车辆转动中心的位置，并假设系统遵循 Ackermann 转向且无侧滑，就可从 `2+1D` 数据中恢复线速度与角速度 [550]；`Galeote-Luque` 等人 [354] 进一步把这一思路扩展到 `3+1D`，可估计五个自由度，即三维线运动加上 `yaw` 与 `pitch`，但不包括 `roll`。更常见的做法，则是将 Doppler 估计与 `IMU` 结合，尤其使用 gyroscope 提供角速度信息。作者举例说，`Huang` 等人 [490] 将消费级 `IMU` 与级联 `SoC radar` 结合，在多样的室内 `3D` 空间中取得了较低漂移；`Kubelka` 等人 [608] 则比较了多种 `3+1D radar` 的配准式方法与 registration-free 的 Doppler + `IMU` 方法 [272]，发现后者在特征稀疏环境下误差最低，报告的 `4.5 km` 轨迹漂移可低至 `0.3%`。

`Figure 9.9` 给出了 radar-inertial velocity estimation system 的 factor graph 表示：系统在滑动窗口内联合估计过去 `K` 个时刻的状态，并使用 Doppler targets 与多组 `IMU` measurements 作为约束。图改编自 Kramer 等人 [605]。

由于 Doppler 型里程计是 radar 非常独特的一条路线，作者随后基于 `Kramer` 等人 [605] 给出一个具体的 Doppler factor graph 例子。其目标是在长度为 `K` 的滑动窗口内，联合估计过去若干时刻传感器平台在 body frame 下的速度，并利用 `IMU` 加速度计积分把这些速度状态连接起来。系统状态可写为

```text
x = [v_s^T, q_s^w^T, b^T]^T
```

其中 `v_s` 是平台速度，`q_s^w` 是用于表示姿态的 orientation quaternion，`b` 是 `IMU` 偏置。作者说明，这样建模的原因在于: radar 给出的速度估计本身不含偏置，但为了补偿重力对加速度计的影响，必须同时估计 `IMU` 的姿态，尤其是 `pitch` 与 `roll`。

在这一建模下，整体代价函数由 Doppler term 与 inertial term 组成；前者对每个 radar target 都引入一个速度残差，后者则由 `IMU` 误差构成。书中进一步给出了 Doppler 权重的定义，即用对应目标回波强度做归一化:

```text
w_d,k = P_d,k / Σ_(j∈D_k) i_j,k    (9.12)
```

其中 `w_d,k` 是第 `k` 帧中目标 `d` 的权重，`i_d,k` 是该目标的回波强度。

在该例子中，一次 radar 观测由一组 targets `D` 组成；对每个 `d ∈ D`，测量量写为

```text
[r, v, θ, ϕ]^T
```

分别表示距离、Doppler 径向速度、方位角和俯仰角。Doppler 速度测量 `v` 等于目标与传感器之间相对速度向量 `v_s` 在从传感器原点指向该目标的单位方向 `r_d,k^s / ||r_d,k^s||` 上的投影，也就是

```text
(v_s)^T (r_d,k^s / ||r_d,k^s||)    (9.13)
```

作者在这里假设场景中的目标都是静止的，只有传感器平台自身在运动。于是，每个 radar target 都为 body-frame 速度估计提供一个约束，其 Doppler 速度残差写为

```text
e_d,k = v_d,k - (v_k^s)^T (r_d,k^s / ||r_d,k^s||)    (9.14)
```

其中 `v_d,k` 是第 `k` 个时刻目标 `d` 的测得径向速度，`v_k^s` 是该时刻传感器速度，`r_d,k^s` 是从传感器指向目标的向量。由于 radar 测量往往含有非高斯噪声和虚假目标，作者建议对 Doppler 残差配合 `Cauchy` robust norm 使用。书中还特别提醒，上述推导默认 radar 与 `IMU` 共点且朝向一致；而一般情况下两者并不重合，文献 [604] 给出了更一般的推导，展示了 body angular rate 与 radar 传感器坐标系速度之间的耦合关系。

### 9.2.2 直接里程计

`Direct radar odometry` 不先提取点、角点或地标，而直接对原始 radargram 进行对齐。典型做法会把两张极坐标 radargram 看成图像，通过 phase correlation、Fourier-Mellin transform 等经典信号处理工具估计相对位姿。

具体来说，在极坐标图像中，方位角变化对应垂直方向平移，因此可以先通过相位相关估计旋转；随后把图像变换到笛卡尔坐标，再估计平移。这类方法利用了几乎全部原始信号，不必做稀疏点提取，因此理论上能保留更多信息。

但直接法有一个关键假设: 同一空间位置的功率回波在时间上足够稳定，这样相关性才有意义。实际中，这个假设常因噪声、动态物体和 radar 特有的外观变化而被破坏。为此，有工作训练卷积神经网络，只保留 radargram 中真正稳定、适合相关匹配的区域，从而避免噪声纹理和虚假回波干扰对齐过程。

### 9.2.3 基于特征的里程计

由于 `2D spinning FMCW radar` 的 radargram 在形式上很像鸟瞰图像，因此一些工作直接引入计算机视觉中的特征提取与匹配思路，如 `SIFT`、`SURF`、`ORB` 等，也有专门为 radar 设计的关键点提取器和描述子，如 `FSCD` 和 `BASD`。

基于特征的方法，首先要找到可以在连续帧中稳定重复出现的关键点，再为其构造局部描述子，之后通过描述子匹配和 `RANSAC` 去除离群点，最后用 `SVD` 或其他最小二乘方法恢复两帧之间的刚体变换。相对于纯配准法，这类方法在初值较差时通常更稳，因为描述子匹配提供了更强的全局辨识能力，而不只是依赖几何邻近性。

但 radar 的局部外观比普通图像更不稳定、更噪，也更依赖观测视角和环境结构，因此特征设计远比视觉困难。近年也出现了使用深度网络自动学习 radar 关键点和描述子的工作，不过这类方法往往需要带 ground-truth 位姿的数据集，训练成本和泛化问题都较突出。

### 9.2.4 基于配准的里程计

基于配准的 radar odometry 与 `LiDAR` 里程计在整体思想上很接近: 先把原始 radar 数据转成 point clouds 或更丰富的局部分布表示，再利用 scan registration 恢复相对运动。不同之处在于，radar 点云往往更稀疏、噪声更强，因此配准难度通常明显高于 `LiDAR`。

对旋转式 radar 而言，首先必须从原始 signal data 中筛选哪些 returns 可视为有效目标，也就是从大量 range bins 中挑出 relevant peaks。前文 `9.1.4` 已讨论过一些典型方法，包括 `CFAR`、利用噪声统计去除冗余或噪声回波 [158]、`BFAR` [25]、或在每个 azimuth 上直接选取最强的 `k` 个 returns。`Kellner` 等人 [549] 还使用 `DBSCAN`，让点提取不仅沿单个方位角进行，也考虑相邻 azimuth 之间的聚类关系。近期还有一些方法借助 machine learning，把 radargram 映射成更接近 `LiDAR` 的 point cloud [195, 1209]。作者指出，这里的关键难点在于选点数量的平衡: 选得太少会丢失信息，选得太多又会把大量噪声带入后续估计 [158]；例如 `CFAR` 在这方面就被认为较难调参 [127]。

在点云抽取完成后，可以直接拿来配准，也可以进一步估计 normals、planes 或局部 point distributions 等附加信息。虽然整体流程和 `LiDAR` scan registration 很像，但 radar data 的稀疏性和噪声使得纯粹的 pair-wise registration 更困难。

因此，一条非常常见的策略，是把当前 radar scan 注册到多个历史 keyframes，或者注册到一个由多帧聚合形成的 local submap，而不是只与上一帧做 frame-to-frame alignment。这样做的好处有三点:

- 通过更多 correspondences 缓解点云稀疏问题。
- 在 feature-poor environments 中减少 drift。
- 对移动物体、遮挡或偶发错误对应具有更强鲁棒性。

作者列举了几类代表性方法。`Continuous-time ICP` [127]、`power-shifted NDT` [614] 和 `CFEAR` [8] 都属于利用 multiple keyframes 或 submaps 的 registration-based radar odometry。以 `CFEAR` 为例，它从 radargrams 中按每个 azimuth 选取最强的 `k` 个 returns 形成 point cloud，然后把每个新点云同时与最近 `s` 个 keyframes 配准。其误差可以是 `point-to-point`、`point-to-line` 或 `point-to-distribution`，类似 `NDT` registration。对每个点，系统还会根据邻域点协方差估计 normal vector，并结合 normal 一致性、平面性以及邻域点数量，对匹配关系进行加权。

`Kung` 等人的方法 [614] 则采用固定阈值从 radargram 中提点，并将多帧点云聚合成一个 `NDT` radar submap，其中每个点的贡献由其 signal strength 加权。`Burnett` 等人 [127] 使用 `BFAR` [25] 提取点云，再将其聚合进 local submap，随后通过 `continuous-time ICP` 结合 `Gaussian process` motion prior 来优化轨迹。

前述方法主要面向 `2D` radar。对最近的 `3+1D radar` 而言，pipeline 相似，但 point extraction 通常在传感器内部已完成，因此研究者较少再显式设计 peak detector。由于 `3+1D` point clouds 天然带有 per-point Doppler velocity，这类方法通常先根据 Doppler 数据做 least-squares ego-velocity estimation，再将明显不符合静态场景速度模型的点视为 moving objects 或 outliers 剔除，之后再用 registration 精配准。`4DRadarSLAM` [1270] 使用一种针对 radar point clouds 改造过的 `GICP`，通过协方差矩阵对远距离点赋予更高不确定性；`4D iRIOM` [1309] 采用 one-to-many distribution-to-distribution matching，以缓解 radar 点云噪声与稀疏性；`EFEAR-4D` [1198] 则把 `CFEAR` 的思路扩展到了 `3+1D` 场景。

### 9.2.5 运动补偿

由于扫描式 radar 一帧往往跨越明显时间范围，因此 `motion compensation` 或 `de-skew` 是前端不可或缺的组成部分。原文给出一个具体结果: 对于频率仅 `4 Hz` 的低速旋转式 radar，仅使用常速度模型进行 `egomotion compensation`，就可把 `ATE (Absolute Trajectory Error)` 降低 `29%` [8]。方法上，系统可根据相邻两帧的相对位姿估计出平台速度，再利用逐点时间戳把每个点补偿回统一时刻。这样对单帧中每个 radar point 做时间对齐，可以显著缓解扫描式 radar 在缓慢扫掠期间累积的运动畸变。原文还指出，同一个常速度模型通常也会被用来给刚体 scan registration 提供初值；当每一帧经过运动补偿后再与前一帧完成配准，就可以把该帧作为一个新节点加入 `SLAM` pose graph。

但这仍然是在离散 pose graph 设定中工作。若采用连续时间轨迹表示，就可以在每个传感器读数的真实时间上直接查询姿态，从而更自然地完成去畸变，并获得更平滑、更准确的轨迹。书中举了两个例子: `Ng` 等人 [802] 在 automotive `SoC radar` pipeline 中采用样条（spline）轨迹表示；`Burnett` 等人 [129] 则使用 `Gaussian processes`，把 spinning radar 与 `IMU` 组合进同一个 factor graph 轨迹模型中。

## 9.3 Radar 地点识别

与视觉和 `LiDAR` 一样，地点识别（place recognition, `PR`）是完整 `Radar SLAM` 的关键模块。它不仅关系到 loop closure detection，也决定系统能否在长期运行中进行重定位。作者指出，对 radar 而言，获取同时对平移和旋转变化保持不变的描述子格外困难，这也是 radar `PR` 比很多人想象中更难的原因。

`Figure 9.10` 展示了从同一位置获取的两次 radar scans。其噪声模式会引入明显的 visual aliasing，从而让 place recognition 变得更困难。

### 9.3.1 Radar 地点识别中的独特挑战

从表面上看，早先章节中的许多 `PR` 方法似乎都可以通过数据格式转换后用于 radar，但 radar 确实存在一系列独特挑战。首先，radar 分辨率较低，细节更少，可辨别的局部特征明显不足。其次，宽波束会带来角度模糊，远处目标常表现为较宽的亮斑，并且当观测距离变化时，其外观会发生显著变化。再加上 radar 的信噪比较低、接收机饱和可能形成强烈的径向条纹，以及噪声模式本身会制造 aliasing，同一地点从很近的两个位置看过去，也可能显得很不一样。

不同 radar 类型的 `PR` 设计也不相同。长距离、`360` 度覆盖的旋转式 radar 更适合借鉴视觉 `PR` 或 `2D LiDAR PR` 的思路，因为它能输出完整 `2D radargram` 或对应点云；而 `SoC radar` 往往视场角更小、量程更短、点云更稀疏、更噪，因此更常沿着 `LiDAR PR` 的方向修改，但必须特别处理小视场和高稀疏性带来的描述子不稳定问题。当前文献中旋转式 radar 的 `PR` 工作更多，一方面因为其宽视场和长量程更利于 outdoor `PR`，另一方面也因为大规模数据集更丰富。

### 9.3.2 基于学习的 Radar 地点识别

对旋转式 radar 而言，把 `2D radargram` 当作图像来处理是一条自然路线。早期工作曾使用 `CNN` 来学习雷达图像的表示，并通过圆柱卷积等设计显式建模极坐标结构，以获得对旋转变化的鲁棒性。查询 radargram 可以与数据库中的参考 radargram 匹配，从而构成基于外观的拓扑式 `PR` 系统；后续工作则进一步在检索后加上位姿细化，形成 topometric localization。

一类非常关键的训练技巧，是利用雷达数据天然按时间顺序采集这一特点，无需精确度量真值位置，也能构造近似监督信号。研究者可以把时间上相邻、位置上相近的帧视为正样本，再通过极坐标平移人工施加旋转增强；而把时间上相距较远的帧视为负样本。这样便能在无精确 metric ground truth 的条件下，以近乎无监督的方式训练 `PR` 网络。

对于 `SoC radar`，学习式 `PR` 往往先将点云投影到 `2D` 图像平面，或直接在 `3D` 点云上做网络编码。有工作使用深度时空编码器配合 `NetVLAD` 生成地点描述向量，并用 `RCS` 信息重排序，过滤不相关静态特征；也有工作直接在 `3D` 点云上使用 kernel point convolutions，并训练“点重要性估计器”来突出真正有助于地点识别的稀疏稳定点。还有方法在前端先利用 Doppler 一致性筛掉明显动态或噪声点，再用专门面向 radar 的 backbone，如 `MinkLoc4D`，替代原本为视觉设计的 `NetVLAD`。

### 9.3.3 基于描述子的 Radar 地点识别

除学习方法外，手工描述子依然在 radar `PR` 中很有效。一条常见路线，是借用 `LiDAR` 领域已有的全局描述子，如 `Scan Context`、`RING`、`M2DP`，再根据 radar 数据格式进行改造。由于旋转式 radar 缺少高度信息，很多 `LiDAR` 描述子中使用的高度统计要被替换成 `RCS`、强度和点密度等信息；同时还要结合噪声过滤与运动补偿，以提升可比性。

例如，有工作把 `M2DP` 从 `3D` 点云适配到由 radargram 提取的 `2D` 点云上，并借助 `PCA` 特征值来判断某个扫描是否“足够有辨识性”。如果一个扫描的点分布呈现明显单向延展，例如高速公路一类缺少横向结构的场景，就不将其用于回环识别。另一些工作把 `RING` 改成适用于 radar 的 sinogram 描述子，并发现引入自相关可以提升在高噪环境中的匹配稳定性。

`Scan Context` 也是非常流行的选择。针对 `2D radargram`，有方法将每个 bin 内所有点的强度和累计，而不只记录点数，从而同时编码点密度与回波强度。为了减轻 radar 噪声与稀疏性的问题，描述子还可由多张经配准、去畸变和保守滤波后的极坐标图像聚合生成。对 `3+1D radar`，原始 `Scan Context` 可继续使用高度维度，但由于 radar 的高度测量噪声较大，也常改用 `Intensity Scan Context`，即在每个 bin 中存储最大强度值而不是最大高度。另一方面，小视场 `SoC radar` 很难像 `360` 度 `LiDAR` 那样天然获得旋转不变性，因此常要根据当前 odometry 先做 yaw 预筛选，再在有限候选范围内匹配描述子。

## 9.4 Radar SLAM

完整的 `Radar SLAM` 系统一般沿用与视觉、`LiDAR` 类似的总体框架: 前端估计 open-loop odometry，地点识别模块提出 loop closure 候选，后端基于 pose graph 或其他全局图优化框架进行轨迹一致化和地图更新。不同之处在于，radar 的观测更稀疏、更噪、物理上更复杂，因此每个模块都必须做出相应适配。

### 9.4.1 地图表示

许多 radar `SLAM` 系统使用的地图表示，与第 5 章介绍过的地图类型是相通的。不过，radar measurements 通常更加稀疏、噪声更大，而且常常包含虚警，因此建图时面临的挑战也更特殊。较早的一些 radar mapping 工作，直接沿用已有目标检测模型建立地图，常见做法是构造 `landmark map`，把每个被检测到的目标都作为地图元素。类似地，目标检测结果也能用于构造 `occupancy grid map`，并在建图时显式利用检测概率。

如果把信号中的单个峰值视作与 `LiDAR` 扫描中的点等价，那么它同样可以被用来构建 `2D occupancy grid map` 或点云地图。进一步地，提取出的点云还可以附带周围点的分布信息，用来估计方向和权重，从而形成类似 surfel 或 `NDT` cell 的表示。对于新一代高分辨率 `SoC radar`，由于其拥有更大的垂直视场和更多 `TX/RX` 天线，直接采用 `3+1D radar point cloud` 作为地图表示已经变得越来越常见。

另一种虽然相对少见、但很有特点的表示，是直接使用 full radar heatmap，也就是在峰值检测之前，为每个方向和距离 bin 保存的回波强度样本。在这种做法里，地图编码的是环境每个位置的全局反射功率，而不是占据证据；不同 heatmap 之间的二维对齐则通过相关运算完成。

书中还专门提到 `Kramer` 与 `Heckman` 提出的基于体素的 radar sensor model。该方法除了提供 odometry 之外，还能在存在视觉遮挡的情况下建立稀疏地图。它延续了 `Octomap` 中基于 `log-odds` 的 occupied/free cell 估计思想，但替换掉了 `Octomap` 原本的 `ray-cast` 模型。后者默认“传感器的第一次接触点”就是唯一与占据相关的位置；而广义化后的 radar 模型则考虑到 radar 具有穿透某些材料的能力，因此不再沿一条射线简单传播信息，而是在整个传感器视场内更新体素概率: 对存在 radar return 的体素提高概率，对漏检位置降低概率。

针对 radar 特性量身设计的另一类栅格表示，来自 `Nuss` 等人的 `probability-hypothesis-density multi-instance Bernoulli filter`。它把 grid cell 看成有限随机集，用专门的状态估计滤波方式处理动态障碍物，并能动态地融合 radar 与 `LiDAR` 数据。

对于 radar 数据天然存在的“低密度、高噪声”问题，近年来也出现了大量 machine learning 增强方法。典型思路是把 `LiDAR` 数据作为 ground truth，用学习模型提升 radar 输出的分辨率并降低噪声。例如有工作构建全局 radar map，然后利用预测网络对局部地图 patch 做超分辨、去噪，并补全稀疏与空洞区域，使结果更接近 `LiDAR` 地图。

最近，最初为 `RGB` 数据提出、后来又被用于 `LiDAR` 的 `neural fields`，也开始被应用到 `2D radar` 数据上。这类表示的核心特点是: 环境以隐式方式存入神经网络，查询空间中的某个点时，网络会返回诸如到最近表面的距离、颜色、透明度等物理量。对于 radar neural field，作者提到 `Borts` 等人的工作采用了基于物理的 radar 传感器模型，学习一个隐式的几何与反射率表示，并能从未见视角合成 radar measurements。此时，传感器接收功率不仅取决于已知的发射功率和天线增益，也取决于 `RCS`；而 `RCS` 本身又由目标物体的尺寸、反射率和方向性共同构成。神经表示学习到的，正是把观测到的 `RCS` 分解为一部分物体面积，以及另一部分“反射率 × 方向性”的乘积。

### 9.4.2 Radar SLAM 流水线

前面各节已经分别讨论了 radar `SLAM` 的主要部件: open-loop odometry、place recognition、map representations，以及若干与 radar 物理特性直接相关的建模问题。本节回顾若干完整 pipeline，说明这些组件在真实系统中如何组合起来。

作者首先强调，这些 graph-based radar `SLAM` frameworks 一般都遵循与图 `9.1` 类似的结构: 前端负责把原始 radar data 过滤成 point cloud，并执行 loop-closure place recognition；后端则维护 pose graph、进行全局优化，并在检测到回环后刷新地图。

`Figure 9.11` 展示了使用 `3+1D radar`（Sensrad Hugin）与 `IMU` 输入所生成的 radar `SLAM` 三维地图示例。颜色表示高度；左图为 loop closure 之前，可见地图左侧存在明显累积的水平与垂直漂移；右图为 loop closure 之后。

`TBV-SLAM`（`trust but verify`）[9] 是使用 `2D spinning radar` 的代表系统之一。它建立在 `CFEAR` `2D` radar odometry 之上，对每张 radar scan 都持续追踪位姿，但出于效率考虑，只把更稀疏的一组 keyframes 加入 `SLAM` pose graph。当累计行驶距离超过阈值，例如 `1.5 m`，系统就插入一个新的 keyframe，并基于其相对于上一个 keyframe 的对齐结果创建 odometry constraint。作者特别指出，图中约束不仅需要相对位姿，还需要 associated covariance matrix。`TBV-SLAM` 的一个经验结论是: 直接根据 registration cost function 的 Hessian 来估计协方差，并不一定优于一个小的固定对角协方差；前者有时反而会低估或高估不确定性，从而在后端优化中造成轻微 misalignment。

`TBV-SLAM` 的另一核心特点，是其“先信任再验证”的 loop closure 策略。系统先用改造版 `Scan Context` 描述子做检索，然后为每个 keyframe 额外构造多个经过 lateral translation augmentation 的描述子，以适应车辆在不同车道上经过同一地点的情形。回环候选不仅根据 descriptor similarity 评分，还会把 odometry uncertainty 传播进来，共同决定候选优先级。随后，系统对候选 pairs 做几何重配准，并通过 overlap measures 与学习式 `CorAl` [7] 对齐质量评估来验证，只保留可验证的最佳候选。这个 pipeline 很好地说明了 radar `SLAM` 的一个现实原则: place descriptor retrieval 远远不够，必须结合 odometry prior 与精细 geometric verification。

另一个著名系统是 `RadarSLAM` [474]。它同样基于 `2D spinning radar`，但 open-loop tracking 不是通过稠密 scan registration 完成，而是直接在 radargram 上提取 blob keypoints，并使用 `Lucas-Kanade` tracker [706] 做帧间跟踪。系统还利用估计出的 travel distance，对一整圈扫描内部的运动执行 constant-velocity compensation。与此不同，回环检测时它不再依赖这些稀疏 keypoints，而是从 radargram 提取更稠密的 point clouds，并为 keyframes 构造 `M2DP` descriptors [445]。为防止像 highway 这样缺乏辨识性的狭长场景造成误检，系统还会用 `PCA` 特征值筛掉过于 elongated 的 keyframes。相比 `TBV-SLAM`，`RadarSLAM` 的 loop verification 更简单，除上述筛选外基本不再做额外回环验证。

对 `2D SoC radar`，`Schuster` 等人 [982] 的方法更进一步，维护的是“pose nodes + radar feature nodes”的联合图，而不只是纯 pose graph。由于 `2D SoC radar` 产生的 detections 远少于 `360` 度 spinning radar 或 `3+1D` radar，因此把所有 detections 都保留在可优化图中在计算上仍然可行。系统使用 `BASD` [907] 提取 landmarks，并在局部区域内直接比较 compact binary descriptors，从而把新观测与地图中的已有 features 关联起来。经 `RANSAC` 剔除 outliers 后，特征和位姿一起被加入图中。该系统假设 open-loop tracking drift 适中，因此没有显式 place recognition 模块，但在发生回环后仍可用 `BASD` 匹配来优化全图。

最新的 `6-DoF` radar `SLAM` 方法，如 `4DRadarSLAM` [1270] 与 `4D iRIOM` [1309]，则直接以 `3+1D SoC radar` 输出的 `3D radar point clouds` 为输入。由于 datacube 到 point cloud 的 filtering 已在传感器端完成，pipeline 的重点转向去噪、moving object filtering 和 scan-to-submap registration。两者都先利用 per-point Doppler radial velocity 做 ego-velocity estimation，并移除不符合静态场景速度模型的 outlier points。`4D iRIOM` 还会进一步保留那些在固定半径内拥有足够多邻居、且邻域分布足够紧凑的点，以执行 denoising。之后，`4DRadarSLAM` 使用 `APDGICP` 进行带各向异性协方差加权的 scan-to-submap registration，对远距离点赋予更高不确定性；`4D iRIOM` 则使用 one-to-many distribution-to-distribution matching。

在 loop closure detection 方面，这两类 `3+1D` 系统都采用 `Scan Context` 思路，但实现细节略有不同。`4D iRIOM` 使用原始 `Scan Context`，而 `4DRadarSLAM` 使用 `Intensity Scan Context` [1150]，以避免 noisy elevation measurements 使描述子失去判别力。除此之外，`4DRadarSLAM` 还会用累计 odometry distance 做阈值验证，拒绝那些尽管描述子相似、但按里程计推断其实相距太远的候选帧。

### 9.4.3 Radar SLAM 中的多模态

到目前为止，书中讨论的大多是以 radar 为唯一外感知输入的系统，至多辅以 `IMU` 或 wheel odometry 这类本体传感。但现实中，多模态 radar 系统越来越重要，尤其是在极端环境下需要同时兼顾几何精度与环境鲁棒性时更是如此。

一种直接路线，是 radar-`LiDAR` 融合建图。系统可根据当前可见度估计在某种条件下信任哪类传感器: 天气良好时更信任 `LiDAR`，低可见度时更信任 radar。Radar 与 `LiDAR` 还可以联合提升地点识别，通过把 radar 注册到 `LiDAR` 地图中来实现跨模态检索。

另一条路线，是与视觉和热成像融合。例如，有工作在经典视觉惯性里程计 `ROVIO` 上加入 radar ego-velocity 估计，也有工作将热相机与 radar 结合构成低可见度下的稳健里程计系统。对于 `3+1D radar`，还可以利用 thermal image 与 radar 点云协同，通过 transformer 匹配连续热图像中的对应点，并用 radar 提供更稳定的深度信息。

多模态还不仅限于传感器间融合。某些系统使用外部先验，如 `OpenStreetMap` 或卫星图像来辅助 radar 定位。基于 `OpenStreetMap` 的方法，通常将建筑物墙线等结构作为先验，再把从 radargram 提取的定向点与这些线特征进行匹配。由于先验地图可能过时、不完整或带有较大不确定性，这一匹配本身相当困难。使用卫星图像的工作，则借助深度网络先从俯视图合成“可能对应的 radargram”，再分别估计旋转与平移对齐。

作者还提到 `UWB` 感知。虽然 `ultra-wideband` 也使用无线电频谱，有时甚至被称为 `UWB radar`，但其与 `mmWave radar` 的测距机理差别很大。在 `SLAM` 框架中，`UWB` 更常被用作地点签名的来源，即通过宽带回波频率响应判断“是否到过这里”，而实际的度量位姿通常仍来自 wheel odometry 或 `IMU`。因此它更像一种特殊的 place recognition 辅助模态。

## 9.5 Radar 数据集

本节简要回顾 radar `SLAM` 文献中的若干代表性数据集，表 `9.1` 也对这些数据集做了概览。整体上，它们大致可分为使用 spinning radar 的数据集，以及使用 `SoC / array radar` 的数据集；还有少量数据集同时包含两类设备。

表 `9.1` 汇总了公开 radar 相关数据集的总体情况，其中 `VO` 表示 visual odometry，`TLS` 表示测绘级 terrestrial laser scans，`RTK` 表示带实时动态修正的 `GPS`。

| 类别 | 数据集 | LiDAR | Cameras | Ground truth | Environment | Inclement Weather |
| --- | --- | --- | --- | --- | --- | --- |
| Spinning radar | Oxford Radar RobotCar [54] | Yes | Stereo/Mono | GPS/IMU + VO | Dense Urban | Rain, Fog |
| Spinning radar | Boreas [128] | Yes | Mono | GPS/IMU + RTK | Sparse Urban | Rain, Snow, Fog |
| Spinning radar | MulRan [566] | Yes | No | SLAM | Mixed Urban | - |
| Spinning radar | RADIATE [1000] | Yes | Stereo | GPS/IMU | Mixed Urban | Rain, Snow |
| Spinning radar | OORD [353] | Yes | Mono | GPS | Urban and Offroad | Snow, Night |
| SoC array radar | nuScenes [137] | Yes | Stereo | GPS/IMU | Mixed Urban and Natural | Rain |
| SoC array radar | RadarScenes [981] | No | Mono | None | Mixed Urban Roadways | Rain, Fog |
| SoC array radar | ColoRadar [606] | Yes | No | SLAM | Varying | - |
| SoC array radar | NTU4DRadLM [1272] | Yes | Mono | SLAM | Mixed Urban | - |
| SoC array radar | MSC-RAD4R [212] | Yes | Stereo | GPS + RTK | Mixed Urban | Smoke, Snow, Night |
| SoC array radar | Snail [482] | Yes | Stereo | TLS | Roadways and Tunnels | Rain, Night |
| SoC array radar | K-Radar [835] | Yes | Stereo | GPS/IMU + RTK | Roadways | Rain, Snow, Fog, Night |
| SoC array radar | TruckScene [324] | Yes | Stereo | GPS/IMU + RTK | Roadways | Rain, Snow, Fog, Night |
| Both | HeRCULES [568] | Yes | Stereo | GPS/IMU + RTK | Mixed Urban and Natural | Rain, Snow, Night |

在 spinning radar 方面，表中的数据集全部采用 Navtech `2D` radar。最早也最有影响力的两个大规模数据集是 `Oxford Radar RobotCar` [54] 和 `MulRan` [566]。`Oxford Radar RobotCar` 记录的是城市道路上的多次 traversal，总里程约 `280 km`，并覆盖多种天气条件。`MulRan` 则包含更丰富的环境类型，既有密集 urban 区域，也有更 rural 的驾驶环境，同时不同 session 之间的时间间隔更长，因此特别有利于 place recognition 研究；不过其总驾驶里程相对更少。`Boreas` [128] 进一步提供了跨一年重复采集的路线数据，总里程约 `385 km`，并包含雨雪等恶劣天气。除 `SLAM` 外，该数据集还提供了车辆、行人、自行车等目标检测 benchmark。与这些城市场景不同，`Oxford Offroad Radar Dataset (OORD)` [353] 更关注非城市环境，包含约 `154 km` 的未铺装道路与山地小径数据。

在 `SoC radar` 方面，公开数据集既包括 `2+1D` 也包括 `3+1D` 数据。某些自动驾驶数据集虽然主要面向 object detection，但也被广泛用于 radar `SLAM` 的算法开发与测试。例如:

- `nuScenes` [137] 结合了五个 Continental `ARS408-21` radars、一台 `LiDAR` 和六个 cameras，聚焦四座城市中的 urban driving，并提供 `23` 类目标检测标注。
- `RadarScenes` [981] 包含四个 `77 GHz` 汽车 `2+1D` radars 和一个 camera，主要服务于 semantic perception，提供 `11` 类对象标注，但缺少精确 ground truth，也没有 `IMU` 和 `LiDAR` 数据。

更贴近 radar `SLAM` 的数据集包括 `ColoRadar` [606]。它提供 `3+1D` Texas Instruments `MMWCAS-RF-EVM` radar、`2+1D` TI radar、`IMU` 和 `3D LiDAR` 数据，特别重要的是，它不仅给出点目标，还包含原始 `ADC` 读数与 `3D heat maps (data cubes)`。该数据集覆盖室内、室外以及地下矿井环境，并提供 `6-DoF` ground-truth tracking。`NTU4DRadLM` [1272] 与 `MSC-RAD4R` [212] 都采用高分辨率 `3+1D` `Oculii Eagle` radar；前者包含结构化校园和非结构化公园环境，后者覆盖 urban / rural 的 on-road driving。`Snail-Radar` [482] 则同时采集了 `Oculii Eagle` 与 Continental `ARS548` 两种高分辨率 `3+1D` radar，并包含手持采集和城市道路驾驶场景。

作者整体传达的意思是，radar 数据集生态已经明显扩展，但相较于视觉和 `LiDAR`，它在 ground truth 完整性、跨设备统一性和基准协议方面仍不够成熟。因此，数据集本身仍是推动 radar `SLAM` 发展的关键基础设施。

## 9.6 延伸阅读与最新趋势

作者认为，真正意义上的 `3D Radar SLAM` 仍然不多，但随着高分辨率 `SoC radar` 的发展，我们可以预期未来会出现更多面向完全 `3D` 里程计与 `3D` 地点识别的工作。这对无人机等当前较少使用 radar 的平台尤其重要。与此同时，多个小视场 radar 组合成星座式系统也可能越来越普遍，而这会连带引出外参标定和跨雷达同步等新问题。

另一个重要趋势，是更充分地利用原始 `radargram` 与 `datacube` 中的光谱信息。传统 `Radar SLAM` 往往把 radar 数据尽快压缩成点云；未来系统则更可能直接在更丰富的原始频谱上工作，从而得到更细致的地图，也提升物体检测与分类能力。相关挑战之一，是如何推动厂商向研究界开放更底层的原始数据接口。

此外，radar 语义分割可能在地点识别中发挥更重要作用。通过显式区分环境中的不同对象类型，系统可以降低 radar 回波中的歧义，从而提高地图与回环识别可靠性。最后，多模态方法仍将是 `Radar SLAM` 最关键的发展方向之一。通过把 radar 与其他传感器，乃至地理先验，如 `OpenStreetMap`，结合起来，系统有望在大尺度 outdoor 场景中获得更高定位鲁棒性。总体而言，radar 未必会在所有指标上取代视觉或 `LiDAR`，但它已经成为让 `SLAM` 在恶劣环境中继续工作的关键组成。

## 第 10 章 Event-based SLAM（事件相机 SLAM）

### 本章概览

细心的读者会注意到，`SLAM` 是一类对真实世界应用至关重要的问题，因为它关系到空间关系理解以及与环境的交互。对 `SLAM` 而言，主传感器的选择会直接决定系统的成功与适应性。Visual `SLAM` 是最普及的一大类方法，因为 cameras 普遍可得、价格可承受，并能产生直观且信息丰富的信号，让机器人在广泛场景中感知世界，例如无需额外基础设施就能构建轻量系统。

尽管现有进展已经很多，最先进的人工视觉系统在真实世界任务中的有效性（鲁棒性和效率）仍不如生物视觉系统。标准 cameras 以固定帧率感知世界，而这一帧率与场景动态无关。因此，它们会在帧与帧之间“失明”，引入延迟，可能丢失跟踪；并且当场景中没有物体运动时，还会产生大量冗余数据。本章追求的正是这样一个富有前瞻性的目标：借助受生物启发的 silicon retinas，也就是 “event cameras”，构建 fast（不受固定帧率限制）、low-power，并且对广泛照明条件都更鲁棒的 visual `SLAM` 系统，因为这类传感器克服了标准相机的若干局限（见 Fig. `10.1`）。

本章将首先介绍 event cameras 的工作原理（`10.1`），以及相关挑战和应用（`10.2`）；随后聚焦事件相机数据的处理方法（`10.3`），以及相应的前端（`10.4`）和后端（`10.5`）。最后，作者会讨论 state-of-the-art systems（`10.6`）、数据集、模拟器与 benchmarks（`10.7`），并在 `10.8` 中总结新趋势和进一步阅读。

## 10.1 传感器说明

事件相机与传统 frame camera 的根本区别，在于它不是周期性读出整幅图像，而是在单个像素的亮度发生足够变化时，立即输出对应事件。换言之，事件相机输出的不是图像序列，而是一串随时间到达的亮度变化“脉冲”。

### 10.1.1 工作原理

与传统相机依赖外部时钟、以固定频率采集整帧图像不同，像 `Dynamic Vision Sensor (DVS)` 这样的事件相机，其像素彼此独立工作，只要场景亮度发生变化，就会异步地产生响应。Figure `10.1` 展示了一个使用下视 `DAVIS` 相机执行自主飞行的无人机例子，其中事件相机的高速和高动态范围特性，被用来应对困难光照条件。造成像素级亮度变化的原因，既可能是场景照明变化，例如灯光闪烁，也可能来自相机与场景之间的相对运动，包括场景中独立运动物体带来的变化。

图 `10.1` 更完整地说，画面展示的是一架搭载下视 `DAVIS` 相机 [111]（`240×180 px`）的无人机，使用 visual-inertial odometry (`VIO`) 算法 [943] 执行自主飞行。插图左侧是在灰度帧中检测并跟踪的特征点，受到明显 motion blur 影响；中间则是对 warped events 做 motion compensation 后得到的更清晰图像；红 / 蓝事件数据则清楚响应场景轮廓。相同的 `VIO` 算法 [943] 也被展示在更高速的场景中，例如把事件相机绑在绳子上高速旋转。图像引自 [361]（©2020 IEEE）。

因此，事件相机的输出不是规则采样的图像帧，而是一串数字“事件（events）”或“脉冲（spikes）”。每个事件都表示某个像素处对数亮度发生了一次变化。这种编码方式本身就受到生物视觉通路中 spike 机制的启发。

更具体地说，每个像素都会在自己上一次触发事件时记住对应的对数亮度 `L`，随后持续监视新的亮度是否相对于这个记忆值产生了足够大的变化 `ΔL`。当该变化达到阈值 `C` 时，就会发送一个事件:

`ΔL = L(x_k, t_k) - L(x_k, t_k - Δt_k) = p_k C`             `(10.1)`

其中，事件本身写成 `e_k = (x_k, t_k, p_k)`，并从芯片中输出。这里 `x_k` 是像素在图像平面中的 `x, y` 位置，`t_k` 是时间戳，`p_k ∈ {+1, -1}` 是 1 bit 极性，表示该变化是亮度增加还是亮度降低，而 `Δt_k` 则表示同一像素距离前一次事件经过的时间。

事件相机是一类典型的数据驱动传感器: 它的输出取决于场景中运动或光照变化的强弱。运动越快，单位时间内产生的事件越多，因为每个像素都会根据自身监测到的亮度变化速度自适应地调整采样频率。Figure `10.2` 中作者用一个旋转圆盘上的黑点做例子，对比了标准相机和事件相机的输出。标准相机只在帧采样时刻输出图像，而事件相机会把亮度变化连续地编码为时空中的螺旋事件轨迹，其中红色表示正事件 `ON spikes`，蓝色表示负事件 `OFF spikes`。

图 `10.2` 从三个角度解释事件相机的工作原理: `(a)` 人眼视网膜的三层模型，以及与之对应的 `DVS` 像素电路；`(b)` `DVS` 像素把光信号转换为事件（spikes）的工作示意图，其中信号颜色与 `(a)` 中各层一一对应；`(c)` 标准相机与事件相机面对“旋转圆盘上的黑点”这一视觉刺激时的响应对比。事件相机会连续发送亮度变化，在时空中形成螺旋状事件轨迹；红色表示正事件（`ON spikes`），蓝色表示负事件（`OFF spikes`）。图像改编自 [883]（©2014 IEEE）。

作者随后解释了其生物启发来源，也就是视觉系统中的 `transient pathway`。按照双通路假说，背侧通路，也被称为 `transient` 或 `where` pathway，主要处理动态视觉信息，例如场景运动；而腹侧通路，也称 `sustained` 或 `what` pathway，则更偏向物体识别与视觉辨认。`DVS` 对应的正是从 photoreceptor 到 ganglion cell 的那部分 transient pathway，只不过它采用了经过简化的三层像素设计，在生物保真度与电路稳定性之间取得折中。三层结构分别实现光信号转换、delta-modulation 与比较功能。

还有一些相机，例如 `Asynchronous time-based image sensor (ATIS)` 或 `Dynamic and Active-Pixel Vision Sensor (DAVIS)`，则试图同时模拟两条视觉通路，因此会同时输出两类信号: `DVS events` 以及灰度信息，例如普通图像帧。关于不同事件相机类型的更多细节，作者引导读者参考文献 `[883, 361]`。

### 10.1.2 事件相机的优势

事件相机的感知原理与主流曝光式相机完全不同，因此也带来了一组非常鲜明的优势。

首先是极高的时间分辨率。事件通常以微秒级时间精度被触发和时间戳化，因此可以观测非常快速的运动，而不会像帧式相机那样因帧间空白期过长而“失明”，也不会在高速运动下产生严重 `motion blur`。第二个优势是低延迟。由于每个像素都独立工作，事件一旦被触发便会以亚毫秒级延迟输出，不必等待整帧曝光结束。

第三个优势是低功耗和低带宽。事件只编码真正发生变化的部分，因此在大量静态区域中不会产生冗余信息。这意味着功耗与数据带宽都可显著低于以相同时间精度工作的传统相机。第四个优势则是极高动态范围 `HDR`。事件相机通常能覆盖超过 `120dB` 的亮度范围，而传统相机常见只有约 `60dB`。这使事件相机可以同时感知非常暗与非常亮的区域，而不过曝或欠曝。

这些特性让事件相机在高速机器人、强明暗对比环境、快速飞行器、AR/VR inside-out tracking 等场景中特别有吸引力。

### 10.1.3 当前设备与趋势

对于刚进入这一方向的研究者，一个常见问题是: 到底该买哪种 event camera，才能解决自己的 `SLAM` 问题？作者指出，尽管事件相机的设计已经出现许多变体，但真正商业化的型号并不多。目前主要厂商包括 `SONY`、`Samsung`、`iniVation / SynSense`、`Prophesee`、`Omnivision` 等。

近年来最明显的趋势之一，是像素尺寸不断缩小，从早期 `DVS128` 的约 `40µm` 降到不足 `5µm`。但事件像素所需电路比传统像素复杂得多，因此要在有限面积内继续缩小像素，本身就是很有挑战的芯片工艺问题。为此，业界逐渐采用 stacked technology 和 backside illumination 等设计，以提升 fill factor 并减小光敏区域间隙。

另一个趋势，是部分设备同时输出事件与灰度图像。早期的 `DAVIS` 或 `ATIS` 这类器件，就同时建模了类似生物视觉中的 transient pathway 和 sustained pathway，因此既输出 `DVS` 事件，也输出灰度图像。较新的高清 event camera 有些则取消灰度输出，将更多面积让给事件阵列本身。也有少量设备支持 RGB 颜色通道的事件输出，不过颜色在许多运动感知任务中并非必要。

同时，越来越多的设备开始集成 `IMU`。这对 `SLAM` 很关键，因为 `IMU` 是视觉的天然补充，能显著提升 `VIO` 和 `SLAM` 的鲁棒性与精度。

作者特别提醒，高空间分辨率的 event camera 并不一定天然优于低分辨率型号。对于 `SLAM` 来说，高分辨率带来的往往不是“白捡的精度”，而是更高带宽、更大处理负载，以及动辄每秒上亿事件的实时处理难题。目前还没有一种通用的算法-硬件组合，能够在不做聚合、降采样或数组化转换的前提下，稳定实时处理如此高的事件率。因此，对计算资源受限的机器人平台而言，较低分辨率，例如 `QVGA` 级别，往往反而更适合算法原型验证和实时运行。未来值得关注的方向包括 hybrid sensors、foveated sensors 以及更紧密结合 `IMU`、frame camera、`LiDAR` 的多模态设备。

## 10.2 挑战与应用

事件相机是一项革命性的视觉采集技术，但它也迫使研究者重新思考视觉算法和硬件设计。最核心的挑战有三类。

第一，事件输出是时空稀疏且异步的，而标准图像是时空稠密且同步的。这意味着绝大多数基于图像序列的经典 `SLAM` 方法无法直接搬过来。第二，事件是运动相关的观测。与普通图像不同，每个事件只携带亮度增加或减少这一位信息，而这一变化既由场景纹理决定，也由相机与场景之间的相对运动决定。第三，事件相机并不“无噪声”。它会受到光子散粒噪声、电路噪声、照度依赖、阈值不一致、低功耗工作模式带来的非理想性等影响。

这些问题共同推动了 event-based `SLAM` 的发展。它不只是一个“把图像换成事件”的问题，而是要求我们重新思考: 应该如何从事件里提取位姿和深度信息？该使用什么地图和轨迹表示，才能反映事件的准连续时间特性？如何在运动依赖的观测下建立数据关联？又如何避免把传统帧式系统中的时间量化、延迟和冗余带入新的系统？

作者还指出，event-based `SLAM` 与其他 event 视觉任务之间一直存在强耦合关系。例如，事件图像重建、光流估计、跟踪与分割等研究，都反过来推动了 `SLAM` 的发展；很多早期系统就是在旋转 `SLAM`、六自由度 `SLAM`、光流和图像重建共同作用下诞生的。

## 10.3 Event-based SLAM 方法概览与分类

作者首先按“同时处理多少事件”来给 event-based `SLAM` 方法做最粗粒度分类。第一类是 `event-by-event` 方法，也就是每到来一个单独事件，系统状态，例如 scene map 或 camera trajectory，就可能更新一次。这种方法理论上可达到最低延迟。第二类则是按 groups / batches / slices / packets 处理事件的方法，它们会引入一定 latency，但往往更便于实现和优化。在后一类方法里，一个非常关键的设计选择就是 packet size 如何设定，常见方案包括固定事件数、固定时间长度，以及两者结合的 hybrid criteria。

与这一维度正交，作者又区分了 `model-based` 和 `data-driven` approaches。前者更多依赖对事件成像机理和几何关系的显式建模；后者则把机器学习模型引入事件处理流程。再模仿 frame-based `SLAM` 的习惯，event-based 方法还可分为 `indirect` 与 `direct` 两类。`Indirect` 方法通常先提取 event corners、lines、normal flow 等特征，再沿用更传统的几何 `SLAM` pipeline；`direct` 方法则更倾向于把 event data 直接映射为 motion 和 scene parameters。

这一划分背后，还对应着不同的 objective / loss design。`Indirect` 方法往往分成两个步骤: 先把事件“转化”为几何原语，再做几何 `SLAM`；而 `direct` 方法通常试图用一步完成从 event data 到运动和场景参数的映射，它们的目标函数更多体现为 photometric- 或 event-rate-based alignment。在这类方法中，事件生成模型 `(10.1)` 及其线性化形式 [361] 是估计方法设计的核心基础。

`Figure 10.3` 说明 event-based `SLAM` 正在被积极探索，系统覆盖了从经典方法到更近年的 deep learning solutions 的大量路线。由于事件由图像平面上的移动边缘触发，因此把场景地图恢复为边缘形式，例如稀疏或半稠密 `3D` edge maps，是很自然的。示例图改编自 `EVO` [913]、`EDS` [461]、`CMax-SLAM` [416]、Kim 等人 [570]、`ESVO2` [811]、`DEVO` [584] 和 Wang 等人 [1153]。

作者特别强调，`data association` 是 event-based vision，尤其是 `SLAM` 中的中心问题。由于事件相机具有极高的时间分辨率，事件之间的关联通常借助时空邻近性来完成；相应地，文献中既有 hard-association，也有 soft-association 策略。

书中进一步指出，上述各种分类各有利弊。用事件相机做 `SLAM` 的难点很高，因此研究历史上实际上是沿着多条复杂度轴逐步推进的: 未知量自由度从低到高，运动类型从纯旋转或 `2D` 场景扩展到完整 `6-DoF`，场景复杂度从简单纹理走向复杂静态和动态场景，任务也从孤立的 `SLAM` 问题不断与 optical flow、tracking、segmentation 等其他 event vision 任务交织在一起。

作者最后指出，从时间线上看，早期 event-based `SLAM` 更偏向 model-based methods，而较新的工作则越来越多地尝试利用 deep-learning–based approaches 的潜力。

## 10.4 Event-based SLAM 系统的前端

Event-based `SLAM` 前端往往包含若干相互协作的子模块，例如特征提取、数据关联、初始化、位姿跟踪、深度估计等。从输入输出角度看，前端接收原始事件流以及可能的辅助信息，如相机标定参数，并输出事件相机位姿序列和场景地图的初步估计。随后，后端在较低频率下对这些变量进行精化，并可将结果反馈给前端，帮助减少漂移。

`Figure 10.4` 概括了一个 event-based `SLAM` pipeline：前端负责计算地图和相机位姿，后端负责进一步精化地图与位姿。由于事件响应的是移动边缘，恢复出的地图通常也是 edge / gradient map。图中的示例是一个 direct rotational `SLAM` pipeline，其 pose 仅由 rotations 构成，而地图退化为 panoramic map [417]；绝对强度图还可通过 `Poisson integration` 恢复。

由于事件是由图像边缘运动触发的，因此在恒定光照假设下，移动的事件相机本质上相当于一个异步边缘探测器。这意味着很多 event-based `SLAM` 系统最终恢复的地图，天然更接近边缘图、梯度图或半稠密结构，而不是传统意义上的全纹理地图。

### 10.4.1 预处理与事件表示

在 `SLAM` 场景中，事件相机会在运动过程中持续输出事件。由于事件稀疏且具有微秒级时间分辨率，理论上每个事件都对应一个不同的相机位姿。这与传统 frame camera 的范式完全不同: 在普通图像中，同一帧内所有像素共享同一个时间戳，因此也共享同一个相机位姿，而这正是经典 multi-view geometry [437] 的基本前提。

正因如此，许多 event-based `SLAM` 方法都会先把原始事件流转换成某种替代表示，例如 event images、time maps / `time surfaces`、voxel grids 等 [361]。这样做的原因很多，包括:

- 与 conventional computer vision methods 兼容。
- 更易于人理解和调试。
- 更方便构造基于图像的 learning pipelines。

但作者同时指出，这一转换过程往往伴随着信息量化，例如把时间相近的事件分组，或者在没有事件的像素位置显式填零。前者会损失原生时间分辨率，后者会损失事件数据原本的 sparsity。

因此，`event representations` 本身已成为一个独立且重要的研究主题 [361, 370]。它通常位于前端的第一步，并会深刻影响后续所有处理阶段: 事件先被转成更熟悉的表示，再喂给传统图像 `SLAM` 方法，或喂给基于图像的深度网络。作者指出，这在很大程度上反映了社区仍在探索“到底如何从事件流中提取最有用信息”，因此会自然地尝试复用成熟的 frame-based methods。

不同系统甚至会在前端内部为不同模块选择不同表示。例如 `EVO` [913] 在其 mapping module 中使用 raw events 以及 `EMVS` [915]，而在 camera tracking module 中则使用 event images。理想情况下，我们希望设计出既保留 event camera 高速、稀疏等核心优势，又不会退化成传统相机问题的事件表示和 `SLAM` 方法；但作者坦言，这仍然是一个新兴研究方向，尤其需要从根本上重新思考异步视觉处理。

### 10.4.2 间接法

`Indirect methods` 一般包含两个步骤。首先，它们从事件中提取和跟踪某类几何原语，例如角点、线段或轨迹；然后使用与传统视觉 `VO` / `SLAM` 类似的几何流程，来估计相机运动和场景结构。

这种路线的优点，是能把海量事件压缩成少量高信息密度的几何原语，从而更高效地利用计算资源，并复用成熟的多视图几何工具箱。通常，位姿跟踪可被表述为一个 feature alignment 问题，通过最小化重投影误差等几何目标函数，在给定地图的条件下恢复相机姿态；而场景结构则可通过对应特征的三角化或反投影获得。

但这条路线也高度依赖稳健的特征提取与跟踪，而这恰恰是事件视觉里仍未完全成熟的一环。事件稀疏、噪声大、运动依赖强，因此即便是“检测角点”这样在帧相机中已经很成熟的问题，在事件流中也更难处理。为缓解这些问题，一些系统会将事件与灰度图像或 `IMU` 融合。

### 10.4.3 直接法

`Direct methods` 不显式提取几何特征，而是尽可能直接利用所有事件来估计运动和场景结构。若事件率过高超出处理能力，系统通常会引入去噪、稀疏采样或其他数据约简机制。

与 indirect 方法相比，direct 方法更强调事件生成模型、事件极性和时空一致性。它们通常通过优化某种目标函数来完成位姿或场景估计，例如 photometric error，或更常见的 event alignment / motion compensation 目标。其中一类最有代表性的方法是 `contrast maximization (CMax)`。其核心思想是“通过估计运动来撤销运动”，也就是寻找那组能把事件正确 warp 到共同参考系上的参数，使得 `Image of Warped Events (IWE)` 最清晰、最聚焦。相关目标可以用方差、梯度、离散度等形式表达。

Direct 方法更充分利用了事件的准连续时间特性，因此在高速场景中常表现出更大潜力；但它们对初始化、噪声建模以及事件相机物理模型的准确性要求也更高。有些问题里，若事件被错误地 warp 到少量像素或单条线附近，还可能产生不希望的全局最优点。

### 10.4.4 基于模型与基于学习的方法

目前绝大多数 event-based `SLAM` 仍是 hand-crafted 的 model-based 系统，即依赖人工设计的物理和几何先验。但近年深度学习已开始快速进入这一领域。一类方法先把事件转换为图像式表示，再用常规神经网络处理；另一类方法则直接使用 `Spiking Neural Networks (SNNs)` 等结构处理事件流。

学习方法又可分为 supervised 与 self-supervised。Self-supervised 方法通常利用事件自身或与其共视场的灰度图像，通过时间一致性或光度一致性损失来训练深度和位姿网络；supervised 方法则直接依赖 ground truth 监督。近年大量车载、无人机和多模态数据集的出现，为这类方法提供了必要训练数据。

学习方法可以替代某一部分 `SLAM` 管线，如特征提取和跟踪，也可能试图做 end-to-end 替代。但它们往往需要大量训练数据，并容易受到 domain shift 影响。作者整体上的态度是: learning 确实很有前景，但 event camera 的物理成像机制非常强，因此真正稳健的方法往往需要 model-aware 设计，而不是纯黑盒。

## 10.5 Event-based SLAM 系统的后端

`SLAM` 后端的职责，是提升前端输出变量与传感器数据之间的一致性，减少误差传播，并增强系统的精度和鲁棒性。与传统视觉 `SLAM` 类似，`bundle adjustment (BA)` 仍是最重要的候选工具之一。

不过，作者指出，event-based `BA` 仍处于相当早期的阶段。许多系统事实上根本没有严格意义上的 refinement back-end，而是采用 tracking-and-mapping 并行结构，让跟踪与建图模块相互提供输入。事件相机最大的优势之一是低延迟，因此很多系统历史上更优先追求简单和快速，而不是引入昂贵的全局优化。

另一方面，event-based 后端之所以困难，还在于它要联合优化大量彼此相关的变量，而事件本身又噪声大、与运动耦合强，导致优化问题高维、代价大，也更容易陷入局部极值。书中据此前端输出类型，将后端也分为 indirect 与 direct 两类。

Indirect back-end 基本继承自传统特征式视觉 `SLAM`。它对事件流中的角点、线段等几何原语做优化，目标一般是最小化重投影误差。这条路线的优点，是可直接复用传统 `SLAM` 中成熟且鲁棒的工具；但它也丢掉了事件中大量无法被少量几何特征压缩掉的信息，而且当前事件角点的稳定性通常仍不如帧式角点。

Direct back-end 则直接面向传感器数据本身，而非几何原语。若系统还配有灰度图像，那么后端往往直接复用帧式视觉系统的 photometric `BA`；而在 event-only 场景中，直接后端可以围绕 motion compensation、`CMax` 或基于事件生成模型的光度误差构造目标函数。由于每个事件信息量都很小，而 `SLAM` 状态变量又很多，这类方法往往需要非常多事件才能支撑精确优化，因此在计算资源、功耗和延迟之间的平衡仍是开放问题。

## 10.6 最先进系统

Table `10.1` 汇总了 event-based `VO/SLAM` 的具体系统，并按照前文引入的分类，总结了它们的一些特征（direct、indirect 等）。虽然本章不可能逐一详细描述所有系统，但原书指出了几条值得注意的趋势。

首先，当前文献仍由 model-based systems 主导；data-driven approaches 还没有完全接管这一领域，尽管未来很可能像其他 computer vision 任务一样发生转变。其次，自这一方向诞生以来，事件相机 `SLAM` 一直是在不同假设下逐步提升复杂度，包括 `(i)` 相机运动类型、`(ii)` 场景类型，以及 `(iii)` 用来简化问题的附加传感器或先验信息。例如，给 event camera 加一个 depth sensor 会减轻仅靠 events 做 depth estimation 的负担，而 `IMU` 则可提供精确角速度信息。

一旦某个 event-based 方法表现良好，它往往会沿着一种近乎标准化的“exploitation roadmap”逐步扩展，类似于 frame-based `SLAM`：例如，单目方法 [913] 可以扩展到 stereo 或 multi-camera 场景 [381]；event-only 方法如 [1301]（或 [584]）则可通过与惯性数据融合 [684, 811]（或 [408]）来增强鲁棒性；基础系统还可以继续扩展到 omnidirectional lenses 等设定。

不过，event-based `SLAM` 仍然是一个 emerging field，因此仍处于强烈的 exploration 阶段。从 Table `10.1` 就能明显看出这一点：系统背后的思想和原理非常多样，继而导向了不同的地图表示、事件表示、损失函数等。围绕如何利用传感器的原生特性来设计新的状态估计方法，仍有大量探索空间。

| System | M/DL | I/D | Event representation | BA | Motion | Scene | Input | Remarks |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Cook [229] | M | D | Event Frame | ✗ | Rot | Natural | E | Interacting network using optical flow |
| Weikersdorfer [1174] | M | I | Individual Event | ✗ | Planar | 2D B&W | E | First filter-based Ev-SLAM |
| PF-SMT [569] | M | D | Individual Event | ✗ | Rot | Natural | E | Two interleaved Bayesian filters |
| Censi [160] | M | D | Event Packet | ✗ | 6DoF | B&W | E+F+D | Filter-based VO based on image gradient |
| EB-SLAM-3D [1175] | M | D | Individual Event | ✗ | 6DoF | Natural | E+D | Augment events with depth sensor |
| Yuan [1259] | M | I | Event Frame | ✗ | 6DoF | B&W | E+I+M | Vertical line-based camera tracking |
| Kueng [610] | M | I | Local Point Set | ✗ | 6DoF | Natural | E+F | Event-based feature tracking VO |
| ETAM [570] | M | D | Individual Events | ✗ | 6DoF | Natural | E | Three interleaved filters |
| CMax-ω [356] | M | D | Individual Events | ✗ | Rot | Natural | E | Contrast Maximization |
| EVO [913] | M | D | Edge Map | ✗ | 6DoF | Natural | E | Event-event geometric alignment |
| EVIO [1302] | M | I | Point sets | ✗ | 6DoF | Natural | E+I | Filter-based and MC features |
| Rebecq [914] | M | I | MC Event Images | ✓ | 6DoF | Natural | E+I | Feature-based, sliding-window back-end |
| RTPT [924] | M | D | Individual Events | ✗ | Rot | Natural | E | Panoramic tracker and mapper |
| Gallego [358] | M | D | Individual Events | ✗ | 6DoF | Natural | E+M | Resilient sensor model |
| Mueggler [778] | M | D | Individual Events | ✓ | 6DoF | Natural | E+I+M | Continuous-time pose estimator |
| USLAM [943] | M | I | MC Event Images | ✓ | 6DoF | Natural | E+F+I | Sensor fusion & sliding-window back-end |
| Chin [204] | M | I | Event Frames | ✓ | Rot | Stars | E | Tailored to star tracking |
| ESVO [1301] | M | D | Timesurfaces (TS) | ✗ | 6DoF | Natural | 2E | Stereo matching on TS patches |
| Hadviger [425] | M | I | Corners on TS | ✗ | 6DoF | Natural | 2E | Cross-corr. feature descriptors |
| CMax-GAE [571] | M | D | Individual Events | ✗ | Rot | Natural | E | Contrast maximization |
| EKLT-VIO [724] | M | I | Individual events | ✓ | 6DoF | Natural | E+F+I | EKLT tracker and VIO back-end |
| EDS [461] | M | D | Event images | ✓ | 6DoF | Natural | E+F | Frame-based back-end (DSO) |
| CB-VIO [683] | M | I | Individual events | ✓ | 6DoF | Natural | E+F+I | Feature tracker and VIO back-end |
| Wang [1153] | M | I | Binary images | ✓ | 6DoF | Natural | 2E | Feature matching |
| El Moudni [303] | M | D | TimeSurfaces TS | ✗ | 6DoF | Natural | 2E | Use ESVO tracker and EMVS mapper |
| ESVIO [183] | M | I | Time surfaces (TS) | ✓ | 6DoF | Natural | 2E+2F+I | Feature tracking from TS |
| ESVIO-direct [684] | M | D | Timesurfaces (TS) | ✓ | 6DoF | Natural | 2E+I | Extension of ESVO |
| PL-EVIO [410] | M | I | Time surfaces (TS) | ✓ | 6DoF | Natural | E+F+I | Point & line features, sliding-window BA |
| CMax-SLAM [416] | M | D | Individual Events | ✓ | Rot | Natural | E | Contrast Maximization refines motion |
| EVI-SAM [409] | M | D,I | Individual Events | ✓ | 6DoF | Natural | E+F+I | Dense mapping |
| Zuo [1316] | M | D | Individual Event | ✗ | 6DoF | Natural | E+D | Augment events with depth sensor |
| DEVO [584] | DL | I | Event voxel grids | ✓ | 6DoF | Natural | E | Event-version of DPVO [1083] |
| EMBA [417] | M | D | Individual Events | ✓ | Rot | Natural | E | Refines motion and gradient map |
| EPBA [418] | M | D | Individual Events | ✓ | Rot | Natural | E | Refines motion and intensity map |
| ES-PTAM [381] | M | D | Events (also as frames) | ✗ | 6DoF | Natural | 2E | Use EVO tracker and EMVS mapper |
| ESVO2 [811] | M | D | Timesurfaces (TS) | ✓ | 6DoF | Natural | 2E+I | Extension of ESVO |
| DEIO [408] | DL | I | Event voxel grids | ✓ | 6DoF | Natural | E+I | Extension of DEVO and DPVO |

表 `10.1` 按时间顺序汇总了 `Event-based Visual SLAM` 方法。各列依次表示: 方法类型（`Model-based` 或 `Deep-Learning-based`、`Direct` 或 `Indirect`）、是否包含全局 refinement 模块（即 back-end / `BA`）、可处理的相机运动类型（`Rotational`、`Planar`、`6-DoF`）与场景类型（高对比黑白等），以及输入数据类型（事件相机 `E`、frame-based camera `F`、depth sensor `D`、`IMU`、Map `M`），其中 `2E` 表示 stereo events（两台事件相机）。

## 10.7 数据集、模拟器与基准

事件相机研究近年的快速增长，离不开数据集和模拟器的持续丰富。由于事件方法往往对精确时间同步、高质量 ground truth 位姿、深度和多传感器标定有很高要求，因此好的 benchmark 对这一领域尤为关键。

### 10.7.1 模拟器

公开可用的 event camera 模拟器已经相当丰富，它们的共同目标，是生成高质量的合成事件数据。`ESIM` 是其中最有代表性的工具之一。它是在更早期模拟器基础上的发展版，而更早的某些方法，其实只是简单地对两帧图像做差分阈值化，从而得到看起来像事件输出的边缘图。`ESIM` 则真正开始模拟事件相机的工作原理，并把渲染引擎与事件模拟器紧密耦合起来，使后者能够根据视觉信号的动态特性自适应地调整渲染频率。

`CARLA` 中的 event camera simulator 又把 `ESIM` 的思路扩展到了更丰富、更复杂的自动驾驶场景。在某些基于事件学习单目深度的工作里，`CARLA` 会从模拟器渲染图像出发，逐像素计算亮度变化，按与 `ESIM` 类似的方式合成事件。Figure `10.5` 就展示了 `CARLA` 中生成的 `RGB` 图像与对应事件。

随着 learning-based 方法越来越依赖海量训练数据，而真实事件数据又由于传感器新颖性而十分稀缺，研究者又开发了 `Video to Events (Vid2E)`。它的目标是把任何已有的普通视频转换成合成事件数据，从而缩小传统视觉数据集和事件视觉数据集之间的鸿沟，使现有视频数据也能被用来训练事件网络。`Vid2E` 的做法是把 `ESIM` 与自适应视频插帧结合起来。因为 `ESIM` 本身可以在任意时间分辨率下生成事件，但普通视频通常只提供低帧率、固定时间间隔的亮度观测，所以必须先用 `Super SloMo` 等插帧方法恢复出更高时间分辨率的中间帧，再交给 `ESIM`。这里中间帧数量的选择非常关键: 如果太少，会导致亮度信号混叠；如果太多，又会带来很大计算负担。

作者特别强调，event camera 模拟器最关键的难点之一，是如何准确建模噪声，以减小 `sim-to-real gap`。例如 `ESIM` 及其衍生系统，采用了一个基于经验观察的简单噪声模型: 事件阈值 `C` 不是固定值，而是服从高斯分布。也就是说，在仿真每一步中，`ESIM` 都会从 `N(C, σ_C²)` 中采样阈值，并允许正、负对比度阈值 `C+` 与 `C-` 分别设置，以更接近真实相机。同时，诸如空间和时间上的阈值波动、电子噪声、像素有限带宽等效应，也都值得被纳入模拟器。

`Vid2E` 对光照采用的是理想事件相机模型。相比之下，`V2E` 基于 `DVS` 电路进一步提出了更真实的噪声模拟器，是首个把 temporal noise、leak events 和与亮度相关的有限带宽都纳入的事件模拟器，同时也保留了与 `Vid2E` 相同的高斯阈值分布。像 `Vid2E` 一样，`V2E` 也使用 `Super SloMo` 来提升输入视频的时间分辨率。Figure `10.5` 对 `V2E` 的处理架构做了详细展示。

`Figure 10.5` 展示了两类 event camera simulators：`(a)` 在 `CARLA` 中使用 `ESIM` 生成的 `RGB` 图像与对应事件 [279]；`(b)` `V2E` 工具 [481] 的详细数据处理流程。

此外，还有更偏 learning-based 的模拟器。`EventGAN` 提出一种端到端深度学习方法，利用已有图像标注数据，从时间相邻的两帧图像出发，通过 `U-Net` 编码器-解码器网络生成事件。它并不直接输出单个事件集合，而是为每个 polarity 生成一个三维时空 voxel grid，这也是许多人工神经网络常用的输入表示。`VISTA 2.0` 则把 `RGB`、`LiDAR` 和 event camera 等多传感器模拟统一进一个自动驾驶策略学习框架中，并利用高保真真实世界数据构造不同天气、光照和道路条件，以提升 sim-to-real transfer 能力。

最后，`Video to Continuous Events (V2CE)` 关注的是时间戳真实性问题。`Vid2E` 与 `V2E` 生成的事件往往仍然发生在离散时间点上，而真实事件是连续时间到达的。对于那些对时间戳分布非常敏感的任务来说，这种差异会显著放大 domain shift。`V2CE` 因此提出一个两阶段流程: 第一阶段利用监督式 `3D U-Net` 预测两个 voxel grids，分别对应正、负极性；第二阶段则再从这些 voxel grid 中恢复更精确的事件时间戳。作者还指出，`V2CE` 在强光饱和区域以及“理想事件生成模型不再成立”的边缘区域上，也能较准确地产生事件。

### 10.7.2 数据集与基准

面向 `Visual Odometry` 与 `SLAM` 的 event-based 数据集，在 `ECDS` 发布之后显著增加。`ECDS` 是首个同时提供同步事件、`IMU` 和 `6-DoF` ground-truth 相机位姿的数据集。在它之前，一些更早的数据集只包含纯旋转运动、简单高对比场景，或者 ground truth 只能依赖 `IMU`、云台编码器或轮式里程计，因此本身容易漂移。`ECDS` 则包含手持 `6-DoF` 运动、慢速与高速两类轨迹、多种场景，并通过 motion-capture system 提供高精度真值。它包含 `11` 个真实事件序列，以及 `2` 个由早期 `ESIM` 版本生成的合成序列。

`Figure 10.6` 给出了若干 Event-SLAM datasets 的细节：`(a)` `DSEC` [372] 数据集中安装在车辆顶部的传感器套件；`(b)` `EDS` [461] 数据集中用于让多传感器共享空间对齐视场的 beamsplitter 结构。

表 `10.2` 对 event-based `SLAM` 数据集做了按时间排序的总览，传感器记号与表 `10.1` 保持一致；关于 stereo 与多传感器数据集的进一步说明，可参考综述 [380]:

| Dataset | Platforms | Pixel Resolution | Sensors |
| --- | --- | --- | --- |
| ECDS [777] | Hand-held | 240×180 | E, F, I |
| RPG-stereo [1300] | Hand-held | 240×180 | 2E |
| MVSEC [1303] | Hand-held, Drone, Car, Bike | 346×240 | 2E, 2F, I, Lidar, GPS |
| UZH-FPV [257] | Drone | 346×260 | E, F, I |
| EV-IMO [768] | Hand-held | 346×260 | E, F, I, Depth |
| EV-IMO2 [126] | Hand-held | 640×480 | 3E, F, I, Depth |
| DSEC [372] | Car | 640×480 | 2E, 2F, Lidar, GPS |
| TUM-VIE [583] | Hand-held | 1280×720 | 2E, 2F, I |
| EDS [461] | Hand-held | 640×480 | E, F(RGB), I |
| VECtor [363] | Hand-held | 640×480 | 2E, 2F, RGB-D, I, Lidar |
| M2DGR [1249] | Ground Robot | 640×480 | E, F, I, Lidar, GPS, Thermal |
| ViViD++ [634] | Hand-held, Car | 240×180, 640×480 | E, F, RGB-D, Thermal, Lidar, GPS |
| FusionPortable [522] | Hand-held, Quadruped Robot | 346×240 | 2E, 2F, I, Lidar, GPS |
| Stereo HKU-VIO [183] | Hand-held | 346×260 | 2E, 2F, I |
| M3DE [163] | Drone, Car, Quadruped Robot | 1280×720 | 2E, 2F, I, Lidar, GPS |
| CoSEC [860] | Car | 1280×720 | 2E, 2F, I, Lidar, GPS |

`RPG stereo dataset` 则由 `8` 条手持 stereo `DAVIS` 办公室序列和一个合成序列组成。虽然它不提供 ground-truth depth，但具有基于 motion capture 的准确位姿真值，因此非常适合做 event stereo `SLAM` 的原型验证与算法评估。

`MVSEC` 是第一个在多平台上系统性提供 ground-truth depth 的事件数据集。它覆盖手持平台、六旋翼、汽车和摩托车，同时配备 `3D LiDAR`、`IMU` 和标准帧相机，采集场景既有室内也有室外，并涵盖多种光照与速度条件。由于其序列较长、真值较全，因此被广泛用于位姿估计、建图、避障和 `3D reconstruction`。

`UZH-FPV` 主要面向自主无人机竞速。它使用一台搭载 `mDAVIS346` 的定制四旋翼，在室内外高速轨迹下采集数据，因此成为高动态 `VIO` 与高速事件视觉算法的重要 benchmark，也被用于多次会议与 workshop 竞赛。

`EV-IMO` 是首个专门用于 independently moving objects 分割的事件数据集，重点关注室内环境中的独立运动物体。它提供逐像素运动掩码、ground-truth egomotion 和 depth。`EV-IMO2` 则在其基础上增加了更多序列、更高质量的相机以及更复杂的场景，因此既是更有挑战性的 benchmark，也能作为训练更强 monocular / stereo event `SLAM` 方法的数据来源。

面向自动驾驶的代表数据集是 `DSEC`。它采用一套多相机平台，包含两台 `VGA` 分辨率事件相机、两台 `RGB` 相机、`Velodyne VLP-16 LiDAR` 和 `RTK GPS`。数据在瑞士的城市与乡村道路、白天、夜间和直射阳光等多种条件下采集，不仅提供 stereo matching 所需的 ground-truth depth，还提供 `Optical Flow` 和 `Disparity` benchmark。相关评测会使用 `N-pixel disparity error`、`MAE` 和 `RMSE` 等指标，衡量结合高分辨率事件数据与 `RGB` 帧的算法性能。

`TUM-VIE` 则采用百万像素级 `Prophesee Gen4` stereo event cameras，并同步采集 `200 Hz IMU` 与 `20 Hz` stereo grayscale frames。它包含手持与头戴两种安装方式，场景覆盖室内外、多类运动、体育活动、`HDR` 与低照度条件，目标是推动 `VIO`、`SLAM`、`3D reconstruction` 与多传感器融合在高分辨率事件感知上的发展。

`EDS` 使用定制 `beamsplitter` 设备，使 `RGB` 图像与事件能够共享同一光轴，这在早期数据集中非常少见。它包含高质量事件、彩色图像与 `IMU`，主要服务于 monocular `VIO`、optical flow、depth estimation 以及复杂运动和光照条件下的稳健视觉里程计研究。

`VECtor` 则是一个面向多模态 event `SLAM` 的综合 benchmark。其平台集成了 stereo 事件 / 帧相机、`RGB-D` 传感器、`128` 线 `LiDAR` 和九轴 `IMU`，覆盖小尺度室内环境与更大尺度、复杂照明条件的室内环境，强调 static / dynamic、low-light 与 `HDR` 等多种使用条件下对 `SLAM` 算法的可靠评估。

作者还提到了一批更近的多传感器数据集。`ViViD++` 使用包含 thermal camera 在内的多传感器平台，目标是推动能处理 poor visibility、运动扰动和外观变化的 `SLAM`。`FusionPortable` 则包含一个 quadruped robot，在走廊、餐厅、道路和花园等不同照明环境中采集数据。最后，`M3ED`，也常被称为 `MVSEC 2.0`，聚焦于机器人中的高速动态运动，组合了一百万像素 stereo event cameras、灰度与彩色相机、`64-beam LiDAR`、高质量 `IMU` 和 `RTK` 定位，并覆盖多平台、结构化与非结构化环境，同时提供位姿、深度和语义标签，因此特别适合推动 dynamic environments 下更稳健的 event-based 感知算法。

整体趋势很清楚: 事件数据集已经不再只服务于单一 `VO` 任务，而是越来越强调 stereo、`VIO`、dense depth、segmentation、多模态融合，以及在真实复杂场景中的鲁棒评估。

### 10.7.3 指标

理想情况下，`SLAM` 系统应该分别评估定位和建图模块的质量。但现实中，这两者往往是相互耦合的，而且高质量深度真值远比 `6-DoF` 轨迹真值更难获取，因此 depth error 往往会被间接吸收到轨迹误差评估里。

概念上，由于 event-based `SLAM` 和经典 `SLAM` 一样输出相机轨迹，因此其评估协议也大多继承自经典 `SLAM`。最常见的是 `Absolute Trajectory Error (ATE)` 与 `Relative Pose Error (RPE)`。`ATE` 评估相机在固定世界坐标系中的长期轨迹精度；`RPE` 评估相邻时间步或固定时间间隔内的相对位姿一致性。平移误差通常是估计位置与真值位置之间的欧氏距离，旋转误差则可通过 `SO(3)` 上的测地距离计算。

此外，也有工作采用 `ARPE`、`ARRE`、`AEE` 等更细粒度指标，分别评估平均相对平移误差、平均相对旋转误差和端点误差。对于事件相机而言，由于系统往往善于处理剧烈快速运动，因此线速度与角速度误差在某些任务中也很有意义。

若深度估计被单独评估，则常使用不同截断距离下的平均深度误差、`RMSE`、`REL` 和 `completion` 等指标。不过，由于高质量 depth ground truth 依旧稀缺，很多 event-based `SLAM` 论文实际上仍以 pose accuracy 为主要评价标准。

## 10.8 延伸阅读与最新趋势

尽管 event-based `SLAM` 已取得长足进展，但作者认为，这项技术仍然非常年轻，真正重要的问题还远远没有解决。根本性的研究目标，是弄清楚该如何设计硬件和软件，才能在鲁棒性、延迟、功耗和精度等方面接近甚至超越生物视觉系统。

一个鲜明矛盾在于，传感器本身是异步的，但今天绝大多数系统仍运行在串行的 von Neumann 处理器上，这在效率和延迟上都不是最理想的。由此引出的关键趋势之一，便是 `neuromorphic computing`。若未来事件相机能与异步、脉冲式处理器、控制器和执行机构协同设计，那么在 AR/VR、持续在线定位等场景中，或许能真正逼近动物视觉那种高效、低功耗和实时反应能力。

作者据此强调，未来要真正释放 event camera 的潜力，往往不是简单把事件流塞进现有算法，而是需要对传感器、处理器和算法做共同设计（co-design）。在传感器层面，未来值得关注的包括更高分辨率、更低延迟的 hybrid sensors、模仿生物视觉中心凹结构的 foveated sensors，以及 near-sensor processing 等新硬件形态。

与此同时，多模态融合仍是最重要的应用趋势之一。事件相机与 frames、`IMU`、`LiDAR`、radar、structured light 等传感器的深度结合，几乎肯定会持续推动更稳健的 `SLAM` 系统出现。作者的结论相当明确: 真正强大的 event-based `SLAM`，很可能来自对传感器、计算架构和状态估计方法的整体重构，而不只是“让传统图像管线兼容事件流”。

## 第 11 章 面向 SLAM 的惯性里程计

### 本章概览

`Inertial Measurement Units (IMUs)` 已成为机器人 `SLAM` 中最普遍的里程计来源之一。`IMU` 测量其所附着刚体的线加速度与旋转角速度。`IMU` 具有非常宽的形态、成本和性能跨度: 从装在飞机上的大型高精度光学传感器，到智能手机与消费设备中的小型但更嘈杂的 `micro-electromechanical systems (MEMS)`。尤其是 `MEMS IMU` 的 low-SWAP（尺寸、重量、功耗低）和低成本特性，使其极其适合作为机器人传感器，并且它们在 `SLAM` 中已经被持续研究了二十多年。

本章首先介绍 `IMU` 的基本事实及其 measurement model（`11.1`）；随后介绍 `IMU preintegration`（`11.2`），以便将高频 `IMU` 数据加入 `factor graph optimization` 框架。接着，作者指出使用 `IMU` 会给优化引入额外变量，例如 sensor biases，因此还需讨论 `IMU` 与外感知传感器（如 camera 或 `LiDAR`）组合系统的 observability properties（`11.3`）。最后，本章展示现代 IMU-centric `SLAM` 系统能实现的能力（`11.4`），并回顾近期趋势（`11.5`）。

## 11.1 惯性感知与导航基础

一个 `6-axis IMU` 包含 accelerometer 和 gyroscope，分别测量传感器相对于惯性参考系的线加速度与角速度。在航空航天领域，惯性导航系统（`INS`）长期被系统研究，其目标是利用初始状态和一段时间内的 `IMU` 测量来估计平台的当前状态，如位姿和速度。原书在这一节一开始就强调，`IMU` 覆盖了从大型高精度光学传感器到小型但更嘈杂的 `MEMS` 传感器的完整谱系；正是后者的 low-SWAP 与低成本，使其特别适合作为机器人系统的传感器。

`INS` 可分为 strapdown 系统和 stabilized 系统。前者中，`IMU` 刚性安装在平台本体上；后者则通过云台或其他结构尽量维持其朝向不随载体一起变化。机器人领域绝大多数系统属于前者，因此常直接把惯性导航称为惯性里程计（inertial odometry），以强调其本质上也是一种会随时间累积漂移的 odometry。

显然，单靠 `IMU` 做 inertial navigation 会随着时间快速漂移，所以在大多数机器人应用中，它都会与其他传感器联合使用，例如 `GPS`、camera 或 `LiDAR`。这时便形成 aided inertial navigation system（`AINS`）。在实际机器人系统中，我们往往直接用“`visual-inertial odometry`”“`visual-inertial SLAM`”这类名称来指代和 `IMU` 结合的具体传感器组合。

原书这里还给了三条脚注性质的重要补充。第一，`observability` 的作用，是说明在什么条件下估计问题是 well-posed 的，也就是是否有可能根据测量恢复出接近 ground truth 的估计。第二，一个 `IMU` 通常还包含 compass，用于测量磁北方向；但在室内和城市等机器人常见环境中，由于大型金属结构与电子设备引起的局部磁扰会造成很大偏置，因此这类传感器在 `SLAM` 里反而较少使用。第三，虽然在航空航天里会严格区分非惯性导航坐标系与惯性坐标系，但在近地、小尺度、低成本 `IMU` 的机器人应用中，地球自转带来的影响通常远小于测量噪声，因此机器人里常把固定在地球某处的 world frame 近似视作 inertial frame。

### 11.1.1 感知原理与测量模型

一个标准 `IMU` 通常包含 `3-axis accelerometer` 和 `3-axis gyroscope`，分别测量角速度与线加速度。Gyroscope 的基本设计原理来源于角动量守恒；accelerometer 则借助质量块的惯性，测量传感器相对于惯性系的运动加速度与重力加速度之间的差值。实际加速度计的实现方式可以很多，例如把 rate gyroscope 安装成摆式结构，或在低摩擦腔体内布置 proof mass，亦或通过比较悬挂金属带的振动差异来感知加速度。

接下来作者给出了标准 `IMU` measurement model。为简化讨论，假设传感器坐标系与机器人 body frame `F^b` 重合，而 world frame `F^w` 近似视作 inertial frame。此时时刻 `t` 的 `IMU` measurements `a(t)` 与 `ω(t)` 通常写成

```text
a(t) = R_w^b(t) (a^w(t) - g^w) + b^a(t) + η^a(t)      (11.1)
ω(t) = ω_b^b(t) + b^g(t) + η^g(t)                     (11.2)
```

其中，`R_w^b(t)` 描述 `IMU` 在时刻 `t` 的姿态，`a^w(t)` 是传感器在世界系中的加速度，`g^w` 是世界系中的重力向量，因此 `R_w^b(t)(a^w(t) - g^w)` 表示 `IMU` 在 body / `IMU` frame 中感受到的 specific force。`ω_b^b(t)` 则是 body frame 相对世界系的瞬时角速度，并以 body frame 表示。`η^g(t)` 与 `η^a(t)` 被建模为 zero-mean Gaussian white noise，而需要一起估计的 `b^g(t)`、`b^a(t)` 则通常被建模为 slowly varying random walks。作者还特地说明，这里的上标 `g` 和 `a` 表示传感器类型，即 gyroscope 与 accelerometer，而不是坐标系。

虽然式 `(11.1)` 与 `(11.2)` 在机器人里已经是非常实用的模型，但若考虑高精度系统或重新标定传感器平台，就需要引入更精细的误差项。由于制造不完美，accelerometer 可能存在 misalignment 与 scale errors，此时模型可扩展为

```text
a(t) = T^a R_w^b(t) (a^w(t) - g^w) + b^a(t) + η^a(t)  (11.3)
```

其中 `T^a` 是 accelerometer 的 shape matrix，用来同时描述 misalignment 与 scale errors。类似地，gyroscope measurement model 也可写成

```text
ω(t) = T^g ω_b^b(t) + b^g(t) + η^g(t)                 (11.4)
```

其中 `T^g` 对应 gyroscope 的 misalignment 与 scale errors。

此外，gyroscope 往往还会受 acceleration 影响，这一现象称为 `g-sensitivity`。若这种影响不可忽略，模型还可进一步扩展为

```text
ω(t) = T^g ω_b^b(t) + T^s R_w^b(t) (a^w(t) - g^w) + b^g(t) + η^g(t)   (11.5)
```

这里 `T^s` 是 `g-sensitivity matrix`，通常也需要通过 calibration 估计。作者的总体结论是: 简化模型在机器人里常已足够，但在高精度惯性导航、重新标定，或需要严格统计一致性的场景下，这些扩展误差项不能被忽略。

### 11.1.2 初始对准

在 `SLAM` 中，我们常把世界坐标系原点设在轨迹起点，并把初始位姿设为单位位姿；但在惯性导航中，由于 `IMU` 测量显式依赖重力，因此世界系通常要与重力方向对齐。换句话说，初始姿态不再是任意可选的，它必须与重力一致。

若机器人初始静止，则低成本 `MEMS IMU` 通常只能测到重力，因此可以从局部重力向量中恢复 roll 和 pitch，但无法恢复绕重力轴的转角，也就是 yaw。书中用 Gram-Schmidt 正交化给出了如下静态初始化公式:

```text
z_w^b = g^b / ||g^b||
x_w^b = (e_1 - z_w^b e_1^T z_w^b) / ||e_1 - z_w^b e_1^T z_w^b||
y_w^b = z_w^b × x_w^b
=> R_w^b = [x_w^b  y_w^b  z_w^b]                               (11.6)
```

其中 `e_1 = [1 0 0]^T`，这里执行的正是 Gram-Schmidt 正交化，而 `×` 表示叉乘。直观地说，旋转矩阵 `R_w^b` 的最后一列 `z_w^b`，就是世界系 `z` 轴在机体系中的方向；由于世界系 `z` 轴与重力对齐，所以 `(11.6)` 先由机体系下测得的重力向量 `g^b` 求出 `z_w^b`，再补出 `x_w^b` 和 `y_w^b` 来完成一个任意 yaw 选择下的正交基。

对高端 `IMU` 来说，gyroscope 还可以感受到 Earth rotation rate，因此可以同时利用重力方向和地球自转角速度做解析对准。若把机体系下的重力向量记为 `g^b`、地球自转角速度记为 `ω_ie^b`，则有

```text
g^b = R_w^b g^w
ω_ie^b = R_w^b ω_ie^w
g^b × ω_ie^b = R_w^b (g^w × ω_ie^w)
=> R_w^b = [ g^{wT} ; ω_ie^{wT} ; (g^w × ω_ie^w)^T ]^(-1)
           [ g^{bT} ; ω_ie^{bT} ; (g^b × ω_ie^b)^T ]          (11.7)
```

原书还特别指出，实际由该式得到的旋转矩阵通常还需要投影回 `SO(3)`，以缓解测量噪声的影响。无论使用哪种方式，alignment 都是惯性估计的第一步，因为后续全部积分都建立在这一参考系选择之上。

## 11.2 IMU 预积分与 Factor Graph（因子图）

前一节已经介绍了 `IMU` 的测量模型，它将 `IMU` 观测与机器人状态，尤其是位姿、速度和 bias 联系起来。原则上，我们当然可以像第 1 章那样基于这些模型直接写出一个 `MAP` 估计器；问题在于，`IMU` 频率通常极高，可能在 `200-1000 Hz` 之间。若为每个采样时刻都在因子图中加入一个状态变量，图会在短时间内变得极其庞大，几乎无法求解。

因此，`IMU preintegration` 的核心价值就在于: 把两个关键帧之间的大量原始 `IMU` 读数压缩成一个相对运动约束，使高频数据不必以原样全部进入图中。更关键的是，普通积分方式在图优化过程中一旦线性化点变化，常常就得从头重积分；而预积分的设计，恰恰就是为了避免这一点。

`Figure 11.1` 直观展示了 `IMU` 与 camera 的采样频率不同。图引自 [335]（©2016 IEEE）。

### 11.2.1 运动积分

本节首先讨论如何仅从 `IMU` 测量出发推断机器人运动。为此，作者引入如下连续时间运动学模型:

```text
Ṙ_b^w = R_b^w (ω_b^b)^
v̇_b^w = a_b^w
ṗ_b^w = v_b^w                                              (11.8)
```

它描述了机体系 `F_b` 相对于世界系 `F_w` 的旋转 `R_b^w`、平移 `p_b^w` 和速度 `v_b^w` 的演化。

若 `Δt` 是 `IMU` 的采样周期，那么时刻 `t + Δt` 的状态可由对 `(11.8)` 积分得到:

```text
R_b^w(t + Δt) = R_b^w(t) Exp(∫_t^{t+Δt} ω_b^b(τ) dτ)       (11.9)
v_b^w(t + Δt) = v_b^w(t) + ∫_t^{t+Δt} a_b^w(τ) dτ
p_b^w(t + Δt) = p_b^w(t) + ∫_t^{t+Δt} v_b^w(τ) dτ          (11.10)
```

其中，在第一式里作者假设区间 `[t, t + Δt]` 内角速度方向不变。若进一步假设在该短时间区间里 `a_b^w` 和 `ω_b^b` 都近似常值，那么可写成更简单的离散传播形式:

```text
R_b^w(t + Δt) = R_b^w(t) Exp(ω_b^b(t) Δt)
v_b^w(t + Δt) = v_b^w(t) + a_b^w(t) Δt
p_b^w(t + Δt) = p_b^w(t) + v_b^w(t) Δt + 1/2 a_b^w(t) Δt²   (11.11)
```

更一般地说，式 `(11.11)` 可以理解为对 `(11.9)` 所示积分应用了 `Euler integration`。再结合式 `(11.1)` 与 `(11.2)`，即可把 `a^w` 与 `ω^b` 写成 `IMU` measurements 的函数，于是上式变成

```text
R(t + Δt) = R(t) Exp((ω~(t) - b^g(t) - η^{gd}(t)) Δt)
v(t + Δt) = v(t) + gΔt + R(t)(a~(t) - b^a(t) - η^{ad}(t)) Δt
p(t + Δt) = p(t) + v(t)Δt + 1/2 gΔt²
           + 1/2 R(t)(a~(t) - b^a(t) - η^{ad}(t)) Δt²       (11.12)
```

这里作者为了简洁起见省略了坐标系下标，此后记号已足够明确。需要注意的是，对速度和位置的这种数值积分默认在两次 measurement 之间的积分期间姿态 `R(t)` 保持不变；对于非零角速度，这并不是微分方程 `(11.8)` 的精确解。不过在实践中，高频 `IMU` 往往能减轻这一近似带来的误差。作者也因此采用 `(11.12)` 作为后续建模与不确定性传播的基础，因为它足够简单，同时又便于推广；更高阶的积分技巧将在 `11.2.3` 再讨论。

作者还指出，离散时间噪声 `η^{gd}` 的协方差与采样率有关，并满足 `Cov(η^{gd}(t)) = (1/Δt) Cov(η^g(t))`；对 `η^{ad}` 也有同样关系。

从形式上看，式 `(11.12)` 已经可以直接当成 `factor graph` 中的概率约束。但如果真的这么做，就必须以极高频率在图中加入状态。直观地说，`(11.12)` 关联的是 `t` 与 `t + Δt` 两个时刻的状态，而 `Δt` 本身就是 `IMU` 的采样周期，这意味着每来一帧 `IMU` measurement，就得在估计问题里新增一个状态，这在计算上通常不可接受。

为缓解这一问题，可以考虑在更长时间区间上做积分。特别是，当问题中已经有其他传感器的 `factor graph`，例如第 7 章中的视觉测量时，就可以只在图中的两个相邻状态之间累计 `IMU` 测量。作者把这些状态称为 `keyframe states`。如果把式 `(11.12)` 在关键帧 `t_i` 与 `t_j` 之间所有的 `Δt` 区间上迭代应用，就得到:

```text
R_j = R_i ∏_{k=i}^{j-1} Exp((ω~_k - b_k^g - η_k^{gd}) Δt)

v_j = v_i + gΔt_{ij}
    + Σ_{k=i}^{j-1} R_k (a~_k - b_k^a - η_k^{ad}) Δt

p_j = p_i
    + Σ_{k=i}^{j-1} [v_k Δt + 1/2 gΔt²
    + 1/2 R_k (a~_k - b_k^a - η_k^{ad}) Δt²]               (11.13)
```

其中，作者为了书写简洁，引入了记号 `Δt_ij = Σ_{k=i}^{j-1} Δt`，并令 `(·)_k = (·)(t_k)`。式 `(11.13)` 已经给出了从 `t_i` 到 `t_j` 的运动估计，但它仍有一个关键缺点: 一旦关键帧 `i` 处的线性化点改变，例如 `Gauss-Newton` 每轮迭代更新了 `R_i`，那么后续所有 `R_k, k = i, ..., j - 1` 都会随之变化，导致 `(11.13)` 中的乘积和求和都必须重新计算。

这正是下一节引入 `IMU preintegration` 的动机。预积分的目标，不只是把高频 `IMU` 数据压缩成两个关键帧之间的一条约束，更重要的是让这条约束在优化中对线性化点变化不那么敏感，从而避免每次迭代都重做整段积分。

### 11.2.2 流形上的 IMU 预积分

流形上的 `IMU preintegration` 的关键思想，是把两个关键帧之间的相对运动增量写在局部坐标系中，并把重力和全局状态依赖项从测量中分离出去。如此一来，得到的“预积分量”不再依赖全局位姿与速度线性化点，因此在图优化过程中更加稳定，也不需要随着每次状态更新都重新从头积分。

更具体地说，书中定义了相对旋转、相对速度和相对位置增量 `ΔRij`、`Δvij`、`Δpij`。这些量被刻意构造为只与 `IMU` 测量有关，而不显式依赖起点时刻的绝对姿态、位置和速度。正因如此，它们能天然形成连接两个关键帧状态的“复合测量”。

#### 11.2.2.1 预积分 IMU 测量

式 `(11.14)` 已经把关键帧 `i` 与 `j` 的状态关系写成了“左边是状态、右边是 measurements”的形式，因此从概念上它已经可以被视为 measurement model。问题在于，它对 measurement noise 的依赖非常复杂，而 `MAP` estimation 又要求我们明确写出 measurements 的概率密度及其 log-likelihood。因此，作者在本节重新整理式 `(11.14)`，把每个惯性测量中的 noise terms 显式隔离出来。这里先假设时刻 `t_i` 的 bias 已知。

对 rotation increment `ΔR_ij`，作者首先用到 `SO(3)` 指数映射的两个性质:

```text
Exp(ϕ + δϕ) ≈ Exp(ϕ) Exp(J_r(ϕ) δϕ)                    (11.16)
Exp(ϕ) R = R Exp(R^T ϕ)                                (11.17)
```

第一条是指数映射对和的一阶近似，第二条可由群的 adjoint 表示推出。利用这两条关系，就可以把 `ΔR_ij` 中的 noise “移到最后”，写成

```text
ΔR_ij = ΔR̃_ij Exp(-δϕ_ij)                              (11.18)
```

其中 `ΔR̃_ij` 是预积分 rotation measurement，`δϕ_ij` 是对应噪声。

接着处理 velocity 与 position。这里还要用到

```text
exp(ϕ^) ≈ I + ϕ^                                        (11.19)
a^ b = - b^ a,   ∀ a,b ∈ R^3                           (11.20)
```

其中第一条是在原点处的一阶近似，第二条是向量 wedge operator 的基本性质。把 `(11.18)` 代回式 `(11.14)` 中的 `Δv_ij`，再使用 `(11.19)` 并忽略高阶噪声项，可得

```text
Δv_ij = Δṽ_ij - δv_ij                                  (11.21)
```

同理，把 `(11.18)` 与 `(11.21)` 再代回 `Δp_ij`，就得到

```text
Δp_ij = Δp̃_ij - δp_ij                                  (11.22)
```

最终，将 `(11.18)`、`(11.21)`、`(11.22)` 统一代回原始定义后，便得到真正可用于 factor graph 的预积分 measurement model:

```text
ΔR̃_ij = R_i^T R_j Exp(δϕ_ij)
Δṽ_ij = R_i^T (v_j - v_i - g Δt_ij) + δv_ij
Δp̃_ij = R_i^T (p_j - p_i - v_i Δt_ij - (1/2) g Δt_ij^2) + δp_ij   (11.23)
```

这里复合 measurement 由待估计状态加上一项随机噪声构成，噪声向量为 `[δϕ_ij^T, δv_ij^T, δp_ij^T]^T`。作者总结说，这样重写后的优势在于: 只要噪声分布合适，这些 measurements 就可以直接作为连接状态 `i` 与 `j` 的因子加入 factor graph。

#### 11.2.2.2 噪声传播

接下来需要推导噪声向量 `[δϕ_ij^T, δv_ij^T, δp_ij^T]^T` 的统计量。虽然我们已经知道把它近似为零均值 Gaussian 很方便，但真正关键的是准确建模它的 covariance。作者因此把预积分 measurement noise 写成

```text
η^Δ_ij = [δϕ_ij^T, δv_ij^T, δp_ij^T]^T ~ N(0, Σ_ij)     (11.24)
```

先看 rotation noise。由 `(11.18)` 可写成

```text
Exp(-δϕ_ij) = Π_k Exp(-ΔR̃_(k+1,j)^T J_r^k η_k^gd Δt)   (11.25)
```

对两侧取 `Log` 并改变符号，再使用 `SO(3)` 上对数映射的一阶近似

```text
Log(Exp(ϕ) Exp(δϕ)) ≈ ϕ + J_r^(-1)(ϕ) δϕ              (11.27)
```

反复应用后，就得到一阶近似下的

```text
δϕ_ij ≈ Σ_k ΔR̃_(k+1,j)^T J_r^k η_k^gd Δt              (11.28)
```

因此 `δϕ_ij` 是零均值 Gaussian。随后，`δv_ij` 和 `δp_ij` 也就容易处理了，因为它们都只是 acceleration noise `η_k^ad` 与 preintegrated rotation noise `δϕ_ij` 的线性组合。书中给出

```text
δv_ij ≈ Σ_k [ -ΔR̃_ik (ã_k - b_i^a)^ δϕ_ik Δt + ΔR̃_ik η_k^ad Δt ]   (11.29)
```

`δp_ij` 也可类似写成 `δv_ik`、`δϕ_ik` 与 `η_k^ad` 的线性组合。由此，`η^Δ_ij` 整体就是各个单次 `IMU` measurement noises 的线性函数，因此只要已知 `η_k^d = [η_k^gd, η_k^ad]` 的 covariance，就可以通过简单线性传播得到 `Σ_ij`。

作者还提到，[335] 给出了更完整的推导，并提供了一个可以随着新 measurements 到来而迭代更新 covariance 的形式，这种写法对 online inference 更友好。

#### 11.2.2.3 融合偏置更新

前面暂时假设预积分时所用的 bias `{b̄_i^a, b̄_i^g}` 是正确且保持不变的；但在真实优化过程中，bias estimate 往往会发生一个小更新 `δb`。一种直接做法是 bias 一变就重新预积分，但那样代价太高。作者采用的办法，是在 `b̄` 处完成一次预积分，然后对 `b ← b̄ + δb` 做一阶修正:

```text
ΔR̃_ij(b_i^g) ≈ ΔR̃_ij(b̄_i^g) Exp((∂ΔR_ij/∂b^g) δb_i^g)                    (11.30)
Δṽ_ij(b_i^g, b_i^a) ≈ Δṽ_ij(b̄_i^g, b̄_i^a) + (∂Δv_ij/∂b^g) δb_i^g + (∂Δv_ij/∂b^a) δb_i^a
Δp̃_ij(b_i^g, b_i^a) ≈ Δp̃_ij(b̄_i^g, b̄_i^a) + (∂Δp_ij/∂b^g) δb_i^g + (∂Δp_ij/∂b^a) δb_i^a
```

这些 Jacobians 描述了 measurements 对 bias 更新的敏感度，并且都可以在 preintegration 阶段预先计算并缓存下来。其推导方式与 `11.2.2.1` 中把 measurements 写成“大量值 + 小扰动”的做法非常类似，详见 [335]。

#### 11.2.2.4 预积分 IMU 因子与偏置模型

有了 `(11.23)` 的 measurement model，以及一阶近似下零均值 Gaussian 的 measurement noise `(11.24)`，就可以直接写出 factor graph 里的 `IMU` residuals。书中把它们记为

```text
r^I_ij = [r^T_(ΔR), r^T_(Δv), r^T_(Δp)]^T ∈ R^9
```

并展开为

```text
r_(ΔR) = Log( ΔR̃_ij(b̄_i^g) Exp((∂ΔR_ij/∂b^g) δb_i^g)^T R_i^T R_j )
r_(Δv) = R_i^T (v_j - v_i - g Δt_ij)
         - Δṽ_ij(b̄_i^g, b̄_i^a) + (∂Δv_ij/∂b^g) δb_i^g + (∂Δv_ij/∂b^a) δb_i^a
r_(Δp) = R_i^T (p_j - p_i - v_i Δt_ij - (1/2) g Δt_ij^2)
         - Δp̃_ij(b̄_i^g, b̄_i^a) + (∂Δp_ij/∂b^g) δb_i^g + (∂Δp_ij/∂b^a) δb_i^a   (11.31)
```

把这些 residuals 通过 `Σ_ij` 加权后加入目标函数，就得到连接关键帧 `i` 与 `j` 的 `IMU factor`。

最后，bias 本身还必须作为状态的一部分进行建模。由于前文已经说明 bias 是 slowly time-varying quantity，因此作者采用 `Brownian motion`，也就是 integrated white noise 模型:

```text
b˙g(t) = η^bg
b˙a(t) = η^ba                                          (11.32)
```

在离散关键帧间，这又会形成额外的 bias evolution factor，用来约束 `b_i` 与 `b_j` 之间的变化。

在离散关键帧间，这会形成一个额外的 bias evolution factor，用来约束 `bi` 和 `bj` 之间的变化。因此，在现代 visual-inertial / inertial `SLAM` 系统中，`IMU` 不只是一个 propagation 工具，而是一个带有自身状态、噪声模型、演化方程与跨时刻约束的完整概率组件。

### 11.2.3 高级预积分技术

标准预积分的优点是简单、有效，但其背后隐含着一系列数值与运动假设，例如 Euler 积分、区间内信号近似常值等。在高动态、低采样率或异步传感器融合场景中，这些假设会带来明显误差。因此，后续研究开始从数值积分精度和连续时间建模两方面改进预积分。

#### 11.2.3.1 数值积分精度

标准预积分在离散时间上依赖 `Euler method`，用它把惯性信号积分为离散时刻上的旋转、速度与位置 pseudo-measurements。这种做法速度快、实现简单，但也会把积分误差直接带进预积分过程，最终表现为额外漂移。

从数值分析角度看，`Euler integration` 本质上就是对输入信号应用 rectangle rule。也就是说，它把连续信号近似成按采样频率分段常值的若干小矩形。对惯性系统来说，这些采样值就是 accelerometer 与 gyroscope measurements。图 11.2 左侧正是用这种方式说明 signal approximation 的。

当 sampling frequency 不够高时，分段常值假设就不能很好地逼近真实输入信号。这样一来，积分误差会快速累积，尤其在对加速度做双重积分得到位置时更明显，图 11.2 右侧就展示了这种误差增长。

`Figure 11.2` 展示了在已知初始条件下使用 `Euler integration` 的结果：上排对应较低采样频率，下排对应较高采样频率。

一种直接补救办法，是提高采样频率，使矩形近似更精细。但在真实惯性导航系统中，采样频率终究受 `IMU` 硬件能力限制，不可能无限制提升。

为此，文献 [630] 提出使用 `Gaussian Process (GP) regression` 来对输入信号做“虚拟上采样”，使 gyroscope 与 accelerometer 数据都可以在任意选定时间戳上被估计出来。这样一来，数值积分会比标准预积分更准确。不过，作者也指出，这类做法本质上仍然建立在 piecewise-constant 的数值积分思路之上，并没有真正发挥 `GP` 连续模型的全部潜力。也正因此，后续工作才进一步发展出更复杂的连续时间积分方法。

这里原书还脚注说明：`GP regression` 是一种 non-parametric、probabilistic interpolation approach，想更深入理解可参考 [908]。

#### 11.2.3.2 连续加速度预积分

另一条路线，是用连续时间表示直接逼近旋转校正后的加速度信号，并对其做解析积分。由于连续时间表示可在任意时间查询，因此它尤其适合与不同步传感器，乃至 event camera 这类完全异步的模态联合使用。

书中特别说明，连续时间预积分通常会先把 rotation 与 translation 两部分暂时拆开处理，因为 rotation space 的非交换性使其更难直接纳入经典积分工具。于是，本节主要讨论在“旋转部分已先求解”的前提下，如何对 translation component 做连续时间预积分，而 rotation 的连续时间处理则留到下一小节。

作者提到几类代表性做法。一类工作先利用零阶积分器处理 gyroscope measurements，再把 velocity 和 position 的预积分写成连续时间线性时变系统，并分别假设 constant accelerometer measurements 或 constant local acceleration。与标准预积分里常见的 `constant global acceleration` 假设相比，`constant local acceleration` 更贴近真实运动，因此在 `EuRoc` 一类数据集上能带来可观的 `VIO` 精度改进。

若想进一步放松 constant-acceleration 假设，就可以直接用解析可积的连续函数逼近输入信号。书中提到两类典型做法。一类是假设旋转校正后的加速度为 `piecewise-linear`，也就是近似 `constant jerk`。这时，从加速度到速度的第一次积分就退化为经典梯形积分规则，已经能比 Euler 带来显著精度提升。另一类则更进一步，用 `GP` 直接建模旋转校正后的加速度信号。由于 `GP` 在线性算子作用下仍保持 `GP` 结构，因此可以解析地推导积分和双重积分结果。

作者给出的示例显示，这种非参数、`model-less` 的连续表示，在速度和位置积分精度上都优于分段线性方法。不过，`GP` 中 kernel 的超参数会直接控制信号平滑度，因此既可以通过数据学习得到，也可以用经验方式设定。

`Figure 11.3` 的上排展示了基于分段线性近似的连续积分，对应 constant-jerk motion assumption；下排展示了使用 `Gaussian Process` regression 的 model-free 连续积分。

#### 11.2.3.3 连续旋转预积分

相比平移部分，rotation preintegration 更难，因为旋转属于 `SO(3)` Lie group，不具备欧式空间的交换性，很多经典积分工具不能直接使用。为此，有工作将旋转表示映射到 Lie algebra 中的 rotation vector，再在这个线性空间里用 `GP` 建模连续旋转函数，并借助虚拟观测值来拟合旋转动态。

作者先强调问题的根源: 解旋转运动学所需的 product integral 在一般情形下没有已知的通用闭式解，因此要想做 continuous-time、model-less 的 rotation preintegration，就必须引入新的表示和优化思路。

一条代表性路线，是把旋转 `R(t)` 改写为 Lie algebra 中的 `rotation vector` `r(t)`，满足 `R(t) = Exp(r(t))`。这样，旋转就被映射到了一个局部线性空间中，可以借助线性工具进行连续建模。在该空间里，系统动力学写成与 `SO(3)` 右 Jacobian 相关的形式；但麻烦在于，`r` 和 `rdot` 本身都不是 `IMU` 直接观测到的量。

关键想法是: 用 `GP` 去建模 `r`，同时引入一组虚拟观测值作为连续旋转轨迹的 control points，再通过一个非线性最小二乘问题，使这些虚拟观测与 gyroscope measurements 共同满足旋转动力学约束。这样得到的，就是一种连续时间、无显式运动模型的旋转预积分方法。书中指出，这类方法相较标准离散预积分，精度提升可达到至少一个数量级。

作者还把它与第 2 章提过的 `STEAM` 连续时间状态估计做了比较。两者都在 Lie algebra 中使用 `GP` 做插值，但这里使用的是 square exponential kernel，因此会得到稠密线性系统；不过对 `IMU` 预积分而言，单个积分窗口通常足够短，所以求解稠密系统在实践中仍可接受。后续工作还进一步将 rotation vector 与旋转校正后的加速度联合估计，从而让预积分协方差显式体现旋转与平移之间的相关性。

## 11.3 辅助惯性导航的可观测性

即使测量模型与预积分都构造正确，如果问题本身在某些方向上不可观，估计器仍然不可能可靠工作。因此，除了“如何融合 `IMU`”，另一件必须搞清楚的事就是“哪些量本来就不可观”。这正是 observability analysis 要回答的问题。

### 11.3.1 线性化测量模型

这里考虑的测量模型假设是: 用来辅助 `IMU` 的外感知传感器，例如 camera 或 `LiDAR`，输出的是几何特征。换言之，作者关注的是那些会生成 landmark-based representations 的 `SLAM` 或 odometry 前端。虽然多数 `AINS` 主要使用点特征，尤其在依赖 camera 的系统中更常见 [456, 652, 644, 895, 375, 335]，但在条件允许时也可以引入线与平面特征 [598, 455, 414, 1235]。这样一来，待估计状态向量就往往需要同时包含机器人自身状态与外部几何特征状态。

更具体地说，书中把每个时刻待估计的 `AINS` 状态写为机器人状态 `x_b` 与以世界坐标系表达的外部特征状态 `x_f^w` 的组合:

`x = {R_b^w, b^g, v^w, b^a, p^w, x_f^w}`  。`(11.37)`

其中 `R_b^w` 是 body frame `F_b` 相对于 world frame `F_w` 的旋转，`p^w` 与 `v^w` 是以世界系表示的位置和速度，`b^g` 与 `b^a` 分别是 body frame 下的 gyroscope bias 与 accelerometer bias，而 `x_f^w` 则可以由点、线、平面，或它们的组合构成。

为了开展后续的可观测性分析，既需要系统动力学模型，也需要外感知测量模型。作者下面先回顾由 `IMU` 驱动的惯性导航运动学，再给出外感知测量方程的线性化形式。

#### 11.3.1.1 线性化 IMU 运动学模型

基于前文 `(11.8)` 与 `(11.32)`，`IMU` 的连续时间运动学写为:

`Ṙ_b^w = R_b^w (ω^b)^,   v̇^w = a^w,   ṗ^w = v^w` ，

并令两类 bias 服从 random walk:

`ḃ^g(t) = η^bg,   ḃ^a(t) = η^ba` 。

其中 `η^bg` 与 `η^ba` 是驱动 gyroscope bias 与 accelerometer bias 的零均值 Gaussian 噪声。为了分析可观测性，作者把这一非线性系统在线性化点附近展开，得到连续时间误差状态模型:

`x̃̇(t) ≃ F(t) x̃(t) + G(t) η(t)` 。

误差状态向量可写为 `x̃ = {θ̃, b̃^g, ṽ^w, b̃^a, p̃^w, x̃_f^w}`。其中旋转部分不是直接在线性空间中表示，而是在当前线性化点的 tangent space 中用 `θ̃` 表示。直观上，这相当于把真实旋转写成线性化点附近的一个小扰动，并使用小角度近似

`R_b^w ≃ R̂_b^w (I + θ̃^)`

来完成线性化。这里 `η(t)` 是堆叠后的噪声项，不仅包含 `η^bg` 与 `η^ba`，也包含把真实角速度与加速度替换成 gyroscope / accelerometer measurements 时引入的 `IMU` 噪声。

由于实际 `AINS` 往往以离散时间实现，还需要从连续时间系统得到离散状态转移矩阵 `Φ_(k+1,k)`，它由

`Φ̇_(k+1,k) = F(t_k) Φ_(k+1,k)`

并以单位阵为初始条件积分得到。书中给出了 `Φ_(k+1,k)` 的块结构形式: 与姿态、速度、位置和 bias 相关的各个子块之间存在标准耦合，而特征子状态对应的传播块则保持为单位阵。各个块 `Φ_ij` 可以解析求出，也可以用数值方式计算 [456]。这一离散转移矩阵随后会与测量 Jacobian 一起构成 observability matrix 的基本组件。

#### 11.3.1.2 外感知测量模型

现在作者给出不同几何特征的测量模型及其线性化形式，因为这些模型是开展线性化 `AINS` 可观测性分析的基础。

对点特征，外感知传感器，如 monocular / stereo camera、声纳和 `LiDAR`，通常都可以统一为 `range-and-bearing` 观测。若特征在传感器坐标系 `Fc` 中的相对位置为 `p_f^c`，则测量可以写成距离与方位的组合，并通过一个带有二值项 `λr`、`λb` 的 selection matrix `Λ` 来表示“当前传感器到底提供 range、bearing，还是两者同时提供”。在当前状态估计附近线性化后，可通过链式法则得到形如 `ẑp ≈ Hx x̃ + η̃x` 的误差方程；根据 `Λ` 的不同取值，Jacobian `Hx` 可以只包含 range 项、只包含 bearing 项，或同时包含二者。

对线特征，作者先用 `Plücker coordinates` 表示由两个三维点定义的直线:

`l^w = [nℓ^w; vℓ^w] = [p1^w × p2^w; p2^w - p1^w]`

其中 `nℓ^w` 是 line moment，`vℓ^w` 是直线方向向量。该直线可从世界系变换到当前相机系，再通过已知内参投影到图像平面。若图像中观测到的线段端点为 `q1` 和 `q2`，则视觉线特征测量可定义为这两个端点到投影线的距离。随后同样可用链式法则线性化，得到对应的测量 Jacobian。

对平面特征，则用平面法向量与原点距离来参数化:

`π^w = [nπ^w; dπ^w]`

它可以被变换到局部传感器坐标系。对点云传感器，如 `LiDAR` 或 depth sensor，作者采用平面到原点的最近点 `pπ^c = dπ^c nπ^c` 作为状态中的平面表示，因此测量可写成 `zπ = pπ^c + ηπ`。对该表达在线性化后，也可得到相应的 plane measurement Jacobian。

这些测量模型在当前状态估计附近线性化后，就能与离散 `IMU` 误差状态模型共同组成统一的 observability analysis 框架。其意义在于，很多状态量究竟是否可观，并不取决于某一类传感器单独存在，而取决于惯性传播与外感知约束是否共同提供了足够信息。

### 11.3.2 可观测性分析

基于前面线性化后的动力学模型与测量模型，现在可以正式进行 observability analysis。作者使用的工具是 observability matrix `M(x̂)` [486]:

`M(x̂) = [ H_x1 Φ_(1,1);  H_x2 Φ_(2,1);  ... ;  H_xk Φ_(k,1) ]` 。

其中 `H_xk` 堆叠了离散时刻 `k` 收集到的所有测量 Jacobian，可能来自点、线或平面特征；记号 `M(x̂)` 则强调 observability matrix 依赖于当前线性化点 `x̂`。该矩阵的 null space `U`，也就是所有满足 `M(x̂) u_i = 0` 的向量所张成的空间，精确描述了 `AINS` 的不可观子空间。若 `U` 为空，系统完全可观；否则，对应的 null vectors 就指出了哪些状态方向无法由现有测量恢复。

已有研究表明 [1233]，一般的 `AINS` 通常具有 `4` 个不可观方向，也就是 null space 中存在四个线性无关向量。这四个方向对应全局 `3D` 位置与全局 yaw 无法由 `IMU` 测量和对未知地标的局部观测唯一确定。

为了更清楚地理解这四维 null space 的结构，作者考虑一个同时把三类几何特征都放入状态向量的例子，即 `x_f^w = {p_f^w, l_f^w, π^w}`，而外感知测量则包含 `z = {z_p, z_l, z_π}`。将相应系统 Jacobian 与测量 Jacobian 代入上式后，可以构造出线性化的 `AINS observability matrix`，并显式求得其 null space `null(M)`。书中给出了四个 null vectors 的具体表达式 `(11.51)`。虽然表达式本身较长，但它们的几何含义非常清楚: 第一个 null vector `u_1` 对应绕重力方向的旋转，因此就是 yaw 不可观；其余三个向量 `u_2:4` 对应系统整体在空间中的平移自由度。

作者还进一步解释了这些向量中的组成部分。例如，`u_g` 与重力方向和初始速度有关；`R_π^w` 是用平面法向量通过 `Gram-Schmidt orthonormalization` 构造出的旋转矩阵；`R_ℓ^w` 则由直线法向与方向向量构成。通过这些构造，可以把点、线、面三类特征在不可观子空间中的耦合关系写成显式形式。

总结来说，observability matrix 存在四维 null space，准确地描述了 `AINS` 的全局位置与 yaw 不可观这一事实。直观上，`IMU` 数据以及对未知点、线、面地标的相对观测，都不会告诉系统它相对于某个绝对世界系的原点和绝对 yaw 在哪里。唯一的例外是 roll 与 pitch，它们可以通过 accelerometer 对重力方向的测量变得可观。

这种不可观性在 `SLAM` 中非常常见，并不是 pathological failure，而只是说明我们可以任意设定世界坐标系的原点与 yaw，因为对应变量只以相对形式被观测。一旦引入提供绝对观测的传感器，例如 `GPS`，这种不确定性就会消失。更值得警惕的是，在某些运动模式或某些线性化点附近，observability matrix 的 null space 还可能继续增大，产生额外的不可观维度；这正是下一节将讨论的内容。

### 11.3.3 退化运动

比四个标准不可观方向更令人头痛的是，某些运动模式会进一步扩大 observability matrix 的 null space，从而引入新的不可观方向。作者将这类情况称为 degenerate motions。

例如，纯平移会使系统的全局姿态都变得不可观，因为缺少转动时，重力测量和 accelerometer bias 之间更容易发生混淆；而常加速度、纯旋转，以及对 monocular 相机而言“朝着特征点正向前进”的运动，则会导致尺度不可观或特征尺度不可观。尤其对 monocular visual-inertial 系统来说，这些退化运动会直接影响是否能稳定恢复 scene scale。

因此，一个好的估计系统不仅依赖传感器质量，也依赖 sufficiently exciting 的运动激励。很多时候，问题并不是“算法失效”，而是机器人当前的运动本身没有向系统提供足够的信息。

## 11.4 视觉惯性里程计与实践考虑

前面几节主要讨论的是惯性建模与理论分析，本节则把这些内容落到最常见的实际系统中: `Visual-Inertial Odometry (VIO)`。作者将讨论 `VIO` 的典型因子图结构、系统性能特点，以及外参标定与时间同步等工程上不可回避的问题。

### 11.4.1 视觉惯性里程计

正如前文所述，惯性测量通常会与其他传感器融合，以减轻纯 inertial odometry 的漂移。本节重点讨论将 camera 与 `IMU` 通过 factor graph 融合的情形。Camera 和 `IMU` 是非常流行的一对组合，因为它们都 inexpensive、lightweight、low-power，而且高度互补: `IMU` 擅长捕捉快速 acceleration 与 rotation，camera 则擅长提供丰富的环境观测信息。一方面，camera 的加入显著降低了纯惯性解的漂移；另一方面，`IMU` 又能让某些纯视觉下不可观的量变得可观，例如 monocular `SLAM` 中的场景尺度，只要运动本身不是退化的。

`VIO` 通常被当作 odometry source 使用，也常被直接用于 trajectory tracking 与 control loop closure。在另一些应用，例如虚拟现实，`VIO` 则负责补偿用户在虚拟环境中的运动。无论是哪种应用，系统都要求非常低的延迟，典型量级在 `10-50 ms`。作者举例说，`Meta Quest 3` 的刷新率大约在 `72-120 Hz` 之间，因此 `VIO` latency 会直接影响用户体验，甚至影响 motion sickness；而对轨迹跟踪控制来说，过大的延迟则会诱发控制器不稳定。

`Figure 11.4` 展示了一个使用 preintegrated `IMU` factors 的 visual-inertial odometry factor graph 示例 [335]：紫色为 preintegrated `IMU` factors，用于约束连续位姿、速度与 bias；蓝色为 bias factors，用于约束 `IMU` bias 随时间的演化；橙色为视觉因子，用于关联 camera poses 与外部 landmarks；黑色则是 priors。

正因如此，基于 factor graph 的 `VIO` 系统通常采用 `fixed-lag smoother`，也就是 `sliding-window optimization`。系统只估计一个 receding horizon 内的状态，例如最近 `5-10` 秒，而不是整条历史轨迹。图 `11.4` 展示了一个典型因子图: 紫色是 preintegrated `IMU` factors，用于约束连续关键帧之间的 pose、velocity 与 bias；蓝色是 bias factors，用于约束 `IMU` bias 随时间的演化；橙色是视觉因子，把 camera poses 与外部 landmarks 连接起来；黑色则是 priors。

滑动窗口长度的选择，本质上是在 computation 和 accuracy 之间权衡。窗口越长，理论上状态估计越精确，但优化问题也越大。随着时间推进，超出窗口的 factors 和 variables 会被逐渐 marginalize 掉。为了进一步减少优化维度，许多实现还会用 `Schur complement` 把 visual landmarks 从状态中消除，例如 [335]。另一条路线是使用像 `iSAM2` 那样的 incremental solver，复用历史优化结果来减少重复计算。作者指出，这种方法在实践中确实可能非常准确 [335]，但它难以为系统提供 latency upper bound，因为在某些时刻运行时间可能出现 spike，这对某些实时应用是不利的。

过去十年里，open-source 的 visual-inertial odometry / `SLAM` 系统大量涌现，代表方法包括 visual-inertial 版 `ORB-SLAM` [785]、`Direct Sparse Visual-Inertial Odometry` [1115]、`VINS-Mono` [895]、`OpenVINS` [375]、`Kimera` [3, 941]、`BASALT` [1117] 和 `DM-VIO` [1042]。作者给出的经验标准是: 一个好的 `VIO` 系统，漂移应低于路径长度的 `1%`，优秀系统则可达到 `0.1%` 左右。

书中还给出一个具体例子: 较新的 `FEJ`-based `Window Bundle Adjustment (WBA)-VINS` [180, 181] 在 `KAIST Urban Dataset` [514] 的 sequence `38` 上运行，处理的是一条长达 `11.42 km`、总时长约 `36` 分钟的城市轨迹。该数据集由搭载 stereo cameras、`2D/3D LiDARs`、`Xsens IMU`、`FoG`、wheel encoders 和 `RTK GPS` 的车辆采集，camera 频率为 `10 Hz`，`IMU` 频率为 `100 Hz`，并通过 `FoG + RTK GPS + wheel encoders` 提供 ground-truth trajectory。该 `VIO` 系统在不使用 loop closure 的纯在线条件下，最终取得约 `2.05` 度和 `21.2 m` 的 `ATE`，相当于整条轨迹长度的约 `0.18%`。

### 11.4.2 外参标定

若 `IMU` 与 camera 之间的相对位姿，也就是 extrinsic calibration，不准确，那么再好的后端也只能在错误模型上优化。因此，外参标定是 aided inertial navigation 成败的关键之一。

相关方法大致可分为 offline calibration 和 online calibration 两类。Offline 方法要求在部署前专门执行标定流程，可能需要标定板、已知运动模式，或某些环境先验。这类方法往往更准确，但流程繁琐，且常依赖专门设备和训练有素的操作者。Online 方法则把外参参数直接并入状态估计问题，在系统运行过程中同时估计，这样一旦传感器装配略有位移，也不必重新专门标定。

不过，online calibration 的代价是问题规模更大，而且在某些运动与观测配置下，外参甚至可能变得不可观，从而让状态估计本身变复杂甚至 ill-posed。作者因此提醒，在线标定并非总是“更方便的替代品”，而是一个需要谨慎设计的估计问题。

### 11.4.3 时间同步

除了空间外参，时间同步同样是惯性辅助系统中的关键问题。若不同传感器之间存在未建模的时间偏移，轨迹估计会产生显著系统误差，benchmark 指标也可能带入虚假 bias。尤其在高动态运动中，哪怕只有几毫秒偏差，也可能显著恶化残差和最终精度。

时间同步可以通过硬件或软件完成。硬件方案通常依赖公共时钟信号触发各传感器采样，但并非所有设备都支持这种同步引脚。软件方案中，一个典型例子是基于以太网的 `PTP (Precision Time Protocol)`；另一些系统则在传感器端打时间戳，并在后处理中对齐。后一种方式通常精度和鲁棒性都不如硬件同步。

如果系统无法严格同步，且又必须在线运行，那么也有方法把时间偏移直接作为状态变量纳入估计问题。对现代 `VIO` 而言，这往往是非常实际的设计选项。

## 11.5 延伸阅读与最新趋势

尽管 inertial odometry 的进展已经在不断走向工业产品，但 `aided inertial navigation` 依旧是研究非常活跃的方向。作者最后概括了几类值得持续关注的新趋势。

第一类是 `Extended Pose Preintegration`。最近的惯性里程计研究，开始使用 extended-pose manifolds 和更高阶的噪声传播，来改进 `IMU preintegration` 的不确定性建模。例如 `Brossard` 等人的工作把地球自转、科里奥利力和离心力都纳入了预积分理论；`Vial` 等人则展示了一个结合线速度传感器和导航级 `IMU` 的 extended pose preintegration 例子，在长达一小时、总长约 `1.8 km` 的海上导航轨迹中，平移误差仅约 `5 m`。

第二类是 `Continuous-time State Representations`。本章主要从预积分角度出发，把 `IMU` 看成减少离散状态变量数量的工具；但也有另一条路线，是直接采用连续时间状态表示，在不增加估计状态维度的前提下吸收大量高频惯性测量。典型例子包括基于 `B-spline` 的表示和基于 `GP priors` 的表示。两者都允许在一组固定状态变量之间，通过插值动力学残差把高频 `IMU` 测量纳入优化。作者还提到，近期工作比较了“把 `IMU` 直接作为连续时间 `GP prior` 的输入”和“把 `IMU` 直接作为 residual measurements”这两种方式，结果表明，后者在 `LiDAR-inertial` 组合中能得到更好的 odometry 精度。还有工作进一步比较了前文讨论过的 `continuous GP-based preintegration` 与标准 `GP-based state representation`，并在 event-based `VIO` 场景中发现，前者在精度和计算效率上都略占优势。

第三类趋势是 `Proprioception-only Odometry`。近年的一些系统开始主要依赖本体感觉传感器来辅助惯性导航。例如在腿式机器人上，可以利用足端接触状态；在轮式平台上，可以利用 wheel-mounted `IMU` 的运动学约束，从而得到误差低于百分之一量级的 `IMU` 里程计。在前一种情况下，关键先验是“足与地面的接触关系”；在后一种情况下，则利用“单平面旋转运动”来约束 `IMU` 偏置，从而抑制 dead-reckoning drift。后者还被进一步扩展成完整 `SLAM` 系统，通过识别随时间变化的道路横滚模式来检测 loop closure，这构成了一个很有意思的近纯惯性、本体感觉 `SLAM` 例子。作者也提醒，尽管惯性传感器通常更稳健，但 `IMU` dropout 或传感器饱和有时会给整个系统造成灾难性影响。为此，也有工作研究在 gyroscope 饱和时，改用 accelerometer 数据去估计角速度，以提升后续 `SLAM` 算法的鲁棒性。

第四类趋势是 `Inertial-only Odometry (IOO)`。如果没有视觉等外部辅助，仅仅对 `IMU` 测量做朴素积分，里程计通常会很快发散。这不仅是纯惯性系统的问题，也是在视觉暂时失效时，视觉惯性系统必须面对的现实风险。例如在移动 AR/VR 中，快速运动的手可能离开跟踪相机视场；或者在无纹理环境里，视觉前端根本无法稳定提取和跟踪特征，只能暂时完全依赖 `IMU`。因此，近年来大量工作开始尝试用学习方法和神经网络来降低 `inertial-only odometry` 的漂移，包括用数据驱动方式建模 `IMU bias`、直接从一段带噪 `IMU` 序列中预测位移，或者在可微积分模块中先估计并移除 bias 再做积分。还有工作使用真实 bias 监督，或者用 conditional diffusion model 把 bias 当作概率分布去近似。作者认为，这些方法已经显示出大幅降低纯惯性漂移的潜力，但泛化能力目前仍有限，例如很难无缝适应不同传感器或训练中未出现过的运动模式。

最后一类趋势，是面向嵌入式边缘平台的 `ultra-efficient and robust VIO`。即便 `SLAM` 算法本身不断进步，小型嵌入式机器人仍然面临严苛的 `SWAP`（size, weight, and power）约束。作者特别指出，在许多系统里，真正昂贵的并不只是运算本身，而是数据管理，尤其是 `RAM` 数据访问。例如在 `Meta XR` 可穿戴设备的 `SLAM` 与 hand-tracking 模块里，主要能耗往往来自内存访问而非纯计算。为降低数据搬运代价，研究者开始探索 `on-sensor computing` 架构，也提出了量化版 `VIO (QVIO)`。对于只具备单精度浮点运算能力、或者必须依赖单精度才能满足实时性的低 `SWaP` 平台，新近也出现了 `square-root` 形式的 information / covariance filters，用于在保持数值稳定性的同时提升效率。此外，还已有面向芯片级 visual-inertial odometry 的 `ASIC` 设计与实现工作。

总体而言，`IMU` 仍然是现代 `SLAM` 系统中最关键的动态骨架之一。但今天真正困难的问题，已经不再是“能否把 `IMU` 用进系统”，而是如何让惯性信息在统计上一致、在系统上鲁棒、在算力与功耗上都可控。

## 第 12 章 面向 SLAM 的腿式里程计

### 本章概览

腿式机器人由于能够穿越高度非结构化地形而日益普及。它们的主要优势在于，腿提供了一种主动悬架（active suspension），使机器人机体的运动能够与地形轮廓解耦。因此，腿式平台能够跨越楼梯、不平整地面以及其他对轮式平台极具挑战的地面障碍。尽管本 handbook 其他章节回顾的 `SLAM` 算法同样可直接应用到腿式机器人上，但腿本身引入的额外 sensing 是一种非常有价值的新信息源，可被用于 odometry。

这一点对腿式 locomotion control 与 planning 尤其重要，因为为了防止跌倒和失败，系统必须具备 high-frequency、low-drift 的实时 pose 与 velocity estimation，而这本身就是一项困难任务。

本章将讨论如何为一台装备了 onboard `IMU` 和 joint sensing（position 与 torque）的腿式机器人估计实时 pose 与 velocity。重点会放在 `leg odometry` 上，也就是如何利用腿部 sensing 来恢复机器人机体的相对运动。作者将先介绍历史背景和预备知识（`12.1`），再介绍用于估计腿式里程计的关键理论工具（`12.2` 与 `12.3`）。其中会解释如何在已知 stance legs 的前提下，从腿部 joint sensing 恢复相对运动（`12.2`）；这使得 `leg odometry` 可以像 wheel odometry 或 inertial preintegration 一样，被看作 `SLAM` 问题中的一种测量。随后还会介绍接触如何被估计及其实践技术（`12.3`），再讨论如何利用 `factor graphs` 将 leg odometry 与其他传感模态结合起来（`12.4`）。最后，本章总结该方向中的开放问题（`12.5`）及其近期趋势（`12.6`）。

## 12.1 历史背景与预备知识

尽管现代腿式平台往往也会搭载 camera、`LiDAR` 和 `IMU`，但腿本身又额外提供了一类独特信息源: 通过利用腿部的运动学与动力学信息，可以获得机器人机体相对运动的测量，这正是 `leg odometry`。该术语是比照轮式车辆的里程计而来，后者通过测量车轮转动量来推断行驶距离。

不过，与轮式平台不同，腿式机器人是通过不断建立和打破与地面的接触来实现运动的。每一步通常包含腾空的 swing phase，以及与地面保持非滑移接触的 stance phase。正因为真正推动机器人前进的是 stance legs，所以 `leg odometry` 本质上可以分解为两个子问题: 一是 contact estimation，即判断哪些腿当前处于支撑期；二是 motion estimation，即在 stance phase 内根据这些腿估计机体的增量运动。

### 12.1.1 历史背景

`Leg odometry` 的发展与“如何让机器真正走起来”这一问题紧密相连。腿式机器人研究可追溯到 20 世纪五六十年代，当时人们试图用步行机械克服轮式车辆在崎岖地形中的局限。一个著名例子是 `General Electric Walking Truck`，这是一台重达 1400 千克、由人类操作者控制的步行机器。随着控制理论发展，研究焦点逐渐转向更小尺度、能实现自动步行行为的平台。

到了 20 世纪 80 年代末，`Raibert` 的单足、双足和四足机器人展示了出色的运动能力，这极大推动了人们将腿式平台用于真实任务的兴趣。为了实现真正自主，必须解决 state estimation 和 odometry 问题。早期经典工作之一，是 `Roston` 与 `Krotkov` 利用腿运动学来估计 `Ambler` 六足机器人的运动。类似思路随后也被应用到 `RHex` 等其他多足平台上。

后续研究逐渐开始把腿式里程计与 `IMU`、视觉等传感器结合，例如通过粒子滤波器或 Kalman filter 进行融合。`Bloesch` 等人的工作为基于滤波的全地形运动学-惯性 odometry 奠定了基础，并进一步推动了四足和双足平台的发展。2012 到 2015 年的 `DARPA Robotics Challenge` 又把 whole-body state estimation 推向前台，要求系统不仅估计 `6-DoF` 位姿，还要估计机器人全身状态，并与 `LiDAR`、stereo vision 等外感知传感器联合工作。

近年随着商用四足和人形平台崛起，社区越来越关注更系统、更稳健的腿式状态估计方法。这不仅体现为针对四足和双足平台的 `Factor Graph` 估计，也体现为 invariant estimation、更加严格的动力学建模，以及在 `DARPA SubT` 这类极端场景中融合多种传感器的 resilient estimation 系统。

### 12.1.2 参考坐标系

腿式机器人中的参考系必须定义得非常清楚。书中使用的核心参考系包括: 固定在地面上的世界系 `Fw`，固定在机器人 floating base 上的机体系 `Fb`，以及附着在每个足端上的 foot frame，例如 `Ff1`、`Ff2` 等。为方便起见，作者假设 `IMU` 与 `Fb` 重合。

此外，每当某只脚与地面建立接触时，还会定义一个或多个临时惯性系 `Fk`。对 humanoid 来说，`Fk` 在 touchdown 时与 foot frame 完全重合；对 point-foot quadruped 而言，接触系通常只与落脚点位置一致，而不一定有足够的姿态信息。很多公式之所以成立，前提都在于这些参考系之间的关系被严格定义清楚。

`Figure 12.1` 给出了腿式机器人的参考系约定：世界系 `Fw` 固定在地球上，base frame `Fb` 附着在主机身上。为不失一般性，图中未单独画出 `IMU` frame，因为它可视为与 `Fb` 重合；当某只脚接触地面时，会定义一个 contact frame `Fk`，它固定于地面、垂直于地面，并与足端 frame `Ff2` 重合。

### 12.1.3 状态定义

严格来说，腿式机器人的状态应包括机体的位姿、速度以及关节状态。但本章默认关节状态由专门传感器直接测量，因此估计问题主要关注机器人 base 的位姿和速度。也正因为如此，作者几乎可以把 state estimation 与 odometry 交替使用，后者就相当于“不含 loop closure 的 `SLAM`”。

书中将机器人状态形式化为位置、姿态、线速度与角速度的组合。例如，世界系中的 base 位置和姿态分别记作 `t` 与 `R`，而线速度和角速度通常在机体系下表达。状态定义并非表面形式问题，它直接决定哪些变量会在滤波器或因子图中被显式估计，哪些量只是通过约束隐含存在。

### 12.1.4 腿式机器人运动学

腿式机器人的运动学描述，可以看作一个 main link，也就是 body / trunk / torso，再加上一条或多条连接其上的 kinematic chains，也就是各条腿。本章主要考虑工程上最常见的两类结构: 每条腿有 `6` 个主动自由度、配平足的 bipeds，以及每条腿有 `3` 个主动自由度、配 point feet 的 quadrupeds（见图 `12.2`）。作者还假设机器人本体是刚体，例如不存在 articulated spine，并忽略其他上肢结构，例如 arms。

`Figure 12.2` 展示了典型腿式机器人的运动链结构：quadrupeds 与 humanoids。

对 `leg odometry` 来说，我们最关心的是足端相对于机器人 body 的相对位姿或相对位置。设 `q ∈ R^N = [q_1, ..., q_N]^T` 表示具有 `N` 个主动自由度的 articulated robot 的 joint positions，即各个 revolute joints 的角位置。图 `12.1` 以及本章后续公式里，作者以 quadruped 和 biped 都取 `N = 12` 作为示例，但也明确指出，其他平台的 `N` 可以不同。Joint positions `q` 通常通过安装在各关节上的 rotary encoders 直接测得；也可以通过机器人运动学模型，结合安装在电机与关节传动之间的编码器读数间接恢复，例如通过测量液压缸行程再反算相应转角。

`q` 的时间导数 `qdot = [qdot_1, ..., qdot_N]^T` 就是 joint velocities。它们通常通过对 encoder readings 做数值微分得到；若读数噪声较大，也可借助安装在各 links 上的 `IMUs` 来辅助估计 joint velocities [1204]。

给定 `q`，对每一只脚 `f`，定义相应的 `forward kinematics` function `fk(q) : R^12 -> SE(3)` [716]，它把 joint positions 映射成该脚相对于机器人 base 的 pose:

```text
T_f^b = fk(q) = [ R_f(q)  p_f(q) ]
                [   0        1   ]                                 (12.2)
```

这里作者进一步定义两个方便的函数: `f_R : R^12 -> SO(3)`，表示足端在 base frame 中的 orientation；以及 `f_p : R^12 -> R^3`，表示足端在 base frame 中的 Cartesian position。对 point-foot quadrupeds 而言，`leg odometry` 通常只使用 `f_p(q)`，因为 point foot 可以在接触点绕转而不改变 joint positions，因此足端姿态本身并不可靠。

对式 `(12.2)` 做时间导数，就得到足端相对于机器人 base 的 Jacobian matrix `J(q) : R^12 -> R^(6×12)` [717]，它可用于把 joint velocities 映射成单只脚的线速度与角速度:

```text
[ v_f^b ]       [ J_v(q) ] qdot
[ ω_f^b ] = J(q) qdot = [ J_ω(q) ] qdot                           (12.3)
```

其中 `J_v(q) ∈ R^(3×12)` 和 `J_ω(q) ∈ R^(3×12)` 分别是 Jacobian 的线速度部分与角速度部分。由于 `q` 包含了所有 `N = 12` 个 joints 的位置，而每条腿在运动学上彼此独立，因此 `J(q)` 实际上是一个稀疏块矩阵，只有与当前那条腿对应的块是非零的。书中把这些非零块记作 `J̄(q)`，它们把某一条腿的关节角速度映射到该腿足端相对于 base 的线速度和角速度。

这些 forward kinematics 和 Jacobian expressions 构成了整章 `leg odometry` 的基础。只要 joint sensing 可用，并且后续接触假设成立，足端相对于 body 的几何约束就能被转化为 base motion 的估计信息。

### 12.1.5 腿式机器人动力学

光有运动学还不够。浮动基座 articulated-body system 的动力学通常可由 `Recursive Newton-Euler` 算法 [319] 写成两组耦合方程。第一组描述 floating base 本体的动力学，它具有 `6 DoF` 且是 underactuated；第二组描述通过主动关节连接到该 base 上的 `N` 个刚体部分，例如典型情形下 `N = 12`。两组运动方程可整体写成矩阵形式:

`M(q) [v_dot; ω_dot; q_ddot] + h(q, q_dot) = [J_b^T; J_q^T] f + [0_6; τ]` 。

其中，`M(q)` 是质量矩阵；它与由 floating-base 线加速度 `v_dot ∈ R^3`、floating-base 角加速度 `ω_dot ∈ R^3` 以及主动关节加速度 `q_ddot ∈ R^12` 组成的堆叠向量相乘。第二项 `h ∈ R^18` 是 bias term，用来汇集 `Coriolis`、离心力和重力效应。右侧最后一项描述广义扭矩，其中 base 的力矩部分为零，因为 base 不受直接驱动，而主动关节的扭矩则由 `τ ∈ R^12` 给出。

对 `leg odometry` 来说，右侧倒数第二项最重要，也就是接触力经 Jacobian 映射回广义力空间的部分。它的维度取决于机器人类型以及当前处于接触状态的腿数。设 `c` 为接触腿数，`d` 为单腿主动关节数，则四足机器人有 `d = 3`，人形机器人有 `d = 6`。此时，`J_b ∈ R^(dc×6)` 是把 base twist 映射到足端速度的 Jacobian，它依赖机器人 forward kinematics 以及 body 在惯性系 `F_w` 中的绝对姿态；`J_q ∈ R^(dc×12)` 则是式 `(12.3)` 中各条腿 Jacobian 的堆叠形式，其排列取决于机器人类型和接触腿集合。

例如，对双足 humanoid，若双脚都接触地面，则有

`J_q = [J_1(q); J_2(q)]` 。

而对 point-foot quadruped，由于腿可以绕接触点自由转动，所以 `J_q` 只使用线速度 Jacobian `J_v`。例如，若第一、第三和第四条腿处于接触状态，则

`J_q = [J_1,v(q); J_3,v(q); J_4,v(q)]` 。

最后一个需要定义的量是 `f ∈ R^(dc)`，它表示所有接触足端处的力和/或力矩集合。对四足机器人，如果四只脚都在地面上，则 `f` 就是四个线性接触力 `f_1, f_2, f_3, f_4` 的堆叠。对双足 humanoid，若双脚都接触地面，则 `f` 同时包含每只脚接触点处三个方向的线性力和力矩，也就是类似 `[f_1; τ_1; f_2; τ_2]` 的结构。

作者还指出，正如第 `12.3.4` 节将进一步说明的，这些接触力与力矩本身就可以被用来推断腿是否处于接触状态。也就是说，动力学不仅描述“机器人如何运动”，还为 `contact estimation` 提供了直接信息来源。

### 12.1.6 关节感知

`Leg odometry` 所依赖的主要本体感觉来自关节感知，也包括直接或间接的接触感知。作者依次介绍了最常见的几类硬件。

#### 12.1.6.1 旋转编码器

`Rotary encoder` 是把旋转轴的角位置转换成模拟或数字信号的机电装置。在腿式机器人中，它们使我们能够直接测量关节角度，从而确定机器人运动学；类似器件也会出现在其他部件上，例如机械式 `LiDAR` 就使用编码器来测量 beam array 的方位角。

编码器可以按多个维度分类，包括工作原理上的 optical 或 magnetic、读数方式上的 absolute 或 incremental，以及输出形式上的 analog 或 digital。书中指出，腿式机器人上最常见的是 absolute 与 relative optical digital encoders；其他类型可参考专门文献 [718]。

`Figure 12.3` 说明了两类光学编码器的工作原理。`(a)` 对 `8-bit optical absolute encoder` 而言，红外光束打到旋转圆盘上，圆盘以特定图案遮挡光线来编码转角；一组 photoresistors 将各扇区中光的有无转换为采用 `Gray code` 编码的二进制数，再转成表示绝对转角的十进制数。该示例编码器的分辨率为 `360 / 256 = 1.41` 度。`(b)` 对 optical incremental encoder 而言，两个 photoresistors `A` 与 `B` 之间有 `90` 度相移；当 `A` 上升沿后跟随 `B` 下降沿时，表示顺时针旋转，而 `AB = 11` 到 `AB = 10` 的状态变化会使计数器加一。

`Absolute optical digital encoders` 如图 12.3(a) 所示，能够以绝对方式测量关节角，也就是说，对同一关节构型它总会给出相同读数。其原理是: 红外光源，例如 `LED`，照向静止部分上一圈按径向布置的感光元件，例如 photoresistors；在光源与感光元件之间，有一张随轴旋转的圆盘。该圆盘被划分为若干同心扇区，每个扇区要么透明、要么不透明，并按照某种模式编码特定角度区间对应的二进制数。这个二进制序列通常按 `Gray code` 排列，使相邻自然数只差一个 bit，从而降低读数错误的概率。书中以 `8-bit encoder` 为例说明，其可能输出共有 `256` 个值，因此角分辨率为 `360 / 256 = 1.41` 度。编码器的角分辨率，本质上就由 bits 数，也就是同心扇区数决定。

`Incremental optical encoders` 如图 12.3(b) 所示，则是相对测量设备。它们上电时把当前位置视作零位，之后只测量相对于该参考点的角度变化。它们不依赖 `Gray code`，而是使用更简单的码盘: 在不透明圆盘上沿半径方向规则开槽，使单个光敏电阻 `A` 在圆盘匀速旋转时输出方波。另一个光敏电阻 `B` 相对 `A` 具有 `90` 度相移，因此两路信号构成的 `2-bit` 量 `AB` 在任意时刻有四种可能取值。系统通过这些状态转移来判断旋转方向，以及应增加还是减少计数 [718]。

由于结构更简单且成本更低，高分辨率 incremental encoders 曾经常与低分辨率 absolute encoders 配合使用：先由 absolute encoder 测得初始角，再由 incremental encoder 持续累积后续变化 [986]。不过，随着 absolute encoders 的分辨率不断提升、成本不断下降，它们正在快速替代 incremental encoders。对 `leg odometry` 而言，encoder 最直接提供的是 joint positions，而 joint velocities 通常还需要对这些角度读数进一步做数值微分。

#### 12.1.6.2 力与力矩传感器

`Force / torque sensors` 是把作用在表面某点上的线性力，或施加在轴上的机械扭矩，转换成电信号的装置。它们主要用于执行器扭矩控制，或者感知末端执行器与环境之间的相互作用。在腿式 locomotion 中，每一步都会有地面作用力产生，因此这些力必须被直接或间接测量，才能识别哪些腿正处于 stance phase。

对力和力矩两种量来说，其工作原理通常相同，只是几何结构不同。传感器内部的表面会被设计成在待测方向上受力时产生轻微形变；在这些表面上贴有一种柔性的可变电阻元件，即 `strain gauge`。`Strain gauge` 的电阻会随变形量成比例变化，其中压缩会降低电阻率，而拉伸会提高电阻率。图 12.4 左侧给出了在线性力测量 `load cell` 上使用 `strain gauge` 的例子 [718]；若要测量 torque，则通常会把一组 `strain gauges` 安装在连接电机轴的柔性轮辐结构上。

`Figure 12.4` 展示了 force / torque sensors 的工作原理。`Strain gauge` 会被贴在受载时可压缩或可弯曲的结构元件上：在左侧的 load cell 内，压缩会表现为电阻率下降；在右侧的 torque sensor 内，多个 `strain gauges` 贴在轮辐式柔性元件上，施加扭矩后这些元件会弯曲。通过一侧压缩、另一侧拉伸等组合信号，系统可以反推出扭矩的大小和方向。

若希望同时测量末端执行器上所有方向的力与力矩，则可使用 `6-axis sensors`。这类传感器内部会布置足够数量和方向组合的 `strain gauges`，从而测出各个方向上的 force 与 torque。它们在 manipulation 中很常见，在 humanoid 足底上也会使用，以直接测量足底与地面的相互作用。

作者还指出，随着可回驱电机（`backdriveable motor`）设计推动低成本动态腿式机器人发展 [543]，很多系统也会直接利用 motor currents 来估计 torque，因为在一定条件下电机电流与输出扭矩成比例。

#### 12.1.6.3 接触传感器

由于 `force / torque sensors` 在 `leg odometry` 中最主要的用途，往往只是判断哪些腿处于支撑状态，因此更低成本的替代方案就是 `contact sensors`。它们输出一个二值量，用来表示某只脚当前是否与地面接触。这类传感器最初主要是为中小型 quadruped 设计的，因为这类机器人常使用球形或圆形橡胶足底。

最常见的 contact sensors 分为 optical 与 mechanical 两类。`Optical contact sensors` 在概念上与 encoder 相似：它们使用一个 `LED-photodiode` 组合，通过一个小孔感知光线。当足端与地面接触时，足底表面会发生足够大的形变，带动一个遮挡板位移并挡住该孔，从而让系统检测到接触。`Mechanical contact sensors` 则更简单，它们通常在足底内部隐藏一个 pushbutton switch，当足端受力足够大时按下开关，从而触发接触信号。

作者同时强调了 contact sensors 的主要缺点。首先，相比代价更高的 `force / torque sensors`，它们的响应时间通常较慢。其次，它们也有与 `F/T sensors` 相似的工程问题：需要把线缆一路布到足端，而且必须承受机器人着地时最直接的冲击，因此较易损坏。正因如此，这类传感器大多只出现在面向室内运行的小型 quadrupeds 上。

## 12.2 运动估计

在已知关节状态和机器人运动学之后，下一步就是估计机器人 base 的增量运动。作者指出，这可以主要通过两条路线实现: 一条是基于 forward kinematics 的相对位姿估计；另一条是基于 differential kinematics 的速度估计。两种方法共同的核心假设都是: 新建立的接触点在一段时间内保持静止。

### 12.2.1 相对位姿估计

作者先用一个沿 `zx` 平面行走的 humanoid 简化例子说明原理。设机器人 base 在相邻两个时刻对应的参考系分别为 `F_b` 与 `F_b'`，足端参考系和关节位置也在这两个时刻相应定义。若接触参考系 `F_k` 在 stance phase 内保持静止，则当足端系与接触系重合时，机器人机体向前移动的位移，就等价于足端相对机体“向后”移动的位移。因此有

`Figure 12.5` 说明了理想接触条件下 leg odometry 的工作方式。左图中若假设 contact frame `Fk` 刚性附着于地面（黄色），就可以据此推断机器人机体的相对运动；右图则等价地从两个相邻时刻出发，表示腿相对于 body frame（黄色）的运动。

```text
T_b'^b = T_k^b (T_k^b')^{-1} = (T_f'^k)^{-1} = (T_f'^b)^{-1} T_f^b = fk(q')^{-1} fk(q)
```

这条式子把 joint states 与机器人相对位姿直接联系起来。若把这些相对位姿沿时间串接起来，理论上就能仅凭 joint sensing 做 dead reckoning [946]。但它只能在 stance phase 中成立；并且对 point-foot 机器人，要仅靠这种方式恢复 `6-DoF` 运动，通常需要始终至少三只脚同时接触地面，因此对 quadruped 并不现实。

为解决这些问题，quadruped 的标准做法 [88] 是在原状态基础上再加入每条腿对应接触点在世界系中的位置 `c_i = t_k^w ∈ R^3`。对 humanoid，由于有 ankle 结构，还可进一步把接触点姿态 `B_i = R_k^w ∈ SO(3)` 一并纳入状态 [947]。另外，考虑到任意时刻并不能保证总有足够多的腿在接触地面，例如 gallop gait 中会出现全部离地阶段，因此系统通常还会配备 `IMU`，把角速度测量交给 `IMU`，并把 `IMU biases` 一并放入状态。

在这些考虑下，书中分别给出 quadruped 与 humanoid 的典型状态形式。quadruped 状态包含 `t_k, R_k, v_k, b_k^a, b_k^ω` 以及四个接触点位置 `c_1, c_2, c_3, c_4`；humanoid 状态则包含 `t_k, R_k, v_k, b_k^a, b_k^ω`，外加两个接触位置 `c_1, c_2` 与两个接触姿态 `B_1, B_2`。

基于这些状态，对任意一条接触腿 `i`，相对位姿关系可以更精确地写成

```text
T_k^b = fk(q) = T^{-1} C_i
```

其中

```text
T = [R t; 0 1]
```

是 base 在固定参考系中的位姿，而

```text
C_i = [B_i c_i; 0 1]
```

是该足端接触在固定参考系中的位姿。将其展开后可得

```text
T^{-1} C_i = [R^T B_i   R^T c_i - R^T t; 0 1]
```

由此可定义右上角两个上层 block 对应的核心量:

```text
f_p(q) = R^T (c_i - t)
f_R(q) = R^T B_i
```

其中 `f_p(q)` 表示固定系中足端相对 base 的位置变化，`f_R(q)` 表示相对姿态变化，且它们都被写成 joint angles 的函数。书中也特别提醒，对 quadruped 只能使用位置项 `f_p(q)`，因为 point feet 无法提供足端姿态估计。

式 `(12.17)` 与 `(12.18)` 正是后续滤波与因子图框架中使用的基本 leg odometry 测量表达。

### 12.2.2 速度估计

另一条常见路线，是直接利用式 `(12.3)` 中的 differential kinematics，从每条 stance leg 获得速度测量。这种做法在 quadruped 上被广泛采用 [89, 143, 577]，在 bipeds 中相对少见 [1084]。它的优点是速度测量很容易并入 filter 或 factor 中，不需要保留历史以避免位置误差累积 [315]，也不必显式维护额外的接触位置或接触姿态状态。缺点则是 joint velocities 通常由 joint positions 做数值微分得到，因此可能因微分和舍入误差而降低估计性能。

与相对位姿估计类似，作者先写出刚性接触条件下足端必须满足的速度关系。若某条腿处于稳定接触，则接触点 `k` 在固定系中的速度必须为零:

```text
v_k^w = 0
```

同时，接触点 `k` 的速度必须与足端 `f` 的速度一致，而该速度又能由 Jacobian 写成

```text
v_k^b = v_f^b = J_v(q) qdot
```

由于角速度可由 `IMU` 直接测得，这里作者只关心线速度。把上面两条关系合并，并考虑 base 与足端之间的 lever arm 后，可得

```text
v_k^w = v_b^w + ω_b^w × t_k^b + v_k^b
0 = v_b^w + ω_b^w × f_p(q) + J_v(q) qdot
v_b^w = - ω_b^w × f_p(q) - J_v(q) qdot
```

这里第一项 `- ω_b^w × f_p(q)` 描述的是 base 到足端 lever arm 引入的附加线速度 [270]，第二项则来自腿相对于 base 的微分运动。式 `(12.21)` 因而把机器人绝对速度与 forward kinematics、differential kinematics 以及 `IMU` 角速度联系起来，可以直接作为 measurement update，或在图优化里作为 factor 使用。

书中还补充说，若按照惯例需要在 body coordinates 中表达速度，只需使用机器人姿态 `R = R_b^w` 做相应坐标变换即可。

## 12.3 接触估计

由于 `leg odometry` 的全部前提都是“当前某只脚在可靠支撑地面”，所以 contact estimation 是整个问题的核心。不过，这里的“接触”含义会随应用而变化。比如在协作机器人里，末端执行器只要“碰到”某个物体，也就是受到了不可忽略的外力，就可以视为在接触；而对 `leg odometry` 来说，一只脚只有在接触点能在一段时间内保持静止时，才可视为真正处于接触。对 point-foot 平台而言，这进一步意味着接触点不能发生滑移。

`Figure 12.6` 说明了 quadruped 腿部的接触点模型：当足端施加的力 `f = [f_x, f_y, f_z]^T` 保持在 friction cone 内时，可认为该腿处于 stance。

形式上，对 quadruped 一类 point-foot 机器人，若地面反作用力 `f = [f_x, f_y, f_z]^T` 满足 friction cone 约束，就可以认为足端没有打滑:

```text
sqrt(fx^2 + fy^2) <= μx,y fz
```

其中 `f_x` 与 `f_y` 是相对于接触平面的切向地面反作用力分量，它们依赖局部地形形态；`μ_x,y` 是摩擦系数，取决于地面与足端材料性质。

对具有平足的 humanoid，还会多出一个关于接触转矩的条件，因为足底不能在地面上发生转动。书中给出了与 `center of pressure (CoP)` 和绕法向轴力矩相关的不等式约束，也就是要求 `CoP` 落在支撑多边形范围内，同时绕法向的接触转矩不能超过相应摩擦极限。

作者随后指出，只要法向力 `f_z` 足够大，式 `(12.22)` 到 `(12.24)` 往往就更容易满足。因此，在实际实现中最常见的接触估计方法仍然是简单地对 `f_z` 做阈值化。不同系统之间真正的差别，更多在于这个力是如何被测量或估计出来的，例如通过 contact sensors、`force / torque sensors`、joint sensing 或 `IMUs`，以及机器人本体结构的具体特性。

### 12.3.1 基于接触传感器

最直接的做法，是依赖专门 contact sensors。它们本质上在硬件层面对法向力 `fz` 做了阈值化处理，当测量力超过某个名义阈值时，输出接触为真。因此，使用这类传感器时，`leg odometry` 可以直接以该二值状态为依据。

其优点是实现简单、逻辑清晰；缺点是只提供离散接触信息，无法表达受力质量，也难以区分“刚刚触地但尚未稳定支撑”和“即将离地或接触不可靠”这类更细状态。

### 12.3.2 基于力/力矩传感器

若足端安装了力 / 力矩传感器，则法向力 `fz` 可随时间连续测得。这样一来，就不再只是简单地做二值判断，而可以根据受力模式推断更细粒度的状态。例如，一个持续增长但仍较小的力，可能意味着足端刚开始撞击地面，但尚未进入稳定支撑期；反之，若法向力下降到某阈值以下，则意味着脚很快就要失去可靠接触。

在这两种情况下，来自该条腿的里程计信息都应被丢弃，或至少增加其测量不确定性。因此，力 / 力矩传感器不仅能判断“是否接触”，还能帮助区分“接触质量是否足够好”。

### 12.3.3 基于惯性测量单元（IMUs）

除了装在机体上的主 `IMU`，有些系统还会在腿链或足端再加装小型 `IMU`。由于 touchdown 等接触事件会在足端引起明显加速度变化，因此这类局部惯性信号可被用来隐式检测支撑腿。

这种方案的优点，是传感器本身不直接承受足底冲击，因此相比嵌入足底的接触或力传感器更不容易损坏；代价则是需要额外的信号处理来从振动与冲击中可靠分离出接触信息。

### 12.3.4 基于关节力矩感知

若机器人设计和集成条件限制，不方便在足端额外安装传感器，则还可通过 joint torque sensing 反推出接触状态。其核心思想是利用动力学模型，把地面反作用力映射回关节扭矩空间。

以 quadruped 为例，书中利用 `J` 的分块结构，从式 `(12.5)` 推得末端执行器处的力可写为

```text
f_i = -(J̄_i,v^T)^(-1) (τ_i - h_q,i - F̄_i vdot)
```

这就是式 `(12.25)`。其中 `f_i ∈ R^3` 和 `τ_i ∈ R^3` 分别是第 `i` 条腿的 `ground reaction force (GRF)` 与关节扭矩；`J̄_i,v` 是足端 Jacobian `J_v` 中第 `i` 个非零分块，在 quadruped 情形下它通常是方阵；`F̄_i ∈ R^(3×3)` 是质量矩阵中的对应分块；`h_i,q ∈ R^3` 则表示该腿的离心 / Coriolis / 重力扭矩项。

需要注意的是，这样得到的 `f_i` 只能在 base 坐标系中被估计。若要恢复真实的 GRF，还缺少两类关键信息:

- 局部地形的倾角。接触力方向取决于接触面的朝向；对 humanoid 而言，踝关节通常可以给出较好的近似，而对 quadruped 来说，若没有 exteroceptive sensing，接触坐标系朝向通常无法直接确定，只能借助其他已接触足端做启发式推断，例如对这些接触点拟合一个平面 [329]。
- 摩擦系数。力的水平分量依赖摩擦系数，而摩擦系数又取决于机器人当前踩踏材料的性质，因此通常只能事先已知，或依据机器人正在经历的滑移程度来间接推断 [513]。

因此，单靠 joint sensing 来建立接触状态在总体上仍是一个开放问题。已有方法通常会以概率方式联合运动学与动力学信息 [497]，或进一步引入学习方法（见第 `12.6` 节）来完成接触检测。

## 12.4 将腿式里程计用于状态估计

有了 leg odometry 测量之后，下一步就是把它们纳入完整的 state estimation framework，与 `IMU`、camera、`LiDAR` 等其他模态共同工作。历史上，腿式状态估计多以滤波方法为主，因为 locomotion control 对高频、低延迟状态估计要求极高；但近几年因子图和平滑方法也越来越常见，特别是在与建图和全局估计结合时。

无论采用滤波还是平滑，要想最优融合 `leg odometry` 与其他传感器，都必须先量化其不确定性。因此作者首先讨论 encoder noise propagation，随后再介绍因子图中的接触预积分、速度预积分及其与外感知的融合。

### 12.4.1 编码器噪声传播

`Encoder noise` 是腿式里程计最直接的误差来源。若把真实 joint positions 记为 `q`，测量值记为 `q̃`，则可写成

```text
q̃ = q + ηq
```

其中 `ηq` 通常被建模为零均值高斯噪声。但因为 forward kinematics 和 differential kinematics 都涉及旋转，因此它们是非线性的，高斯噪声经过非线性映射后并不会严格保持高斯。

因此，作者采用与前面章节类似的一阶线性化思路: 在当前测量附近使用 Jacobian 展开 kinematics 函数，把 joint-space 的噪声线性传播到足端相对位姿或速度测量上。对 forward kinematics，可写成

`f_p(q + η_q) ≈ f_p(q) + J_c(q) η_q`  。`(12.27)`

其中 `J_c(q)` 是 body manipulator Jacobian，也就是通常的 manipulator Jacobian `J(q)` 在 contact frame 中的表达形式。

类似地，对受 encoder noise `η_q` 与 encoder velocity noise `η_qdot` 共同影响的速度测量，有

`J(q + η_q) (qdot + η_qdot) ≈ J(q) qdot + [∂(J(q) qdot)/∂q] η_q + J(q) η_qdot`  。`(12.28)`

从式 `(12.27)` 与 `(12.28)` 可以看出，所有乘在噪声项前的附加项都只是给定 encoder measurement 下的线性组合，因此可以重新归并为一个单独的高斯项。这样就能在近似高斯框架下为后续滤波器或因子图构造合理的测量协方差。

### 12.4.2 Factor Graph（因子图）平滑

为了生成 locomotion behaviors，同时避免跌倒等灾难性失败，腿式机器人对实时控制和高频状态估计有非常严格的要求。历史上，这些需求通常通过 `EKF`、`UKF` 或 invariant `EKF` 等非线性 Kalman Filter 来满足。这些滤波器一般融合高频传感器数据，如惯性与运动学信息，为控制器供给状态；而外感知传感器更新有时也会纳入控制环路，但更多仍被分配给 mapping 与 planning。

Kalman filtering 方法的一个局限是，它要求显式区分过程模型与测量模型；当缺少合适过程模型时，常常只能退化为 constant velocity model，或更常见地使用 `IMU` propagation。相比之下，`Factor Graph` 方法更加一般，因为它把过程模型和测量模型统一看作“状态与测量之间的关系”。

腿式系统里的 `Factor Graph` 方法主要在估计状态数量与时间范围上有所不同。当图中只包含两个相邻状态时，因子图看起来很像滤波器；`Two-State Implicit Filter (TSIF)` [92] 就是一个例子。当窗口增大后，图不仅能估计当前状态，还能修正窗口内的一段历史状态。状态采样频率则是一个系统设计问题: 若以高频（例如 `IMU` 频率）加入更多状态，估计器设计会更直接，但为了维持有界计算量，就不得不减小时间窗口；相反，若希望在固定状态数下覆盖更长时间，则可像第 11 章里的 `IMU preintegration` 那样对测量做预积分。

原书接着给出两个相关文献中的例子，用来展示 preintegration theory 与前面介绍的 leg odometry 概念如何进入 factor graph estimation framework。重点分别是 bipeds 的 `contact preintegration` [440] 和 quadrupeds 的 `velocity bias preintegration` [1190]。在这两种情形下，第 `12.2` 节中的测量都会被改写成图中因子的 residual 与 covariance。Figure `12.7` 展示了两类典型因子图结构: 上图是带 preintegrated contacts 的图，下图是带 preintegrated velocity 的图；其他约束两个状态的外感知测量，也都可以很自然地再作为额外因子加入图中。

`Figure 12.7` 给出了两类预积分因子图结构：上图是 preintegrated contact 的因子图，下图是 preintegrated velocity 的因子图。其他外感知测量，例如那些约束两个状态的传感器观测，也可以很容易作为额外因子（洋红色）加入其中。

#### 12.4.2.1 接触预积分

接触预积分的想法与 `IMU preintegration` 非常相似。对于 humanoid，如果某条支撑腿在一段时间内持续保持 stance，则其相对接触运动增量可以被压缩成一个连接两个状态的因子。作者引用 Hartley 等人的做法，将其写成一个包含 prior factor、预积分 `IMU factor`、forward kinematics factor 和 contact preintegration factor 的因子图，所提出的图结构见 Figure `12.7a`。其中 prior factor（黑色）用于锚定整张图，预积分 `IMU factor`（橙色）引入来自 `IMU` 的 motion prior；对 humanoid 还额外加入 `forward kinematics factor`（绿色）来约束足端 contact frames 的姿态，而 `contact preintegration factor`（蓝色）则编码两条腿接触状态之间的相对运动。

其中，`forward kinematics factor` 用于把足端接触系姿态与机器人 base 姿态通过支撑腿的 forward kinematics 联系起来。把上一节推导的 encoder noise propagation 代入相对位姿测量 `(12.17)` 和 `(12.18)` 后，可得该因子的 residual 与 covariance:

`r_F = Log(C_i^(-1) T f_k(q̃))`  。`(12.29)`

`Σ_F = J_c(q) Σ_q J_c^T(q)`  。`(12.30)`

这类因子的本质约束是“机器人 base 与接触坐标系之间的相对位姿，应与由关节读数推得的运动学结果一致”，而不确定性则来自关节编码器噪声经 Jacobian 传播后的结果。

`Contact preintegration factor` 则进一步约束两个时刻之间接触点的演化。理想情况下，若 stance leg 没有 slip，则接触框架的旋转和位置在支撑阶段应保持不变；实际中，接触点速度会受到 slip 等效应影响，因此作者将其建模为高斯噪声。对于相邻两个状态 `x_i` 和 `x_j`，有

`ΔR̃_ij = B_i^T B_j Exp(δθ_ij) = I`  。`(12.31)`

`Δc̃_ij = B_i^T (c_j - c_i) + δd_ij = 0`  。`(12.32)`

这里使用的是式 `(12.15)` 中 contact frames `C_i` 与 `C_j` 的旋转与平移分量。其中 `δθ_ij` 与 `δd_ij` 是预积分接触噪声项，用来将接触点速度的不确定性建模为零均值高斯变量。于是可得接触预积分因子的 residual 与 covariance:

`r_C = [ Log(B_i^T B_j) ; B_i^T (c_j - c_i) ]`  。`(12.33)`

`Σ_C = [ Σ_w Δt_ij   0 ; 0   Σ_v Δt_ij ]`  。`(12.34)`

这里 `Σ_C` 由接触角速度协方差 `Σ_w` 与线速度协方差 `Σ_v` 在时间区间 `Δt_ij` 上积分得到。原书也明确指出，这一因子只在同一条 stance leg 且接触模式不切换时成立，因此它无法直接覆盖腿切换动力学；后续工作才针对 contact frame switch 做了扩展。

#### 12.4.2.2 速度预积分

对 point-foot quadruped 而言，仅靠 forward kinematics 和接触预积分往往不足以完整约束相对 `6-DoF` 运动，因此作者介绍了另一类速度预积分因子。其核心思想，是把上一节由 `leg odometry` 推出的 base linear velocity 当作高频测量，对其进行预积分，从而得到约束两个状态之间相对平移变化的因子。

书中给出了一条含噪速度模型:

```text
ṽ = - Jv(q) qdot - ω × fp(q) + ηv
```

也就是

`ṽ = - J_v(q) qdot - ω × f_p(q) + η_v`  。`(12.35)`

若再假设在 `t_i` 到 `t_j` 之间 base 线速度近似常值，就能把这些速度测量预积分为一个相对位移量:

`Δt̃_ij = Δt_ij + δp_ij = Σ_(k=i)^(j-1) ΔR̃_ik ṽ_k Δt + δp_ij`  。`(12.36)`

其中 `δp_ij` 是预积分速度噪声项。

在此基础上，就能构造 `velocity preintegration factor`。它的残差与协方差为

`r_V = R_i^T (t_j - t_i) - Δt_ij`  。`(12.37)`

`Σ_(V,ij) = Σ_(k=i)^(j-1) Σ_(V,ik) + A Σ_v A^T`  。`(12.38)`

其中矩阵 `A = ΔR̃_ik Δt`。因此，这个因子比较的是: 当前两状态间的相对平移 `R_i^T(t_j - t_i)`，是否与由腿式里程计速度预积分得到的 `Δt_ij` 一致；其协方差则由各小时间段速度噪声传播并叠加得到。这样，速度预积分因子便可与 `IMU` 因子一同进入图中，为四足平台提供更直接的平移约束。

#### 12.4.2.3 处理多重测量

前面讨论时往往默认每次只利用一条腿的测量，但实际中多条腿常常同时处于接触状态。这既带来冗余和鲁棒性，也带来不一致风险: 不同腿若提供互相矛盾的信息，系统该信哪一条？

书中指出，最简单的一种做法，是像 [315] 那样只挑选当前被认为最可靠的那条腿，把其他腿的信息全部丢弃。另一个很自然的思路，则是把每条腿及其 measurements 都独立处理；当 contact poses（或 contact positions）本身显式成为状态的一部分时，这样做尤其直接 [440, 577]。

如果 contact poses 并未显式进入状态，那么所有处于 stance 的腿在同一时刻获得的 velocity measurements 仍然可以被视为相互独立；不过，为了减轻 filter 或 factor graph 的计算负担，实践中往往更倾向于先把这些速度测量平均成一个单独 measurement 再使用 [1190]。因此，多腿同时接触时的关键系统设计问题，就是如何在 redundancy、robustness 与 computational load 之间做平衡。

### 12.4.3 与外感知传感器集成用于 SLAM

正如前文所示，`leg odometry` 的主要价值并不在于独自完成长期定位，而在于为更大系统提供低漂移、短时可靠的运动先验。这样一来，构建在其上的 `SLAM` 系统可以获得更平滑的相邻节点约束，从而在回环闭合时减少大幅度突变修正。

与 `LiDAR`、camera 这样的外感知传感器集成，在滤波框架和因子图框架中都非常自然: 前者增加新的测量更新，后者增加新的因子即可。真正困难的地方在于，多源传感器工作频率不同、噪声特性不同、失效模式也不同。

如果某个模态违反了零均值高斯假设，或干脆完全失效，那么滤波器或因子图都可能被拖垮。为此，很多系统采用 loosely coupled 结构，让 `Visual-Inertial`、`Legged-Inertial`、`LiDAR-Inertial` 等子系统并行运行，再对输出结果做 triage 和选择。另一类 tightly coupled 方法则把所有因子直接放入同一图中，并通过在因子层面做 triage、跳过失效传感器，或使用 robust cost function 来提升系统整体稳健性。

## 12.5 开放挑战

尽管 `leg odometry` 已经相当有潜力，但它依赖的许多关键假设在现实中经常被打破，例如 rigid body、rigid contact、no slip。作者据此总结了若干仍未彻底解决的开放问题。

### 12.5.1 腿部形变

本章中推导的 `leg odometry` 方程几乎都默认机器人是完美刚体。但在现实里，这一假设经常不成立。一旦腿部在受力时发生弹性弯曲，`forward kinematics` 计算出来的就只是末端执行器的理想位置，而不是其真实位置，因此 `leg odometry` measurements 会带上系统性偏差。类似地，即便接触点本身不动，只要施加在腿上的力使其弯曲，joint angles 也会发生变化。图 `12.8` 直观地展示了这一点: 虽然真实的 base 到接触点变换没有发生系统所期望的变化，但在刚腿假设下，forward kinematics 会把这一现象错误解释成 base 向上运动。

`Figure 12.8` 展示了 quadruped 上的腿部形变示例。左图是真实的 robot base 到 contact point 的变换；由于 forward kinematics 假设腿是刚性的，它会像右图所示那样，把这一现象错误地估计为向上运动。

当这种问题只在很短时间段内出现时，一种曾经使用过的策略 [143] 是分析 force profile，在检测到冲击或异常载荷的那些时间段内直接拒绝对应 measurements。

作者还指出，对 bipeds 来说，由于腿通常更长，leg flexibility 问题往往会更严重。一种理论上可行的路线，是利用 leg load 与 flexibility 的相关性来建模，即腿承受负载越大，弯曲也越明显。若能非常细致地建模机器人结构几何与材料属性，就有可能把 bending properties 显式纳入估计。然而，这种方法通常十分复杂，而且很难泛化到其他平台。

因此，实践中更常见的路线，是在腿部各个 links 上额外集成 `IMU`，利用这些额外的本体感觉信号去估计真实的 link orientations，再与 joint readings 结合起来修正纯运动学解 [1127]。这也是目前处理腿部形变最常见的思路。

### 12.5.2 非刚性接触与打滑

若机器人踩在松软或可塌陷地面上，足端可能在接触状态不变时继续向下陷入地面；若地面柔顺或摩擦不足，还可能出现显著 slip。此时，“接触点静止”这一假设就被破坏了。即便从估计现象上看，这和腿部形变带来的误差很相似，其根本原因却不同: 这次不是机器人腿在变，而是接触点本身在运动。

`Figure 12.9` 展示了 quadruped 上的 ground deformation 示例。机器人最初接触的是平坦地面；在保持接触状态的同时，地面发生形变，足端向下陷入其中（左图）。随着足端下沉、joint angles 变化，这一过程会被系统解释成相对于最初 touchdown 点的向上运动（右图）。

在这种情况下，单靠 `leg odometry` 往往不够。系统需要额外的外感知传感器，例如 camera，使接触点速度重新变得可观。相关工作可将 contact velocity 作为显式状态估计，或把足端位置的时间导数一并纳入模型。

## 12.6 延伸阅读与最新趋势

作者最后总结了几类非常值得关注的趋势。第一类是 `learning-based contact estimation`。由于 rigid contact 假设极易被打破，而精确建模又非常困难，因此越来越多方法开始使用数据驱动方式学习 contact state。有的工作通过监督学习训练 contact classifier；有的通过聚类等无监督方法，仅依赖 joint sensing 和 inertial sensing 来恢复接触信号。随着深度学习发展，也出现了使用神经网络直接判断足端接触状态的方法，它们在结构化与非结构化环境中都表现出更好的泛化能力。另一方面，借助视觉和触觉融合的新型 haptic sensor，也可能进一步提升 force 和 contact estimation。

第二类趋势是 `end-to-end learning`。传统高频 `leg odometry` 和 proprioceptive state estimation，原本是为了支持闭环 model-based locomotion control 而发展出来的。但强化学习在 locomotion 控制上的成功，开始挑战某些传统模块的必要性。有些 `RL` 控制器只需要 base orientation 相对重力的方向，以及机体瞬时速度，就能实现稳定行走；更进一步的工作甚至直接从 joint 与 inertial 原始数据中学习这些信息。这并不意味着 state estimation 已经不再需要，而是说明对于某些 locomotion 目标，估计器的角色和接口可能正在变化。

作者指出，若目标是更复杂的导航行为，例如跨越障碍、面向目标位置前进甚至 robot parkour，那么系统仍然需要可持续的 odometry 估计，因为机器人必须知道自己相对于起点或目标的位置变化。已有少数工作尝试将 state estimation 一并纳入 locomotion policy 的学习过程，直接预测 body velocity、feet height 和 contact state 等变量。虽然这类方法目前主要服务于 locomotion，本质上却也可能为未来 `SLAM` 中的 proprioceptive odometry 提供新思路。

第三个重要方向是 humanoid robots。人形机器人之所以长期吸引人，是因为它们天然适合人类环境、工具和交通工具。但要让 humanoid 真正承担配送、仓储和制造等复杂任务，就必须解决 whole-body estimation、长期可靠定位、间歇性多部位接触、柔顺安全交互等一系列远比四足 locomotion 更复杂的问题。脚、手、手臂甚至躯干都可能成为间歇接触点，而接触也往往不再满足本章中反复使用的 rigid contact 假设。

换言之，`leg odometry for SLAM` 远远还没有“做完”。它正在从支撑四足 locomotion 的一个子模块，逐渐扩展为面向更一般腿式平台，尤其是 humanoid 和复杂操作机器人，全身状态估计与 `SLAM` 的关键组成部分。
