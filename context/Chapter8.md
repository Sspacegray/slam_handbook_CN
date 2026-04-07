# 第 8 章 LiDAR SLAM

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

作者在这里还举例说明：`RING++` [1211] 使用的是一个 `120 × 120` 像素的表示，并把像素均匀铺设在 `[-80, 80]` 米的网格上。

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

