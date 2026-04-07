## 第 8 章 LiDAR SLAM

### 本章概览

本章转向 `LiDAR SLAM`。与视觉不同，`LiDAR` 可以直接提供场景的 range information，因此几何约束通常更直接、更稳定。但这并不意味着问题更简单。`LiDAR SLAM` 仍需要处理 scan registration、motion distortion、place recognition、loop closure、backend `PGO`、map update，以及更大规模的 multi-session / multi-robot 扩展。

作者本章的组织方式与 visual `SLAM` 相似: 先介绍 `LiDAR` 感知基础与分类，再讨论 odometry、place recognition、完整系统结构、后端 pose-graph optimization、地图更新和多机器人扩展，最后总结鲁棒性、多传感器融合与长期地图管理等当前趋势。

## 8.1 LiDAR 感知基础与分类

通过在 `LiDAR` 传感器内部让激光发射器与探测器绕一个或两个轴旋转，就能够逐步构建出围绕传感器的环境点云。最基础的测距原理是 `time-of-flight (TOF)`，即通过测量发射光脉冲返回到探测器所需的时间来推断距离。`TOF LiDAR` 能获得高分辨率测量，但它对外界光照较为敏感，这会降低 `signal-to-noise ratio (SNR)`，进而影响测量精度与频率 [597]。除了 `TOF` 之外，最早为 radar 传感器发展的 `Amplitude Modulated Continuous Wave (AMCW)` 和 `Frequency Modulated Continuous Wave (FMCW)` 技术，如今也已被引入 `LiDAR`。

作者随后按 sensing mechanism 对 `LiDAR` 分类 [934]，包括 `mechanical LiDAR`、`scanning solid-state LiDAR`、`flash LiDAR`，以及使用宏观扫描 `Risley prisms` 的传感器。本章重点讨论其中两类最常用的设备: `2D/3D mechanical LiDAR` 与 `macroscopic scanning LiDAR`。

`Mechanical LiDAR` 是目前最常见的一类。它利用旋转机构来引导激光束方向，但也因此会受到机械磨损和数据采集速率较低的限制。最简单的 `2D mechanical LiDAR` 使用一个旋转镜面来偏转单束激光，并同时测量距离，如图 8.1(a) 所示。通过编码盘 `encoder disk` 可以测得激光束的角度，再与距离观测配对，形成一个 `2D` profile measurement。由于其工作原理所限，这类设备一次只能扫描一个独立的二维平面。

对“单一传感器实现完整 `3D` 扫描”的需求，很大程度上来自 `2000` 年代的 `DARPA Grand Challenges` 自动驾驶竞赛，并推动了先驱性的 `Velodyne HDL-64E` 传感器出现。它曾被当时多数参赛队伍使用 [1114, 771, 535]。在 `3D mechanical LiDAR` 中，多束激光发射器安装在同一旋转机构上，各自对应不同的俯仰角；整个机构再沿方位角方向旋转 `360` 度，如图 8.1(b) 所示。由此得到的点云能够对目标与周围环境形成高细节的 `3D` 表示，如图 8.2 所示，这一点已在多项研究中得到讨论 [1177, 556, 934]。随后，该技术继续演进，`LiDAR` 的体积与价格都显著下降。

近年来，基于更多物理机制的原型传感器也不断出现，包括 `solid-state LiDAR`、`Risley prisms` 与 `polygonal mirrors`。与传统扫描式激光不同，这些传感器中很多使用 `Micro-electromechanical Systems (MEMS)` 镜面技术 [467] 或 `optical phased arrays (OPA)` [446] 来避免或至少减少机械旋转。这一点很重要，因为主动驱动的机械部件更少，往往意味着更长的寿命和更高的环境建图可靠性。

其中一个值得注意的进展是 `Risley prisms` [667]。它们能够在极小物理运动量下实现快速、可控的 beam steering，因此传感器可以做得更紧凑，不过当前通常仍伴随更受限的 `field of view (FOV)`。

上述所有传感器都会输出一组单独的距离测量，并为每个测量附带一个 intensity 值，也常称为 `remission`，表示有多少 `LiDAR` 光束被反射回来。结合各束光的方向角信息，这些距离观测就可以转换为 `2D` 或 `3D` 点云。

不过，较新的 `LiDAR` 技术已经不再局限于默认的 range measurement。比如 `FMCW LiDAR` 会持续发射频率变化的光，并可通过检测频移来测量被照射物体的相对速度，这与 `FMCW radar` 的机制类似。这种能力在动态环境或困难场景中很有价值 [1200]，但系统通常也会更复杂、更昂贵。另一种创新机制是 `flash LiDAR`，它还可以提供类似 camera photometric measurements 的 ambient channels。不同技术在功耗、重量和成本上的特性差异，使得 `LiDAR odometry` 与 `SLAM` 在不同应用中拥有多样化的硬件选择。

## 8.2 LiDAR 里程计

`LiDAR odometry` 的任务，是根据连续 scans 之间的几何一致性估计局部相对运动。最基本的问题是 scan registration，但一个真正稳健的 `LiDAR` odometry system 还必须处理点云运动畸变、局部地图维护、几何退化和实时性约束。

### 8.2.1 扫描配准基础

scan registration 的目标，是在两个 point clouds 或 scan-map 之间估计一个最一致的 rigid transformation。无论方法细节如何变化，其核心始终围绕两个问题展开: 一是如何定义 residual 或 distance measure，二是如何建立 correspondences。

#### 8.2.1.1 配准残差中的距离度量

如前所述，`ICP` 在第 `k` 次迭代中寻找一组变换 `(R^k, t^k)`，使 source point cloud `P` 与 target point cloud `Q` 之间的总配准误差最小:

`R^k, t^k = argmin Σ_(p,q)∈C d(p, Rq + t)` 。

这里的关键首先在于，如何选择距离函数 `d(·)`。书中指出，最常见的几何元素是 points、lines 与 planes，如图 8.3 所示。

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

### 8.2.2 LiDAR 里程计的常见组成

成熟的 `LiDAR` odometry 不只是一个简单的 `ICP` loop，而是一组相互配合的模块，包括 motion distortion compensation、feature extraction 或 direct point-wise alignment、local mapping 以及 pose refinement。

#### 8.2.2.1 点云运动畸变补偿

许多 `LiDAR` 传感器通过在一段时间内完成扫描来形成一帧点云。这意味着一帧内不同点实际上对应不同时间。若平台在运动，整帧点云就会产生 motion distortion。

因此，很多系统需要在 registration 之前先做 deskew / undistortion。若不做补偿，一帧点云就不是单一时刻的静态快照，而是由不同采样时刻的点拼接而成，最终会显著损害配准精度与鲁棒性。

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

作者在这里总结说，`LiDAR` odometry 的表现是多个因素共同作用的结果，包括传感器扫描几何、畸变补偿、残差设计、对应关系估计、局部地图结构和退化场景处理。

因此，并不存在一个 universally best 的 `LiDAR` odometry 方案。不同系统通常是在 accuracy、robustness、runtime 与实现复杂度之间做工程折中。

### 8.3 LiDAR 地点识别

要从单纯 odometry 走向完整 `LiDAR SLAM`，必须让系统具备 place recognition 能力，即能够识别“当前所在位置是否和历史中某个地点相同”。这是 loop closure detection 和重定位的基础。

与 visual data 不同，`LiDAR` 能稳定提供周围环境的一致性度量 `3D` 信息，因此对光照变化通常不如相机敏感。但作者强调，`LiDAR place recognition` 依然面临一组非常独特的困难。

第一是 `sparse data`。与密集且规则排列的图像像素不同，传统 `LiDAR` 获取的点在空间中的分布稀疏而不规则，局部密度还会随传感器类型和量程变化。正因如此，`LiDAR place recognition` 往往不像视觉那样天然适合为每个点都构造一个局部 keypoint descriptor。为弥补结构缺失，很多方法转而使用具有语义意义的点云 segment，或者为整帧 scan 计算一个 global descriptor。随着深度学习发展，学习式 local keypoint descriptors 也重新活跃起来。

第二是 `structural aliasing`。在长走廊、高速公路或规则办公环境中，不同地点可能在 `LiDAR` 扫描中呈现非常相似的结构布局。视觉系统有时还能借助纹理、海报或装饰来区分这些地方，但单靠一帧 `LiDAR` scan，区分“同样形状但不同位置”的场景往往非常困难。像 `Scan Context` 这类 global descriptors 在这种情形下可能失效，而使用 object-level clusters 或更高层语义特征的方法，则往往更有机会缓解这种结构混淆。

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

### 8.4 LiDAR SLAM

完整 `LiDAR SLAM` 系统在 odometry 之外还包含 place recognition、loop closure verification、backend `PGO` 与 map update。它的关键不只是发现回环，而是让全局轨迹和地图在引入新约束后保持整体一致。

### 8.4.1 LiDAR SLAM 系统结构

作者给出了一个典型的 `LiDAR SLAM` system decomposition。首先，odometry module 在传感器帧率下运行，通常以 frame-to-map 的方式根据 active local map 估计当前 pose。其次，place recognition module 根据当前 scan 或 submap 检索潜在的 loop closure 候选。

但 place recognition 只能说明“两个地点可能相同”，并不能直接提供精确相对位姿。为了得到准确的 loop constraint，系统通常还要对相关 scans 做精配准，最常见的是 `ICP`。若缺少好的初值，还可能需要现代 global registration 方法来提供 initialization，尤其在 overlap 不大的情况下更是如此。

随后，系统会依据几何验证结果、时间间隔、旅行距离、`RANSAC` 一致性等 heuristics 来判断一个 loop closure 是否可信，避免把错误回环加入图中。最后，backend 模块基于全体约束做 `PGO`，并把修正后的轨迹反映到地图表示上。

### 8.4.2 Pose-graph Optimization 与地图更新

在 `LiDAR SLAM` 后端中，`pose-graph optimization` 是最核心的步骤之一。它的任务是综合 odometry constraints 和 loop closures，对整条轨迹进行全局一致化修正。由于图里通常只包含相对位姿约束，因此这种图也被称为 pose graph。

作者指出，这类问题可使用通用优化器，如 `g2o` 或 `GTSAM`。由于约束集通常非常稀疏，配合稀疏矩阵重排、重线性化和增量求解器，拥有上千节点的 pose graph 也可以在很短时间内完成更新。

但仅仅纠正轨迹还不够，因为地图本身也包含历史测量积分出来的结构。一个简单做法是根据修正后轨迹重新把所有历史观测融回地图；但这要求系统长期保存所有原始观测，在大规模环境中往往难以接受。因此另一条路线是 deform map，或者把 map elements，如 surfels 或 submaps，显式链接到 poses 上，使它们能随 pose graph 的修正一起被拉回一致状态。

作者还讨论了 scalability。实际系统不会在传感器频率上给 pose graph 每一帧都加节点，而往往只在每隔一定距离、一定信息量或者每个 submap 粒度上加入节点。这样才能把 `LiDAR SLAM` 扩展到城市级地图。

此外，backend 还必须考虑 odometry 自身的不确定性，以及 loop closure 约束中可能存在的错误。作者指出，可以通过 data-driven covariance 来更合理地分配误差，也可以采用 robust `PGO` 方法对可疑约束做 down-weight、禁用或剔除。

### 8.4.3 多机器人与多会话 LiDAR SLAM

随着单机器人单次任务 `LiDAR SLAM` 的成熟，研究自然延伸到了 multi-session 与 multi-robot 场景。这样做的好处很多: 可以把多个不同时刻的扫描结果合并起来延展地图，也可以让多台机器人共享一个共同地图用于协同探索，还可以对同一地点的多次测绘做时序对比以检测环境变化。

但作者特别指出，这里面存在一个常被低估的问题: 即便现代 `LiDAR SLAM` 精度已经很高，一张最终地图内部仍会保留小量误差。若把两次任务最终输出的 point cloud 直接做刚体配准，很容易产生 double-wall 等假变化现象。因此，更合理的做法通常是把每次任务的后端约束保留下来，在统一图里共同优化。

一种做法是把多个 session 的约束直接转移到单一全局图中；另一种做法则是保留每个 session 自己的局部图结构，再通过额外变量把各自坐标系联系起来。作者提到 `anchor node` 就是一个很有代表性的辅助变量，它可以显式表示不同任务之间的坐标系偏移，从而方便地实现多 session 地图的联合优化。

作者还指出，多 session `SLAM` 与单 session `SLAM` 的一个根本差异在于: 在完全断开的任务之间，没有现成的几何先验来生成第一个跨任务约束。因此，place recognition 在这里几乎成为唯一的桥梁。

#### 8.4.3.1 多机器人 SLAM

real-time multi-robot `SLAM` 比多 session 更进一步，因为它要求多个机器人在现场同时采集数据，并在线融合成统一地图。这样做能够让机器人团队共享探索结果、避免重复劳动、联合选择 frontiers，并共同支持搜索救援等任务。若能做到这一点，机器人团队就可以真正协同工作，高效探索区域、识别可通行路线，并发现感兴趣的人或物体。这种能力既适用于 search and rescue，也适用于军事等场景。

作者区分了 centralized 与 decentralized / distributed 两类系统。前者由现场机器人把观测传回 base station，由中心节点统一构图和优化；后者则让每台机器人本地构建自己的 `SLAM` 地图，再把多个本地图在某种中心节点或分布式协议中融合。采用 centralized 架构时，移动平台本身的 sensing 与 compute 可以保持相对简单，例如仓储场景中的一些机器人系统就是如此。

为了说明该领域的演进，作者回顾了两个重要挑战赛。2010 年的 `MAGIC` 挑战中，获胜队伍使用多台带 `2D LiDAR` 的机器人，在基地站完成全局 `2D` 多机器人 pose-graph 地图构建。更具体地说，Team Michigan 部署了 `14` 台 `3D` 打印机器人，每台都配备 `2D LiDAR` 与 camera；机器人本体执行 `2D LiDAR odometry`，再把 pose-graph constraints 传回 base station，由后者拼装成全局 `2D` multi-robot map。

到 2021 年的 `DARPA Subterranean Challenge`，系统已经发展到在复杂 `3D` 地下环境中融合 `3D multi-beam LiDAR`、visual odometry、wheel / legged odometry，甚至 thermal odometry，以应对隧道等几何退化场景。参赛队伍后来发表了系统综述 [299]，比较了各家的 fielded systems。一个共同特点是，多数队伍采用 semi-decentralized 架构：每台机器人在机载计算机上维护 pose-graph-based `SLAM` 地图，再由中心 base station 汇总成 multi-robot `SLAM` map。

这些比赛也暴露了新的关键问题: compression、communication 与 consistency maintenance。由于机器人团队必须维护动态无线 mesh network，把数据持续回传给基站，因此地图表示和压缩方式直接影响系统成败。作者举例说，CSIRO 的 `WildCat SLAM` [600] 使用 compressed surfel representation 表示 local submaps，这种 surfel map 相比原始 point clouds 占用空间小得多，从而显著降低传输 map 与 pose-graph constraints 所需带宽；在决赛阶段，每台机器人完整地图仅需 `21.5 MB`。

如作者所言，最复杂的问题其实是 fully distributed `SLAM`：不再依赖单一中心，而是要求每台机器人都在通信与规模约束下，自己维护整体联合地图的表示。已有一些工作 [493, 1094] 开始探索这一方向，重点研究如何逐步在机器人之间共享约束集与 local submaps。这里最关键的问题之一，就是如何在持续异步通信下保持整张地图的一致性。

## 8.5 延伸阅读与最新趋势

作者指出，`LiDAR SLAM` 尽管已经非常成熟，但仍有不少挑战。首先是 `robust and resilient perception`。在结构贫乏走廊、地下矿井、烟雾、雪、雾和粉尘等环境中，当前 `LiDAR SLAM` 系统仍经常表现不佳。更进一步地，很多系统当前仍主要通过经验 feature engineering 和手工参数调节达到最佳性能，而不是具备形式化的鲁棒性分析和自适应能力。

place recognition 方面也依然很活跃。研究方向包括跨不同 `LiDAR` 类型更稳健的 retrieval、与 radar 或 `OpenStreetMap` 的异构 place recognition、结合 intensity 信息的检索、跨多 session 的长期 place recognition，以及 change detection 与 lifelong map management。

最后，多传感器融合也是 `LiDAR SLAM` 的关键趋势。像 radar 在雨雾和烟尘中表现更好，视觉在某些几何退化环境下能弥补 `LiDAR` 的弱点，因此多模态融合是构建更 robust、更 resilient 系统的重要方向。不过，一旦传感器增多，系统又会面临冗余、同步、计算负载和信息选择等新问题。作者据此强调，未来的重点不仅是“多装几种传感器”，更是“如何高效选取最可靠的信息，并以轻量、可扩展的方式融合它们”。

