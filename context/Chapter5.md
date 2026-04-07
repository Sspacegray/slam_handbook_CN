# 第 5 章 稠密地图表示

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

