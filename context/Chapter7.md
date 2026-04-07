# 第 7 章 Visual SLAM（视觉 SLAM）

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

作者指出，这类 photometric calibration 通常只需要在尺度意义上成立；而且它主要对那些希望获得 dense、textured scene representations，或依赖图像间 photo-consistency 的方法才真正重要。

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

对式 `(7.3)` 中重投影误差的最小化，通常需要借助非线性迭代优化来完成，而这类优化要想收敛，就要求初始值足够准确。因此，在视频序列最初几帧中如何初始化 visual `SLAM` 的状态，对后续正确跟踪至关重要；尤其对 monocular 系统更是如此，因为单帧并不能观测完整状态。

### 7.3.8 地图表示

地图表示是 visual `SLAM` 的关键组成，因为它决定了环境将如何被建模和存储，以服务于 navigation、mapping 和 localization。一个设计良好的地图表示，需要在 accuracy、memory efficiency 和 computational cost 之间取得平衡。关于 dense map representations 的更广泛讨论，作者建议参见第 `5` 章。

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

作者回顾了 `Schur complement` 的经典做法。若有一个线性系统，且其中 `D` 可逆：

```text
[ A  B ] [x1] = [b1]
[ C  D ] [x2]   [b2]
```

则可通过左乘一个变换矩阵，将其化为先解 `x1`、再解 `x2` 的形式：

```text
[ I  -BD^(-1) ] [ A  B ] [x1] = [ I  -BD^(-1) ] [b1]
[ 0      I    ] [ C  D ] [x2]   [ 0      I    ] [b2]

[ A - BD^(-1)C     0 ] [x1] = [ b1 - BD^(-1)b2 ]
[      C           D ] [x2]   [       b2       ]
```

也就是

```text
(A - BD^(-1)C) x1 = b1 - BD^(-1) b2
D x2 = b2 - C x1
```

在 `BA` 中，每次迭代都需要解一个形如

```text
[ H_cc   H_cp ] [d_c] = [b_c]
[ H_cp^T H_pp ] [d_p]   [b_p]
```

的线性系统。应用 `Schur complement` 之后，可按三步求解：先对 points 做消元，得到 reduced camera system；然后先解相机增量；最后再回代求 points：

```text
H_cc^red = H_cc - H_cp H_pp^(-1) H_cp^T
H_cc^red d_c = b_c - H_cp H_pp^(-1) b_p                             (7.7)
H_pp d_p = b_p - H_cp^T d_c
```

由于 `H_pp` 是 block diagonal，`Schur complement` 的构造以及最终 point solution 都可以逐点非常高效地完成。正如图 `7.10` 所示，reduced camera system 会变得没那么稀疏，因为其中会出现“看到共同 points 的相机对”之间的耦合块。例如示例中 camera `1` 与 `2` 之间存在耦合块，而 `1` 与 `3` 之间则没有。对于 local `BA`，reduced camera system 往往接近满阵，因此可以使用 dense matrix solvers；而 full `BA` 拥有更多 keyframes，但它们之间的 covisibility 更稀疏，因此更适合对 reduced camera system 使用 sparse solver。

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

