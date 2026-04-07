## 第 7 章 Visual SLAM（视觉 SLAM）

### 本章概览

本章围绕 `Visual SLAM` 展开。作者先从历史上追溯 visual `SLAM` 与 photogrammetry、`Bundle Adjustment (BA)`、`Structure from Motion (SfM)` 的渊源，然后系统梳理一个典型视觉 `SLAM` 系统的 pipeline。接着，本章会逐步介绍相机模型、关键点、重投影误差、光度误差、图像对齐与 `BA` 等视觉 `SLAM` 的基础构件，最后讨论完整系统、稠密重建、深度相机、多模态融合以及当前研究趋势。

作者强调，视觉 `SLAM` 的难点并不只是“从图像里提几何”。相机本质上观测的是三维场景在图像平面上的投影，因此所有深度、位姿和地图信息都必须通过数据关联、几何模型以及优化过程间接恢复出来。这种“信息丰富但几何不直接”的特性，决定了视觉 `SLAM` 既强大又脆弱。

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

以 `ORB-SLAM` 一类系统为例，跟踪线程通常只解决当前帧的 pose estimation，而不更新全部地图点。这往往通过 pose-only `BA` 完成，即在地图点保持固定的情况下，只优化当前相机位姿。与此同时，mapping thread 则只在关键帧上做更重的局部或全局优化。

作者指出，使用 keyframes 的原因在于全量 `BA` 在帧率上运行通常不可承受，因此需要把优化频率降到 keyframe rate。关键帧可以按固定时间插入，但更常见、更合理的策略是只在当前帧带来显著新信息时把它升级为 keyframe。

另一个关键问题是 locality。对于大场景来说，相机当前观测对地图中远处部分的影响通常很弱，除了回环发生时才需要全局协调。因此很多 visual `SLAM` 系统采用 local `BA`，只在当前关键帧及其 covisibility neighborhood 上做优化，而不是每次都优化整个图。作者用 `ORB-SLAM` 的 covisibility graph 举例说明，这种局部窗口设计能在保证精度的同时控制计算成本。

### 7.4.2 直接图像对齐

在 direct methods 中，图像对齐不再围绕离散关键点，而是围绕 photometric consistency 展开。作者以 `LSD-SLAM` 和 `DSO` 为代表进行说明。

`LSD-SLAM` 通过交替优化 depth map 与 camera motion 来实现 tracking 和 mapping，并进一步使用 `PGO` 保持全局一致。`DSO` 则进一步走向联合优化，即在一个滑动窗口内直接对 structure 和 motion 做 photometric `BA`。其鲁棒损失通常通过小 patch 上的平方差实现，同时还会处理曝光时间未知的问题。

作者强调，这类 direct system 的优势在于它们不进行中间抽象，而是更接近“直接对原始感知数据求最大后验估计”，因此在精度上可能优于关键点方法。与此同时，为了达到 CPU 上的实时性，系统往往需要对滑动窗口大小、边缘化策略和优化结构做大量 carefully designed engineering。

### 7.4.3 求解 BA

虽然 `BA` 理论上可以用一般的变量消元技术求解，但它的 Jacobian 和 Hessian 具有极其特殊的稀疏结构，因此存在专门更高效的求解方法。

每个图像观测只依赖一个 camera 和一个 point，因此 observation Jacobian 十分稀疏，对应 Hessian 也呈现典型 block structure: camera-camera 对角块、point-point 对角块，以及 camera-point 耦合块。由于三维点数量通常比相机数量多得多，一个非常自然的思路是先消掉 points，再求 cameras。

作者回顾了 `Schur complement` 的经典做法。通过先消元 points，可以得到 reduced camera system；解出 cameras 后，再反求 points。由于 `H_pp` 是 block diagonal，这一过程可以非常高效地逐点进行。local `BA` 中 reduced camera system 往往较稠密，因此可以用 dense solver；full `BA` 中 keyframes 更多，但 covisibility 更稀疏，因此则更适合 sparse solver。

### 7.4.4 再谈 Bundle Adjustment

本节最后作者提醒，现代 `BA` 并不只是“写下一个最小二乘然后交给 `Gauss-Newton`”。真正的系统还需要面对 robust loss、变量参数化、关键帧删除、边缘化、rolling shutter、曝光问题以及与 loop closure 的耦合。

因此，`BA` 既是一个数学优化问题，也是一个系统架构问题。实际中的精度、鲁棒性和实时性，很大程度上取决于这些 surrounding design choices，而不只是某一个求解器本身。

## 7.5 完整视觉 SLAM 系统示例

作者回顾了一些完整 visual `SLAM` systems，以说明上述模块如何在真实系统中协同工作。`PTAM` 是双线程 tracking / mapping 分工的经典起点；`ORB-SLAM` 系列则把 keypoint extraction、place recognition、local mapping、loop closing 与 optional full `BA` 统一到了一个非常成熟的框架中。

这些系统表明，visual `SLAM` 的成功不来自单个“神奇模块”，而是来自 detector、initializer、tracker、mapper、loop closer 与 optimizer 在时间和数据流上的精细协同。

## 7.6 实时稠密重建

作者接着讨论 visual `SLAM` 的另一个重要方向: real-time dense reconstruction。视觉系统并不一定只输出 sparse points，还可以利用 depth estimation、multi-view consistency 和 `GPU` 计算构建更密、更完整的三维模型。

无论是基于深度图融合、`TSDF` 还是 surfel-based 表示，这类方法都显著提升了系统在重建、交互和 AR/VR 场景中的能力。但它们也更依赖准确的位姿估计，并带来更高的内存与算力需求。

## 7.7 基于深度相机的 SLAM

随着 Microsoft `Kinect` 的出现，depth-sensing cameras 开始成为大众可获得的传感器。典型 `RGB-D` cameras 通常基于 structured light 或 time-of-flight 原理工作，同时输出 depth image 与 color image stream。从传感特性上看，它们介于普通 camera 与 `LiDAR` 之间: 不像 `LiDAR` 那样逐点扫描，而是直接给出一个瞬时的二维深度阵列；但与普通相机相比，又天然提供了每个像素的深度信息。

配合合适算法，这类传感器对 `3D` 感知非常强大，但书中也明确指出了它们的典型局限: 一方面多用于 indoor 场景，因为红外主动感知容易受到 sunlight 干扰；另一方面量程通常有限，常见设备大致只适用于 `5` 米以内的近距离环境。

作者随后回顾了 `KinectFusion` 这一经典系统。它建立在更早期的 range image fusion 思路之上，主张从移动中的 `RGB-D` camera 同时恢复相机运动与三维结构。其核心思想，是把每一张 depth image 编码成一个 projective signed distance function `di(x)`，其中每个 voxel `x` 上都保存到最近表面的有符号距离；随后再通过加权平均将多帧深度融合成一个全局距离函数 `D(x)`。在假设 depth 方向噪声近似 Gaussian 的情况下，这种加权平均实际上就是距离函数的最大似然估计。为了增强稳健性，工程上通常不会直接平均完整 `SDF`，而是平均 `truncated signed distance functions (TSDFs)`，使每个表面点只在局部区域内影响重建。

权重 `ωi(x)` 则用于表达各表面测量的置信度，它通常依赖具体传感器特性，并会随物体距离增长而衰减。对重建中的空洞问题，可以在后处理阶段填补，也可以直接修改加权策略，使重建结果更接近 watertight surface。

在相机跟踪方面，早期 `RGB-D SLAM` 往往通过 `ICP` 对齐连续 depth point clouds；后续方法则越来越倾向于直接最小化 color consistency 与 depth consistency 残差，而不是只做纯几何点云配准。也就是说，深度相机不仅降低了尺度恢复和初始化难度，还让 dense tracking 与 dense reconstruction 更容易统一到同一个系统里。

不过作者也提醒，depth camera 只是把问题从“从几何中恢复深度”转成“如何处理深度噪声、深度量程和光照适应性”。它并没有自动消除所有困难。

## 7.8 视觉与其他模态的融合

视觉虽然信息量极大，但在高速运动、低纹理、强曝光变化和长期漂移场景中也很容易失败。因此，把视觉与其他 sensing modalities 结合，已经成为现代 high-performance `SLAM` 的重要方向。

### 7.8.1 惯性测量单元（IMU）

`IMU` 是视觉最常见的配套模态。它可以提供高频短时运动约束，弥补相机在快速运动、低纹理甚至 rolling shutter 环境下的不足。因此 visual-inertial odometry 和 visual-inertial `SLAM` 已经成为机器人和 AR/VR 中最实用的一条路线。

当然，这也带来了额外挑战，包括时间同步、外参标定、bias 建模和预积分等问题。作者将在第 11 章更详细展开这部分内容。

### 7.8.2 用于全局定位的 GPS 与 WiFi

`GPS` 与 `WiFi` 这类外部信号可以为视觉系统提供 coarse but absolute 的位置先验。虽然它们通常不如局部视觉几何那样精细，但对抑制长期漂移、帮助重定位和提供全局锚定都非常有价值。

## 7.9 延伸阅读与最新趋势

在章节最后，作者指出 visual `SLAM` 的几个关键趋势仍在持续推进: 更强的 learned feature 与 learned retrieval、更鲁棒的 direct / feature hybrid pipelines、更高质量的 dense reconstruction，以及与 `IMU`、`LiDAR`、depth 和语义信息的深度融合。

同时，visual `SLAM` 也越来越强调长期运行能力、动态场景处理、资源受限设备上的部署以及学习模块与传统几何后端的协同。作者的整体观点非常清晰: visual `SLAM` 并不会由某一条路线“一统天下”，而是 feature-based、direct、dense、semantic 和 multi-modal 方法正在逐渐互补融合。

