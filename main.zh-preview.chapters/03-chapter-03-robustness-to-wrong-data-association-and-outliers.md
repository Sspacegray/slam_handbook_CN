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

`RANSAC` 是最经典、应用也最广泛的 outlier rejection 工具之一，在很多 landmark-based `SLAM` 系统里都是关键组件。

以 feature-based visual `SLAM` 为例，系统通常会在相邻图像帧之间建立 `2D-2D correspondences`。这些 correspondences 来自 optical flow 跟踪或 descriptor matching，因此天然可能含有错误匹配。`RANSAC` 的作用，就是在把这些 correspondences 传给后端之前，尽快找出其中明显不符合几何约束的部分。

`RANSAC` 依赖两个基本事实。第一，正确的 inlier correspondences 必须满足某类几何约束。例如双目或两帧视觉几何中的 `epipolar constraint`，就要求匹配像素对必须与同一个相对相机运动一致。更一般地，可以把这种约束写成 `C(zi, x) ≤ γ`，其中 `x` 是局部状态变量，`γ` 用于容忍噪声。

第二，只要 outliers 比例没有高到离谱，就可以把 inliers 视为“最大的、对同一个状态 `x` 保持一致的 measurement subset”。这对应计算机视觉里的 `consensus maximization` 问题。精确求解这个组合问题代价很高，因此 `RANSAC` 不去穷举所有候选子集，而是反复采样一个足以估计状态的 `minimal set`，利用 `minimal solver` 先给出一个局部状态估计，再看有多少 measurements 与它一致。

标准 `RANSAC` 的流程很直接:

1. 随机采样一个最小测量子集。
2. 用 minimal solver 估计状态。
3. 统计与该状态一致的 measurements，形成 `consensus set`。
4. 保留当前最大的 consensus set，迭代直到达到停止条件。

`RANSAC` 在两个条件下特别有效:

- inlier 比例足够高
- minimal set 足够小

比如五点法估计相机相对位姿时，minimal set 很小，因此 `RANSAC` 往往几毫秒内就能收敛到不错的结果，同时还能给后端提供一个相当有价值的初值。

但它的局限也很清楚: 若 inlier 比例很低，或者问题的 minimal set 本身很大，那么所需采样次数会变得不切实际。例如在大型 `pose-graph SLAM` 中，想估计整个图结构的最小测量集本身就可能接近一棵 spanning tree，此时 `RANSAC` 就不再合适。

### 3.2.2 基于图论的离群点剔除与成对一致性最大化

当 outlier 比例很高，或 minimal solver 不现实时，`PCM` 提供了另一条路线。它不依赖对“最小子集”的反复采样，而是通过图论去找“内部两两一致”的最大测量集合。

这里的关键，是为每对 measurements 定义一个 `pairwise consistency function`。与 `RANSAC` 用到的几何约束不同，这类 consistency function 通常不直接依赖未知状态变量，因此可以只通过测量本身来判断两条观测是否彼此兼容。

书中给了两个例子:

- 在 RGB-D landmark-based visual `SLAM` 中，若两组 `3D-3D correspondences` 都是正确的，那么对应点对之间的距离在不同帧之间应该保持不变。
- 在 `pose-graph SLAM` 中，若两条 loop closures 都正确，那么把它们与中间 odometry 链组成一个闭环时，整体应当接近 identity。

更一般地，这类约束可以写成 `F(zi, zj) ≤ γ`。于是，outlier rejection 就可重新表述为: 在候选 measurements 集合 `M` 中，找出一个最大的子集 `S`，使得其中任意两条 measurements 都满足 pairwise consistency。这就是 `Pairwise Consistency Maximization (PCM)`。

PCM 的重要优势在于，它天然可以转成图问题。构建一个 `consistency graph`:

- 每个 measurement 对应一个节点
- 若两条 measurements 满足 pairwise consistency，就在对应节点间连边

这样，PCM 就等价于寻找图中的 `maximum clique`，也就是“任意两点都有边相连”的最大节点子集。于是，原本的 outlier rejection 问题可以直接借助成熟的 maximum clique 算法求解。

PCM 的典型流程是:

1. 为具体问题设计 consistency function `F`
2. 对所有 measurement pairs 计算一致性
3. 构建 consistency graph
4. 求解其 `maximum clique`
5. 返回 clique 中的 measurements 作为 inliers

与 `RANSAC` 相比，`PCM` 在高 outlier 比例下往往更强，尤其适合 `pose-graph SLAM` 这类 minimal set 巨大的问题。但它也并非全能: 一方面，一致性函数设计高度依赖问题本身；另一方面，`maximum clique` 依然是 NP-hard 问题，在稠密图上精确求解可能较慢，因此实践中常需借助启发式近似算法。

## 3.3 提升 SLAM Back-end 对离群点的鲁棒性

即便 front-end 使用了 `RANSAC` 或 `PCM`，它们也未必能把所有 outliers 都完全剔除。少量残留 outliers 进入后端后，仍然可能把标准 least-squares 解严重拖偏。因此，`SLAM back-end` 也必须具备一定的 outlier robustness。

本节从 `M-estimation` 出发，讨论如何把标准平方损失替换为 `robust loss functions`，以及如何用更高效或更稳定的方式优化这些目标。

### 3.3.1 迭代重加权最小二乘

`M-estimation` 的核心思想，是把 least-squares objective 中的平方损失替换为某个增长速度低于二次的 `robust loss` `ρ`。它的目的，是让极大的 residual 不再对总损失产生不成比例的支配作用。对于一个好的 robust loss，它的导数，也就是 `influence function`，在 residual 很大时应当显著减弱，理想情况下甚至趋近于 `0`。

这带来两个后果。第一，我们失去了原本专门为 least squares 设计的高效求解器优势；第二，robust objective 往往非凸，直接做 gradient descent 容易卡在坏的局部极小。

`Iteratively Reweighted Least Squares (IRLS)` 的想法，是在每次迭代里，把 `M-estimation` 转化成一个带权 least-squares 子问题。也就是说，在当前估计附近，根据上一轮的 residual，为每个 measurement 分配一个权重 `wi`:

- residual 小，权重大
- residual 大，权重小

于是优化器每一轮都在解一个 weighted least-squares 问题，然后再按当前 residual 重新更新权重。这样既能继续使用 `Gauss-Newton`、`Levenberg-Marquardt` 等成熟 least-squares solver，又能逐步逼近 robust objective 的最优解。

书中的实验表明，`IRLS` 相比单纯 gradient descent 收敛更快，通常几十次迭代就能收敛，而 gradient descent 往往需要成千上万步。不过，这种速度优势有时会以精度略降为代价，而且其收敛仍然非常依赖初始化。

### 3.3.2 Black-Rangarajan 对偶

常见的 `IRLS` 权重更新规则虽然实践中很常用，但从推导上看略显启发式，而且在某些非光滑 robust losses 上不够优雅。`Black-Rangarajan duality` 提供了一个更系统的解释框架。

其核心做法，是为每个 measurement 引入一个显式的权重变量 `wi ∈ [0, 1]`。以 `truncated quadratic loss` 为例，原本“超过阈值就截断”的 robust loss，可以改写成:

- 一个 weighted least-squares 项
- 加上一个只依赖于 `wi` 的 `outlier process` 惩罚项

这样，`wi = 1` 可以被理解为“该 measurement 被视为 inlier”，`wi = 0` 则对应“视为 outlier 并几乎从优化里剔除”。在这个视角下，robust optimization 不再只是“损失函数换了个形状”，而是变成了“同时估计状态变量 `x` 与 measurement 是否可信”。

`Black-Rangarajan duality` 还能推广到一大类 robust losses，例如 `Geman-McClure`。这样一来，不同 robust loss 都可以通过引入对应的 outlier process，被统一放进一个“带权 least-squares + 权重惩罚”的框架里。

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

围绕 `consensus maximization`，文献中已经发展出大量 `RANSAC` 变体，包括:

- 引入局部优化的版本
- 用更复杂的评分准则替代 consensus set 大小的版本
- 在采样策略上做偏置的版本
- 可微 `RANSAC`
- 在阈值未知情况下估计 inliers 的版本

此外，也有工作尝试精确求解 `consensus maximization`，通常基于 `branch-and-bound`。这类方法有全局最优保证，但最坏情况下复杂度仍为指数级，难以扩展到高维问题。

对 `PCM` 而言，后续研究一方面把它推广到了 `group-k consistency`，也就是不再只考虑两两一致，而考虑 `k` 个 measurements 的联合一致性；另一方面，也出现了把二值一致性放松成连续权重的“软最大团”思路。这些 graph-theoretic outlier rejection 方法已经被用于地下探索、LiDAR localization、多机器人建图等多个场景。

整体上看，本章呈现的是一条清晰主线:

- 在 front-end 中，依靠几何约束或图结构尽早过滤 gross outliers
- 在 back-end 中，依靠 robust losses、`IRLS`、`GNC` 等方法抑制残留 outliers 的影响

现代鲁棒 `SLAM` 系统往往需要把这两条路线结合起来。

