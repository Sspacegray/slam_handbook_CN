# 第 3 章 面向错误数据关联与离群点的鲁棒性

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

