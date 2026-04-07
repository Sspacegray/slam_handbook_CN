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

为追踪这些新测量，需要把原来的 pose graph 扩展成一个更大的 measurement graph：新增地标顶点集 `V_m` 与地标观测边集 `E_m`；同时把原来对应机器人相对位姿观测的边记为 `E_r`。在这种建模下，最大似然问题除了原有的 pose-pose 项外，还会加入一项 pose-to-landmark 残差 `‖m_k - t_i - R_i m̃_ik‖^2`。

接下来的发展与前一节基本一致。最优旋转变量仍可通过运行 `SE-Sync` 获得，只不过代价矩阵 `Q̃` 必须纳入地图点带来的影响。具体而言，权重矩阵 `Ω` 与测量矩阵 `D̃` 需要被重新定义为:

`Ω = BlockDiag(Ω_τ, Ω_μ),   Ω_τ = Diag(τ_1,...,τ_Np),   Ω_μ = Diag(μ_1,...,μ_Nm)` 。

同时，投影矩阵 `Π` 也要改写，以适应新的 measurement graph `G = (V ∪ V_m, E_r ∪ E_m)`。作者特别指出，连接 Laplacian `L̃(G^ρ)` 本身并不会因为新增地图点观测而改变。

一旦最优旋转求出，pose translations 与 map points 仍能像式 `(6.16)` 那样闭式恢复:

`[(t*)^T (m*)^T]^T = -vec(R* Ṽ^T L(W^τ)^†)` 。

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

之后，这一思路被推广到了 rotation averaging、multi-robot `SLAM`、range-aided `SLAM`、3D registration、multi-set registration、两视图几何、PnP、calibration 以及 pose-and-shape estimation 等问题。近年来还出现了处理 anisotropic noise 与 outliers 的 certifiable methods，说明这一方向的覆盖范围正在持续扩大。

但作者也指出，仍有三个非常重要的开放问题。第一，仍有不少 `SLAM` 问题很难直接纳入 certifiable pipeline。例如 visual `SLAM` 中的透视投影会带来有理函数目标，而不是简单多项式；虽然原则上可以通过引入额外变量把它们改写成 `POP`，但在每个 keypoint 都引入新变量通常是完全不现实的。同样，`IMU` 测量模型也尚未自然地导向高效的 certifiable algorithm。

第二，即便理论上有 `SDP` relaxation，可扩展性仍然是巨大挑战。某些问题可借助 `Riemannian Staircase` 获得高效求解，但更多情况下仍不得不依赖 interior-point methods 或专门设计的 ad hoc solvers。前者在前端小规模问题上很有效，但在大规模后端图优化上往往难以承受；后者虽然更 scalable，却通常仍比局部求解器慢得多。因此，近年来也有研究致力于通过压缩 monomial basis、减少约束数量等手段，让 `SDP` 更小、更快、更少退化。

第三，当前文献更多是在“给定一个具体实例后，为这一次实例计算最优性证书”，而不是从更根本的层面理解“到底什么时候 relaxation 会是精确的”。也就是说，我们还缺少一个统一理论，来预测 exactness 如何随着噪声大小、图结构甚至 outlier 比例而变化。

关于带 outliers 的问题，作者指出，虽然已有工作把 outlier-free certifiable algorithm 套在 `Graduated Non-Convexity` 的外循环里以获得经验鲁棒性，但近年来开始直接构造面向 robust estimation 的 certifiable methods。这些工作已经覆盖 `PGO`、rotation estimation、3D registration、absolute pose estimation 以及 pose-and-shape estimation 等问题。不过，带 outlier 的 moment relaxation 仍然很大、很慢，因此也有一些新工作转而尝试用局部方法求解，再利用 moment-relaxation 的洞见仅做“最优性检查”。

这里还存在一些尚未解决的挑战。一个问题是，目前很多 robust certifiable methods 默认 odometry backbone 是无异常值的，也就是说仅把 loop closure 或部分高风险观测包进 robust loss。这在很多实际系统中是可接受的，但在视觉 `SLAM` 中，当 feature tracking 崩溃时，odometry 自身也可能变成不可靠观测；在 multi-robot `SLAM` 中，不同机器人之间根本没有这样的干净 backbone。若把 odometry 本身也纳入 robust loss，当前 relaxations 往往会变松，精确性显著下降，而如何改进这些 relaxations 仍不清楚。

另一个更深的问题是: 即便 robust estimator 达到了“优化意义上的最优”，如果大多数测量本身都是 outliers，那最终得到的最优解仍然可能是严重错误的。作者据此提出，一个非常值得探索的方向，是设计能够恢复多个 hypothesis、同时保证至少有一个 hypothesis 正确的 certifiable algorithm，这与统计学中的 list-decodable regression 有很强关联。

最后，在 uncertainty quantification 方面，本章也指出了若干仍未解决的问题。传统协方差分析依赖已知且准确的 measurement covariance；一旦这些噪声协方差本身不准，`FIM` 和 `CRLB` 就会变得不可靠。近年来开始出现用学习方法估计 measurement covariance 乃至整个 measurement model 的研究，这与第 4 章讨论的 differentiable optimization 也有呼应。此外，在存在 outliers 的情况下，传统协方差分析往往无法反映实际的多峰或偏斜后验分布，因此如何为下游任务提供真正可信的不确定性量化，仍然是一个开放方向。

