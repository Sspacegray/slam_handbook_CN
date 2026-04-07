# 第 1 章 面向 SLAM 的 Factor Graphs（因子图）

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

