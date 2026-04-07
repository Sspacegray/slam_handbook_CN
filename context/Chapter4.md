# 第 4 章 可微优化

### 本章概览

正如第 1 章所述，现代 `SLAM` 系统通常遵循 front-end / back-end 架构。前端负责预处理 sensor data，并给出机器人轨迹和环境地图的初始估计；后端则通过求解优化问题，对这些初始估计进行精化，以提升整体精度。近年来，machine learning 尤其是 deep neural networks，为 `SLAM` 前端中的若干功能提供了新的实现方式，例如 feature detection / matching [970, 267, 1266] 与 front-end motion estimation [1166, 1081]。这些方法通过大规模数据训练神经网络，在没有被显式编程执行任务的情况下完成估计。与此同时，geometry-based techniques 仍然是 `SLAM` 后端不可替代的核心组成部分，因为它们在求解优化问题、从而产生 globally consistent estimates 方面仍然具有通用性和有效性 [1207]。

原则上，我们当然可以把 learning-based `SLAM` front-end 的输出直接“接”到 back-end 上，但 learning-based techniques 的引入实际上打开了一个不再单向传递信息的接口：back-end 的几何优化结果也可以反过来为前端学习提供 supervisory signal。于是，整个问题自然可以被表述为一个 `bilevel optimization`：上层是 neural-network-based optimization，用来训练前端；下层则是 geometry-based optimization，用来根据给定前端输出求出 `SLAM` 解。能够 end-to-end 地穿过优化器计算 gradients，正是求解这类 bilevel optimization 的核心；它使神经模型能够利用优化器中蕴含的 geometric priors。这样的灵活性已经在 structure from motion [1079]、motion planning [81, 1215]、`SLAM` [511, 1081]、`BA` [1067, 1266]、state estimation [1243, 182] 和 image alignment [715] 等广泛问题上带来了有前景的最新结果。

图 `4.1` 说明，现代 `SLAM` 系统往往同时包含 neural networks 与 nonlinear least squares。若把这两个模块分开优化，往往会引入 compound errors；因此，更理想的做法是把整个系统表述为 end-to-end 的 `bilevel optimization`，同时包含 upper-level cost 与 lower-level cost。

本章将介绍如何对 `SLAM` 中常见的 `nonlinear least squares (NLS)` 问题求导。具体来说，`4.1` 先回顾 `NLS` 问题本身；`4.2` 讨论如何穿过 `NLS` 做微分；`4.3` 说明如何对定义在 manifold 上的问题求导；`4.4` 讨论上述微分过程的数值挑战，并介绍相关 machine learning libraries；最后，`4.5` 给出当代 `SLAM` 系统中 differentiable optimization 的实例与趋势。

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

作者接着指出，`group action` 的 Jacobian 还取决于具体作用到的对象 `v ∈ V`。若群作用写作 `χ · v`，则可定义关于群变量与被作用变量的 Jacobian：

`J_(χ·v)^χ ≜ D(χ · v) / Dχ`                                                       `(4.37a)`

`J_(χ·v)^v ≜ D(χ · v) / Dv`                                                       `(4.37b)`

其中 `χ ∈ M`，`v ∈ V`。这类 Jacobian 在实际系统中很常见，因为状态变量往往并不是彼此直接相加，而是通过群作用去变换点、方向或局部坐标。

`Example 4.5 (Robot Arm)`：考虑一个有两个关节的机械臂，两个关节旋转 `R1` 与 `R2` 都属于 `SO(3)`，末端执行器的最终姿态由两者复合得到：

`R = R1 ◦ R2`                                                                     `(4.38)`

若要分析 `R1` 与 `R2` 的微小扰动如何影响最终姿态 `R`，就可以直接使用 composition Jacobian：

`∂(R1 ◦ R2) / ∂R1 = lim_(τ→0) ((R2^(-1) τ^∧ R2)^∨ / τ)`                          `(4.39)`

`∂(R1 ◦ R2) / ∂R2 = I`

这个例子说明，对第一关节 `R1` 的调整，会通过由第二关节当前状态决定的变换影响最终姿态 `R`；而第二关节 `R2` 的变化则会直接作用到 `R`，不受第一关节 `R1` 影响。

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

