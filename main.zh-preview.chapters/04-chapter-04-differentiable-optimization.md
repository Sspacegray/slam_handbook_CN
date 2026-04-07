## 第 4 章 可微优化

### 本章概览

深度学习与几何优化的融合，已经成为现代 `SLAM` 的重要趋势。本章讨论的核心问题是: 当系统里既有 neural networks，又有 `nonlinear least squares (NLS)` 优化模块时，如何把后者也变成可微的，使整个系统能够端到端训练。

更具体地说，本章说明如何对 `SLAM` 中常见的非线性最小二乘问题求导；这使得上层学习模块能够通过优化器返回的几何解，反向更新其参数，从而利用几何先验改进学习效果。

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

此时真正困难的地方，并不是上层损失 `U(y, x*)` 本身，而是如何求 `x*` 对 `y` 的导数。

### 4.2.1 展开式微分

`Unrolled differentiation` 的思路最直接: 把下层优化器的每一步迭代都显式展开，然后对整个迭代过程做 `automatic differentiation (AutoDiff)`。

若下层优化器从初始化 `x0 = Φ0(y)` 出发，经过 `T` 步更新得到 `xT`，那么就可以把 `xT` 看成 `y` 的复合函数，再对其直接反传。这样，原本的 bilevel problem 就被近似成一个显式可微的单层问题。

这种方法的优点是概念直观、实现直接，而且适用于多种迭代优化器；缺点是计算和内存代价高，因为必须保留整条优化轨迹。

### 4.2.2 截断展开式微分

完整 unrolling 代价很高，尤其当迭代步数多、状态维度大时。因此，一个自然改进是 `truncated unrolled differentiation`，也就是只保留最近若干步迭代历史，而忽略更久远的依赖。

这样做可以显著减少时间与显存开销，而在实践中往往仍能提供质量不错的近似梯度。进一步地，在更严格的资源约束下，甚至可以只保留一步更新，把梯度估计简化成更局部的近似。

代价当然是梯度不再精确。但在大规模学习系统里，这种偏差常常是可接受的，因为换来的好处是显著更好的可训练性与运行效率。

### 4.2.3 隐式微分

`Implicit differentiation` 不显式展开整个优化过程，而是利用“最优解满足最优性条件”这一事实，从下层问题的最优性条件出发，直接推导 `x*` 对 `y` 的导数。

这种方法的关键优势在于:

- 不需要存整条优化轨迹
- 通常更节省内存
- 在许多情形下能得到更稳定的梯度

代价是推导和实现通常更复杂，而且需要求解一个与 Hessian 或 Jacobian 相关的线性系统。总体而言，`unrolled differentiation` 与 `implicit differentiation` 代表了 differentiable optimization 的两条主路线，分别偏向“显式展开”和“隐式求导”。

## 4.3 流形上的微分

当优化变量定义在 manifold 上时，求导不再只是普通 Euclidean calculus。`SLAM` 中的 pose、rotation 等变量常位于 `Lie groups` 上，因此需要把上一章关于 manifold optimization 的工具与本章的 differentiable optimization 结合起来。

### 4.3.1 Lie 群导数

对 `Lie groups` 求导时，核心仍然是利用 `Lie algebra` 作为局部线性空间。很多导数最终都可通过:

- `Exp` / `Log`
- `wedge` / `vee`
- 左右 `Jacobian`
- `adjoint`

这些基本算子与映射来表达。换句话说，`Lie group derivatives` 的本质，是先在局部 tangent space 里写出变化，再通过 group 结构把它们映射回原变量。

### 4.3.2 流形上的微分运算

在实践中，流形上的可微操作通常包括:

- group composition 的求导
- inverse 的求导
- `Exp` / `Log` 的求导
- 使用 `retraction` 的更新步骤求导

本节的重点不是给出所有细节公式，而是说明: 若要让 `SLAM` 中的 differentiable optimization 真正工作，必须把这些 manifold primitives 当作一等可微对象来实现，而不是把 pose 粗暴塞回普通向量空间里处理。

## 4.4 自动微分与现代库的数值挑战

让优化器“理论上可微”是一回事，让它“数值上稳定地可微”是另一回事。本节讨论 differentiable optimization 在实现时的数值难点，以及现代相关库如何处理这些问题。

一个代表性例子是 `Lie groups` 上的 `Exp` / `Log` 映射。例如 quaternion 的指数映射中会出现诸如 `sin(||ν||)/||ν||` 这样的表达。若 `||ν||` 很小，直接数值计算会遇到精度问题甚至除零风险。因此，工程实现通常会在小角度区域切换到 `Taylor expansion`，用稳定近似替代直接公式。

本章以 `PyPose` 的 `LieTensor` 为例说明这些问题。它试图把 `Lie groups` 与 `Lie algebras` 作为一类可微数据结构统一支持，并兼容 `CPU`、`GPU` 等主流硬件平台。

### 4.4.1 可微优化实现示例

为了在端到端训练中真正使用 `bilevel optimization`，系统不仅需要标准深度学习优化器，还需要能处理 `SLAM` 常见几何问题的二阶或约束优化器。例如 `LM`、鲁棒核、`IRLS`、信赖域等都可能进入 differentiable optimization pipeline。

书中以一个加权 least-squares 问题为例，说明 `LM` 的核心计算包括:

- 构造近似 Hessian
- 引入 damping factor
- 解线性系统得到更新步

若再引入 robust kernel，就需要对 residual 和 Jacobian 做相应修正。在实践中，这里经常结合 `IRLS`。作者还提到 `Triggs correction` 与 `FastTriggs` 等实现技巧，用来在引入 robust kernel 的同时保持数值稳定与效率。

此外，简单的 `LM` 可能因为初值不好而不收敛，因此实际库还往往支持:

- adaptive damping
- trust region
- dogleg
- 更灵活的 solver / kernel / strategy / corrector 接口

这说明 differentiable optimization 不只是“把优化器包一层 gradient”，而是需要系统级地兼顾求解器、鲁棒性、数值稳定性和接口设计。

### 4.4.2 相关开源库

相关开源库大致可以分成三类。

第一类是 `linear algebra libraries`，例如:

- `NumPy`
- `Eigen`
- `ArrayFire`

它们提供高效的向量矩阵运算，是上层机器学习与优化库的基础。

第二类是 `machine learning libraries`，例如:

- `TensorFlow`
- `PyTorch`
- `JAX`
- `MXNet`

这些框架强调 tensor 计算与 `automatic differentiation`，也是现代端到端学习系统的主力。

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

近年来，深度学习在视觉里程计、VIO、跟踪与重建中的进展非常快。纯 data-driven 方法通常在感知能力上很强，尤其在视觉 tracking 方面往往比手工特征更有优势。但它们也面临监督数据昂贵、泛化困难和物理可解释性不足等问题。

因此，越来越多工作转向 hybrid methods，也就是把几何优化与深度学习结合起来。例如:

- 把 `BA` 层直接嵌入网络
- 让网络预测 coarse pose / feature / code，再由几何层做 refinement
- 让后端 `PGO` 或 `BA` 通过反传为前端网络提供全局几何监督

这一方向的核心目标，是减少“前端学习模块”和“后端几何模块”分开优化时累积的误差，让系统整体在端到端意义下变得更一致。

