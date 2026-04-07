# 第 2 章 高级状态变量表示

### 本章概览

上一章详细说明了如何使用 `factor-graph` 范式建立并求解一个 `SLAM` 问题，但当时刻意略过了状态变量本身的一些细节。本章重新审视这些待估计状态的性质，并引入现代 `SLAM` 中非常常见的两个主题。第一，是如何处理带有约束的状态变量。这类约束通常定义了一个 manifold，因此在优化时不能再把变量简单当作普通向量处理。`SLAM` 中最常见的例子，就是与机器人旋转相关的状态，尤其是三维旋转与位姿。第二，是时间建模方式本身。上一章默认机器人按离散时间步在世界中运动，而本章进一步介绍平滑的连续时间轨迹表示，并说明它们如何与 `factor graph` 框架自然兼容。

本章前半部分聚焦 `optimization on manifolds`，重点讨论旋转、位姿、`matrix Lie groups`、`Lie algebra` 以及在这些结构上进行 `MAP optimization` 的方式（Section `2.1`）。后半部分转向 `continuous-time trajectories`，介绍参数化样条（splines）、由 `kernel trick` 引出的非参数方法，以及 `Gaussian Processes (GPs)` 在轨迹表示中的作用，并最后把这些方法推广到 `Lie groups` 上（Section `2.2`）。本章最后以延伸阅读与研究趋势收尾（Section `2.3`）。

## 2.1 流形上的优化

在某些机器人问题里，我们可以直接把未知量表示成普通向量；但在大多数实际场景中，都必须处理三维旋转以及其他不属于向量空间的 manifold。粗略地说，manifold 可以理解成一个拓扑上闭合的曲面，例如圆周边界或球面。它的关键性质是: 在每个点的局部邻域内，它都近似于 Euclidean space。因此，manifold 上的优化需要比普通向量空间更细致的工具链。

本节讨论如何在 manifold 上做优化，并把上一章面向向量空间的优化框架推广过来。书中用球面作为示意例子: manifold 本体记作 `M`，而在某个点 `χ ∈ M` 附近，可以用切空间 `TχM` 作为局部坐标系来开展优化。

### 2.1.1 旋转与位姿

在 `SLAM` 中，最常见的两个 manifold 是用于表示 rotations 和 poses 的 manifold。旋转通常发生在二维平面或三维空间，因此旋转 manifold 一般写作特殊正交群 `SO(d)`，其中 `d = 2` 或 `3`。

二维平面上的 rotation matrix `Rb_a ∈ SO(2)` 可以写成:

`Rb_a = [[cosθ, -sinθ], [sinθ, cosθ]]`

这里 `θ ∈ R` 是唯一的自由度，也就是旋转角。它可以把在参考系 `Fa` 中表示的二维向量 `ℓa` 旋转到 `Fb` 中，得到 `ℓb = Rb_a ℓa`。

三维旋转矩阵 `Rb_a ∈ SO(3)` 的作用与此类似，只不过是把三维向量从一个坐标系变换到另一个坐标系。虽然三维 rotation matrix 有 `9` 个矩阵元素，但只有 `3` 个自由度，例如 `roll`、`pitch` 和 `yaw`。无论二维还是三维，rotation matrix 都必须满足

`Rb_a^T Rb_a = I`，且 `det(Rb_a) = 1`

这些约束正是它无法被当作普通向量随意加减的原因。

机器人的 pose 同时包含旋转 `Rb_a ∈ SO(d)` 和平移 `t^b_a ∈ R^d`，总共有 `3(d - 1)` 个自由度。有时我们把这两部分分开表示，写作 `{R, t} ∈ SO(d) × R^d`；另一种更紧凑的表示方式，是把它们组装成 `(d + 1) × (d + 1)` 的 transformation matrix:

`T^b_a = [[R^b_a, t^b_a], [0, 1]]`

所有这类 transformation matrices 组成的 manifold 被称为特殊欧氏群 `SE(d)`，其中 `d = 2` 表示平面运动，`d = 3` 表示三维运动。使用 `SE(d)` 的一个直接好处，是对 landmarks 的平移和旋转都能统一写成一次矩阵乘法，并且可以在 homogeneous coordinates 下方便表达。

不过，无论是 `SO(d)` 还是 `SE(d)`，由于它们都受结构约束，因此都不是向量。比如，把两个 rotation matrices 直接相加，并不会得到另一个合法的 rotation matrix。幸运的是，`SO(d)` 和 `SE(d)` 同时还是 `matrix Lie groups`。正是这种额外结构，使我们能够在不显式引入约束的前提下，继续完成 `factor-graph SLAM` 中的 `MAP optimization`。

### 2.1.2 矩阵 Lie 群

要在 `SO(d)` 和 `SE(d)` 上做优化，关键是利用它们作为 group 所具有的代数结构。首先，`matrix Lie groups` 在矩阵乘法下是封闭的。也就是说，如果 `Rc_b, Rb_a ∈ SO(d)`，那么它们的乘积 `Rc_a = Rc_b Rb_a` 仍然属于 `SO(d)`。

更重要的是，每个 `matrix Lie group` 都伴随着一个非常有用的 companion structure，称为 `Lie algebra`。`Lie algebra` 同时也是该 `Lie group` 在单位元（identity element）处的切空间（tangent space）。从本章的角度看，`Lie algebra` 最重要的两点是:

- 它本身是一个向量空间，其维度正好等于对应 `Lie group` 的自由度数目。
- 从 `Lie algebra` 到 `Lie group` 之间，存在成熟而标准的映射，也就是 `matrix exponential`。

以 `SO(2)` 为例，我们可以从一个标量角度 `θ` 出发，先通过 `wedge operator` 得到 `Lie algebra` 元素 `θ∧`，再通过 `Exp(θ) = exp(θ∧)` 构造 rotation matrix。反过来，也可以通过 `matrix logarithm` 回到 `Lie algebra`，再用 `vee operator` 取得对应向量表示，即 `θ = Log(R) = (log(R))∨`。

对于 `SO(3)`，`wedge operator` 对应的是把 `R^3` 中的向量写成一个 skew-symmetric matrix；对于 `SE(d)`，`Lie algebra` 元素 `ξ` 则把旋转部分 `θ` 与平移部分 `ρ` 一起编码进 `ξ∧ ∈ se(d)`，并通过

`T = Exp(ξ) = exp(ξ∧) ∈ SE(d)`

生成位姿变换。对这些常见 `Lie groups`，从 `Lie algebra` 到 `Lie group` 的映射都有成熟的 closed-form expressions，因此实践中通常不需要真的去使用无穷级数展开。

### 2.1.3 Lie 群优化

有了 `matrix Lie groups` 之后，我们就可以像在线性最小二乘中做线性化那样，对含 `Lie group` 变量的测量模型进行局部近似，从而继续开展 `MAP inference`。

书中给出的例子是一个相机观测模型。假设 `hi(·)` 接收在相机坐标系下表示的 homogeneous landmark `ℓ̃^c_i`，输出其图像像素坐标 `zi ∈ R^2`。于是生成式传感器模型可以写为:

`zi = hi(T^c_w ℓ̃^w_i) + ηi`

其中 `T^c_w ∈ SE(3)` 是世界坐标系相对于相机坐标系的 pose，`ℓ̃^w_i` 是世界坐标系下的 homogeneous landmark，`ηi` 是传感器噪声。若世界中的 landmark 已知，那么我们就希望通过最小化重投影误差来估计 `T^c_w`，这对应经典的 `perspective-n-point (PNP)` 问题。

在线性化时，关键是不要直接在线性空间里“加一个 pose”，而是通过 `Lie algebra` 构造一个小扰动:

`T^c_w = T^{c0}_w Exp(ξ)`

这里 `ξ ∈ R^6` 表示相对于初始猜测 `T^{c0}_w` 的一个小位姿增量。这种写法常被简写成

`T^c_w = T^{c0}_w ⊕ ξ`

由于 `SE(3)` 在乘法下封闭，更新后的结果仍然保证是合法 pose。更重要的是，`ξ` 的维度刚好等于位姿真实自由度，因此我们不需要在优化中额外处理约束。

若只保留 `Exp(ξ)` 的一阶项，就有近似

`T^c_w ≈ T^{c0}_w (I + ξ∧)`

把它代回测量函数，再结合相机模型自身的一阶 Taylor 展开，就能得到与上一章完全平行的线性化误差形式。也就是说，原本的非线性 measurement factor，最终仍然会被写成一个对扰动变量 `ξ` 线性的 least-squares term。

求得最优扰动 `ξ*` 之后，还必须按照之前选定的扰动方式更新初值:

`T^{c0}_w ← T^{c0}_w ⊕ ξ*`

这样才能保证更新后的解继续落在 `SE(3)` 上，而不是偏离 manifold。整个优化过程和上一章一样，仍然是迭代进行，直到包括 `ξ` 在内的全部状态增量都足够小为止。

这种方法不仅适用于测量函数的输入在 manifold 上的情况，也适用于测量本身位于 manifold 上的情况。例如，若传感器直接给出一个带噪 pose measurement `Zi ∈ SE(3)`，则误差通常在 `Lie algebra` 中定义，通过 `Log(·)` 把 group 中的偏差映射回切空间，然后再进行 least-squares 优化。

从更高层看，这其实是 `Riemannian optimization` 的一个实例。我们利用 `Lie algebra` 这个切空间，把优化约束在 manifold 的切向方向上；而通过 `⊕` 更新或 `retraction`，又把结果重新投回 manifold。除了 `matrix exponential` 之外，也存在其他可用的 `retractions`，整个框架并不局限于 `Lie groups`。

### 2.1.4 不确定性与 Lie 群

在估计问题中，我们通常把状态看成随机变量，并用分布来表示其不确定性。对普通向量变量 `x`，最常见的写法是

`x = μ + δ,   δ ~ N(0, Σ)`

其中 `μ` 是均值，`Σ` 是协方差，`δ` 是零均值 Gaussian noise。

但对 `Lie groups`，不能再直接把噪声加到群元素上。比如对一个 rotation matrix `R ∈ SO(d)`，若直接写成 `R + δ`，结果一般不再是合法 rotation matrix。更合理的做法，是先在 tangent space 中定义 Gaussian noise，再通过 `matrix exponential` 把它映射到 group 上:

`R = R̄ ⊕ δ = R̄ Exp(δ),   δ ~ N(0, Σ)`

对 `SE(d)` 也完全类似:

`T = T̄ ⊕ δ = T̄ Exp(δ),   δ ~ N(0, Σ)`

这样定义之后，`R` 保证仍在 `SO(d)`，`T` 保证仍在 `SE(d)`。换句话说，我们并不是直接在旋转或位姿空间里定义 Gaussian，而是在其 tangent space 中定义 Gaussian，再映射回 manifold。这种做法在本章讨论的“局部不确定性”场景下非常自然。当然，也可以直接在 rotations 上定义分布，某些情况下那样会带来额外的计算优势，本书第六章还会再回到这一点。

### 2.1.5 Lie 群补充内容

关于 `Lie groups`，其实还有许多可以展开的内容。为了不打断主线，本节把后续章节会频繁用到的若干补充工具集中放在一起说明。更细致的导数计算会留到 Section `4.3`。

### 2.1.5.1 `⊕` 与 `⊖` 运算符

前面已经见过 `⊕` 运算符，它把一个 `Lie algebra` 向量与一个 `Lie group` 元素组合起来。例如对 `SE(d)`:

`T = T0 ⊕ ξ = T0 Exp(ξ) = T0 exp(ξ∧)`

与此同时，我们也经常需要比较两个 `Lie group` 元素之间的“差值”。这时可以定义 `⊖` 运算符。仍以 `SE(d)` 为例:

`ξ = T ⊖ T0 = Log(T0^{-1} T) = (log(T0^{-1} T))∨`

`⊕` 和 `⊖` 的好处，在于它们把底层 `exp/log` 与左右乘法的细节统一封装掉了，使公式表达更干净。

### 2.1.5.2 逆运算

在处理 perturbation 时，经常会遇到对一个逆变换做扰动的情况。对 `SE(d)`，如果

`T = T0 ⊕ ξ`

那么它的逆满足

`(T0 ⊕ ξ)^{-1} = (-ξ) ⊕ T0^{-1}`

这表明当对逆矩阵做扰动时，扰动会从右侧移到左侧，并带上负号。

### 2.1.5.3 伴随映射（Adjoint）

`adjoint` 提供了一种把 `Lie group` 元素看成其 `Lie algebra` 上线性变换的方法。对 `SO(d)`，adjoint representation 与 group 本身一致，因此细节较少；对 `SE(d)`，adjoint 则是状态估计里非常重要的工具。

对给定 pose `T`，`SE(d)` 的 adjoint map 可以把局部坐标系中的 `Lie algebra` 元素 `ξ∧`，转换成全局坐标系中的另一个 `Lie algebra` 元素 `ϵ∧`，满足

`ϵ∧ ⊕ T = T ⊕ ξ∧`

并且可以写成所谓的 `inner automorphism` 或 `conjugation`:

`ϵ∧ = Ad_T ξ∧ = T ξ∧ T^{-1}`

如果把它写成线性变换矩阵，那么 `Ad(T)` 就是把 `ξ ∈ R^6` 直接映射到另一个 `R^6` 向量的矩阵表达。对 `SE(d)`，这个 adjoint representation 可以直接由 homogeneous transformation matrix 的分块构造出来。

adjoint 的一个重要用途，是在不做近似的前提下，把 perturbation 从已知变换的一侧搬到另一侧:

`T Exp(ξ) = Exp(Ad(T) ξ) T`

在状态估计推导里，这类换边操作十分常见。

### 2.1.5.4 Jacobian（雅可比）

每个 `Lie group` 都对应自己的 `Jacobian`，用于把 group 元素的变化与 `Lie algebra` 元素联系起来。以 `SO(d)` 为例，经典运动学方程，也就是 `Poisson's equation`，把旋转矩阵 `R ∈ SO(d)` 与角速度 `ω` 联系起来，可写为

`Ṙ = ω^∧ R`

这里 `ω` 表示相对于某一参考系、并在局部坐标中表达的角速度。若进一步把旋转参数化为 `R = Exp(θ)`，则等价地可以写成

`θ̇ = J^{-1}(θ) ω`

其中 `J(θ)` 就是 `SO(d)` 的左 `Jacobian`。它在处理多个 `matrix exponentials` 的组合时特别有用。例如，当 `θ1` 足够小时，有近似关系

`Exp(θ1) Exp(θ2) ≈ Exp(θ2 + J(θ2)^{-1} θ1)`

书中还给出了 `J(θ)` 的级数表达式：

`J(θ) = Σ_(n=0)^∞ 1/(n+1)! (θ^∧)^n`

闭式表达式可参见 `Barfoot` [47]。作者也说明，在后文中常会重载记号，把 `SE(d)` 的左 `Jacobian` 同样记作 `J(ξ)`，具体含义由上下文判断。总体而言，`Lie group Jacobians` 不仅对经典状态估计至关重要，也构成了现代可微神经网络 pose estimation 方法的基础。

## 2.2 连续时间轨迹

`continuous-time trajectories` 提供了一种描述平滑机器人运动的方式。到目前为止，我们默认需要估计的是一串离散时刻的 poses；但实际机器人运动往往是平滑的，因此直接引入连续时间轨迹表示会更加自然。

这类方法主要有两大类。第一类是参数化方法，用一组已知 temporal basis functions 线性组合出一条平滑轨迹。常见选择包括具有局部支撑的样条（splines），例如 `B-splines` 或 cubic Hermite polynomials。局部支撑的好处，是 factor graph 仍然保持稀疏。第二类是非参数方法，其表达能力更强，通常通过 `kernel functions` 建模；特别是把时间作为自变量时，一维 `Gaussian Process (GP)` 可以直接用来表示轨迹。如果核函数选取得当，由此得到的 factor graph 同样可以非常稀疏。

除了轨迹平滑性之外，连续时间表示在处理高频传感器和异步传感器时尤其有价值。若对每一条测量都单独引入一个 pose 变量，那么在 `IMU` 这类高频传感器、或多传感器不同步采样的场景下，factor graph 会迅速膨胀。连续时间轨迹则允许我们用远少于测量数目的状态变量描述整条运动，同时仍然保留每个 measurement 的精确时间戳。

这对存在 motion distortion 的传感器尤为重要，例如 spinning `LiDAR`、`Radar`，甚至 rolling-shutter camera。连续时间方法允许我们把每个点或像素都与其真实采样时刻的轨迹状态对齐。

最后，在 `MAP inference` 完成之后，连续时间轨迹还能支持对任意时刻的轨迹进行高效查询，而不局限于测量发生的时刻。也就是说，`measurement times`、`estimation variables` 和 `query times` 可以彼此分离，这正是连续时间方法的重要优势之一。

### 2.2.1 样条

参数化连续时间轨迹的基本思路，是把 pose 写成若干已知 temporal basis functions `Ψk(t)` 的加权和:

`p(t) = Σ_k Ψk(t) ck`

这里 `ck` 是待估计系数。为了先把思路讲清楚，书中先回到向量空间情形，再在后面讨论如何推广到 `Lie groups`。

常见的 basis functions 是 splines，也就是分段多项式。样条的关键优势，在于它们通常具有 `local support`，也就是在自己影响范围之外函数值为零。这样一来，在任意时刻 `t`，真正起作用的 basis functions 只占很小一部分，因此对应的 factor graph 依然稀疏。

若机器人在时间 `ti` 观测到一个 landmark `ℓ`，则传感器模型可以写作

`zi = hi(p(ti), ℓ) + ηi`

把 spline 表达代入，就得到一个关于活动系数变量和 landmark 变量的 measurement factor。由于在 `ti` 时刻只有少量 basis functions 非零，我们仍然可以把它写成标准的 `zi = hi(xi) + ηi` 形式，于是上一章的非线性 least-squares 框架就可以原样套用。

如果 basis functions 本身足够可微，那么轨迹的导数同样易于计算:

`ṗ(t) = Σ_k Ψ̇k(t) ck`

因此，哪怕传感器输出与速度甚至更高阶导数有关，也仍然可以围绕同一批 spline coefficients 做统一优化。

当 `MAP inference` 求得最优系数之后，我们就能在任意时刻用 spline 表达式去查询轨迹及其导数。若推断过程中同时得到系数的协方差，那么由于 spline 表达式对系数是线性的，查询时刻的 pose 协方差也可以方便地传播出来；而局部支撑性质意味着我们只需相关系数的局部边缘协方差。

### 2.2.2 从参数化到非参数化

基础参数化方法的主要难点，在于我们必须事先决定使用什么类型、多少个 basis functions。若 basis functions 太多，就容易对 measurement data 过拟合；若太少，轨迹表达能力又不够，最终会得到过于平滑的解。这正是转向非参数方法的动机之一。

为了简化讨论，书中暂时只考虑 pose 变量，不讨论 landmarks。在线性化之后，参数化方法的 least-squares term 会呈现出一个关于 spline coefficient updates `δc` 的优化问题。为了抑制过拟合，还会显式加入一个 regularizer `||δc||^2`，相当于偏好较小的 spline coefficients。

不过，实际关心的通常不是这些 coefficients 本身，而是各测量时刻的 pose 更新 `δ = Ψ δc`。经过代数整理后，优化问题可以改写成一个直接以 pose updates 为未知量的线性系统:

`(A^T A + K^{-1}) δ* = A^T b`

其中 `K = ΨΨ^T` 被称为 `kernel matrix`，起到 regularization 或 smoothing 的作用。

到这里，作者引入著名的 `kernel trick`。它的核心思想是: 不再显式表示 basis functions，而只使用它们之间的内积。于是我们可以直接用一个选定的 kernel function `K(t, t')` 来填充 kernel matrix，而不再估计 spline 的显式系数。这样一来，方法就从参数化转向了非参数化。

当然，代价是我们需要调节核函数的 hyperparameters，例如 squared-exponential kernel 的 length scale，以获得想要的轨迹平滑程度。另一个问题是，若直接这么做，`K^{-1}` 往往会是稠密的，从而导致推断代价很高。下一节要解决的，正是如何选取一种既是 GP kernel、又能保证逆核矩阵稀疏的构造方式。

### 2.2.3 Gaussian Processes（高斯过程）

本节构造一类特殊的 kernel functions，使得对应的 inverse kernel matrix 天然稀疏，因此 factor graph 也保持稀疏。与其从显式 basis functions 出发再套 `kernel trick`，作者改从一个 `linear, time-invariant stochastic differential equation (SDE)` 出发:

`ẋ(t) = A x(t) + L w(t)`

其中 `w(t) = GP(0, Q δ(t - t'))` 是零均值白噪声 `Gaussian Process`，`Q` 是 `power-spectral density matrix`，`δ(·)` 是 Dirac delta function。这里的想法，是把这个 SDE 作为 trajectory 的 motion prior。

这个 SDE 可以解析积分，得到状态 `x(t)` 关于初始时刻状态和过程噪声的表达。由于系统是线性且由 Gaussian noise 驱动，因此 `x(t)` 本身也是一个 `Gaussian Process`。在假设初始状态均值为零的简化情形下，可以进一步写出它的 covariance function，也就是 kernel function `K(t, t')`。

虽然这个 covariance 看上去比较复杂，但在所有 measurement times 上求值得到的 kernel matrix 可以整理成一个非常整洁的形式:

`K = Φ Q Φ^T`

更关键的是，我们真正需要的是 `K^{-1}`。由于 `Q` 是 block-diagonal，而 `Φ^{-1}` 只有主对角和次对角上有非零块，所以

`K^{-1} = Φ^{-T} Q^{-1} Φ^{-1}`

最终会变成 `block-tridiagonal` 结构。根据上一章关于稀疏信息矩阵与 factor graph 的讨论，这立刻意味着: 整条轨迹的 motion prior 虽然作用于所有时刻，但它完全可以由一张非常稀疏的 factor graph 表达出来，通常只包含一个起点 unary factor 和一串相邻状态之间的 binary factors。

这种稀疏性的根本原因，在于我们所选 SDE 的状态 `x(t)` 是 `Markovian`。从实践角度讲，不同的 motion prior 可能要求把状态扩充到更高阶。例如，如果希望使用 `constant-velocity prior`，那么状态就不仅包含 pose `p(t)`，还包含其导数 `v(t) = ṗ(t)`。这类做法常被称作 `simultaneous trajectory estimation and mapping (STEAM)`，可以看作 `SLAM` 的一种连续时间变体。

从统计学习的角度看，这套连续时间轨迹建模其实就是 `Gaussian Process regression`。因此，在 measurement times 上完成估计后，我们还可以利用 `GP interpolation` 在任意查询时刻高效恢复轨迹的均值与协方差；若采用本节这种稀疏 kernel 构造，每次查询的代价相对于测量数 `M` 是常数级的，因为只需要查询时刻两侧最近的已估计状态。

更进一步，GP interpolation 还能帮助我们减少 control points 的数量。比如，对一整帧 `LiDAR` 扫描，只放一个控制点，但仍然利用该扫描中每一个点的真实时间戳。于是，连续时间框架下的 `measurement times`、`estimation times` 和 `query times` 可以完全分离。与参数化 spline 方法不同的是，在 GP 框架里，即便设置较多 estimation times，也不会像样条那样直接导致过拟合，因为 kernel 本身已经承担了正则化作用；当然，控制点仍需要足够密，才能表达轨迹细节。

### 2.2.4 Lie 群上的样条与 GPs

无论是 splines 还是 GP continuous-time methods，都可以推广到状态位于 manifold 上的情形。若这些 manifolds 是 `Lie groups`，两类方法都会借助 `Lie algebra` 完成推广，只是切入方式不同。作者先解释 splines，再讨论 GPs。

### 2.2.4.1 Lie 群上的样条

让 splines 适用于 `Lie groups` 的关键，是采用 `cumulative formulation`。在线性向量空间中，如果把所有自由度都用同一组标量 basis functions `ψk(t)` 参数化，那么原始 spline 表达

`p(t) = Σ_k ψk(t) pk`

可以改写成累计形式:

`p(t) = ψ1^c(t) p1 + Σ_{k=2}^K ψk^c(t) (pk - p_{k-1})`

其中 `ψk^c(t)` 是 cumulative basis functions，也就是从第 `k` 个 basis 往后的累加。对于均匀时间间隔下的线性样条，这套 cumulative basis functions 在任意时刻只会激活极少数项。比如在线性样条中，如果 `(k - 1)T ≤ t < kT`，就可以写成

`p(t) = p_{k-1} + ψk^c(t) (pk - p_{k-1})`

此时只需评估一个 basis function。

把这个思想搬到 `Lie groups` 上时，向量加法不再可用，因此要用 group 运算，也就是矩阵乘法，来替代求和。对 `SE(d)` 的线性样条，插值可以写成

`T(t) = Exp(ψk^c(t) Log(Tk T_{k-1}^{-1})) · T_{k-1}`

也就是说，我们不是在线性空间里插值，而是在相邻 control-point poses 之间，沿 `Lie algebra` 中定义的“相对位姿”去做插值。

一旦有了 `T(t)` 的表达，就可以把某个 measurement time `ti` 上的插值 pose 直接代入测量模型，并对 control points `Tk` 做线性化。由于 cumulative basis functions 依然具有稀疏激活性，因此每条测量只会关联极少数控制点。

对线性样条而言，上式还能改写成

`T(t) = (Tk T_{k-1}^{-1})^{αk(t)} T_{k-1}`

当对涉及 `T(t)` 的表达做线性化时，可以沿用 Section `2.1.3` 中的优化方法。这里作者选择在左侧进行 perturbation，并指出右侧 perturbation 也可以采用类似构造。于是有

`Exp(ξ(t)) T^0(t) = Exp(ξk) T_k^0 T_{k-1}^{0,-1} Exp(-ξ_{k-1})^{αk(t)} Exp(ξ_{k-1}) T_{k-1}^0`

作者的目标，是把插值 pose 的 perturbation `ξ(t)` 与两侧 control-point poses 的 perturbations `ξ_{k-1}` 和 `ξk` 联系起来。其一阶近似结果为

`ξ(t) ≈ (I - A(αk(t))) ξ_{k-1} + A(αk(t)) ξk`

其中

`A(αk(t)) = αk(t) J(αk(t) Log(T_k^0 T_{k-1}^{0,-1})) J(Log(T_k^0 T_{k-1}^{0,-1}))^{-1}`

这里 `J(·)` 是 `SE(d)` 的左 `Jacobian`。有了这个关系，就能把测量时刻的 pose 变化映射到两侧 bracketing control-point poses 上。

例如，对式 `(2.16)` 的线性化 measurement model，若把它改写为关于 pose 与 perturbation 的误差形式，则有

`e_i(t) ≈ z_i - h(T_i^0(t) l_i) - H_i ξ(t)`

再把上式中的 `ξ(t)` 用 `(2.64)` 代入，就得到一个直接关于相邻两个 control-point poses 的线性化误差项：

`e_i(t) ≈ z_i - h(T_i^0(t) l_i) - H_i (I - A(αk(t))) ξ_{k-1} - H_i A(αk(t)) ξk`

作者特别提醒，此时还需要把 nominal pose 也替换为 `T^0(t) = (T_k^0 T_{k-1}^{0,-1})^{αk(t)} T_{k-1}^0` 后再带入 `h` 和 `H_i`。本质上，这一步就是把 spline 插值的导数链式传到相邻两个 control points 上。高阶样条也可以按同样思路推广，只是活跃控制点数量会更多一些。

### 2.2.4.2 Lie 群上的 Gaussian Processes

要把 `Gaussian Processes (GPs)` 用到 `Lie group` 上，我们同样要借助它的 `Lie algebra`。图 `2.6` 先给出了本节思想的一个直观预告: 由这些构造得到的 `GP motion-prior factors` 会直接进入连续时间估计的图模型中。作者也提醒，与向量空间情形一样，具体的 control-point state 还可能包含额外的轨迹导数，这取决于一开始选定的 motion prior。

在 `Lie group` 上使用 `GP` 的做法，是像 spline 一样，在一组 control-point states 之间定义局部变量。图 `2.7` 用 `SE(d)` 给出了示意。设相邻控制点为 `T_k` 与 `T_{k+1}`，则可定义局部变量

```text
ξ_k(t_k) = 0
ξ_k(t) = Log(T(t) T_k^(-1))
ξ_k(t_{k+1}) = Log(T_{k+1} T_k^(-1))
```

也就是说，用来导出 kernel function 的 `SDE` 不再直接作用于全局位姿，而是作用于这些局部变量。以 `SE(d)` 上的 `random-walk prior` 为例，可选取如下 `SDE`：

```text
ξ̇_k(t) = w(t),   w(t) = GP(0, Q δ(t - t'))
```

这里需要注意，`SDE` 是用 control points `T_k` 与 `T_{k+1}` 之间的局部变量来定义的。对这个 `SDE`，其状态转移函数就是 `Φ(t, s) = I`，因此做随机积分后有

```text
ξ_k(t) = ξ_k(t_k) + ∫_(t_k)^t w(s) ds
```

进一步取均值与协方差后，就得到对应的 motion prior

```text
ξ_k(t) ~ GP(0, min(t, t') Q)
```

若将 control-point poses 按每隔 `T` 秒均匀放置，则对应的 inverse kernel matrix 可写成

```text
K^(-1) = Φ^(-1) Q^(-1) Φ^(-T)
```

其中

```text
Φ^(-1) = [ I           ]
         [ -I   I      ]
         [      ..  .. ]
         [         -I I]

Q = diag(K(t_1, t_1), TQ, ..., TQ)
```

这说明即使使用 `GP`，其逆核结构依然非常稀疏。作者随后给出了每一段误差项在局部变量中的写法：

```text
e_k = Log(T̄_1 T_1^(-1)),                 k = 1
e_k = ξ_(k-1)(t_k) - ξ_(k-1)(t_(k-1)),   k > 1
```

其中 `T̄_1` 表示某个先验初始位姿。在全局变量中，同样的误差可写为

```text
e_k = Log(T̄_1 T_1^(-1)),   k = 1
e_k = Log(T_k T_(k-1)^(-1)), k > 1
```

图 `2.6` 展示了这种 `random-walk GP motion prior` 在 factor graph 中的样子。与上一节讨论线性 spline 时一样，如果我们想在其他时刻查询轨迹，也可以使用 `GP interpolation`。对于 `random-walk prior`，最终结果再次是线性插值 [47]：

```text
T(t) = (T_k T_(k-1)^(-1))^(α_k(t)) T_(k-1)
```

其中

```text
α_k(t) = (t - (k - 1)T) / T,   (k - 1)T ≤ t < kT
```

这里与 spline 方法的一个关键区别是，这种线性插值并不是通过显式选择某个 spline basis 人为指定出来的，而是由一开始所选的 `SDE` 间接导出的。若最初采用更高阶的 `SDE`，那么插值就会自然对应更高阶的 splines。

最后一个需要解决的问题，是如何把这些误差项线性化，以用于 `MAP estimation`。作者再次使用前面介绍过的 `Lie group perturbation` 方法。以式 `(2.73)` 的第二种情况为例，可写成

```text
e_k = Log( Exp(ξ_k) T_k^0 T_(k-1)^0^(-1) Exp(-ξ_(k-1)) )
    ≈ Log(T_k^0 T_(k-1)^0^(-1)) + ξ_k - Ad_(T_k^0 T_(k-1)^0^(-1)) ξ_(k-1)
```

其中 `T_k^0` 与 `T_(k-1)^0` 是当前估计，`ξ_k` 和 `ξ_(k-1)` 是待求的 perturbations，`Ad(·)` 则是 `SE(d)` 上的 adjoint。这个线性化形式就能在每次迭代中直接插入标准的 `MAP` 估计框架。

另外，如果在这个 `random-walk` 例子中，我们希望用式 `(2.74)` 减少 control points 数量，那么也可以沿用前面线性 spline 的同一套做法，因为这两种方法最终都退化成 `SE(d)` control points 之间的线性插值。作者最后总结说，spline 与 `GP` 在 `Lie group` 上做连续时间估计时，最大的区别在于: `GP` 方法会显式引入 motion-prior terms 来正则化问题，而 spline 方法本身并不会自动提供这种正则项。

## 2.3 延伸阅读与最新趋势

关于 manifold，尤其是 `Lie groups` 上的状态估计，已经有非常丰富的文献。机器人学里最具代表性的基础参考之一，是 Chirikjian 的系列工作。这些书在理论上扎实而系统，讨论了如何在 manifolds 上处理不确定性，甚至包括全局层面的建模。若想更深入理解 manifold optimization，Boumal 的著作是非常好的参考，且特别照顾到机器人应用语境。对于本章这种“局部不确定性 + `Lie group` 优化”的处理方式，Barfoot 的书则给出了更贴近工程实践的系统阐述。

连续时间估计方面，Talbot 等人的综述系统梳理了参数化与非参数化方法；Barfoot 的材料也对非参数方法做了较为详细的讨论。

与上一章类似，本章中的许多主题其实都会在后续章节继续出现。例如，如何在 manifold 上处理离群点、如何把算法变成可微的、以及如何把这些状态表示工具用于不同类型的传感器，都会在后文中被进一步展开。因此，本章可以视为现代 `SLAM` 状态表示与轨迹建模工具箱的基础准备。

