# 第 11 章 面向 SLAM 的惯性里程计

### 本章概览

`Inertial Measurement Units (IMUs)` 已成为机器人 `SLAM` 中最普遍的里程计来源之一。`IMU` 测量其所附着刚体的线加速度与旋转角速度。`IMU` 具有非常宽的形态、成本和性能跨度: 从装在飞机上的大型高精度光学传感器，到智能手机与消费设备中的小型但更嘈杂的 `micro-electromechanical systems (MEMS)`。尤其是 `MEMS IMU` 的 low-SWAP（尺寸、重量、功耗低）和低成本特性，使其极其适合作为机器人传感器，并且它们在 `SLAM` 中已经被持续研究了二十多年。

本章首先介绍 `IMU` 的基本事实及其 measurement model（`11.1`）；随后介绍 `IMU preintegration`（`11.2`），以便将高频 `IMU` 数据加入 `factor graph optimization` 框架。接着，作者指出使用 `IMU` 会给优化引入额外变量，例如 sensor biases，因此还需讨论 `IMU` 与外感知传感器（如 camera 或 `LiDAR`）组合系统的 observability properties（`11.3`）。最后，本章展示现代 IMU-centric `SLAM` 系统能实现的能力（`11.4`），并回顾近期趋势（`11.5`）。

## 11.1 惯性感知与导航基础

一个 `6-axis IMU` 包含 accelerometer 和 gyroscope，分别测量传感器相对于惯性参考系的线加速度与角速度。在航空航天领域，惯性导航系统（`INS`）长期被系统研究，其目标是利用初始状态和一段时间内的 `IMU` 测量来估计平台的当前状态，如位姿和速度。原书在这一节一开始就强调，`IMU` 覆盖了从大型高精度光学传感器到小型但更嘈杂的 `MEMS` 传感器的完整谱系；正是后者的 low-SWAP 与低成本，使其特别适合作为机器人系统的传感器。

`INS` 可分为 strapdown 系统和 stabilized 系统。前者中，`IMU` 刚性安装在平台本体上；后者则通过云台或其他结构尽量维持其朝向不随载体一起变化。机器人领域绝大多数系统属于前者，因此常直接把惯性导航称为惯性里程计（inertial odometry），以强调其本质上也是一种会随时间累积漂移的 odometry。

显然，单靠 `IMU` 做 inertial navigation 会随着时间快速漂移，所以在大多数机器人应用中，它都会与其他传感器联合使用，例如 `GPS`、camera 或 `LiDAR`。这时便形成 aided inertial navigation system（`AINS`）。在实际机器人系统中，我们往往直接用“`visual-inertial odometry`”“`visual-inertial SLAM`”这类名称来指代和 `IMU` 结合的具体传感器组合。

原书这里还给了三条脚注性质的重要补充。第一，`observability` 的作用，是说明在什么条件下估计问题是 well-posed 的，也就是是否有可能根据测量恢复出接近 ground truth 的估计。第二，一个 `IMU` 通常还包含 compass，用于测量磁北方向；但在室内和城市等机器人常见环境中，由于大型金属结构与电子设备引起的局部磁扰会造成很大偏置，因此这类传感器在 `SLAM` 里反而较少使用。第三，虽然在航空航天里会严格区分非惯性导航坐标系与惯性坐标系，但在近地、小尺度、低成本 `IMU` 的机器人应用中，地球自转带来的影响通常远小于测量噪声，因此机器人里常把固定在地球某处的 world frame 近似视作 inertial frame。

### 11.1.1 感知原理与测量模型

一个标准 `IMU` 通常包含 `3-axis accelerometer` 和 `3-axis gyroscope`，分别测量角速度与线加速度。Gyroscope 的基本设计原理来源于角动量守恒；accelerometer 则借助质量块的惯性，测量传感器相对于惯性系的运动加速度与重力加速度之间的差值。实际加速度计的实现方式可以很多，例如把 rate gyroscope 安装成摆式结构，或在低摩擦腔体内布置 proof mass，亦或通过比较悬挂金属带的振动差异来感知加速度。

接下来作者给出了标准 `IMU` measurement model。为简化讨论，假设传感器坐标系与机器人 body frame `F^b` 重合，而 world frame `F^w` 近似视作 inertial frame。此时时刻 `t` 的 `IMU` measurements `a(t)` 与 `ω(t)` 通常写成

```text
a(t) = R_w^b(t) (a^w(t) - g^w) + b^a(t) + η^a(t)      (11.1)
ω(t) = ω_b^b(t) + b^g(t) + η^g(t)                     (11.2)
```

其中，`R_w^b(t)` 描述 `IMU` 在时刻 `t` 的姿态，`a^w(t)` 是传感器在世界系中的加速度，`g^w` 是世界系中的重力向量，因此 `R_w^b(t)(a^w(t) - g^w)` 表示 `IMU` 在 body / `IMU` frame 中感受到的 specific force。`ω_b^b(t)` 则是 body frame 相对世界系的瞬时角速度，并以 body frame 表示。`η^g(t)` 与 `η^a(t)` 被建模为 zero-mean Gaussian white noise，而需要一起估计的 `b^g(t)`、`b^a(t)` 则通常被建模为 slowly varying random walks。作者还特地说明，这里的上标 `g` 和 `a` 表示传感器类型，即 gyroscope 与 accelerometer，而不是坐标系。

虽然式 `(11.1)` 与 `(11.2)` 在机器人里已经是非常实用的模型，但若考虑高精度系统或重新标定传感器平台，就需要引入更精细的误差项。由于制造不完美，accelerometer 可能存在 misalignment 与 scale errors，此时模型可扩展为

```text
a(t) = T^a R_w^b(t) (a^w(t) - g^w) + b^a(t) + η^a(t)  (11.3)
```

其中 `T^a` 是 accelerometer 的 shape matrix，用来同时描述 misalignment 与 scale errors。类似地，gyroscope measurement model 也可写成

```text
ω(t) = T^g ω_b^b(t) + b^g(t) + η^g(t)                 (11.4)
```

其中 `T^g` 对应 gyroscope 的 misalignment 与 scale errors。

此外，gyroscope 往往还会受 acceleration 影响，这一现象称为 `g-sensitivity`。若这种影响不可忽略，模型还可进一步扩展为

```text
ω(t) = T^g ω_b^b(t) + T^s R_w^b(t) (a^w(t) - g^w) + b^g(t) + η^g(t)   (11.5)
```

这里 `T^s` 是 `g-sensitivity matrix`，通常也需要通过 calibration 估计。作者的总体结论是: 简化模型在机器人里常已足够，但在高精度惯性导航、重新标定，或需要严格统计一致性的场景下，这些扩展误差项不能被忽略。

### 11.1.2 初始对准

在 `SLAM` 中，我们常把世界坐标系原点设在轨迹起点，并把初始位姿设为单位位姿；但在惯性导航中，由于 `IMU` 测量显式依赖重力，因此世界系通常要与重力方向对齐。换句话说，初始姿态不再是任意可选的，它必须与重力一致。

若机器人初始静止，则低成本 `MEMS IMU` 通常只能测到重力，因此可以从局部重力向量中恢复 roll 和 pitch，但无法恢复绕重力轴的转角，也就是 yaw。书中用 Gram-Schmidt 正交化给出了如下静态初始化公式:

```text
z_w^b = g^b / ||g^b||
x_w^b = (e_1 - z_w^b e_1^T z_w^b) / ||e_1 - z_w^b e_1^T z_w^b||
y_w^b = z_w^b × x_w^b
=> R_w^b = [x_w^b  y_w^b  z_w^b]                               (11.6)
```

其中 `e_1 = [1 0 0]^T`，这里执行的正是 Gram-Schmidt 正交化，而 `×` 表示叉乘。直观地说，旋转矩阵 `R_w^b` 的最后一列 `z_w^b`，就是世界系 `z` 轴在机体系中的方向；由于世界系 `z` 轴与重力对齐，所以 `(11.6)` 先由机体系下测得的重力向量 `g^b` 求出 `z_w^b`，再补出 `x_w^b` 和 `y_w^b` 来完成一个任意 yaw 选择下的正交基。

对高端 `IMU` 来说，gyroscope 还可以感受到 Earth rotation rate，因此可以同时利用重力方向和地球自转角速度做解析对准。若把机体系下的重力向量记为 `g^b`、地球自转角速度记为 `ω_ie^b`，则有

```text
g^b = R_w^b g^w
ω_ie^b = R_w^b ω_ie^w
g^b × ω_ie^b = R_w^b (g^w × ω_ie^w)
=> R_w^b = [ g^{wT} ; ω_ie^{wT} ; (g^w × ω_ie^w)^T ]^(-1)
           [ g^{bT} ; ω_ie^{bT} ; (g^b × ω_ie^b)^T ]          (11.7)
```

原书还特别指出，实际由该式得到的旋转矩阵通常还需要投影回 `SO(3)`，以缓解测量噪声的影响。无论使用哪种方式，alignment 都是惯性估计的第一步，因为后续全部积分都建立在这一参考系选择之上。

## 11.2 IMU 预积分与 Factor Graph（因子图）

前一节已经介绍了 `IMU` 的测量模型，它将 `IMU` 观测与机器人状态，尤其是位姿、速度和 bias 联系起来。原则上，我们当然可以像第 1 章那样基于这些模型直接写出一个 `MAP` 估计器；问题在于，`IMU` 频率通常极高，可能在 `200-1000 Hz` 之间。若为每个采样时刻都在因子图中加入一个状态变量，图会在短时间内变得极其庞大，几乎无法求解。

因此，`IMU preintegration` 的核心价值就在于: 把两个关键帧之间的大量原始 `IMU` 读数压缩成一个相对运动约束，使高频数据不必以原样全部进入图中。更关键的是，普通积分方式在图优化过程中一旦线性化点变化，常常就得从头重积分；而预积分的设计，恰恰就是为了避免这一点。

`Figure 11.1` 直观展示了 `IMU` 与 camera 的采样频率不同。图引自 [335]（©2016 IEEE）。

### 11.2.1 运动积分

本节首先讨论如何仅从 `IMU` 测量出发推断机器人运动。为此，作者引入如下连续时间运动学模型:

```text
Ṙ_b^w = R_b^w (ω_b^b)^
v̇_b^w = a_b^w
ṗ_b^w = v_b^w                                              (11.8)
```

它描述了机体系 `F_b` 相对于世界系 `F_w` 的旋转 `R_b^w`、平移 `p_b^w` 和速度 `v_b^w` 的演化。

若 `Δt` 是 `IMU` 的采样周期，那么时刻 `t + Δt` 的状态可由对 `(11.8)` 积分得到:

```text
R_b^w(t + Δt) = R_b^w(t) Exp(∫_t^{t+Δt} ω_b^b(τ) dτ)       (11.9)
v_b^w(t + Δt) = v_b^w(t) + ∫_t^{t+Δt} a_b^w(τ) dτ
p_b^w(t + Δt) = p_b^w(t) + ∫_t^{t+Δt} v_b^w(τ) dτ          (11.10)
```

其中，在第一式里作者假设区间 `[t, t + Δt]` 内角速度方向不变。若进一步假设在该短时间区间里 `a_b^w` 和 `ω_b^b` 都近似常值，那么可写成更简单的离散传播形式:

```text
R_b^w(t + Δt) = R_b^w(t) Exp(ω_b^b(t) Δt)
v_b^w(t + Δt) = v_b^w(t) + a_b^w(t) Δt
p_b^w(t + Δt) = p_b^w(t) + v_b^w(t) Δt + 1/2 a_b^w(t) Δt²   (11.11)
```

更一般地说，式 `(11.11)` 可以理解为对 `(11.9)` 所示积分应用了 `Euler integration`。再结合式 `(11.1)` 与 `(11.2)`，即可把 `a^w` 与 `ω^b` 写成 `IMU` measurements 的函数，于是上式变成

```text
R(t + Δt) = R(t) Exp((ω~(t) - b^g(t) - η^{gd}(t)) Δt)
v(t + Δt) = v(t) + gΔt + R(t)(a~(t) - b^a(t) - η^{ad}(t)) Δt
p(t + Δt) = p(t) + v(t)Δt + 1/2 gΔt²
           + 1/2 R(t)(a~(t) - b^a(t) - η^{ad}(t)) Δt²       (11.12)
```

这里作者为了简洁起见省略了坐标系下标，此后记号已足够明确。需要注意的是，对速度和位置的这种数值积分默认在两次 measurement 之间的积分期间姿态 `R(t)` 保持不变；对于非零角速度，这并不是微分方程 `(11.8)` 的精确解。不过在实践中，高频 `IMU` 往往能减轻这一近似带来的误差。作者也因此采用 `(11.12)` 作为后续建模与不确定性传播的基础，因为它足够简单，同时又便于推广；更高阶的积分技巧将在 `11.2.3` 再讨论。

作者还指出，离散时间噪声 `η^{gd}` 的协方差与采样率有关，并满足 `Cov(η^{gd}(t)) = (1/Δt) Cov(η^g(t))`；对 `η^{ad}` 也有同样关系。

从形式上看，式 `(11.12)` 已经可以直接当成 `factor graph` 中的概率约束。但如果真的这么做，就必须以极高频率在图中加入状态。直观地说，`(11.12)` 关联的是 `t` 与 `t + Δt` 两个时刻的状态，而 `Δt` 本身就是 `IMU` 的采样周期，这意味着每来一帧 `IMU` measurement，就得在估计问题里新增一个状态，这在计算上通常不可接受。

为缓解这一问题，可以考虑在更长时间区间上做积分。特别是，当问题中已经有其他传感器的 `factor graph`，例如第 7 章中的视觉测量时，就可以只在图中的两个相邻状态之间累计 `IMU` 测量。作者把这些状态称为 `keyframe states`。如果把式 `(11.12)` 在关键帧 `t_i` 与 `t_j` 之间所有的 `Δt` 区间上迭代应用，就得到:

```text
R_j = R_i ∏_{k=i}^{j-1} Exp((ω~_k - b_k^g - η_k^{gd}) Δt)

v_j = v_i + gΔt_{ij}
    + Σ_{k=i}^{j-1} R_k (a~_k - b_k^a - η_k^{ad}) Δt

p_j = p_i
    + Σ_{k=i}^{j-1} [v_k Δt + 1/2 gΔt²
    + 1/2 R_k (a~_k - b_k^a - η_k^{ad}) Δt²]               (11.13)
```

其中，作者为了书写简洁，引入了记号 `Δt_ij = Σ_{k=i}^{j-1} Δt`，并令 `(·)_k = (·)(t_k)`。式 `(11.13)` 已经给出了从 `t_i` 到 `t_j` 的运动估计，但它仍有一个关键缺点: 一旦关键帧 `i` 处的线性化点改变，例如 `Gauss-Newton` 每轮迭代更新了 `R_i`，那么后续所有 `R_k, k = i, ..., j - 1` 都会随之变化，导致 `(11.13)` 中的乘积和求和都必须重新计算。

原书这里还有两条脚注式补充。第一，之所以使用 `keyframe states` 这一术语，是因为在许多含 `IMU` 与 camera 的应用中，factor graph 里的状态通常只会加在 camera frames 的一个子集上，也就是 keyframes（见第 `7` 章）；但这里并不失一般性，`keyframe states` 完全可以按任意策略实例化，例如每个 camera frame、每次 `LiDAR` scan，或者每隔 `n` 个 `IMU` measurements 加一个状态。第二，为了简化讨论，这里假设 `IMU` 与其他传感器是同步的，且 `IMU` measurements 恰好采样在 `t_i` 和 `t_j`；实际系统中可以通过插值来近似这一情况，关于 temporal synchronization 的进一步讨论见 `11.4.3`。

这正是下一节引入 `IMU preintegration` 的动机。预积分的目标，不只是把高频 `IMU` 数据压缩成两个关键帧之间的一条约束，更重要的是让这条约束在优化中对线性化点变化不那么敏感，从而避免每次迭代都重做整段积分。

### 11.2.2 流形上的 IMU 预积分

流形上的 `IMU preintegration` 的关键思想，是把两个关键帧之间的相对运动增量写在局部坐标系中，并把重力和全局状态依赖项从测量中分离出去。如此一来，得到的“预积分量”不再依赖全局位姿与速度线性化点，因此在图优化过程中更加稳定，也不需要随着每次状态更新都重新从头积分。

更具体地说，书中定义了相对旋转、相对速度和相对位置增量 `ΔRij`、`Δvij`、`Δpij`。这些量被刻意构造为只与 `IMU` 测量有关，而不显式依赖起点时刻的绝对姿态、位置和速度。正因如此，它们能天然形成连接两个关键帧状态的“复合测量”。

#### 11.2.2.1 预积分 IMU 测量

式 `(11.14)` 已经把关键帧 `i` 与 `j` 的状态关系写成了“左边是状态、右边是 measurements”的形式，因此从概念上它已经可以被视为 measurement model。问题在于，它对 measurement noise 的依赖非常复杂，而 `MAP` estimation 又要求我们明确写出 measurements 的概率密度及其 log-likelihood。因此，作者在本节重新整理式 `(11.14)`，把每个惯性测量中的 noise terms 显式隔离出来。这里先假设时刻 `t_i` 的 bias 已知。

对 rotation increment `ΔR_ij`，作者首先用到 `SO(3)` 指数映射的两个性质:

```text
Exp(ϕ + δϕ) ≈ Exp(ϕ) Exp(J_r(ϕ) δϕ)                    (11.16)
Exp(ϕ) R = R Exp(R^T ϕ)                                (11.17)
```

第一条是指数映射对和的一阶近似，第二条可由群的 adjoint 表示推出。利用这两条关系，就可以把 `ΔR_ij` 中的 noise “移到最后”，写成

```text
ΔR_ij = ΔR̃_ij Exp(-δϕ_ij)                              (11.18)
```

其中 `ΔR̃_ij` 是预积分 rotation measurement，`δϕ_ij` 是对应噪声。

接着处理 velocity 与 position。这里还要用到

```text
exp(ϕ^) ≈ I + ϕ^                                        (11.19)
a^ b = - b^ a,   ∀ a,b ∈ R^3                           (11.20)
```

其中第一条是在原点处的一阶近似，第二条是向量 wedge operator 的基本性质。把 `(11.18)` 代回式 `(11.14)` 中的 `Δv_ij`，再使用 `(11.19)` 并忽略高阶噪声项，可得

```text
Δv_ij = Δṽ_ij - δv_ij                                  (11.21)
```

同理，把 `(11.18)` 与 `(11.21)` 再代回 `Δp_ij`，就得到

```text
Δp_ij = Δp̃_ij - δp_ij                                  (11.22)
```

最终，将 `(11.18)`、`(11.21)`、`(11.22)` 统一代回原始定义后，便得到真正可用于 factor graph 的预积分 measurement model:

```text
ΔR̃_ij = R_i^T R_j Exp(δϕ_ij)
Δṽ_ij = R_i^T (v_j - v_i - g Δt_ij) + δv_ij
Δp̃_ij = R_i^T (p_j - p_i - v_i Δt_ij - (1/2) g Δt_ij^2) + δp_ij   (11.23)
```

这里复合 measurement 由待估计状态加上一项随机噪声构成，噪声向量为 `[δϕ_ij^T, δv_ij^T, δp_ij^T]^T`。作者总结说，这样重写后的优势在于: 只要噪声分布合适，这些 measurements 就可以直接作为连接状态 `i` 与 `j` 的因子加入 factor graph。

#### 11.2.2.2 噪声传播

接下来需要推导噪声向量 `[δϕ_ij^T, δv_ij^T, δp_ij^T]^T` 的统计量。虽然我们已经知道把它近似为零均值 Gaussian 很方便，但真正关键的是准确建模它的 covariance。作者因此把预积分 measurement noise 写成

```text
η^Δ_ij = [δϕ_ij^T, δv_ij^T, δp_ij^T]^T ~ N(0, Σ_ij)     (11.24)
```

先看 rotation noise。由 `(11.18)` 可写成

```text
Exp(-δϕ_ij) = Π_k Exp(-ΔR̃_(k+1,j)^T J_r^k η_k^gd Δt)   (11.25)
```

对两侧取 `Log` 并改变符号，再使用 `SO(3)` 上对数映射的一阶近似

```text
Log(Exp(ϕ) Exp(δϕ)) ≈ ϕ + J_r^(-1)(ϕ) δϕ              (11.27)
```

反复应用后，就得到一阶近似下的

```text
δϕ_ij ≈ Σ_k ΔR̃_(k+1,j)^T J_r^k η_k^gd Δt              (11.28)
```

因此 `δϕ_ij` 是零均值 Gaussian。随后，`δv_ij` 和 `δp_ij` 也就容易处理了，因为它们都只是 acceleration noise `η_k^ad` 与 preintegrated rotation noise `δϕ_ij` 的线性组合。书中给出

```text
δv_ij ≈ Σ_k [ -ΔR̃_ik (ã_k - b_i^a)^ δϕ_ik Δt + ΔR̃_ik η_k^ad Δt ]   (11.29)
```

`δp_ij` 也可类似写成 `δv_ik`、`δϕ_ik` 与 `η_k^ad` 的线性组合。由此，`η^Δ_ij` 整体就是各个单次 `IMU` measurement noises 的线性函数，因此只要已知 `η_k^d = [η_k^gd, η_k^ad]` 的 covariance，就可以通过简单线性传播得到 `Σ_ij`。

作者还提到，[335] 给出了更完整的推导，并提供了一个可以随着新 measurements 到来而迭代更新 covariance 的形式，这种写法对 online inference 更友好。

#### 11.2.2.3 融合偏置更新

前面暂时假设预积分时所用的 bias `{b̄_i^a, b̄_i^g}` 是正确且保持不变的；但在真实优化过程中，bias estimate 往往会发生一个小更新 `δb`。一种直接做法是 bias 一变就重新预积分，但那样代价太高。作者采用的办法，是在 `b̄` 处完成一次预积分，然后对 `b ← b̄ + δb` 做一阶修正:

```text
ΔR̃_ij(b_i^g) ≈ ΔR̃_ij(b̄_i^g) Exp((∂ΔR_ij/∂b^g) δb_i^g)                    (11.30)
Δṽ_ij(b_i^g, b_i^a) ≈ Δṽ_ij(b̄_i^g, b̄_i^a) + (∂Δv_ij/∂b^g) δb_i^g + (∂Δv_ij/∂b^a) δb_i^a
Δp̃_ij(b_i^g, b_i^a) ≈ Δp̃_ij(b̄_i^g, b̄_i^a) + (∂Δp_ij/∂b^g) δb_i^g + (∂Δp_ij/∂b^a) δb_i^a
```

这些 Jacobians 描述了 measurements 对 bias 更新的敏感度，并且都可以在 preintegration 阶段预先计算并缓存下来。其推导方式与 `11.2.2.1` 中把 measurements 写成“大量值 + 小扰动”的做法非常类似，详见 [335]。

#### 11.2.2.4 预积分 IMU 因子与偏置模型

有了 `(11.23)` 的 measurement model，以及一阶近似下零均值 Gaussian 的 measurement noise `(11.24)`，就可以直接写出 factor graph 里的 `IMU` residuals。书中把它们记为

```text
r^I_ij = [r^T_(ΔR), r^T_(Δv), r^T_(Δp)]^T ∈ R^9
```

并展开为

```text
r_(ΔR) = Log( ΔR̃_ij(b̄_i^g) Exp((∂ΔR_ij/∂b^g) δb_i^g)^T R_i^T R_j )
r_(Δv) = R_i^T (v_j - v_i - g Δt_ij)
         - Δṽ_ij(b̄_i^g, b̄_i^a) + (∂Δv_ij/∂b^g) δb_i^g + (∂Δv_ij/∂b^a) δb_i^a
r_(Δp) = R_i^T (p_j - p_i - v_i Δt_ij - (1/2) g Δt_ij^2)
         - Δp̃_ij(b̄_i^g, b̄_i^a) + (∂Δp_ij/∂b^g) δb_i^g + (∂Δp_ij/∂b^a) δb_i^a   (11.31)
```

把这些 residuals 通过 `Σ_ij` 加权后加入目标函数，就得到连接关键帧 `i` 与 `j` 的 `IMU factor`。

最后，bias 本身还必须作为状态的一部分进行建模。由于前文已经说明 bias 是 slowly time-varying quantity，因此作者采用 `Brownian motion`，也就是 integrated white noise 模型:

```text
b˙g(t) = η^bg
b˙a(t) = η^ba                                          (11.32)
```

在离散关键帧间，这又会形成额外的 bias evolution factor，用来约束 `b_i` 与 `b_j` 之间的变化。

在离散关键帧间，这会形成一个额外的 bias evolution factor，用来约束 `bi` 和 `bj` 之间的变化。因此，在现代 visual-inertial / inertial `SLAM` 系统中，`IMU` 不只是一个 propagation 工具，而是一个带有自身状态、噪声模型、演化方程与跨时刻约束的完整概率组件。

### 11.2.3 高级预积分技术

本节讨论标准预积分方法的一些局限，并介绍更新的替代方案。我们先看式 `(11.14)` 中隐含的信号与运动假设，再回顾若干用于缓解这些假设的工作；这些方法在辅助惯性导航中可带来更准确的预积分 measurements，从而提升 localization 与 mapping 的精度。作者也特别说明，这里不逐一展开各方法的完整推导，若需更详细处理应直接参考对应论文。

#### 11.2.3.1 数值积分精度

标准预积分在离散时间上依赖 `Euler method`，用它把惯性信号积分为离散时刻上的旋转、速度与位置 pseudo-measurements。这种做法速度快、实现简单，但也会把积分误差直接带进预积分过程，最终表现为额外漂移。

从数值分析角度看，`Euler integration` 本质上就是对输入信号应用 rectangle rule。也就是说，它把连续信号近似成按采样频率分段常值的若干小矩形。对惯性系统来说，这些采样值就是 accelerometer 与 gyroscope measurements。图 11.2 左侧正是用这种方式说明 signal approximation 的。

当 sampling frequency 不够高时，分段常值假设就不能很好地逼近真实输入信号。这样一来，积分误差会快速累积，尤其在对加速度做双重积分得到位置时更明显，图 11.2 右侧就展示了这种误差增长。

`Figure 11.2` 展示了在已知初始条件下使用 `Euler integration` 的结果：上排对应较低采样频率，下排对应较高采样频率。

一种直接补救办法，是提高采样频率，使矩形近似更精细。但在真实惯性导航系统中，采样频率终究受 `IMU` 硬件能力限制，不可能无限制提升。

为此，文献 [630] 提出使用 `Gaussian Process (GP) regression` 来对输入信号做“虚拟上采样”，使 gyroscope 与 accelerometer 数据都可以在任意选定时间戳上被估计出来。这样一来，数值积分会比标准预积分更准确。不过，作者也指出，这类做法本质上仍然建立在 piecewise-constant 的数值积分思路之上，并没有真正发挥 `GP` 连续模型的全部潜力。也正因此，后续工作才进一步发展出更复杂的连续时间积分方法。

这里原书还脚注说明：`GP regression` 是一种 non-parametric、probabilistic interpolation approach，想更深入理解可参考 [908]。

#### 11.2.3.2 连续加速度预积分

另一条路线，是用连续时间表示直接逼近旋转校正后的加速度信号，并对其做解析积分。由于连续时间表示可在任意时间查询，因此它尤其适合与不同步传感器，乃至 event camera 这类完全异步的模态联合使用。

书中特别说明，连续时间预积分通常会先把 rotation 与 translation 两部分暂时拆开处理，因为 rotation space 的非交换性使其更难直接纳入经典积分工具。于是，本节主要讨论在“旋转部分已先求解”的前提下，如何对 translation component 做连续时间预积分，而 rotation 的连续时间处理则留到下一小节。

作者提到几类代表性做法。一类工作先利用零阶积分器处理 gyroscope measurements，再把 velocity 和 position 的预积分写成连续时间线性时变系统，并分别假设 constant accelerometer measurements 或 constant local acceleration。与标准预积分里常见的 `constant global acceleration` 假设相比，`constant local acceleration` 更贴近真实运动，因此在 `EuRoc` 一类数据集上能带来可观的 `VIO` 精度改进。

若想进一步放松 constant-acceleration 假设，就可以直接用解析可积的连续函数逼近输入信号。书中提到两类典型做法。一类是假设旋转校正后的加速度为 `piecewise-linear`，也就是近似 `constant jerk`。这时，从加速度到速度的第一次积分就退化为经典梯形积分规则，已经能比 Euler 带来显著精度提升。另一类则更进一步，用 `GP` 直接建模旋转校正后的加速度信号。由于 `GP` 在线性算子作用下仍保持 `GP` 结构，因此可以解析地推导积分和双重积分结果。

作者给出的示例显示，这种非参数、`model-less` 的连续表示，在速度和位置积分精度上都优于分段线性方法。不过，`GP` 中 kernel 的超参数会直接控制信号平滑度，因此既可以通过数据学习得到，也可以用经验方式设定。

`Figure 11.3` 的上排展示了基于分段线性近似的连续积分，对应 constant-jerk motion assumption；下排展示了使用 `Gaussian Process` regression 的 model-free 连续积分。

#### 11.2.3.3 连续旋转预积分

相比平移部分，rotation preintegration 更难，因为旋转属于 `SO(3)` Lie group，不具备欧式空间的交换性，很多经典积分工具不能直接使用。为此，有工作将旋转表示映射到 Lie algebra 中的 rotation vector，再在这个线性空间里用 `GP` 建模连续旋转函数，并借助虚拟观测值来拟合旋转动态。

作者先强调问题的根源: 解旋转运动学所需的 product integral 在一般情形下没有已知的通用闭式解，因此要想做 continuous-time、model-less 的 rotation preintegration，就必须引入新的表示和优化思路。

一条代表性路线，是把旋转 `R(t)` 改写为 Lie algebra 中的 `rotation vector` `r(t)`，满足 `R(t) = Exp(r(t))`。这样，旋转就被映射到了一个局部线性空间中，可以借助线性工具进行连续建模。在该空间里，系统动力学写成与 `SO(3)` 右 Jacobian 相关的形式；但麻烦在于，`r` 和 `rdot` 本身都不是 `IMU` 直接观测到的量。

关键想法是: 用 `GP` 去建模 `r`，同时引入一组虚拟观测值作为连续旋转轨迹的 control points，再通过一个非线性最小二乘问题，使这些虚拟观测与 gyroscope measurements 共同满足旋转动力学约束。这样得到的，就是一种连续时间、无显式运动模型的旋转预积分方法。书中指出，这类方法相较标准离散预积分，精度提升可达到至少一个数量级。

作者还把它与第 2 章提过的 `STEAM` 连续时间状态估计做了比较。两者都在 Lie algebra 中使用 `GP` 做插值，但这里使用的是 square exponential kernel，因此会得到稠密线性系统；不过对 `IMU` 预积分而言，单个积分窗口通常足够短，所以求解稠密系统在实践中仍可接受。后续工作还进一步将 rotation vector 与旋转校正后的加速度联合估计，从而让预积分协方差显式体现旋转与平移之间的相关性。

## 11.3 辅助惯性导航的可观测性

即使测量模型与预积分都构造正确，如果问题本身在某些方向上不可观，估计器仍然不可能可靠工作。因此，除了“如何融合 `IMU`”，另一件必须搞清楚的事就是“哪些量本来就不可观”。这正是 observability analysis 要回答的问题。

### 11.3.1 线性化测量模型

这里考虑的测量模型假设是: 用来辅助 `IMU` 的外感知传感器，例如 camera 或 `LiDAR`，输出的是几何特征。换言之，作者关注的是那些会生成 landmark-based representations 的 `SLAM` 或 odometry 前端。虽然多数 `AINS` 主要使用点特征，尤其在依赖 camera 的系统中更常见 [456, 652, 644, 895, 375, 335]，但在条件允许时也可以引入线与平面特征 [598, 455, 414, 1235]。这样一来，待估计状态向量就往往需要同时包含机器人自身状态与外部几何特征状态。

更具体地说，书中把每个时刻待估计的 `AINS` 状态写为机器人状态 `x_b` 与以世界坐标系表达的外部特征状态 `x_f^w` 的组合:

`x = {R_b^w, b^g, v^w, b^a, p^w, x_f^w}`  。`(11.37)`

其中 `R_b^w` 是 body frame `F_b` 相对于 world frame `F_w` 的旋转，`p^w` 与 `v^w` 是以世界系表示的位置和速度，`b^g` 与 `b^a` 分别是 body frame 下的 gyroscope bias 与 accelerometer bias，而 `x_f^w` 则可以由点、线、平面，或它们的组合构成。

为了开展后续的可观测性分析，既需要系统动力学模型，也需要外感知测量模型。作者下面先回顾由 `IMU` 驱动的惯性导航运动学，再给出外感知测量方程的线性化形式。

#### 11.3.1.1 线性化 IMU 运动学模型

基于前文 `(11.8)` 与 `(11.32)`，`IMU` 的连续时间运动学写为:

`Ṙ_b^w = R_b^w (ω^b)^,   v̇^w = a^w,   ṗ^w = v^w` ，

并令两类 bias 服从 random walk:

`ḃ^g(t) = η^bg,   ḃ^a(t) = η^ba` 。

其中 `η^bg` 与 `η^ba` 是驱动 gyroscope bias 与 accelerometer bias 的零均值 Gaussian 噪声。为了分析可观测性，作者把这一非线性系统在线性化点附近展开，得到连续时间误差状态模型:

`x̃̇(t) ≃ F(t) x̃(t) + G(t) η(t)` 。

误差状态向量可写为 `x̃ = {θ̃, b̃^g, ṽ^w, b̃^a, p̃^w, x̃_f^w}`。其中旋转部分不是直接在线性空间中表示，而是在当前线性化点的 tangent space 中用 `θ̃` 表示。直观上，这相当于把真实旋转写成线性化点附近的一个小扰动，并使用小角度近似

`R_b^w ≃ R̂_b^w (I + θ̃^)`

来完成线性化。这里 `η(t)` 是堆叠后的噪声项，不仅包含 `η^bg` 与 `η^ba`，也包含把真实角速度与加速度替换成 gyroscope / accelerometer measurements 时引入的 `IMU` 噪声。

由于实际 `AINS` 往往以离散时间实现，还需要从连续时间系统得到离散状态转移矩阵 `Φ_(k+1,k)`，它由

`Φ̇_(k+1,k) = F(t_k) Φ_(k+1,k)`

并以单位阵为初始条件积分得到。书中给出了 `Φ_(k+1,k)` 的块结构形式: 与姿态、速度、位置和 bias 相关的各个子块之间存在标准耦合，而特征子状态对应的传播块则保持为单位阵。各个块 `Φ_ij` 可以解析求出，也可以用数值方式计算 [456]。这一离散转移矩阵随后会与测量 Jacobian 一起构成 observability matrix 的基本组件。

#### 11.3.1.2 外感知测量模型

现在作者给出不同几何特征的测量模型及其线性化形式，因为这些模型是开展线性化 `AINS` 可观测性分析的基础。

对点特征，外感知传感器，如 monocular / stereo camera、声纳和 `LiDAR`，通常都可以统一为 `range-and-bearing` 观测。若特征在传感器坐标系 `Fc` 中的相对位置为 `p_f^c`，则测量可以写成距离与方位的组合，并通过一个带有二值项 `λr`、`λb` 的 selection matrix `Λ` 来表示“当前传感器到底提供 range、bearing，还是两者同时提供”。在当前状态估计附近线性化后，可通过链式法则得到形如 `ẑp ≈ Hx x̃ + η̃x` 的误差方程；根据 `Λ` 的不同取值，Jacobian `Hx` 可以只包含 range 项、只包含 bearing 项，或同时包含二者。

对线特征，作者先用 `Plücker coordinates` 表示由两个三维点定义的直线:

`l^w = [nℓ^w; vℓ^w] = [p1^w × p2^w; p2^w - p1^w]`

其中 `nℓ^w` 是 line moment，`vℓ^w` 是直线方向向量。该直线可从世界系变换到当前相机系，再通过已知内参投影到图像平面。若图像中观测到的线段端点为 `q1` 和 `q2`，则视觉线特征测量可定义为这两个端点到投影线的距离。随后同样可用链式法则线性化，得到对应的测量 Jacobian。

对平面特征，则用平面法向量与原点距离来参数化:

`π^w = [nπ^w; dπ^w]`

它可以被变换到局部传感器坐标系。对点云传感器，如 `LiDAR` 或 depth sensor，作者采用平面到原点的最近点 `pπ^c = dπ^c nπ^c` 作为状态中的平面表示，因此测量可写成 `zπ = pπ^c + ηπ`。对该表达在线性化后，也可得到相应的 plane measurement Jacobian。

这些测量模型在当前状态估计附近线性化后，就能与离散 `IMU` 误差状态模型共同组成统一的 observability analysis 框架。其意义在于，很多状态量究竟是否可观，并不取决于某一类传感器单独存在，而取决于惯性传播与外感知约束是否共同提供了足够信息。

### 11.3.2 可观测性分析

基于前面线性化后的动力学模型与测量模型，现在可以正式进行 observability analysis。作者使用的工具是 observability matrix `M(x̂)` [486]:

`M(x̂) = [ H_x1 Φ_(1,1);  H_x2 Φ_(2,1);  ... ;  H_xk Φ_(k,1) ]` 。

其中 `H_xk` 堆叠了离散时刻 `k` 收集到的所有测量 Jacobian，可能来自点、线或平面特征；记号 `M(x̂)` 则强调 observability matrix 依赖于当前线性化点 `x̂`。该矩阵的 null space `U`，也就是所有满足 `M(x̂) u_i = 0` 的向量所张成的空间，精确描述了 `AINS` 的不可观子空间。若 `U` 为空，系统完全可观；否则，对应的 null vectors 就指出了哪些状态方向无法由现有测量恢复。

已有研究表明 [1233]，一般的 `AINS` 通常具有 `4` 个不可观方向，也就是 null space 中存在四个线性无关向量。这四个方向对应全局 `3D` 位置与全局 yaw 无法由 `IMU` 测量和对未知地标的局部观测唯一确定。

为了更清楚地理解这四维 null space 的结构，作者考虑一个同时把三类几何特征都放入状态向量的例子，即 `x_f^w = {p_f^w, l_f^w, π^w}`，而外感知测量则包含 `z = {z_p, z_l, z_π}`。将相应系统 Jacobian 与测量 Jacobian 代入上式后，可以构造出线性化的 `AINS observability matrix`，并显式求得其 null space `null(M)`。书中给出了四个 null vectors 的具体表达式 `(11.51)`。虽然表达式本身较长，但它们的几何含义非常清楚: 第一个 null vector `u_1` 对应绕重力方向的旋转，因此就是 yaw 不可观；其余三个向量 `u_2:4` 对应系统整体在空间中的平移自由度。

作者还进一步解释了这些向量中的组成部分。例如，`u_g` 与重力方向和初始速度有关；`R_π^w` 是用平面法向量通过 `Gram-Schmidt orthonormalization` 构造出的旋转矩阵；`R_ℓ^w` 则由直线法向与方向向量构成。通过这些构造，可以把点、线、面三类特征在不可观子空间中的耦合关系写成显式形式。

总结来说，observability matrix 存在四维 null space，准确地描述了 `AINS` 的全局位置与 yaw 不可观这一事实。直观上，`IMU` 数据以及对未知点、线、面地标的相对观测，都不会告诉系统它相对于某个绝对世界系的原点和绝对 yaw 在哪里。唯一的例外是 roll 与 pitch，它们可以通过 accelerometer 对重力方向的测量变得可观。

这种不可观性在 `SLAM` 中非常常见，并不是 pathological failure，而只是说明我们可以任意设定世界坐标系的原点与 yaw，因为对应变量只以相对形式被观测。一旦引入提供绝对观测的传感器，例如 `GPS`，这种不确定性就会消失。更值得警惕的是，在某些运动模式或某些线性化点附近，observability matrix 的 null space 还可能继续增大，产生额外的不可观维度；这正是下一节将讨论的内容。

### 11.3.3 退化运动

比四个标准不可观方向更令人头痛的是，某些运动模式会进一步扩大 observability matrix 的 null space，从而引入新的不可观方向。作者将这类情况称为 degenerate motions。

例如，纯平移会使系统的全局姿态都变得不可观，因为缺少转动时，重力测量和 accelerometer bias 之间更容易发生混淆；而常加速度、纯旋转，以及对 monocular 相机而言“朝着特征点正向前进”的运动，则会导致尺度不可观或特征尺度不可观。这里作者还特别说明，这三类退化结论成立的前提是：传感器到特征的距离要显著大于传感器与机器人机体之间的外参平移（若二者并不重合），也就是通常满足 `||p_f^c|| >> ||p_b^c||`。尤其对 monocular visual-inertial 系统来说，这些退化运动会直接影响是否能稳定恢复 scene scale。

作者还补充指出：若完全不使用 `IMU`，那么 landmark-based `SLAM` 的 null space 至少会是 `6-dimensional`，因为此时除了整个系统的 `3D position` 之外，完整的 `3D rotation` 也同样不可观。

表 `11.1` 总结了 `AINS` 的退化运动。

| 运动（Motion） | 传感器（Sensor） | 不可观量（Unobservable） |
| --- | --- | --- |
| 1. 纯平移（Pure translation） | 通用（General） | 全局朝向（Global orientation） |
| 2. 常加速度（Constant acceleration） | 单目相机（Mono cam） | 系统尺度（System scale） |
| 3. 纯旋转（Pure rotation） | 单目相机（Mono cam） | 特征尺度（Feature scale） |
| 4. 朝着点特征前进（Moving toward point feature） | 单目相机（Mono cam） | 特征尺度（Feature scale） |

因此，一个好的估计系统不仅依赖传感器质量，也依赖 sufficiently exciting 的运动激励。很多时候，问题并不是“算法失效”，而是机器人当前的运动本身没有向系统提供足够的信息。

## 11.4 视觉惯性里程计与实践考虑

前面几节主要讨论的是惯性建模与理论分析，本节则把这些内容落到最常见的实际系统中: `Visual-Inertial Odometry (VIO)`。作者将讨论 `VIO` 的典型因子图结构、系统性能特点，以及外参标定与时间同步等工程上不可回避的问题。

### 11.4.1 视觉惯性里程计

正如前文所述，惯性测量通常会与其他传感器融合，以减轻纯 inertial odometry 的漂移。本节重点讨论将 camera 与 `IMU` 通过 factor graph 融合的情形。Camera 和 `IMU` 是非常流行的一对组合，因为它们都 inexpensive、lightweight、low-power，而且高度互补: `IMU` 擅长捕捉快速 acceleration 与 rotation，camera 则擅长提供丰富的环境观测信息。一方面，camera 的加入显著降低了纯惯性解的漂移；另一方面，`IMU` 又能让某些纯视觉下不可观的量变得可观，例如 monocular `SLAM` 中的场景尺度，只要运动本身不是退化的。

作者也提醒，在机器人领域中，惯性数据同样常与其他传感器结合使用，包括 `LiDAR` 和 radar，可参见第 `8` 章与第 `9` 章。

`VIO` 通常被当作 odometry source 使用，也常被直接用于 trajectory tracking 与 control loop closure。在另一些应用，例如虚拟现实，`VIO` 则负责补偿用户在虚拟环境中的运动。无论是哪种应用，系统都要求非常低的延迟，典型量级在 `10-50 ms`。作者举例说，`Meta Quest 3` 的刷新率大约在 `72-120 Hz` 之间，因此 `VIO` latency 会直接影响用户体验，甚至影响 motion sickness；而对轨迹跟踪控制来说，过大的延迟则会诱发控制器不稳定。

`Figure 11.4` 展示了一个使用 preintegrated `IMU` factors 的 visual-inertial odometry factor graph 示例 [335]：紫色为 preintegrated `IMU` factors，用于约束连续位姿、速度与 bias；蓝色为 bias factors，用于约束 `IMU` bias 随时间的演化；橙色为视觉因子，用于关联 camera poses 与外部 landmarks；黑色则是 priors。

正因如此，基于 factor graph 的 `VIO` 系统通常采用 `fixed-lag smoother`，也就是 `sliding-window optimization`。系统只估计一个 receding horizon 内的状态，例如最近 `5-10` 秒，而不是整条历史轨迹。图 `11.4` 展示了一个典型因子图: 紫色是 preintegrated `IMU` factors，用于约束连续关键帧之间的 pose、velocity 与 bias；蓝色是 bias factors，用于约束 `IMU` bias 随时间的演化；橙色是视觉因子，把 camera poses 与外部 landmarks 连接起来；黑色则是 priors。

滑动窗口长度的选择，本质上是在 computation 和 accuracy 之间权衡。窗口越长，理论上状态估计越精确，但优化问题也越大。随着时间推进，超出窗口的 factors 和 variables 会被逐渐 marginalize 掉。为了进一步减少优化维度，许多实现还会用 `Schur complement` 把 visual landmarks 从状态中消除，例如 [335]。另一条路线是使用像 `iSAM2` 那样的 incremental solver，复用历史优化结果来减少重复计算。作者指出，这种方法在实践中确实可能非常准确 [335]，但它难以为系统提供 latency upper bound，因为在某些时刻运行时间可能出现 spike，这对某些实时应用是不利的。

过去十年里，open-source 的 visual-inertial odometry / `SLAM` 系统大量涌现，代表方法包括 visual-inertial 版 `ORB-SLAM` [785]、`Direct Sparse Visual-Inertial Odometry` [1115]、`VINS-Mono` [895]、`OpenVINS` [375]、`Kimera` [3, 941]、`BASALT` [1117] 和 `DM-VIO` [1042]。作者给出的经验标准是: 一个好的 `VIO` 系统，漂移应低于路径长度的 `1%`，优秀系统则可达到 `0.1%` 左右。

书中还给出一个具体例子: 较新的 `FEJ`-based `Window Bundle Adjustment (WBA)-VINS` [180, 181] 在 `KAIST Urban Dataset` [514] 的 sequence `38` 上运行，处理的是一条长达 `11.42 km`、总时长约 `36` 分钟的城市轨迹。该数据集由搭载 stereo cameras、`2D/3D LiDARs`、`Xsens IMU`、`FoG`、wheel encoders 和 `RTK GPS` 的车辆采集，camera 频率为 `10 Hz`，`IMU` 频率为 `100 Hz`，并通过 `FoG + RTK GPS + wheel encoders` 提供 ground-truth trajectory。该 `VIO` 系统在不使用 loop closure 的纯在线条件下，最终取得约 `2.05` 度和 `21.2 m` 的 `ATE`，相当于整条轨迹长度的约 `0.18%`。

### 11.4.2 外参标定

若 `IMU` 与 camera 之间的相对位姿，也就是 extrinsic calibration，不准确，那么再好的后端也只能在错误模型上优化。因此，外参标定是 aided inertial navigation 成败的关键之一。

相关方法大致可分为 offline calibration 和 online calibration 两类。Offline 方法要求在部署前专门执行标定流程，可能需要标定板、已知运动模式，或某些环境先验。这类方法往往更准确，但流程繁琐，且常依赖专门设备和训练有素的操作者。Online 方法则把外参参数直接并入状态估计问题，在系统运行过程中同时估计，这样一旦传感器装配略有位移，也不必重新专门标定。

不过，online calibration 的代价是问题规模更大，而且在某些运动与观测配置下，外参甚至可能变得不可观，从而让状态估计本身变复杂甚至 ill-posed。作者因此提醒，在线标定并非总是“更方便的替代品”，而是一个需要谨慎设计的估计问题。

### 11.4.3 时间同步

除了空间外参，时间同步同样是惯性辅助系统中的关键问题。若不同传感器之间存在未建模的时间偏移，轨迹估计会产生显著系统误差，benchmark 指标也可能带入虚假 bias。尤其在高动态运动中，哪怕只有几毫秒偏差，也可能显著恶化残差和最终精度。

时间同步可以通过硬件或软件完成。硬件方案通常依赖公共时钟信号触发各传感器采样，但并非所有设备都支持这种同步引脚。软件方案中，一个典型例子是基于以太网的 `PTP (Precision Time Protocol)`；另一些系统则在传感器端打时间戳，并在后处理中对齐。后一种方式通常精度和鲁棒性都不如硬件同步。

如果系统无法严格同步，且又必须在线运行，那么也有方法把时间偏移直接作为状态变量纳入估计问题。对现代 `VIO` 而言，这往往是非常实际的设计选项。

## 11.5 延伸阅读与最新趋势

尽管 inertial odometry 的进展已经在不断走向工业产品，但 `aided inertial navigation` 依旧是研究非常活跃的方向。作者最后概括了几类值得持续关注的新趋势。

第一类是 `Extended Pose Preintegration`。最近的惯性里程计研究，开始使用 extended-pose manifolds 和更高阶的噪声传播，来改进 `IMU preintegration` 的不确定性建模。例如 `Brossard` 等人的工作把地球自转、科里奥利力和离心力都纳入了预积分理论；`Vial` 等人则展示了一个结合线速度传感器和导航级 `IMU` 的 extended pose preintegration 例子，在长达一小时、总长约 `1.8 km` 的海上导航轨迹中，平移误差仅约 `5 m`。

第二类是 `Continuous-time State Representations`。本章主要从预积分角度出发，把 `IMU` 看成减少离散状态变量数量的工具；但也有另一条路线，是直接采用连续时间状态表示，在不增加估计状态维度的前提下吸收大量高频惯性测量。典型例子包括基于 `B-spline` 的表示和基于 `GP priors` 的表示。两者都允许在一组固定状态变量之间，通过插值动力学残差把高频 `IMU` 测量纳入优化。作者还提到，近期工作比较了“把 `IMU` 直接作为连续时间 `GP prior` 的输入”和“把 `IMU` 直接作为 residual measurements”这两种方式，结果表明，后者在 `LiDAR-inertial` 组合中能得到更好的 odometry 精度。还有工作进一步比较了前文讨论过的 `continuous GP-based preintegration` 与标准 `GP-based state representation`，并在 event-based `VIO` 场景中发现，前者在精度和计算效率上都略占优势。

第三类趋势是 `Proprioception-only Odometry`。近年的一些系统开始主要依赖本体感觉传感器来辅助惯性导航。例如在腿式机器人上，可以利用足端接触状态；在轮式平台上，可以利用 wheel-mounted `IMU` 的运动学约束，从而得到误差低于百分之一量级的 `IMU` 里程计。在前一种情况下，关键先验是“足与地面的接触关系”；在后一种情况下，则利用“单平面旋转运动”来约束 `IMU` 偏置，从而抑制 dead-reckoning drift。后者还被进一步扩展成完整 `SLAM` 系统，通过识别随时间变化的道路横滚模式来检测 loop closure，这构成了一个很有意思的近纯惯性、本体感觉 `SLAM` 例子。作者也提醒，尽管惯性传感器通常更稳健，但 `IMU` dropout 或传感器饱和有时会给整个系统造成灾难性影响。为此，也有工作研究在 gyroscope 饱和时，改用 accelerometer 数据去估计角速度，以提升后续 `SLAM` 算法的鲁棒性。

第四类趋势是 `Inertial-only Odometry (IOO)`。如果没有视觉等外部辅助，仅仅对 `IMU` 测量做朴素积分，里程计通常会很快发散。这不仅是纯惯性系统的问题，也是在视觉暂时失效时，视觉惯性系统必须面对的现实风险。例如在移动 AR/VR 中，快速运动的手可能离开跟踪相机视场；或者在无纹理环境里，视觉前端根本无法稳定提取和跟踪特征，只能暂时完全依赖 `IMU`。因此，近年来大量工作开始尝试用学习方法和神经网络来降低 `inertial-only odometry` 的漂移，包括用数据驱动方式建模 `IMU bias`、直接从一段带噪 `IMU` 序列中预测位移，或者在可微积分模块中先估计并移除 bias 再做积分。还有工作使用真实 bias 监督，或者用 conditional diffusion model 把 bias 当作概率分布去近似。作者认为，这些方法已经显示出大幅降低纯惯性漂移的潜力，但泛化能力目前仍有限，例如很难无缝适应不同传感器或训练中未出现过的运动模式。

最后一类趋势，是面向嵌入式边缘平台的 `ultra-efficient and robust VIO`。即便 `SLAM` 算法本身不断进步，小型嵌入式机器人仍然面临严苛的 `SWAP`（size, weight, and power）约束。作者特别指出，在许多系统里，真正昂贵的并不只是运算本身，而是数据管理，尤其是 `RAM` 数据访问。例如在 `Meta XR` 可穿戴设备的 `SLAM` 与 hand-tracking 模块里，主要能耗往往来自内存访问而非纯计算。为降低数据搬运代价，研究者开始探索 `on-sensor computing` 架构，也提出了量化版 `VIO (QVIO)`。对于只具备单精度浮点运算能力、或者必须依赖单精度才能满足实时性的低 `SWaP` 平台，新近也出现了 `square-root` 形式的 information / covariance filters，用于在保持数值稳定性的同时提升效率。此外，还已有面向芯片级 visual-inertial odometry 的 `ASIC` 设计与实现工作。

总体而言，`IMU` 仍然是现代 `SLAM` 系统中最关键的动态骨架之一。但今天真正困难的问题，已经不再是“能否把 `IMU` 用进系统”，而是如何让惯性信息在统计上一致、在系统上鲁棒、在算力与功耗上都可控。

