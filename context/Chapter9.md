# 第 9 章 Radar SLAM

### 本章概览

本章讨论在 `SLAM` 中使用 radar（`RAdio Detection and Ranging`）。与 cameras 和 `LiDAR` 相比，radar 长期以来相对“采用不足 / 使用不足（undersubscribed）”；但由于它能够在低可见度条件下工作、具备远距离感知能力，并且原生就能产生速度信息，因此其受关注程度正在上升。

本章将依次讨论机器人中常见的 radar 传感器类型、它们独特的感知原理，以及伴随而来的若干挑战（`9.1`）；随后介绍 radar filtering、radar odometry（`9.2`）、place recognition（`9.3`）以及完整的 `Radar SLAM`（`9.4`）。Figure `9.1` 展示了这些模块如何组合在一起。最后，作者会讨论 radar `SLAM` 数据集（`9.5`）以及 radar 在未来的发展前景（`9.6`）。

Figure `9.1` 的核心含义是: radar-based `SLAM` 的信息流与其他 `SLAM` 系统总体模式相同，但每个处理环节都需要针对 radar 的特性进行调整。原始 radar measurements（来自 spinning radar 或 phased-array radar）先经过 radar filtering，形成 pseudo-measurements；前端利用这些量完成里程计估计，地点识别模块输出 loop-closure constraints；后端再通过基于 `factor graph optimization` 的 `Radar SLAM` 联合估计最终 pose 与 map。

## 9.1 Radar 简介

本节介绍机器人中最常见的两类 radar: 旋转式 radar（spinning radar）与 `system-on-a-chip (SoC) radar`，并回顾 `mmWave`、`FMCW`、`Radar Cross Section (RCS)`、Doppler 效应与角度估计等基础概念。

### 9.1.1 传感器类型

机器人里常见的两大 radar 类别，主要差异在于它们如何发射、接收并整理回波。旋转式 radar 通过机械方式转动单一天线；`SoC radar` 则通过多发射/多接收天线阵列配合片上处理来推断目标角度、距离和速度。两类传感器各有优劣，其输出数据形式也完全不同。

Figure `9.2` 在这里提供了一个很有信息量的对比。图中展示了机器人中最常见的两类 radar：spinning radar（左上）与 phased-array `SoC radar`（右上），以及它们各自生成的主要数据产品；这些原始数据之后通常还会经过 radar filtering，转成稀疏 point cloud（下方）。图右下角给出了一个“充满烟雾的隧道”例子，其中红色是 `LiDAR` 点云，由于烟雾影响而量程明显受限；蓝色是 `2D spinning radar` 点云，可以在远距离清晰看到隧道壁；绿色是 `SoC radar` 生成的 `3D` 点云，能够看到车辆前方的墙面和地面。图左下角则给出了同一场景下的 thermal image、可见光 / `RGB` 图像，以及从中提取蓝色点云的 polar radargram。

#### 9.1.1.1 旋转式 Radar

旋转式 radar 也常被称为 scanning radar 或 imaging radar。它借助转动 radar 传感器逐步扫描周围环境，从而形成精确的极坐标表示，典型输出就是 `polar radargram`。这类成像 radar 的一个突出优点，是量程通常很远，许多设备能够探测到 `100` 米以上的目标；在某些工作模式下，还可以利用 Doppler 效应估计这些目标的速度。

与 `LiDAR` 相比，旋转式 radar 的一个明显局限，是它通常只能在二维平面内提供数据，无法直接测量被探测目标的高度。即便回波实际上可能来自天线垂直波束宽度覆盖的不同高度，这一限制依然存在。此外，由于它常常在每个天线方位角只发射和接收一次脉冲，因此有些设备并不直接提供速度信息，因为速度估计需要多个脉冲；较新的工作则会利用多个相邻方位角的信号来恢复速度。

作者还指出，像当前广泛使用的 `3D LiDAR` 那样，构造带有多条垂直波束的机械旋转式 `3D radar` 在工程上并不可行，因为 radar 的天线和聚焦机构相对于激光二极管系统要大得多。

#### 9.1.1.2 SoC Radar

`System-on-a-chip (SoC) radar` 将处理单元集成在少量芯片上，这些芯片要么直接安装在贴片天线阵列上，要么集成在印刷电路板中。由于其结构紧凑且没有机械运动部件，`SoC radar` 通常更轻、更省电，也更适合车载和机器人部署。

这类传感器的性能在很大程度上取决于天线阵列设计，以及厂商如何将多个天线的测量进行专有信号处理。其主要输出通常是 `radar datacube`。在表示上，`SoC` 回波往往以方位角、俯仰角和距离为球坐标轴来组织；再加上径向速度这一维后，常被称为 `3+1D` 或 `4D radar`。若垂直方向分辨率有限，只能得到距离、方位与速度，则通常被归为 `2+1D` 阵列 radar。

### 9.1.2 Radar 感知原理

本节回顾 radar 在机器人中的一些基本工作方式、天线结构、数据类型和核心挑战，并说明 radar 相较于其他测距传感器的独特性。`MmWave (millimeter-wave) radar` 指电磁波波长约为 1 到 10 毫米、频率约在 30 到 300 GHz 之间的雷达系统。由于汽车高级驾驶辅助系统（`ADAS`）对 76 到 81 GHz 频段的产业推动，很多机器人 radar 也工作在这一频段。

#### 9.1.2.1 Radar Cross Section

`MmWave radar` 通过发射电磁波并接收目标反射回来的回波完成感知。目标反射这些电磁波的能力，通常由 `Radar Cross Section (RCS)` 描述。`RCS` 受目标材料、形状和尺寸影响，可以理解为“与目标具有相同反射能量的假想球体大小”。因此，大型、坚固的结构，如车辆或厚实混凝土墙壁，往往具有更高的 `RCS`；而小型物体或行人则更低。

所谓 radar intensity，是指 radar 从目标接收到的回波强度，它是 radar 发射功率与目标 `RCS` 的函数。从本质上说，这个强度量主要由 radar 的发射功率和所遇目标的 `RCS` 决定。文献中指出，这一强度量对于提取目标的语义细节或辅助导航十分重要，因为较强回波往往对应环境中更鲜明、更容易识别的结构特征 [1239]。此外，radar 信号强度还会受到天线类型、所发射电磁脉冲的特性，以及天线检测目标回波能力的影响。

#### 9.1.2.2 FMCW 测距

一个 radar 至少包含一个发射天线（`TX`）和一个接收天线（`RX`）。虽然有时 `TX` 与 `RX` 可以是同一个单元，但多数 `SoC` 传感器会把它们分开布置，以便集成多个 `TX` 和 `RX` 天线。在 `Frequency Modulated Continuous Wave (FMCW)` radar 中，`TX` 发射的是频率随时间稳定上升的射频脉冲，也就是 `chirp`。最常见的是 sawtooth modulation，后文也会讨论另一种基于 `chirp` 的 triangular modulation。`Chirp` 打到环境中的目标后产生反射，`RX` 接收这些回波，并将其与原始 `TX chirp` 混频或相减，生成 `Intermediate Frequency (IF)` 信号。

`Figure 9.3` 展示了 `Frequency Modulated Continuous-Wave (FMCW)` radar 的工作方式：系统发射 `chirps`，也就是频率递增或递减的波；这些波被目标反射后再由接收端接收。距离以及速度，都是通过分析反射信号相对于发射信号的频率偏移和相位偏移来确定的。常见的频率调制策略包括 sawtooth 和 triangular，其中后者的优势在于可以把 Doppler frequency shift 与 range shift 解耦。

由于发射与回波具有相同斜率，混频后的 `IF` 会变成频率近似恒定的正弦波 `sin(2π f0 t + ϕ0)`。其中 `f0` 只依赖目标距离，而 `ϕ0` 则表现为一个额外的相位偏移。因此，`FMCW` 的关键思想，是把时间信息编码到连续波上，从而可以据此执行测距计算。

作者特别指出，这一点与 amplitude modulation 很不同。后者的测量精度主要依赖频带宽度；而 frequency modulation 对 `signal-to-noise ratio` 和目标 `RCS` 变化通常更稳健。在 `FMCW radar` 中，一串带有明确时间间隔的 chirps 仍然保持相干性，这意味着每个 chirp 都能被准确地对应回其所属的发射-接收对以及它在序列中的位置。因此，系统不仅能在 amplitude 上关联信号，也能在 phase 上关联信号。这相比早期依赖非相干幅值测量、并常需人工从 clutter 中挑选有效信号的 time-of-flight pulsed radar，是一次很大的进步。

在公式上，若电磁波速度为 `c`，回波到达初始时刻为 `t0`，目标距离满足

```text
d = c t0 / 2
```

当发射 `chirp` 的带宽为 `B = fM - fm`、持续时间为 `T`、斜率为 `S = B / T` 时，中频 `f0` 与距离 `d` 之间可写成

```text
f0 = 2Bd / (Tc) = 2Sd / c  =>  d = c f0 / (2S)
```

虽然书中公式在频域写出，但真实系统中信号通常是以数字形式采集的。常见做法是使用高速 `Analog-to-Digital Converter (ADC)` 对回波进行高频采样，典型采样间隔可达到 `100` 纳秒甚至更短。随后，采样数据会经过一系列 `Fast Fourier Transform (FFT)` 运算，生成 frequency-amplitude 图，再从中识别信号峰值。对单个 `chirp` 及其回波执行 `FFT` 可以得到 range frequency；对多组 chirp 序列进一步做 `FFT`，则可用于提取 velocity frequency。

#### 9.1.2.3 用锯齿调制确定距离与速度

Sawtooth frequency modulation 常见于 `SoC radar` 和部分旋转式 radar。此时 `IF` 信号相位 `ϕ0` 可写为波长 `λ` 与距离 `d` 的函数:

```text
ϕ0 = 2π fm t0 = 4π d / λ
```

虽然 `f0` 和 `ϕ0` 都与距离有关，但相位只在距离变化很小且不发生相位包裹时更有用，因此它通常不直接用来做绝对测距，而是用来估计小的距离变化 `Δd`，也就是径向速度。

在 Doppler 速度估计中，至少需要连续两个 up-chirp。由于两个 chirp 之间时间间隔非常短，量级可到 40 微秒，因此它们的测距频率几乎相同，但相位会不同。相位差 `Δϕ` 对应目标在这段时间内的位移，从而得到

```text
Δϕ = 4π Δd / λ = 4π v T / λ
v = λ Δϕ / (4π T)
```

由此也能看出，最大无歧义速度受 `|Δϕ| < π` 限制，因此 `vmax = λ / (4T)`，它只由信号波长与 chirp 间隔决定。这解释了为何 radar 天然适合进行 Doppler-based 自运动估计。

`Figure 9.4` 说明 radar 可以利用 Doppler effect 测量设备与其正在成像的场景之间的径向速度。

#### 9.1.2.4 用三角调制确定距离与速度

Triangular frequency modulation（见图 `9.3`）也可以使用，例如它有时会用于旋转式 radar。实际上，当由一个 up chirp 导出中频时，其中包含两个部分：`(i)` 由信号传播时延产生的频移 `f0,t`，以及 `ii)` 当 radar 与目标之间存在相对速度时，由 Doppler 效应产生的频移 `f0,d`。因此有

```text
f0,up = f0,t + f0,d
```

然而，如果在一个 up chirp 之后再跟一个由同一目标反射回来的 down chirp，那么时间项对应的频移符号会翻转，而 Doppler 项的符号不会翻转：

```text
f0,down = -f0,t + f0,d
```

由这两个式子可以解出 `f0,t` 与 `f0,d`：

```text
f0,t = (f0,up - f0,down) / 2
f0,d = (f0,up + f0,down) / 2
```

最后，可据此计算距离和速度：

```text
d = c f0,t / (2S)
v = c f0,d / (2ST)
```

需要注意的是，前面式 `(9.2)` 中给出的距离计算并没有对 Doppler 效应进行校正，而这里的做法进行了校正。使用 triangular modulation 的代价，是距离和速度计算会引入更高延迟，因为此时需要用到稍旧的数据；但它的好处是我们不需要处理信号相位。

#### 9.1.2.5 SoC Radar 的角度估计

对旋转式 radar 而言，确定目标角度是很直接的，因为系统在每个时刻都只朝单一方位角聚焦波束。`SoC radar` 的角度估计则更复杂一些，但也可以通过与前面类似的相位差计算来完成。

若存在多个接收单元，它们之间相隔距离 `ℓ`，那么反射回波中就会出现传播距离差 `Δd`。根据几何关系 `Δd = ℓ sin θ`，第二个接收天线比第一个多走了一段 `ℓ sin θ` 的路径，如图 `9.5` 所示。这对应于两个 `RX` 天线接收到的信号之间具有如下相位差：

`Figure 9.5` 展示了 phased-array radar 的 angle-of-arrival estimation：`TX` 发射天线发出的信号到达两个相距 `ℓ` 的 `RX` 接收天线时，其相位差对应于目标的入射角 `θ`。

```text
Δϕ = (2π / λ) ℓ sin θ
```

给定测得的相位差 `Δϕ`，目标到达角 `θ` 可计算为

```text
θ = arcsin( λ Δϕ / (2π ℓ) )
```

原则上，一个 `TX` 和两个 `RX` 天线就足以确定目标角度；但如果需要更高分辨率，或者希望区分多个彼此接近的目标，就需要两个以上的 `RX` 天线。此时，返回信号在每个 `RX` 上都会额外累积一个 `Δϕ` 偏移。对这些 `RX` 通道上的信号序列进行采样，并对该序列执行 `FFT`，就可以较可靠地估计 `Δϕ`。

上面的例子实际上对应一种 `SIMO` phased-array radar，也就是 `single input, multiple output`。而用于 `SLAM` 的多数 radar 传感器则是 `MIMO`（`multiple input, multiple output`），具有多个 `TX` 和 `RX` 天线。只要 `RX` 天线能够区分来自不同 `TX` 的信号，就可以通过额外增加一个 `TX`，而不是把 `RX` 数量翻倍，来达到相同分辨率。为保证多个 `TX` 信号彼此不相关（orthogonal），可采用多种技术，例如 frequency division、code division，或 `time-division multiple access (TDMA)`，即让每个发射器使用不同时间槽。同样的原理还可以扩展到二维 `TX-RX` 阵列，同时测量 azimuth 与 elevation，从而产生 `3+1D` 数据。

### 9.1.3 Radar 应用面临的挑战

与任何传感器一样，radar 也有一系列需要认真对待的噪声与失真来源。相较视觉和 `LiDAR`，radar 特有的问题包括多种形式的 measurement noise、宽波束导致的距离偏置与稀疏读数、接收机饱和、`speckle noise` 和 `multipath` 等。这些问题都会直接影响后续的点云提取、配准、回环识别与建图质量。

`Figure 9.6` 展示了将 `(a)` 中的 polar radargram 变换到 `(b)` 中的 Euclidean coordinates 后的结果，并在 `(c)` 中放大显示了 radar 相比其他传感器更独特的几类噪声。最常见的是 speckle noise，图中绿色圆圈标出了具有歧义的 clutter；multipath reflections 则来自回波先撞击附近墙面或地面再返回天线，从而生成对真实目标的镜像反射，图中红圈标出了这类重复回波。原图取自 Mulran dataset [566]。

#### 9.1.3.1 Speckle 噪声

Radar 测量噪声来自多个来源，包括热噪声、电子缺陷以及目标 `RCS` 的变化。当 radar 发射一个电磁脉冲时，它会接收天线视场范围内所有物体反射回来的能量。这个脉冲与物体相互作用后会让 radar 波发生散射，并导致相长和相消干涉。这些作用既可能产生虚假信号，也可能抵消本来真实存在的合法回波，而不论这些信号最初来自哪里。

这些因素会使频域中的信号发生变化，其中最突出的峰值往往同时混合了真实目标和虚警。若缺少区分真实与虚假回波的机制，传感器最终就会产生一种散乱点状的输出模式，这就是所谓的 `speckle noise`。对于对位姿估计和特征匹配都很关键的 landmark 识别而言，通常必须跨多个 scans 去估计这些反射的不确定性。

#### 9.1.3.2 多路径效应

`Multipath` 是另一种典型误差来源。一个目标的回波并不一定沿最短路径返回，有些信号会先反射地面或墙面，再回到接收机。结果是，同一个真实目标可能对应多条不同传播路径，系统于是会感知到一些似乎位于墙后、地面下方或其他不真实位置的“鬼影目标（ghost objects）”。如果不在前端或后端中抑制这些离群观测，点云地图和定位结果都会受到显著影响。

#### 9.1.3.3 运动诱导畸变

和旋转式 `LiDAR` 一样，扫描式 radar 也会发生 `motion-induced distortion`。旋转式 radar 通常需要完成一整圈扫描才能形成一张极坐标图像，因此如果平台在这段时间内在运动，同一物体在一圈开始和结束时的测量位置就会不同。对于低频设备，这一问题尤其严重。例如常见的 Navtech 成像 radar 工作频率只有 4 Hz，在车辆平台上非常容易出现明显畸变；即便是更快的 10 Hz 或 50 Hz 设备，只要平台速度足够高，畸变仍不可忽视。

### 9.1.4 Radar 滤波

由于 radar 中虚假目标的分布会随时间变化，而且参数往往不可预测，因此静态阈值这类简单方法很难稳定工作。许多 radar 处理方法都要沿单个方位角分析一维信号，并判断哪些 range bins 对应真实目标，哪些只是噪声。

`Constant False Alarm Rate (CFAR)` 是最常见的一类方法。它将频域信号划分为若干 cell，以滑动窗口方式对“待测单元（CUT）”与其周围训练单元进行比较，从而动态估计背景干扰水平，并设置自适应阈值。经典版本如 `CA-CFAR` 使用均值统计量，`OS-CFAR` 则使用更鲁棒的排序统计量。`BFAR` 则是在 `CFAR` 的输出之上做简单仿射变换，以在自适应阈值和固定阈值之间取得更合适的折中。

不过，作者也指出，`Radar SLAM` 未必总是需要复杂滤波器。与传统目标检测相比，里程计更关心的是“哪些点能在不同时间、不同视角下稳定重复出现”。因此，很多系统采用更保守的策略，例如在每个方位角仅保留超过静态阈值的 `k` 个最强回波，其中 `k` 可从 `1` 开始逐步增加；也可以使用统计阈值，仅保留强度高于均值一个标准差的点。

`Figure 9.7` 给出了 radar filtering 的示例，对比了 `CFAR`、constant power threshold 和 `k-strongest` 三类策略。沿单个 azimuth 方向的 power/range 曲线以灰色绘出；真实目标回波会表现为尖峰，但其中也夹杂多个歧义峰值。`CFAR` 生成的自适应阈值用黑线表示，蓝点是 `CFAR` 报告出的 detections，在该例中包含了若干 false alarms；而 `k-strongest` 滤波器在这里取 `k = 12`，更保守，只返回主目标附近的点。图引自 Adolfsson 等人 [6]（©2021 IEEE）。

原文还补充了两类基于学习的代表做法。`Cheng` 等人 [195] 使用 `generative adversarial network (GAN)`，根据 `range-Doppler velocity matrices` 生成点云；`Xu` 等人 [1209] 则联合训练 regressor 和 classifier，其中 regressor 输出更高分辨率、质量更好的 depth readings，classifier 负责估计数据是否已经超出量程。它们的共同目标，都是从 radargram 中尽量只保留那些若换成 `LiDAR` 也会落在真实表面上的回波。

## 9.2 Radar 里程计

`Radar odometry` 的目标，是根据时间有序的 radar 观测估计传感器的自运动。典型方法会维护某种中间表示，如最近 `N` 帧读数、局部子地图或连续裁剪的局部地图，并关注局部尺度上的位姿估计精度。与其他模态一样，如果没有显式回环约束，再优秀的 radar 里程计也会随时间累积漂移。以旋转式 radar 为例，优秀方法的平移漂移量级通常仍在每 100 米约 1% 到 2% 左右。

`Figure 9.8` 给出了 Oxford Radar RobotCar dataset [127] 上的 open-loop radar odometry 示例，其中使用的是 `Navtech 2D spinning radar` 与 `CFEAR` 方法 [8]。蓝色曲线是真值轨迹，橙色曲线是里程计估计，灰色点是从 radargram 中提取的 point targets。

Radar 的一个独特之处，在于很多传感器能提供逐点 Doppler 速度估计，因此可构造几乎不依赖显式对应关系的 odometry。除此之外，作者还将现有方法概括为 direct、feature-based、registration-based，以及 motion compensation 几条路线。

### 9.2.1 Doppler 里程计

如果 radar 能直接测得目标的径向速度，那么就可以利用多个目标的 Doppler 约束来估计平台线速度。相关方法通常通过 `RANSAC` 和最小二乘来联合多个回波，得到对传感器速度的稳健估计。这种方法的优点，是物理意义明确，且不依赖传统 scan matching 中繁琐的点对应过程。

不过，Doppler 只能测量速度在视线方向上的投影，因此单个 radar 一般无法仅凭这些约束恢复完整角速度，除非利用额外运动学假设。例如，若已知 radar 相对车辆转动中心的位置，并假设系统遵循 Ackermann 转向且无侧滑，就可从 `2+1D` 数据中恢复线速度与角速度 [550]；`Galeote-Luque` 等人 [354] 进一步把这一思路扩展到 `3+1D`，可估计五个自由度，即三维线运动加上 `yaw` 与 `pitch`，但不包括 `roll`。更常见的做法，则是将 Doppler 估计与 `IMU` 结合，尤其使用 gyroscope 提供角速度信息。作者举例说，`Huang` 等人 [490] 将消费级 `IMU` 与级联 `SoC radar` 结合，在多样的室内 `3D` 空间中取得了较低漂移；`Kubelka` 等人 [608] 则比较了多种 `3+1D radar` 的配准式方法与 registration-free 的 Doppler + `IMU` 方法 [272]，发现后者在特征稀疏环境下误差最低，报告的 `4.5 km` 轨迹漂移可低至 `0.3%`。

`Figure 9.9` 给出了 radar-inertial velocity estimation system 的 factor graph 表示：系统在滑动窗口内联合估计过去 `K` 个时刻的状态，并使用 Doppler targets 与多组 `IMU` measurements 作为约束。图改编自 Kramer 等人 [605]。

由于 Doppler 型里程计是 radar 非常独特的一条路线，作者随后基于 `Kramer` 等人 [605] 给出一个具体的 Doppler factor graph 例子。其目标是在长度为 `K` 的滑动窗口内，联合估计过去若干时刻传感器平台在 body frame 下的速度，并利用 `IMU` 加速度计积分把这些速度状态连接起来。系统状态可写为

```text
x = [v_s^T, q_s^w^T, b^T]^T
```

其中 `v_s` 是平台速度，`q_s^w` 是用于表示姿态的 orientation quaternion，`b` 是 `IMU` 偏置。作者说明，这样建模的原因在于: radar 给出的速度估计本身不含偏置，但为了补偿重力对加速度计的影响，必须同时估计 `IMU` 的姿态，尤其是 `pitch` 与 `roll`。

在这一建模下，整体代价函数由 Doppler term 与 inertial term 组成；前者对每个 radar target 都引入一个速度残差，后者则由 `IMU` 误差构成。书中进一步给出了 Doppler 权重的定义，即用对应目标回波强度做归一化:

```text
w_d,k = P_d,k / Σ_(j∈D_k) i_j,k    (9.12)
```

其中 `w_d,k` 是第 `k` 帧中目标 `d` 的权重，`i_d,k` 是该目标的回波强度。

这里作者还补充说明：这类 filtering techniques 可以被看作与 `9.2.3` 节的 key-point extraction 类似，只是它们并不提取 descriptors。

在该例子中，一次 radar 观测由一组 targets `D` 组成；对每个 `d ∈ D`，测量量写为

```text
[r, v, θ, ϕ]^T
```

分别表示距离、Doppler 径向速度、方位角和俯仰角。Doppler 速度测量 `v` 等于目标与传感器之间相对速度向量 `v_s` 在从传感器原点指向该目标的单位方向 `r_d,k^s / ||r_d,k^s||` 上的投影，也就是

```text
(v_s)^T (r_d,k^s / ||r_d,k^s||)    (9.13)
```

作者在这里假设场景中的目标都是静止的，只有传感器平台自身在运动。于是，每个 radar target 都为 body-frame 速度估计提供一个约束，其 Doppler 速度残差写为

```text
e_d,k = v_d,k - (v_k^s)^T (r_d,k^s / ||r_d,k^s||)    (9.14)
```

其中 `v_d,k` 是第 `k` 个时刻目标 `d` 的测得径向速度，`v_k^s` 是该时刻传感器速度，`r_d,k^s` 是从传感器指向目标的向量。由于 radar 测量往往含有非高斯噪声和虚假目标，作者建议对 Doppler 残差配合 `Cauchy` robust norm 使用。书中还特别提醒，上述推导默认 radar 与 `IMU` 共点且朝向一致；而一般情况下两者并不重合，文献 [604] 给出了更一般的推导，展示了 body angular rate 与 radar 传感器坐标系速度之间的耦合关系。

### 9.2.2 直接里程计

`Direct radar odometry` 不先提取点、角点或地标，而直接对原始 radargram 进行对齐。典型做法会把两张极坐标 radargram 看成图像，通过 phase correlation、Fourier-Mellin transform 等经典信号处理工具估计相对位姿。

具体来说，在极坐标图像中，方位角变化对应垂直方向平移，因此可以先通过相位相关估计旋转；随后把图像变换到笛卡尔坐标，再估计平移。这类方法利用了几乎全部原始信号，不必做稀疏点提取，因此理论上能保留更多信息。

但直接法有一个关键假设: 同一空间位置的功率回波在时间上足够稳定，这样相关性才有意义。实际中，这个假设常因噪声、动态物体和 radar 特有的外观变化而被破坏。为此，有工作训练卷积神经网络，只保留 radargram 中真正稳定、适合相关匹配的区域，从而避免噪声纹理和虚假回波干扰对齐过程。

### 9.2.3 基于特征的里程计

由于 `2D spinning FMCW radar` 的 radargram 在形式上很像鸟瞰图像，因此一些工作直接引入计算机视觉中的特征提取与匹配思路，如 `SIFT`、`SURF`、`ORB` 等，也有专门为 radar 设计的关键点提取器和描述子，如 `FSCD` 和 `BASD`。

基于特征的方法，首先要找到可以在连续帧中稳定重复出现的关键点，再为其构造局部描述子，之后通过描述子匹配和 `RANSAC` 去除离群点，最后用 `SVD` 或其他最小二乘方法恢复两帧之间的刚体变换。相对于纯配准法，这类方法在初值较差时通常更稳，因为描述子匹配提供了更强的全局辨识能力，而不只是依赖几何邻近性。

但 radar 的局部外观比普通图像更不稳定、更噪，也更依赖观测视角和环境结构，因此特征设计远比视觉困难。近年也出现了使用深度网络自动学习 radar 关键点和描述子的工作，不过这类方法往往需要带 ground-truth 位姿的数据集，训练成本和泛化问题都较突出。

若想看更全面的综述，作者在这里特别提示可参考 [121]。

### 9.2.4 基于配准的里程计

基于配准的 radar odometry 与 `LiDAR` 里程计在整体思想上很接近: 先把原始 radar 数据转成 point clouds 或更丰富的局部分布表示，再利用 scan registration 恢复相对运动。不同之处在于，radar 点云往往更稀疏、噪声更强，因此配准难度通常明显高于 `LiDAR`。

对旋转式 radar 而言，首先必须从原始 signal data 中筛选哪些 returns 可视为有效目标，也就是从大量 range bins 中挑出 relevant peaks。前文 `9.1.4` 已讨论过一些典型方法，包括 `CFAR`、利用噪声统计去除冗余或噪声回波 [158]、`BFAR` [25]、或在每个 azimuth 上直接选取最强的 `k` 个 returns。`Kellner` 等人 [549] 还使用 `DBSCAN`，让点提取不仅沿单个方位角进行，也考虑相邻 azimuth 之间的聚类关系。近期还有一些方法借助 machine learning，把 radargram 映射成更接近 `LiDAR` 的 point cloud [195, 1209]。作者指出，这里的关键难点在于选点数量的平衡: 选得太少会丢失信息，选得太多又会把大量噪声带入后续估计 [158]；例如 `CFAR` 在这方面就被认为较难调参 [127]。

在点云抽取完成后，可以直接拿来配准，也可以进一步估计 normals、planes 或局部 point distributions 等附加信息。虽然整体流程和 `LiDAR` scan registration 很像，但 radar data 的稀疏性和噪声使得纯粹的 pair-wise registration 更困难。

因此，一条非常常见的策略，是把当前 radar scan 注册到多个历史 keyframes，或者注册到一个由多帧聚合形成的 local submap，而不是只与上一帧做 frame-to-frame alignment。这样做的好处有三点:

- 通过更多 correspondences 缓解点云稀疏问题。
- 在 feature-poor environments 中减少 drift。
- 对移动物体、遮挡或偶发错误对应具有更强鲁棒性。

作者列举了几类代表性方法。`Continuous-time ICP` [127]、`power-shifted NDT` [614] 和 `CFEAR` [8] 都属于利用 multiple keyframes 或 submaps 的 registration-based radar odometry。以 `CFEAR` 为例，它从 radargrams 中按每个 azimuth 选取最强的 `k` 个 returns 形成 point cloud，然后把每个新点云同时与最近 `s` 个 keyframes 配准。其误差可以是 `point-to-point`、`point-to-line` 或 `point-to-distribution`，类似 `NDT` registration。对每个点，系统还会根据邻域点协方差估计 normal vector，并结合 normal 一致性、平面性以及邻域点数量，对匹配关系进行加权。

`Kung` 等人的方法 [614] 则采用固定阈值从 radargram 中提点，并将多帧点云聚合成一个 `NDT` radar submap，其中每个点的贡献由其 signal strength 加权。`Burnett` 等人 [127] 使用 `BFAR` [25] 提取点云，再将其聚合进 local submap，随后通过 `continuous-time ICP` 结合 `Gaussian process` motion prior 来优化轨迹。

前述方法主要面向 `2D` radar。对最近的 `3+1D radar` 而言，pipeline 相似，但 point extraction 通常在传感器内部已完成，因此研究者较少再显式设计 peak detector。由于 `3+1D` point clouds 天然带有 per-point Doppler velocity，这类方法通常先根据 Doppler 数据做 least-squares ego-velocity estimation，再将明显不符合静态场景速度模型的点视为 moving objects 或 outliers 剔除，之后再用 registration 精配准。`4DRadarSLAM` [1270] 使用一种针对 radar point clouds 改造过的 `GICP`，通过协方差矩阵对远距离点赋予更高不确定性；`4D iRIOM` [1309] 采用 one-to-many distribution-to-distribution matching，以缓解 radar 点云噪声与稀疏性；`EFEAR-4D` [1198] 则把 `CFEAR` 的思路扩展到了 `3+1D` 场景。

### 9.2.5 运动补偿

由于扫描式 radar 一帧往往跨越明显时间范围，因此 `motion compensation` 或 `de-skew` 是前端不可或缺的组成部分。原文给出一个具体结果: 对于频率仅 `4 Hz` 的低速旋转式 radar，仅使用常速度模型进行 `egomotion compensation`，就可把 `ATE (Absolute Trajectory Error)` 降低 `29%` [8]。方法上，系统可根据相邻两帧的相对位姿估计出平台速度，再利用逐点时间戳把每个点补偿回统一时刻。这样对单帧中每个 radar point 做时间对齐，可以显著缓解扫描式 radar 在缓慢扫掠期间累积的运动畸变。原文还指出，同一个常速度模型通常也会被用来给刚体 scan registration 提供初值；当每一帧经过运动补偿后再与前一帧完成配准，就可以把该帧作为一个新节点加入 `SLAM` pose graph。

但这仍然是在离散 pose graph 设定中工作。若采用连续时间轨迹表示，就可以在每个传感器读数的真实时间上直接查询姿态，从而更自然地完成去畸变，并获得更平滑、更准确的轨迹。书中举了两个例子: `Ng` 等人 [802] 在 automotive `SoC radar` pipeline 中采用样条（spline）轨迹表示；`Burnett` 等人 [129] 则使用 `Gaussian processes`，把 spinning radar 与 `IMU` 组合进同一个 factor graph 轨迹模型中。

## 9.3 Radar 地点识别

与视觉和 `LiDAR` 一样，地点识别（place recognition, `PR`）是完整 `Radar SLAM` 的关键模块。它不仅关系到 loop closure detection，也决定系统能否在长期运行中进行重定位。作者指出，对 radar 而言，获取同时对平移和旋转变化保持不变的描述子格外困难，这也是 radar `PR` 比很多人想象中更难的原因。

`Figure 9.10` 展示了从同一位置获取的两次 radar scans。其噪声模式会引入明显的 visual aliasing，从而让 place recognition 变得更困难。

### 9.3.1 Radar 地点识别中的独特挑战

从表面上看，早先章节中的许多 `PR` 方法似乎都可以通过数据格式转换后用于 radar，但 radar 确实存在一系列独特挑战。首先，radar 分辨率较低，细节更少，可辨别的局部特征明显不足。其次，宽波束会带来角度模糊，远处目标常表现为较宽的亮斑，并且当观测距离变化时，其外观会发生显著变化。再加上 radar 的信噪比较低、接收机饱和可能形成强烈的径向条纹，以及噪声模式本身会制造 aliasing，同一地点从很近的两个位置看过去，也可能显得很不一样。

不同 radar 类型的 `PR` 设计也不相同。长距离、`360` 度覆盖的旋转式 radar 更适合借鉴视觉 `PR` 或 `2D LiDAR PR` 的思路，因为它能输出完整 `2D radargram` 或对应点云；而 `SoC radar` 往往视场角更小、量程更短、点云更稀疏、更噪，因此更常沿着 `LiDAR PR` 的方向修改，但必须特别处理小视场和高稀疏性带来的描述子不稳定问题。当前文献中旋转式 radar 的 `PR` 工作更多，一方面因为其宽视场和长量程更利于 outdoor `PR`，另一方面也因为大规模数据集更丰富。

### 9.3.2 基于学习的 Radar 地点识别

对旋转式 radar 而言，把 `2D radargram` 当作图像来处理是一条自然路线。早期工作曾使用 `CNN` 来学习雷达图像的表示，并通过圆柱卷积等设计显式建模极坐标结构，以获得对旋转变化的鲁棒性。查询 radargram 可以与数据库中的参考 radargram 匹配，从而构成基于外观的拓扑式 `PR` 系统；后续工作则进一步在检索后加上位姿细化，形成 topometric localization。

一类非常关键的训练技巧，是利用雷达数据天然按时间顺序采集这一特点，无需精确度量真值位置，也能构造近似监督信号。研究者可以把时间上相邻、位置上相近的帧视为正样本，再通过极坐标平移人工施加旋转增强；而把时间上相距较远的帧视为负样本。这样便能在无精确 metric ground truth 的条件下，以近乎无监督的方式训练 `PR` 网络。

对于 `SoC radar`，学习式 `PR` 往往先将点云投影到 `2D` 图像平面，或直接在 `3D` 点云上做网络编码。有工作使用深度时空编码器配合 `NetVLAD` 生成地点描述向量，并用 `RCS` 信息重排序，过滤不相关静态特征；也有工作直接在 `3D` 点云上使用 kernel point convolutions，并训练“点重要性估计器”来突出真正有助于地点识别的稀疏稳定点。还有方法在前端先利用 Doppler 一致性筛掉明显动态或噪声点，再用专门面向 radar 的 backbone，如 `MinkLoc4D`，替代原本为视觉设计的 `NetVLAD`。

### 9.3.3 基于描述子的 Radar 地点识别

除学习方法外，手工描述子依然在 radar `PR` 中很有效。一条常见路线，是借用 `LiDAR` 领域已有的全局描述子，如 `Scan Context`、`RING`、`M2DP`，再根据 radar 数据格式进行改造。由于旋转式 radar 缺少高度信息，很多 `LiDAR` 描述子中使用的高度统计要被替换成 `RCS`、强度和点密度等信息；同时还要结合噪声过滤与运动补偿，以提升可比性。

例如，有工作把 `M2DP` 从 `3D` 点云适配到由 radargram 提取的 `2D` 点云上，并借助 `PCA` 特征值来判断某个扫描是否“足够有辨识性”。如果一个扫描的点分布呈现明显单向延展，例如高速公路一类缺少横向结构的场景，就不将其用于回环识别。另一些工作把 `RING` 改成适用于 radar 的 sinogram 描述子，并发现引入自相关可以提升在高噪环境中的匹配稳定性。

`Scan Context` 也是非常流行的选择。针对 `2D radargram`，有方法将每个 bin 内所有点的强度和累计，而不只记录点数，从而同时编码点密度与回波强度。为了减轻 radar 噪声与稀疏性的问题，描述子还可由多张经配准、去畸变和保守滤波后的极坐标图像聚合生成。对 `3+1D radar`，原始 `Scan Context` 可继续使用高度维度，但由于 radar 的高度测量噪声较大，也常改用 `Intensity Scan Context`，即在每个 bin 中存储最大强度值而不是最大高度。另一方面，小视场 `SoC radar` 很难像 `360` 度 `LiDAR` 那样天然获得旋转不变性，因此常要根据当前 odometry 先做 yaw 预筛选，再在有限候选范围内匹配描述子。

## 9.4 Radar SLAM

完整的 `Radar SLAM` 系统一般沿用与视觉、`LiDAR` 类似的总体框架: 前端估计 open-loop odometry，地点识别模块提出 loop closure 候选，后端基于 pose graph 或其他全局图优化框架进行轨迹一致化和地图更新。不同之处在于，radar 的观测更稀疏、更噪、物理上更复杂，因此每个模块都必须做出相应适配。

### 9.4.1 地图表示

许多 radar `SLAM` 系统使用的地图表示，与第 5 章介绍过的地图类型是相通的。不过，radar measurements 通常更加稀疏、噪声更大，而且常常包含虚警，因此建图时面临的挑战也更特殊。较早的一些 radar mapping 工作，直接沿用已有目标检测模型建立地图，常见做法是构造 `landmark map`，把每个被检测到的目标都作为地图元素。类似地，目标检测结果也能用于构造 `occupancy grid map`，并在建图时显式利用检测概率。

如果把信号中的单个峰值视作与 `LiDAR` 扫描中的点等价，那么它同样可以被用来构建 `2D occupancy grid map` 或点云地图。进一步地，提取出的点云还可以附带周围点的分布信息，用来估计方向和权重，从而形成类似 surfel 或 `NDT` cell 的表示。对于新一代高分辨率 `SoC radar`，由于其拥有更大的垂直视场和更多 `TX/RX` 天线，直接采用 `3+1D radar point cloud` 作为地图表示已经变得越来越常见。

另一种虽然相对少见、但很有特点的表示，是直接使用 full radar heatmap，也就是在峰值检测之前，为每个方向和距离 bin 保存的回波强度样本。在这种做法里，地图编码的是环境每个位置的全局反射功率，而不是占据证据；不同 heatmap 之间的二维对齐则通过相关运算完成。

书中还专门提到 `Kramer` 与 `Heckman` 提出的基于体素的 radar sensor model。该方法除了提供 odometry 之外，还能在存在视觉遮挡的情况下建立稀疏地图。它延续了 `Octomap` 中基于 `log-odds` 的 occupied/free cell 估计思想，但替换掉了 `Octomap` 原本的 `ray-cast` 模型。后者默认“传感器的第一次接触点”就是唯一与占据相关的位置；而广义化后的 radar 模型则考虑到 radar 具有穿透某些材料的能力，因此不再沿一条射线简单传播信息，而是在整个传感器视场内更新体素概率: 对存在 radar return 的体素提高概率，对漏检位置降低概率。

针对 radar 特性量身设计的另一类栅格表示，来自 `Nuss` 等人的 `probability-hypothesis-density multi-instance Bernoulli filter`。它把 grid cell 看成有限随机集，用专门的状态估计滤波方式处理动态障碍物，并能动态地融合 radar 与 `LiDAR` 数据。

对于 radar 数据天然存在的“低密度、高噪声”问题，近年来也出现了大量 machine learning 增强方法。典型思路是把 `LiDAR` 数据作为 ground truth，用学习模型提升 radar 输出的分辨率并降低噪声。例如有工作构建全局 radar map，然后利用预测网络对局部地图 patch 做超分辨、去噪，并补全稀疏与空洞区域，使结果更接近 `LiDAR` 地图。

最近，最初为 `RGB` 数据提出、后来又被用于 `LiDAR` 的 `neural fields`，也开始被应用到 `2D radar` 数据上。这类表示的核心特点是: 环境以隐式方式存入神经网络，查询空间中的某个点时，网络会返回诸如到最近表面的距离、颜色、透明度等物理量。对于 radar neural field，作者提到 `Borts` 等人的工作采用了基于物理的 radar 传感器模型，学习一个隐式的几何与反射率表示，并能从未见视角合成 radar measurements。此时，传感器接收功率不仅取决于已知的发射功率和天线增益，也取决于 `RCS`；而 `RCS` 本身又由目标物体的尺寸、反射率和方向性共同构成。神经表示学习到的，正是把观测到的 `RCS` 分解为一部分物体面积，以及另一部分“反射率 × 方向性”的乘积。

### 9.4.2 Radar SLAM 流水线

前面各节已经分别讨论了 radar `SLAM` 的主要部件: open-loop odometry、place recognition、map representations，以及若干与 radar 物理特性直接相关的建模问题。本节回顾若干完整 pipeline，说明这些组件在真实系统中如何组合起来。

作者首先强调，这些 graph-based radar `SLAM` frameworks 一般都遵循与图 `9.1` 类似的结构: 前端负责把原始 radar data 过滤成 point cloud，并执行 loop-closure place recognition；后端则维护 pose graph、进行全局优化，并在检测到回环后刷新地图。

`Figure 9.11` 展示了使用 `3+1D radar`（Sensrad Hugin）与 `IMU` 输入所生成的 radar `SLAM` 三维地图示例。颜色表示高度；左图为 loop closure 之前，可见地图左侧存在明显累积的水平与垂直漂移；右图为 loop closure 之后。

`TBV-SLAM`（`trust but verify`）[9] 是使用 `2D spinning radar` 的代表系统之一。它建立在 `CFEAR` `2D` radar odometry 之上，对每张 radar scan 都持续追踪位姿，但出于效率考虑，只把更稀疏的一组 keyframes 加入 `SLAM` pose graph。当累计行驶距离超过阈值，例如 `1.5 m`，系统就插入一个新的 keyframe，并基于其相对于上一个 keyframe 的对齐结果创建 odometry constraint。作者特别指出，图中约束不仅需要相对位姿，还需要 associated covariance matrix。`TBV-SLAM` 的一个经验结论是: 直接根据 registration cost function 的 Hessian 来估计协方差，并不一定优于一个小的固定对角协方差；前者有时反而会低估或高估不确定性，从而在后端优化中造成轻微 misalignment。

`TBV-SLAM` 的另一核心特点，是其“先信任再验证”的 loop closure 策略。系统先用改造版 `Scan Context` 描述子做检索，然后为每个 keyframe 额外构造多个经过 lateral translation augmentation 的描述子，以适应车辆在不同车道上经过同一地点的情形。回环候选不仅根据 descriptor similarity 评分，还会把 odometry uncertainty 传播进来，共同决定候选优先级。随后，系统对候选 pairs 做几何重配准，并通过 overlap measures 与学习式 `CorAl` [7] 对齐质量评估来验证，只保留可验证的最佳候选。这个 pipeline 很好地说明了 radar `SLAM` 的一个现实原则: place descriptor retrieval 远远不够，必须结合 odometry prior 与精细 geometric verification。

另一个著名系统是 `RadarSLAM` [474]。它同样基于 `2D spinning radar`，但 open-loop tracking 不是通过稠密 scan registration 完成，而是直接在 radargram 上提取 blob keypoints，并使用 `Lucas-Kanade` tracker [706] 做帧间跟踪。系统还利用估计出的 travel distance，对一整圈扫描内部的运动执行 constant-velocity compensation。与此不同，回环检测时它不再依赖这些稀疏 keypoints，而是从 radargram 提取更稠密的 point clouds，并为 keyframes 构造 `M2DP` descriptors [445]。为防止像 highway 这样缺乏辨识性的狭长场景造成误检，系统还会用 `PCA` 特征值筛掉过于 elongated 的 keyframes。相比 `TBV-SLAM`，`RadarSLAM` 的 loop verification 更简单，除上述筛选外基本不再做额外回环验证。

对 `2D SoC radar`，`Schuster` 等人 [982] 的方法更进一步，维护的是“pose nodes + radar feature nodes”的联合图，而不只是纯 pose graph。由于 `2D SoC radar` 产生的 detections 远少于 `360` 度 spinning radar 或 `3+1D` radar，因此把所有 detections 都保留在可优化图中在计算上仍然可行。系统使用 `BASD` [907] 提取 landmarks，并在局部区域内直接比较 compact binary descriptors，从而把新观测与地图中的已有 features 关联起来。经 `RANSAC` 剔除 outliers 后，特征和位姿一起被加入图中。该系统假设 open-loop tracking drift 适中，因此没有显式 place recognition 模块，但在发生回环后仍可用 `BASD` 匹配来优化全图。

最新的 `6-DoF` radar `SLAM` 方法，如 `4DRadarSLAM` [1270] 与 `4D iRIOM` [1309]，则直接以 `3+1D SoC radar` 输出的 `3D radar point clouds` 为输入。由于 datacube 到 point cloud 的 filtering 已在传感器端完成，pipeline 的重点转向去噪、moving object filtering 和 scan-to-submap registration。两者都先利用 per-point Doppler radial velocity 做 ego-velocity estimation，并移除不符合静态场景速度模型的 outlier points。`4D iRIOM` 还会进一步保留那些在固定半径内拥有足够多邻居、且邻域分布足够紧凑的点，以执行 denoising。之后，`4DRadarSLAM` 使用 `APDGICP` 进行带各向异性协方差加权的 scan-to-submap registration，对远距离点赋予更高不确定性；`4D iRIOM` 则使用 one-to-many distribution-to-distribution matching。

在 loop closure detection 方面，这两类 `3+1D` 系统都采用 `Scan Context` 思路，但实现细节略有不同。`4D iRIOM` 使用原始 `Scan Context`，而 `4DRadarSLAM` 使用 `Intensity Scan Context` [1150]，以避免 noisy elevation measurements 使描述子失去判别力。除此之外，`4DRadarSLAM` 还会用累计 odometry distance 做阈值验证，拒绝那些尽管描述子相似、但按里程计推断其实相距太远的候选帧。

### 9.4.3 Radar SLAM 中的多模态

到目前为止，书中讨论的大多是以 radar 为唯一外感知输入的系统，至多辅以 `IMU` 或 wheel odometry 这类本体传感。但现实中，多模态 radar 系统越来越重要，尤其是在极端环境下需要同时兼顾几何精度与环境鲁棒性时更是如此。

一种直接路线，是 radar-`LiDAR` 融合建图。系统可根据当前可见度估计在某种条件下信任哪类传感器: 天气良好时更信任 `LiDAR`，低可见度时更信任 radar。Radar 与 `LiDAR` 还可以联合提升地点识别，通过把 radar 注册到 `LiDAR` 地图中来实现跨模态检索。

另一条路线，是与视觉和热成像融合。例如，有工作在经典视觉惯性里程计 `ROVIO` 上加入 radar ego-velocity 估计，也有工作将热相机与 radar 结合构成低可见度下的稳健里程计系统。对于 `3+1D radar`，还可以利用 thermal image 与 radar 点云协同，通过 transformer 匹配连续热图像中的对应点，并用 radar 提供更稳定的深度信息。

多模态还不仅限于传感器间融合。某些系统使用外部先验，如 `OpenStreetMap` 或卫星图像来辅助 radar 定位。基于 `OpenStreetMap` 的方法，通常将建筑物墙线等结构作为先验，再把从 radargram 提取的定向点与这些线特征进行匹配。由于先验地图可能过时、不完整或带有较大不确定性，这一匹配本身相当困难。使用卫星图像的工作，则借助深度网络先从俯视图合成“可能对应的 radargram”，再分别估计旋转与平移对齐。

作者还提到 `UWB` 感知。虽然 `ultra-wideband` 也使用无线电频谱，有时甚至被称为 `UWB radar`，但其与 `mmWave radar` 的测距机理差别很大。在 `SLAM` 框架中，`UWB` 更常被用作地点签名的来源，即通过宽带回波频率响应判断“是否到过这里”，而实际的度量位姿通常仍来自 wheel odometry 或 `IMU`。因此它更像一种特殊的 place recognition 辅助模态。

## 9.5 Radar 数据集

本节简要回顾 radar `SLAM` 文献中的若干代表性数据集，表 `9.1` 也对这些数据集做了概览。整体上，它们大致可分为使用 spinning radar 的数据集，以及使用 `SoC / array radar` 的数据集；还有少量数据集同时包含两类设备。

表 `9.1` 汇总了公开 radar 相关数据集的总体情况，其中 `VO` 表示 visual odometry，`TLS` 表示测绘级 terrestrial laser scans，`RTK` 表示带实时动态修正的 `GPS`。

| 类别 | 数据集 | LiDAR | Cameras | Ground truth | Environment | Inclement Weather |
| --- | --- | --- | --- | --- | --- | --- |
| Spinning radar | Oxford Radar RobotCar [54] | Yes | Stereo/Mono | GPS/IMU + VO | Dense Urban | Rain, Fog |
| Spinning radar | Boreas [128] | Yes | Mono | GPS/IMU + RTK | Sparse Urban | Rain, Snow, Fog |
| Spinning radar | MulRan [566] | Yes | No | SLAM | Mixed Urban | - |
| Spinning radar | RADIATE [1000] | Yes | Stereo | GPS/IMU | Mixed Urban | Rain, Snow |
| Spinning radar | OORD [353] | Yes | Mono | GPS | Urban and Offroad | Snow, Night |
| SoC array radar | nuScenes [137] | Yes | Stereo | GPS/IMU | Mixed Urban and Natural | Rain |
| SoC array radar | RadarScenes [981] | No | Mono | None | Mixed Urban Roadways | Rain, Fog |
| SoC array radar | ColoRadar [606] | Yes | No | SLAM | Varying | - |
| SoC array radar | NTU4DRadLM [1272] | Yes | Mono | SLAM | Mixed Urban | - |
| SoC array radar | MSC-RAD4R [212] | Yes | Stereo | GPS + RTK | Mixed Urban | Smoke, Snow, Night |
| SoC array radar | Snail [482] | Yes | Stereo | TLS | Roadways and Tunnels | Rain, Night |
| SoC array radar | K-Radar [835] | Yes | Stereo | GPS/IMU + RTK | Roadways | Rain, Snow, Fog, Night |
| SoC array radar | TruckScene [324] | Yes | Stereo | GPS/IMU + RTK | Roadways | Rain, Snow, Fog, Night |
| Both | HeRCULES [568] | Yes | Stereo | GPS/IMU + RTK | Mixed Urban and Natural | Rain, Snow, Night |

在 spinning radar 方面，表中的数据集全部采用 Navtech `2D` radar。最早也最有影响力的两个大规模数据集是 `Oxford Radar RobotCar` [54] 和 `MulRan` [566]。`Oxford Radar RobotCar` 记录的是城市道路上的多次 traversal，总里程约 `280 km`，并覆盖多种天气条件。`MulRan` 则包含更丰富的环境类型，既有密集 urban 区域，也有更 rural 的驾驶环境，同时不同 session 之间的时间间隔更长，因此特别有利于 place recognition 研究；不过其总驾驶里程相对更少。`Boreas` [128] 进一步提供了跨一年重复采集的路线数据，总里程约 `385 km`，并包含雨雪等恶劣天气。除 `SLAM` 外，该数据集还提供了车辆、行人、自行车等目标检测 benchmark。与这些城市场景不同，`Oxford Offroad Radar Dataset (OORD)` [353] 更关注非城市环境，包含约 `154 km` 的未铺装道路与山地小径数据。

在 `SoC radar` 方面，公开数据集既包括 `2+1D` 也包括 `3+1D` 数据。某些自动驾驶数据集虽然主要面向 object detection，但也被广泛用于 radar `SLAM` 的算法开发与测试。例如:

- `nuScenes` [137] 结合了五个 Continental `ARS408-21` radars、一台 `LiDAR` 和六个 cameras，聚焦四座城市中的 urban driving，并提供 `23` 类目标检测标注。
- `RadarScenes` [981] 包含四个 `77 GHz` 汽车 `2+1D` radars 和一个 camera，主要服务于 semantic perception，提供 `11` 类对象标注，但缺少精确 ground truth，也没有 `IMU` 和 `LiDAR` 数据。

更贴近 radar `SLAM` 的数据集包括 `ColoRadar` [606]。它提供 `3+1D` Texas Instruments `MMWCAS-RF-EVM` radar、`2+1D` TI radar、`IMU` 和 `3D LiDAR` 数据，特别重要的是，它不仅给出点目标，还包含原始 `ADC` 读数与 `3D heat maps (data cubes)`。该数据集覆盖室内、室外以及地下矿井环境，并提供 `6-DoF` ground-truth tracking。`NTU4DRadLM` [1272] 与 `MSC-RAD4R` [212] 都采用高分辨率 `3+1D` `Oculii Eagle` radar；前者包含结构化校园和非结构化公园环境，后者覆盖 urban / rural 的 on-road driving。`Snail-Radar` [482] 则同时采集了 `Oculii Eagle` 与 Continental `ARS548` 两种高分辨率 `3+1D` radar，并包含手持采集和城市道路驾驶场景。

作者整体传达的意思是，radar 数据集生态已经明显扩展，但相较于视觉和 `LiDAR`，它在 ground truth 完整性、跨设备统一性和基准协议方面仍不够成熟。因此，数据集本身仍是推动 radar `SLAM` 发展的关键基础设施。

## 9.6 延伸阅读与最新趋势

作者认为，真正意义上的 `3D Radar SLAM` 仍然不多，但随着高分辨率 `SoC radar` 的发展，我们可以预期未来会出现更多面向完全 `3D` 里程计与 `3D` 地点识别的工作。这对无人机等当前较少使用 radar 的平台尤其重要。与此同时，多个小视场 radar 组合成星座式系统也可能越来越普遍，而这会连带引出外参标定和跨雷达同步等新问题。

另一个重要趋势，是更充分地利用原始 `radargram` 与 `datacube` 中的光谱信息。传统 `Radar SLAM` 往往把 radar 数据尽快压缩成点云；未来系统则更可能直接在更丰富的原始频谱上工作，从而得到更细致的地图，也提升物体检测与分类能力。相关挑战之一，是如何推动厂商向研究界开放更底层的原始数据接口。

此外，radar 语义分割可能在地点识别中发挥更重要作用。通过显式区分环境中的不同对象类型，系统可以降低 radar 回波中的歧义，从而提高地图与回环识别可靠性。最后，多模态方法仍将是 `Radar SLAM` 最关键的发展方向之一。通过把 radar 与其他传感器，乃至地理先验，如 `OpenStreetMap`，结合起来，系统有望在大尺度 outdoor 场景中获得更高定位鲁棒性。总体而言，radar 未必会在所有指标上取代视觉或 `LiDAR`，但它已经成为让 `SLAM` 在恶劣环境中继续工作的关键组成。

