# 第 12 章 面向 SLAM 的腿式里程计

### 本章概览

腿式机器人由于能够穿越高度非结构化地形而日益普及。它们的主要优势在于，腿提供了一种主动悬架（active suspension），使机器人机体的运动能够与地形轮廓解耦。因此，腿式平台能够跨越楼梯、不平整地面以及其他对轮式平台极具挑战的地面障碍。尽管本 handbook 其他章节回顾的 `SLAM` 算法同样可直接应用到腿式机器人上，但腿本身引入的额外 sensing 是一种非常有价值的新信息源，可被用于 odometry。

这一点对腿式 locomotion control 与 planning 尤其重要，因为为了防止跌倒和失败，系统必须具备 high-frequency、low-drift 的实时 pose 与 velocity estimation，而这本身就是一项困难任务。

本章将讨论如何为一台装备了 onboard `IMU` 和 joint sensing（position 与 torque）的腿式机器人估计实时 pose 与 velocity。重点会放在 `leg odometry` 上，也就是如何利用腿部 sensing 来恢复机器人机体的相对运动。作者将先介绍历史背景和预备知识（`12.1`），再介绍用于估计腿式里程计的关键理论工具（`12.2` 与 `12.3`）。其中会解释如何在已知 stance legs 的前提下，从腿部 joint sensing 恢复相对运动（`12.2`）；这使得 `leg odometry` 可以像 wheel odometry 或 inertial preintegration 一样，被看作 `SLAM` 问题中的一种测量。随后还会介绍接触如何被估计及其实践技术（`12.3`），再讨论如何利用 `factor graphs` 将 leg odometry 与其他传感模态结合起来（`12.4`）。最后，本章总结该方向中的开放问题（`12.5`）及其近期趋势（`12.6`）。

## 12.1 历史背景与预备知识

尽管现代腿式平台往往也会搭载 camera、`LiDAR` 和 `IMU`，但腿本身又额外提供了一类独特信息源: 通过利用腿部的运动学与动力学信息，可以获得机器人机体相对运动的测量，这正是 `leg odometry`。该术语是比照轮式车辆的里程计而来，后者通过测量车轮转动量来推断行驶距离。

不过，与轮式平台不同，腿式机器人是通过不断建立和打破与地面的接触来实现运动的。每一步通常包含腾空的 swing phase，以及与地面保持非滑移接触的 stance phase。正因为真正推动机器人前进的是 stance legs，所以 `leg odometry` 本质上可以分解为两个子问题: 一是 contact estimation，即判断哪些腿当前处于支撑期；二是 motion estimation，即在 stance phase 内根据这些腿估计机体的增量运动。

### 12.1.1 历史背景

`Leg odometry` 的发展与“如何让机器真正走起来”这一问题紧密相连。腿式机器人研究可追溯到 20 世纪五六十年代，当时人们试图用步行机械克服轮式车辆在崎岖地形中的局限。一个著名例子是 `General Electric Walking Truck`，这是一台重达 1400 千克、由人类操作者控制的步行机器。随着控制理论发展，研究焦点逐渐转向更小尺度、能实现自动步行行为的平台。

到了 20 世纪 80 年代末，`Raibert` 的单足、双足和四足机器人展示了出色的运动能力，这极大推动了人们将腿式平台用于真实任务的兴趣。为了实现真正自主，必须解决 state estimation 和 odometry 问题。早期经典工作之一，是 `Roston` 与 `Krotkov` 利用腿运动学来估计 `Ambler` 六足机器人的运动。类似思路随后也被应用到 `RHex` 等其他多足平台上。

后续研究逐渐开始把腿式里程计与 `IMU`、视觉等传感器结合，例如通过粒子滤波器或 Kalman filter 进行融合。`Bloesch` 等人的工作为基于滤波的全地形运动学-惯性 odometry 奠定了基础，并进一步推动了四足和双足平台的发展。2012 到 2015 年的 `DARPA Robotics Challenge` 又把 whole-body state estimation 推向前台，要求系统不仅估计 `6-DoF` 位姿，还要估计机器人全身状态，并与 `LiDAR`、stereo vision 等外感知传感器联合工作。

近年随着商用四足和人形平台崛起，社区越来越关注更系统、更稳健的腿式状态估计方法。这不仅体现为针对四足和双足平台的 `Factor Graph` 估计，也体现为 invariant estimation、更加严格的动力学建模，以及在 `DARPA SubT` 这类极端场景中融合多种传感器的 resilient estimation 系统。

### 12.1.2 参考坐标系

腿式机器人中的参考系必须定义得非常清楚。书中使用的核心参考系包括: 固定在地面上的世界系 `Fw`，固定在机器人 floating base 上的机体系 `Fb`，以及附着在每个足端上的 foot frame，例如 `Ff1`、`Ff2` 等。为方便起见，作者假设 `IMU` 与 `Fb` 重合。

此外，每当某只脚与地面建立接触时，还会定义一个或多个临时惯性系 `Fk`。对 humanoid 来说，`Fk` 在 touchdown 时与 foot frame 完全重合；对 point-foot quadruped 而言，接触系通常只与落脚点位置一致，而不一定有足够的姿态信息。很多公式之所以成立，前提都在于这些参考系之间的关系被严格定义清楚。

### 12.1.3 状态定义

严格来说，腿式机器人的状态应包括机体的位姿、速度以及关节状态。但本章默认关节状态由专门传感器直接测量，因此估计问题主要关注机器人 base 的位姿和速度。也正因为如此，作者几乎可以把 state estimation 与 odometry 交替使用，后者就相当于“不含 loop closure 的 `SLAM`”。

书中将机器人状态形式化为位置、姿态、线速度与角速度的组合。例如，世界系中的 base 位置和姿态分别记作 `t` 与 `R`，而线速度和角速度通常在机体系下表达。状态定义并非表面形式问题，它直接决定哪些变量会在滤波器或因子图中被显式估计，哪些量只是通过约束隐含存在。

### 12.1.4 腿式机器人运动学

腿式机器人的运动学描述，可以看作一个 main link，也就是 body / trunk / torso，再加上一条或多条连接其上的 kinematic chains，也就是各条腿。本章主要考虑工程上最常见的两类结构: 每条腿有 `6` 个主动自由度、配平足的 bipeds，以及每条腿有 `3` 个主动自由度、配 point feet 的 quadrupeds（见图 `12.2`）。作者还假设机器人本体是刚体，例如不存在 articulated spine，并忽略其他上肢结构，例如 arms。

对 `leg odometry` 来说，我们最关心的是足端相对于机器人 body 的相对位姿或相对位置。设 `q ∈ R^N = [q_1, ..., q_N]^T` 表示具有 `N` 个主动自由度的 articulated robot 的 joint positions，即各个 revolute joints 的角位置。图 `12.1` 以及本章后续公式里，作者以 quadruped 和 biped 都取 `N = 12` 作为示例，但也明确指出，其他平台的 `N` 可以不同。Joint positions `q` 通常通过安装在各关节上的 rotary encoders 直接测得；也可以通过机器人运动学模型，结合安装在电机与关节传动之间的编码器读数间接恢复，例如通过测量液压缸行程再反算相应转角。

`q` 的时间导数 `qdot = [qdot_1, ..., qdot_N]^T` 就是 joint velocities。它们通常通过对 encoder readings 做数值微分得到；若读数噪声较大，也可借助安装在各 links 上的 `IMUs` 来辅助估计 joint velocities [1204]。

给定 `q`，对每一只脚 `f`，定义相应的 `forward kinematics` function `fk(q) : R^12 -> SE(3)` [716]，它把 joint positions 映射成该脚相对于机器人 base 的 pose:

```text
T_f^b = fk(q) = [ R_f(q)  p_f(q) ]
                [   0        1   ]                                 (12.2)
```

这里作者进一步定义两个方便的函数: `f_R : R^12 -> SO(3)`，表示足端在 base frame 中的 orientation；以及 `f_p : R^12 -> R^3`，表示足端在 base frame 中的 Cartesian position。对 point-foot quadrupeds 而言，`leg odometry` 通常只使用 `f_p(q)`，因为 point foot 可以在接触点绕转而不改变 joint positions，因此足端姿态本身并不可靠。

对式 `(12.2)` 做时间导数，就得到足端相对于机器人 base 的 Jacobian matrix `J(q) : R^12 -> R^(6×12)` [717]，它可用于把 joint velocities 映射成单只脚的线速度与角速度:

```text
[ v_f^b ]       [ J_v(q) ] qdot
[ ω_f^b ] = J(q) qdot = [ J_ω(q) ] qdot                           (12.3)
```

其中 `J_v(q) ∈ R^(3×12)` 和 `J_ω(q) ∈ R^(3×12)` 分别是 Jacobian 的线速度部分与角速度部分。由于 `q` 包含了所有 `N = 12` 个 joints 的位置，而每条腿在运动学上彼此独立，因此 `J(q)` 实际上是一个稀疏块矩阵，只有与当前那条腿对应的块是非零的。书中把这些非零块记作 `J̄(q)`，它们把某一条腿的关节角速度映射到该腿足端相对于 base 的线速度和角速度。

这些 forward kinematics 和 Jacobian expressions 构成了整章 `leg odometry` 的基础。只要 joint sensing 可用，并且后续接触假设成立，足端相对于 body 的几何约束就能被转化为 base motion 的估计信息。

### 12.1.5 腿式机器人动力学

光有运动学还不够。浮动基座 articulated-body system 的动力学通常可由 `Recursive Newton-Euler` 算法 [319] 写成两组耦合方程。第一组描述 floating base 本体的动力学，它具有 `6 DoF` 且是 underactuated；第二组描述通过主动关节连接到该 base 上的 `N` 个刚体部分，例如典型情形下 `N = 12`。两组运动方程可整体写成矩阵形式:

`M(q) [v_dot; ω_dot; q_ddot] + h(q, q_dot) = [J_b^T; J_q^T] f + [0_6; τ]` 。

其中，`M(q)` 是质量矩阵；它与由 floating-base 线加速度 `v_dot ∈ R^3`、floating-base 角加速度 `ω_dot ∈ R^3` 以及主动关节加速度 `q_ddot ∈ R^12` 组成的堆叠向量相乘。第二项 `h ∈ R^18` 是 bias term，用来汇集 `Coriolis`、离心力和重力效应。右侧最后一项描述广义扭矩，其中 base 的力矩部分为零，因为 base 不受直接驱动，而主动关节的扭矩则由 `τ ∈ R^12` 给出。

对 `leg odometry` 来说，右侧倒数第二项最重要，也就是接触力经 Jacobian 映射回广义力空间的部分。它的维度取决于机器人类型以及当前处于接触状态的腿数。设 `c` 为接触腿数，`d` 为单腿主动关节数，则四足机器人有 `d = 3`，人形机器人有 `d = 6`。此时，`J_b ∈ R^(dc×6)` 是把 base twist 映射到足端速度的 Jacobian，它依赖机器人 forward kinematics 以及 body 在惯性系 `F_w` 中的绝对姿态；`J_q ∈ R^(dc×12)` 则是式 `(12.3)` 中各条腿 Jacobian 的堆叠形式，其排列取决于机器人类型和接触腿集合。

例如，对双足 humanoid，若双脚都接触地面，则有

`J_q = [J_1(q); J_2(q)]` 。

而对 point-foot quadruped，由于腿可以绕接触点自由转动，所以 `J_q` 只使用线速度 Jacobian `J_v`。例如，若第一、第三和第四条腿处于接触状态，则

`J_q = [J_1,v(q); J_3,v(q); J_4,v(q)]` 。

最后一个需要定义的量是 `f ∈ R^(dc)`，它表示所有接触足端处的力和/或力矩集合。对四足机器人，如果四只脚都在地面上，则 `f` 就是四个线性接触力 `f_1, f_2, f_3, f_4` 的堆叠。对双足 humanoid，若双脚都接触地面，则 `f` 同时包含每只脚接触点处三个方向的线性力和力矩，也就是类似 `[f_1; τ_1; f_2; τ_2]` 的结构。

作者还指出，正如第 `12.3.4` 节将进一步说明的，这些接触力与力矩本身就可以被用来推断腿是否处于接触状态。也就是说，动力学不仅描述“机器人如何运动”，还为 `contact estimation` 提供了直接信息来源。

### 12.1.6 关节感知

`Leg odometry` 所依赖的主要本体感觉来自关节感知，也包括直接或间接的接触感知。作者依次介绍了最常见的几类硬件。

#### 12.1.6.1 旋转编码器

`Rotary encoder` 是把旋转轴的角位置转换成模拟或数字信号的机电装置。在腿式机器人中，它们使我们能够直接测量关节角度，从而确定机器人运动学；类似器件也会出现在其他部件上，例如机械式 `LiDAR` 就使用编码器来测量 beam array 的方位角。

编码器可以按多个维度分类，包括工作原理上的 optical 或 magnetic、读数方式上的 absolute 或 incremental，以及输出形式上的 analog 或 digital。书中指出，腿式机器人上最常见的是 absolute 与 relative optical digital encoders；其他类型可参考专门文献 [718]。

`Absolute optical digital encoders` 如图 12.3(a) 所示，能够以绝对方式测量关节角，也就是说，对同一关节构型它总会给出相同读数。其原理是: 红外光源，例如 `LED`，照向静止部分上一圈按径向布置的感光元件，例如 photoresistors；在光源与感光元件之间，有一张随轴旋转的圆盘。该圆盘被划分为若干同心扇区，每个扇区要么透明、要么不透明，并按照某种模式编码特定角度区间对应的二进制数。这个二进制序列通常按 `Gray code` 排列，使相邻自然数只差一个 bit，从而降低读数错误的概率。书中以 `8-bit encoder` 为例说明，其可能输出共有 `256` 个值，因此角分辨率为 `360 / 256 = 1.41` 度。编码器的角分辨率，本质上就由 bits 数，也就是同心扇区数决定。

`Incremental optical encoders` 如图 12.3(b) 所示，则是相对测量设备。它们上电时把当前位置视作零位，之后只测量相对于该参考点的角度变化。它们不依赖 `Gray code`，而是使用更简单的码盘: 在不透明圆盘上沿半径方向规则开槽，使单个光敏电阻 `A` 在圆盘匀速旋转时输出方波。另一个光敏电阻 `B` 相对 `A` 具有 `90` 度相移，因此两路信号构成的 `2-bit` 量 `AB` 在任意时刻有四种可能取值。系统通过这些状态转移来判断旋转方向，以及应增加还是减少计数 [718]。

由于结构更简单且成本更低，高分辨率 incremental encoders 曾经常与低分辨率 absolute encoders 配合使用：先由 absolute encoder 测得初始角，再由 incremental encoder 持续累积后续变化 [986]。不过，随着 absolute encoders 的分辨率不断提升、成本不断下降，它们正在快速替代 incremental encoders。对 `leg odometry` 而言，encoder 最直接提供的是 joint positions，而 joint velocities 通常还需要对这些角度读数进一步做数值微分。

#### 12.1.6.2 力与力矩传感器

`Force / torque sensors` 是把作用在表面某点上的线性力，或施加在轴上的机械扭矩，转换成电信号的装置。它们主要用于执行器扭矩控制，或者感知末端执行器与环境之间的相互作用。在腿式 locomotion 中，每一步都会有地面作用力产生，因此这些力必须被直接或间接测量，才能识别哪些腿正处于 stance phase。

对力和力矩两种量来说，其工作原理通常相同，只是几何结构不同。传感器内部的表面会被设计成在待测方向上受力时产生轻微形变；在这些表面上贴有一种柔性的可变电阻元件，即 `strain gauge`。`Strain gauge` 的电阻会随变形量成比例变化，其中压缩会降低电阻率，而拉伸会提高电阻率。图 12.4 左侧给出了在线性力测量 `load cell` 上使用 `strain gauge` 的例子 [718]；若要测量 torque，则通常会把一组 `strain gauges` 安装在连接电机轴的柔性轮辐结构上。

若希望同时测量末端执行器上所有方向的力与力矩，则可使用 `6-axis sensors`。这类传感器内部会布置足够数量和方向组合的 `strain gauges`，从而测出各个方向上的 force 与 torque。它们在 manipulation 中很常见，在 humanoid 足底上也会使用，以直接测量足底与地面的相互作用。

作者还指出，随着可回驱电机（`backdriveable motor`）设计推动低成本动态腿式机器人发展 [543]，很多系统也会直接利用 motor currents 来估计 torque，因为在一定条件下电机电流与输出扭矩成比例。

#### 12.1.6.3 接触传感器

由于 `force / torque sensors` 在 `leg odometry` 中最主要的用途，往往只是判断哪些腿处于支撑状态，因此更低成本的替代方案就是 `contact sensors`。它们输出一个二值量，用来表示某只脚当前是否与地面接触。这类传感器最初主要是为中小型 quadruped 设计的，因为这类机器人常使用球形或圆形橡胶足底。

最常见的 contact sensors 分为 optical 与 mechanical 两类。`Optical contact sensors` 在概念上与 encoder 相似：它们使用一个 `LED-photodiode` 组合，通过一个小孔感知光线。当足端与地面接触时，足底表面会发生足够大的形变，带动一个遮挡板位移并挡住该孔，从而让系统检测到接触。`Mechanical contact sensors` 则更简单，它们通常在足底内部隐藏一个 pushbutton switch，当足端受力足够大时按下开关，从而触发接触信号。

作者同时强调了 contact sensors 的主要缺点。首先，相比代价更高的 `force / torque sensors`，它们的响应时间通常较慢。其次，它们也有与 `F/T sensors` 相似的工程问题：需要把线缆一路布到足端，而且必须承受机器人着地时最直接的冲击，因此较易损坏。正因如此，这类传感器大多只出现在面向室内运行的小型 quadrupeds 上。

## 12.2 运动估计

在已知关节状态和机器人运动学之后，下一步就是估计机器人 base 的增量运动。作者指出，这可以主要通过两条路线实现: 一条是基于 forward kinematics 的相对位姿估计；另一条是基于 differential kinematics 的速度估计。两种方法共同的核心假设都是: 新建立的接触点在一段时间内保持静止。

### 12.2.1 相对位姿估计

作者先用一个沿 `zx` 平面行走的 humanoid 简化例子说明原理。设机器人 base 在相邻两个时刻对应的参考系分别为 `F_b` 与 `F_b'`，足端参考系和关节位置也在这两个时刻相应定义。若接触参考系 `F_k` 在 stance phase 内保持静止，则当足端系与接触系重合时，机器人机体向前移动的位移，就等价于足端相对机体“向后”移动的位移。因此有

```text
T_b'^b = T_k^b (T_k^b')^{-1} = (T_f'^k)^{-1} = (T_f'^b)^{-1} T_f^b = fk(q')^{-1} fk(q)
```

这条式子把 joint states 与机器人相对位姿直接联系起来。若把这些相对位姿沿时间串接起来，理论上就能仅凭 joint sensing 做 dead reckoning [946]。但它只能在 stance phase 中成立；并且对 point-foot 机器人，要仅靠这种方式恢复 `6-DoF` 运动，通常需要始终至少三只脚同时接触地面，因此对 quadruped 并不现实。

为解决这些问题，quadruped 的标准做法 [88] 是在原状态基础上再加入每条腿对应接触点在世界系中的位置 `c_i = t_k^w ∈ R^3`。对 humanoid，由于有 ankle 结构，还可进一步把接触点姿态 `B_i = R_k^w ∈ SO(3)` 一并纳入状态 [947]。另外，考虑到任意时刻并不能保证总有足够多的腿在接触地面，例如 gallop gait 中会出现全部离地阶段，因此系统通常还会配备 `IMU`，把角速度测量交给 `IMU`，并把 `IMU biases` 一并放入状态。

在这些考虑下，书中分别给出 quadruped 与 humanoid 的典型状态形式。quadruped 状态包含 `t_k, R_k, v_k, b_k^a, b_k^ω` 以及四个接触点位置 `c_1, c_2, c_3, c_4`；humanoid 状态则包含 `t_k, R_k, v_k, b_k^a, b_k^ω`，外加两个接触位置 `c_1, c_2` 与两个接触姿态 `B_1, B_2`。

基于这些状态，对任意一条接触腿 `i`，相对位姿关系可以更精确地写成

```text
T_k^b = fk(q) = T^{-1} C_i
```

其中

```text
T = [R t; 0 1]
```

是 base 在固定参考系中的位姿，而

```text
C_i = [B_i c_i; 0 1]
```

是该足端接触在固定参考系中的位姿。将其展开后可得

```text
T^{-1} C_i = [R^T B_i   R^T c_i - R^T t; 0 1]
```

由此可定义右上角两个上层 block 对应的核心量:

```text
f_p(q) = R^T (c_i - t)
f_R(q) = R^T B_i
```

其中 `f_p(q)` 表示固定系中足端相对 base 的位置变化，`f_R(q)` 表示相对姿态变化，且它们都被写成 joint angles 的函数。书中也特别提醒，对 quadruped 只能使用位置项 `f_p(q)`，因为 point feet 无法提供足端姿态估计。

式 `(12.17)` 与 `(12.18)` 正是后续滤波与因子图框架中使用的基本 leg odometry 测量表达。

### 12.2.2 速度估计

另一条常见路线，是直接利用式 `(12.3)` 中的 differential kinematics，从每条 stance leg 获得速度测量。这种做法在 quadruped 上被广泛采用 [89, 143, 577]，在 bipeds 中相对少见 [1084]。它的优点是速度测量很容易并入 filter 或 factor 中，不需要保留历史以避免位置误差累积 [315]，也不必显式维护额外的接触位置或接触姿态状态。缺点则是 joint velocities 通常由 joint positions 做数值微分得到，因此可能因微分和舍入误差而降低估计性能。

与相对位姿估计类似，作者先写出刚性接触条件下足端必须满足的速度关系。若某条腿处于稳定接触，则接触点 `k` 在固定系中的速度必须为零:

```text
v_k^w = 0
```

同时，接触点 `k` 的速度必须与足端 `f` 的速度一致，而该速度又能由 Jacobian 写成

```text
v_k^b = v_f^b = J_v(q) qdot
```

由于角速度可由 `IMU` 直接测得，这里作者只关心线速度。把上面两条关系合并，并考虑 base 与足端之间的 lever arm 后，可得

```text
v_k^w = v_b^w + ω_b^w × t_k^b + v_k^b
0 = v_b^w + ω_b^w × f_p(q) + J_v(q) qdot
v_b^w = - ω_b^w × f_p(q) - J_v(q) qdot
```

这里第一项 `- ω_b^w × f_p(q)` 描述的是 base 到足端 lever arm 引入的附加线速度 [270]，第二项则来自腿相对于 base 的微分运动。式 `(12.21)` 因而把机器人绝对速度与 forward kinematics、differential kinematics 以及 `IMU` 角速度联系起来，可以直接作为 measurement update，或在图优化里作为 factor 使用。

书中还补充说，若按照惯例需要在 body coordinates 中表达速度，只需使用机器人姿态 `R = R_b^w` 做相应坐标变换即可。

## 12.3 接触估计

由于 `leg odometry` 的全部前提都是“当前某只脚在可靠支撑地面”，所以 contact estimation 是整个问题的核心。不过，这里的“接触”含义会随应用而变化。比如在协作机器人里，末端执行器只要“碰到”某个物体，也就是受到了不可忽略的外力，就可以视为在接触；而对 `leg odometry` 来说，一只脚只有在接触点能在一段时间内保持静止时，才可视为真正处于接触。对 point-foot 平台而言，这进一步意味着接触点不能发生滑移。

形式上，对 quadruped 一类 point-foot 机器人，若地面反作用力 `f = [f_x, f_y, f_z]^T` 满足 friction cone 约束，就可以认为足端没有打滑:

```text
sqrt(fx^2 + fy^2) <= μx,y fz
```

其中 `f_x` 与 `f_y` 是相对于接触平面的切向地面反作用力分量，它们依赖局部地形形态；`μ_x,y` 是摩擦系数，取决于地面与足端材料性质。

对具有平足的 humanoid，还会多出一个关于接触转矩的条件，因为足底不能在地面上发生转动。书中给出了与 `center of pressure (CoP)` 和绕法向轴力矩相关的不等式约束，也就是要求 `CoP` 落在支撑多边形范围内，同时绕法向的接触转矩不能超过相应摩擦极限。

作者随后指出，只要法向力 `f_z` 足够大，式 `(12.22)` 到 `(12.24)` 往往就更容易满足。因此，在实际实现中最常见的接触估计方法仍然是简单地对 `f_z` 做阈值化。不同系统之间真正的差别，更多在于这个力是如何被测量或估计出来的，例如通过 contact sensors、`force / torque sensors`、joint sensing 或 `IMUs`，以及机器人本体结构的具体特性。

### 12.3.1 基于接触传感器

最直接的做法，是依赖专门 contact sensors。它们本质上在硬件层面对法向力 `fz` 做了阈值化处理，当测量力超过某个名义阈值时，输出接触为真。因此，使用这类传感器时，`leg odometry` 可以直接以该二值状态为依据。

其优点是实现简单、逻辑清晰；缺点是只提供离散接触信息，无法表达受力质量，也难以区分“刚刚触地但尚未稳定支撑”和“即将离地或接触不可靠”这类更细状态。

### 12.3.2 基于力/力矩传感器

若足端安装了力 / 力矩传感器，则法向力 `fz` 可随时间连续测得。这样一来，就不再只是简单地做二值判断，而可以根据受力模式推断更细粒度的状态。例如，一个持续增长但仍较小的力，可能意味着足端刚开始撞击地面，但尚未进入稳定支撑期；反之，若法向力下降到某阈值以下，则意味着脚很快就要失去可靠接触。

在这两种情况下，来自该条腿的里程计信息都应被丢弃，或至少增加其测量不确定性。因此，力 / 力矩传感器不仅能判断“是否接触”，还能帮助区分“接触质量是否足够好”。

### 12.3.3 基于惯性测量单元（IMUs）

除了装在机体上的主 `IMU`，有些系统还会在腿链或足端再加装小型 `IMU`。由于 touchdown 等接触事件会在足端引起明显加速度变化，因此这类局部惯性信号可被用来隐式检测支撑腿。

这种方案的优点，是传感器本身不直接承受足底冲击，因此相比嵌入足底的接触或力传感器更不容易损坏；代价则是需要额外的信号处理来从振动与冲击中可靠分离出接触信息。

### 12.3.4 基于关节力矩感知

若机器人设计和集成条件限制，不方便在足端额外安装传感器，则还可通过 joint torque sensing 反推出接触状态。其核心思想是利用动力学模型，把地面反作用力映射回关节扭矩空间。

以 quadruped 为例，书中利用 `J` 的分块结构，从式 `(12.5)` 推得末端执行器处的力可写为

```text
f_i = -(J̄_i,v^T)^(-1) (τ_i - h_q,i - F̄_i vdot)
```

这就是式 `(12.25)`。其中 `f_i ∈ R^3` 和 `τ_i ∈ R^3` 分别是第 `i` 条腿的 `ground reaction force (GRF)` 与关节扭矩；`J̄_i,v` 是足端 Jacobian `J_v` 中第 `i` 个非零分块，在 quadruped 情形下它通常是方阵；`F̄_i ∈ R^(3×3)` 是质量矩阵中的对应分块；`h_i,q ∈ R^3` 则表示该腿的离心 / Coriolis / 重力扭矩项。

需要注意的是，这样得到的 `f_i` 只能在 base 坐标系中被估计。若要恢复真实的 GRF，还缺少两类关键信息:

- 局部地形的倾角。接触力方向取决于接触面的朝向；对 humanoid 而言，踝关节通常可以给出较好的近似，而对 quadruped 来说，若没有 exteroceptive sensing，接触坐标系朝向通常无法直接确定，只能借助其他已接触足端做启发式推断，例如对这些接触点拟合一个平面 [329]。
- 摩擦系数。力的水平分量依赖摩擦系数，而摩擦系数又取决于机器人当前踩踏材料的性质，因此通常只能事先已知，或依据机器人正在经历的滑移程度来间接推断 [513]。

因此，单靠 joint sensing 来建立接触状态在总体上仍是一个开放问题。已有方法通常会以概率方式联合运动学与动力学信息 [497]，或进一步引入学习方法（见第 `12.6` 节）来完成接触检测。

## 12.4 将腿式里程计用于状态估计

有了 leg odometry 测量之后，下一步就是把它们纳入完整的 state estimation framework，与 `IMU`、camera、`LiDAR` 等其他模态共同工作。历史上，腿式状态估计多以滤波方法为主，因为 locomotion control 对高频、低延迟状态估计要求极高；但近几年因子图和平滑方法也越来越常见，特别是在与建图和全局估计结合时。

无论采用滤波还是平滑，要想最优融合 `leg odometry` 与其他传感器，都必须先量化其不确定性。因此作者首先讨论 encoder noise propagation，随后再介绍因子图中的接触预积分、速度预积分及其与外感知的融合。

### 12.4.1 编码器噪声传播

`Encoder noise` 是腿式里程计最直接的误差来源。若把真实 joint positions 记为 `q`，测量值记为 `q̃`，则可写成

```text
q̃ = q + ηq
```

其中 `ηq` 通常被建模为零均值高斯噪声。但因为 forward kinematics 和 differential kinematics 都涉及旋转，因此它们是非线性的，高斯噪声经过非线性映射后并不会严格保持高斯。

因此，作者采用与前面章节类似的一阶线性化思路: 在当前测量附近使用 Jacobian 展开 kinematics 函数，把 joint-space 的噪声线性传播到足端相对位姿或速度测量上。对 forward kinematics，可写成

`f_p(q + η_q) ≈ f_p(q) + J_c(q) η_q`  。`(12.27)`

其中 `J_c(q)` 是 body manipulator Jacobian，也就是通常的 manipulator Jacobian `J(q)` 在 contact frame 中的表达形式。

类似地，对受 encoder noise `η_q` 与 encoder velocity noise `η_qdot` 共同影响的速度测量，有

`J(q + η_q) (qdot + η_qdot) ≈ J(q) qdot + [∂(J(q) qdot)/∂q] η_q + J(q) η_qdot`  。`(12.28)`

从式 `(12.27)` 与 `(12.28)` 可以看出，所有乘在噪声项前的附加项都只是给定 encoder measurement 下的线性组合，因此可以重新归并为一个单独的高斯项。这样就能在近似高斯框架下为后续滤波器或因子图构造合理的测量协方差。

### 12.4.2 Factor Graph（因子图）平滑

为了生成 locomotion behaviors，同时避免跌倒等灾难性失败，腿式机器人对实时控制和高频状态估计有非常严格的要求。历史上，这些需求通常通过 `EKF`、`UKF` 或 invariant `EKF` 等非线性 Kalman Filter 来满足。这些滤波器一般融合高频传感器数据，如惯性与运动学信息，为控制器供给状态；而外感知传感器更新有时也会纳入控制环路，但更多仍被分配给 mapping 与 planning。

Kalman filtering 方法的一个局限是，它要求显式区分过程模型与测量模型；当缺少合适过程模型时，常常只能退化为 constant velocity model，或更常见地使用 `IMU` propagation。相比之下，`Factor Graph` 方法更加一般，因为它把过程模型和测量模型统一看作“状态与测量之间的关系”。

腿式系统里的 `Factor Graph` 方法主要在估计状态数量与时间范围上有所不同。当图中只包含两个相邻状态时，因子图看起来很像滤波器；`Two-State Implicit Filter (TSIF)` [92] 就是一个例子。当窗口增大后，图不仅能估计当前状态，还能修正窗口内的一段历史状态。状态采样频率则是一个系统设计问题: 若以高频（例如 `IMU` 频率）加入更多状态，估计器设计会更直接，但为了维持有界计算量，就不得不减小时间窗口；相反，若希望在固定状态数下覆盖更长时间，则可像第 11 章里的 `IMU preintegration` 那样对测量做预积分。

原书接着给出两个相关文献中的例子，用来展示 preintegration theory 与前面介绍的 leg odometry 概念如何进入 factor graph estimation framework。重点分别是 bipeds 的 `contact preintegration` [440] 和 quadrupeds 的 `velocity bias preintegration` [1190]。在这两种情形下，第 `12.2` 节中的测量都会被改写成图中因子的 residual 与 covariance。Figure `12.7` 展示了两类典型因子图结构: 上图是带 preintegrated contacts 的图，下图是带 preintegrated velocity 的图；其他约束两个状态的外感知测量，也都可以很自然地再作为额外因子加入图中。

#### 12.4.2.1 接触预积分

接触预积分的想法与 `IMU preintegration` 非常相似。对于 humanoid，如果某条支撑腿在一段时间内持续保持 stance，则其相对接触运动增量可以被压缩成一个连接两个状态的因子。作者引用 Hartley 等人的做法，将其写成一个包含 prior factor、预积分 `IMU factor`、forward kinematics factor 和 contact preintegration factor 的因子图，所提出的图结构见 Figure `12.7a`。其中 prior factor（黑色）用于锚定整张图，预积分 `IMU factor`（橙色）引入来自 `IMU` 的 motion prior；对 humanoid 还额外加入 `forward kinematics factor`（绿色）来约束足端 contact frames 的姿态，而 `contact preintegration factor`（蓝色）则编码两条腿接触状态之间的相对运动。

其中，`forward kinematics factor` 用于把足端接触系姿态与机器人 base 姿态通过支撑腿的 forward kinematics 联系起来。把上一节推导的 encoder noise propagation 代入相对位姿测量 `(12.17)` 和 `(12.18)` 后，可得该因子的 residual 与 covariance:

`r_F = Log(C_i^(-1) T f_k(q̃))`  。`(12.29)`

`Σ_F = J_c(q) Σ_q J_c^T(q)`  。`(12.30)`

这类因子的本质约束是“机器人 base 与接触坐标系之间的相对位姿，应与由关节读数推得的运动学结果一致”，而不确定性则来自关节编码器噪声经 Jacobian 传播后的结果。

`Contact preintegration factor` 则进一步约束两个时刻之间接触点的演化。理想情况下，若 stance leg 没有 slip，则接触框架的旋转和位置在支撑阶段应保持不变；实际中，接触点速度会受到 slip 等效应影响，因此作者将其建模为高斯噪声。对于相邻两个状态 `x_i` 和 `x_j`，有

`ΔR̃_ij = B_i^T B_j Exp(δθ_ij) = I`  。`(12.31)`

`Δc̃_ij = B_i^T (c_j - c_i) + δd_ij = 0`  。`(12.32)`

这里使用的是式 `(12.15)` 中 contact frames `C_i` 与 `C_j` 的旋转与平移分量。其中 `δθ_ij` 与 `δd_ij` 是预积分接触噪声项，用来将接触点速度的不确定性建模为零均值高斯变量。于是可得接触预积分因子的 residual 与 covariance:

`r_C = [ Log(B_i^T B_j) ; B_i^T (c_j - c_i) ]`  。`(12.33)`

`Σ_C = [ Σ_w Δt_ij   0 ; 0   Σ_v Δt_ij ]`  。`(12.34)`

这里 `Σ_C` 由接触角速度协方差 `Σ_w` 与线速度协方差 `Σ_v` 在时间区间 `Δt_ij` 上积分得到。原书也明确指出，这一因子只在同一条 stance leg 且接触模式不切换时成立，因此它无法直接覆盖腿切换动力学；后续工作才针对 contact frame switch 做了扩展。

#### 12.4.2.2 速度预积分

对 point-foot quadruped 而言，仅靠 forward kinematics 和接触预积分往往不足以完整约束相对 `6-DoF` 运动，因此作者介绍了另一类速度预积分因子。其核心思想，是把上一节由 `leg odometry` 推出的 base linear velocity 当作高频测量，对其进行预积分，从而得到约束两个状态之间相对平移变化的因子。

书中给出了一条含噪速度模型:

```text
ṽ = - Jv(q) qdot - ω × fp(q) + ηv
```

也就是

`ṽ = - J_v(q) qdot - ω × f_p(q) + η_v`  。`(12.35)`

若再假设在 `t_i` 到 `t_j` 之间 base 线速度近似常值，就能把这些速度测量预积分为一个相对位移量:

`Δt̃_ij = Δt_ij + δp_ij = Σ_(k=i)^(j-1) ΔR̃_ik ṽ_k Δt + δp_ij`  。`(12.36)`

其中 `δp_ij` 是预积分速度噪声项。

在此基础上，就能构造 `velocity preintegration factor`。它的残差与协方差为

`r_V = R_i^T (t_j - t_i) - Δt_ij`  。`(12.37)`

`Σ_(V,ij) = Σ_(k=i)^(j-1) Σ_(V,ik) + A Σ_v A^T`  。`(12.38)`

其中矩阵 `A = ΔR̃_ik Δt`。因此，这个因子比较的是: 当前两状态间的相对平移 `R_i^T(t_j - t_i)`，是否与由腿式里程计速度预积分得到的 `Δt_ij` 一致；其协方差则由各小时间段速度噪声传播并叠加得到。这样，速度预积分因子便可与 `IMU` 因子一同进入图中，为四足平台提供更直接的平移约束。

#### 12.4.2.3 处理多重测量

前面讨论时往往默认每次只利用一条腿的测量，但实际中多条腿常常同时处于接触状态。这既带来冗余和鲁棒性，也带来不一致风险: 不同腿若提供互相矛盾的信息，系统该信哪一条？

书中指出，最简单的一种做法，是像 [315] 那样只挑选当前被认为最可靠的那条腿，把其他腿的信息全部丢弃。另一个很自然的思路，则是把每条腿及其 measurements 都独立处理；当 contact poses（或 contact positions）本身显式成为状态的一部分时，这样做尤其直接 [440, 577]。

如果 contact poses 并未显式进入状态，那么所有处于 stance 的腿在同一时刻获得的 velocity measurements 仍然可以被视为相互独立；不过，为了减轻 filter 或 factor graph 的计算负担，实践中往往更倾向于先把这些速度测量平均成一个单独 measurement 再使用 [1190]。因此，多腿同时接触时的关键系统设计问题，就是如何在 redundancy、robustness 与 computational load 之间做平衡。

### 12.4.3 与外感知传感器集成用于 SLAM

正如前文所示，`leg odometry` 的主要价值并不在于独自完成长期定位，而在于为更大系统提供低漂移、短时可靠的运动先验。这样一来，构建在其上的 `SLAM` 系统可以获得更平滑的相邻节点约束，从而在回环闭合时减少大幅度突变修正。

与 `LiDAR`、camera 这样的外感知传感器集成，在滤波框架和因子图框架中都非常自然: 前者增加新的测量更新，后者增加新的因子即可。真正困难的地方在于，多源传感器工作频率不同、噪声特性不同、失效模式也不同。

如果某个模态违反了零均值高斯假设，或干脆完全失效，那么滤波器或因子图都可能被拖垮。为此，很多系统采用 loosely coupled 结构，让 `Visual-Inertial`、`Legged-Inertial`、`LiDAR-Inertial` 等子系统并行运行，再对输出结果做 triage 和选择。另一类 tightly coupled 方法则把所有因子直接放入同一图中，并通过在因子层面做 triage、跳过失效传感器，或使用 robust cost function 来提升系统整体稳健性。

## 12.5 开放挑战

尽管 `leg odometry` 已经相当有潜力，但它依赖的许多关键假设在现实中经常被打破，例如 rigid body、rigid contact、no slip。作者据此总结了若干仍未彻底解决的开放问题。

### 12.5.1 腿部形变

本章中推导的 `leg odometry` 方程几乎都默认机器人是完美刚体。但在现实里，这一假设经常不成立。一旦腿部在受力时发生弹性弯曲，`forward kinematics` 计算出来的就只是末端执行器的理想位置，而不是其真实位置，因此 `leg odometry` measurements 会带上系统性偏差。类似地，即便接触点本身不动，只要施加在腿上的力使其弯曲，joint angles 也会发生变化。图 `12.8` 直观地展示了这一点: 虽然真实的 base 到接触点变换没有发生系统所期望的变化，但在刚腿假设下，forward kinematics 会把这一现象错误解释成 base 向上运动。

当这种问题只在很短时间段内出现时，一种曾经使用过的策略 [143] 是分析 force profile，在检测到冲击或异常载荷的那些时间段内直接拒绝对应 measurements。

作者还指出，对 bipeds 来说，由于腿通常更长，leg flexibility 问题往往会更严重。一种理论上可行的路线，是利用 leg load 与 flexibility 的相关性来建模，即腿承受负载越大，弯曲也越明显。若能非常细致地建模机器人结构几何与材料属性，就有可能把 bending properties 显式纳入估计。然而，这种方法通常十分复杂，而且很难泛化到其他平台。

因此，实践中更常见的路线，是在腿部各个 links 上额外集成 `IMU`，利用这些额外的本体感觉信号去估计真实的 link orientations，再与 joint readings 结合起来修正纯运动学解 [1127]。这也是目前处理腿部形变最常见的思路。

### 12.5.2 非刚性接触与打滑

若机器人踩在松软或可塌陷地面上，足端可能在接触状态不变时继续向下陷入地面；若地面柔顺或摩擦不足，还可能出现显著 slip。此时，“接触点静止”这一假设就被破坏了。即便从估计现象上看，这和腿部形变带来的误差很相似，其根本原因却不同: 这次不是机器人腿在变，而是接触点本身在运动。

在这种情况下，单靠 `leg odometry` 往往不够。系统需要额外的外感知传感器，例如 camera，使接触点速度重新变得可观。相关工作可将 contact velocity 作为显式状态估计，或把足端位置的时间导数一并纳入模型。

## 12.6 延伸阅读与最新趋势

作者最后总结了几类非常值得关注的趋势。第一类是 `learning-based contact estimation`。由于 rigid contact 假设极易被打破，而精确建模又非常困难，因此越来越多方法开始使用数据驱动方式学习 contact state。有的工作通过监督学习训练 contact classifier；有的通过聚类等无监督方法，仅依赖 joint sensing 和 inertial sensing 来恢复接触信号。随着深度学习发展，也出现了使用神经网络直接判断足端接触状态的方法，它们在结构化与非结构化环境中都表现出更好的泛化能力。另一方面，借助视觉和触觉融合的新型 haptic sensor，也可能进一步提升 force 和 contact estimation。

第二类趋势是 `end-to-end learning`。传统高频 `leg odometry` 和 proprioceptive state estimation，原本是为了支持闭环 model-based locomotion control 而发展出来的。但强化学习在 locomotion 控制上的成功，开始挑战某些传统模块的必要性。有些 `RL` 控制器只需要 base orientation 相对重力的方向，以及机体瞬时速度，就能实现稳定行走；更进一步的工作甚至直接从 joint 与 inertial 原始数据中学习这些信息。这并不意味着 state estimation 已经不再需要，而是说明对于某些 locomotion 目标，估计器的角色和接口可能正在变化。

作者指出，若目标是更复杂的导航行为，例如跨越障碍、面向目标位置前进甚至 robot parkour，那么系统仍然需要可持续的 odometry 估计，因为机器人必须知道自己相对于起点或目标的位置变化。已有少数工作尝试将 state estimation 一并纳入 locomotion policy 的学习过程，直接预测 body velocity、feet height 和 contact state 等变量。虽然这类方法目前主要服务于 locomotion，本质上却也可能为未来 `SLAM` 中的 proprioceptive odometry 提供新思路。

第三个重要方向是 humanoid robots。人形机器人之所以长期吸引人，是因为它们天然适合人类环境、工具和交通工具。但要让 humanoid 真正承担配送、仓储和制造等复杂任务，就必须解决 whole-body estimation、长期可靠定位、间歇性多部位接触、柔顺安全交互等一系列远比四足 locomotion 更复杂的问题。脚、手、手臂甚至躯干都可能成为间歇接触点，而接触也往往不再满足本章中反复使用的 rigid contact 假设。

换言之，`leg odometry for SLAM` 远远还没有“做完”。它正在从支撑四足 locomotion 的一个子模块，逐渐扩展为面向更一般腿式平台，尤其是 humanoid 和复杂操作机器人，全身状态估计与 `SLAM` 的关键组成部分。
