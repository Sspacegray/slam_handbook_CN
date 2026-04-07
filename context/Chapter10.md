# 第 10 章 Event-based SLAM（事件相机 SLAM）

### 本章概览

细心的读者会注意到，`SLAM` 是一类对真实世界应用至关重要的问题，因为它关系到空间关系理解以及与环境的交互。对 `SLAM` 而言，主传感器的选择会直接决定系统的成功与适应性。Visual `SLAM` 是最普及的一大类方法，因为 cameras 普遍可得、价格可承受，并能产生直观且信息丰富的信号，让机器人在广泛场景中感知世界，例如无需额外基础设施就能构建轻量系统。

尽管现有进展已经很多，最先进的人工视觉系统在真实世界任务中的有效性（鲁棒性和效率）仍不如生物视觉系统。标准 cameras 以固定帧率感知世界，而这一帧率与场景动态无关。因此，它们会在帧与帧之间“失明”，引入延迟，可能丢失跟踪；并且当场景中没有物体运动时，还会产生大量冗余数据。本章追求的正是这样一个富有前瞻性的目标：借助受生物启发的 silicon retinas，也就是 “event cameras”，构建 fast（不受固定帧率限制）、low-power，并且对广泛照明条件都更鲁棒的 visual `SLAM` 系统，因为这类传感器克服了标准相机的若干局限（见 Fig. `10.1`）。

本章将首先介绍 event cameras 的工作原理（`10.1`），以及相关挑战和应用（`10.2`）；随后聚焦事件相机数据的处理方法（`10.3`），以及相应的前端（`10.4`）和后端（`10.5`）。最后，作者会讨论 state-of-the-art systems（`10.6`）、数据集、模拟器与 benchmarks（`10.7`），并在 `10.8` 中总结新趋势和进一步阅读。

## 10.1 传感器说明

事件相机与传统 frame camera 的根本区别，在于它不是周期性读出整幅图像，而是在单个像素的亮度发生足够变化时，立即输出对应事件。换言之，事件相机输出的不是图像序列，而是一串随时间到达的亮度变化“脉冲”。

### 10.1.1 工作原理

与传统相机依赖外部时钟、以固定频率采集整帧图像不同，像 `Dynamic Vision Sensor (DVS)` 这样的事件相机，其像素彼此独立工作，只要场景亮度发生变化，就会异步地产生响应。Figure `10.1` 展示了一个使用下视 `DAVIS` 相机执行自主飞行的无人机例子，其中事件相机的高速和高动态范围特性，被用来应对困难光照条件。造成像素级亮度变化的原因，既可能是场景照明变化，例如灯光闪烁，也可能来自相机与场景之间的相对运动，包括场景中独立运动物体带来的变化。

图 `10.1` 更完整地说，画面展示的是一架搭载下视 `DAVIS` 相机 [111]（`240×180 px`）的无人机，使用 visual-inertial odometry (`VIO`) 算法 [943] 执行自主飞行。插图左侧是在灰度帧中检测并跟踪的特征点，受到明显 motion blur 影响；中间则是对 warped events 做 motion compensation 后得到的更清晰图像；红 / 蓝事件数据则清楚响应场景轮廓。相同的 `VIO` 算法 [943] 也被展示在更高速的场景中，例如把事件相机绑在绳子上高速旋转。图像引自 [361]（©2020 IEEE）。

因此，事件相机的输出不是规则采样的图像帧，而是一串数字“事件（events）”或“脉冲（spikes）”。每个事件都表示某个像素处对数亮度发生了一次变化。这种编码方式本身就受到生物视觉通路中 spike 机制的启发。

更具体地说，每个像素都会在自己上一次触发事件时记住对应的对数亮度 `L`，随后持续监视新的亮度是否相对于这个记忆值产生了足够大的变化 `ΔL`。当该变化达到阈值 `C` 时，就会发送一个事件:

`ΔL = L(x_k, t_k) - L(x_k, t_k - Δt_k) = p_k C`             `(10.1)`

其中，事件本身写成 `e_k = (x_k, t_k, p_k)`，并从芯片中输出。这里 `x_k` 是像素在图像平面中的 `x, y` 位置，`t_k` 是时间戳，`p_k ∈ {+1, -1}` 是 1 bit 极性，表示该变化是亮度增加还是亮度降低，而 `Δt_k` 则表示同一像素距离前一次事件经过的时间。

事件相机是一类典型的数据驱动传感器: 它的输出取决于场景中运动或光照变化的强弱。运动越快，单位时间内产生的事件越多，因为每个像素都会根据自身监测到的亮度变化速度自适应地调整采样频率。Figure `10.2` 中作者用一个旋转圆盘上的黑点做例子，对比了标准相机和事件相机的输出。标准相机只在帧采样时刻输出图像，而事件相机会把亮度变化连续地编码为时空中的螺旋事件轨迹，其中红色表示正事件 `ON spikes`，蓝色表示负事件 `OFF spikes`。

图 `10.2` 从三个角度解释事件相机的工作原理: `(a)` 人眼视网膜的三层模型，以及与之对应的 `DVS` 像素电路；`(b)` `DVS` 像素把光信号转换为事件（spikes）的工作示意图，其中信号颜色与 `(a)` 中各层一一对应；`(c)` 标准相机与事件相机面对“旋转圆盘上的黑点”这一视觉刺激时的响应对比。事件相机会连续发送亮度变化，在时空中形成螺旋状事件轨迹；红色表示正事件（`ON spikes`），蓝色表示负事件（`OFF spikes`）。图像改编自 [883]（©2014 IEEE）。

作者随后解释了其生物启发来源，也就是视觉系统中的 `transient pathway`。按照双通路假说，背侧通路，也被称为 `transient` 或 `where` pathway，主要处理动态视觉信息，例如场景运动；而腹侧通路，也称 `sustained` 或 `what` pathway，则更偏向物体识别与视觉辨认。`DVS` 对应的正是从 photoreceptor 到 ganglion cell 的那部分 transient pathway，只不过它采用了经过简化的三层像素设计，在生物保真度与电路稳定性之间取得折中。三层结构分别实现光信号转换、delta-modulation 与比较功能。

还有一些相机，例如 `Asynchronous time-based image sensor (ATIS)` 或 `Dynamic and Active-Pixel Vision Sensor (DAVIS)`，则试图同时模拟两条视觉通路，因此会同时输出两类信号: `DVS events` 以及灰度信息，例如普通图像帧。关于不同事件相机类型的更多细节，作者引导读者参考文献 `[883, 361]`。

### 10.1.2 事件相机的优势

事件相机的感知原理与主流曝光式相机完全不同，因此也带来了一组非常鲜明的优势。

首先是极高的时间分辨率。事件通常以微秒级时间精度被触发和时间戳化，因此可以观测非常快速的运动，而不会像帧式相机那样因帧间空白期过长而“失明”，也不会在高速运动下产生严重 `motion blur`。第二个优势是低延迟。由于每个像素都独立工作，事件一旦被触发便会以亚毫秒级延迟输出，不必等待整帧曝光结束。

第三个优势是低功耗和低带宽。事件只编码真正发生变化的部分，因此在大量静态区域中不会产生冗余信息。这意味着功耗与数据带宽都可显著低于以相同时间精度工作的传统相机。第四个优势则是极高动态范围 `HDR`。事件相机通常能覆盖超过 `120dB` 的亮度范围，而传统相机常见只有约 `60dB`。这使事件相机可以同时感知非常暗与非常亮的区域，而不过曝或欠曝。

这些特性让事件相机在高速机器人、强明暗对比环境、快速飞行器、AR/VR inside-out tracking 等场景中特别有吸引力。

### 10.1.3 当前设备与趋势

对于刚进入这一方向的研究者，一个常见问题是: 到底该买哪种 event camera，才能解决自己的 `SLAM` 问题？作者指出，尽管事件相机的设计已经出现许多变体，但真正商业化的型号并不多。目前主要厂商包括 `SONY`、`Samsung`、`iniVation / SynSense`、`Prophesee`、`Omnivision` 等。

近年来最明显的趋势之一，是像素尺寸不断缩小，从早期 `DVS128` 的约 `40µm` 降到不足 `5µm`。但事件像素所需电路比传统像素复杂得多，因此要在有限面积内继续缩小像素，本身就是很有挑战的芯片工艺问题。为此，业界逐渐采用 stacked technology 和 backside illumination 等设计，以提升 fill factor 并减小光敏区域间隙。

另一个趋势，是部分设备同时输出事件与灰度图像。早期的 `DAVIS` 或 `ATIS` 这类器件，就同时建模了类似生物视觉中的 transient pathway 和 sustained pathway，因此既输出 `DVS` 事件，也输出灰度图像。较新的高清 event camera 有些则取消灰度输出，将更多面积让给事件阵列本身。也有少量设备支持 RGB 颜色通道的事件输出，不过颜色在许多运动感知任务中并非必要。

同时，越来越多的设备开始集成 `IMU`。这对 `SLAM` 很关键，因为 `IMU` 是视觉的天然补充，能显著提升 `VIO` 和 `SLAM` 的鲁棒性与精度。

作者特别提醒，高空间分辨率的 event camera 并不一定天然优于低分辨率型号。对于 `SLAM` 来说，高分辨率带来的往往不是“白捡的精度”，而是更高带宽、更大处理负载，以及动辄每秒上亿事件的实时处理难题。目前还没有一种通用的算法-硬件组合，能够在不做聚合、降采样或数组化转换的前提下，稳定实时处理如此高的事件率。因此，对计算资源受限的机器人平台而言，较低分辨率，例如 `QVGA` 级别，往往反而更适合算法原型验证和实时运行。未来值得关注的方向包括 hybrid sensors、foveated sensors 以及更紧密结合 `IMU`、frame camera、`LiDAR` 的多模态设备。

## 10.2 挑战与应用

事件相机是一项革命性的视觉采集技术，但它也迫使研究者重新思考视觉算法和硬件设计。最核心的挑战有三类。

第一，事件输出是时空稀疏且异步的，而标准图像是时空稠密且同步的。这意味着绝大多数基于图像序列的经典 `SLAM` 方法无法直接搬过来。第二，事件是运动相关的观测。与普通图像不同，每个事件只携带亮度增加或减少这一位信息，而这一变化既由场景纹理决定，也由相机与场景之间的相对运动决定。第三，事件相机并不“无噪声”。它会受到光子散粒噪声、电路噪声、照度依赖、阈值不一致、低功耗工作模式带来的非理想性等影响。

这些问题共同推动了 event-based `SLAM` 的发展。它不只是一个“把图像换成事件”的问题，而是要求我们重新思考: 应该如何从事件里提取位姿和深度信息？该使用什么地图和轨迹表示，才能反映事件的准连续时间特性？如何在运动依赖的观测下建立数据关联？又如何避免把传统帧式系统中的时间量化、延迟和冗余带入新的系统？

作者还指出，event-based `SLAM` 与其他 event 视觉任务之间一直存在强耦合关系。例如，事件图像重建、光流估计、跟踪与分割等研究，都反过来推动了 `SLAM` 的发展；很多早期系统就是在旋转 `SLAM`、六自由度 `SLAM`、光流和图像重建共同作用下诞生的。

## 10.3 Event-based SLAM 方法概览与分类

作者首先按“同时处理多少事件”来给 event-based `SLAM` 方法做最粗粒度分类。第一类是 `event-by-event` 方法，也就是每到来一个单独事件，系统状态，例如 scene map 或 camera trajectory，就可能更新一次。这种方法理论上可达到最低延迟。第二类则是按 groups / batches / slices / packets 处理事件的方法，它们会引入一定 latency，但往往更便于实现和优化。在后一类方法里，一个非常关键的设计选择就是 packet size 如何设定，常见方案包括固定事件数、固定时间长度，以及两者结合的 hybrid criteria。

与这一维度正交，作者又区分了 `model-based` 和 `data-driven` approaches。前者更多依赖对事件成像机理和几何关系的显式建模；后者则把机器学习模型引入事件处理流程。再模仿 frame-based `SLAM` 的习惯，event-based 方法还可分为 `indirect` 与 `direct` 两类。`Indirect` 方法通常先提取 event corners、lines、normal flow 等特征，再沿用更传统的几何 `SLAM` pipeline；`direct` 方法则更倾向于把 event data 直接映射为 motion 和 scene parameters。

这一划分背后，还对应着不同的 objective / loss design。`Indirect` 方法往往分成两个步骤: 先把事件“转化”为几何原语，再做几何 `SLAM`；而 `direct` 方法通常试图用一步完成从 event data 到运动和场景参数的映射，它们的目标函数更多体现为 photometric- 或 event-rate-based alignment。在这类方法中，事件生成模型 `(10.1)` 及其线性化形式 [361] 是估计方法设计的核心基础。

`Figure 10.3` 说明 event-based `SLAM` 正在被积极探索，系统覆盖了从经典方法到更近年的 deep learning solutions 的大量路线。由于事件由图像平面上的移动边缘触发，因此把场景地图恢复为边缘形式，例如稀疏或半稠密 `3D` edge maps，是很自然的。示例图改编自 `EVO` [913]、`EDS` [461]、`CMax-SLAM` [416]、Kim 等人 [570]、`ESVO2` [811]、`DEVO` [584] 和 Wang 等人 [1153]。

作者特别强调，`data association` 是 event-based vision，尤其是 `SLAM` 中的中心问题。由于事件相机具有极高的时间分辨率，事件之间的关联通常借助时空邻近性来完成；相应地，文献中既有 hard-association，也有 soft-association 策略。

书中进一步指出，上述各种分类各有利弊。用事件相机做 `SLAM` 的难点很高，因此研究历史上实际上是沿着多条复杂度轴逐步推进的: 未知量自由度从低到高，运动类型从纯旋转或 `2D` 场景扩展到完整 `6-DoF`，场景复杂度从简单纹理走向复杂静态和动态场景，任务也从孤立的 `SLAM` 问题不断与 optical flow、tracking、segmentation 等其他 event vision 任务交织在一起。

作者最后指出，从时间线上看，早期 event-based `SLAM` 更偏向 model-based methods，而较新的工作则越来越多地尝试利用 deep-learning–based approaches 的潜力。

## 10.4 Event-based SLAM 系统的前端

Event-based `SLAM` 前端往往包含若干相互协作的子模块，例如特征提取、数据关联、初始化、位姿跟踪、深度估计等。从输入输出角度看，前端接收原始事件流以及可能的辅助信息，如相机标定参数，并输出事件相机位姿序列和场景地图的初步估计。随后，后端在较低频率下对这些变量进行精化，并可将结果反馈给前端，帮助减少漂移。

`Figure 10.4` 概括了一个 event-based `SLAM` pipeline：前端负责计算地图和相机位姿，后端负责进一步精化地图与位姿。由于事件响应的是移动边缘，恢复出的地图通常也是 edge / gradient map。图中的示例是一个 direct rotational `SLAM` pipeline，其 pose 仅由 rotations 构成，而地图退化为 panoramic map [417]；绝对强度图还可通过 `Poisson integration` 恢复。

由于事件是由图像边缘运动触发的，因此在恒定光照假设下，移动的事件相机本质上相当于一个异步边缘探测器。这意味着很多 event-based `SLAM` 系统最终恢复的地图，天然更接近边缘图、梯度图或半稠密结构，而不是传统意义上的全纹理地图。

### 10.4.1 预处理与事件表示

在 `SLAM` 场景中，事件相机会在运动过程中持续输出事件。由于事件稀疏且具有微秒级时间分辨率，理论上每个事件都对应一个不同的相机位姿。这与传统 frame camera 的范式完全不同: 在普通图像中，同一帧内所有像素共享同一个时间戳，因此也共享同一个相机位姿，而这正是经典 multi-view geometry [437] 的基本前提。

正因如此，许多 event-based `SLAM` 方法都会先把原始事件流转换成某种替代表示，例如 event images、time maps / `time surfaces`、voxel grids 等 [361]。这样做的原因很多，包括:

- 与 conventional computer vision methods 兼容。
- 更易于人理解和调试。
- 更方便构造基于图像的 learning pipelines。

但作者同时指出，这一转换过程往往伴随着信息量化，例如把时间相近的事件分组，或者在没有事件的像素位置显式填零。前者会损失原生时间分辨率，后者会损失事件数据原本的 sparsity。

因此，`event representations` 本身已成为一个独立且重要的研究主题 [361, 370]。它通常位于前端的第一步，并会深刻影响后续所有处理阶段: 事件先被转成更熟悉的表示，再喂给传统图像 `SLAM` 方法，或喂给基于图像的深度网络。作者指出，这在很大程度上反映了社区仍在探索“到底如何从事件流中提取最有用信息”，因此会自然地尝试复用成熟的 frame-based methods。

不同系统甚至会在前端内部为不同模块选择不同表示。例如 `EVO` [913] 在其 mapping module 中使用 raw events 以及 `EMVS` [915]，而在 camera tracking module 中则使用 event images。理想情况下，我们希望设计出既保留 event camera 高速、稀疏等核心优势，又不会退化成传统相机问题的事件表示和 `SLAM` 方法；但作者坦言，这仍然是一个新兴研究方向，尤其需要从根本上重新思考异步视觉处理。

### 10.4.2 间接法

`Indirect methods` 一般包含两个步骤。首先，它们从事件中提取和跟踪某类几何原语，例如角点、线段或轨迹；然后使用与传统视觉 `VO` / `SLAM` 类似的几何流程，来估计相机运动和场景结构。

这种路线的优点，是能把海量事件压缩成少量高信息密度的几何原语，从而更高效地利用计算资源，并复用成熟的多视图几何工具箱。通常，位姿跟踪可被表述为一个 feature alignment 问题，通过最小化重投影误差等几何目标函数，在给定地图的条件下恢复相机姿态；而场景结构则可通过对应特征的三角化或反投影获得。

但这条路线也高度依赖稳健的特征提取与跟踪，而这恰恰是事件视觉里仍未完全成熟的一环。事件稀疏、噪声大、运动依赖强，因此即便是“检测角点”这样在帧相机中已经很成熟的问题，在事件流中也更难处理。为缓解这些问题，一些系统会将事件与灰度图像或 `IMU` 融合。

### 10.4.3 直接法

`Direct methods` 不显式提取几何特征，而是尽可能直接利用所有事件来估计运动和场景结构。若事件率过高超出处理能力，系统通常会引入去噪、稀疏采样或其他数据约简机制。

与 indirect 方法相比，direct 方法更强调事件生成模型、事件极性和时空一致性。它们通常通过优化某种目标函数来完成位姿或场景估计，例如 photometric error，或更常见的 event alignment / motion compensation 目标。其中一类最有代表性的方法是 `contrast maximization (CMax)`。其核心思想是“通过估计运动来撤销运动”，也就是寻找那组能把事件正确 warp 到共同参考系上的参数，使得 `Image of Warped Events (IWE)` 最清晰、最聚焦。相关目标可以用方差、梯度、离散度等形式表达。

Direct 方法更充分利用了事件的准连续时间特性，因此在高速场景中常表现出更大潜力；但它们对初始化、噪声建模以及事件相机物理模型的准确性要求也更高。有些问题里，若事件被错误地 warp 到少量像素或单条线附近，还可能产生不希望的全局最优点。

### 10.4.4 基于模型与基于学习的方法

目前绝大多数 event-based `SLAM` 仍是 hand-crafted 的 model-based 系统，即依赖人工设计的物理和几何先验。但近年深度学习已开始快速进入这一领域。一类方法先把事件转换为图像式表示，再用常规神经网络处理；另一类方法则直接使用 `Spiking Neural Networks (SNNs)` 等结构处理事件流。

学习方法又可分为 supervised 与 self-supervised。Self-supervised 方法通常利用事件自身或与其共视场的灰度图像，通过时间一致性或光度一致性损失来训练深度和位姿网络；supervised 方法则直接依赖 ground truth 监督。近年大量车载、无人机和多模态数据集的出现，为这类方法提供了必要训练数据。

学习方法可以替代某一部分 `SLAM` 管线，如特征提取和跟踪，也可能试图做 end-to-end 替代。但它们往往需要大量训练数据，并容易受到 domain shift 影响。作者整体上的态度是: learning 确实很有前景，但 event camera 的物理成像机制非常强，因此真正稳健的方法往往需要 model-aware 设计，而不是纯黑盒。

## 10.5 Event-based SLAM 系统的后端

`SLAM` 后端的职责，是提升前端输出变量与传感器数据之间的一致性，减少误差传播，并增强系统的精度和鲁棒性。与传统视觉 `SLAM` 类似，`bundle adjustment (BA)` 仍是最重要的候选工具之一。

不过，作者指出，event-based `BA` 仍处于相当早期的阶段。许多系统事实上根本没有严格意义上的 refinement back-end，而是采用 tracking-and-mapping 并行结构，让跟踪与建图模块相互提供输入。事件相机最大的优势之一是低延迟，因此很多系统历史上更优先追求简单和快速，而不是引入昂贵的全局优化。

另一方面，event-based 后端之所以困难，还在于它要联合优化大量彼此相关的变量，而事件本身又噪声大、与运动耦合强，导致优化问题高维、代价大，也更容易陷入局部极值。书中据此前端输出类型，将后端也分为 indirect 与 direct 两类。

Indirect back-end 基本继承自传统特征式视觉 `SLAM`。它对事件流中的角点、线段等几何原语做优化，目标一般是最小化重投影误差。这条路线的优点，是可直接复用传统 `SLAM` 中成熟且鲁棒的工具；但它也丢掉了事件中大量无法被少量几何特征压缩掉的信息，而且当前事件角点的稳定性通常仍不如帧式角点。

Direct back-end 则直接面向传感器数据本身，而非几何原语。若系统还配有灰度图像，那么后端往往直接复用帧式视觉系统的 photometric `BA`；而在 event-only 场景中，直接后端可以围绕 motion compensation、`CMax` 或基于事件生成模型的光度误差构造目标函数。由于每个事件信息量都很小，而 `SLAM` 状态变量又很多，这类方法往往需要非常多事件才能支撑精确优化，因此在计算资源、功耗和延迟之间的平衡仍是开放问题。

## 10.6 最先进系统

Table `10.1` 汇总了 event-based `VO/SLAM` 的具体系统，并按照前文引入的分类，总结了它们的一些特征（direct、indirect 等）。虽然本章不可能逐一详细描述所有系统，但原书指出了几条值得注意的趋势。

首先，当前文献仍由 model-based systems 主导；data-driven approaches 还没有完全接管这一领域，尽管未来很可能像其他 computer vision 任务一样发生转变。其次，自这一方向诞生以来，事件相机 `SLAM` 一直是在不同假设下逐步提升复杂度，包括 `(i)` 相机运动类型、`(ii)` 场景类型，以及 `(iii)` 用来简化问题的附加传感器或先验信息。例如，给 event camera 加一个 depth sensor 会减轻仅靠 events 做 depth estimation 的负担，而 `IMU` 则可提供精确角速度信息。

一旦某个 event-based 方法表现良好，它往往会沿着一种近乎标准化的“exploitation roadmap”逐步扩展，类似于 frame-based `SLAM`：例如，单目方法 [913] 可以扩展到 stereo 或 multi-camera 场景 [381]；event-only 方法如 [1301]（或 [584]）则可通过与惯性数据融合 [684, 811]（或 [408]）来增强鲁棒性；基础系统还可以继续扩展到 omnidirectional lenses 等设定。

不过，event-based `SLAM` 仍然是一个 emerging field，因此仍处于强烈的 exploration 阶段。从 Table `10.1` 就能明显看出这一点：系统背后的思想和原理非常多样，继而导向了不同的地图表示、事件表示、损失函数等。围绕如何利用传感器的原生特性来设计新的状态估计方法，仍有大量探索空间。

| System | M/DL | I/D | Event representation | BA | Motion | Scene | Input | Remarks |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Cook [229] | M | D | Event Frame | ✗ | Rot | Natural | E | Interacting network using optical flow |
| Weikersdorfer [1174] | M | I | Individual Event | ✗ | Planar | 2D B&W | E | First filter-based Ev-SLAM |
| PF-SMT [569] | M | D | Individual Event | ✗ | Rot | Natural | E | Two interleaved Bayesian filters |
| Censi [160] | M | D | Event Packet | ✗ | 6DoF | B&W | E+F+D | Filter-based VO based on image gradient |
| EB-SLAM-3D [1175] | M | D | Individual Event | ✗ | 6DoF | Natural | E+D | Augment events with depth sensor |
| Yuan [1259] | M | I | Event Frame | ✗ | 6DoF | B&W | E+I+M | Vertical line-based camera tracking |
| Kueng [610] | M | I | Local Point Set | ✗ | 6DoF | Natural | E+F | Event-based feature tracking VO |
| ETAM [570] | M | D | Individual Events | ✗ | 6DoF | Natural | E | Three interleaved filters |
| CMax-ω [356] | M | D | Individual Events | ✗ | Rot | Natural | E | Contrast Maximization |
| EVO [913] | M | D | Edge Map | ✗ | 6DoF | Natural | E | Event-event geometric alignment |
| EVIO [1302] | M | I | Point sets | ✗ | 6DoF | Natural | E+I | Filter-based and MC features |
| Rebecq [914] | M | I | MC Event Images | ✓ | 6DoF | Natural | E+I | Feature-based, sliding-window back-end |
| RTPT [924] | M | D | Individual Events | ✗ | Rot | Natural | E | Panoramic tracker and mapper |
| Gallego [358] | M | D | Individual Events | ✗ | 6DoF | Natural | E+M | Resilient sensor model |
| Mueggler [778] | M | D | Individual Events | ✓ | 6DoF | Natural | E+I+M | Continuous-time pose estimator |
| USLAM [943] | M | I | MC Event Images | ✓ | 6DoF | Natural | E+F+I | Sensor fusion & sliding-window back-end |
| Chin [204] | M | I | Event Frames | ✓ | Rot | Stars | E | Tailored to star tracking |
| ESVO [1301] | M | D | Timesurfaces (TS) | ✗ | 6DoF | Natural | 2E | Stereo matching on TS patches |
| Hadviger [425] | M | I | Corners on TS | ✗ | 6DoF | Natural | 2E | Cross-corr. feature descriptors |
| CMax-GAE [571] | M | D | Individual Events | ✗ | Rot | Natural | E | Contrast maximization |
| EKLT-VIO [724] | M | I | Individual events | ✓ | 6DoF | Natural | E+F+I | EKLT tracker and VIO back-end |
| EDS [461] | M | D | Event images | ✓ | 6DoF | Natural | E+F | Frame-based back-end (DSO) |
| CB-VIO [683] | M | I | Individual events | ✓ | 6DoF | Natural | E+F+I | Feature tracker and VIO back-end |
| Wang [1153] | M | I | Binary images | ✓ | 6DoF | Natural | 2E | Feature matching |
| El Moudni [303] | M | D | TimeSurfaces TS | ✗ | 6DoF | Natural | 2E | Use ESVO tracker and EMVS mapper |
| ESVIO [183] | M | I | Time surfaces (TS) | ✓ | 6DoF | Natural | 2E+2F+I | Feature tracking from TS |
| ESVIO-direct [684] | M | D | Timesurfaces (TS) | ✓ | 6DoF | Natural | 2E+I | Extension of ESVO |
| PL-EVIO [410] | M | I | Time surfaces (TS) | ✓ | 6DoF | Natural | E+F+I | Point & line features, sliding-window BA |
| CMax-SLAM [416] | M | D | Individual Events | ✓ | Rot | Natural | E | Contrast Maximization refines motion |
| EVI-SAM [409] | M | D,I | Individual Events | ✓ | 6DoF | Natural | E+F+I | Dense mapping |
| Zuo [1316] | M | D | Individual Event | ✗ | 6DoF | Natural | E+D | Augment events with depth sensor |
| DEVO [584] | DL | I | Event voxel grids | ✓ | 6DoF | Natural | E | Event-version of DPVO [1083] |
| EMBA [417] | M | D | Individual Events | ✓ | Rot | Natural | E | Refines motion and gradient map |
| EPBA [418] | M | D | Individual Events | ✓ | Rot | Natural | E | Refines motion and intensity map |
| ES-PTAM [381] | M | D | Events (also as frames) | ✗ | 6DoF | Natural | 2E | Use EVO tracker and EMVS mapper |
| ESVO2 [811] | M | D | Timesurfaces (TS) | ✓ | 6DoF | Natural | 2E+I | Extension of ESVO |
| DEIO [408] | DL | I | Event voxel grids | ✓ | 6DoF | Natural | E+I | Extension of DEVO and DPVO |

表 `10.1` 按时间顺序汇总了 `Event-based Visual SLAM` 方法。各列依次表示: 方法类型（`Model-based` 或 `Deep-Learning-based`、`Direct` 或 `Indirect`）、是否包含全局 refinement 模块（即 back-end / `BA`）、可处理的相机运动类型（`Rotational`、`Planar`、`6-DoF`）与场景类型（高对比黑白等），以及输入数据类型（事件相机 `E`、frame-based camera `F`、depth sensor `D`、`IMU`、Map `M`），其中 `2E` 表示 stereo events（两台事件相机）。

## 10.7 数据集、模拟器与基准

事件相机研究近年的快速增长，离不开数据集和模拟器的持续丰富。由于事件方法往往对精确时间同步、高质量 ground truth 位姿、深度和多传感器标定有很高要求，因此好的 benchmark 对这一领域尤为关键。

### 10.7.1 模拟器

公开可用的 event camera 模拟器已经相当丰富，它们的共同目标，是生成高质量的合成事件数据。`ESIM` 是其中最有代表性的工具之一。它是在更早期模拟器基础上的发展版，而更早的某些方法，其实只是简单地对两帧图像做差分阈值化，从而得到看起来像事件输出的边缘图。`ESIM` 则真正开始模拟事件相机的工作原理，并把渲染引擎与事件模拟器紧密耦合起来，使后者能够根据视觉信号的动态特性自适应地调整渲染频率。

`CARLA` 中的 event camera simulator 又把 `ESIM` 的思路扩展到了更丰富、更复杂的自动驾驶场景。在某些基于事件学习单目深度的工作里，`CARLA` 会从模拟器渲染图像出发，逐像素计算亮度变化，按与 `ESIM` 类似的方式合成事件。Figure `10.5` 就展示了 `CARLA` 中生成的 `RGB` 图像与对应事件。

随着 learning-based 方法越来越依赖海量训练数据，而真实事件数据又由于传感器新颖性而十分稀缺，研究者又开发了 `Video to Events (Vid2E)`。它的目标是把任何已有的普通视频转换成合成事件数据，从而缩小传统视觉数据集和事件视觉数据集之间的鸿沟，使现有视频数据也能被用来训练事件网络。`Vid2E` 的做法是把 `ESIM` 与自适应视频插帧结合起来。因为 `ESIM` 本身可以在任意时间分辨率下生成事件，但普通视频通常只提供低帧率、固定时间间隔的亮度观测，所以必须先用 `Super SloMo` 等插帧方法恢复出更高时间分辨率的中间帧，再交给 `ESIM`。这里中间帧数量的选择非常关键: 如果太少，会导致亮度信号混叠；如果太多，又会带来很大计算负担。

作者特别强调，event camera 模拟器最关键的难点之一，是如何准确建模噪声，以减小 `sim-to-real gap`。例如 `ESIM` 及其衍生系统，采用了一个基于经验观察的简单噪声模型: 事件阈值 `C` 不是固定值，而是服从高斯分布。也就是说，在仿真每一步中，`ESIM` 都会从 `N(C, σ_C²)` 中采样阈值，并允许正、负对比度阈值 `C+` 与 `C-` 分别设置，以更接近真实相机。同时，诸如空间和时间上的阈值波动、电子噪声、像素有限带宽等效应，也都值得被纳入模拟器。

`Vid2E` 对光照采用的是理想事件相机模型。相比之下，`V2E` 基于 `DVS` 电路进一步提出了更真实的噪声模拟器，是首个把 temporal noise、leak events 和与亮度相关的有限带宽都纳入的事件模拟器，同时也保留了与 `Vid2E` 相同的高斯阈值分布。像 `Vid2E` 一样，`V2E` 也使用 `Super SloMo` 来提升输入视频的时间分辨率。Figure `10.5` 对 `V2E` 的处理架构做了详细展示。

`Figure 10.5` 展示了两类 event camera simulators：`(a)` 在 `CARLA` 中使用 `ESIM` 生成的 `RGB` 图像与对应事件 [279]；`(b)` `V2E` 工具 [481] 的详细数据处理流程。

此外，还有更偏 learning-based 的模拟器。`EventGAN` 提出一种端到端深度学习方法，利用已有图像标注数据，从时间相邻的两帧图像出发，通过 `U-Net` 编码器-解码器网络生成事件。它并不直接输出单个事件集合，而是为每个 polarity 生成一个三维时空 voxel grid，这也是许多人工神经网络常用的输入表示。`VISTA 2.0` 则把 `RGB`、`LiDAR` 和 event camera 等多传感器模拟统一进一个自动驾驶策略学习框架中，并利用高保真真实世界数据构造不同天气、光照和道路条件，以提升 sim-to-real transfer 能力。

最后，`Video to Continuous Events (V2CE)` 关注的是时间戳真实性问题。`Vid2E` 与 `V2E` 生成的事件往往仍然发生在离散时间点上，而真实事件是连续时间到达的。对于那些对时间戳分布非常敏感的任务来说，这种差异会显著放大 domain shift。`V2CE` 因此提出一个两阶段流程: 第一阶段利用监督式 `3D U-Net` 预测两个 voxel grids，分别对应正、负极性；第二阶段则再从这些 voxel grid 中恢复更精确的事件时间戳。作者还指出，`V2CE` 在强光饱和区域以及“理想事件生成模型不再成立”的边缘区域上，也能较准确地产生事件。

### 10.7.2 数据集与基准

面向 `Visual Odometry` 与 `SLAM` 的 event-based 数据集，在 `ECDS` 发布之后显著增加。`ECDS` 是首个同时提供同步事件、`IMU` 和 `6-DoF` ground-truth 相机位姿的数据集。在它之前，一些更早的数据集只包含纯旋转运动、简单高对比场景，或者 ground truth 只能依赖 `IMU`、云台编码器或轮式里程计，因此本身容易漂移。`ECDS` 则包含手持 `6-DoF` 运动、慢速与高速两类轨迹、多种场景，并通过 motion-capture system 提供高精度真值。它包含 `11` 个真实事件序列，以及 `2` 个由早期 `ESIM` 版本生成的合成序列。

`Figure 10.6` 给出了若干 Event-SLAM datasets 的细节：`(a)` `DSEC` [372] 数据集中安装在车辆顶部的传感器套件；`(b)` `EDS` [461] 数据集中用于让多传感器共享空间对齐视场的 beamsplitter 结构。

表 `10.2` 对 event-based `SLAM` 数据集做了按时间排序的总览，传感器记号与表 `10.1` 保持一致；关于 stereo 与多传感器数据集的进一步说明，可参考综述 [380]:

| Dataset | Platforms | Pixel Resolution | Sensors |
| --- | --- | --- | --- |
| ECDS [777] | Hand-held | 240×180 | E, F, I |
| RPG-stereo [1300] | Hand-held | 240×180 | 2E |
| MVSEC [1303] | Hand-held, Drone, Car, Bike | 346×240 | 2E, 2F, I, Lidar, GPS |
| UZH-FPV [257] | Drone | 346×260 | E, F, I |
| EV-IMO [768] | Hand-held | 346×260 | E, F, I, Depth |
| EV-IMO2 [126] | Hand-held | 640×480 | 3E, F, I, Depth |
| DSEC [372] | Car | 640×480 | 2E, 2F, Lidar, GPS |
| TUM-VIE [583] | Hand-held | 1280×720 | 2E, 2F, I |
| EDS [461] | Hand-held | 640×480 | E, F(RGB), I |
| VECtor [363] | Hand-held | 640×480 | 2E, 2F, RGB-D, I, Lidar |
| M2DGR [1249] | Ground Robot | 640×480 | E, F, I, Lidar, GPS, Thermal |
| ViViD++ [634] | Hand-held, Car | 240×180, 640×480 | E, F, RGB-D, Thermal, Lidar, GPS |
| FusionPortable [522] | Hand-held, Quadruped Robot | 346×240 | 2E, 2F, I, Lidar, GPS |
| Stereo HKU-VIO [183] | Hand-held | 346×260 | 2E, 2F, I |
| M3DE [163] | Drone, Car, Quadruped Robot | 1280×720 | 2E, 2F, I, Lidar, GPS |
| CoSEC [860] | Car | 1280×720 | 2E, 2F, I, Lidar, GPS |

`RPG stereo dataset` 则由 `8` 条手持 stereo `DAVIS` 办公室序列和一个合成序列组成。虽然它不提供 ground-truth depth，但具有基于 motion capture 的准确位姿真值，因此非常适合做 event stereo `SLAM` 的原型验证与算法评估。

`MVSEC` 是第一个在多平台上系统性提供 ground-truth depth 的事件数据集。它覆盖手持平台、六旋翼、汽车和摩托车，同时配备 `3D LiDAR`、`IMU` 和标准帧相机，采集场景既有室内也有室外，并涵盖多种光照与速度条件。由于其序列较长、真值较全，因此被广泛用于位姿估计、建图、避障和 `3D reconstruction`。

`UZH-FPV` 主要面向自主无人机竞速。它使用一台搭载 `mDAVIS346` 的定制四旋翼，在室内外高速轨迹下采集数据，因此成为高动态 `VIO` 与高速事件视觉算法的重要 benchmark，也被用于多次会议与 workshop 竞赛。

`EV-IMO` 是首个专门用于 independently moving objects 分割的事件数据集，重点关注室内环境中的独立运动物体。它提供逐像素运动掩码、ground-truth egomotion 和 depth。`EV-IMO2` 则在其基础上增加了更多序列、更高质量的相机以及更复杂的场景，因此既是更有挑战性的 benchmark，也能作为训练更强 monocular / stereo event `SLAM` 方法的数据来源。

面向自动驾驶的代表数据集是 `DSEC`。它采用一套多相机平台，包含两台 `VGA` 分辨率事件相机、两台 `RGB` 相机、`Velodyne VLP-16 LiDAR` 和 `RTK GPS`。数据在瑞士的城市与乡村道路、白天、夜间和直射阳光等多种条件下采集，不仅提供 stereo matching 所需的 ground-truth depth，还提供 `Optical Flow` 和 `Disparity` benchmark。相关评测会使用 `N-pixel disparity error`、`MAE` 和 `RMSE` 等指标，衡量结合高分辨率事件数据与 `RGB` 帧的算法性能。

`TUM-VIE` 则采用百万像素级 `Prophesee Gen4` stereo event cameras，并同步采集 `200 Hz IMU` 与 `20 Hz` stereo grayscale frames。它包含手持与头戴两种安装方式，场景覆盖室内外、多类运动、体育活动、`HDR` 与低照度条件，目标是推动 `VIO`、`SLAM`、`3D reconstruction` 与多传感器融合在高分辨率事件感知上的发展。

`EDS` 使用定制 `beamsplitter` 设备，使 `RGB` 图像与事件能够共享同一光轴，这在早期数据集中非常少见。它包含高质量事件、彩色图像与 `IMU`，主要服务于 monocular `VIO`、optical flow、depth estimation 以及复杂运动和光照条件下的稳健视觉里程计研究。

`VECtor` 则是一个面向多模态 event `SLAM` 的综合 benchmark。其平台集成了 stereo 事件 / 帧相机、`RGB-D` 传感器、`128` 线 `LiDAR` 和九轴 `IMU`，覆盖小尺度室内环境与更大尺度、复杂照明条件的室内环境，强调 static / dynamic、low-light 与 `HDR` 等多种使用条件下对 `SLAM` 算法的可靠评估。

作者还提到了一批更近的多传感器数据集。`ViViD++` 使用包含 thermal camera 在内的多传感器平台，目标是推动能处理 poor visibility、运动扰动和外观变化的 `SLAM`。`FusionPortable` 则包含一个 quadruped robot，在走廊、餐厅、道路和花园等不同照明环境中采集数据。最后，`M3ED`，也常被称为 `MVSEC 2.0`，聚焦于机器人中的高速动态运动，组合了一百万像素 stereo event cameras、灰度与彩色相机、`64-beam LiDAR`、高质量 `IMU` 和 `RTK` 定位，并覆盖多平台、结构化与非结构化环境，同时提供位姿、深度和语义标签，因此特别适合推动 dynamic environments 下更稳健的 event-based 感知算法。

整体趋势很清楚: 事件数据集已经不再只服务于单一 `VO` 任务，而是越来越强调 stereo、`VIO`、dense depth、segmentation、多模态融合，以及在真实复杂场景中的鲁棒评估。

### 10.7.3 指标

理想情况下，`SLAM` 系统应该分别评估定位和建图模块的质量。但现实中，这两者往往是相互耦合的，而且高质量深度真值远比 `6-DoF` 轨迹真值更难获取，因此 depth error 往往会被间接吸收到轨迹误差评估里。

概念上，由于 event-based `SLAM` 和经典 `SLAM` 一样输出相机轨迹，因此其评估协议也大多继承自经典 `SLAM`。最常见的是 `Absolute Trajectory Error (ATE)` 与 `Relative Pose Error (RPE)`。`ATE` 评估相机在固定世界坐标系中的长期轨迹精度；`RPE` 评估相邻时间步或固定时间间隔内的相对位姿一致性。平移误差通常是估计位置与真值位置之间的欧氏距离，旋转误差则可通过 `SO(3)` 上的测地距离计算。

部分研究还会把位置误差再相对于平均场景深度或总行驶距离做归一化，从而让该误差对场景尺度或轨迹尺度保持不变。

此外，也有工作采用 `ARPE`、`ARRE`、`AEE` 等更细粒度指标，分别评估平均相对平移误差、平均相对旋转误差和端点误差。对于事件相机而言，由于系统往往善于处理剧烈快速运动，因此线速度与角速度误差在某些任务中也很有意义。

若深度估计被单独评估，则常使用不同截断距离下的平均深度误差、`RMSE`、`REL` 和 `completion` 等指标。不过，由于高质量 depth ground truth 依旧稀缺，很多 event-based `SLAM` 论文实际上仍以 pose accuracy 为主要评价标准。

## 10.8 延伸阅读与最新趋势

尽管 event-based `SLAM` 已取得长足进展，但作者认为，这项技术仍然非常年轻，真正重要的问题还远远没有解决。根本性的研究目标，是弄清楚该如何设计硬件和软件，才能在鲁棒性、延迟、功耗和精度等方面接近甚至超越生物视觉系统。

一个鲜明矛盾在于，传感器本身是异步的，但今天绝大多数系统仍运行在串行的 von Neumann 处理器上，这在效率和延迟上都不是最理想的。由此引出的关键趋势之一，便是 `neuromorphic computing`。若未来事件相机能与异步、脉冲式处理器、控制器和执行机构协同设计，那么在 AR/VR、持续在线定位等场景中，或许能真正逼近动物视觉那种高效、低功耗和实时反应能力。

作者据此强调，未来要真正释放 event camera 的潜力，往往不是简单把事件流塞进现有算法，而是需要对传感器、处理器和算法做共同设计（co-design）。在传感器层面，未来值得关注的包括更高分辨率、更低延迟的 hybrid sensors、模仿生物视觉中心凹结构的 foveated sensors，以及 near-sensor processing 等新硬件形态。

与此同时，多模态融合仍是最重要的应用趋势之一。事件相机与 frames、`IMU`、`LiDAR`、radar、structured light 等传感器的深度结合，几乎肯定会持续推动更稳健的 `SLAM` 系统出现。作者的结论相当明确: 真正强大的 event-based `SLAM`，很可能来自对传感器、计算架构和状态估计方法的整体重构，而不只是“让传统图像管线兼容事件流”。

