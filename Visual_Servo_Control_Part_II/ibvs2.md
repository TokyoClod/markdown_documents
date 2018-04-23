# Visual servo control, Part II: Advanced approaches

本文是视觉伺服教程的第二部分。在第一部分，我们介绍了基础概念和基本方法。这里我我们将会讨论更高级一些的方法，并介绍目前的一些研究。

正如第一部分介绍的，视觉伺服的结构依赖以下的关系：
$$\mathbf{\dot{s}}=\mathbf{L_s}\mathbf{v}_c \text{ (1)}$$
其中$s$是一系列导数通过交互矩阵$\bf L_s$与速度$\mathbf{v}_c$线性相关的几何特征。通过这种关系来设计使得当前视觉特征$\bf s$与目标值$\bf s^\star$之间误差$\bf e=s-s^\star$最小的控制结构。

一种经典的控制结构为：
$$\mathbf{v}_c=-\lambda \hat{\mathbf{L}_\mathbf{e}^+} \mathbf{e} \text{ (2)}$$
其中$\mathbf{L}_\mathbf{e}$定义为：
$$\mathbf{\dot{e}}=\mathbf{L}_\mathbf{e}\mathbf{v}_c \text{ (3)}$$
$\hat{\mathbf{L}_\mathbf{e}^+}$是$\mathbf{L}_\mathbf{e}$伪逆的近似。因为控制结构需要的3-D参数无法直接从图像中得到，所以必须对它进行估计。考虑基于位置的视觉伺服（PBVS）中,3-D参数同事在误差$e$与交互矩阵中出现，基本的IBVS中，每个点的深度出现在了关于平移运动的交互矩阵中。即使是选取$\hat{\mathbf{L}_\mathbf{e}^+} = \hat{\mathbf{L}_\mathbf{e}^\star}^+$，也需要每个点在目标位置的深度信息。在所有其他情况下，都需要在每次迭代过程中对每个点的深度尽心估计。

我们通过介绍两种交互矩阵的估计方法来开始第二部分。首先，我么你介绍如何利用对极几何来估计可以用来构建交互矩阵的三维参数。然后我们介绍如何直接估计$\hat{\mathbf{L}_\mathbf{e}^+}$的数值。通过这些方法，我们介绍视觉伺服中高级的方法。这些方法都是为了弥补PBVS与IBVS的缺点。然后我们考虑目标跟踪任务，其中，目标不再是静止的了。最后，我们概况了同时考虑“眼在手”系统与“眼对手”系统的建模问题。

## 3-D参数估计

如果使用的是校准过的立体相机，那么所有3-D参数都可以轻易获取。相似的，如果目标的3-D模型已知，那么通过位置估计算法也可以计算出所有3-D参数。但是，这样的估计收到图像噪声影响会变得很不稳定。可以通过同一场景在不同角度下拍摄的图像的对极几何来估计3-D参数。事实上，视觉伺服中，很容易获得这样的图像对：当前位置和目标位置。相反的，我们也可以通过视觉伺服任务中的图像量测直接估计交互矩阵。接下来，我们讨论这两种方法。

## 对极几何

给定一系列当前图像与目标图像对，并且相机已经经过校准，那么就可以恢复基础矩阵[Fundamental matrix](https://en.wikipedia.org/wiki/Fundamental_matrix_(computer_vision))，或者本质矩阵[essential matrix](https://en.wikipedia.org/wiki/Essential_matrix)并应用到视觉伺服中。

## 直接估计

## 高级的视觉伺服结构

我们现象介绍已经提出的一些视觉伺服结构，它们都是用来改进IBVS与PBVS的性能。第一种结合了它们的优点而且避免了它们的缺点。

### 混合视觉伺服

假设我们已经有了一个很好的$\bf \omega_c$的控制律，例如PBVS中的
$$\omega_c = -\lambda\,\theta\mathbf{u} \text{ (7)}$$
其中，如果目标物体的3-D模型可以获得，那么$\theta\mathbf{u}$可以从位置估计算法中计算的到，也可以用对极几何或齐次矩阵来进行部分位置估计。我们怎么在传统的IBVS中结合这个呢？

考虑控制平移自由度的特征向量$\mathbf{s}_t$与误差$\mathbf{e}_t$，我们可以讲交互矩阵分解为：
$$\begin{array}{lr}  
    \begin{aligned}
        \dot{\mathbf{s}_t} &= \mathbf{L_{s_t}v_c}\\  
        &=\begin{bmatrix} \mathbf{L}_v  & \mathbf{L}_\omega \end{bmatrix} \begin{bmatrix}
    \mathbf{v}_c  \\ \mathbf{\omega}_c \end{bmatrix}\\
        &=\mathbf{L}_v\mathbf{v}_c+\mathbf{L}_\omega\mathbf{\omega}_c
    \end{aligned}
  \end{array}$$

现在，设置$\dot{\mathbf{e}}_t=-\lambda\mathbf{e}_t$，我们可以解得平移运动的控制输入为：
$$\begin{array}{lr}  
    \begin{aligned}
        -\lambda\mathbf{e}_t&=\dot{\mathbf{e}}_t=\dot{\mathbf{s}}_t=\mathbf{L}_v\mathbf{v}_c+\mathbf{L}_\omega\mathbf{\omega}_c\\
        &\Rightarrow\mathbf{v}_c=-\mathbf{L}_v^+(\lambda\mathbf{e}_t+\mathbf{L}_\omega\mathbf{\omega}_c)
    \end{aligned} 
  \end{array}\text{ (8)}$$

我们可以将$(\lambda\mathbf{e}_t+\mathbf{L}_\omega\mathbf{\omega}_c)$看为修正过的误差项，它结合了原来的误差与因为旋转运动$\omega_c$引入的误差。平移控制输入$\mathbf{v}_c=-\mathbf{L}_v^+(\lambda\mathbf{e}_t+\mathbf{L}_\omega\mathbf{\omega}_c)$会让这个误差趋于0。

这种方法被称为2和1.5维（2 1/2-D）视觉伺服，即将结合后的IBVS与PBVS如此划分。更准确的来说，选择图像上点的坐标作为$\mathbf{s}_t$，和高度的对数，所以$\mathbf{L}_v$总是一个三角可逆矩阵。再准确一些，我们有$\mathbf{s}_t=(\mathbf{x},\log{\mathbf{Z}}),\mathbf{s}^\star_t=(\mathbf{x^\star},\log{\mathbf{Z^\star}}),\mathbf{e}^\star_t=(\mathbf{x}-\mathbf{x^\star},\log{\rho_{Z}})$，其中$\log{\rho_Z}=Z/Z^\star$，且：
$$\begin{array}{lr}  
    \begin{aligned}
        \mathbf{L}_v     &=\frac{1}{Z^\star\rho_Z}\begin{bmatrix} -1&0&x\\0&-1&y\\0&0&-1 \end{bmatrix}   \\  
        \mathbf{L}_\omega&=\begin{bmatrix} xy&-(1+x^2)&y\\1+y^2&-xy&-x\\-y&x&0 \end{bmatrix} 
    \end{aligned}
  \end{array}$$
注意到系数$\rho_Z$可以直接从之前介绍的部分位置估计算法中获得。

如果我么你回到视觉伺服的全局描述问题上，我们有$\mathbf{e}=(\mathbf{e}_t,\theta\mathbf{u})$，其中$\mathbf{L}_e$给出如下：
$$\mathbf{L_e}=
    \begin{bmatrix} 
       \mathbf{L}_v & \mathbf{L}_\omega\\
       0            & \mathbf{L}_{\theta\mathbf{u}}  
    \end{bmatrix}$$
通过应用(2)，我们立刻可以得到控制律(7)和(8)。

如果我们考虑教程I中选择的例子来比较不同控制器的效果，利用（8）得到的结果如图1所示。这里，$\mathbf{s}_t$中考虑的点是目标的重心$\mathbf{x}_g$。考虑到这个点的图像轨迹是期望的直线，并且相机速度分量都能很好的收敛，让这个控制器非常接近第一个PBVS方法。

对于稳定性，很显然这个控制器是全局渐进稳定的。更多的，正是因为$\mathbf{L_e}$的三角形式，可以使用齐次矩阵估计的方法，在有标定误差的情况下分析系统的稳定性。最后，这个控制结构中唯一不知道的常数参数是$\bf Z^\star$，可以用自适应方法在线估计。

还可以设计其它的混合结构。例如，在【16】中，$\mathbf{s}_t$的第三个分量不同，并且被设计的使得留在相机视野中的目标点相距尽可能远。【17】中提出了另一种方法。这种情况下，$\mathbf{s}=({}^{c^\star}\mathbf{t}_c,\mathbf{x}_g,\theta u_z)$，这种方式产生了以下三角块结构的交互形式：
$$\mathbf{L_e}=
    \begin{bmatrix}
        \mathbf{R} & \mathbf{0} \\
        \mathbf{L}_v' & \mathbf{L}_\omega'
    \end{bmatrix}$$
其中的$\mathbf{L}_v'$与$\mathbf{L}_\omega'$可以很简单的计算得到。在完美情况下，相机的轨迹是一条直线（因为${}^{c^\star}\mathbf{t}_c$是$\mathbf{s}$的一部分），且目标图像重心轨迹也是直线（因为$\mathbf{x}_g$也是$\mathbf{s}$的一部分）。相机的平移自由度致力于实现3-D的直线，而相机的旋转自由度则致力于实现2-D的直线和补偿因为平移运动产生的二维运动的$\mathbf{x}_g$。从图2中可以看到，实际情况中，这种方法尤其让人满意。

### 分段视觉伺服

### 图像点柱面坐标的IBVS