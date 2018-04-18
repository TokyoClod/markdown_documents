<script type="text/javascript" src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=default"></script>
# Visual Servo Control--Part I: Basic Approaches

## 视觉伺服的基本组成

基于视觉的控制结构的目的是最小化如下定义的误差：
$$\mathbf{e(t)} = s(\mathbf{m}(t),a)-s^{\star} \text {，(1)}$$
该公式非常通用，它包含了很多方法，正如我们在下文即将看到的。(1)中参数定义如下。向量$\mathbf{m}(t)$是一系列图像量测（例如，感兴趣点的图像坐标，或者是物体中心点的坐标）。这些图像量测用来计算一个k维的视觉特征$s(\mathbf{m}(t),a)$，其中$a$是代表了系统隐含的额外信息的一系列参数（例如，粗糙的相机固有参数，或是物体的3-D模型）。向量$s^{\star}$包含了这些特征的期望值。  

在教程的第一部分（本文），我们考虑目标全局位置固定且不运动的情况，即$s^{\star}$是一个常数，且$s$的变化仅与相机运动有关。另外，我们考虑控制一个6自由度相机的运动的情况。例如，一个相机被固定在六自由度机械臂的末端执行器上。在教程的第二部分，我们会考虑更加一般的情况。  

视觉伺服的结构的区别主要在于如何设计$s$。在本文中，我们将看到两种十分不同的方法。首先，我们介绍基于图像的视觉伺服（IBVS），其中，$s$包含了一系列可以直接从图像数据中获得的信息作为特征。然后，我们介绍基于位置的视觉伺服（PBVS），其中，$s$包含了一系列必须从图像测量中估计出来的3-D信息。  

一旦$s$选定好了，控制结构的设计将会非常简单。也许最直接的方法就是设计一个速度控制器。为了达到这个目的，我们需要知道$s$在时域中的导数与相机速度的关系。用$\mathbf{v}_c=(v_c,\mathbf{\omega}_c)$来表示相机在空间中的速度，$v_c$表示相机坐标系原点的瞬时线速度，$\mathbf{\omega}_c$表示相机坐标系原点的瞬时角速度。$\dot{s}$与$v_c$的关系给出如下：
$$\mathbf{\dot{s}}=\mathbf{L_s}\mathbf{v}_c \text{ (2)}$$
其中，$\mathbf{L_s}\in\mathbb{R}^{k\times6}$被称为关于$s$的交互作用矩阵。在某些文献中也被称为特征雅克比。  

利用（1）和（2），我们可以立刻获得相机速度与误差在时域的导数：
$$\mathbf{\dot{e}}=\mathbf{L_e}\mathbf{v}_c\text{ (3)}$$
其中，$\mathbf{L_e}=\mathbf{L_s}$。考虑讲$\mathbf{v}_c$作为机器人控制器的输入，理论上如果我们想要保证误差能够以指数速度收敛（例如，$\mathbf{\dot{e}}=-\lambda\mathbf{e}$)，我们可以得到：
$$\mathbf{v}_c = -\lambda\mathbf{L^{+}_e}\mathbf{e} \text{ (4)}$$
其中$\mathbf{L^{+}_e}\in\mathbb{R}^{6\times k}$是$\mathbf{L_e}$的广义逆矩阵，也就是当$\mathbf{L_e}$满秩时，$\mathbf{L^{+}_e}=({\mathbf{L_e}}^T \mathbf{L_e})^{-1} \mathbf{L_e}^T$。这样子的选择可以保证$\left\| \mathbf{\dot{e}}-\lambda\mathbf{L_e}\mathbf{L^{+}_e}\mathbf{e} \right\|$与$\left\|\mathbf{v}_c\right\|$最小。当$k=6$时，如果det $\mathbf{L_e}\not=0 $，那$\mathbf{L_e}$就是可逆的，控制律$\mathbf{v}_c = -\lambda\mathbf{L^{-1}_e}\mathbf{e}$  

在实际的视觉伺服系统中，无论是$\mathbf{L_e}$还是$\mathbf{L^{+}_e}$都不可能精确地获得。因此，需要实现对这两个矩阵中的至少一个进行估计。在本文中，我们把交互矩阵伪逆的近似、交互矩阵近似的伪逆，都用$\mathbf{\hat{L^{+}_e}}$表示。利用这个标记，实际上控制律变为：
$$\mathbf{v}_c = -\lambda\mathbf{\hat{L^{+}_e}}\mathbf{e} \text{ (5)}$$

这就是大多是视觉伺服控制器实现的基础设计。剩下的只需要填充细节：如何选取$s$？$L_s$的表达式是什么？我们要如何估计$\mathbf{\hat{L^{+}_e}}$？闭环系统的表现如何？接下来本文将回答这几个问题。

## 经典的基于图像的视觉伺服

传统的IBVS，利用一系列图像平面中的点来定义$s$(其它的选择也可以，我们将在教程第二部分介绍)。图像量测$\bf{m}$通常使用这些点的像素平面坐标，$s(\mathbf{m}(t),a)$中的$a$就是从像素坐标表示的图像量测到特征转换中所需的相机固有参数。

## 交互矩阵

将$x$作为特征的话，上述等式可以写为：
$$\mathbf{\dot{x}}=\mathbf{L_x} \mathbf{v}_c \text{ (10)}$$
关于$x$的交互矩阵$\mathbf{L_x}$为：
$$\mathbf{L_x} = \begin{bmatrix}
-1/Z  & 0     & x/Z  & xy     & -(1+x^2) & y      \\
0     & -1/Z  & y/Z  & 1+y^2  & -xy      & -x
\end{bmatrix} \text{ (11)}$$

在矩阵$\mathbf{L_x}$中，$Z$表示点相对于相机坐标系的深度。因此，任何使用这种形式的交互矩阵都必须估计$Z$的值。相似的，相机内参也参与了$x,y$的计算。因此，$\mathbf{L_x}$不能直接在（4）中使用，而且必须使用$\mathbf{\hat{L_x}}$这个估计值。

为了控制6个自由度，需要至少三个特征点(例如，我们要求$k\geq6$)。如果我们使用特征向量$\bf{x=(x_1,x_2,x_3)}$，我们仅仅需要把这些点的交互矩阵进行堆叠得到：
$$\bf{L_x} = \begin{bmatrix}
L_{x_1}  \\
L_{x_2}  \\
L_{x_3} \end{bmatrix}$$
这种情况下，会存在某些情况使得$\mathbf{L_x}$奇异。更多的，相机存在4个不同的位置，使得$e=0$，例如，没有办法区分这四个局部极小值。还有一些其他原因，使得通常需要使用多余三个点。

## 交互矩阵的估计

有好几种方式来构建控制律中的$\mathbf{\hat{L^{+}_e}}$。其中一种常用的结构就是很直接地选取$\mathbf{\hat{L^{+}_e}} = \mathbf{L^{+}_e}$，如果$\mathbf{L_e}=\mathbf{L_x}$已知的话；也就是说，当每个点的深度$Z$已知。实际上，这些参数必须在控制器的每次迭代过程中估计。本文中使用的经典的位置估计算法将会在下个章节讲述。另一种方法是选取$\mathbf{\hat{L^{+}_e}} = \mathbf{L^{+}_{e^\star}}$，其中，$\mathbf{L^{+}_{e^\star}}$是$\mathbf{L_e}$在目标位置$e=e^\star=0$时的值。这种情况下，$\mathbf{\hat{L^{+}_e}}$是一个常数，并且只需要为每个点设置预期的高度，也就是说视觉伺服的过程中没有变化的3-D参数需要估计。最后，还可以选取$\mathbf{\hat{L^{+}_e}} = (\mathbf{L_e}+\mathbf{L^{+}_{e^\star}})/2$，这种方法在最近的文献中被提出。同样的，需要知道每个点实时的深度信息。

## IBVS的几何解释

很容易为上述的控制结构提供集合解释。图5对应了平面上4个点围绕光轴的纯旋转运动，懂初始状态（蓝色）到目标状态（红色）。

正如之前所示，控制结构中使用$\mathbf{L^{+}_e}$来试图保证误差$e$的指数收敛。这意味着，当当图像平面内点的$x,y$坐标来补偿这个误差时，图像平面内点的轨迹是一条从初始位置到目标位置的直线，如果可能的话。图像上的运动轨迹用绿色画出。为了实现这种运动，可以很容易推断出相机是在做围绕光轴的旋转运动，但是还混合了沿着光轴平移撤退运动。出现这种不期望的运动是因为选择的特征的问题，和交互矩阵第三、第六列的耦合。如果初始位置和目标位置的旋转角度很大，这种现象将被放大，导致在某种特殊情况下产生一个$\pi$ rad的旋转且最终并没有给控制器引入旋转运动。另一方面，如果旋转角度很小，那这种现象就几乎会完全消失。总的来说，这种（ibvs控制器）现象在一定程度上是可以让人满意的（当误差不大的时候），但当误差很大的时候就很糟糕了。我们可以在文章的最后部分看到，这些结果与IBVS的局部渐进稳定性是一致的。

如果我们在控制结构中使用$\mathbf{L^{+}_{e^\star}}$，在图像平面内的运动如图5的蓝色线表示。事实上，如果我们考虑和之前相同的控制结构，但是改成从$s^\star$到$s$，那我们可以得到：
$$\mathbf{v}_c = -\lambda\mathbf{L^{+}_{e^\star}}\mathbf{(s^\star-s)}$$
同样产生了一条从红色点到蓝色点的直线轨迹，用棕色来表示。返回到我们的问题上，该控制结构计算了完全相反的相机速度：
$$\mathbf{v}_c = -\lambda\mathbf{L^{+}_{e^\star}}\mathbf{-(s^\star)}$$

因此在图像中的运动就是用红色画出来的红点。转化到蓝色的点，相机速度产生了蓝色的图像运动，再次与围绕光轴的旋转运动对应，并且还有不期望的沿着光轴的前进的平移运动。在初始误差的大小问题上，与我们之前的分析相同。我们可以这么说，只要误差明显减小，两种控制结构都将趋近于同一种结构（因为当$\bf e=e^\star$时，$\bf L_e=L_e^{\star}$，当误差趋近于0时，都能表现的很好，相机运动只需要补偿一个很小的旋转角度。

如果我们使用$\mathbf{\hat{L^{+}_e}} = (\mathbf{L_e}+\mathbf{L^{+}_{e^\star}})/2$，从直觉上我们可以很清晰的看到，$\mathbf{L_e}$与$\mathbf{L^{+}_{e^\star}}$的平均值在图像中的运动用黑色画出的。除了$\pi$ rad附近的旋转运动，相机的运动就几乎是一个纯粹的绕着光轴的旋转运动了，没有任何我们不需要的平移运动。

## 基于位置的视觉伺服（PBVS）

基于位置的控制结构使用了相机相对于某些参考系的位置来定义$\bf s$。从一系列图像中的量测中来计算出位置需要已知相机的内参以及我们关注的目标物体的3-D模型。这种经典的计算机视觉问题被称为3-D定位问题。

考虑三个坐标系：当前相机坐标系$\mathcal{F_c}$，目标相机坐标系$\mathcal{F_{c^\star}}$，固定在目标上的参考系$\mathcal{F_o}$。我们这里使用标准的记号，使用前置的上标来表示一系列点定义的坐标系。因此，坐标向量${}^c\mathbf{t_o}$与${}^{c^\star}\mathbf{t_o}$来表示物体坐标系相对于当前相机坐标系与目标相机坐标系的原点坐标。更多的，用$\mathbf{R}={}^{c^\star}\mathbf{R}_c$来表示当前相机坐标系到目标相机坐标系的旋转矩阵。

我们可以将$\bf s$定义为$(\mathbf{t},\;\theta \mathbf{u})$，其中，$\mathbf{t}$是旋转向量,$\theta \mathbf{u}$给出了旋转的角（轴）参数化。我们现在讨论$\mathbf{t}$的两种选取方式及对应的控制律。

如果$\mathbf{t}$是相对于目标物体坐标系$\mathcal{F_o}$定义的，我们可以得到$\mathbf{s}=({}^c\mathbf{t}_o,\theta\mathbf{u})$，$\mathbf{s^\star}=({}^{c^\star}\mathbf{t}_o,0)$，并且$\mathbf{e}=({}^c\mathbf{t}_o-{}^{c^\star}\mathbf{t}_o,\theta \mathbf{u})$。这种情况下，关于$\mathbf{e}$的交互矩阵为：
$$\mathbf{L_e} = 
    \begin{bmatrix}
    -\mathbf{I}_3  & [{}^c\mathbf{t_o}]_{\times}      \\
    0              & \mathbf{L}_{\theta \mathbf{u}}
    \end{bmatrix} \text{ (13)}$$

其中，$\mathbf{L}_{\theta \mathbf{u}}$为：
$$\mathbf{L}_{\theta \mathbf{u}} = \mathbf{I}_3-\frac{\theta}{2}[\mathbf{u}]_{\times}+(1-\frac{sinc\,\theta}{sinc^2\,\frac{\theta}{2}})[\mathbf{u}]_{\times}^2 \text{ (14)}$$
其中$sinc\,x$定义为$xsinc\,x = \sin x$且$sinc \,0=1$。

由本文一开始的推到可以得到控制结构为：
$$\mathbf{v}_c = -\lambda\hat{\mathbf{L^{-1}_{e}}}\mathbf{e}$$
因为$s$的维数$k$为6，正好是相机自由度的维数。通过设置：
$$\hat{\mathbf{L^{-1}_{e}}}=
    \begin{bmatrix}
    -\mathbf{I}_3  & [{}^c\mathbf{t_o}]_{\times}\mathbf{L}_{\theta \mathbf{u}}^{-1} \\
    0     & \mathbf{L}_{\theta \mathbf{u}}^{-1}
    \end{bmatrix} \text{ (15)}$$

简化后我们可以得到：
$$\left \{ 
    \begin{array}{lr}  
        \mathbf{v}_c=-\lambda(({}^{c^\star}\mathbf{t}_o-{}^c\mathbf{t}_o)+[{}^c\mathbf{t_o}]_{\times}\theta \mathbf{u})
         \\  
        \mathbf{\omega}_c=-\lambda \theta \mathbf{u}
    \end{array}
    \right. \text{ (16)}$$

其中$\mathbf{L}_{\theta \mathbf{u}}$满足$\mathbf{L}_{\theta \mathbf{u}}^{-1}\theta \mathbf{u}=\theta \mathbf{u}$

这样的PBVS结构能够以指数收敛的旋转速度跟踪测地线，并且$s$中的平移参数也能以相同的速度减小。这就见识了图7中的相机速度分量能够很好的指数收敛。更多的，图像中物体坐标系原点的轨迹就是一条直线（这里四个点的中心被选为原点）。但是，相机的运动轨迹不是直线。

另一种PBVS结构可以设计为$\mathbf{s}=({}^{c^\star}\mathbf{t}_c,\theta \mathbf{u})$。这种情况下，我们有$\bf s^\star=0,e=s$，并且
$$\mathbf{L_e} = 
    \begin{bmatrix}
    \mathbf{R}  & 0      \\
    0           & \mathbf{L}_{\theta \mathbf{u}}
    \end{bmatrix} \text{ (17)}$$

注意到平移和旋转运动的耦合，我们可以使用一个简单的控制结构：
$$\left \{ 
    \begin{array}{lr}  
        \mathbf{v}_c=-\lambda\mathbf{R^T}{}^{c^\star}\mathbf{t}_o
         \\  
        \mathbf{\omega}_c=-\lambda \theta \mathbf{u}
    \end{array}
    \right. \text{ (16)}$$

这种情况下，如图8所示，如果（18）中的位置参数可以被完美的估计，那么相机的轨迹将是一条纯粹的直线，但是图像的轨迹就不如之前的好了。在某些特殊的情况下某些点甚至会离开相机的视野范围。

## 稳定性分析

本节中，我们讨论关于控制器稳定性的基础。我们使用李雅普诺夫法来评估闭环控制器的稳定性。尤其的，考虑用平方差范数$\mathcal{L}=1/2\left\|\mathbf{e}(t) \right\|^2$定义的备选李雅普诺夫函数，它的导数为：
$$\begin{array}{lr}  
    \begin{aligned}
        \dot{\mathcal{L}} &= \mathbf{e}^T\dot{\mathbf{e}} \\  
        &=-\lambda\mathbf{e}^T\mathbf{L_e}\mathbf{\hat{L^{+}_e}}\dot{\mathbf{e}}
    \end{aligned}
  \end{array}$$
  
  当以下充分条件能满足时：
  $$\mathbf{L_e}\mathbf{\hat{L^{+}_e}}>0  \text{ (19)}$$
  系统能够全局渐进稳定。

如果特征点的数量等于相机自由度的个数（例如，$k=6$），并且选取的控制律满足$\mathbf{L_e}$与$\mathbf{\hat{L^{+}_e}}$都满秩为6，那么，只要$\mathbf{\hat{L^{+}_e}}$中的近似不要太严重，那么条件（19）就能满足。

## IBVS的稳定性分析

正如之前讨论的，大多数的IBVS方法都有$k>6$，因此，条件（19）几乎从不会满足因为$\mathbf{L_e}\mathbf{\hat{L^{+}_e}}\in\mathbb{R}^{k\times k}$最多秩只能为6。因此，$\mathbf{L_e}\mathbf{\hat{L^{+}_e}}$有非平凡的化零子空间。这种情况下，例如$\mathbf{e}\in\mathbf{ker}\mathbf{\hat{L^{+}_e}}$这样的情况对应的会是局部最小值。图9描述了到这样局部最小值的情况。从图9(d)可以看出，$\bf e$中的每个分量都以指数速度很好地收敛，在图像中的轨迹也是一条直线。但是此时的误差不是完全等于0，从图9(c)中可以看出此时到达了一个局部相距目标位置很远的极小值点。所以IBVS只有局部渐进稳定性。

为了研究$(\mathbf{k}>6)$时的局部渐渐稳定性，我们首先定义一个新的误差$\bf e'=\mathbf{\hat{L^{+}_e}}e$，该误差的导数为：
$$\begin{array}{lr}  
    \begin{aligned}
        \dot{\mathbf{e}}' &= \mathbf{\hat{L^{+}_e}}\dot{\mathbf{e}} +\dot{ \mathbf{\hat{L^{+}_e}}}\mathbf{e}\\  
        &=(\mathbf{\hat{L^{+}_e}}\mathbf{L_e}+\mathbf{O})\mathbf{v}_c
    \end{aligned}
  \end{array}$$
其中当$\mathbf{e}=0$时，$\mathbf{O}\in\mathbb{R}^{6\times6}$。利用（5）中的控制结构，我们得到：
$$\dot{\mathbf{e}}'=-\lambda(\mathbf{\hat{L^{+}_e}}\mathbf{L_e}+\mathbf{O})\mathbf{e}'$$
如果
$$\mathbf{\hat{L^{+}_e}}\mathbf{L_e}>0 \text{  (20)}$$
那么上面的等式将在$\mathbf{e}=\mathbf{e^\star}=0$的邻域内渐进稳定。
其中，$\mathbf{\hat{L^{+}_e}}\mathbf{L_e}\in\mathbb{R}^{6\times 6}$。事实上，如果我们只对局部渐进稳定感兴趣的话，只需要考虑线性化后的系统$\dot{\mathbf{e}}'=-\lambda\mathbf{\hat{L^{+}_e}}\mathbf{L_e}\mathbf{e}'$

此外，如果选取的特征和控制接受的设计使得$\mathbf{L_e}$与$\mathbf{\hat{L^{+}_e}}$都满秩为6，那条件（20）必定会被满足。

最后，我们必须指出，不存在任何$\bf e\not=e^\star$的情况使得$\bf e\in ker\,\mathbf{\hat{L^{+}_e}}$同时在$\bf e^\star$的邻域内和对应位置$\bf p^\star$的邻域内。对应$\mathbf{v}_c=0$与$\mathbf{e}\not=\mathbf{e^\star}$的情况为局部最小值。如果位置$p$存在，那么最好将它限制在$\bf p^\star$周围的邻域内，这样可以存在一个从$\bf p$到$\bf p^\star$的速度$\bf v$。这个相机速度会实现误差$\bf \dot{e}=L_ev$的变化。但是，这样的变化不属于$\mathbf{ker}\,\mathbf{\hat{L^{+}_e}}$，因为$\mathbf{\hat{L^{+}_e}}\mathbf{L_e}>0$。因此，当且仅当$\mathbf{\dot{e}}=0$的时候，例如$\mathbf{e}=\mathbf{e^\star}$时，才能在$\bf p^\star$的邻域内得到$\mathbf{v}_c=0$。

尽管$k>6$的时候可以得到渐进稳定性，但是我们无法确保全局渐进稳定。例如，如图9所示。可能会存在局部最小值的点使得$\bf e\in ker\,\mathbf{\hat{L^{+}_e}}$，但不是在目标邻域内。如何选取领域大小来确保系统的稳定性和收敛还是一个开放的问题，尽管在实际使用中这样的邻域可以大的惊人。