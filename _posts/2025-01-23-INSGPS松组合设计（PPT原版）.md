---
layout: default
title: INSGPS松组合设计（PPT原版）
category: 组合导航
nav_order: 14
---

# INS/GPS 松组合导航系统设计 —— 详细讲解版

> **来源**：课程总结PPT + 2020年考试真题
> **核心内容**：15维状态、五步法设计流程、有色噪声处理
> **适用**：期末考试、研究生入学考试

---

## 本章核心框架

利用 Kalman 滤波解决 INS/GPS 松组合导航问题，基本步骤为：

$$
\boxed{
\text{选状态} \rightarrow \text{写量测} \rightarrow \text{离散化} \rightarrow \text{滤波估计} \rightarrow \text{导航校正}
}
$$

---

## 一、选取状态量，列写状态方程

### 1.1 状态变量选取

采用**间接法**组合导航，选择 **15 维误差状态变量**为：

$$
\boxed{
X=
[
\phi_x,\phi_y,\phi_z,
\delta V_x,\delta V_y,\delta V_z,
\delta L,\delta \lambda,\delta h,
\varepsilon_x^b,\varepsilon_y^b,\varepsilon_z^b,
\nabla_x^b,\nabla_y^b,\nabla_z^b
]^T
}
$$

也可写成分块形式：

$$
\boxed{
X=
[
\phi^n,
\delta V^n,
\delta P,
\varepsilon^b,
\nabla^b
]^T
}
$$

其中各分量的含义：

$$
\phi^n=[\phi_x,\phi_y,\phi_z]^T
$$

为**平台失准角误差**（姿态误差）；

$$
\delta V^n=[\delta V_x,\delta V_y,\delta V_z]^T
$$

为**速度误差**；

$$
\delta P=[\delta L,\delta \lambda,\delta h]^T
$$

为**位置误差**（纬度、经度、高度）；

$$
\varepsilon^b=[\varepsilon_x^b,\varepsilon_y^b,\varepsilon_z^b]^T
$$

为**陀螺漂移**；

$$
\nabla^b=[\nabla_x^b,\nabla_y^b,\nabla_z^b]^T
$$

为**加速度计误差**。

因此状态向量可表示为：

$$
\boxed{
X_{15\times 1}
==============

[
\phi^n_{3\times1},
\delta V^n_{3\times1},
\delta P_{3\times1},
\varepsilon^b_{3\times1},
\nabla^b_{3\times1}
]^T
}
$$

### 1.2 陀螺漂移与加速度计误差建模

**PPT 原版假设**：陀螺漂移、加速度计误差均为**有色噪声**

通常采用**一阶马尔可夫过程**建模。

**陀螺漂移**：

$$
\boxed{
\dot{\varepsilon}_x^b
=====================

-\frac{1}{T_{gx}}\varepsilon_x^b+w_{gx}
}
$$

$$
\boxed{
\dot{\varepsilon}_y^b
=====================

-\frac{1}{T_{gy}}\varepsilon_y^b+w_{gy}
}
$$

$$
\boxed{
\dot{\varepsilon}_z^b
=====================

-\frac{1}{T_{gz}}\varepsilon_z^b+w_{gz}
}
$$

**加速度计误差**：

$$
\boxed{
\dot{\nabla}_x^b
================

-\frac{1}{T_{ax}}\nabla_x^b+w_{ax}
}
$$

$$
\boxed{
\dot{\nabla}_y^b
================

-\frac{1}{T_{ay}}\nabla_y^b+w_{ay}
}
$$

$$
\boxed{
\dot{\nabla}_z^b
================

-\frac{1}{T_{az}}\nabla_z^b+w_{az}
}
$$

噪声向量为：

$$
\boxed{
W=
[
w_{gx},w_{gy},w_{gz},
w_{ax},w_{ay},w_{az}
]^T
}
$$

### 1.3 组合系统状态方程

连续状态方程写为：

$$
\boxed{
\dot X(t)=F(t)X(t)+G(t)W(t)
}
$$

其中：

$$
\boxed{
X(t)_{15\times1}
}
$$

$$
\boxed{
F(t)_{15\times15}
}
$$

$$
\boxed{
G(t)_{15\times6}
}
$$

$$
\boxed{
W(t)_{6\times1}
}
$$

即：

$$
\boxed{
\dot X_{15\times1}
==================

F_{15\times15}X_{15\times1}
+
G_{15\times6}W_{6\times1}
}
$$

### 1.4 状态矩阵分块形式

按照 PPT 的写法：

$$
\boxed{
F(t)=
\begin{bmatrix}
F_{INS}(t)_{9\times9} & F_S(t)_{9\times6}\\
0_{6\times9} & F_M(t)_{6\times6}
\end{bmatrix}_{15\times15}
}
$$

其中：

- $F_{INS}(t)$ 表示姿态误差、速度误差、位置误差之间的耦合关系；
- $F_S(t)$ 表示陀螺漂移、加速度计误差对导航误差的影响；
- $F_M(t)$ 表示陀螺漂移和加速度计误差自身的马尔可夫动态。

其中：

$$
\boxed{
F_S(t)=
\begin{bmatrix}
-C_b^n & 0_{3\times3}\\
0_{3\times3} & C_b^n\\
0_{3\times3} & 0_{3\times3}
\end{bmatrix}_{9\times6}
}
$$

含义是：
- $\varepsilon^b$ 主要影响姿态误差方程；
- $\nabla^b$ 主要影响速度误差方程。

传感器误差动态矩阵为：

$$
\boxed{
F_M(t)=
\operatorname{diag}
\left[
-\frac{1}{T_{gx}},
-\frac{1}{T_{gy}},
-\frac{1}{T_{gz}},
-\frac{1}{T_{ax}},
-\frac{1}{T_{ay}},
-\frac{1}{T_{az}}
\right]
}
$$

噪声驱动矩阵为：

$$
\boxed{
G(t)=
\begin{bmatrix}
0_{9\times3} & 0_{9\times3}\\
I_3 & 0_{3\times3}\\
0_{3\times3} & I_3
\end{bmatrix}_{15\times6}
}
$$

---

## 二、选取量测量，列写量测方程

INS/GPS 松组合采用两个导航系统关于同一导航参数的差值作为量测量。

PPT 原版常用 **速度 + 位置组合**：

$$
\boxed{
Z(t)=
\begin{bmatrix}
Z_v(t)\\
Z_p(t)
\end{bmatrix}
=============

\begin{bmatrix}
V_{Ix}-V_{Gx}\\
V_{Iy}-V_{Gy}\\
V_{Iz}-V_{Gz}\\
L_I-L_G\\
\lambda_I-\lambda_G\\
h_I-h_G
\end{bmatrix}
}
$$

其中：
- $I$ 表示 INS 输出；
- $G$ 表示 GPS 输出。

因为采用间接法，所以：

$$
V_{Ix}-V_{Gx}\approx \delta V_x
$$

$$
V_{Iy}-V_{Gy}\approx \delta V_y
$$

$$
V_{Iz}-V_{Gz}\approx \delta V_z
$$

$$
L_I-L_G\approx \delta L
$$

$$
\lambda_I-\lambda_G\approx \delta \lambda
$$

$$
h_I-h_G\approx \delta h
$$

因此量测方程为：

$$
\boxed{
Z(t)=H(t)X(t)+V(t)
}
$$

其中：

$$
\boxed{
Z(t)_{6\times1}
}
$$

$$
\boxed{
H(t)_{6\times15}
}
$$

$$
\boxed{
X(t)_{15\times1}
}
$$

$$
\boxed{
V(t)_{6\times1}
}
$$

由于状态顺序为：

$$
X=
[
\phi^n,
\delta V^n,
\delta P,
\varepsilon^b,
\nabla^b
]^T
$$

所以量测矩阵为：

$$
\boxed{
H(t)=
\begin{bmatrix}
0_{6\times3} & I_{6\times6} & 0_{6\times6}
\end{bmatrix}_{6\times15}
}
$$

也就是量测量直接对应状态中的：

$$
[
\delta V^n,\delta P
]^T
$$

---

## 三、连续系统离散化

连续系统为：

$$
\boxed{
\dot X(t)=F(t)X(t)+G(t)W(t)
}
$$

$$
\boxed{
Z(t)=H(t)X(t)+V(t)
}
$$

实际计算机中要离散化为：

$$
\boxed{
X_k=\Phi_{k,k-1}X_{k-1}+\Gamma_{k-1}W_{k-1}
}
$$

$$
\boxed{
Z_k=H_kX_k+V_k
}
$$

其中状态转移矩阵为：

$$
\boxed{
\Phi_{k,k-1}
============

I+F(t_k)T+
\frac{[F(t_k)T]^2}{2!}
+\cdots
}
$$

工程或考试中常用一阶近似：

$$
\boxed{
\Phi_{k,k-1}\approx I+F(t_k)T
}
$$

噪声驱动矩阵离散化为：

$$
\boxed{
\Gamma_{k-1}
============

\int_{t_{k-1}}^{t_k}
\Phi(t_k,\tau)G(\tau)d\tau
}
$$

常用近似为：

$$
\boxed{
\Gamma_{k-1}\approx G(t_k)T
}
$$

---

## 四、利用离散 Kalman 滤波递推估计状态

状态一步预测：

$$
\boxed{
\hat X_{k/k-1}
==============

\Phi_{k,k-1}\hat X_{k-1}
}
$$

一步预测均方误差：

$$
\boxed{
P_{k/k-1}
=========

\Phi_{k,k-1}P_{k-1}\Phi_{k,k-1}^T
+
\Gamma_{k-1}Q_{k-1}\Gamma_{k-1}^T
}
$$

滤波增益：

$$
\boxed{
K_k=
P_{k/k-1}H_k^T
\left(
H_kP_{k/k-1}H_k^T+R_k
\right)^{-1}
}
$$

状态估值更新：

$$
\boxed{
\hat X_k
========

\hat X_{k/k-1}
+
K_k
\left[
Z_k-H_k\hat X_{k/k-1}
\right]
}
$$

估计均方误差更新：

$$
\boxed{
P_k=(I-K_kH_k)P_{k/k-1}
}
$$

或者写成 Joseph 稳定形式：

$$
\boxed{
P_k=
(I-K_kH_k)P_{k/k-1}(I-K_kH_k)^T
+
K_kR_kK_k^T
}
$$

---

## 五、利用估计出的状态校正导航参数

滤波得到：

$$
\boxed{
\hat X_k=
[
\hat\phi^n,
\delta \hat V^n,
\delta \hat P,
\hat\varepsilon^b,
\hat\nabla^b
]^T
}
$$

其中：

$$
\delta \hat P=
[
\delta \hat L,
\delta \hat \lambda,
\delta \hat h
]^T
$$

$$
\delta \hat V^n=
[
\delta \hat V_x,
\delta \hat V_y,
\delta \hat V_z
]^T
$$

采用输出校正时：

$$
\boxed{
L=L_I-\delta \hat L
}
$$

$$
\boxed{
\lambda=\lambda_I-\delta \hat \lambda
}
$$

$$
\boxed{
h=h_I-\delta \hat h
}
$$

$$
\boxed{
V_x=V_{Ix}-\delta \hat V_x
}
$$

$$
\boxed{
V_y=V_{Iy}-\delta \hat V_y
}
$$

$$
\boxed{
V_z=V_{Iz}-\delta \hat V_z
}
$$

同时也可以利用 $\hat\phi^n$、$\hat\varepsilon^b$、$\hat\nabla^b$ 对 SINS 姿态、陀螺漂移和加速度计误差进行**反馈校正**。

---

## 应试记忆口诀

$$
\boxed{
\text{十五维状态：三姿态、三速度、三位置、三陀螺、三加表；}
}
$$

$$
\boxed{
\text{量测取差值，写成 } Z=HX+V\text{；}
}
$$

$$
\boxed{
\text{连续先离散，再用 Kalman 递推，最后校正 INS 输出。}
}
$$

---

**祝考试顺利！** 🎯
