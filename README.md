# 轴孔装配仿真环境

[toc]

## 环境&运行

mujoco，dm_control（在mujoco200的版本下开发，210应该也可以使用），py-opencv（用于显示仿真中的相机画面）。

运行main.py即可经由Torque.torque_calc函数控制末端位姿，能够通过cv2显示相机图像，通过time_step内的observation读取关节角度等信息。

运行insert_peg.py即可运行viewer观察模型情况，目前大概认为策略不能和viewer一起运行，viewer内的策略是时序的而不是状态反馈的，所以只能用来3D观察，也能看到site之类的。

## 建模

### 坐标系说明

世界坐标系原点在地板中点，x轴向右，y轴向内，z轴向上（建模坐标系下）。关节角度q读取，均以z轴逆时针取角度正，会持续增长，不会规范到$-\pi,\pi$。

机械臂控制时坐标系为x轴向右，y轴向上。算法中末端姿态由$q_1+q_2+q_3$计算。得出的末端姿态会以向上为0度，insert时角度会在$-\pi,\pi$之间跳跃，不太方便。故在计算目标姿态误差（PID）时取末端姿态实际上为$q_1+q_2+q_3+\pi/2$，变成向右为0度，与数学中定义一致。

摩擦系数$\mu=0.7$，仿真步长0.001s，控制步长0.01s

未注明的尺寸均为SI国际单位制

### 几何尺寸

#### 机械臂

arm_root，蓝色圆柱，机械臂基座。中心位于世界坐标系[0.4 0 0]，半径0.024

arm_root关节，damping=0，控制range$[-12,12]$，无角度限制。

upper_arm，第一个臂，长度0.18，半径0.02，初始方向沿着z轴

arm_shoulder关节，damping=0，控制range$[-12,12]$，角度限制$[-170\degree,170\degree]$

middle_arm，第二个臂，长度0.15，半径0.015，初始方向沿z轴

arm_wrist关节，damping=0，控制range$[-12,12]$，角度限制$[-160\degree，160\degree]$

blade，插入轴，相当于第三个臂，长度0.113，半径0.005，初始方向沿z轴

guard，轴上的挡板，位于blade上0.043，与槽高度相当

site:peg，位于blade上0.063

site:peg_tip，位于blade末端，可以进行力传感（peg_tip_touch），只能反馈力的大小，不能反映方向

#### 插槽

slot，三个box包围形成的槽，槽按数值上宽度是0.1，属于过渡配合（恰好），高度能够被机械臂上的guard件限制

site:slot_end，在槽开口处，可以用来计算距离

#### 虚影（插入目标位姿）

body:target_peg，和body:slot保持一致即可让虚影位于插槽内。

site:target_peg，与site:peg对应点，可以用于衡量插入程度、距离奖励。

site:target_peg_tip，与site:peg_tip对应，可以计算插入距离。

### 质量&惯量

所有柱子采用圆柱体，质量均匀分布，每一节臂的质量均为0.1kg。根据物理学惯量知识可知，绕圆柱高上中点处旋转轴旋转的转动惯量为：
$$
I_x=I_y=\frac{m}{12}(3R^2+L^2)
$$

## 控制

使用operation space control控制，在笛卡尔空间下给出末端的期望加速度，这里为$a_x,a_y,\beta_z$三自由度。加速度信号由位置和速度信号产生，即：
$$
\begin{bmatrix}
a_x\\a_y\\\beta_z
\end{bmatrix}=
\bold{K_p}
\begin{bmatrix}
\Delta x\\\Delta y\\\Delta deg
\end{bmatrix}
+
\bold{K_d}
\begin{bmatrix}
\Delta \dot{x}\\\Delta \dot{y}\\\Delta \dot{deg}
\end{bmatrix}
$$
其中$\bold{K_p}$和$\bold{K_d}$增益矩阵一般是解耦对角阵。通过Torque.torque_calc(q,q_dot,goal)函数调用，参数均需要列向量。这部分后续应该还需要依据轨迹规划情况继续调整，目前只是调了个欠阻尼，收敛速度并不够。
