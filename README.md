# MSCKF学习
# Open-VINS

### <font color='green'>DAY 1</font>

#### ov_core namespace

####        这一部分代码可以被不同的定位系统(msckf,优化)使用，是整个SLAM定位系统的基础部分。

####        主要采用了JPL四元数表示。包含了旋转矩阵，四元数以及向量之间的转换以及运算。引入了李群，李代数。

#### 知识点：

#### override

#### ov_type namespace

+ #### class

  + #### class JPLQuat：位姿四元数		

  + #### class Vec：三维向量

  + #### class PoseJPL：位姿状态

  + #### class IMU：IMU状态

  + #### class Landmark：SLAM路标点

  + #### class Type：Type

    

#### ov_msckf namespace

+ #### 基于FEJ的可扩展滑动窗口误差状态模型视觉-惯性估计器

+ #### 相机内参和外参在线标定

+ #### 相机和IMU之间的时间在线标定

+ #### 3D SLAM地标的不同表示方法



#### ov_init namespace 

#### 实现状态初始化

+ #### VI系统的静态和动态初始化

  