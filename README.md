# MSCKF学习
# Open-VINS

### <font color='green'>DAY 1</font>

#### ov_core namespace

####        这一部分代码可以被不同的定位系统(msckf,优化)使用，是整个SLAM定位系统的基础部分。

####        主要采用了JPL四元数表示。包含了旋转矩阵，四元数以及向量之间的转换以及运算。引入了李群，李代数。

#### ov_core关键组成：

+ #### 三维特征初始化

+ #### KLT，特征点法

+ #### B样条曲线(SE(3))

+ #### 数据集真实值读取

+ #### 四元数和其他流形数学运算

+ #### 预积分

#### 知识点：

#### override

#### ov_type namespace

+ #### class

  + #### class JPLQuat：位姿四元数	

  左乘误差状态

  ```c++
#include "Type.h"
  #include "utils/quat_ops.h"
  ```
```
  
+ #### class Vec：三维向量
  
向量变量
  
  ```c++
  #include "Type.h"
```

  + #### class PoseJPL：位姿状态

  六自由度位姿

  ```c++
  #include <Eigen/Eigen>
  #include <memory>
  ```

  + #### class IMU：IMU状态
  
    位姿，速度，陀螺零偏，加表零偏
  
    ```c++
    #include "PoseJPL.h"
    #include "utils/quat_ops.h"
    ```
  
  + #### class Landmark：SLAM路标点

  ```c++
  #include "LandmarkRepresentation.h"
  #include "Vec.h"
  #include "utils/colors.h"
  #include "utils/print.h"
  ```

  + #### class Type：Type

  描述变量如何被表示或更新，每个变量以误差状态形式的表示，及其对应的协方差状态。

#### C++知识点：

+ shared_ptr

```c++
std::shared_ptr<PoseJPL>
```

每种智能指针都是以类模板的方式实现的，shared_ptr 也不例外。shared_ptr<T>（其中 T 表示指针指向的具体数据类型）的定义位于`<memory>`头文件，并位于 std 命名空间中，因此在使用该类型指针时，程序中应包含如下 2 行代码：

```c++
#include <memory>
using namespace std;
```

第 2 行代码并不是必须的，也可以不添加，则后续在使用 shared_ptr 智能指针时，就需要明确指明`std::`。

```c++
_v = std::shared_ptr<Vec>(new Vec(3))
```

成功构建了一个 shared_ptr 智能指针，其指向一块存有3这个 Vec 类型数据的堆内存空间。

+ override

```c++
void set_local_id(int new_id) override
```

C++ override从字面意思上，是覆盖的意思，实际上在C++中它是覆盖了一个方法并且对其**重写**，从而达到不同的作用。override是C++11中的一个继承控制关键字。override确保在派生类中声明的重载函数跟基类的虚函数有相同的声明。

基类中有普通虚函数：

```c++
virtual void set_local_id(int new_id) { _id = new_id; }
```

+ namespace

namespace是C++中的关键字，用来定义一个命名空间，不同头文件中也可以使用名称相同的命名空间，但前提是位于该命名空间中的成员必须保证互不相同。

+ 析构函数

创建对象时系统会自动调用构造函数进行初始化工作，同样，销毁对象时系统也会自动调用一个函数来进行清理工作，例如释放分配的内存、关闭打开的文件等，这个函数就是析构函数。

析构函数（Destructor）也是一种特殊的成员函数，没有返回值，不需要程序员显式调用（程序员也没法显式调用），而是在销毁对象时自动执行。构造函数的名字和类名相同，而析构函数的名字是在类名前面加一个`~`符号。

注意：析构函数没有参数，不能被重载，因此一个类只能有一个析构函数。如果用户没有定义，编译器会自动生成一个默认的析构函数。

+ 构造函数

构造函数必须是 public 属性的，否则创建对象时无法调用。



#### ov_msckf namespace

+ #### 基于FEJ的可扩展滑动窗口误差状态模型视觉-惯性估计器

+ #### 相机内参和外参在线标定

+ #### 相机和IMU之间的时间在线标定

+ #### 3D SLAM地标的不同表示方法



#### ov_init namespace 

#### 实现状态初始化

+ #### VI系统的静态和动态初始化




#### Open_vins项目特点：

+ #### 滑动窗口视觉惯性MSCKF系统

+ #### 模块化协方差类型系统

+ #### 五种不同的特征点表达方式

+ #### 相机与IMU标定

  + #### 相机与IMU外参数

  + #### 相机与IMU时间同步

  + #### 相机内参

+ #### 多种前端跟踪方式

  + #### 单目相机

  + #### 同步双目相机

  + #### KLT和特征点法

+ #### 静态和动态初始化

+ #### 零速修正

  

### <font color='green'>DAY 2</font>

1,ov_core前端学习

#### open_vins前端主要有三种方式：

+ #### 光流KLT

+ #### 描述子

+ #### Aruco跟踪

#### chinese noted

#### 2,ov_msckf启动OpenVINS

#### 启动函数：ros1_serial_msckf.cpp

#### 3,ov_eval评价标准

+ #### ATE（绝对轨迹误差）

####          估计轨迹和真实轨迹对齐后，每个时间戳对应的误差都被计算得

#### 到，之后会被平均。

#### 计算公式：$e_{A T E}=\frac{1}{N} \sum_{i=1}^N \sqrt{\frac{1}{K} \sum_{k=1}^K\left\|\mathbf{x}_{k, i} \boxminus \hat{\mathbf{x}}_{k, i}^{+}\right\|_2^2}$

#### 其中，N是算法运行的次数，K是位姿测量个数，$\hat{\mathbf{x}}^{+}$是估计轨迹。

+ #### RPE（相对位姿误差）

#### 可以计算许多小段范围内的导航误差，

+ #### RMSE（均方根误差）

+ #### NEES（规范估计误差平方）

+ #### 计算量评价

#### 4,Futrure Roadmap

+ #### 更先进的IMU积分方法，以及建立IMU内参模型

### <font color='green'>DAY 3</font>

#### 1,ov_msckf IMU预积分推导

#### 主要框架：

+ #### Propagator构造函数，填充陀螺仪和加速度计的噪声和随机游走，还有重力

+ #### feed_imu IMU回调，不断的读入IMU的时间戳，角速度和加速度

+ #### interpolate_data IMU数据线性化插值

+ #### IMU预积分公式RK4——predict_mean_rk4

+ #### IMU预积分公式——predict_mean_discrete

+ #### propagate_and_clone，IMU状态预测和误差状态转移矩阵计算

+ #### select_imu_readings 取t0到t1时刻内的IMU数据

+ #### predict_and_compute IMU预积分和协方差预测

### <font color='green'>DAY 4</font>

#### 1,ov_msckf 量测更新推导

+ #### 最小均方差估计

+ 

+ #### 条件概率分布

+ #### 线性量测更新

+ #### 更新公式与推导

### <font color='green'>DAY 5</font>

#### 1，可观性问题

+ #### 能观性矩阵的零空间维度反映了系统不可观的维度

+ #### VINS系统不可观维度等于4，通常为平移和绕重力方向的yaw

+ #### 能观不一致性的原因是EKF的转移和观测Jacobian矩阵的线性化点不同而造成的

+ #### EKF错误的可观现象导致yaw的协方差较小，从而系统整体更新出现累计误差