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

  + #### class Vec：三维向量

  + #### class PoseJPL：位姿状态

  + #### class IMU：IMU状态

    位姿，速度，陀螺零偏，加表零偏

  + #### class Landmark：SLAM路标点

  + #### class Type：Type
  

#### 知识点：

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

  