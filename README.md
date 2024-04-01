# 简介（Introduction）

基于PCL，分别实现了point2point，point2plane，plane2plane三种形式的icp算法，输入原始轨迹，并完成原始轨迹到真实轨迹(groudTruth)的对齐。

# 编译与运行(Compile&Run)

mkdir build cd build cmake .. make ./[target algorithm]

# 目录（Content）

CMakeLists.txt 目录CMake配置文件 icp_pp.cpp 主文件

# 详解（Details）

在icp_pp.cpp文件中分别有四个函数：

PointToPlane 对应点到面形式ICP实现 PointToPoint 对应点到点形式ICP实现 icpPlaneToPlane 对应面到面形式ICP实现 icpPointToPlane 对应另一种点到面ICP实现。

# 使用方法（Step）

1. 在yaml文件中导入tum格式的slam输出的轨迹:`slamTrajectoryFile`， 真值轨迹`gtTrajectoryFile`和输出轨迹`outputTrajectoryFile`
2. 设置icp方法`icp_pattern`：`icpPlaneToPlane icpPointToPlane PointToPlane PointToPoint NDT`
3. 设置初始参数`init_params`：初值越准，匹配的效果越好

note: 

* 一次成功不了，多试几种icp方法
* 初值要给定尽量和gt近似，不能相差太远，否则也会无法对齐轨迹

# 效果(Output)

note: 白色是原轨迹，蓝色是真值轨迹，绿色是ICP对齐后的轨迹

**对齐前：**

![image-20240401221022850](https://ubuntu-desktop-pics.oss-cn-beijing.aliyuncs.com/image-20240401221022850.png)

**icp对齐后：**

![img](https://ji83440guul.feishu.cn/space/api/box/stream/download/asynccode/?code=YTQ1MjQ2Nzk0OWQ4OWQ0OWNlM2Q3Y2IzMGE1ZTNkMjFfdldhSnlrQWRQalRxdHVZeHdQNlE5SFZVOUJ1RjFTOUFfVG9rZW46SGU1WWJKTFh3b1VnanJ4dTY2bGNSVUFRbmQ5XzE3MTE5ODA0MzU6MTcxMTk4NDAzNV9WNA)
