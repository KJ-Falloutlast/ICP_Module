# 简介（Introduction）
基于PCL，分别实现了point2point，point2plane，plane2plane三种形式的icp算法，输入原始轨迹，并完成原始轨迹到真实轨迹(groudTruth)的对齐。

# 编译与运行(Compile&Run)
mkdir build
cd build
cmake ..
make
./[target algorithm]
目录（Content）
CMakeLists.txt 目录CMake配置文件
icp.cpp 主文件
详解（Details）
在icp_pp.cpp文件中分别有四个函数：

PointToPlane 对应点到面形式ICP实现
PointToPoint 对应点到点形式ICP实现
icpPlaneToPlane 对应面到面形式ICP实现
icpPointToPlane 对应另一种点到面ICP实现
