#include <pcl/io/pcd_io.h>          //I/O操作头文件
#include <pcl/point_types.h>        //点类型定义头文件
#include <pcl/registration/icp.h>   //ICP配准类相关头文件
#include <sstream> 
#include <iostream>  
#include <fstream>  
#include <vector>  
#include <Eigen/Dense>  
#include <Eigen/Geometry>  
#include <yaml-cpp/yaml.h>
#include <boost/thread/thread.hpp> 
#include <boost/thread/mutex.hpp> 
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/src/Core/Matrix.h>
#include <string>
#include <map>

/** @brief simple icp code
 * @author JamesHu
 * @date \2024.3.30
 * @github  https://github.com/KJ-Falloutlast/ICP_Module.git
 */
using ICPFunPtr = Eigen::Matrix4f (*)(pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Matrix4f&);

struct TumTrajectoryPoint 
{  
    double timestamp;  
    Eigen::Vector3d position;  
    Eigen::Quaternionf rotation;  
};  

//  读取TUM格式的轨迹文件  
std::vector<TumTrajectoryPoint> readTumTrajectory(const std::string& filename) {  
    std::vector<TumTrajectoryPoint> points;  
    std::ifstream file(filename);  
    if (!file.is_open()) {  
        std::cerr << "Failed to open file: " << filename << std::endl;  
        return points;  
    }  
  
    std::string line;  
    while (std::getline(file, line)) {  
        if (line.empty() || line[0] == '#') continue; // 忽略空行和注释  
  
        std::istringstream iss(line);  
        double timestamp;  
        Eigen::Vector3d position;  
        Eigen::Quaternionf rotation;  
        TumTrajectoryPoint trajPoint;
        // 解析每一行的数据，这里假设你已经知道数据的格式  
        iss >> timestamp >> position(0) >> position(1) >> position(2) >> rotation.w() >> rotation.x() >> rotation.y() >> rotation.z();  
        trajPoint.timestamp = timestamp;
        trajPoint.position = position;
        trajPoint.rotation = rotation;
        points.emplace_back(trajPoint);  
    }  
  
    file.close();  
    return points;  
}  

// 将轨迹点写入TUM格式的文件  
void writeTUMTrajectory(const std::string& filename, const std::vector<TumTrajectoryPoint>& trajectory) {  
    std::ofstream file(filename);  
    if (!file.is_open()) {  
        std::cerr << "Failed to open file for writing: " << filename << std::endl;  
        return;  
    }  
  
    for (const auto& point : trajectory) {  
        // 输出时间戳、位置和旋转（四元数）
        // 获取里程计的时间戳
        double timestamp = point.timestamp;

        // 获取traj的位置信息
        double x = point.position.x();
        double y = point.position.y();
        double z = point.position.z();

        // 获取traj的姿态信息（四元数表示）
        double qx = point.rotation.x();
        double qy = point.rotation.y();
        double qz = point.rotation.z();
        double qw = point.rotation.w();
        file << std::fixed << std::setprecision(9) << timestamp << " "
            << x << " " << y << " " << z << " "
            << qx << " " << qy << " " << qz << " " << qw << std::endl;
    }  
    file.close();  
}  


Eigen::Matrix4f PointToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                             Eigen::Matrix4f& guess)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud1, *src);
  pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud2, *tgt);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
  norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
  norm_est.setKSearch(10);
  norm_est.setInputCloud(tgt);
  norm_est.compute(*tgt);

  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
  typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;
  boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
  icp.setTransformationEstimation(point_to_plane);  // key
  /*
  typedef pcl::registration::CorrespondenceEstimationNormalShooting<PointNormal,
  PointNormal, PointNormal> CorrEstNS;
  CorrEstNS::Ptr corrEst(new CorrEstNS);
  icp.setCorrespondenceEstimation(corrEst);
  */
  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  // icp.setRANSACOutlierRejectionThreshold(ransac_par);
  icp.setRANSACIterations(20);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-3);
  pcl::PointCloud<pcl::PointNormal> output;
  icp.align(output);  // align 的另一个重载可以设置一个初始矩阵guess
  std::cout << "score: " << icp.getFitnessScore() << endl;
  return icp.getFinalTransformation();
}

Eigen::Matrix4f icpPlaneToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI>::Ptr tar, Eigen::Matrix4f& guess)
{
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;  // GICP 泛化的ICP，或者叫Plane to Plane
                                                                              // ICP
  icp.setTransformationEpsilon(0.0000000001);
  icp.setMaxCorrespondenceDistance(2.0);
  icp.setMaximumIterations(50);
  icp.setRANSACIterations(20);
  icp.setInputTarget(tar);
  icp.setInputSource(src);
  pcl::PointCloud<pcl::PointXYZI> unused_result;
  icp.align(unused_result, guess);
  std::cout << "score: " << icp.getFitnessScore();
  return icp.getFinalTransformation();
}

void addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
               pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZI>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(15);
  normalEstimator.compute(*normals);

  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}

Eigen::Matrix4f icpPointToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI>::Ptr tar, Eigen::Matrix4f& guess)
{
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_trans_normals(new pcl::PointCloud<pcl::PointXYZINormal>());

  addNormal(tar, cloud_target_normals);
  addNormal(src, cloud_source_normals);

  pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(
      new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
  icp->setTransformationEpsilon(0.0000000001);
  icp->setMaxCorrespondenceDistance(2.0);
  icp->setMaximumIterations(50);
  icp->setRANSACIterations(20);
  icp->setInputSource(cloud_source_normals);  //
  icp->setInputTarget(cloud_target_normals);
  icp->align(*cloud_source_trans_normals, guess);  //
  std::cout << "score: " << icp->getFitnessScore() << endl;
  return icp->getFinalTransformation();
}


Eigen::Matrix4f PointToPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target,
                             Eigen::Matrix4f& guess)
{
    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // pcl::IterativeClosestPointWithNormals<pcl::PointXYZI, pcl::PointXYZI> icp;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
    tree1->setInputCloud(cloud_source);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZI>);
    tree2->setInputCloud(cloud_target);
    icp.setSearchMethodSource(tree1);
    icp.setSearchMethodTarget(tree2);
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.setMaxCorrespondenceDistance(150);  // 1500
    icp.setTransformationEpsilon(1e-3);
    icp.setEuclideanFitnessEpsilon(0.01);  // 0.1
    icp.setMaximumIterations(100);         // 100
    pcl::PointCloud<pcl::PointXYZI> output;
    icp.align(output);
    //打印配准相关输入信息
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << "Transformation: "<< "\n" << icp.getFinalTransformation() << std::endl;
    return icp.getFinalTransformation();
}

Eigen::Matrix4f genTransformation(Eigen::Vector3f& r, Eigen::Vector3f& t)
{
  Eigen::AngleAxisf init_rotation_x(r.x(), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(r.y(), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(r.z(), Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(t.x(), t.y(), t.z());
  return (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
}

void displayAngel(Eigen::Matrix4f& transformation)
{
    double rx, ry, rz, tx, ty, tz;
    ry = (180 / M_PI) * atan(-(transformation(2, 0) / sqrt(pow(transformation(0, 0), 2) + pow(transformation(1, 0), 2))));
    rz = (180 / M_PI) * atan((transformation(1, 0) / transformation(0, 0)));
    rx = (180 / M_PI) * atan((transformation(2, 1) / transformation(2, 2)));
    tx = transformation(0, 3);
    ty = transformation(1, 3);
    tz = transformation(2, 3);
    std::cout << rx << '\n' << ry << '\n' << rz << '\n' << tx << '\n' << ty << '\n' << tz << std::endl;
}

Eigen::Matrix4f
run(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI>::Ptr tar, const std::vector<float>& params, ICPFunPtr icp)
{
  Eigen::Vector3f r(params[0], params[1], params[2]);
  Eigen::Vector3f t(params[3], params[4], params[5]);
  // Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f guess = genTransformation(r, t);
  return icp(src, tar, guess);
}

void initialzeOutputFile(std::vector<TumTrajectoryPoint>& finalTrajectory,
                        const std::vector<TumTrajectoryPoint> slamTrajectory,
                        const std::vector<TumTrajectoryPoint> gtTrajectory)
{
    for (int i = 0; i < slamTrajectory.size(); i++)
    {
        finalTrajectory[i].timestamp = gtTrajectory[i].timestamp;//和groudTruth轨迹的时间戳对齐，解决evo_ape时间戳不对齐对的问题        
        // finalTrajectory[i].timestamp = slamTrajectory[i].timestamp;
        finalTrajectory[i].rotation = slamTrajectory[i].rotation;//用原来的旋转初始化
    }
}
int main()
{

    const auto yaml = YAML::LoadFile("../config/config.yaml");
    const std::string slamTrajectoryFile = yaml["slamTrajectoryFile"].as<std::string>();
    const std::string gtTrajectoryFile = yaml["gtTrajectoryFile"].as<std::string>();
    const std::string outputTrajectoryFile = yaml["outputTrajectoryFile"].as<std::string>();//output也需要被初始化
    const auto init_params = yaml["init_params"].as<std::vector<float>>();
    const auto icp_pattern = yaml["icp_pattern"].as<std::string>();

    //创建两个pcl::PointCloud<pcl::PointXYZI>共享指针，并初始化它们
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target_transform(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr Final(new pcl::PointCloud<pcl::PointXYZI>);

    // 读取SLAM轨迹和真值轨迹 必须要初始化，否则会coredump
    std::vector<TumTrajectoryPoint> slamTrajectory = readTumTrajectory(slamTrajectoryFile);  
    std::vector<TumTrajectoryPoint> gtTrajectory = readTumTrajectory(gtTrajectoryFile);  
    std::vector<TumTrajectoryPoint> finalTrajectory;
    finalTrajectory.resize(slamTrajectory.size());
    
    //需要初始化slamTrajectory，否则会coredumped(指针泄露)
    cloud_source->resize(slamTrajectory.size());  
    cloud_target->resize(slamTrajectory.size());  
    cloud_target_transform->resize(slamTrajectory.size());  
    Final->resize(slamTrajectory.size());  
    initialzeOutputFile(finalTrajectory, slamTrajectory, gtTrajectory);//把slamTraj中的非position部分加入到finalTraj中
    
    //source
    for (int i = 0; i < slamTrajectory.size(); i++)
    {
        cloud_source->points[i].x = slamTrajectory[i].position[0];//coredumped?
        cloud_source->points[i].y = slamTrajectory[i].position[1];
        cloud_source->points[i].z = slamTrajectory[i].position[2];
    }

    //target
    for (int i = 0; i < gtTrajectory.size(); i++)
    {
        cloud_target->points[i].x = gtTrajectory[i].position[0];
        cloud_target->points[i].y = gtTrajectory[i].position[1];
        cloud_target->points[i].z = gtTrajectory[i].position[2];
    }

    // 确保两个轨迹点的数量是一致的，或者至少是可配对的 (目前暂时不要一致)
    // if (slamTrajectory.size() != gtTrajectory.size()) {  
    //     std::cerr << "The number of points in SLAM and ground truth trajectories does not match!" << std::endl;  
    //     return -1;  
    // }  


    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    std::map<std::string, ICPFunPtr> icp_map{{"icpPlaneToPlane", &icpPlaneToPlane},
                                            {"icpPointToPlane", &icpPointToPlane},
                                            {"PointToPlane", &PointToPlane},
                                            {"PointToPoint", &PointToPoint}};

    // clang-format on: can't use Matrix4d, because the Matrix4f is strictly used in the pcl lib
    Eigen::Matrix4f transformation = run(cloud_source, cloud_target, init_params, icp_map[icp_pattern]);
    displayAngel(transformation);
    // std::cout << "1: " << std::endl;
    pcl::transformPointCloud(*cloud_source, *cloud_target_transform, transformation);//表示从cloud_source到cloud_target_transform的变换T
    // std::cout << "2: " << std::endl;

    // 遍历转换后点云中的每个点 s,并将转换后点的position和rotation进行更新
    for (size_t i = 0; i < cloud_target_transform->size(); ++i) {  
        // 获取当前点的坐标  
        const pcl::PointXYZI& point = cloud_target_transform->points[i];  
            
        // 将坐标复制到finalTrajectory中对应点的position属性  
        finalTrajectory[i].position(0) = point.x;  
        finalTrajectory[i].position(1) = point.y;  
        finalTrajectory[i].position(2) = point.z; 
        // T_W_B0 = T_W_B0 * (T_B0_B01).inv: 计算transform_target到W系下的变换
        Eigen::Matrix3f transformation_R = transformation.block<3,3>(0, 0);
        Eigen::Quaternionf transformation_q(transformation_R);
        finalTrajectory[i].rotation = finalTrajectory[i].rotation * transformation_q.inverse();
    }
    // std::cout << "3: " << std::endl;
    writeTUMTrajectory(outputTrajectoryFile, finalTrajectory);  
    std::cout << "Trajectory written to " << outputTrajectoryFile << std::endl;  
    // std::cout << "4: " << std::endl;


    // display
    pcl::visualization::PCLVisualizer p;
    p.setWindowName("Could after calibration");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tgt_after_transform_h(cloud_target_transform, 0, 255,
                                                                                            0);//transform后的点云(green)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tgt_h(cloud_target, 0, 0, 255);//target点云(blue)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> src_h(cloud_source, 255, 255, 255);//source点云(white)
    p.addPointCloud(cloud_target_transform, tgt_after_transform_h, "target_after_transform");
    p.addPointCloud(cloud_target, tgt_h, "target");
    p.addPointCloud(cloud_source, src_h, "source");
    p.setSize(1200, 900);
    p.spin();
    return 0;
}
