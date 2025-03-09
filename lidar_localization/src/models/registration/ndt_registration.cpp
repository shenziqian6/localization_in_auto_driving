/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {

NDTRegistration::NDTRegistration(const YAML::Node& node)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
    
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

//第一步设置参数
bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    //设置分辨率
    ndt_ptr_->setResolution(res);
    //设置步长
    ndt_ptr_->setStepSize(step_size);
    //设置变换误差
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    //设置最大迭代次数
    ndt_ptr_->setMaximumIterations(max_iter);

    std::cout << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}
//第二步，设置目标点云
bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    
    ndt_ptr_->setInputTarget(input_target);

    return true;
}
/*
    这是在回环检测的匹配情况：
    //旋转和平移后的点与目标点集中的点在同一坐标系下
    //predict_pose这里是Tw1_lidar    input_source是lidar坐标系的点云Tlidar_cloud   
    //这里是将input_source转换到input_target同一个坐标系，然后进行ndt匹配  因为检测到回环，所以input_source和input_target这两个点云应该相距不远，只需要将这两个点云切换到同一个坐标系
*/

bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    //设置源点云
    ndt_ptr_->setInputSource(input_source);
    /*
    align 是 pcl::NormalDistributionsTransform 类中的一个关键方法，
    用于执行实际的点云配准操作。它的功能是将源点云（input_source）与
    目标点云（input_target）进行匹配，从而计算出源点云到目标点云的变
    换（如位姿或变换矩阵）。
    */
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    /*
    具体来说，result_pose是将源点云从其原坐标系变换到目标点云所在的坐标系的位姿矩阵。这意味着result_pose反映了源点云相对于目标点云（或世界坐标系）的最终位姿变换。
    */
    result_pose = ndt_ptr_->getFinalTransformation();

    return true;
}

float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
}
}