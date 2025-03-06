/*
 * @Description: 点云匹配模块的基类
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:25:11
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class RegistrationInterface {
  public:
    virtual ~RegistrationInterface() = default;
    /*
    在C++中，当基类中的虚函数被声明为= 0时，
    这个函数被称为纯虚函数。这种情况下，
    任何继承自该基类的子类必须实现这个纯虚函数，
    否则子类将是一个抽象类，无法被实例化。
    因此，子类通常需要重写这个函数，除非子类本身也被设计为抽象类。
    */
    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
    virtual float GetFitnessScore() = 0;
};
} 

#endif