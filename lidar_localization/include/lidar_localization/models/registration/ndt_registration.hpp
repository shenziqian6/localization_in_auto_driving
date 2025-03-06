/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class NDTRegistration: public RegistrationInterface {
  public:
    NDTRegistration(const YAML::Node& node);
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
/*
pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr
是一种智能指针，具体来说，它是boost::shared_ptr的typedef。
这种智能指针会自动管理动态分配的内存。
当最后一个共享指针超出作用域时，所指
向的对象会被自动释放，因此不需要手动
调用delete操作符。这种机制有效地防止
了内存泄漏和双重删除的问题，使得内存
管理更加安全和高效。
*/
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
};
}

#endif