/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-28 18:50:16
 */
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;  //四元数，表示了3D空间的旋转
    q = pose.block<3,3>(0,0);

    return q;
}
}