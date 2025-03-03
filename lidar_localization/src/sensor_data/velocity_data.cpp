/*
 * @Description: velocity data
 * @Author: Ren Qian
 * @Date: 2020-02-23 22:20:41
 */
#include "lidar_localization/sensor_data/velocity_data.hpp"

#include "glog/logging.h"

namespace lidar_localization {
bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    VelocityData front_data = UnsyncedData.at(0);
    VelocityData back_data = UnsyncedData.at(1);
    VelocityData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
    synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
    synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}

void VelocityData::TransformCoordinate(Eigen::Matrix4f transform_matrix) {
    Eigen::Matrix4d matrix = transform_matrix.cast<double>();
    Eigen::Matrix3d t_R = matrix.block<3,3>(0,0);
    Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    w = t_R * w;
    v = t_R * v;

    Eigen::Vector3d r(matrix(0,3), matrix(1,3), matrix(2,3));
    Eigen::Vector3d delta_v;
    /*
    当刚体进行旋转运动时，质点在旋转参考系中的线速度不仅仅包括其相对旋转参考系的运动（v_rel） ，还包括由于旋转本身引起的速度。
    总的线速度可以表示为：v = v_rel + ω × r，其中ω是角速度向量，r是质点的位置向量。

    ω × r（w叉乘r）用于计算由于旋转引起的速度变化  其中ω是角速度向量，r是质点的位置向量。
    那么，叉乘的具体计算是怎么样的呢？对于向量a = (a1, a2, a3)和向量b = (b1, b2, b3)，它们的叉乘a × b的结果是：

    (a × b)_0 = a2 * b3 - a3 * b2
    (a × b)_1 = a3 * b1 - a1 * b3
    (a × b)_2 = a1 * b2 - a2 * b1
    */
    delta_v(0) = w(1) * r(2) - w(2) * r(1);
    delta_v(1) = w(2) * r(0) - w(0) * r(2);
    delta_v(2) = w(1) * r(1) - w(1) * r(0);
    v = v + delta_v;

    angular_velocity.x = w(0);
    angular_velocity.y = w(1);
    angular_velocity.z = w(2);
    linear_velocity.x = v(0);
    linear_velocity.y = v(1);
    linear_velocity.z = v(2);
}
}