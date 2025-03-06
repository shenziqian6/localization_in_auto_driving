/*
 * @Description: key frames 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/key_frames_publisher.hpp"

#include <Eigen/Dense>

namespace lidar_localization {
KeyFramesPublisher::KeyFramesPublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string frame_id,
                                     int buff_size)
    :nh_(nh), frame_id_(frame_id) {

    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
}

void KeyFramesPublisher::Publish(const std::deque<KeyFrame>& key_frames) {
    /*
    1、std_msgs::Header header  
        stamp（时间戳，类型为 ros::Time）
        frame_id（坐标系ID，类型为 std::string）
    */
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;

    for (size_t i = 0; i < key_frames.size(); ++i) {
        KeyFrame key_frame = key_frames.at(i);
        /*
        1、geometry_msgs::PoseStamped pose  
            header（同上，类型为 std_msgs::Header）
            pose（姿态，类型为 geometry_msgs::Pose）
        2、 Pose (geometry_msgs::Pose)
            类型: geometry_msgs::Pose
            描述: 包含位置和方向信息。
            详细内容:
            cpp
            geometry_msgs::Pose pose  
            position（位置，类型为 geometry_msgs::Point）
            orientation（方向，类型为 geometry_msgs::Quaternion）
        3、Point (geometry_msgs::Point)
            类型: geometry_msgs::Point
            描述: 表示三维空间中的一个点。
            详细内容:
            cpp
            geometry_msgs::Point position  
            x（x坐标，类型为 float64）
            y（y坐标，类型为 float64）
            z（z坐标，类型为 float64）
        4、 Quaternion (geometry_msgs::Quaternion)
            类型: geometry_msgs::Quaternion
            描述: 表示三维空间中的旋转（四元数）。
            详细内容:
            cpp
            geometry_msgs::Quaternion orientation  
            x（x分量，类型为 float64）
            y（y分量，类型为 float64）
            z（z分量，类型为 float64）
            w（w分量，类型为 float64）
        */
        geometry_msgs::PoseStamped pose_stamped;
        ros::Time ros_time((float)key_frame.time);
        pose_stamped.header.stamp = ros_time;
        pose_stamped.header.frame_id = frame_id_;

        pose_stamped.header.seq = key_frame.index;

        pose_stamped.pose.position.x = key_frame.pose(0,3);
        pose_stamped.pose.position.y = key_frame.pose(1,3);
        pose_stamped.pose.position.z = key_frame.pose(2,3);

        Eigen::Quaternionf q = key_frame.GetQuaternion();
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path.poses.push_back(pose_stamped);
    }

    publisher_.publish(path);
}

bool KeyFramesPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}