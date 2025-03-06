/*
 * @Description: 闭环检测算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/mapping/loop_closing/loop_closing.hpp"

#include <cmath>
#include <algorithm>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"
#include "lidar_localization/tools/print_info.hpp"

namespace lidar_localization {
LoopClosing::LoopClosing() {
    InitWithConfig();
}

bool LoopClosing::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/loop_closing.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------闭环检测初始化-------------------" << std::endl;
    InitParam(config_node);
    InitDataPath(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("map", map_filter_ptr_, config_node);
    InitFilter("scan", scan_filter_ptr_, config_node);

    return true;
}

bool LoopClosing::InitParam(const YAML::Node& config_node) {
    extend_frame_num_ = config_node["extend_frame_num"].as<int>();
    loop_step_ = config_node["loop_step"].as<int>();
    diff_num_ = config_node["diff_num"].as<int>();
    detect_area_ = config_node["detect_area"].as<float>();
    fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();

    return true;
}

bool LoopClosing::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    key_frames_path_ = data_path + "/slam_data/key_frames";

    return true;
}

bool LoopClosing::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "闭环点云匹配方式为：" << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

bool LoopClosing::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "闭环的" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr =std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

bool LoopClosing::Update(const KeyFrame key_frame, const KeyFrame key_gnss) {
    has_new_loop_pose_ = false;

    all_key_frames_.push_back(key_frame);
    all_key_gnss_.push_back(key_gnss);

    int key_frame_index = 0;
    //找到距离最近的关键帧
    if (!DetectNearestKeyFrame(key_frame_index))
        return false;

    if (!CloudRegistration(key_frame_index))
        return false;

    has_new_loop_pose_ = true;
    return true;
}

bool LoopClosing::DetectNearestKeyFrame(int& key_frame_index) {
    static int skip_cnt = 0;
    /*
    2）为避免频繁检测，每检测一次，就做一次等待

       这个等待的帧数，也可以通过配置文件设置，对应的参数为loop_step，即每隔loop_step个关键帧检测一次。
       loop_step: 5 # 防止检测过于频繁，每隔loop_step个关键帧检测一次闭环
        detect_area: 10.0 # 检测区域，只有两帧距离小于这个值，才做闭环匹配
        diff_num: 100 # 过于小的闭环没有意义，所以只有两帧之间的关键帧个数超出这个值再做检测
    */
    static int skip_num = loop_step_;
    if (++skip_cnt < skip_num)
        return false;

    if ((int)all_key_gnss_.size() < diff_num_ + 1)
        return false;

    int key_num = (int)all_key_gnss_.size();
    float min_distance = 1000000.0;
    float distance = 0.0;

    KeyFrame history_key_frame;
    KeyFrame current_key_frame = all_key_gnss_.back();

    key_frame_index = -1;
    for (int i = 0; i < key_num - 1; ++i) {
    /*
        1）设置最小时间差

        这里的时间差，严格来讲是关键帧路程差，关键帧的编号是随着车的前进顺序递增产生的，
        我们设置一个参数diff_num，假如当前帧编号为index，那么寻找匹配的帧应该从编号index-diff_num之前的帧中去查找。
        diff_num这个值可以通过配置文件设置。
    */
        if (key_num - i < diff_num_)
            break;
        
        history_key_frame = all_key_gnss_.at(i);
        distance = fabs(current_key_frame.pose(0,3) - history_key_frame.pose(0,3)) + 
                   fabs(current_key_frame.pose(1,3) - history_key_frame.pose(1,3)) + 
                   fabs(current_key_frame.pose(2,3) - history_key_frame.pose(2,3));
        if (distance < min_distance) {
            min_distance = distance;
            key_frame_index = i;
        }
    }
    /*
    # 匹配时为了精度更高，应该选用scan-to-map的方式
    # map是以历史帧为中心，往前后时刻各选取extend_frame_num个关键帧，放在一起拼接成的
    extend_frame_num: 5 
    */
    if (key_frame_index < extend_frame_num_)  //这表示在开头，抛弃掉
        return false;

    skip_cnt = 0;
    skip_num = (int)min_distance;
    //detect_area: 10.0 # 检测区域，只有两帧距离小于这个值，才做闭环匹配
    if (min_distance > detect_area_) {
        /*
        为了解决如果明知道某一段距离内不会有符合条件的历史帧，那么检测就不必要
        3）根据当前最小距离计算重新计算等待时间
        假如本次检测，当前帧和满足条件1）的历史帧中的最短距离是100米，
        而我们需要的是两米范围内的关键帧，那么我们有理由认为在接下来的至少98米路程内，
        是不会有满足条件的历史帧的，那么这段距离内就没必要检测了。
        */
        skip_num = std::max((int)(min_distance / 2.0), loop_step_);
        return false;
    } else {
        skip_num = loop_step_;
        return true;
    }
}

bool LoopClosing::CloudRegistration(int key_frame_index) {
    // 生成地图
    CloudData::CLOUD_PTR map_cloud_ptr(new CloudData::CLOUD());
    Eigen::Matrix4f map_pose = Eigen::Matrix4f::Identity();
    //这里获取到检测到回环的那个关键帧map_pose  和关键帧对应的雷达点云map_cloud_ptr
    JointMap(key_frame_index, map_cloud_ptr, map_pose);

    // 生成当前scan
    CloudData::CLOUD_PTR scan_cloud_ptr(new CloudData::CLOUD());
    Eigen::Matrix4f scan_pose = Eigen::Matrix4f::Identity();
    //获取到当前帧的雷达点云scan_cloud_ptr，和当前关键帧scan_pose
    JointScan(scan_cloud_ptr, scan_pose);

    // 匹配
    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    Registration(map_cloud_ptr, scan_cloud_ptr, scan_pose, result_pose);

    // 计算相对位姿  Tlast_curr=     Tlidar-last_w1*Tw1_lidar-curr
    current_loop_pose_.pose = map_pose.inverse() * result_pose;

    // 判断是否有效
    if (registration_ptr_->GetFitnessScore() > fitness_score_limit_)
        return false;
    
    static int loop_close_cnt = 0;
    loop_close_cnt ++;

    std::cout << "检测到闭环 "<<  loop_close_cnt
              << ": 帧" << current_loop_pose_.index0 
              << "------>" << "帧" << current_loop_pose_.index1 << std::endl
              << "fitness score: " << registration_ptr_->GetFitnessScore() 
              << std::endl << std::endl;

    // std::cout << "相对位姿 x y z roll pitch yaw:";
    // PrintInfo::PrintPose("", current_loop_pose_.pose);

    return true;
}

bool LoopClosing::JointMap(int key_frame_index, CloudData::CLOUD_PTR& map_cloud_ptr, Eigen::Matrix4f& map_pose) {
    //将距离最近的关键帧拿出来
    map_pose = all_key_gnss_.at(key_frame_index).pose;
    current_loop_pose_.index0 = all_key_frames_.at(key_frame_index).index;
    
    // 合成地图  这里的w1是gnss的雷达坐标系     w2是lidar以初始点为原点的雷达坐标系
    //   Tw1_w2=Tw1_lidar*Tlidar_w2;  就是将lidar坐标系于gnss坐标系对齐
    Eigen::Matrix4f pose_to_gnss = map_pose * all_key_frames_.at(key_frame_index).pose.inverse();
    //该历史帧为中心，按时间往前和往后各索引几个关键帧，拼接成一个小地图
    for (int i = key_frame_index - extend_frame_num_; i < key_frame_index + extend_frame_num_; ++i) {
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.at(i).index) + ".pcd";
        
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        pcl::io::loadPCDFile(file_path, *cloud_ptr);  //将关键帧导进来
        //Tw1_lidar=Tw1_w2*Tw2_lidar   
        Eigen::Matrix4f cloud_pose = pose_to_gnss * all_key_frames_.at(i).pose;
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, cloud_pose);  //将雷达的点云由雷达坐标系变换到世界坐标系

        *map_cloud_ptr += *cloud_ptr;
    }
    map_filter_ptr_->Filter(map_cloud_ptr, map_cloud_ptr);
    return true;
}

bool LoopClosing::JointScan(CloudData::CLOUD_PTR& scan_cloud_ptr, Eigen::Matrix4f& scan_pose) {
    scan_pose = all_key_gnss_.back().pose;
    current_loop_pose_.index1 = all_key_frames_.back().index;
    current_loop_pose_.time = all_key_frames_.back().time;

    std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.back().index) + ".pcd";
    pcl::io::loadPCDFile(file_path, *scan_cloud_ptr);
    scan_filter_ptr_->Filter(scan_cloud_ptr, scan_cloud_ptr);

    return true;
}

bool LoopClosing::Registration(CloudData::CLOUD_PTR& map_cloud_ptr, 
                               CloudData::CLOUD_PTR& scan_cloud_ptr, 
                               Eigen::Matrix4f& scan_pose, 
                               Eigen::Matrix4f& result_pose) {
    // 点云匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->SetInputTarget(map_cloud_ptr);
    registration_ptr_->ScanMatch(scan_cloud_ptr, scan_pose, result_cloud_ptr, result_pose);

    return true;
}

bool LoopClosing::HasNewLoopPose() {
    return has_new_loop_pose_;
}

LoopPose& LoopClosing::GetCurrentLoopPose() {
    return current_loop_pose_;
}
}