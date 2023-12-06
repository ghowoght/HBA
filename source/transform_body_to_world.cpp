/**
 * @file transform_body_to_world.cpp
 * @author Linfu Wei (ghowoght@qq.com)
 * @brief 将点云从机体坐标系转换到世界坐标系 
 * @version 1.0
 * @date 2023-12-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>

int main(int argc, char ** argv){
    ros::init(argc, argv, "transform_body_to_world");
    ros::NodeHandle nh;

    std::string path = "";
    ros::param::get("~path", path);
    if(path == ""){
        std::cout << "please input path" << std::endl;
        return -1;
    }
    std::cout << "path: " << path << std::endl;

    // 读取文件夹下所有文件夹名,并存入vector    
    std::vector<std::string> sub_paths;
    for (const auto & entry : std::filesystem::directory_iterator(path)){
        sub_paths.push_back(entry.path());
    }
    std::sort(sub_paths.begin(), sub_paths.end());
    for(auto & sub_path : sub_paths){
        std::cout << sub_path << std::endl;
    }

    auto sub_path = sub_paths.back();

    auto hba_path = sub_path + "/hba_output";
    auto pcd_path = hba_path + "/pcd";
    auto pcd_out_path = hba_path + "/pcd_world";
    if(!std::filesystem::exists(pcd_out_path)){
        std::filesystem::create_directory(pcd_out_path);
    }
    auto pose_file = hba_path + "/pose.json";
    auto pcd_total_file = hba_path + "/pcd_total.pcd";

    // 读取位姿
    std::vector<Eigen::Matrix3d> Rs;
    std::vector<Eigen::Vector3d> ts;
    std::ifstream pose_ifs(pose_file);
    if(!pose_ifs.is_open()){
        std::cout << "open pose file failed" << std::endl;
        return -1;
    }
    while(!pose_ifs.eof()){
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        pose_ifs >> t[0] >> t[1] >> t[2];
        Eigen::Quaterniond q;
        pose_ifs >> q.w() >> q.x() >> q.y() >> q.z();
        R = q.toRotationMatrix();
        Rs.push_back(R);
        ts.push_back(t);
    }
    pose_ifs.close();
    std::cout << "pose size: " << Rs.size() << std::endl;

    // 读取点云
    std::vector<std::string> pcd_files;
    for (const auto & entry : std::filesystem::directory_iterator(pcd_path)){
        pcd_files.push_back(entry.path());
    }

    std::sort(pcd_files.begin(), pcd_files.end(), [](const std::string & a, const std::string & b){
        return std::stoi(a.substr(a.find_last_of('/') + 1, a.find_last_of('.') - a.find_last_of('/') - 1)) < std::stoi(b.substr(b.find_last_of('/') + 1, b.find_last_of('.') - b.find_last_of('/') - 1));
    });

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_total(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i = 0; i < pcd_files.size(); ++i){
        std::cout << "pcd file: " << pcd_files[i] << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI>);
        if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files[i], *cloud) == -1){
            std::cout << "load pcd file failed" << std::endl;
            return -1;
        }
        for(int j = 0; j < cloud->size(); ++j){
            Eigen::Vector3d p(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
            p = Rs[i] * p + ts[i];
            pcl::PointXYZI p_world;
            p_world.x = p[0];
            p_world.y = p[1];
            p_world.z = p[2];
            p_world.intensity = cloud->points[j].intensity;
            cloud_world->push_back(p_world);
        }
        *cloud_total += *cloud_world;
        pcl::io::savePCDFileBinary(pcd_out_path + "/" + std::to_string(i) + ".pcd", *cloud_world);
    }
    pcl::io::savePCDFileBinary(pcd_total_file, *cloud_total);

}