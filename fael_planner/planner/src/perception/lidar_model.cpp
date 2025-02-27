//
// Created by hjl on 2022/4/7.
//

#include "perception/lidar_model.h"

LidarModel::LidarModel(double max_range,
                       double vertical_angle_theta, double vertical_angle_min, double vertical_angle_max,
                       double horizontal_angle_theta, double horizontal_angle_min, double horizontal_angle_max) {
    max_range_ = max_range;
    //垂直方向扫描分辨率以及最小/最大角度（单位度数）。通过垂直方向扫描范围和theta计算出线数
    vertical_angle_theta_ = vertical_angle_theta;
    vertical_angle_min_ = vertical_angle_min;
    vertical_angle_max_ = vertical_angle_max;
    //Calculate the number of lines by the vertical scan range and theta
    line_num_ = static_cast<int>((vertical_angle_max_ - vertical_angle_min_) / vertical_angle_theta_) + 1;

    horizontal_angle_theta_ = horizontal_angle_theta;
    horizontal_angle_min_ = horizontal_angle_min;
    horizontal_angle_max_ = horizontal_angle_max;
    //Calculate the number of horizontal points by the horizontal scan range and theta
    horizontal_point_num_ =
            static_cast<int>((horizontal_angle_max_ - horizontal_angle_min_) / horizontal_angle_theta_);

    lidar_point_cloud_.clear();
    for (int i = 0; i < line_num_; ++i) {
        for (int j = 0; j < horizontal_point_num_; ++j) {
            double theta = (horizontal_angle_min_ + j * horizontal_angle_theta_) * M_PI / 180;
            double phi = (vertical_angle_min_ + i * vertical_angle_theta_) * M_PI / 180;
            pcl::PointXYZ point(max_range * cos(phi) * cos(theta), max_range * cos(phi) * sin(theta),
                                max_range * sin(phi));
            //Store the scanned point cloud (taking the lidar as the origin)
            lidar_point_cloud_[i][j] = point;//在该方向若打到最大距离时，对应的点坐标。
        }
    }
}


pcl::PointCloud<pcl::PointXYZ>
LidarModel::expandNoRay(pcl::PointCloud<pcl::PointXYZ> &input_cloud) {
    std::map<int, std::map<int, bool>> model_cloud;
    for (int i = 0; i < line_num_; ++i) {
        for (int j = 0; j < horizontal_point_num_; ++j) {
            model_cloud[i][j] = false;
        }
    }
    //Convert the xyz coordinate to the index of a two-dimensional array, 
    //and set the element corresponding to the index to true
    for (const auto &point:input_cloud) {
        
        double theta = atan2(point.y, point.x) * 180 / M_PI;
        if (theta < 0) {
            theta += 360.0;
        }
        double phi = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int j = round((theta - horizontal_angle_min_) / horizontal_angle_theta_);
        if (j < 0) {
            j = 0;
        } else if (j >= horizontal_point_num_) {
            j = horizontal_point_num_ - 1;
        }
        
        int i = round((phi - vertical_angle_min_) / vertical_angle_theta_);
        if (i < 0)
            i = 0;
        else if (i >= line_num_)
            i = line_num_ - 1;

        model_cloud[i][j] = true;//该方向实际探测到了点。
    }

    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    for (int i = 0; i < line_num_; ++i) {
        for (int j = 0; j < horizontal_point_num_; ++j) {
            if (!model_cloud[i][j])
                output_cloud.push_back(lidar_point_cloud_[i][j]);
        }
    }

    return output_cloud;
}
//将实际观测结果覆盖到理想模型中，从而得到一个“填充了实际观测点 + 未观测方向保留满量程点”的网格化雷达点云。
pcl::PointCloud<pcl::PointXYZ> LidarModel::combineCloud(pcl::PointCloud<pcl::PointXYZ> &input_cloud) {
    std::map<int, std::map<int, pcl::PointXYZ>> model_cloud = lidar_point_cloud_;
    for (const auto &point:input_cloud) {
        double theta = atan2(point.y, point.x) * 180 / M_PI;
        if (theta < 0) {
            theta += 360.0;
        }
        double phi = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int j = round((theta - horizontal_angle_min_) / horizontal_angle_theta_);
        if (j < 0) {
            j = 0;
        } else if (j >= horizontal_point_num_) {
            j = horizontal_point_num_ - 1;
        }

        int i = round((phi - vertical_angle_min_) / vertical_angle_theta_);
        if (i < 0)
            i = 0;
        else if (i >= line_num_)
            i = line_num_ - 1;

        model_cloud[i][j] = point;
    }

    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    for (int i = 0; i < line_num_; ++i) {
        for (int j = 0; j < horizontal_point_num_; ++j) {
            output_cloud.push_back(model_cloud[i][j]);
        }
    }

    return output_cloud;
}
