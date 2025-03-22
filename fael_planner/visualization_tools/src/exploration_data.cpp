//
// Created by hjl on 2021/7/30.
//

#include "exploration_data.h"

exploration_data::exploration_data(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private), path_length_sum(0.0),
                                                                        last_position(0, 0, 0), known_cell_num(0), known_plane_cell_num(0),
                                                                        known_map_resolution(0), known_space_volume(0), system_inited(false),
                                                                        exploration_finish(false), seq(0)
{
    //known_cell_num、known_plane_cell_num、known_map_resolution、known_space_volume：
    //分别存储地图中已知体素数量、已知平面体素数量、地图分辨率以及根据平面体素计算的已知面积（或体积），均初始化为0。
    init_sub = nh_.subscribe<std_msgs::Float64>("explorer_inited", 1, &exploration_data::explorationInitCallback, this);
    finish_sub = nh_.subscribe<std_msgs::Float64>("explorer_finish", 1, &exploration_data::explorationFinishCallback, this);
    odom_sub = nh_.subscribe<nav_msgs::Odometry>("odometry", 1, &exploration_data::odomCallback, this);
    // point_cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>("point_cloud", 1, &exploration_data::pointCloudCallback, this);
    map_frontiers_sub = nh_.subscribe<ufomap_manager::UfomapWithFrontiers>("ufomap_and_frontiers", 1,
                                                                           &exploration_data::mapAndFrontiersCallback,
                                                                           this);
    iteration_time_sub = nh_.subscribe<visualization_tools::IterationTime>("iteration_time", 1,
                                                                           &exploration_data::iterationTimeCallback,
                                                                           this);
    //发布话题：
    // "traved_distance_time"：发布累计行驶距离相关信息。
    // "explored_volume_time"：发布已探索面积（或体积）。
    // "explored_volume_traved_dist_time"：同时发布探索面积和累计行驶距离的数据。
    // "run_time"：发布每次迭代耗时信息。
    traved_dist_pub = nh_.advertise<visualization_tools::TravedDistTime>("traved_distance_time", 1);
    explorated_volume_pub = nh_.advertise<visualization_tools::ExploredVolumeTime>("explored_volume_time", 1);
    explorated_volume_traved_dist_time_pub = nh_.advertise<visualization_tools::ExploredVolumeTravedDistTime>(
        "explored_volume_traved_dist_time", 1);

    iteration_time_pub = nh_.advertise<visualization_tools::IterationTime>("run_time", 1);
    pub_timer = nh_private_.createTimer(ros::Duration(0.2), &exploration_data::pubExplorationData, this);

    explore_finish_pub = nh_.advertise<std_msgs::Bool>("exploration_data_finish", 1);

    const std::string &ns = ros::this_node::getName();
    std::string pkg_path = ros::package::getPath("visualization_tools");
    std::string txt_path = pkg_path + "/../../files/exploration_data/";

    // exploredVolumeVoxelSize = 0.4;
    // if (!ros::param::get(ns + "/exploredVolumeVoxelSize", exploredVolumeVoxelSize))
    // {
    //     ROS_WARN("No exploredVolumeVoxelSize specified. Looking for %s. Default is 0.4", (ns + "/exploredVolumeVoxelSize").c_str());
    // }

    max_volume = 100000000.0;
    if (!ros::param::get(ns + "/map_area", max_volume))
    {
        ROS_WARN("No map_area specified. Looking for %s. Default is 100000000", (ns + "/map_area").c_str());
    }

    max_time = 100000000.0;
    if (!ros::param::get(ns + "/max_time", max_time))
    {
        ROS_WARN("No max_time specified. Looking for %s. Default is 100000000", (ns + "/max_time").c_str());
    }

    // // 点云初始化
    // laserCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // exploredVolumeCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // exploredVolumeCloud2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // exploredPlaneCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // // 点云下采样
    // exploredVolumeDwzFilter.reset(new pcl::VoxelGrid<pcl::PointXYZI>());
    // exploredVolumeDwzFilter->setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

    // 构造三个文本文件的完整路径，用于存储：
    // 时间、累计行驶距离和探索面积数据；
    // 每次迭代耗时数据；
    // 机器人轨迹（位置数据）。
    distance_volume_txt_name = txt_path + "time_distance_volume.txt";
    iteration_time_txt_name = txt_path + "iteration_time.txt";
    trajectory_txt_name = txt_path + "trajectory.txt";
    comp_data_txt_name = txt_path + "comp_data.txt";

    std::ofstream fout;
    fout.open(distance_volume_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    fout << "explored time \t"
         << "distance/path-length \t"
         << "explored volume(m2) \t"
         << "known plane cell num \t"
         << "known voxel num(3d) \t"
         << std::endl;
    fout.close();

    sum_iteration_time = 0;
    iteration_num = 0;
    fout.open(iteration_time_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    fout << "explored time \t"
         << " iteration time \t"
         << "iteration_num \t"
         << "average time(s) \t"
         << std::endl;
    fout.close();

    fout.open(trajectory_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    fout << "explored time \t"
         << " x \t"
         << "y \t"
         << "z \t"
         << std::endl;
    fout.close();

    fout.open(comp_data_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    fout << "explored time \t"
         << "x \t"
         << "y \t"
         << "z \t"
         << "explored volume(m2) \t"
         << "iteration time \t"
         << "distance/path-length \t"
         << std::endl;
}

void exploration_data::explorationInitCallback(const std_msgs::Float64ConstPtr &init)
{

    if (!system_inited)
    {
        system_init_time = init->data;
        system_inited = true;
    }
    ROS_INFO("visualization statics start.");
}

void exploration_data::explorationFinishCallback(const std_msgs::Float64ConstPtr &finish)
{

    exploration_finish = true;
    finish_time = finish->data;
}

void exploration_data::odomCallback(const nav_msgs::OdometryConstPtr &input)
{

    tf::Vector3 current_position(input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z);
    double dist_delta = current_position.distance(last_position);
    last_position = current_position;

    if (system_inited)
    {
        path_length_sum += dist_delta;//累计机器人的行驶距离。
    }
}

// void exploration_data::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
// {//frame_id: "jackal/velodyne/VLP_16"
//     if (!system_inited)
//     {
//         return;
//     }
//     laserCloud->clear();
//     pcl::fromROSMsg(*cloud, *laserCloud);

//     *exploredVolumeCloud += *laserCloud;
//     //对点云进行下采样：
//     exploredVolumeCloud2->clear();
//     exploredVolumeDwzFilter->setInputCloud(exploredVolumeCloud);
//     exploredVolumeDwzFilter->filter(*exploredVolumeCloud2);

//     pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
//     exploredVolumeCloud = exploredVolumeCloud2;
//     exploredVolumeCloud2 = tempCloud;
//     //计算已探索体积和面积：
//     exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

//     exploredPlaneCloud->clear();
//     double offset = 0.6;
//     for (const auto& point : exploredVolumeCloud->points) {
//         double adjust_z = point.z + offset;
//         if (adjust_z <= exploredVolumeVoxelSize) {
//             exploredPlaneCloud->push_back(point);
//         }
//     }
//     exploredArea = exploredVolumeVoxelSize * exploredVolumeVoxelSize * exploredPlaneCloud->points.size();
// }

void exploration_data::mapAndFrontiersCallback(const ufomap_manager::UfomapWithFrontiersConstPtr &msg)
{

    known_map_resolution = msg->ufomap.info.resolution;
    known_cell_num = msg->known_cell_codes.size();
    known_plane_cell_num = msg->known_plane_cell_num;
    known_space_volume =
        static_cast<double>(known_plane_cell_num) * known_map_resolution * known_map_resolution; // 暂时发布面积
}

void exploration_data::iterationTimeCallback(const visualization_tools::IterationTimeConstPtr &msg)
{
    if (system_inited && !exploration_finish)
    {//系统已初始化且探索未结束：

        visualization_tools::IterationTime time;
        time.timeConsumed = msg->current_time - system_init_time;
        time.iterationTime = msg->iterationTime;
        iteration_time_pub.publish(time);

        sum_iteration_time += time.iterationTime;
        iteration_num++;
        std::ofstream fout;
        fout.open(iteration_time_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << time.timeConsumed << "\t" << time.iterationTime << "\t" << iteration_num << "\t"
             << sum_iteration_time / iteration_num << "\t"
             << std::endl;
        fout.close();
    }
}

void exploration_data::pubExplorationData(const ros::TimerEvent &event)
{

    if (system_inited && !exploration_finish)
    {
        visualization_tools::ExploredVolumeTravedDistTime exploration_data;
        exploration_data.header.stamp = ros::Time::now();
        exploration_data.header.frame_id = "world";
        exploration_data.header.seq = ++seq;

        exploration_data.exploredVolume = known_space_volume;
        // exploration_data.exploredVolume = exploredArea;
        exploration_data.travedDist = path_length_sum;
        double time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                     std::chrono::high_resolution_clock::now().time_since_epoch())
                                                     .count()) /
                             1000000;
        exploration_data.timeConsumed = time_second - system_init_time;
        exploration_data.iterationTime = sum_iteration_time / iteration_num;
        //如果已探索面积超过设定最大体积的 95%，则：
        // 标记 exploration_finish 为 true，
        // 发布 exploration_data_finish 消息，通知系统探索结束，
        if (exploration_data.exploredVolume > 0.95 * max_volume)
        {
            exploration_finish = true;
            std_msgs::Bool explore_finish;
            explore_finish.data = true;
            explore_finish_pub.publish(explore_finish);
            ROS_INFO("max volume threshold is reach...");
        }

        if (time_second - system_init_time > max_time)
        {
            exploration_finish = true;
            std_msgs::Bool explore_finish;
            explore_finish.data = true;
            explore_finish_pub.publish(explore_finish);
            ROS_INFO("max time is reach...");
        }

        explorated_volume_traved_dist_time_pub.publish(exploration_data);

        std::ofstream fout;
        fout.open(distance_volume_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << exploration_data.timeConsumed << "\t" << exploration_data.travedDist << "\t"
             << exploration_data.exploredVolume << "\t" << known_plane_cell_num << "\t"
             << known_cell_num << std::endl;
        fout.close();

        fout.open(trajectory_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << exploration_data.timeConsumed << "\t" << last_position.x() << "\t" << last_position.y() << "\t"
             << last_position.z() << std::endl;
        fout.close();

        // fout.open(comp_data_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        // fout << exploration_data.timeConsumed << "\t" << last_position.x() << "\t" << last_position.y() << "\t"
        //      << last_position.z() << "\t" << exploration_data.exploredVolume << "\t" << exploration_data.iterationTime
        //      << "\t" << exploration_data.travedDist << std::endl;

        fout.open(comp_data_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << std::fixed << std::setprecision(6)
        << exploration_data.timeConsumed << "\t"
        << last_position.x() << "\t" 
        << last_position.y() << "\t"
        << last_position.z() << "\t"
        << exploration_data.exploredVolume << "\t"
        << exploration_data.iterationTime << "\t"
        << exploration_data.travedDist << std::endl;
        fout.close();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration_data");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    exploration_data data(nh, nh_private);

    while (ros::ok())
    {
        ros::spinOnce();

        if (data.exploration_finish)
        {
            std_msgs::Bool explore_finish;
            explore_finish.data = true;
            data.explore_finish_pub.publish(explore_finish);
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            ros::shutdown();
        }
    }
    return 0;
}
