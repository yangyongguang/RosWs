#ifndef GROUNDREMOVE_H_
#define GROUNDREMOVE_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include "bin.h"
#include "segment.h"
#include <iostream>
#include <chrono>
#include <thread>

// for visualzie
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <mutex>
#include <algorithm>

// #include <sensor_msgs/PointCloud2.h>  // ros PointCloud2
// #include <sensor_msgs/PointCloud.h>

struct GroundSegmentationParams{
    GroundSegmentationParams():
    visualize(true),
    // r_min_square(0.3 * 0.3),
    r_min_square(3.4 * 3.4),
    // r_max_square(20 * 20),
    r_max_square(120 * 120),
    // n_bins(30),
    n_bins(80),
    // n_segments(180),
    n_segments(120),
    max_dist_to_line(0.15),
    max_slope(0.25),  // 控制斜率， 斜率是直线拟合求出来的
    n_threads(1),
    max_error_square(0.01),
    long_threshold(2.0),
    max_long_height(0.2),
    max_start_height(0.35),
    sensor_height(1.73),
    line_search_angle(0.2),
    //###########
    r_max_bin(2),
    r_min_bin(0.05),
    /*另外添加的参数*/
    tHmin(-1.9),
    tHmax(-0.5),
    tHDiff(0.4),
    hSensor(1.73),    
    //###########
    min_split_dist(0.1)
    { }
    /*另外添加的参数*/
    double tHmin;
    double tHmax;
    double tHDiff;
    double hSensor;

    ///////////////
    // bin 的设置
    double r_min_bin;
    double r_max_bin;
    // 是否可视化
    bool visualize;

    // 最小范围距离
    double r_min_square;

    double r_max_square;
    int n_bins;
    int n_segments;
    // 距离直线段最大允许距离， 被判断为地面的最大允许距离直线的距离
    double max_dist_to_line;
    double max_slope;
    double max_error_square;
    // 认为俩个点足够远的条件
    double long_threshold;
    // 
    double max_long_height;
    // 足够重新开始
    // Maximum heigh of starting line to be labelled ground.
    // 斜率不能太大， 足够长，当前点计算的 z 与 期望的 z 不能太大
    double max_start_height;
    // 传感器距离地面的高度
    double sensor_height;
    // 左右直线的搜索幅度距离
    double line_search_angle;
    // 线程数量
    int n_threads;

    double min_split_dist;
};

typedef sensor_msgs::PointCloud PointCloud;
// typedef std::pair<PointXYZ, PointXYZ> PointLine;
typedef std::pair<geometry_msgs::Point, geometry_msgs::Point> PointLine;


class GroundSegmentation
{
public:
    GroundSegmentationParams params_;
    GroundSegmentation(ros::Publisher & line_pub, const GroundSegmentationParams & params = GroundSegmentationParams());
    // 全局地图
    std::vector<Segment> segments_;

    // Bin 的 index of every point
    std::vector<std::pair<int, int>> bin_index_;

    // 2D cooordinates (d, z) of every point in its respective segment
    std::vector<Bin::MinZPoint> segment_coordinates_;

    // 记录每个 bin 中使用的是那个点的索引
    // std::vector<std::vector<int>> bin2PointIdx_;

    // 计数用的
    std::atomic<int> count_;
    // 发布线段用的
    ros::Publisher marker_pub;
    visualization_msgs::Marker line_list;   
    visualization_msgs::Marker point_cmp;
    visualization_msgs::Marker line_res_vis;

    // 分割函数   19423264
    void segment(const PointCloud & cloud, std::vector<int> * segmentation);

    void insertPoints(const PointCloud & cloud);

    void insertPointThread(const PointCloud & cloud, const size_t start_idx, const size_t end_idx);

    void visualizeLine(const PointCloud & cloud, std::list<PointLine> &lines);

    geometry_msgs::Point minZPointTo3d(const Bin::MinZPoint & mzpoint, const double & angle);
    geometry_msgs::Point minZPointTo3d(const double & d, const double & angle);

    void printParams();

    void getLines(std::list<PointLine> *lines);

    void lineFitThread(const unsigned int & start_idx,
                       const unsigned int & end_idx,
                       std::list<PointLine> *lines,
                       std::mutex *lines_mutex);

    /* 后面添加的 */
    void updateGround();
    
    void updateGroundThread(const size_t & start_idx, const size_t & end_idx);

    void applayMedianFilter();
    
    void outlierFilter();

    void outlierFilterThread(const size_t & start_idx, const size_t & end_idx);

    void groundAndElevated(const PointCloud & cloud, std::vector<int> * segmentation);

    void groundAndElevatedThread(const PointCloud & cloud,
                                std::vector<int> * segmentation,
                                unsigned int start_idx,
                                unsigned int end_idx
                                );
    
};

// class rvizPublisher
// {
// public:
//     rvizPublisher(){};
//     rvizPublisher();

// public:
//     void pbCircle(ros::Publish<sensor_msgs::Marker> & pub, const double & radia, const PointCloud & cloud);
// }

#endif /*GROUNDREMOVE*/

