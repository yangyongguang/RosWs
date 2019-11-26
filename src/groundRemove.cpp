#include "groundRemove.h"

GroundSegmentation::GroundSegmentation(ros::Publisher & line_pub, const GroundSegmentationParams & params):
                        params_(params),
                        segments_(params_.n_segments, Segment(params_.n_bins,
                                          params_.max_slope,
                                          params_.max_error_square,
                                          params_.long_threshold,
                                          params_.max_long_height,
                                          params_.max_start_height,
                                          params_.sensor_height,
                                          params_.r_max_bin,
                                          params_.r_min_bin,
                                          params_.tHmin,
                                          params_.tHmax,
                                          params_.tHDiff,
                                          params_.hSensor,
                                          params.min_split_dist))
                        // bin2PointIdx_(params_.n_segments, std::vector<int> (params_.n_bins, -1))
{
    // 在使用 atiomic 的时候，初始化全部在 : 后实现， 不再括号内部实现， 原因还不知道
    // Segment Seg(params_.n_bins,
    //             params_.max_slope,
    //             params_.max_error_square,
    //             params_.long_threshold,
    //             params_.max_long_height,
    //             params_.max_start_height,
    //             params_.sensor_height);
    std::atomic_init(&count_, 0);
    marker_pub = line_pub;
    //########################################################################
    
    // line_list.action = visualization_msgs::Marker::ADD;
    // line_list.pose.orientation.w = 1.0;
    // line_list.id = 2;
    // line_list.scale.x = 0.1; // 线段粗细

    // line_list.type = visualization_msgs::Marker::LINE_LIST;
    // // color
    // line_list.color.a = 1.0;
    // line_list.color.r = 1.0;
    //########################################################################
}

class SegmentaionNode
{
public:
    SegmentaionNode(ros::NodeHandle &nh,
    const std::string & ground_topic,
    const std::string & obstacle_topic,
    const std::string & line_topic,
    const GroundSegmentationParams &params,
    const bool &latch = false):params_(params)
    {
        // nh_ = nh;
        ground_pub_ = nh.advertise<sensor_msgs::PointCloud>(ground_topic, 1, latch);
        obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud>(obstacle_topic, 1, latch);
        line_topic_ = nh.advertise<visualization_msgs::Marker>(line_topic, 1, latch);        
    }

    void scanCallBack(const sensor_msgs::PointCloud2 cloud2)
    {
        sensor_msgs::PointCloud cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);
        //std::cout << "point size :" << cloud.points.size() << std::endl;
        // code start from here

        GroundSegmentation segmenter(line_topic_, params_);

        // labels all cloud points to be ground of not
        std::vector<int> labels;

        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        /////////////////////////////////////////////////////////////
        segmenter.segment(cloud, &labels);   
        sensor_msgs::PointCloud ground_cloud, obstacle_cloud;
        ground_cloud.header = cloud.header;
        obstacle_cloud.header = cloud.header;

        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            if (labels[i] == 1)
                ground_cloud.points.push_back(cloud.points[i]);
            else
                obstacle_cloud.points.push_back(cloud.points[i]);
        }
        
        ground_pub_.publish(ground_cloud);
        obstacle_pub_.publish(obstacle_cloud);
        // /////////////////////////////////////////////////////////////
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms = end - start;
        std::cout << "Done Took " << fp_ms.count() << " ms" << std::endl;
        // code end here
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher ground_pub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher line_topic_;
    GroundSegmentationParams params_;


};
// 插入点云

// void rvizPublisher::pbCircle(ros::Publish<sensor_msgs::Marker> & pub, const double & radia, const PointCloud & cloud)
// {
//     pub.header = cloud.header;
//     geometry_msgs::
// }

void GroundSegmentation::insertPoints(const PointCloud & cloud)
{
    // printParams();
    std::vector<std::thread> threads(params_.n_threads);
    int num_pre_thread = cloud.points.size() / params_.n_threads; 

    // std::cout << "params_.n_threads " << params_.n_threads << std::endl;  
    for (int idx = 0; idx < params_.n_threads - 1; ++idx)
    {
        // std::cout << "thread in for :" << idx << std::endl;
        int start_idx = idx * num_pre_thread;
        int end_idx = (idx + 1) * num_pre_thread - 1;
        threads[idx] = std::thread(&GroundSegmentation::insertPointThread, 
                                    this, cloud, start_idx, end_idx);        
    }

    const size_t start_idx = num_pre_thread * (params_.n_threads - 1);
    const size_t end_idx = cloud.points.size() - 1;
    threads[params_.n_threads - 1] = std::thread(&GroundSegmentation::insertPointThread, 
                                    this, cloud, start_idx, end_idx);

    // 等待所有的线程结束任务
    for (auto it = threads.begin(); it != threads.end(); ++it)
    {
        it->join();
    }
    
    // 判断每个点的是否有点
    // #########################################################
    // int num_hasPoint = 0, num_bin_hasPoint = 0;
    // for (int seg_idx = 0; seg_idx < params_.n_segments; ++seg_idx)
    // {
    //     for (int bin_idx = 0; bin_idx < params_.n_bins; ++bin_idx)
    //     {
    //         Bin currBin = segments_[seg_idx][bin_idx];
    //         if (currBin.hasPoint())
    //             num_hasPoint++;   
    //         if (segments_[seg_idx][bin_idx].hasPoint())
    //             num_bin_hasPoint++;

    //     }        
    // }

    // std::cout << "num_haspoint " << num_hasPoint << std::endl;      
    // std::cout << "num_bin_hasPoint " << num_bin_hasPoint << std::endl;    
    // getchar();


    // ######################################################
    // if (params_.visualize)
    // {
        
    //     visualizeLine(cloud);

    // }
    // std::cout << "tatal num points: " << cloud.points.size() << std::endl;
    // std::cout << "total has been processed:" << count_ << std::endl;
}

// 注意添加点的时候， 点是不能均匀分布的， 所以需要最后一个线程多分一点点云
void GroundSegmentation::insertPointThread(const PointCloud & cloud, 
                        const size_t start_idx, 
                        const size_t end_idx)
{
    // detaX = (r_max_bin - r_min_bin) / (n_bins - 1)
    // bin_idx = (range - r_min + r_min_bin + 2 * detaX) / (r_min_bin + detaX);
    // std::cout << "thread id is :" << std::this_thread::get_id() << std::endl;
    // double detaX = (params_.r_max_bin - params_.r_min_bin) / (params_.n_bins - 1);
    const double segment_step = 2 * M_PI / params_.n_segments;
    const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
             / params_.n_bins;
    const double r_min = sqrt(params_.r_min_square);

    // double a = detaX / 2;
    // double b = (params_.r_min_bin - detaX / 2);

    // 处理每个线程的点
    for (int idx = start_idx; idx <= end_idx; ++idx)
    {
        // count_++;
        PointXYZ point(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);
        const double range_square = point.x * point.x + point.y * point.y;
        const double range = sqrt(range_square);
        if (range_square < params_.r_max_square && range_square > params_.r_min_square)
        {
            const double angle = std::atan2(point.y , point.x);
            const unsigned int bin_idx = (range - r_min) / bin_step;
            // double c = r_min - params_.r_min_bin - range;
            // unsigned int bin_idx = (-b + sqrt(b * b  - 4 * a * c)) / (2 * a);
            assert(bin_idx >= 0 && bin_idx < params_.n_bins);
            // unsigned int bin_idx = (range - r_min + params_.r_min_bin + 2 * detaX) /
            //                             (params_.r_min_bin + detaX);
            // if (bin_idx >= params_.n_bins)
            // {
            //     std::cout << "range : " << range << "\n";
            //     std::cout << "detaX : " << detaX << "\n";
            //     std::cout << "bin_idx : " << bin_idx << std::endl;
            //     bin_idx = params_.n_bins - 1;
            // }
            const unsigned int segment_idx = (angle + M_PI) / segment_step;
            bin_index_[idx] = std::make_pair(segment_idx, bin_idx);
            segments_[segment_idx == params_.n_segments? 0:segment_idx][bin_idx].addPoint(range, point.z);
            // std::cout << "segment_idx " << segment_idx <<"\n" << "bin_idx " << bin_idx << "\n";
            // std::cout << "segments_[segment_idx][bin_idx].hasPoint() " <<
            //              segments_[segment_idx][bin_idx].hasPoint() << "\n";
            // bin2PointIdx_[segment_idx][bin_idx] = idx;
        }    
        else
        {
            bin_index_[idx] = std::make_pair(-1, -1); 
        }
        // 保存每个点的 range 和 point.z 避免重复计算， 空间换时间策略
        segment_coordinates_[idx] = Bin::MinZPoint(range, point.z);            
    }    
    
}


// 可视化线段， 原始线段， 还未处理的线段
void GroundSegmentation::visualizeLine(const PointCloud & cloud, std::list<PointLine> &lines)
{
    // std::cout << "visualzieLine lines has " << 2 * lines.size() << " points\n";
    //line_list.header.frame_id = "velodyne64";
    line_list.header = cloud.header;
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.scale.x = 0.1; // 线段粗细

    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // color
    line_list.color.a = 1;
    line_list.color.r = 1.0;

    // line_list.header.stamp = ros::Time::now();
    line_list.points.clear();
    // std::cout << "before line_list has point " << line_list.points.size() << std:: endl;    

    // 用来做对比的点的位置
    point_cmp.header = cloud.header;
    point_cmp.ns = "lines";
    point_cmp.action = visualization_msgs::Marker::ADD;
    point_cmp.pose.orientation.w = 1.0;
    point_cmp.scale.x = 0.1;
    point_cmp.scale.y = 0.1;

    point_cmp.color.a = 1.0;
    point_cmp.color.b = 1.0;

    point_cmp.type = visualization_msgs::Marker::LINE_LIST;

    // 可视化地面线段
    line_res_vis.header = cloud.header;
    line_res_vis.ns = "lines";
    line_res_vis.action = visualization_msgs::Marker::ADD;
    line_res_vis.pose.orientation.w = 1.0;
    line_res_vis.scale.x = 0.1;
    line_res_vis.scale.y = 0.1;

    line_res_vis.color.a = 1.0;
    line_res_vis.color.g = 1.0;

    line_res_vis.type = visualization_msgs::Marker::LINE_LIST;
    // 结束可视化地面线段
    const double seg_step = 2 * M_PI / params_.n_segments;
    double detaX = (params_.r_max_bin - params_.r_min_bin) / (params_.n_bins - 1);
    for (int bin_idx = 0; bin_idx < params_.n_bins; ++bin_idx)
    {
        for (int seg_idx = 0; seg_idx <= 1; ++seg_idx)
        {
            double angle = -M_PI + seg_step * seg_idx;
            double d = sqrt(params_.r_min_square) + params_.r_min_bin * (bin_idx - 1) +
                        bin_idx * (bin_idx - 1) * detaX / 2;
            geometry_msgs::Point pt = minZPointTo3d(d, angle);
            point_cmp.points.push_back(pt);
        }
    }
    marker_pub.publish(point_cmp);
    // #############################################
    for (int seg_idx = 0; seg_idx < params_.n_segments; ++seg_idx)
    // for (int seg_idx = 0; seg_idx < 2; ++seg_idx)
    {
        //  -M_PI + seg_step/2 + seg_step * start_index;
        // double angle = seg_step * seg_idx;   // 不知道是不对的
        double angle = -M_PI + seg_step * seg_idx;
        // line_list.points.clear();  
        // Bin preBin = segments_[seg_idx][0];
        for (int bin_idx = 1; bin_idx < params_.n_bins; ++bin_idx)
        {                
            // if (!segments_[seg_idx][bin_idx].hasPoint())
            //     continue;
            // Bin currBin = segments_[seg_idx][bin_idx];
            if (!segments_[seg_idx][bin_idx - 1].hasPoint() || 
                !segments_[seg_idx][bin_idx].hasPoint())
            {
                // std::cout << "preBin " << segments_[seg_idx][bin_idx - 1].hasPoint() << std::endl;
                // std::cout << "currBin " << segments_[seg_idx][bin_idx].hasPoint() << std::endl;
                continue;
            }
            // std::cout << "corret condition " << std::endl;
            // add first point
            Bin::MinZPoint mzpoint = segments_[seg_idx][bin_idx].getMinZPoint();
            geometry_msgs::Point point = minZPointTo3d(mzpoint, angle);
            line_list.points.push_back(point);

            // add second point
            Bin::MinZPoint prepoint = segments_[seg_idx][bin_idx - 1].getMinZPoint();
            point = minZPointTo3d(prepoint, angle);
            line_list.points.push_back(point);
            // preBin = currBin;
        }        
    }
    // 可视化 insert Point 的点
    // marker_pub.publish(line_list);

    // 可视化线段结果
    /*
    std::list<PointLine> &lines
    typedef std::pair<geometry_msgs::Point, geometry_msgs::Point> PointLine;
    */

   for (auto it = lines.begin(); it != lines.end(); ++it)
   {
       line_res_vis.points.emplace_back(it->first);
       line_res_vis.points.emplace_back(it->second);    
   }

    marker_pub.publish(line_res_vis);
    // marker_pub.publish(line_list);
    // std::cout << "after line res vis has point " << line_res_vis.points.size() << " points " << std:: endl;

}

geometry_msgs::Point GroundSegmentation::minZPointTo3d(const Bin::MinZPoint & mzpoint, const double & angle)
{
    geometry_msgs::Point point;
    point.x = cos(angle) * mzpoint.d;
    point.y = sin(angle) * mzpoint.d;
    point.z = mzpoint.z;
    return point;
}

geometry_msgs::Point GroundSegmentation::minZPointTo3d(const double & d, const double & angle)
{
    geometry_msgs::Point point;
    point.x = cos(angle) * d;
    point.y = sin(angle) * d;
    point.z = 0;
    return point;
}

void GroundSegmentation::printParams()
{
    std::cout << "params_.line_search_angle : " << params_.line_search_angle << std::endl;
    std::cout << "params_.long_threshold : " << params_.long_threshold << std::endl;
    std::cout << "params_.max_dist_to_line : " << params_.max_dist_to_line << std::endl;
    std::cout << "params_.max_error_square : " << params_.max_error_square << std::endl;
    std::cout << "params_.max_long_height : " << params_.max_long_height << std::endl;
    std::cout << "params_.max_slope : " << params_.max_slope << std::endl;
    std::cout << "params_.max_start_height : " << params_.max_start_height << std::endl;
    std::cout << "params_.n_bins : " << params_.n_bins << std::endl;
    std::cout << "params_.n_segments : " << params_.n_segments << std::endl;
    std::cout << "params_.n_threads : " << params_.n_threads << std::endl;
    std::cout << "params_.r_max_square : " << params_.r_max_square << std::endl;
    std::cout << "params_.r_max_bin : " << params_.r_max_bin << std::endl;
    std::cout << "params_.r_min_bin : " << params_.r_min_bin<< std::endl;
    std::cout << "params_.sensor_height : " << params_.sensor_height << std::endl;
    std::cout << "params_.visualize : " << params_.visualize << std::endl;
}

// 获取所得到的线段， 为了后面可视化调试使用
void GroundSegmentation::getLines(std::list<PointLine> *lines)
{
    bool visulize = lines;
    std::mutex line_mutex;
    // 多线程
    std::vector<std::thread> thread_vec(params_.n_threads);

    std::chrono::high_resolution_clock::time_point start_fit = std::chrono::high_resolution_clock::now();
    for (int idx = 0; idx < params_.n_threads; ++idx)
    {
        const unsigned int start_idx = params_.n_segments / params_.n_threads * idx;
        const unsigned int end_idx = params_.n_segments / params_.n_threads * (idx + 1);
        thread_vec[idx] = std::thread(&GroundSegmentation::lineFitThread, this, 
                                                    start_idx, end_idx,lines, &line_mutex);
    }

    // 等待所有线程完成任务
    for(auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }

    std::chrono::high_resolution_clock::time_point end_fit = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end_fit - start_fit;
    // std::cout << "getLines took about " << fp_ms.count() << " ms" << std::endl;
    // std::cout << "getLines has " << lines->size() << " points\n";
}

// 线程分配 线段拟合
void GroundSegmentation::lineFitThread(const unsigned int & start_idx,
                       const unsigned int & end_idx,
                       std::list<PointLine> *lines,
                       std::mutex *lines_mutex)
{
    //
    const bool visualize = lines;
    const double seg_step = 2 * M_PI / params_.n_segments;
    double angle = -M_PI + start_idx * seg_step;
    for (int idx = start_idx; idx < end_idx; ++idx)
    {
        // segments_[idx].fitSegmentLines();   
        // printf("new segment \n\n\n\n");    
        segments_[idx].splitAndMerger(); 
        if (visualize)
        {            
            std::list<Segment::Line> segment_lines;
            segments_[idx].getLines(&segment_lines);
            // std::cout << "after segments_[idx].fitSegmentLines lines has  " << segment_lines.size() << " points\n";
            for (auto line_iter = segment_lines.begin(); line_iter != segment_lines.end(); ++line_iter)
            {
                geometry_msgs::Point start = minZPointTo3d(line_iter->first, angle);
                geometry_msgs::Point end = minZPointTo3d(line_iter->second, angle);
                lines_mutex->lock();  // 在添加元素的时候， 调用容器， 最好加上 lock 防止出错
                lines->emplace_back(start, end); // 后续可以绘制在 rviz 中使用
                lines_mutex->unlock();
            }
        }

        angle += seg_step;
    }
}

void GroundSegmentation::updateGround()
{
    std::vector<std::thread> thread_vec(params_.n_threads);
    size_t start_idx = 0, end_idx = 0;
    for (int idx = 0; idx < params_.n_threads; ++idx)
    {
        start_idx = params_.n_segments / params_.n_threads * idx;
        end_idx = params_.n_segments / params_.n_threads * (idx + 1);
        thread_vec[idx] = std::thread(&GroundSegmentation::updateGroundThread, this, start_idx, end_idx);
    }

    for (std::vector<std::thread>::iterator it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}

void GroundSegmentation::updateGroundThread(const size_t & start_idx, const size_t & end_idx)
{
    for (int idx = start_idx; idx < end_idx; ++idx)
    {
        segments_[idx].updateHeightAndGround();
    }
}

void GroundSegmentation::outlierFilter()
{
    std::vector<std::thread> thread_vec(params_.n_threads);
    size_t start_idx = 0, end_idx = 0;
    for (int idx = 0; idx < params_.n_threads; ++idx)
    {
        start_idx = params_.n_segments / params_.n_threads * idx;
        end_idx = params_.n_segments / params_.n_threads * (idx + 1);
        thread_vec[idx] = std::thread(&GroundSegmentation::outlierFilterThread, this, start_idx, end_idx);
    }

    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}

void GroundSegmentation::outlierFilterThread(const size_t & start_idx, const size_t & end_idx)
{
    for (int idx = start_idx; idx < end_idx; ++idx)
    {
        segments_[idx].outlierFilter();
    }
}

void GroundSegmentation::applayMedianFilter()
{
    for (int segIdx = 1; segIdx < params_.n_segments - 1; ++segIdx)
    {
        for (int binIdx = 1; binIdx < params_.n_bins - 1; ++binIdx)
        {
            // printf("%d, %d\n", segIdx, binIdx);
            if (segments_[segIdx][binIdx + 1].isThisGround()&&
                segments_[segIdx][binIdx - 1].isThisGround() &&
                segments_[segIdx + 1][binIdx].isThisGround() &&
                segments_[segIdx - 1][binIdx].isThisGround())
            {
                    std::vector<double> sur{segments_[segIdx][binIdx + 1].getHeight(),
                                            segments_[segIdx][binIdx + 1].getHeight(),
                                            segments_[segIdx + 1][binIdx].getHeight(),
                                            segments_[segIdx - 1][binIdx].getHeight()};
                    std::sort(sur.begin(), sur.end());
                    double m1 = sur[1];
                    double m2 = sur[2];
                    double median = (m1 + m2) / 2;
                    segments_[segIdx][binIdx].updateHeight(median);
                    // 将其设置为 地面点 跟新其高度， 如果条件成熟的话
                    segments_[segIdx][binIdx].updateGround();
            }                
        }
    }
}

void GroundSegmentation::groundAndElevated(const PointCloud & cloud, std::vector<int> * segmentation)
{

    int num_pre_thread = cloud.points.size() / params_.n_threads;
    std::vector<std::thread> thread_vec(params_.n_threads);
    // std::mutex mtx, mtx2;
    for (unsigned int idx = 0; idx < params_.n_threads - 1; ++idx)
    {
        unsigned int start_idx = idx * num_pre_thread;
        unsigned int end_idx = (idx + 1) * num_pre_thread;
        thread_vec[idx] = std::thread(&GroundSegmentation::groundAndElevatedThread, this, 
                cloud, segmentation, start_idx, end_idx);    
    }
    // printf("groundAndElevated\n");
    unsigned int start_idx = (params_.n_threads - 1) * num_pre_thread;
    unsigned int end_idx = cloud.points.size() - 1;
    thread_vec[params_.n_threads - 1] = std::thread(&GroundSegmentation::groundAndElevatedThread, this, 
                cloud, segmentation, start_idx, end_idx);
    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}

void GroundSegmentation::groundAndElevatedThread(const PointCloud & cloud,
                                std::vector<int> *segmentation,
                                unsigned int start_idx,
                                unsigned int end_idx
                                    )
{
    // printf("groundAndElevatedThread\n");
    for (int idx = start_idx; idx < end_idx; ++idx)
    {
        // printf("Thread %d\n",idx);
        const int binIdx = bin_index_[idx].second;
        const int segIdx = bin_index_[idx].first;
        if (binIdx == -1) continue;
        // printf("binIdx : %d, segIdx : %d\n", binIdx, segIdx);
        // 当前所在的 bin 是否是地面点
        if (segments_[segIdx][binIdx].isThisGround())
        {
            double hGround = segments_[segIdx][binIdx].getHGround();
            if (cloud.points[idx].z < (hGround + 0.25))
            {
                // 地面 1 
                // printf("isGround : segmentation")
                segmentation->at(idx) = 1;
            }
            else
            {
                segmentation->at(idx) = 0;
            }            
        }
    }
}
// mian segmentation function
void GroundSegmentation::segment(const PointCloud & cloud, std::vector<int> * segmentation)
{
    // std::cout << "Segmentation cloud with " << cloud.points.size() << " Points." << std::endl;
    // 初始化
    segmentation->clear();
    int num_points = cloud.points.size();
    segmentation->resize(num_points, 0);
    segment_coordinates_.resize(num_points);
    bin_index_.resize(num_points);

    // 分配点云 insertPoint
    insertPoints(cloud);   
    int groundRemoveID = 1;
    if (groundRemoveID == 0)
    {   
        // std::cout << "inserPoints finished \n" ;
        /*接下来是使用 很简单的梯度方法， 来自于
        3D-LIDAR Muliti Object Tracking for Autonomous Driving*/
        /////////////////////////////////////////////////////  去地开始 //////////////////////////    
        updateGround();
        // std::cout << "updateGround finished \n";
        applayMedianFilter();
        // std::cout << "applayMedianFilter finished \n";
        outlierFilter();
        // std::cout << "outlierFilter finished \n";
        groundAndElevated(cloud, segmentation);
        // std::cout << "groundAndElevated finished \n";
        /////////////////////////////////////////////////////  去地结束 //////////////////////////}
    }
    else
    {   
        // 获取线段 
        /////////////////////////////////////////////////// 使用 getline 的代码 ////////////////////
        /*时间超时了很多*/  
        std::list<PointLine> lines;
        if (params_.visualize) getLines(&lines);
        else getLines(nullptr);    
        // 可视化部分结果
        if (params_.visualize) visualizeLine(cloud, lines);

        // 给点云分类
   }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"ground_segmentation");
    ros::NodeHandle nh("~");

    GroundSegmentationParams params;

    std::string input_topic;
    input_topic = "/apollo/sensor/velodyne64/PointCloud2";
    ros::Subscriber cloud_sub;
    std::string ground_topic, obstack_topic, line_topic;
    ground_topic = "ground_topic";
    obstack_topic = "obstack_topic";
    line_topic = "line_topic";
    bool latch = false;
    SegmentaionNode node(nh, ground_topic, obstack_topic, line_topic, params, latch);
    cloud_sub = nh.subscribe(input_topic, 1, &SegmentaionNode::scanCallBack, &node);
    ros::spin();
    std::cout << "code finished " << std::endl;
    return 0;
}


