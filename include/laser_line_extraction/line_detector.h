//
// Created by waxz on 8/21/18.
//

#ifndef LASER_LINE_EXTRACTION_LINE_DETECTOR_H
#define LASER_LINE_EXTRACTION_LINE_DETECTOR_H
#include "laser_line_extraction/line_extraction_ros.h"
#include <cpp_utils/listener.h>
#include <cpp_utils/time.h>
#include <cpp_utils/geometry.h>
#include <cpp_utils/container.h>
#include <cpp_utils/levmarq.h>
#include <cpp_utils/threading.h>
#include <ros/console.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>

#include <string>

// sort points in triangle
struct AngleCompare{
    bool operator()(const type_util::Point2d &v1, const type_util::Point2d &v2){
        return atan2(v1.y,v1.x) > atan2(v2.y,v2.x);

    }
};

namespace line_extraction{
    // select pair of lines, match triangle condition
    struct SimpleShape{
        std::vector<line_extraction::Line> lines;
        type_util::Point2d intersect;
        double intersect_angle;
    };





// get laser data
// mask valid data
// extract line
// choose valid data
    class LineSegmentDetector:line_extraction::LineExtractionROS {
    protected:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // topic sub
        rosnode::Listener listener;

        //shared data
        std::shared_ptr<sensor_msgs::LaserScan> scan_data_;


        // parameter
        double range_min_;
        double max_range_;
        double angle_min_;
        double angle_max_;

        bool debug_mode_;


        //result
        std::vector<line_extraction::Line> lines_;
        void processData();
        void initParam();

    public:
        LineSegmentDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
        bool getLaser(sensor_msgs::LaserScan &scan);
        void pubMarkers(std::vector<line_extraction::Line> lines);
        // get lines(0) or cluster(1)
        std::vector<line_extraction::Line> getLines(int mode = 0);
    };


    // detect only one triangle
// given relative position and shape

// judge a line segment as triangle;
// rule 1: length [0.05 , 0.2]
// rule 2: angle [ -1, 1]
// rule 3: distance [0.1, 4]
// rule 5: pair angle [ pi*100/180, pi*150/180]
// rule 6: pair intersection and distance to each end [0.15,0.25]
// rule 7: pair intersection angle
    class SimpleTriangleDetector {
    protected:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        line_extraction::LineSegmentDetector lsd_;

        // debug
        geometry_msgs::PoseStamped targetPose_;
        geometry_msgs::PoseArray targetPoints_;

        ros::Publisher targetPub_;
        ros::Publisher pointsPub_;

        // cache
        sensor_msgs::LaserScan latestScan_;

        valarray<float> cache_cos_;
        valarray<float> cache_sin_;
        valarray<float> cache_angle_;

        // param
        double min_segLength_;
        double max_segLength_;
        double min_segAngle_;
        double max_segAngle_;
        double min_segDist_;
        double max_segDist_;

        double min_shapeAngle_;
        double max_shapeAngle_;
        double min_pairInterLength_;
        double max_pairInterLength_;
        double min_pairInterAngle_;
        double max_pairInterAngle_;

        double min_grow_dist_;
        double min_gap_dist_;

        int filter_window_;

        double max_fit_error_;

        double triangle_direction_;

        // method
        void initParams();

        void cacheData();
        bool fitModel(vector<type_util::Point2d> & pointsInModel,double x0, double x1, double x2,geometry_msgs::PoseStamped & ModelPose);

    public:
        SimpleTriangleDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
        vector<geometry_msgs::PoseStamped> detect();



    };


    struct smoothPose{
        int cnt_;
        int num_;
        valarray<double> xs_;
        valarray<double> ys_;
        valarray<double> yaws_;

        double mean_x_;
        double mean_y_;
        double mean_yaw_;

        smoothPose(int cnt = 1 ):
                cnt_(cnt),xs_(cnt), ys_(cnt),yaws_(cnt_){
            num_ = 0;
        }

        void update(double x, double y, double yaw){
            if(num_ == cnt_-1){
                xs_ = xs_.cshift(1);
                ys_ = ys_.cshift(1);
                yaws_ = yaws_.cshift(1);
            }
            xs_[num_] = x;
            ys_[num_] = y;
            yaws_[num_] = yaw;

            if(num_ < cnt_-1){
                num_++;

            }
        }
        void getMean(){
            mean_x_ = xs_.sum()/xs_.size();
            mean_y_ = ys_.sum()/ys_.size();
            mean_yaw_ = yaws_.sum()/yaws_.size();

        }

        bool full(){
            return num_==cnt_-1;
        }

        void clear(){
            smoothPose newpose(this->cnt_);
            this->xs_ = newpose.xs_;
            this->ys_ = newpose.ys_;
            this->yaws_ = newpose.yaws_;
            this->num_ = 0;

        }

    };

    class TargetPublish {
    protected:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // detector
        line_extraction::SimpleTriangleDetector sd_;
        rosnode::Listener listener_;

        // pose pub
        ros::Publisher fake_pose_pub_;

        string base_frame_id_;

        string laser_frame_id_;
        string target_framde_id_;


        string fake_pose_topic_;

        string cmd_topic_;

        std::shared_ptr<std_msgs::String> cmd_data_ptr_;

        bool running_;


        ros::Time lastOkTime_;
        int expire_sec_;


        bool broadcast_tf_;

        bool pub_pose_;

        tf::Transform baseToLaser_tf_;
        geometry_msgs::PoseStamped triangle_pose_;
        tf::StampedTransform stampedTransform_;

        line_extraction::smoothPose smoothPose_;


        // threading
        threading_util::ThreadClass<geometry_msgs::PoseStamped> pubthreadClass_;
        threading_util::ThreadClass<tf::StampedTransform> tfthreadClass_;

        threading_util::Func_pub<geometry_msgs::PoseStamped> pubThread_;
        threading_util::Func_tfb<tf::StampedTransform> tfThread_;
    public:
        TargetPublish(ros::NodeHandle nh, ros::NodeHandle nh_private);
        void publish();


    };




}








#endif //LASER_LINE_EXTRACTION_LINE_DETECTOR_H
