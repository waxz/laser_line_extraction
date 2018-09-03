//
// Created by waxz on 8/23/18.
//

#include <cpp_utils/geometry.h>
#include <cpp_utils/listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cpp_utils/threading.h>
//get triangle_pose and odom
//combine a triangle_odom

class OdomPub{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    rosnode::Listener listener_;
    string inputPoseTopic_;
    string realOdomTopic_;
    string outOdomTopic_;

    double cov_x_;
    double cov_y_;

    double cov_yaw_;
    ros::Publisher outOdomPub_;
    nav_msgs::Odometry odom_;

    std::shared_ptr<nav_msgs::Odometry> latestInOdom_;
    std::shared_ptr<geometry_msgs::PoseStamped> latestInPose_;

    // threading
    threading_util::ThreadClass<tf::StampedTransform> tfthreadClass_;

    threading_util::Func_tfsync<tf::StampedTransform> tfThread_;

public:
    OdomPub(ros::NodeHandle nh, ros::NodeHandle nh_private):
            nh_(nh),
            nh_private_(nh_private),
            listener_(nh_,nh_private_),
            tfThread_(50)
    {

        inputPoseTopic_ = "triangle_pose";
        realOdomTopic_ = "odom";
        outOdomTopic_ = "triangle_odom";

        nh_private_.param("cov_x",cov_x_,0.05);
        nh_private_.param("cov_y",cov_y_,0.1);
        nh_private_.param("cov_yaw",cov_yaw_,0.1);

        outOdomPub_ = nh_.advertise<nav_msgs::Odometry>(outOdomTopic_,1);

        auto res = listener_.createSubcriber<nav_msgs::Odometry>(realOdomTopic_,1);

        latestInOdom_ = std::get<0>(res);


        auto res2 = listener_.createSubcriber<geometry_msgs::PoseStamped>(inputPoseTopic_,1);
        latestInPose_ = std::get<0>(res2);

        //threading
        tfthreadClass_.setTarget(tfThread_);
//        tfthreadClass_.start();


    }
    void run(){
        bool getpose = listener_.getOneMessage(inputPoseTopic_,-1);

        if(!getpose){
//            ROS_INFO_STREAM("no "<<inputPoseTopic_);

            return;

        }
        bool getodom = listener_.getOneMessage(realOdomTopic_,-1);
        if(!getodom){
//            ROS_INFO_STREAM("no "<<realOdomTopic_);

            return;

        }

        odom_ = *latestInOdom_;

        odom_.header.frame_id = "base_triangle";
        odom_.child_frame_id = "base_link";
        odom_.pose.pose = latestInPose_.get()->pose;
        odom_.pose.covariance[0] = cov_x_;
        odom_.pose.covariance[7] = cov_y_;
        odom_.pose.covariance[35] = cov_yaw_;
        odom_.header.stamp = ros::Time::now();

        outOdomPub_.publish(odom_);
//        ROS_INFO_STREAM("pub odom "<<odom_);


    }

};



int main(int argc, char **argv){
    ros::init(argc,argv,"line_detector_ekf_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    double rate;
    nh_private.param("rate",rate,20.0);
    ros::Rate r(rate);



    OdomPub op(nh,nh_private);

    while (ros::ok()){

        op.run();


        r.sleep();
    }

}
