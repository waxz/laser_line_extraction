//
// Created by waxz on 8/21/18.
//
#include <laser_line_extraction/line_detector.h>






int main(int argc, char **argv)
{

    ROS_DEBUG("Starting line_extraction_node.");

    ros::init(argc, argv, "line_detector_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    line_extraction::SimpleShelfDetector sd_(nh,nh_local);


    double frequency;
    nh_local.param<double>("frequency", frequency, 25);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);

    time_util::Timer t;
    while (ros::ok())
    {
        t.start();
        sd_.detect();
        t.stop();
        ROS_INFO("full time %.4f",t.elapsedSeconds());
        rate.sleep();
    }
    return 0;
}

