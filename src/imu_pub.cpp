//
// Created by waxz on 9/4/18.
//

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>


int main(int argc, char **argv){

    ros::init(argc,argv,"imu_pub");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data",1);

    ros::Rate r(20);
    sensor_msgs::Imu imu;
    imu.header.frame_id = "base_imu";
    imu.orientation_covariance[0] = 0.1;
    imu.orientation_covariance[4] = 0.1;
    imu.orientation_covariance[8] = 0.05;


    imu.orientation.w = 1.0;

    while (ros::ok()){

        imu.header.stamp = ros::Time::now();


        imu_pub.publish(imu);

        r.sleep();



    }


}