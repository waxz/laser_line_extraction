//
// Created by waxz on 9/3/18.
//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <cpp_utils/types.h>
#include <cpp_utils/geometry.h>
#include <random>
void getTriangleScan(double x, double y, double yaw, double angle,double sidelen, double angle_increment,double stddev,sensor_msgs::LaserScan &scan,std::default_random_engine &generator) {
    // line1 line2
    type_util::Point2d p1,p2,p0,pz,pr;
    p0.x = x;
    p0.y = y;
    p1.x = p0.x + sidelen*cos(yaw+0.5*angle - M_PI);
    p1.y = p0.y + sidelen*sin(yaw+0.5*angle- M_PI);
    p2.x = p0.x + sidelen*cos(yaw-0.5*angle- M_PI);
    p2.y = p0.y + sidelen*sin(yaw-0.5*angle- M_PI);

    pz.x = 0;
    pz.y = 0;



    double angle_min = atan2(p1.y,p1.x);
    double angle_max = atan2(p2.y,p2.x);
    angle_increment = M_PI/(angle_increment*180.0);

    double angle_tmp = -10;
    double angle_turn = atan2(p0.y,p0.x);
    std::vector<type_util::Point2d> points;

    scan.angle_min = angle_min - 0.1;
    scan.angle_max = angle_max + 0.1;
    scan.angle_increment = angle_increment;
    int pointNum = (angle_max - angle_min)/angle_increment;

    scan.ranges.clear();

    double mean = 0.0;
    std::normal_distribution<double> distribution(mean,stddev);
    double noise;
    for(double a = 0.0; a<0.1; a+=angle_increment){
        noise = distribution(generator);

        scan.ranges.push_back( x + 0.2 + noise );

    }

    for (int i=0;i<pointNum;i++){
        angle_tmp = angle_min+i*angle_increment;
        pr.x = cos(angle_tmp);
        pr.y = sin(angle_tmp);
        if(angle_tmp < angle_turn){
            geometry_util::LineLineIntersect(p0,p1,pz,pr,pr);

        }else{
            geometry_util::LineLineIntersect(p0,p2,pz,pr,pr);

        }
        noise = distribution(generator);
        scan.ranges.push_back(sqrt(pr.x*pr.x + pr.y*pr.y) + noise );
    }
    for(double a = 0.0; a<0.1; a+=angle_increment){
        noise = distribution(generator);

        scan.ranges.push_back( x + 0.2 + noise );

    }
    scan.range_min = 0.0;
    scan.range_max = 10.0;
}
int main(int argc, char **argv){
    ros::init(argc,argv,"triangle_sim_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::string scan_topic_ = "scan_filtered";
    ros::Publisher scan_pub_ = nh.advertise<sensor_msgs::LaserScan>(scan_topic_,1);

    double x,y,yaw,angle,sideLen,angleRes,stddev;

    sensor_msgs::LaserScan scan;
    scan.header.frame_id = "base_laser";

    ros::Rate r(25);

    std::default_random_engine generator;

    while (ros::ok()){
        nh_private.param("x",x,1.0);
        nh_private.param("y",y,0.0);
        nh_private.param("yaw",yaw,0.0);
        nh_private.param("angle",angle,M_PI*120.0/180.0);

        nh_private.param("sideLen",sideLen,0.25);
        nh_private.param("angleRes",angleRes,3.0);
        nh_private.param("stddev",stddev,0.0001);

        getTriangleScan(x,y,yaw,angle,sideLen,angleRes,stddev,scan,generator);

        scan.header.stamp = ros::Time::now();
        scan_pub_.publish(scan);

        r.sleep();




    }
}