#include "laser_line_extraction/line_extraction_ros.h"
#include <cpp_utils/listener.h>
#include <cpp_utils/time.h>
#include <ros/console.h>

#include <string>

// get laser data
// mask valid data
// extract line
// choose valid data
class LineSegmentDetector:line_extraction::LineExtractionROS{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // topic sub
    rosnode::Listener listener;

    //shared data
    std::shared_ptr<sensor_msgs::LaserScan> scan_data_;


    // parameter
    double range_min_;
    double range_max_;
    double angle_min_;
    double angle_max_;


    //result
    std::vector<line_extraction::Line> lines;




    // method
    void initParam(){
        nh_private_.param("max_range",range_max_,4.0);
        nh_private_.param("min_angle",angle_min_,-0.5*M_PI);
        nh_private_.param("max_angle",angle_max_,0.5*M_PI);


    };

    void processData(){
        if (!data_cached_)
        {
            auto scan = *scan_data_;
            sensor_msgs::LaserScan::ConstPtr  scan_ptr(&scan);
            cacheData(scan_ptr);
            data_cached_ = true;
        }

        std::vector<double> scan_ranges_doubles((*scan_data_).ranges.begin(), (*scan_data_).ranges.end());
        line_extraction_.setRangeData(scan_ranges_doubles);
    }


public:
    LineSegmentDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
    nh_(nh),
    nh_private_(nh_private),
    line_extraction::LineExtractionROS(nh, nh_private),
    listener(nh, nh_private){
        initParam();


        auto res = listener.createSubcriber<sensor_msgs::LaserScan>(this->scan_topic_,1);

        scan_data_ = std::get<0>(res);

    }
    std::vector<line_extraction::Line> getLines(){
        lines.clear();

        bool getMsg = listener.getOneMessage(this->scan_topic_,0.1);
        if (!getMsg){
            ROS_ERROR("No scan data !!\n");
            std::cout<<std::endl;

            return lines;
        }
        ROS_ERROR("Get  scan data !!\n");
        std::cout<<std::endl;
        time_util::Timer timer;
        timer.start();


        // process data
        // get ptr from msg
        // https://answers.ros.org/question/196697/get-constptr-from-message/
        boost::shared_ptr<sensor_msgs::LaserScan> scan_ptr_(boost::make_shared<sensor_msgs::LaserScan>(*scan_data_) );

        this->laserScanCallback(scan_ptr_);


        // Extract the lines
        this->line_extraction_.extractLines(lines);
        printf("get lines num = %d",int(lines.size()));
        timer.stop();
        printf("time %.3f\n",timer.elapsedSeconds());


        // Also publish markers if parameter publish_markers is set to true
        if (this->pub_markers_)
        {
            // Populate message
            laser_line_extraction::LineSegmentList msg;
            this->populateLineSegListMsg(lines, msg);
            this->line_publisher_.publish(msg);
            visualization_msgs::Marker marker_msg;
            this->populateMarkerMsg(lines, marker_msg);
            this->marker_publisher_.publish(marker_msg);

        }


        return lines;

    }


};

int main(int argc, char **argv)
{
#if 0
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
#endif
    ROS_DEBUG("Starting line_extraction_node.");

    ros::init(argc, argv, "line_detector_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    LineSegmentDetector line_extractor(nh, nh_local);

    double frequency;
    nh_local.param<double>("frequency", frequency, 25);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);

    // get data
    // if successful
    // extract line

    while (ros::ok())
    {
        line_extractor.getLines();
        rate.sleep();
    }
    return 0;
}

