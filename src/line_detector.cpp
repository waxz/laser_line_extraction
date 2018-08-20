#include "laser_line_extraction/line_extraction_ros.h"
#include <cpp_utils/listener.h>
#include <cpp_utils/time.h>
#include <cpp_utils/geometry.h>
#include <cpp_utils/container.h>
#include <cpp_utils/levmarq.h>
#include <ros/console.h>
#include <geometry_msgs/PoseArray.h>

#include <string>

// get laser data
// mask valid data
// extract line
// choose valid data
class LineSegmentDetector:line_extraction::LineExtractionROS{
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




    // method
    void initParam(){
        nh_private_.param("max_range",max_range_,1.0);
        nh_private_.param("min_angle",angle_min_,-0.5*M_PI);
        nh_private_.param("max_angle",angle_max_,0.5*M_PI);


    };

    void processData(){
        // mask ranges
        // set range > max_range to 0
        auto msg = *scan_data_;

        valarray<float> r = container_util::createValarrayFromVector(msg.ranges);
        r[r>float(max_range_)] = 0.0;

        // window mean filter
#if 0

        decltype(r) rL = r[std::slice(0,r.size()-1,1)];
        decltype(r) rR = r[std::slice(1,r.size()-1,1)];
        auto diff = rR - rL;
        valarray<bool> maskContinous = (diff > 0.001f) && (diff < 0.05f) ;
        valarray<bool> maskGap = (diff > 0.007f)  ;
        valarray<int> ids = container_util::createRangeValarray(maskGap.size(),0);
        valarray<int> maskids = ids[maskGap];
        auto r_filtered = r;
        r_filtered = 0f;
        int widow = 2;

        for(int i=0;i<maskids.size();i++){
            int s,e;

            // select start id and end id
            if(i==0){
                if(maskids[i]>2*widow){

                }

            }else if(i==maskids.size()-1){
                if(maskids[i]>2*widow){

                }
            } else if(i>0 && i < maskids.size()){
                if(maskids[i]>2*widow){

                }
            }
        }

#endif
        msg.ranges = container_util::createVectorFromValarray(r);

        boost::shared_ptr<sensor_msgs::LaserScan> scan_ptr_(boost::make_shared<sensor_msgs::LaserScan>(msg) );

        this->laserScanCallback(scan_ptr_);
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
//        listener.getOneMessage(scan_topic_,-1);


    }
    std::vector<line_extraction::Line> getLines(){
        lines_.clear();

        // todo:debug with block
        bool getMsg;
        if(debug_mode_){
            getMsg = listener.getOneMessage(this->scan_topic_,-1);

        }else{
            getMsg = listener.getOneMessage(this->scan_topic_,0.01);

        }
        if (!getMsg){
            std::cout<<std::endl;

            return lines_;
        }
        std::cout<<std::endl;
        time_util::Timer timer;
        timer.start();


        // process data
        // get ptr from msg
        // https://answers.ros.org/question/196697/get-constptr-from-message/
        processData();



        // Extract the lines
        this->line_extraction_.extractLines(lines_);
        printf("get lines_ num = %d",int(lines_.size()));
        timer.stop();
        printf("time %.3f\n",timer.elapsedSeconds());


        // Also publish markers if parameter publish_markers is set to true
        if (this->pub_markers_)
        {
#if 0
            pubMarkers(lines_);
#endif
        }


        return lines_;

    }

    void pubMarkers(std::vector<line_extraction::Line> lines){
        // Populate message
        ROS_INFO("publish markers!!");
        laser_line_extraction::LineSegmentList msg;
        this->populateLineSegListMsg(lines, msg);
        this->line_publisher_.publish(msg);
        visualization_msgs::Marker marker_msg;
        this->populateMarkerMsg(lines, marker_msg);
        this->marker_publisher_.publish(marker_msg);


    }

    bool getLaser(sensor_msgs::LaserScan &scan){
        scan = *scan_data_;
        return (!scan.ranges.empty());
    }




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

struct SimpleShape{
    std::vector<line_extraction::Line> lines;
    type_util::Point2d intersect;
    double intersect_angle;
};

struct AngleCompare{
    bool operator()(type_util::Point2d &v1, type_util::Point2d &v2){
        return atan2(v1.y,v1.x) > atan2(v2.y,v2.x);

    }
};

class SimpleTriangleDetector{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    LineSegmentDetector lsd_;

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



    // method
    void initParams(){
        nh_private_.param("min_segLength",min_segLength_,0.1);
        nh_private_.param("max_segLength",max_segLength_,0.25);
        nh_private_.param("min_segAngle",min_segAngle_,-0.4*M_PI);
        nh_private_.param("max_segAngle",max_segAngle_,0.4*M_PI);
        nh_private_.param("min_segDist",min_segDist_,0.1);
        nh_private_.param("max_segDist",max_segDist_,2.5);



        nh_private_.param("min_shapeAngle",min_shapeAngle_,M_PI*100.0/180.0);
        nh_private_.param("max_shapeAngle",max_shapeAngle_,M_PI*150.0/180.0);
        nh_private_.param("min_pairInterAngle",min_pairInterAngle_,-0.4*M_PI);
        nh_private_.param("max_pairInterAngle",max_pairInterAngle_,0.4*M_PI);
        nh_private_.param("min_pairInterLength",min_pairInterLength_,0.15);
        nh_private_.param("max_pairInterLength",max_pairInterLength_,0.30);

        nh_private_.param("min_grow_dist",min_grow_dist_,0.02);
        nh_private_.param("min_gap_dist",min_gap_dist_,0.05);

        nh_private_.param("filter_window",filter_window_,4);




    };

    void cacheData(){
        if ( latestScan_.ranges.size() == cache_angle_.size() && latestScan_.angle_min == cache_angle_[0]){
            return;
        }
        auto r = container_util::createValarrayFromVector(latestScan_.ranges);
        for(int i =0;i<r.size();i++){
            r[i]=latestScan_.angle_min + i*latestScan_.angle_increment;
        }
        cache_cos_ = cos(r);
        cache_sin_ = sin(r);

    }

    geometry_msgs::PoseStamped fitModel(vector<type_util::Point2d> & pointsInModel,SimpleShape &shape){

        targetPoints_.header = latestScan_.header;
        geometry_msgs::Pose pose;

        auto q = tf::createQuaternionFromYaw(shape.intersect_angle);

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();



        // convert points to Eigen
        int m = pointsInModel.size();
        Eigen::MatrixXd measuredValues(m, 2);
        for (int i = 0; i < m; i++) {
            measuredValues(i, 0) = pointsInModel[i].x;
            measuredValues(i, 1) = pointsInModel[i].y;

            pose.position.x = pointsInModel[i].x;
            pose.position.y = pointsInModel[i].y;

            targetPoints_.poses.push_back(pose);
        }

        Eigen::VectorXd x(3);
        decltype(x) model(1);

        x(0) = shape.intersect.x;             // initial value for 'a'
        x(1) = shape.intersect.y;             // initial value for 'b'
        x(2) = shape.intersect_angle;             // initial value for 'c'
        model(0) =  M_PI*120.0/180.0;
        ROS_INFO_STREAM("get init result \n"<<x);

#if 1

        opt_util::SimpleSolver<opt_util::LineFunctor> sm;
        sm.updataModel(model);
        sm.setParams(x);
        sm.feedData(measuredValues);
        int status = sm.solve();
        x = sm.getParam();
        ROS_INFO_STREAM("get optimize result \n"<<x);

#endif
        targetPose_.header = latestScan_.header;

        targetPose_.pose.position.x = x(0);
        targetPose_.pose.position.y = x(1);
        targetPose_.pose.position.z = 0.0;

        targetPose_.pose.orientation.x = q.x();
        targetPose_.pose.orientation.y = q.y();
        targetPose_.pose.orientation.z = q.z();
        targetPose_.pose.orientation.w = q.w();





        targetPub_.publish(targetPose_);
        pointsPub_.publish(targetPoints_);

        return targetPose_;





    }



public:
    SimpleTriangleDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
            nh_(nh),
            nh_private_(nh_private),
            lsd_(nh,nh_private){
        initParams();



        targetPub_ = nh_.advertise<geometry_msgs::PoseStamped>("targetPose",1);
        pointsPub_ = nh_.advertise<geometry_msgs::PoseArray>("targetPoints",1);



    };

    vector<geometry_msgs::PoseStamped> detect(){
        vector<geometry_msgs::PoseStamped> targets;
        // get lines
        auto lines = lsd_.getLines();

        // get pair
        int matchCnt = 0;
        std::vector<SimpleShape> pairResults;
        for(int i=0;i<lines.size();i++){
            // calculate pair rule
            double length = lines[i].length();
            bool cond1 = length > min_segLength_ && length < max_segLength_;
            double angle = lines[i].getAngle();
            bool cond2 = angle > min_segAngle_ && angle < max_segAngle_;
            double radius = lines[i].getRadius();
            bool cond3 = radius > min_segDist_ && radius < max_segDist_;

            // match seg rule
            if(cond1&&cond2&&cond3){
                matchCnt ++;
            } else{
                matchCnt =0;
            }

            // calculate pair rule
            if(matchCnt == 2){
                // get intersection
                auto l1 = lines[i-1];
                auto l2 = lines[i];
                type_util::Point2d
                        l1_s(l1.getStart()[0], l1.getStart()[1]),
                        l1_e(l1.getEnd()[0],l1.getEnd()[1]),
                        l2_s(l2.getStart()[0], l2.getStart()[1]),
                        l2_e(l2.getEnd()[0],l2.getEnd()[1]),
                        intersect;
                bool succ = geometry_util::LineLineIntersect(l1_s,l1_e,l2_s,l2_e,intersect);

                if(succ){
                    // if match rule


                    double shapeAngle = M_PI - (l2.getAngle() - l1.getAngle());
                    bool cond4 = shapeAngle > min_shapeAngle_ && shapeAngle < max_shapeAngle_;
                    double intersectAngle = atan2(intersect.y,intersect.x);
                    bool cond5 = intersectAngle > min_pairInterAngle_ && intersectAngle < max_pairInterAngle_;

                    bool cond6 = intersectAngle > atan2(l1_s.y,l1_s.x) && intersectAngle < atan2(l2_e.y,l2_e.x);

                    double intersectLen1 = geometry_util::PointToPointDistance(l1_s,intersect);
                    double intersectLen2 = geometry_util::PointToPointDistance(l2_e,intersect);
                    bool cond7 = intersectLen1 > min_pairInterLength_ &&
                            intersectLen1 < max_pairInterLength_ &&
                            intersectLen2 > min_pairInterLength_ &&
                            intersectLen2 < max_pairInterLength_;

                    if (cond4 && cond5 && cond6 && cond7){
                        //get pair


                        // publish
                        decltype(lines) selectLines = {l1,l2};
                        SimpleShape shape;
                        shape.lines = selectLines;
                        shape.intersect = intersect;
                        shape.intersect_angle = 0.5*(l2.getAngle() + l1.getAngle());
                        pairResults.push_back(shape);
#if 1
                        lsd_.pubMarkers(selectLines);
#endif
                    }
                }
                // final
                matchCnt = 0;
            }
        }
        // merger and remove noise
        if (!pairResults.empty()){
            // get laser
            lsd_.getLaser(latestScan_);
            cacheData();
            auto ranges = container_util::createValarrayFromVector(latestScan_.ranges);
            auto xs = ranges*cache_cos_;
            auto ys = ranges*cache_sin_;


            // grow
            for(int i=0;i<pairResults.size();i++){

                // add points
                auto l1 = pairResults[i].lines[0];
                auto l1_idx = l1.getIndices();
                auto l2 = pairResults[i].lines[1];
                auto l2_idx = l2.getIndices();


                type_util::Point2d
                        l1_s(l1.getStart()[0], l1.getStart()[1]),
                        l1_e(l1.getEnd()[0],l1.getEnd()[1]),
                        l2_s(l2.getStart()[0], l2.getStart()[1]),
                        l2_e(l2.getEnd()[0],l2.getEnd()[1]),
                        intersect;

                // create pointset
                vector<type_util::Point2d> PointsInModel;





                // grow to left
                double dist ;
                type_util::Point2d p;
#if 0

                for(int j = l1_idx[0];j>0;j--){
                    p.x = xs[j];
                    p.y = ys[j];
                    geometry_util::PointToLineDistance(l1_s,l1_e,p,dist);

                    if(dist < min_grow_dist_){
                        PointsInModel.push_back(p);
                    }


                    if (dist > min_gap_dist_)
                        break;

                }

                // grow to midel
                for(int j = l1_idx[l1_idx.size()-1];j<l2_idx[0];j++){
                    p.x = xs[j];
                    p.y = ys[j];
                    geometry_util::PointToLineDistance(l1_s,l1_e,p,dist);
                    double  dist2;
                    geometry_util::PointToLineDistance(l2_s,l2_e,p,dist2);
                    dist = std::min(dist,dist2);


                    if(dist < min_grow_dist_){
                        PointsInModel.push_back(p);
                    }





                }
                // grow to right
                for(int j = l2_idx[l2_idx.size()-1];j<xs.size();j++){
                    p.x = xs[j];
                    p.y = ys[j];
                    geometry_util::PointToLineDistance(l2_s,l2_e,p,dist);
                    if(dist < min_grow_dist_){
                        PointsInModel.push_back(p);
                    }


                    if (dist > min_gap_dist_)
                        break;

                }

#endif
                // add select line
                for(int j = l1_idx[0];j<l1_idx[l1_idx.size()-1];j++){
                    p.x = xs[j];
                    p.y = ys[j];
                    PointsInModel.push_back(p);


                }
                for(int j = l2_idx[0];j<l2_idx[l2_idx.size()-1];j++){
                    p.x = xs[j];
                    p.y = ys[j];
                    PointsInModel.push_back(p);
                }

                // sort
                std::sort(PointsInModel.begin(),PointsInModel.end(),AngleCompare());

                // filter
                vector<double> PointsInModel_xs;
                vector<double> PointsInModel_ys;
                for(int ip =0;ip<PointsInModel.size();ip++){
                    PointsInModel_xs.push_back(PointsInModel[ip].x);
                    PointsInModel_ys.push_back(PointsInModel[ip].y);

                }

                auto tmp_xs = container_util::createValarrayFromVector(PointsInModel_xs);
                auto tmp_ys = container_util::createValarrayFromVector(PointsInModel_ys);


                PointsInModel.clear();

                int wz = 2*filter_window_+1;
                for(int ip = filter_window_ ;ip<tmp_xs.size() - filter_window_-1;ip++){
                    p.x = valarray<double>(tmp_xs[std::slice(ip - filter_window_,wz,1)]).sum()/(wz);
                    p.y = valarray<double>(tmp_ys[std::slice(ip - filter_window_,wz,1)]).sum()/(wz);
                    PointsInModel.push_back(p);
                }

                // fit model
                auto res = fitModel(PointsInModel,pairResults[i]);
                targets.push_back(res);
            }

        }


        return targets;


    }

};

class TargetPublish{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    SimpleTriangleDetector sd_;

public:
    TargetPublish(ros::NodeHandle nh, ros::NodeHandle nh_private):
            nh_(nh),
            nh_private_(nh_private),
            sd_(nh,nh_private)
    {

    }
    void publish(){

    };

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


    double frequency;
    nh_local.param<double>("frequency", frequency, 25);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);

    bool debug_mode;
    nh_local.param("debug",debug_mode, false);

    // get data
    // if successful
    // extract line
    SimpleTriangleDetector sd(nh,nh_local);

    time_util::Timer t;
    while (ros::ok())
    {
        nh_local.param("debug",debug_mode, false);
#if 0
        LineSegmentDetector line_extractor(nh, nh_local);
        line_extractor.getLines();
#endif

        if(debug_mode){
            auto sd_new = SimpleTriangleDetector(nh,nh_local);
            sd_new.detect();
            continue;
        }


        t.start();
        sd.detect();
        t.stop();
        ROS_INFO("full time %.4f",t.elapsedSeconds());
        rate.sleep();
    }
    return 0;
}

