//
// Created by waxz on 9/20/18.
//

#include <ros/ros.h>
#include <vector>
#include <valarray>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <XmlRpc.h>

#include <laser_line_extraction/line_detector.h>

#include <cpp_utils/levmarq.h>
using std::vector;
using std::valarray;
using std::string;
//
class MarkerTracker{

protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    line_extraction::LineSegmentDetector lsd_;

    // params
    double min_score_;

    void initParams(){
        nh_private_.param("min_score", min_score_, 0.5);

    }

    // template
    // how to define target, config param
    // contation marker position and final relative robot pose
    // a marker point array ,in robot frame
    // or line with start and end point
    // frame_id : line type or point array type
    // point array first

    vector<geometry_msgs::PointStamped> marker_template_;
    vector<geometry_msgs::PointStamped> marker_detect_;
    std::vector<line_extraction::Line> line_detect_;
    string marker_param_name_;
    bool get_template_;
    bool getMarkerTemplate(){
        XmlRpc::XmlRpcValue params;
        nh_private_.param(marker_param_name_,params);
        get_template_ = false;

        if (params.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (params.size() < 2){
                return false;
            }
            marker_template_.clear();
            geometry_msgs::PointStamped p;
            for (size_t i = 0; i < params.size(); i++) {
                string s = params[i]["header"]["frame_id"];
                p.header.frame_id = s;
                p.point.x = params[i]["point"]["x"];
                p.point.y = params[i]["point"]["y"];
                p.point.z = params[i]["point"]["z"];

                marker_template_.push_back(p);
            }

        } else{
            return false;
        }

        get_template_ = true;
        return true;

    }


    // get  marker from line extraction

    valarray<double> getFeature(const vector<geometry_msgs::PointStamped> &vec){
        // rule :2 point
        // rule :more than 2 point
        valarray<double > feat(vec.size());
        vector<double> feature;
        return feat;
    }

    double getSCore(){

        double  score = 0.0;

        if (marker_detect_.empty() ||marker_detect_.size() != marker_template_.size()){
            return 100.0;
        }
        auto feat_template = getFeature(marker_template_);
        auto feat_sensor = getFeature(marker_detect_);
        //get feature

        // rule 1 : feature diff
        score += (abs(feat_template - feat_sensor)).sum()/feat_sensor.size();

#if 0
        for(int i=0;i<marker_detect_.size();i++){
            score += sqrt(pow(marker_detect_[i].point.x - marker_template_[i].point.x,2) + pow(marker_detect_[i].point.y - marker_template_[i].point.y,2) );
        }

#endif

        return  score;
    }



    vector<geometry_msgs::PointStamped> getPointsFromLines(){

        vector<geometry_msgs::PointStamped> markerPoints;
        geometry_msgs::PointStamped p;
        for (int i = 0; i < line_detect_.size(); i++){
            p.point.x = 0.5*(line_detect_[i].getStart()[0] + line_detect_[i].getEnd()[0]);
            p.point.y = 0.5*(line_detect_[i].getStart()[1] + line_detect_[i].getEnd()[1]);

            markerPoints.push_back(p);
        }
        return markerPoints;
    }

    // get best match for each point in marker template
    // eg: [0,2,3] mean point (index 0, 2, 3) in markers match point (index 0, 1, 2) in markers template
    vector<int> chooseBestMatch(){

        vector<int> res;
        marker_detect_ = getPointsFromLines();

        // seach point in markers
        double  score = getSCore();

        if (score < min_score_){

            for(int i=0; i <marker_template_.size();i++){
                res.push_back(i);
            }
        }


        return res;
    }

    bool  fitMode(const vector<int> &bestMatch , geometry_msgs::Pose &targetPose){
        geometry_msgs::Pose target_pose;

        // prepare data
        // convert points to Eigen
        int n = bestMatch.size();
        int m;
        for(int i=0;i<n;i++){
            m += line_detect_[bestMatch[i]].numPoints() ;
        }
        Eigen::MatrixXd measuredValues(m, 2);
        int idx = 0;
        valarray<float> xs , ys;
        lsd_.getXsYs(xs, ys);
        for (int i = 0; i < n; i++) {
            auto indice = line_detect_[i].getIndices();
            for (int j=0 ; j< line_detect_[i].numPoints();j++){
                measuredValues(idx, 0) = xs[indice[j]];
                measuredValues(idx, 1) = ys[indice[j]];
                idx ++;
            }
        }

        // optimize param
        Eigen::VectorXd x(3);
        // x,y,yaw

        // model param
        decltype(x) model(marker_template_.size() * 2);
        // each belt center in base frame


        // get initial guess
        // get first two point as;

#if 0
        x(0) = x0;             // initial value for 'a'
        x(1) = x1;             // initial value for 'b'
        x(2) = x2;             // initial value for 'c'
        model(0) = triangle_direction_;

#endif



        // fit

        return true;
    }

    void computePose(){
        marker_detect_.clear();
        line_detect_ = lsd_.getLines(line_extraction::LineSegmentDetector::detectMode::segments);

        auto bestMatch = chooseBestMatch();


        // check validation

        // simple check

        // get feature vector and choose best

    }




    // verify

    // fit model

    // get target pose

public:
    MarkerTracker(ros::NodeHandle nh, ros::NodeHandle nh_private):
            nh_(nh), nh_private_(nh_private),
            lsd_(nh,nh_private){
        marker_template_.clear();
        marker_detect_.clear();
        marker_param_name_ = "marker_params";
        get_template_ = false;
        getMarkerTemplate();

        initParams();
    }

};

class SimpleLightBeltDetector{

};

int main(int argc, char **argv){

    ros::init(argc, argv, "light_belt_detector_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");



}