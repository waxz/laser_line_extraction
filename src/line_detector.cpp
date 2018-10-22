#include <laser_line_extraction/line_detector.h>
#include <cpp_utils/svdlinefitting.h>
#include <cpp_utils/eigen_util.h>

// get laser data
// mask valid data
// extract line
// choose valid data

line_extraction::LineSegmentDetector::LineSegmentDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
        line_extraction::LineExtractionROS(nh, nh_private),
        nh_(nh),
        nh_private_(nh_private),
        listener(nh, nh_private)

{
    initParam();


    auto res = listener.createSubcriber<sensor_msgs::LaserScan>(this->scan_topic_,1);

    scan_data_ = std::get<0>(res);
//        listener.getOneMessage(scan_topic_,-1);


}
void line_extraction::LineSegmentDetector::initParam(){
    nh_private_.param("max_range",max_range_,1.0);
    nh_private_.param("min_range",min_range_,0.02);

    nh_private_.param("min_angle",angle_min_,-0.5*M_PI);
    nh_private_.param("max_angle",angle_max_,0.5*M_PI);
    nh_private_.param("min_intensity",min_intensity_,100.0);
    nh_private_.param("filter_window",filter_window_,4);

};
void line_extraction::LineSegmentDetector::processData(){
    // mask ranges
    // set range > max_range to 0

    // mask intensities
    // set intesity < min_intensity to 0
    auto msg = *scan_data_;


    valarray<float> r = container_util::createValarrayFromVector(msg.ranges);
    valarray<float> i = container_util::createValarrayFromVector(msg.intensities);
    r[i<float(min_intensity_)] = 0.0;
    r[r>float(max_range_)] = 0.0;


#if 0
    // filter
    // get xs ys

    valarray<float> xs, ys,Xs, Ys;
    cacheData();
    xs = r*cache_cos_;
    ys = r*cache_sin_;
    Xs = xs;
    Ys = ys;

    int wz = 2*filter_window_+1;

    for(int ip = filter_window_ ;ip<xs.size() - filter_window_-1;ip++){
        if (r[ip - filter_window_] == 0.0 || r[ip + filter_window_] == 0.0 ){
            continue;
        }
        Xs[ip] = valarray<float>(xs[std::slice(ip - filter_window_,wz,1)]).sum()/(wz);
        Ys[ip] = valarray<float>(ys[std::slice(ip - filter_window_,wz,1)]).sum()/(wz);
    }

    r = sqrt(Xs*Xs + Ys*Ys);

#endif
    msg.ranges = container_util::createVectorFromValarray(r);

    boost::shared_ptr<sensor_msgs::LaserScan> scan_ptr_(boost::make_shared<sensor_msgs::LaserScan>(msg) );

    this->laserScanCallback(scan_ptr_);
}

void line_extraction::LineSegmentDetector::getLines( std::vector<line_extraction::Line> &line, detectMode mode){
    line.clear();
    time_util::Timer timer;
    timer.start();

    bool getMsg;
    getMsg = listener.getOneMessage(this->scan_topic_,-1);
#if 0
    if (!getMsg){
        std::cout<<std::endl;

        return ;
    }
#endif
    timer.stop();
    std::cout<<std::endl;

    printf("wait msg  time %.6f\n",timer.elapsedSeconds());
    std::cout<<std::endl;

    timer.start();

#if 0
    printf("line_extraction_ time %.6f\n",timer.elapsedSeconds());
    std::cout<<std::endl;
#endif
    // process data
    // get ptr from msg
    // https://answers.ros.org/question/196697/get-constptr-from-message/
    processData();



    // Extract the lines or cluster
    if(mode == detectMode::lines){
        this->line_extraction_.extractLines(line);

    }else if(mode == detectMode::segments){
        this->line_extraction_.extractSegments(line);
    }else if (mode == detectMode::lights){
        this->line_extraction_.extractLightBoards(line);
    }
    printf("get lines_ num = %d",int(line.size()));
    timer.stop();
    std::cout<<std::endl;

    printf("line_extraction_ time %.6f\n",timer.elapsedSeconds());
    std::cout<<std::endl;

    timer.start();

    // Also publish markers if parameter publish_markers is set to true
    if (this->pub_markers_)
    {
#if 1
        pubMarkers(line);
#endif
    }
    timer.stop();
    std::cout<<std::endl;

    printf("pub_markers_ time %.6f\n",timer.elapsedSeconds());
    std::cout<<std::endl;


}

void line_extraction::LineSegmentDetector::pubMarkers(std::vector<line_extraction::Line> lines){
    // Populate message
    ROS_INFO("publish markers!!");
    laser_line_extraction::LineSegmentList msg;
    this->populateLineSegListMsg(lines, msg);
    this->line_publisher_.publish(msg);
    visualization_msgs::Marker marker_msg;
    this->populateMarkerMsg(lines, marker_msg);
    this->marker_publisher_.publish(marker_msg);


}

bool line_extraction::LineSegmentDetector::getLaser(sensor_msgs::LaserScan &scan){
    scan = *scan_data_;
    return (!scan.ranges.empty());
}

void line_extraction::LineSegmentDetector::cacheData() {
    if ( scan_data_.get()->ranges.size() == cache_angle_.size() && scan_data_.get()->angle_min == cache_angle_[0]){
        return;
    }
    cache_angle_ = container_util::createValarrayFromVector(scan_data_.get()->ranges);
    float angle_min = scan_data_.get()->angle_min;
    float angle_increment = scan_data_.get()->angle_increment;
    for(int i =0;i<cache_angle_.size();i++){
        cache_angle_[i]=angle_min + i*angle_increment;
    }
    cache_cos_ = cos(cache_angle_);
    cache_sin_ = sin(cache_angle_);
}

bool line_extraction::LineSegmentDetector::getXsYs(valarray<float> &xs, valarray<float> &ys) {
    cacheData();
    auto ranges = container_util::createValarrayFromVector(scan_data_.get()->ranges);
    xs = ranges*cache_cos_;
    ys = ranges*cache_sin_;
    return true;
}
bool line_extraction::LineSegmentDetector::getAngles(valarray<float> &angles) {
    cacheData();
    angles = cache_angle_;
    return true;
}


#if 1


    // method
    void line_extraction::SimpleTriangleDetector::initParams(){
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

        nh_private_.param("max_fit_error",max_fit_error_, 0.01);
        nh_private_.param("triangle_direction",triangle_direction_, -1.0);

        nh_private_.param("use_fit_line",use_fit_line_, false);

        // marker type : light-belt, triangle, point-array
        nh_private_.param("marker_type",marker_type_,std::string("light_belt"));

        // marker for locat
        nh_private_.param("max_marker_length",max_marker_length_,0.06);
        nh_private_.param("max_marker_initial_dist",max_marker_initial_dist_, 0.5);

        // point array
        nh_private_.param("max_marker_dist_diff",max_marker_dist_diff_,0.05);
        nh_private_.param("min_update_d",min_update_d_,0.1);
        nh_private_.param("min_update_a",min_update_a_,0.1);


        nh_private_.param("x_conv",x_conv_, 0.05);
        nh_private_.param("y_conv",y_conv_, 0.05);
        nh_private_.param("yaw_conv",yaw_conv_, 0.05);




    };




    bool line_extraction::SimpleTriangleDetector::fitModel(vector<type_util::Point2d> & pointsInModel,double x0, double x1, double x2,geometry_msgs::PoseStamped & ModelPose){

        targetPoints_.header = latestScan_.header;
        geometry_msgs::Pose pose = tf_util::createPoseFromXYYaw(0.0,0.0,x2);
#if 0
        auto q = tf::createQuaternionFromYaw(x2);

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

#endif

        // convert points to Eigen
        int m = pointsInModel.size();
        Eigen::MatrixXd measuredValues(2, m);
        for (int i = 0; i < m; i++) {
            measuredValues(0, i) = pointsInModel[i].x;
            measuredValues(1, i) = pointsInModel[i].y;

            pose.position.x = pointsInModel[i].x;
            pose.position.y = pointsInModel[i].y;

            targetPoints_.poses.push_back(pose);
        }

        // optimize param
        Eigen::VectorXd x(4);
        // model param
        Eigen::MatrixXd model(1,1);

        x(0) = x0;             // initial value for 'a'
        x(1) = x1;             // initial value for 'b'
        x(2) = x2;             // initial value for 'c'
        x(3) =  M_PI*120.0/180.0;
        model(0,0) = triangle_direction_;
        ROS_INFO_STREAM("get init result \n"<<x);

#if 1

        opt_util::SimpleSolver<opt_util::VFunctor> sm;
        sm.updataModel(model);
        sm.setParams(x);
        sm.feedData(measuredValues);
        int status = sm.solve();
        auto meanerror = sm.getMeanError();

        // check fit error and
        ROS_INFO_STREAM("get optimize result \n"<<sm.getParam()<<"mean error"<<meanerror);

        if(meanerror < max_fit_error_){
            x = sm.getParam();

        } else{
            return false;
        }


#endif
        targetPose_.header = latestScan_.header;
        targetPose_.pose = tf_util::createPoseFromXYYaw(x(0),x(1),x(2));
#if 0

        targetPose_.pose.position.x = x(0);
        targetPose_.pose.position.y = x(1);
        targetPose_.pose.position.z = 0.0;
        q = tf::createQuaternionFromYaw(x(2));

        targetPose_.pose.orientation.x = q.x();
        targetPose_.pose.orientation.y = q.y();
        targetPose_.pose.orientation.z = q.z();
        targetPose_.pose.orientation.w = q.w();
#endif




        targetPub_.publish(targetPose_);
        pointsPub_.publish(targetPoints_);
        ModelPose = targetPose_;

        return true;





    }



    line_extraction::SimpleTriangleDetector::SimpleTriangleDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
            nh_(nh),
            nh_private_(nh_private),
            listener_(nh,nh_private),
            lsd_(nh,nh_private){
        initParams();



        targetPub_ = nh_.advertise<geometry_msgs::PoseStamped>("targetPose",1);
        pointsPub_ = nh_.advertise<geometry_msgs::PoseArray>("targetPoints",1);
        initPosePub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);

        initPose_.header.frame_id = "map";
        initPose_.pose.covariance[0] = x_conv_;
        initPose_.pose.covariance[7] = y_conv_;
        initPose_.pose.covariance[35] = yaw_conv_;




        baseToLaser_tf_.setIdentity();
        mapToOdom_tf_.setIdentity();
        odomToBase_tf_.setIdentity();

        map_frame_id_ = "map";
        odom_frame_id_ = "odom";
        base_frame_id_ = "base_link";




    };
line_extraction::SimpleTriangleDetector::~SimpleTriangleDetector() {
    nh_private_.setParam("/amcl/tf_broadcast", true);

}

    vector<geometry_msgs::PoseStamped> line_extraction::SimpleTriangleDetector::detect(){
        vector<geometry_msgs::PoseStamped> targets;
        // get lines


        if (marker_type_ == "light-belt"){
            std::vector<line_extraction::Line> lines;
            lsd_.getLines(lines);
            if (lines.size() == 1){
                auto lightLine = lines[0];

                // fit line
                vector<type_util::Point2d> pointInLine;
                type_util::Point2d p;

                valarray<float> xs ,ys;
                lsd_.getXsYs(xs, ys);

                auto indices = lightLine.getIndices();

                for (int i=0;i<indices.size();i++){
                    p.x = xs[indices[i]];
                    p.y = ys[indices[i]];
                    pointInLine.push_back(p);
                }
                double a,b,c;
                fit_util::svdfit(pointInLine,a, b, c);
                // ax + by = c;
                // get two point
                type_util::Point2d line_s(0.0, c/b), line_e(c/a, 0.0);
                type_util::Point2d p1(lightLine.getStart()[0],lightLine.getStart()[1]);
                type_util::Point2d p2(lightLine.getEnd()[0],lightLine.getEnd()[1]);

                type_util::Point2d fit_s, fit_e;
                bool suc1 = geometry_util::getPedal(line_s,line_e,p1,fit_s);
                bool suc2 = geometry_util::getPedal(line_s,line_e,p1,fit_e);


                geometry_msgs::PoseStamped pose;

                if (suc1 &&suc2 && use_fit_line_){

                    pose.pose.position.x = 0.5*( fit_s.x + fit_e.x);
                    pose.pose.position.y = 0.5*( fit_s.y + fit_e.y);

                    double  yaw = atan2(fit_e.y - fit_s.y, fit_e.x - fit_s.x) - 0.5*M_PI;
                    auto q = tf_util::createQuaternionFromYaw(yaw);

                    tf::quaternionTFToMsg(q,pose.pose.orientation);
                }else{
                    pose.pose.position.x = 0.5*( lightLine.getEnd()[0] + lightLine.getStart()[0]);
                    pose.pose.position.y = 0.5*( lightLine.getEnd()[1] + lightLine.getStart()[1]);

                    auto q = tf_util::createQuaternionFromYaw(lightLine.getAngle());

                    tf::quaternionTFToMsg(q,pose.pose.orientation);
                }

                targets.push_back(pose);

            }
        } else if(marker_type_ == "triangle"){
            std::vector<line_extraction::Line> lines;
            lsd_.getLines(lines);

            // first get parameter
#if 1

            // get pair
        int matchCnt = 0;
        std::vector<line_extraction::SimpleShape> pairResults;
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
                matchCnt =1;
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
                    ROS_INFO("get line \n1 =[%.3f,%.3f],[%.3f,%.3f]\n2 = [%.3f,%.3f],[%.3f,%.3f]\ni = [%.3f,%.3f]",
                             l1_s.x, l1_s.y,l1_e.x,l1_e.y,l2_s.x,l2_s.y,l2_e.x,l2_e.y,intersect.x,intersect.y);


                    double gap_dist = geometry_util::PointToPointDistance(l1_e,l2_s);
                    bool cond8 = gap_dist<0.5*min_segLength_;
                    if(!cond8){
                        continue;
                    }
                    double shapeAngle = M_PI - triangle_direction_*(l2.getAngle() - l1.getAngle());
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


                    if (cond4 && cond5 && cond6 && cond7&&cond8){
                        //get pair


                        // publish
                        decltype(lines) selectLines = {l1,l2};
                        line_extraction::SimpleShape shape;
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
            valarray<float > xs;
            valarray<float > ys;
            lsd_.getXsYs(xs,ys);


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
#if 1

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
                geometry_msgs::PoseStamped targetPose;
                bool fitsucc = fitModel(PointsInModel,pairResults[i].intersect.x,pairResults[i].intersect.y,pairResults[i].intersect_angle,targetPose);
                if(fitsucc){
                    targets.push_back(targetPose);

                }
            }

        }

#endif
        } else if(marker_type_ == "points-array"){

            std::cout<<"start get lines "<< std::endl;

            // cheange detector to simple detector
            //
            time_util::Timer timer;
            timer.start();

            std::vector<line_extraction::Line> lines;
            lsd_.getLines(lines, line_extraction::LineSegmentDetector::detectMode::lights);

#if 1
            timer.stop();
            std::cout<<std::endl;

            printf("lsd_.getLines time %.6f\n",timer.elapsedSeconds());
            std::cout<<std::endl;
#endif
            std::cout<<"get lines "<< lines.size()<< std::endl;

#if 0
            // todo :bypass debug line_detector
            return targets;
#endif

            // get 3 or 4 point
            // 3: flat board
            // 4 : shelf
            bool board_or_shelf ;

            // get params
#if 0
            nh_private_.param("points_array",params_);
#endif
            nh_private_.getParam("points_array",params_);
            board_or_shelf = params_.size() == 3;
            if (params_.size() == 3){
                board_or_shelf = true;
            } else if (params_.size() == 4){
                board_or_shelf = false;
            } else{
                return targets;
            }



            // get line segmentation

            // get 3 or 4 point
            int data_num = (board_or_shelf)? 3:4;

            if (lines.size() >= data_num){
                // timer to record run time
                time_util::Timer timer;
                timer.start();


                // get laserscan angle data
                lsd_.getAngles(cache_angle_);
                lsd_.getLaser(latestScan_);


                // eigen matrix
                // model 1 data [angle]
                Eigen::MatrixXd measureData(1, data_num);
                // model 2 data [x,y,idx] , idx = match index in model
                Eigen::MatrixXd measureData2;

                Eigen::MatrixXd model(2,data_num);
                Eigen::VectorXd x(3);
                // initial guess
                x << 0.0, 0.0, 0.0;

                // fill model data
                for (int di = 0;di <data_num;di++){
                    auto id1 = lines[di].getIndices();
                    model(0,di) = params_[di]["x"];
                    model(1,di) = params_[di]["y"];
                }

                // look up tf
                ros::Time tn = latestScan_.header.stamp;
#if 1
                // get base im map tf
                // transform model point to vase frame;
                std::cout<<"looking up tf "<< std::endl;

                // look up base to laser frame for once
                if(baseToLaser_tf_.getOrigin().x() == 0.0){
                    listener_.getTransform(base_frame_id_,latestScan_.header.frame_id,baseToLaser_tf_,tn,0.1,
                                                   true);
                }

                // get map odom tf
                bool gettf1 = listener_.getTransform(map_frame_id_,odom_frame_id_,mapToOdom_tf_,tn,0.01,
                                               false);

                // get odom base tf
                // look up odom to base tf
                bool gettf2 = listener_.getTransform(odom_frame_id_,base_frame_id_,odomToBase_tf_,tn,0.01,
                                               false);

                if ( ! gettf1 || ! gettf2 ){
                    return targets;
                }

                std::cout<<"get up tf  ok"<< std::endl;

#endif
                //get map to laser tf
                tf::Transform mapToLaser_tf = mapToOdom_tf_*odomToBase_tf_*baseToLaser_tf_;
                // get relative pose in laser frame


                // tranform model data to relative position
                decltype(model) origin_pos = model;
                ROS_ERROR_STREAM("get  model pose \n"<<model);


                eigen_util::TransformationMatrix2d trans_laser(mapToLaser_tf.getOrigin().x(), mapToLaser_tf.getOrigin().y(), tf::getYaw(mapToLaser_tf.getRotation()));
                model = trans_laser.inverse()*model;
                ROS_ERROR_STREAM("get laser in map  pose \n"<<trans_laser.matrix()<<"\n inverse \n"<< trans_laser.inverse().matrix());

                ROS_ERROR_STREAM("get relative model pose \n"<<model);

                // how many combinition
                size_t dist_pair_num = 0;
                for (size_t i = data_num - 1; i > 0; i--){
                    dist_pair_num += i;
                }

                // match vector [0,0,0]
                valarray<int> match_vec(0,dist_pair_num);

                Eigen::VectorXd model_dist(dist_pair_num);

                // model dist vec
                int idx = 0;
                for(int i = 0;i<data_num - 1;i++){
                    for (int j = i+1;j<data_num;j++){
                        double dist = (model.col(i) - model.col(j)).norm();
                        model_dist(idx) = dist;
                        idx ++;
                    }
                }

                // get measure dist vec
                int lines_size = lines.size();
                for (int i = 0; i< lines_size - 1; i++){
                    for(int j = i+1; j < lines_size; j++ ){

                    }
                }
                // if fit any dist ,set that idx to 1

                // decide to detect full data_num marker or not
//                matchMarkers();



                //detect all marker
                // or track partial marker
                bool full_detect = false;
                if (full_detect){

                }else{

                    // find closest marker
                    //

                }


                // check markers base  on distance
                // new lines vector
                //
                std::vector<double > diff_score_vec;

                for (int i = 0; i<= lines.size() - data_num ;i++){

                    auto line_select = decltype(lines)(&lines[i], &lines[i+data_num] );

                    // check dist btween pair
                    double check_dist_diff = 0.0;
                    for(int j = 0; j < data_num - 1; j++){
                        for (int k= j+1 ;k < data_num; k++){
                            auto l1 = line_select[j], l2 = line_select[k];

                            type_util::Point2d p1(0.5*(l1.getStart()[0] + l1.getEnd()[0]),0.5*(l1.getStart()[1] + l1.getEnd()[1])),
                                    p2(0.5*(l2.getStart()[0] + l2.getEnd()[0]),0.5*(l2.getStart()[1] + l2.getEnd()[1])),
                                    m1(model(0,j),model(1,j)), m2(model(0,k),model(1,k));

                            if (l1.length() > max_marker_length_ || l2.length() > max_marker_length_){
                                check_dist_diff = 10.0;
                                break;
                            }

                            double dist1 = geometry_util::PointToPointDistance(p1,p2);
                            double dist2 = geometry_util::PointToPointDistance(m1,m2);

                            double diff = fabs(dist1 - dist2);
                            if(diff > check_dist_diff){
                                check_dist_diff = diff;
                            }
                            if(check_dist_diff > max_marker_dist_diff_){
                                break;
                            }



                        }
                        if(check_dist_diff > max_marker_dist_diff_){
                            break;
                        }
                    }

                    ROS_ERROR("get id %d, score %.3f",i, check_dist_diff);
                    diff_score_vec.push_back(check_dist_diff);


                }

                // get best choice
                auto best_i = container_util::argMin(diff_score_vec);
                auto best_score = diff_score_vec[best_i];
                ROS_ERROR("best_id %d, score %.3f", best_i,best_score);
                if (best_score > max_marker_dist_diff_){
                    ROS_ERROR("first_step failue");
                    return targets;

                }



                lines =  decltype(lines)(&lines[best_i], &lines[best_i+data_num] );

                // fit two model
                // model 1) : angle only model
                // model 2) : 2d position model
#if 0
                // model 1
                // fill measureData from laser
                for (int di = 0;di <data_num;di++){
                    auto id1 = lines[di].getIndices();
                    measureData(0,di) =  valarray<float>(cache_angle_[std::slice(id1[di],id1.size(),1)]).sum()/id1.size();
                }

                ROS_ERROR_STREAM("model 1 measuredata \n"<<measureData);

                opt_util::SimpleSolver<opt_util::AngleFunctor> sm;

                sm.updataModel(model);

                sm.setParams(x);
                sm.feedData(measureData);

                int status = sm.solve();
                auto meanerror = sm.getMeanError();

                ROS_ERROR_STREAM("get x \n"<<x<<std::endl << "error "<<meanerror);
                x = sm.getParam();



#endif





                // model 2
                std::vector<double> mdata;
                std::valarray<float > xs, ys;
                lsd_.getXsYs(xs,ys);
                for(int di = 0 ; di<data_num ;di ++){
                    auto id1 = lines[di].getIndices();

                    for (int dj=0;dj < id1.size(); dj++){

                        // push x
                        mdata.push_back(static_cast<double> (xs[id1[dj]]) );
                        // y
                        mdata.push_back(static_cast<double> (ys[id1[dj]]) );
                        // index
                        mdata.push_back(model(0,di));
                        mdata.push_back(model(1,di));

                    }
                }
                measureData2 = Eigen::Map<Eigen::MatrixXd>(mdata.data(), 4, mdata.size()/4);


                ROS_ERROR_STREAM("model 2 measuredata \n"<<measureData2);



                opt_util::SimpleSolver<opt_util::AngleRangeFunctor> sm2;

                sm2.updataModel(model);

                sm2.setParams(x);

                sm2.feedData(measureData2);


                int ststus2 = sm2.solve();

                auto meanerror2 = sm2.getMeanError();

                auto x2 = sm2.getParam();

                ROS_ERROR_STREAM("get x2 \n"<<x2<<std::endl << "error "<<meanerror2);

                auto t = timer.elapsedSeconds();
                ROS_INFO("fit time %.4f",t);
                // check error
                //rule 1: mean error
                // rule 2: relative angle
                if (meanerror2 > max_fit_error_  ){
                    return targets;
                }

                // get transform position
                eigen_util::TransformationMatrix2d trans2(x2(0), x2(1), x2(2));


                decltype(model) pos = trans2*model;



                geometry_msgs::PoseStamped origin_pose;

                geometry_msgs::PoseStamped pose;
                pose.pose.position.z = baseToLaser_tf_.getOrigin().z();
                origin_pose.pose.position.z = baseToLaser_tf_.getOrigin().z();

                double  yaw;

                if (board_or_shelf){
#if 0
                    pose.pose.position.x = pos(0,1);
                    pose.pose.position.y = pos(1,1);
#endif

                    Eigen::VectorXd m(2);

                    // new pose
                    m = pos.rowwise().mean();
                    pose.pose.position.x = m(0);
                    pose.pose.position.y = m(1);

                    yaw = atan2(pos(1,1) - pos(1,0), pos(0,1) - pos(0,0)) - 0.5*M_PI;
                    tf::quaternionTFToMsg(tf_util::createQuaternionFromYaw(yaw),pose.pose.orientation);

                    // origin pose
                    m = origin_pos.rowwise().mean();
                    origin_pose.pose.position.x = m(0);
                    origin_pose.pose.position.y = m(1);

                    yaw = atan2(origin_pos(1,1) - origin_pos(1,0), origin_pos(0,1) - origin_pos(0,0)) - 0.5*M_PI;
                    tf::quaternionTFToMsg(tf_util::createQuaternionFromYaw(yaw),origin_pose.pose.orientation);



                } else{

                    Eigen::VectorXd m(2);

                    // new pose
                    m = pos.rowwise().mean();
                    pose.pose.position.x = m(0);
                    pose.pose.position.y = m(1);

                    yaw = atan2(pos(1,1) - pos(1,0), pos(0,1) - pos(0,0)) ;
                    tf::quaternionTFToMsg(tf_util::createQuaternionFromYaw(yaw),pose.pose.orientation);

                    // origin pose
                    m = origin_pos.rowwise().mean();
                    origin_pose.pose.position.x = m(0);
                    origin_pose.pose.position.y = m(1);

                    yaw = atan2(origin_pos(1,1) - origin_pos(1,0), origin_pos(0,1) - origin_pos(0,0)) ;
                    tf::quaternionTFToMsg(tf_util::createQuaternionFromYaw(yaw),origin_pose.pose.orientation);

                }




                pose.header = latestScan_.header;
                pose.header.stamp = tn;



                targetPub_.publish(pose);


                // how to use result
                // 1) punlish target in laser frame
# if 0
                targets.push_back(pose);
#endif

                // 2) publish odom in map frame

                // succ

                // get map odom tf

                tf::Transform origin_pose_tf, pose_tf;
                tf::poseMsgToTF(origin_pose.pose,origin_pose_tf);
                tf::poseMsgToTF(pose.pose,pose_tf);
                mapToOdom_tf_ = origin_pose_tf*pose_tf.inverse()*baseToLaser_tf_.inverse()*odomToBase_tf_.inverse();

                tf::poseTFToMsg(mapToOdom_tf_, map_odom_pose_.pose);

                // reset amcl
#if 0
                initPose_.header.stamp = ros::Time::now();
                tf::poseTFToMsg(mapToOdom_tf_*odomToBase_tf_,initPose_.pose.pose);

                initPosePub_.publish(initPose_);
#endif
                // todo : bypass tf update
#if 1
                nh_private_.setParam("/amcl/tf_broadcast", false);
#endif
#if 1
                targets.push_back(map_odom_pose_);
#endif



            }



        }



        return targets;


    }


#endif


namespace line_extraction{
    class SimpleLightBeltDetector{
    protected:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        rosnode::Listener listener_;

        string scan_topic_;
        double min_intensity_;
        double min_line_length_;

        std::shared_ptr<sensor_msgs::LaserScan> scan_data_;

        void initPrams(){
            nh_private_.param("scan_topic", scan_topic_, string("scan_filtered"));
            nh_private_.param("min_intensity",min_intensity_,1000.0);
            nh_private_.param("min_line_length",min_line_length_,0.4);

        }
    public:
        SimpleLightBeltDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
                nh_(nh),
        nh_private_(nh_private),
        listener_(nh, nh_private){

            auto res = listener_.createSubcriber<sensor_msgs::LaserScan>(scan_topic_,1);
            scan_data_ = std::get<0>(res);

        }


    };

}


line_extraction::TargetPublish::TargetPublish(ros::NodeHandle nh, ros::NodeHandle nh_private):
            nh_(nh),
            nh_private_(nh_private),
            sd_(nh,nh_private),
            listener_(nh,nh_private),
            fake_pose_topic_("triangle_pose"),
            cmd_data_ptr_(std::make_shared<std_msgs::Header>()),
            smoothPose_(2),
            pubThread_(50,fake_pose_topic_,nh),
            tfThread_(20)
    {
        pubthreadClass_.setTarget(pubThread_);
        tfthreadClass_.setTarget(tfThread_);

        base_frame_id_ = "base_link";
        laser_frame_id_ = "base_laser";
        target_framde_id_ = "base_triangle";

        fake_pose_topic_ = "triangle_pose";

        cmd_topic_ = "task_switch";

        baseToLaser_tf_.setIdentity();

        nh_private_.param("expire_sec",expire_sec_,5);
        nh_private_.param("detect_time_tol",detect_time_tol_, 0.5);
        lastOkTime_ = ros::Time::now();

        nh_private_.param("broadcast_tf",broadcast_tf_, false);
        nh_private_.param("pub_pose",pub_pose_, true);
        nh_private_.param("broadcast_map_odom_tf", broadcast_map_odom_tf_, false);
        nh_private_.param("pub_lighthouse", pub_lighthouse_, false);
        nh_private_.param("min_update_d",min_update_d_,0.1);
        nh_private_.param("min_update_a",min_update_a_,0.1);


        auto res = listener_.createSubcriber<std_msgs::Header>(cmd_topic_,1);
        cmd_data_ptr_ = std::get<0>(res);

        running_ = false;


        lighthouse_pose_topic_ = "lighthouse_pose";
        lighthouse_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(lighthouse_pose_topic_,1);


        odomToBase_tf_.setIdentity();








    }

line_extraction::TargetPublish::~TargetPublish() {
    nh_private_.setParam("/amcl/tf_broadcast", true);

}
    void line_extraction::TargetPublish::publish(){
        // broadcast base_link base_triangle tf
        // publish base_link pose in base_triangle
        // publish base_triangle pose in base_link
        // how to change state
        if(!running_){


            bool getmsg = listener_.getOneMessage(cmd_topic_,0.05);

            if(getmsg){
                if ( cmd_data_ptr_.get()->frame_id == "lighthouse_planner" || cmd_data_ptr_.get()->frame_id.empty() )
                {
                    if ( cmd_data_ptr_.get()->seq == 0 )
                    {
                        running_ = false;
                    }
                    else
                    {
                        running_ = true;
                    }
                }

            }
            if(!running_){
                smoothPose_.clear();
                pubthreadClass_.pause();
                tfthreadClass_.pause();
                if(broadcast_map_odom_tf_){
                    ROS_ERROR("stop reflector localization");

                    nh_private_.setParam("/amcl/tf_broadcast", true);
                }
                return;
            }
        }

        std::cout<<"start detect "<<std::endl;
        ros::Time tn = ros::Time::now();


        // skip
        tf::Transform odomToBase_tf;
        bool gettf2 = listener_.getTransform("odom","base_link",odomToBase_tf,tn,0.02,
                                             false);
        bool detected = false;
        nh_private_.param("/amcl/tf_broadcast",detected);

        if (!gettf2){
            ROS_ERROR("get odom base tf fail! skip");
            if ((tn -lastOkTime_).toSec() > detect_time_tol_){

                if (detected){
                    running_ = false;
                }

            }
            return;
        }

        auto base_move = odomToBase_tf_.inverse()*odomToBase_tf;
        double move_d = sqrt(pow(base_move.getOrigin().x(),2) + pow(base_move.getOrigin().y(),2) );
        double move_a = fabs(tf::getYaw(base_move.getRotation()));

        ROS_ERROR_STREAM("base move d:"<<move_d<<"a :"<<move_a);
        bool to_update_tf = true;

        if (move_d < min_update_d_ && move_a < min_update_a_){

            ROS_ERROR("little move!!");
            if (smoothPose_.num_ != 0){
                ROS_ERROR("skip!!");

                if ((tn -lastOkTime_).toSec() > detect_time_tol_){
                    if (detected){
                        running_ = false;
                    }

                }

                to_update_tf = false ;

            }

        }
        if (to_update_tf){
            odomToBase_tf_ = odomToBase_tf;
        }

        ROS_ERROR("start detect!!");


        auto targets = sd_.detect();
        if(targets.size() == 1){

            // publish
            auto pose = targets[0];

            smoothPose_.update(pose.pose.position.x,pose.pose.position.y, tf::getYaw(pose.pose.orientation));

            // use smooth cache

#if 1

            if (!smoothPose_.full()){
               return;

            }
#endif
            ROS_ERROR("start pub!!");

            //
            tf::Transform transform;
            tf::poseMsgToTF(pose.pose,transform);

#if 0
            smoothPose_.getMean();

            transform.setOrigin(tf::Vector3(smoothPose_.mean_x_,smoothPose_.mean_y_,0.0));


            transform.setRotation(tf::createQuaternionFromYaw(smoothPose_.mean_yaw_));
#endif

            if ( broadcast_tf_){

                if(baseToLaser_tf_.getOrigin().x() == 0.0){
                    bool gettf = listener_.getTransform(base_frame_id_,laser_frame_id_,baseToLaser_tf_,ros::Time::now(),0.1,
                                                        false);
                    if(!gettf){
                        return;
                    }
                }

                ros::Duration transform_tolerance;
                transform_tolerance.fromSec(0.1);
                ros::Time transform_expiration = (tn + transform_tolerance);
                stampedTransform_ = tf::StampedTransform(baseToLaser_tf_*transform,
                                                         transform_expiration,
                                                         base_frame_id_, target_framde_id_);
                tfthreadClass_.syncArg(stampedTransform_);
            }

            if (pub_pose_){
                // publish base in triangle
                base_in_triangle_.header.stamp = tn;
                base_in_triangle_.header.frame_id = target_framde_id_;
                tf::poseTFToMsg((baseToLaser_tf_*transform).inverse(),base_in_triangle_.pose);
                pubthreadClass_.syncArg(base_in_triangle_);
            }



            lastOkTime_ = tn;

            // publish triangle in base
            if (pub_lighthouse_){
                triangle_in_base_.header.stamp = tn;
                triangle_in_base_.header.frame_id = base_frame_id_;
                tf::poseTFToMsg(baseToLaser_tf_*transform,triangle_in_base_.pose);
                lighthouse_pose_pub_.publish(triangle_in_base_);
            }
            if(broadcast_map_odom_tf_){
                ros::Duration transform_tolerance;
                transform_tolerance.fromSec(0.1);
                ros::Time transform_expiration = (tn + transform_tolerance);
                stampedTransform_ = tf::StampedTransform(transform,
                                                         transform_expiration,
                                                         "map", "odom");
                if (to_update_tf){
                    tfthreadClass_.syncArg(stampedTransform_);
                }

            }




            if ( broadcast_tf_&&!tfthreadClass_.isRunning()){
                tfthreadClass_.start();
            }
            if (pub_pose_&&!pubthreadClass_.isRunning()){
                pubthreadClass_.start();
            }

            if(broadcast_map_odom_tf_ &&!tfthreadClass_.isRunning() ){
                tfthreadClass_.start();

            }

        }else{
            // detect fail
            if((smoothPose_.num_ == 0)){
                triangle_in_base_.header.frame_id = "detect_failed" ;
                lighthouse_pose_pub_.publish(triangle_in_base_);
                pubthreadClass_.pause();
                tfthreadClass_.pause();
                return;
            }
            ros::Time tn = ros::Time::now();

            auto dur = tn -lastOkTime_;
//            ROS_ERROR("dur time : %f",dur.toSec());
            if (dur.toSec() < detect_time_tol_){

                return;
            } else{
                triangle_in_base_.header.frame_id = "detect_failed" ;
                lighthouse_pose_pub_.publish(triangle_in_base_);

            }
            if(dur.toSec()<expire_sec_){
                tf::Transform tranform_change,triangle_tf;
                tranform_change.setIdentity();
                bool get = listener_.getTransformChange("odom",base_frame_id_,tranform_change,
                                             base_in_triangle_.header.stamp,tn,0.01,false);
//                ROS_ERROR("time : %f, get chane tf %d,x=%.3f,y=%.3f,yaw=%.3f",dur.toSec(), get,tranform_change.getOrigin().x(),tranform_change.getOrigin().y(),
//                tf::getYaw(tranform_change.getRotation()));


                if ( broadcast_tf_){
                    ros::Duration transform_tolerance;
                    transform_tolerance.fromSec(0.1);
                    ros::Time transform_expiration = (tn + transform_tolerance);
                    stampedTransform_ = tf::StampedTransform(tranform_change.inverse()*stampedTransform_,
                                                             transform_expiration,
                                                             base_frame_id_, target_framde_id_);
                    tfthreadClass_.syncArg(stampedTransform_);
                }

                if(pub_pose_){
                    tf::poseMsgToTF(base_in_triangle_.pose,triangle_tf);
                    tf::poseTFToMsg(triangle_tf*tranform_change,base_in_triangle_.pose);
                    base_in_triangle_.header.stamp = tn;
                    pubthreadClass_.syncArg(base_in_triangle_);
                }
                if(broadcast_map_odom_tf_){

                }

                smoothPose_.num_=1;

                // publish triangle in base
                triangle_in_base_.header.stamp = tn;
#if 0
                triangle_in_base_.header.frame_id = "detect_failed";

#endif
                tf::poseMsgToTF(triangle_in_base_.pose,triangle_tf);
                tf::poseTFToMsg(tranform_change.inverse()*triangle_tf,triangle_in_base_.pose);
#if 0
                lighthouse_pose_pub_.publish(triangle_in_base_);
#endif

            }else{
                smoothPose_.clear();
                pubthreadClass_.pause();
                tfthreadClass_.pause();
                running_ = false;
                if(broadcast_map_odom_tf_){
                    ROS_ERROR("stop reflector localization; timeout ");

                    nh_private_.setParam("/amcl/tf_broadcast", true);
                }
            };

        }

    };




