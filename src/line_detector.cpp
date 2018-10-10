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

std::vector<line_extraction::Line> line_extraction::LineSegmentDetector::getLines(detectMode mode){
    lines_.clear();

    bool getMsg;
    getMsg = listener.getOneMessage(this->scan_topic_,-1);

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



    // Extract the lines or cluster
    if(mode == detectMode::lines){
        this->line_extraction_.extractLines(lines_);

    }else if(mode == detectMode::segments){
        this->line_extraction_.extractSegments(lines_);
    }
    printf("get lines_ num = %d",int(lines_.size()));
    std::cout<<std::endl;
    timer.stop();
    printf("time %.3f\n",timer.elapsedSeconds());


    // Also publish markers if parameter publish_markers is set to true
    if (this->pub_markers_)
    {
#if 1
        pubMarkers(lines_);
#endif
    }


    return lines_;

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

        //triangle_direction



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
        Eigen::MatrixXd measuredValues(m, 2);
        for (int i = 0; i < m; i++) {
            measuredValues(i, 0) = pointsInModel[i].x;
            measuredValues(i, 1) = pointsInModel[i].y;

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

        baseToLaser_tf_.setIdentity();
        mapToOdom_tf_.setIdentity();
        odomToBase_tf_.setIdentity();

        map_frame_id_ = "map";
        odom_frame_id_ = "odom";
        base_frame_id_ = "base_link";




    };

    vector<geometry_msgs::PoseStamped> line_extraction::SimpleTriangleDetector::detect(){
        vector<geometry_msgs::PoseStamped> targets;
        // get lines


        if (marker_type_ == "light-belt"){
            auto lines = lsd_.getLines();
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
            auto lines = lsd_.getLines();

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
            //todo; get line segments as small as posiible, tune parameter
            // set up strong condition, detect order right to left





            std::cout<<"start get lines "<< std::endl;

            auto lines = lsd_.getLines(line_extraction::LineSegmentDetector::detectMode::segments);

            std::cout<<"get lines "<< lines.size()<< std::endl;

            // get 3 or 4 point
            // 3: flat board
            // 4 : shelf
            bool board_or_shelf ;

            // get params
#if 0
            nh_private_.param("points_array",params_);
#endif
            ros::param::getCached("~points_array",params_);
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





                time_util::Timer timer;
                timer.start();


                lsd_.getAngles(cache_angle_);

                // matrix shape

                Eigen::MatrixXd measureData(data_num,1);
                Eigen::MatrixXd model(2,data_num);
                Eigen::VectorXd x(data_num);

                // fill data
                for (int di = 0;di <data_num;di++){
                    auto id1 = lines[di].getIndices();
                    model(0,di) = params_[di]["x"];
                    model(1,di) = params_[di]["y"];
                }





                lsd_.getLaser(latestScan_);
                ros::Time tn = ros::Time::now();

#if 1
                // get base im map tf
                // transform model point to vase frame;
                bool gettf;
                std::cout<<"looking up tf "<< std::endl;

                // look up base to laser frame for once

                if(baseToLaser_tf_.getOrigin().x() == 0.0){
                    gettf = listener_.getTransform(base_frame_id_,latestScan_.header.frame_id,baseToLaser_tf_,tn,0.1,
                                                   true);

                }



                // get map odom tf
                bool gettf1 = listener_.getTransform(map_frame_id_,odom_frame_id_,mapToOdom_tf_,tn,0.1,
                                               false);


                // get odom base tf
                // look up odom to base tf
                bool gettf2 = listener_.getTransform(odom_frame_id_,base_frame_id_,odomToBase_tf_,tn,0.1,
                                               false);



                if ( ! gettf1 || ! gettf2 ){
                    return targets;
                }

                std::cout<<"get up tf  ok"<< std::endl;

#endif
                //get map to laser tf
                tf::Transform mapToLaser_tf = mapToOdom_tf_*odomToBase_tf_*baseToLaser_tf_;
                // get relative pose in laser frame

                decltype(model) origin_pos = model;

                eigen_util::TransformationMatrix2d trans_laser(mapToLaser_tf.getOrigin().x(), mapToLaser_tf.getOrigin().y(), tf::getYaw(mapToLaser_tf.getRotation()));
                model = trans_laser.inverse()*model;

                std::cout << "get model pose \n"<<model<<std::endl;

                // check markers base  on distance
                // new lines vector
                decltype(lines) new_lines ;
                Eigen::VectorXd model_m(2);

                model_m = model.rowwise().mean();
                type_util::Point2d model_point;
                model_point.x = model_m(0);
                model_point.y = model_m(1);
                for (int i = 0; i< lines.size();i++){

                    auto l = lines[i];
                    if (l.length() > max_marker_length_){
                        continue;
                    }
                    type_util::Point2d p;
                    p.x = 0.5*(l.getStart()[0] + l.getEnd()[0]);
                    p.y = 0.5*(l.getStart()[1] + l.getEnd()[1]);

                    double dist = geometry_util::PointToPointDistance(model_point,p);

                    if (dist > max_marker_initial_dist_){
                        continue;
                    }

                    new_lines.push_back(lines[i]);
                }

                if (new_lines.size() != data_num){
                    return targets;
                }

                for (int di = 0;di <data_num;di++){
                    auto id1 = lines[di].getIndices();
                    measureData(di,0) =  valarray<float>(cache_angle_[std::slice(id1[di],id1.size(),1)]).sum()/id1.size();
                }


                // initial guess
                x << 0.0, 0.0, 0.0;

                opt_util::SimpleSolver<opt_util::AngleFunctor> sm;
                sm.updataModel(model);

                sm.setParams(x);

                sm.feedData(measureData);

                int status = sm.solve();

                auto meanerror = sm.getMeanError();

                x = sm.getParam();

                std::cout<<"get x \n"<<x<<std::endl << "error "<<meanerror << std::endl;

                auto t = timer.elapsedSeconds();
                ROS_INFO("fit time %.4f",t);
                // check error
                //rule 1: mean error
                // rule 2: relative angle
                if (meanerror > max_fit_error_  ){
                    return targets;
                }

                // get transform position
                eigen_util::TransformationMatrix2d trans(x(0), x(1), x(2));
                std::cout << "get origin pose \n"<<origin_pos<<std::endl;

                decltype(model) pos = trans*model;
                std::cout << "get transform pose \n"<<pos<<std::endl;


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
                nh_private_.setParam("/amcl/tf_broadcast", false);

                // get map odom tf

                tf::Transform origin_pose_tf, pose_tf;
                tf::poseMsgToTF(origin_pose.pose,origin_pose_tf);
                tf::poseMsgToTF(pose.pose,pose_tf);
                mapToOdom_tf_ = origin_pose_tf*pose_tf.inverse()*baseToLaser_tf_.inverse()*odomToBase_tf_.inverse();

                geometry_msgs::PoseStamped map_odom_pose;
                tf::poseTFToMsg(mapToOdom_tf_, map_odom_pose.pose);

                targets.push_back(map_odom_pose);



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
            pubThread_(50,fake_pose_topic_,nh),
            tfThread_(20),
            smoothPose_(2),
            cmd_data_ptr_(std::make_shared<std_msgs::Header>())
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


        auto res = listener_.createSubcriber<std_msgs::Header>(cmd_topic_,1);
        cmd_data_ptr_ = std::get<0>(res);

        running_ = false;


        lighthouse_pose_topic_ = "lighthouse_pose";
        lighthouse_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(lighthouse_pose_topic_,1);








    }

    void line_extraction::TargetPublish::publish(){
        // broadcast base_link base_triangle tf
        // publish base_link pose in base_triangle
        // publish base_triangle pose in base_link
        // how to change state
        if(!running_){

            bool getmsg = listener_.getOneMessage(cmd_topic_,0.1);

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
            smoothPose_.getMean();

            //
            tf::Transform transform;
            tf::poseMsgToTF(pose.pose,transform);

#if 0
            transform.setOrigin(tf::Vector3(smoothPose_.mean_x_,smoothPose_.mean_y_,0.0));


            transform.setRotation(tf::createQuaternionFromYaw(smoothPose_.mean_yaw_));
#endif
            ros::Time tn = ros::Time::now();

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
                tfthreadClass_.syncArg(stampedTransform_);

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
                    ROS_ERROR("stop reflector localization");

                    nh_private_.setParam("/amcl/tf_broadcast", true);
                }
            };

        }

    };
#if 0
line_extraction::SimpleShelfDetector::SimpleShelfDetector(ros::NodeHandle nh, ros::NodeHandle nh_private) :line_extraction::SimpleTriangleDetector(nh,nh_private) {


    getDetectParams();
    initParams();



}

void line_extraction::SimpleShelfDetector::initParams() {

    // inital param
    nh_private_.param("min_front_len",min_front_len_,0.8);
    nh_private_.param("max_front_len",max_front_len_,1.2);
    min_front_len_ *= front_len_;
    max_front_len_ *= front_len_;

    nh_private_.param("min_side_len",min_side_len_,0.8);
    nh_private_.param("max_side_len",max_side_len_,1.2);
    min_side_len_ *= side_len_;
    max_side_len_ *= side_len_;

    nh_private_.param("min_90_",min_90_,0.8);
    nh_private_.param("max_90_",max_90_,1.2);

    min_90_ *= 0.5*M_PI;
    max_90_ *= 0.5*M_PI;

    nh_private_.param("min_front_angle",min_front_angle_,0.4*M_PI);
    nh_private_.param("max_front_angle",max_front_angle_,0.6*M_PI);

    nh_private_.param("max_front_x",max_front_x_,1.5);
    nh_private_.param("max_front_y",max_front_y_,0.3);



    nh_private_.param("min_full_detect_dist",min_full_detect_dist_,2.0);
    nh_private_.param("min_match_radius",min_match_radius_,0.1);


}

void line_extraction::SimpleShelfDetector::getDetectParams() {

    // get desk params

    // only two param

    nh_private_.param("front_len", front_len_, 0.5);
    nh_private_.param("side_len", side_len_, 0.5);


}

bool line_extraction::SimpleShelfDetector::fitModel(vector<type_util::Point2d> &pointsInModel, double x0, double x1,
                                                    double x2, geometry_msgs::PoseStamped &ModelPose) {

    return false;
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
    Eigen::MatrixXd measuredValues(m, 2);
    for (int i = 0; i < m; i++) {
        measuredValues(i, 0) = pointsInModel[i].x;
        measuredValues(i, 1) = pointsInModel[i].y;

        pose.position.x = pointsInModel[i].x;
        pose.position.y = pointsInModel[i].y;

        targetPoints_.poses.push_back(pose);
    }

    Eigen::VectorXd x(4);
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





// detect shelf
// from scratch : strict mode ,must match specific pattern
// keep state, avoid detect from scratch
// task complete condition successful or failed
//
//


// publish posestaamped : position + frameid
// detect obscale
/*1.detect 2 leg
 * 2. track and detect
 * 3. base on desk position choose to detect 4 leg
 * before laser enter desk, avoid losing  track of back leg
 *
 * */

vector<geometry_msgs::PoseStamped> line_extraction::SimpleShelfDetector::detect() {
    std::cout<<"SimpleShelfDetector"<<std::endl;
    vector<geometry_msgs::PoseStamped> targets;
#if 0
    SimpleTriangleDetector::detect();
#endif
    // keep track of detection stage
    // stage 0 : in front of entry, first detect
    // stage 1 : keep track of each leg


    // get all cluster with a small length
    auto lines = lsd_.getLines(line_extraction::LineSegmentDetector::detectMode::segments);
    // if get no lines ,return
    if (lines.empty()){

        return targets;
    }

    // detect stage: 0 initial detect , 1 detect ok
    // detect 2 or 4
    std::vector<shelfDetectStage> shelfVec(20,shelfDetectStage(front_len_,side_len_));
    shelfVec.clear();
    if (latest_shelf_.stage_ == 0){
        // first detect legs base on order: f1,f2 is required , b1, b2 is optional

        // iter on all cluster
        // get 2,4,0 legs ,not 3

        // first find 2

        // if find 2 find 4

        if(lines.size() >= 4 ){
            for(int i=0; i< lines.size();i++){
                for(int j=i+1;j < lines.size();j++){

                    auto l1 = lines[i];
                    auto l2 = lines[j];
                    type_util::Point2d
                            l1_s(l1.getStart()[0], l1.getStart()[1]),
                            l1_e(l1.getEnd()[0],l1.getEnd()[1]),
                            l2_s(l2.getStart()[0], l2.getStart()[1]),
                            l2_e(l2.getEnd()[0],l2.getEnd()[1]);

                    type_util::Point2d l1_c(0.5*(l1_s.x + l1_e.x),0.5*(l1_s.y + l1_e.y));
                    type_util::Point2d l2_c(0.5*(l2_s.x + l2_e.x),0.5*(l2_s.y + l2_e.y));
                    type_util::Point2d front_c(0.5*(l2_c.x + l1_c.x),0.5*(l2_c.y + l1_c.y));

                    // get line between two lines's center
                    double dist = geometry_util::PointToPointDistance(l1_c,l2_c);

                    double angle21 = atan2(l2_c.y - l1_c.y, l2_c.x - l1_c.x);

                    // get length

                    // get angle

                    bool cond1 = dist > min_front_len_ && dist < max_front_len_;
                    bool cond2 = angle21 > min_front_angle_ && angle21 < max_front_angle_;
                    bool cond3 = fabs(front_c.y) < max_front_y_;
                    bool cond4 = front_c.x < max_front_x_;

                    // match all condition
                    if(cond1&&cond2&&cond3&&cond4){
                        // find b1 b2
                        //
                        if (j - i >= 3){
                            for(int k = i+1;k<j-1;k++){
                                auto l3 = lines[k];
                                type_util::Point2d
                                        l3_s(l3.getStart()[0], l3.getStart()[1]),
                                        l3_e(l3.getEnd()[0],l3.getEnd()[1]);
                                type_util::Point2d l3_c(0.5*(l3_s.x + l3_e.x),0.5*(l3_s.y + l3_e.y));

                                // check condition
                                double dist_31 = geometry_util::PointToPointDistance(l3_c,l1_c);
                                double angle312 = angle21 - atan2(l3_c.y - l1_c.y, l3_c.x - l1_c.x);
                                bool cond5 = angle312 > min_90_ && angle312 < max_90_;
                                bool cond7 = dist_31>min_side_len_ && dist_31 < max_side_len_;



                                // find 3
                                if(cond7&&cond5){
                                    for(int l = k+1; l< j;l++){
                                        auto l4 = lines[k];
                                        type_util::Point2d
                                                l4_s(l4.getStart()[0], l4.getStart()[1]),
                                                l4_e(l4.getEnd()[0],l4.getEnd()[1]);
                                        type_util::Point2d l4_c(0.5*(l4_s.x + l4_e.x),0.5*(l4_s.y + l4_e.y));

                                        double dist_42 = geometry_util::PointToPointDistance(l4_c,l2_c);
                                        double angle421 = angle21 - atan2(l4_c.y - l2_c.y, l4_c.x - l2_c.x);
                                        bool cond6 = angle421 > min_90_ && angle421 < max_90_;
                                        bool cond8 = dist_42 > min_side_len_ && dist_42 < max_side_len_;

                                        if (cond6 && cond8){
                                            shelfDetectStage s(front_len_, side_len_);
                                            s.f1_ = l1_c;
                                            s.f2_ = l2_c;
                                            s.b1_ = l3_c;
                                            s.b2_ = l4_c;
                                            s.setValidF1(1,i);
                                            s.setValidF1(1,j);
                                            s.setValidF1(1,k);
                                            s.setValidF1(1,l);



                                        }


                                    }
                                }


                            }

                        }



                    }

                }
            }






        }


    }else if(latest_shelf_.stage_ == 1){
        // check position
        // if not in desk, only f1, f2 is reauired
        // if in desk [at min_dist ], b1, b2 is required
    }

    if(!shelfVec.empty()){
        lsd_.getLaser(latestScan_);
//        cacheData();
        auto ranges = container_util::createValarrayFromVector(latestScan_.ranges);
        auto xs = ranges*cache_cos_;
        auto ys = ranges*cache_sin_;

        // sort score
        // find best match
        std::sort(shelfVec.begin(), shelfVec.end(),[](const shelfDetectStage &v1, const shelfDetectStage &v2){ return v2.getScore()>v1.getScore(); });

        auto bestShelf = shelfVec[0];
        type_util::Point2d pos;
        double yaw;
        bestShelf.getPosition(pos, yaw);

        // optimazition
        ROS_ERROR("get shelf at [%.3f,%.3f],%.3f",pos.x, pos.y, yaw);


        // fit model
        std::vector<type_util::Point2d> pointsInScan;

        // prepare data

        type_util::Point2d p;
        for (int i=0; i <4 ; i++){
            if(bestShelf.valid_[i] == 1){
                auto ids = lines[bestShelf.segment_id_[i]].getIndices();

                for(auto id:ids){
                    p.x = xs[id];
                    p.y = ys[id];
                    pointsInScan.push_back(p);

                }
            }
        }

        // get easteam result

        bestShelf.getPosition(pos,yaw);

        geometry_msgs::PoseStamped pose;
        fitModel(pointsInScan,pos.x,pos.y,yaw,pose);
    }




    //
    std::cout<<min_gap_dist_<<std::endl;

    // sort legs according to distance
//    std::sort(lines.begin(),lines.end(),LineRangeComp());
    // search from lea
    //findFourside();



    for(int i=0;i<lines.size();i++){
        // first find two front leg
        for(int j=i+1;j<lines.size();j++){
            auto l1 = lines[i];
            auto l2 = lines[j];
            type_util::Point2d
                    l1_s(l1.getStart()[0], l1.getStart()[1]),
                    l1_e(l1.getEnd()[0],l1.getEnd()[1]),
                    l2_s(l2.getStart()[0], l2.getStart()[1]),
                    l2_e(l2.getEnd()[0],l2.getEnd()[1]),
                    intersect;
            type_util::Point2d l1_c(0.5*(l1_s.x + l1_e.x),0.5*(l1_s.y + l1_e.y));
            type_util::Point2d l2_c(0.5*(l2_s.x + l2_e.x),0.5*(l2_s.y + l2_e.y));
            type_util::Point2d front_c(0.5*(l2_c.x + l1_c.x),0.5*(l2_c.y + l1_c.y));

            // get line between two lines's center
            double dist = geometry_util::PointToPointDistance(l1_c,l2_c);

            double angle21 = atan2(l2_c.y - l1_c.y, l2_c.x - l1_c.x);

            // get length

            // get angle

            bool cond1 = dist > min_front_len_ && dist < max_front_len_;
            bool cond2 = angle21 > min_front_angle_ && angle21 < max_front_angle_;
            bool cond3 = fabs(front_c.y) < max_front_y_;
            bool cond4 = front_c.x < max_front_x_;


            // if have found one shelf
            latest_shelf_.findF1(l1_c,0.05);
            latest_shelf_.findF2(l2_c,0.05);


            // find front leg
            if ( (latest_shelf_.valid_[2] == 1  && latest_shelf_.valid_[3] == 1 )|cond1&&cond2&&cond3&&cond4){

                // try to find back leg
                shelfDetectStage s ;
                if (cond1&&cond2&&cond3&&cond4){
                    s.f1_ = l1_c;
                    s.f2_ = l2_c;
                    s.setValidF1(1);
                    s.setValidF2(1);

                }else{
                    if (latest_shelf_.findF1(l1_c,0.05)){
                        s.f1_ = l1_c;
                        s.setValidF1(1);


                    }
                    if (latest_shelf_.findF1(l2_c,0.05)){
                        s.f1_ = l2_c;
                        s.setValidF1(1);


                    }
                    if (latest_shelf_.findF2(l1_c,0.05)){
                        s.f2_ = l1_c;
                        s.setValidF2(1);


                    }
                    if (latest_shelf_.findF2(l1_c,0.05)){
                        s.f2_ = l1_c;
                        s.setValidF2(1);


                    }

                }

                for (int k = i ;k <= j;k++){
                    // b1 and b2
                    auto l3 = lines[k];
                    type_util::Point2d
                            l3_s(l3.getStart()[0], l3.getStart()[1]),
                            l3_e(l3.getEnd()[0],l3.getEnd()[1]);
                    type_util::Point2d l3_c(0.5*(l3_s.x + l3_e.x),0.5*(l3_s.y + l3_e.y));

                    // check condition
                    double dist_31 = geometry_util::PointToPointDistance(l3_c,l1_c);
                    double dist_32 = geometry_util::PointToPointDistance(l3_c,l2_c);

                    double angle312 = angle21 - atan2(l3_c.y - l1_c.y, l3_c.x - l1_c.x);

                    double angle321 = angle21 - atan2(l3_c.y - l2_c.y, l3_c.x - l2_c.x);

                    bool cond5 = angle312 > 0.4*M_PI && angle312 < 0*6*M_PI;
                    bool cond6 = dist_31>min_front_len_ && dist_31 < max_front_len_;

                    bool cond7 = dist_32 > min_front_len_ && dist_32 < max_front_len_;

                    bool cond8 = true;

                    bool cond9;
                    bool cond10;


                    if (cond5 && cond6 && cond7){
                        s.b1_ = l3_c;
                        s.setValidB1(1);
                        // update vec, compute score
                        s.score_ = 0.0;

                        shelfVec.push_back(s);
                        continue;
                    }

                    if(cond8 && cond9 && cond1){
                        s.b2_ = l3_c;
                        s.setValidB2(1);
                        // update vec, compute score
                        s.score_ = 0.0;

                        shelfVec.push_back(s);
                        break;
                    }

                    // update vec, compute score
                    s.score_ = 0.0;

                    shelfVec.push_back(s);
                }








                //decide wheather to fing back leg



            }







        }




    }


    std::sort(shelfVec.begin(), shelfVec.end(),[](const shelfDetectStage &v1, const shelfDetectStage &v2){ return v2.score_>v1.score_; });








    return targets;



}
#endif



