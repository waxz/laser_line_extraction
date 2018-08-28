#include <laser_line_extraction/line_detector.h>


// get laser data
// mask valid data
// extract line
// choose valid data

line_extraction::LineSegmentDetector::LineSegmentDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
nh_(nh),
nh_private_(nh_private),
line_extraction::LineExtractionROS(nh, nh_private),
listener(nh, nh_private){
    initParam();


    auto res = listener.createSubcriber<sensor_msgs::LaserScan>(this->scan_topic_,1);

    scan_data_ = std::get<0>(res);
//        listener.getOneMessage(scan_topic_,-1);


}
void line_extraction::LineSegmentDetector::initParam(){
    nh_private_.param("max_range",max_range_,1.0);
    nh_private_.param("min_angle",angle_min_,-0.5*M_PI);
    nh_private_.param("max_angle",angle_max_,0.5*M_PI);


};
void line_extraction::LineSegmentDetector::processData(){
    // mask ranges
    // set range > max_range to 0
    auto msg = *scan_data_;

    valarray<float> r = container_util::createValarrayFromVector(msg.ranges);
    r[r>float(max_range_)] = 0.0;


    msg.ranges = container_util::createVectorFromValarray(r);

    boost::shared_ptr<sensor_msgs::LaserScan> scan_ptr_(boost::make_shared<sensor_msgs::LaserScan>(msg) );

    this->laserScanCallback(scan_ptr_);
}

std::vector<line_extraction::Line> line_extraction::LineSegmentDetector::getLines(){
    lines_.clear();

    // todo:debug with block
    bool getMsg;
    if(debug_mode_){
        getMsg = listener.getOneMessage(this->scan_topic_,-1);

    }else{
        getMsg = listener.getOneMessage(this->scan_topic_,0.05);

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


        //triangle_direction



    };

    void line_extraction::SimpleTriangleDetector::cacheData(){
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

        Eigen::VectorXd x(4);
        decltype(x) model(1);

        x(0) = x0;             // initial value for 'a'
        x(1) = x1;             // initial value for 'b'
        x(2) = x2;             // initial value for 'c'
        x(3) =  M_PI*120.0/180.0;
        model(0) = triangle_direction_;
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
            lsd_(nh,nh_private){
        initParams();



        targetPub_ = nh_.advertise<geometry_msgs::PoseStamped>("targetPose",1);
        pointsPub_ = nh_.advertise<geometry_msgs::PoseArray>("targetPoints",1);



    };

    vector<geometry_msgs::PoseStamped> line_extraction::SimpleTriangleDetector::detect(){
        vector<geometry_msgs::PoseStamped> targets;
        // get lines
        auto lines = lsd_.getLines();

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


        return targets;


    }


#endif


namespace line_extraction{

}


line_extraction::TargetPublish::TargetPublish(ros::NodeHandle nh, ros::NodeHandle nh_private):
            nh_(nh),
            nh_private_(nh_private),
            sd_(nh,nh_private),
            fake_pose_topic_("triangle_pose"),
            pubThread_(50,fake_pose_topic_,nh),
            tfThread_(20),
            listener_(nh,nh_private),
            smoothPose_(10)
    {
        pubthreadClass_.setTarget(pubThread_);
        tfthreadClass_.setTarget(tfThread_);

        base_frame_id_ = "base_link";
        laser_frame_id_ = "base_laser";
        target_framde_id_ = "base_triangle";

        fake_pose_topic_ = "triangle_pose";

        baseToLaser_tf_.setIdentity();

        nh_private_.param("expire_sec",expire_sec_,5);
        lastOkTime_ = ros::Time::now();

        nh_private_.param("broadcast_tf",broadcast_tf_, false);
        nh_private_.param("pub_pose",pub_pose_, true);







    }

    void line_extraction::TargetPublish::publish(){
        auto targets = sd_.detect();
        if(targets.size() == 1){
            // todo : bypass in debug model ;get base to laser tf
#if 1
            if(baseToLaser_tf_.getOrigin().x() == 0.0){
                bool gettf = listener_.getTransform(base_frame_id_,laser_frame_id_,baseToLaser_tf_,ros::Time::now(),0.1,
                                                    false);
                if(!gettf){
                    return;
                }
            }
#endif
            // publish
            auto pose = targets[0];

            smoothPose_.update(pose.pose.position.x,pose.pose.position.y, tf::getYaw(pose.pose.orientation));

#if 0

            if (!smoothPose_.full()){
               return;

            }
#endif
            smoothPose_.getMean();

            //
            tf::Transform transform;
            tf::poseMsgToTF(pose.pose,transform);

            transform.setRotation(tf::createQuaternionFromYaw(smoothPose_.mean_yaw_));
            transform.setOrigin(tf::Vector3(smoothPose_.mean_x_,smoothPose_.mean_y_,0.0));

#if 1

            ros::Time tn = ros::Time::now();
            ros::Duration transform_tolerance;
            transform_tolerance.fromSec(0.1);
            ros::Time transform_expiration = (tn + transform_tolerance);
            tf::StampedTransform stampedTransform = tf::StampedTransform(baseToLaser_tf_*transform,
                                                                       transform_expiration,
                                                                       base_frame_id_, target_framde_id_);
            tfthreadClass_.syncArg(stampedTransform);

#endif
            triangle_pose_.header.stamp = tn;
            triangle_pose_.header.frame_id = target_framde_id_;
            tf::poseTFToMsg((baseToLaser_tf_*transform).inverse(),triangle_pose_.pose);
            pubthreadClass_.syncArg(triangle_pose_);
            lastOkTime_ = tn;



            if ( broadcast_tf_&&!tfthreadClass_.isRunning()){
                tfthreadClass_.start();
            }
            if (pub_pose_&&!pubthreadClass_.isRunning()){
                pubthreadClass_.start();
            }
        }else{
            if(!smoothPose_.full()){
                return;
            }
            ros::Time tn = ros::Time::now();

            auto dur = tn -lastOkTime_;
            if(dur.toSec()<expire_sec_){
                tf::Transform tranform_change,triangle_tf;
                tranform_change.setIdentity();
                bool get = listener_.getTransformChange("odom",base_frame_id_,tranform_change,
                                             triangle_pose_.header.stamp,tn,0.1,false);


                tf::poseMsgToTF(triangle_pose_.pose,triangle_tf);
                tf::poseTFToMsg(triangle_tf*tranform_change,triangle_pose_.pose);
                triangle_pose_.header.stamp = tn;
                pubthreadClass_.syncArg(triangle_pose_);

            }else{
                smoothPose_.clear();
                pubthreadClass_.pause();
            };






#if 0
            smoothPose_.clear();
            triangle_pose_.pose.position.x = 100000;
            triangle_pose_.pose.position.y = 100000;

            pubthreadClass_.syncArg(triangle_pose_);
#endif
        }

    };



