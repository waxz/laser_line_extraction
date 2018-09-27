//
// Created by waxz on 8/17/18.
//

#ifndef LASER_LINE_EXTRACTION_GEOMETRY_H
#define LASER_LINE_EXTRACTION_GEOMETRY_H
#include <iostream>
#include <cmath>
#include <cpp_utils/types.h>


//source : https://rosettacode.org/wiki/Find_the_intersection_of_two_lines#C.2B.2B
namespace geometry_util{
    /** Calculate determinant of matrix:
	[a b]
	[c d]
*/
    inline double Det(double a, double b, double c, double d)
    {
        return a*d - b*c;
    }

///Calculate intersection of two lines.
///\return true if found, false if not found or error
    bool LineLineIntersect(double x1, double y1, //Line 1 start
                           double x2, double y2, //Line 1 end
                           double x3, double y3, //Line 2 start
                           double x4, double y4, //Line 2 end
                           double &ixOut, double &iyOut) //Output
    {
        using namespace std;
        double detL1 = Det(x1, y1, x2, y2);
        double detL2 = Det(x3, y3, x4, y4);
        double x1mx2 = x1 - x2;
        double x3mx4 = x3 - x4;
        double y1my2 = y1 - y2;
        double y3my4 = y3 - y4;

        double xnom = Det(detL1, x1mx2, detL2, x3mx4);
        double ynom = Det(detL1, y1my2, detL2, y3my4);
        double denom = Det(x1mx2, y1my2, x3mx4, y3my4);
        if(denom == 0.0)//Lines don't seem to cross
        {
            cerr<<"Error, Lines don't seem to cross! "<<endl;
            ixOut = NAN;
            iyOut = NAN;
            return false;
        }

        ixOut = xnom / denom;
        iyOut = ynom / denom;
        if(!std::isfinite(ixOut) || !std::isfinite(iyOut)) //Probably a numerical issue
        {
            cerr<<"Error, Probably a numerical issue !"<<endl;

            return false;

        }
//        cout<<"Ok, get intersection: "<<ixOut<<","<<iyOut<<endl;


        return true; //All OK
    }


    bool LineLineIntersect(type_util::Point2d L1_start, //Line 1 start
                           type_util::Point2d L1_end, //Line 1 end
                           type_util::Point2d L2_start, //Line 2 start
                           type_util::Point2d L2_end, //Line 2 end
                           type_util::Point2d &intersect) //Output
    {
        return LineLineIntersect(L1_start.x,L1_start.y,L1_end.x,L1_end.y,
        L2_start.x,L2_start.y,L2_end.x,L2_end.y,intersect.x,intersect.y);
    }

    double PointToPointDistance(type_util::Point2d p1,type_util::Point2d p2){
        return sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2));
    }

    bool PointToLineDistance(type_util::Point2d L1_start, //Line 1 start
                               type_util::Point2d L1_end, //Line 1 end
                               type_util::Point2d p1, double & dist){

        double l1_yaw = atan2(L1_end.y - L1_start.y, L1_end.x - L1_start.x);
        double l2_yaw = l1_yaw + 0.5*M_PI;

        auto p2 = p1;
        p2.x = p1.x + 0.1*cos(l2_yaw);
        p2.y = p1.y + 0.1*sin(l2_yaw);


        bool status = LineLineIntersect(L1_start,L1_end,p1,p2,p2);
        dist = PointToPointDistance(p1,p2);
        return status;





    }
    double NormalAngle(double angle){
        return atan2(sin(angle),cos(angle));
    }

    bool getPedal(type_util::Point2d L1_start, //Line 1 start
                                type_util::Point2d L1_end, //Line 1 end
                                type_util::Point2d p1, type_util::Point2d &pedal){
        double yaw1 = atan2(L1_end.y - L1_start.y , L1_end.x - L1_start.x);
        double yaw2 = yaw1 + 0.5*M_PI;
        type_util::Point2d p2;
        p2.x = p1.x + cos(yaw2);
        p2.y = p1.y + sin(yaw2);

        return LineLineIntersect(L1_start,L1_end,p1,p2,pedal);
    }

    // ridged transformation

    type_util::Quaterniond toQuaternion(double pitch, double roll, double yaw)
    {
        type_util::Quaterniond q;
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);

        q.w = cy * cr * cp + sy * sr * sp;
        q.x = cy * sr * cp - sy * cr * sp;
        q.y = cy * cr * sp + sy * sr * cp;
        q.z = sy * cr * cp - cy * sr * sp;
        return q;
    }

    void toEulerAngle(const type_util::Quaterniond& q, double& roll, double& pitch, double& yaw)
    {
        // roll (x-axis rotation)
        double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = +2.0 * (q.w * q.y - q.z * q.x);
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw = atan2(siny_cosp, cosy_cosp);
    }
}



#endif //LASER_LINE_EXTRACTION_GEOMETRY_H
