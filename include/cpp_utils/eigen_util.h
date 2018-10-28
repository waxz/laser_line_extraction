//
// Created by waxz on 9/26/18.
//

#ifndef LASER_LINE_EXTRACTION_EIGEN_UTIL_H
#define LASER_LINE_EXTRACTION_EIGEN_UTIL_H
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eigen_util{


    Eigen::Matrix2d getRotation2dFromYaw(double yaw, bool cw = true){
        //yaw is a counterclockwise rotation of alpha about the zaxis
        Eigen::Matrix2d rotation;

        rotation << cos(yaw), ((cw)? -1 : 1 )*sin(yaw),
                ((cw)? 1 : -1 )*sin(yaw),cos(yaw);

        return rotation;

        /*
     * http://planning.cs.uiuc.edu/node102.html
     * A yaw is a counterclockwise rotation of alpha about the zaxis. The rotation matrix is given by


     R_z(alpha) = [
                  cos(alpha) -sin(alpha) 0
                  sin(alpha) sin(alpha)  0
                  0          0           1
                  ]


     * */

    }

    Eigen::Matrix3d getTransform2d(double x, double y, double yaw){
        //http://f1tenth.org/slides/l3-1.pdf
        Eigen::Matrix3d trans;
        trans.setZero();
        Eigen::Vector3d t;
        t<< x, y, 1.0;
        trans.block(0,0,2,2) = getRotation2dFromYaw(yaw);
        trans.block(0,2,3,1) = t;

        return trans;


    }


    class TransformationMatrix2d{
    private:
        Eigen::Matrix3d trans_;

    public:

        TransformationMatrix2d(double x, double y, double yaw){
            trans_ = getTransform2d(x, y, yaw);
        }

        TransformationMatrix2d& operator = (const Eigen::Matrix3d &matrix){
            trans_ = matrix;
            return *this;
        }

        TransformationMatrix2d (const Eigen::Matrix3d &matrix){
            trans_ = matrix;
        }

        Eigen::Vector2d operator *(  Eigen::Vector2d x){
#if 0
            Eigen::Vector3d x_;
            x_ << x(0), x(1), 1.0;

            auto res = trans_*x_;

            x(0) = res(0);
            x(1) = res(1);
            return x;
#endif
#if 1
            Eigen::Vector3d x_;
            x_.setOnes();
            x_.segment(0,2) = x;


            return (trans_*x_).head(2);
#endif

        }
        Eigen::MatrixXd operator *( const Eigen::MatrixXd &x){
            // x : 2d col vector 2*n
            Eigen::MatrixXd x_(3, x.cols());
            x_.setOnes();
            x_.block(0,0,2, x.cols()) = x;


            return (trans_*x_).block(0,0,2, x.cols());



        }

        Eigen::MatrixXd matrix(){
            return trans_;
        }

        TransformationMatrix2d operator *(TransformationMatrix2d rv){
            TransformationMatrix2d res(this->trans_*rv.matrix());
            return res ;
        }

        TransformationMatrix2d inverse(){
            TransformationMatrix2d res(this->trans_.inverse());
            return res ;
        }

    };

    inline Eigen::MatrixXd getDistMatrix(const Eigen::MatrixXd &m) {

        int sz = m.cols();
        Eigen::MatrixXd M(sz, sz);
        M.setZero();
        for (int i = 0; i < sz; i++) {
            for (int j = i + 1; j < sz; j++) {
                M(i, j) = (m.col(i) - m.col(j)).norm();
            }
        }

        return M;
    }
}



#endif //LASER_LINE_EXTRACTION_EIGEN_UTIL_H
