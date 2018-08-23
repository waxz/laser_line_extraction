//
// Created by waxz on 18-7-21.
//

#ifndef LOCATE_REFLECTION_LEVMARQ_H
#define LOCATE_REFLECTION_LEVMARQ_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>

#include "types.h"
#include <unsupported/Eigen/NonLinearOptimization>

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;


#include <string>
#include <iostream>
#include <vector>

namespace opt_util {
    using std::cout;
    using std::endl;
    using std::vector;

// generator
// given 3 parameter [x0,y0,yaw1], (x,y) as turn point, yaw as one side angle
// for x<x0, y = k1x+b, for x>x0 , y= k2x+b
    Point2DVector SampleGen(double start, double end, int num, double random_ratio = 0.0) {
        double x0, y0, k0, k1, k2, angle;
        // line function ax + by = c
        angle = 0.5 * M_PI;
        double yaw0 = 0.0*M_PI;
        k1 = tan(yaw0 + 0.5 * angle);
        k2 = tan(yaw0 - 0.5 * angle);
        x0 = 1.123;
        y0 = 0.0;
        printf("real param \n x0: %f,y0: %f,yaw0: %f",x0,y0,yaw0);

        // sample range
        Point2DVector points;

        for (int cnt = 0; cnt < num; cnt++) {
            double x = static_cast<double>(start + cnt * (end - start) / num );
            Eigen::Vector2d point;
            point(1) = x;
            double noise = random_ratio * drand48() / 100.0;
            point(0) = (x < y0) ? x0 + (x - y0) / k1 + noise : x0 + (x - y0) / k2 + noise;
            //2.0 * x + 5.0 + drand48() / 10.0;
            points.push_back(point);


        }


        return points;


    }


    struct SimpleFunctor {

        // Compute 'm' errors, one for each data point, for the given parameter values in 'x'
        virtual int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const = 0;

        virtual void updataModel(const Eigen::VectorXd &model) = 0;
        // 'm' pairs of (x, f(x))
        Eigen::MatrixXd measuredValues;



        // Compute the jacobian of the errors
        int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const {
            // 'x' has dimensions n x 1
            // It contains the current estimates for the parameters.

            // 'fjac' has dimensions m x n
            // It will contain the jacobian of the errors, calculated numerically in this case.

            double epsilon;
            epsilon = 1e-5f;

            for (int i = 0; i < x.size(); i++) {
                Eigen::VectorXd xPlus(x);
                xPlus(i) += epsilon;
                Eigen::VectorXd xMinus(x);
                xMinus(i) -= epsilon;

                Eigen::VectorXd fvecPlus(values());
                operator()(xPlus, fvecPlus);

                Eigen::VectorXd fvecMinus(values());
                operator()(xMinus, fvecMinus);

                Eigen::VectorXd fvecDiff(values());
                fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

                fjac.block(0, i, values(), 1) = fvecDiff;
            }

            return 0;
        }

        // Number of data points, i.e. values.
        int m;

        // Returns 'm', the number of values.
        int values() const { return m; }

        // The number of parameters, i.e. inputs.
        int n;

        // Returns 'n', the number of inputs.
        int inputs() const { return n; }

        void feedData(const Eigen::MatrixXd &data) {
            measuredValues = data;
            m = static_cast<int>(data.rows());

        }

    };

    template<class T>
    struct SimpleSolver {
        T functor_;
        Eigen::VectorXd x_;

        void setParams(const Eigen::VectorXd &x) {
            // 'x' is vector of length 'n' containing the initial values for the parameters.
            // The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
            // The LM optimization inputs should not be confused with the x input values.
            x_ = x;
            functor_.n = x.rows();
        }

        void feedData(const Eigen::MatrixXd &data) {
            functor_.feedData(data);

        }

        void updataModel(const Eigen::VectorXd &model) {
            functor_.updataModel(model);
        }

        int solve() {
            // Run the LM optimization
            // Create a LevenbergMarquardt object and pass it the functor_.
            Eigen::LevenbergMarquardt<T, double> lm(functor_);
            int status = lm.minimize(x_);
            printf("LM optimization status: %d\n", status);
            return status;
        }

        Eigen::VectorXd getParam() {
            return x_;
        }
        double getMeanError(){
            Eigen::VectorXd fvec(functor_.m);
            functor_(x_,fvec);
            double meanerror = fvec.array().abs().mean();
            return meanerror;


        }

    };

    struct LineFunctor : SimpleFunctor {
        // define model param
        double paramAngle_;
        bool modelUpdated_;

        LineFunctor() {
            paramAngle_ = 0.0;
            modelUpdated_ = false;
        }

        //update model param
        void updataModel(const Eigen::VectorXd &model) {
            paramAngle_ = model(0);
            modelUpdated_ = true;
        }

        // predict
        int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
            if(!modelUpdated_){
                printf("model not updated\n");
            }
            // 'x' has dimensions n x 1
            // It contains the current estimates for the parameters.

            // 'fvec' has dimensions m x 1
            // It will contain the error for each data point.

            double x0 = x(0);
            double y0 = x(1);
            double yaw0 = x(2);
            double k1, k2;

            double turnAngle = 0.25*M_PI;//paramAngle_ - 0.5*M_PI;



            // fix: assuption error ,the model may be   x = f(y)
            // or y = f(x)

            double inputValue , predictValue;
            // x = f(y)
            if(fabs(yaw0)<=turnAngle || fabs(yaw0 - M_PI)<=turnAngle){
                if(fabs(yaw0)<0.5*M_PI){
                    k1 = tan(yaw0 + 0.5 * paramAngle_);
                    k2 = tan(yaw0  - 0.5 * paramAngle_);
                }else if(fabs(yaw0)>0.5*M_PI){
                    k1 = tan(yaw0 - 0.5 * paramAngle_);
                    k2 = tan(yaw0  + 0.5 * paramAngle_);
                }




                for (int i = 0; i < values(); i++) {
                    inputValue = measuredValues(i, 1);
                    predictValue = measuredValues(i, 0);

                    // error = y_true - y_predict
                    fvec(i) = predictValue - ((inputValue < y0) ? x0 + (inputValue - y0) / k1 : x0 + (inputValue - y0) / k2);
                }
            } else {
                // y = f(x)
                if(yaw0 > turnAngle){
                    k1 = tan(yaw0 - 0.5 * paramAngle_);
                    k2 = tan(yaw0  + 0.5 * paramAngle_);
                } else if(yaw0 < -turnAngle){
                    k1 = tan(yaw0 + 0.5 * paramAngle_);
                    k2 = tan(yaw0  - 0.5 * paramAngle_);
                }



                for (int i = 0; i < values(); i++){
                    inputValue = measuredValues(i, 0);
                    predictValue = measuredValues(i, 1);

                    // error = y_true - y_predict
                    fvec(i) = predictValue - ((inputValue < x0) ? y0 + (inputValue - x0) * k1 : y0 + (inputValue - x0) * k2);
                }


            }




            return 0;
        }


    };

    struct VFunctor : SimpleFunctor {
        // define model param
        bool modelUpdated_;

        VFunctor() {
            modelUpdated_ = false;
        }

        //update model param
        void updataModel(const Eigen::VectorXd &model) {
            modelUpdated_ = true;
        }

        // predict
        int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
            // 'x' has dimensions n x 1
            // It contains the current estimates for the parameters.

            // 'fvec' has dimensions m x 1
            // It will contain the error for each data point.

            double x0 = x(0);
            double y0 = x(1);
            double yaw0 = x(2);
            double angle = x(3);
            double k1, k2;

            double turnAngle = 0.25*M_PI;//paramAngle_ - 0.5*M_PI;

            double angle_1, angle_2;


            // fix: assuption error ,the model may be   x = f(y)
            // or y = f(x)

            double inputValue , predictValue;
            // x = f(y)
            if(fabs(yaw0)<=turnAngle || fabs(yaw0 - M_PI)<=turnAngle){
                if(fabs(yaw0)<0.5*M_PI){
                    angle_1 = yaw0 + 0.5 * angle;
                    angle_2 = yaw0  - 0.5 * angle;
                }else if(fabs(yaw0)>0.5*M_PI){
                    angle_1 = yaw0 - 0.5 * angle;
                    angle_2 = yaw0  + 0.5 * angle;
                }
                k1 = tan(angle_1);
                k2 = tan(angle_2);





                for (int i = 0; i < values(); i++) {
                    inputValue = measuredValues(i, 1);
                    predictValue = measuredValues(i, 0);

                    // error = y_true - y_predict
                    fvec(i) = predictValue - ((inputValue < y0) ? x0 + (inputValue - y0) / k1 : x0 + (inputValue - y0) / k2);
//                    fvec(i) = (inputValue < y0) ? predictValue - (x0 + (inputValue - y0) / k1)*fabs(sin(angle_1)) : predictValue - (x0 + (inputValue - y0) / k2)*fabs(sin(angle_2));

                }
            } else {
                // y = f(x)
                if(yaw0 > turnAngle){
                    angle_1 = yaw0 - 0.5 * angle;
                    angle_2 = yaw0  + 0.5 * angle;
                } else if(yaw0 < -turnAngle){
                    angle_1 = yaw0 + 0.5 * angle;
                    angle_2 = yaw0  - 0.5 * angle;
                }
                k1 = tan(angle_1);
                k2 = tan(angle_2);


                for (int i = 0; i < values(); i++){
                    inputValue = measuredValues(i, 0);
                    predictValue = measuredValues(i, 1);

                    // error = y_true - y_predict
                    fvec(i) = predictValue - ((inputValue < x0) ? y0 + (inputValue - x0) * k1 : y0 + (inputValue - x0) * k2);
//                    fvec(i) = (inputValue < x0) ? (predictValue - (y0 + (inputValue - x0) * k1))*fabs(cos(angle_1)) : (predictValue - (y0 + (inputValue - x0) * k2))*fabs(cos(angle_2));

                }


            }




            return 0;
        }


    };

}

#if 0
Eigen::MatrixXd measuredValues(m, 2);
    for (int i = 0; i < m; i++) {
        measuredValues(i, 0) = Points[i](0);
        measuredValues(i, 1) = Points[i](1);
        if (i < 0.5 * m)
            LinePoints.push_back(type_util::Point2d(Points[i](0), Points[i](1)));
    }

    Eigen::VectorXd x(3);
    decltype(x) model(1);

    x(0) = 0.8;             // initial value for 'a'
    x(1) = 1.5;             // initial value for 'b'
    x(2) = 0.9;             // initial value for 'c'
    model(0) = 0.5 * M_PI;


    opt_util::SimpleSolver<opt_util::LineFunctor> sm;
    sm.updataModel(model);
    sm.setParams(x);
    sm.feedData(measuredValues);
    timer.start();
    int status = sm.solve();
    x = sm.getParam();


#endif

#endif //LOCATE_REFLECTION_LEVMARQ_H
