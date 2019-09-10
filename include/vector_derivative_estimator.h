#ifndef VECTORDERIVATIVEESTIMATOR_H
#define VECTORDERIVATIVEESTIMATOR_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>

// LS estimator for a first order approximatoion of the derivative of a state vector wrt time, thanks Anup
struct VectorDerivativeEstimator
{
    int stateSize;
    bool firstUpdate;
    ros::Time tLast;
    Eigen::VectorXf  xHat;
    Eigen::MatrixXf P;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;
    Eigen::MatrixXf F;
    Eigen::MatrixXf H;
    Eigen::MatrixXf HT;
    Eigen::MatrixXf II;
    Eigen::MatrixXf S;
    Eigen::MatrixXf SI;
    Eigen::MatrixXf K;

    VectorDerivativeEstimator();

    void initialize(int stateSizeInit);

    Eigen::VectorXf update(Eigen::VectorXf newMeasure, ros::Time newTime);

    Eigen::VectorXf update(Eigen::VectorXf newMeasure, ros::Time newTime, Eigen::VectorXf newMeasureExpected);

    Eigen::VectorXf update(std::vector<Eigen::VectorXf> newMeasures, ros::Time newTime, Eigen::VectorXf newMeasureExpected);

    Eigen::VectorXf xDot(Eigen::VectorXf x);
};

#endif
