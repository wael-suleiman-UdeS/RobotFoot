#ifndef ZMP_H
#define ZMP_H

#include "../../ThirdParty/Eigen/Dense"

class ZMP
{
public:

ZMP();
~ZMP();

Eigen::MatrixXf SpatialZMP(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory, float increment);
Eigen::MatrixXf TemporalZMP(Eigen::MatrixXf zmpSteps, int tp, float tEch);
//Eigen::MatrixXf TemporalZMP(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory, int Tp, float TEch);

private:

};

#endif  //ZMP_H
