#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "../../ThirdParty/Eigen/Dense"

class Kinematics
{
public:

Kinematics();
~Kinematics();

Eigen::MatrixXf UpdateDH(float L4, float L5, Eigen::VectorXf q);
Eigen::Matrix4f MatrixHomogene(Eigen::MatrixXf DH);
Eigen::MatrixXf Jacobian(Eigen::MatrixXf DH, int returnChoice);

private:

};

#endif  //KINEMATICS_H
