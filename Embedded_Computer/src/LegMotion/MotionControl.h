/**
******************************************************************************^M
* @file    MotionControl.h
* @authors  Camille Hébert & Antoine Rioux
* @date    2013-11-19
* @brief   Class to control leg movements
******************************************************************************^M
*/


#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include "../../ThirdParty/Eigen/Dense"
#include "DenavitHartenberg.h"

class MotionControl
{
public:

	MotionControl();
	~MotionControl();

	void Move(Eigen::MatrixXf trajectoryMatrix);

	std::vector<double> UpdateQ(Eigen::VectorXf currentTrajectoryMatrixLine, std::vector<double> currentMotorsPosition);

	std::vector<double> GetInitialQPosition();

private:

	enum trajectoryMatrixIndex {
		time,
		rightFootPosX,
		rightFootPosY,
		rightFootPosZ,
		rightFootAngle,
		leftFootPosX,
		leftFootPosY,
		leftFootPosZ,
		leftFootAngle,
		groundedFoot,
		pelvisPosX,
		pelvisPosY,
		pelvisPosZ
	};

	void InitRotationMatrices();
	void CalculateError(Eigen::Vector3f& ePosToPelvis, Eigen::Vector3f& eThetaToPelvis, Eigen::Vector3f& ePosToFoot, Eigen::Vector3f& eThetaToFoot,
							Eigen::VectorXf& currentTrajectoryMatrixLine, Eigen::VectorXf qMotors);

	//Eigen::VectorXf m_q;
	Eigen::VectorXf m_vInitialQPosition;
	Eigen::Vector3f m_TdToPelvis;
	Eigen::Vector3f m_TdToFoot;

	float m_distanceThreshold;
	float m_damping;
	int m_angleThreshold;
	int m_nbIterationMax;

	DenavitHartenberg m_DH_RightToLeft;
	DenavitHartenberg m_DH_LeftToRight;

	DenavitHartenberg* m_DH;

};

#endif  //MOTIONCONTROL_H
