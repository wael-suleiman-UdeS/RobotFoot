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


#define Debug
#ifdef Debug
#include <iostream>
#include <fstream>
#include <cstdlib>
#endif

class MotionControl
{
public:

	MotionControl(float distanceThreshold);
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
		rightFootAnglePitch,
		rightFootAngleRoll,
		rightFootAngleYaw,
		leftFootPosX,
		leftFootPosY,
		leftFootPosZ,
		leftFootAnglePitch,
		leftFootAngleRoll,
		leftFootAngleYaw,
		groundedFoot,
		pelvisPosX,
		pelvisPosY,
		pelvisPosZ,
		pelvisAnglePitch,
		pelvisAngleRoll,
		pelvisAngleYaw
	};

	void InitRotationMatrices();
	void CalculateError(Eigen::Vector3f& ePosToPelvis, Eigen::Vector3f& eThetaToPelvis, Eigen::Vector3f& ePosToFoot, Eigen::Vector3f& eThetaToFoot,
							Eigen::VectorXf& currentTrajectoryMatrixLine, Eigen::VectorXf qMotors);

	Eigen::VectorXf m_q;
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


#ifdef Debug
	std::ofstream myfile;
	std::ofstream myfileQ;
	std::ofstream myfileEPOS1;
	std::ofstream myfileEPOS2;
	std::ofstream myfileETHETA1;
	std::ofstream myfileETHETA2;
#endif
};

#endif  //MOTIONCONTROL_H
