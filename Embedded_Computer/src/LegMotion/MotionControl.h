#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include "../../ThirdParty/Eigen/Dense"
#include "DenavitHartenberg.h"

class MotionControl
{
public:

	MotionControl();
	~MotionControl();

	void Walk(Eigen::MatrixXf trajectoryMatrix);
	void Kick(Eigen::MatrixXf kickMatrix);

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
		pelvisPosY
	};

	void InitRotationMatrices();
	void CalculateError(Eigen::Vector3f& ePosToPelvis, Eigen::Vector3f& eThetaToPelvis, Eigen::Vector3f& ePosToFoot, Eigen::Vector3f& eThetaToFoot,
							Eigen::MatrixXf& trajectoryMatrix, DenavitHartenberg* DH, int i);

	Eigen::VectorXf m_q;
	Eigen::Vector3f m_TdToPelvis;
	Eigen::Vector3f m_TdToFoot;

	float m_distanceThreshold;
	int m_angleThreshold;
	int m_nbIterationMax;

};

#endif  //MOTIONCONTROL_H
