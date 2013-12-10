/**
******************************************************************************^M
* @file    MotionControl.cpp
* @authors  Camille Hébert & Antoine Rioux
* @date    2013-11-19
* @brief   Class to control leg movements
******************************************************************************^M
*/

#include "MotionControl.h"

#include "Trajectory.h"
#include "EigenUtils.h"
#include "Utilities/logger.h"

using namespace std;

namespace
{
	//const float ANGLE1 = 0.35;
	//const float ANGLE2 = -0.7;

const float ANGLE1 = 0.6;
const float ANGLE2 = -1.2;
}

MotionControl::MotionControl(float distanceThreshold, float angleThreshold, float maxPosError, int iterationMax)
: m_distanceThreshold(distanceThreshold)
, m_angleThreshold(angleThreshold)
, m_maxPosError(maxPosError)
, m_nbIterationMax(iterationMax)
{
	m_TdToPelvis = Eigen::Vector3f(0,0,0);
	m_TdToFoot = Eigen::Vector3f(0,0,0);
	m_vInitialQPosition.resize(12);
	m_vInitialQPosition << 0.0f, ANGLE1, ANGLE2, ANGLE1, 0.0f, 0.0f, 0.0f, 0.0f, -ANGLE1, -ANGLE2, -ANGLE1, 0.0f;
	m_q = m_vInitialQPosition;

	m_damping = 0.9f;

	m_DH_RightToLeft = DenavitHartenberg(m_vInitialQPosition, DenavitHartenberg::Leg::GroundRight);
	m_DH_LeftToRight = DenavitHartenberg(m_vInitialQPosition, DenavitHartenberg::Leg::GroundLeft);



#ifdef Debug
	myfile.open ("matrixPositions.txt");
	myfileQ.open ("matrixQ.txt");
	myfileEPOS1.open ("ePos1.txt");
	myfileEPOS2.open ("ePos2.txt");
	myfileETHETA1.open ("eTheta1.txt");
	myfileETHETA2.open ("eTheta2.txt");
#endif
}

MotionControl::~MotionControl()
{
#ifdef Debug
	myfile.close();
	myfileQ.close();
	myfileEPOS1.close();
	myfileEPOS2.close();
	myfileETHETA1.close();
	myfileETHETA2.close();
#endif
}

std::vector<double> MotionControl::GetInitialQPosition()
{
	float dangle1 = ANGLE1*180/M_PI;
	float dangle2 = ANGLE2*180/M_PI;
	std::vector<double> initialQ = {0.0f, 0.0f, dangle1, dangle2, dangle1, 0.0f, 0.0f, 0.0f, -dangle1, -dangle2, -dangle1, 0.0f};

	return initialQ;
}

std::vector<double> MotionControl::UpdateQ(Eigen::VectorXf currentTrajectoryMatrixLine, std::vector<double> currentMotorsPosition)
{
	//pelvis angle offset
	m_TdToPelvis(0) = currentTrajectoryMatrixLine(pelvisAngleYaw);
	m_TdToPelvis(1) = currentTrajectoryMatrixLine(pelvisAnglePitch);
	m_TdToPelvis(2) = currentTrajectoryMatrixLine(pelvisAngleRoll);

	Eigen::VectorXf qMotors(12);
	Eigen::VectorXf qToDisplay(12);

	qMotors = Eigen::Map<Eigen::VectorXd>((double*)&currentMotorsPosition[0], 12).cast<float>();

	qMotors*=M_PI/180;

	bool calculationDone = false;
	int NbIterations = 0;

	if(currentTrajectoryMatrixLine(groundedFoot) == DenavitHartenberg::Leg::GroundLeft) //1 is left foot fixed
		m_DH = &m_DH_LeftToRight;
	else
		m_DH = &m_DH_RightToLeft;

	Eigen::Vector3f ePosToPelvis, ePosToFoot, eThetaToPelvis, eThetaToFoot;
	while(!calculationDone)
	{
		NbIterations++;

		m_DH->Update(qMotors);

		CalculateError(ePosToPelvis, eThetaToPelvis, ePosToFoot, eThetaToFoot, currentTrajectoryMatrixLine, qMotors);

		Eigen::MatrixXf jacobienne = m_DH->Jacobian(DenavitHartenberg::DHSection::ToPelvis, 1);
		Eigen::MatrixXf J1inv = EigenUtils::PseudoInverse(jacobienne.topRows(3));
		Eigen::MatrixXf J2inv = EigenUtils::PseudoInverse(jacobienne.bottomRows(3));

		Eigen::MatrixXf jacobienne2 = m_DH->Jacobian(DenavitHartenberg::DHSection::ToFoot, 1);
		Eigen::MatrixXf J3inv = EigenUtils::PseudoInverse(jacobienne2.topRows(3));
		Eigen::MatrixXf J4inv = EigenUtils::PseudoInverse(jacobienne2.bottomRows(3));

		Eigen::VectorXf tachePriorite1 = J1inv * ePosToPelvis; 														//Position au sol
		Eigen::VectorXf tachePriorite2 = J2inv * (eThetaToPelvis - (jacobienne.bottomRows(3) * tachePriorite1)); 	//Angle au sol
		Eigen::VectorXf tachePriorite3 = J3inv * ePosToFoot; 														//Position en mouvement
		Eigen::VectorXf tachePriorite4 = J4inv * (eThetaToFoot - (jacobienne2.bottomRows(3) * tachePriorite3)); 	//Angle en mouvement

		if(currentTrajectoryMatrixLine(groundedFoot) == DenavitHartenberg::Leg::GroundLeft)
		{
			qMotors.reverseInPlace();
			qMotors.head(6) = qMotors.head(6) + m_damping * (tachePriorite1 + tachePriorite2);
			qMotors.tail(6) = qMotors.tail(6) + m_damping * (tachePriorite3 + tachePriorite4);
			qMotors.reverseInPlace();
		}
		else
		{
			qMotors.head(6) = qMotors.head(6) + m_damping * (tachePriorite1 + tachePriorite2);
			qMotors.tail(6) = qMotors.tail(6) + m_damping * (tachePriorite3 + tachePriorite4);
		}

		m_DH->Update(qMotors);

		CalculateError(ePosToPelvis, ePosToFoot, eThetaToPelvis, eThetaToFoot, currentTrajectoryMatrixLine, qMotors);

		if(abs(ePosToPelvis.norm()) > m_maxPosError)
		{

			Logger::getInstance(Logger::LogLvl::ERROR) << "ePosToPelvis too big, value = " << ePosToPelvis.norm() << std::endl;
			std::terminate();
		}
		if(abs(ePosToFoot.norm()) > m_maxPosError)
		{
			Logger::getInstance(Logger::LogLvl::ERROR) << "ePotToFoot too big, value = " << ePosToFoot.norm() << std::endl;
			std::terminate();
		}

		calculationDone = ePosToPelvis.norm() < m_distanceThreshold && ePosToFoot.norm() < m_distanceThreshold &&
				abs(eThetaToPelvis(0)) < m_angleThreshold && abs(eThetaToPelvis(1)) < m_angleThreshold && abs(eThetaToPelvis(2)) < m_angleThreshold &&
				abs(eThetaToFoot(0)) < m_angleThreshold && abs(eThetaToFoot(1)) < m_angleThreshold && abs(eThetaToFoot(2)) < m_angleThreshold;


		//*******************A enlever plus tard*************************//
		calculationDone = NbIterations >= m_nbIterationMax;

		//Change the order of Q so that it works with the motors
		qToDisplay.tail(6) = qMotors.tail(6);
		qMotors.reverseInPlace();
		qToDisplay.head(6) = qMotors.tail(6);
		qMotors.reverseInPlace();
		qToDisplay*=180/M_PI;
	}

	//std::vector<float> stdQToDisplay(12);
	//Eigen::Map<Eigen::VectorXf>(stdQToDisplay.data(), 12) = qToDisplay;

	std::vector<double> stdQToDisplay(12);
	Eigen::Map<Eigen::VectorXd>(stdQToDisplay.data(), 12) = qToDisplay.cast<double>();


#ifdef Debug
			myfileQ << qToDisplay.transpose() << endl;
			myfileEPOS1 << ePosToPelvis << endl;
			myfileEPOS2 << ePosToFoot << endl;
			myfileETHETA1 << eThetaToPelvis << endl;
			myfileETHETA2 << eThetaToFoot << endl;
#endif

	return stdQToDisplay;
}


void MotionControl::Move(Eigen::MatrixXf trajectoryMatrix)
{
#ifdef Debug
	ofstream myfile;
	myfile.open ("matrixPositions.txt");

	ofstream myfileQ;
	myfileQ.open ("matrixQ.txt");

	ofstream myfileEPOS1;
	myfileEPOS1.open ("ePos1.txt");

	ofstream myfileEPOS2;
	myfileEPOS2.open ("ePos2.txt");

	ofstream myfileETHETA1;
	myfileETHETA1.open ("eTheta1.txt");

	ofstream myfileETHETA2;
	myfileETHETA2.open ("eTheta2.txt");

	Eigen::VectorXf qToDisplay(12);
#endif

	for(int i = 0; i < trajectoryMatrix.rows(); ++i)
	{

		if(i == 1144)
			int x = 0;

		//pelvis angle offset
		m_TdToPelvis(0) = -trajectoryMatrix(i, pelvisAngleRoll);
		m_TdToPelvis(1) = trajectoryMatrix(i, pelvisAngleYaw);
		m_TdToPelvis(2) = trajectoryMatrix(i, pelvisAnglePitch);

		bool calculationDone = false;
		int NbIterations = 0;

		if(trajectoryMatrix(i, groundedFoot) == DenavitHartenberg::Leg::GroundLeft) //1 is left foot fixed
			m_DH = &m_DH_LeftToRight;
		else
			m_DH = &m_DH_RightToLeft;

		Eigen::Vector3f ePosToPelvis, ePosToFoot, eThetaToPelvis, eThetaToFoot;
		while(!calculationDone)
		{
			NbIterations++;

			//Eigen::MatrixXf eclipseWatchSuck = m_q;

			m_DH->Update(m_q);

			Eigen::VectorXf matrixLine = trajectoryMatrix.row(i);
			CalculateError(ePosToPelvis, eThetaToPelvis, ePosToFoot, eThetaToFoot, matrixLine, m_q);

			Eigen::MatrixXf jacobienne = m_DH->Jacobian(DenavitHartenberg::DHSection::ToPelvis, 1);
			Eigen::MatrixXf J1inv = EigenUtils::PseudoInverse(jacobienne.topRows(3));
			Eigen::MatrixXf J2inv = EigenUtils::PseudoInverse(jacobienne.bottomRows(3));

			myfile << m_DH->positionMatrix << endl;

			Eigen::MatrixXf jacobienne2 = m_DH->Jacobian(DenavitHartenberg::DHSection::ToFoot, 1);
			Eigen::MatrixXf J3inv = EigenUtils::PseudoInverse(jacobienne2.topRows(3));
			Eigen::MatrixXf J4inv = EigenUtils::PseudoInverse(jacobienne2.bottomRows(3));

			Eigen::VectorXf tachePriorite1 = J1inv * ePosToPelvis; 														//Position au sol
			Eigen::VectorXf tachePriorite2 = J2inv * (eThetaToPelvis - (jacobienne.bottomRows(3) * tachePriorite1)); 	//Angle au sol
			Eigen::VectorXf tachePriorite3 = J3inv * ePosToFoot; 														//Position en mouvement
			Eigen::VectorXf tachePriorite4 = J4inv * (eThetaToFoot - (jacobienne2.bottomRows(3) * tachePriorite3)); 	//Angle en mouvement

			if(trajectoryMatrix(i, groundedFoot) == DenavitHartenberg::Leg::GroundLeft)
			{
				m_q.reverseInPlace();
				m_q.head(6) = m_q.head(6) + m_damping * (tachePriorite1 + tachePriorite2);
				m_q.tail(6) = m_q.tail(6) + m_damping * (tachePriorite3 + tachePriorite4);
				m_q.reverseInPlace();
			}
			else
			{
				m_q.head(6) = m_q.head(6) + m_damping * (tachePriorite1 + tachePriorite2);
				m_q.tail(6) = m_q.tail(6) + m_damping * (tachePriorite3 + tachePriorite4);
			}

			m_DH->Update(m_q);

			CalculateError(ePosToPelvis, ePosToFoot, eThetaToPelvis, eThetaToFoot, matrixLine, m_q);

			if(ePosToPelvis.norm() > m_maxPosError || ePosToFoot.norm() > m_maxPosError)
			{
				std::terminate();
			}

			calculationDone = ePosToPelvis.norm() < m_distanceThreshold && ePosToFoot.norm() < m_distanceThreshold &&
					abs(eThetaToPelvis(0)) < m_angleThreshold && abs(eThetaToPelvis(1)) < m_angleThreshold && abs(eThetaToPelvis(2)) < m_angleThreshold &&
					abs(eThetaToFoot(0)) < m_angleThreshold && abs(eThetaToFoot(1)) < m_angleThreshold && abs(eThetaToFoot(2)) < m_angleThreshold;


			calculationDone = NbIterations >= m_nbIterationMax;

			qToDisplay.tail(6) = m_q.tail(6);
			m_q.reverseInPlace();
			qToDisplay.head(6) = m_q.tail(6);
			m_q.reverseInPlace();
			qToDisplay*=180/M_PI;

		}
#ifdef Debug
			myfileQ << qToDisplay.transpose() << endl;
			myfileEPOS1 << ePosToPelvis << endl;
			myfileEPOS2 << ePosToFoot << endl;
			myfileETHETA1 << eThetaToPelvis << endl;
			myfileETHETA2 << eThetaToFoot << endl;
#endif
	}

#ifdef Debug
	myfile.close();
	myfileQ.close();
	myfileEPOS1.close();
	myfileEPOS2.close();
	myfileETHETA1.close();
	myfileETHETA2.close();
#endif
}

void MotionControl::CalculateError(Eigen::Vector3f& ePosToPelvis, Eigen::Vector3f& eThetaToPelvis, Eigen::Vector3f& ePosToFoot,
		Eigen::Vector3f& eThetaToFoot, Eigen::VectorXf& currentTrajectoryMatrixLine, Eigen::VectorXf qMotors)
{
	Eigen::Vector3f vRightFootPos;
	vRightFootPos << currentTrajectoryMatrixLine(rightFootPosX), currentTrajectoryMatrixLine(rightFootPosY), currentTrajectoryMatrixLine(rightFootPosZ);
	Eigen::Vector3f vLeftFootPos;
	vLeftFootPos << currentTrajectoryMatrixLine(leftFootPosX), currentTrajectoryMatrixLine(leftFootPosY), currentTrajectoryMatrixLine(leftFootPosZ);
	Eigen::Vector3f vPelvisPos;
	vPelvisPos << currentTrajectoryMatrixLine(pelvisPosX), currentTrajectoryMatrixLine(pelvisPosY), currentTrajectoryMatrixLine(pelvisPosZ);

	Eigen::Vector3f vDesiredThetaToPelvis;
	Eigen::Vector3f vDesiredThetaToFoot;

	//Fixed foot position
	Eigen::Vector3f Pe0p;
	if(currentTrajectoryMatrixLine(groundedFoot) == DenavitHartenberg::Leg::GroundLeft)
	{
		Eigen::Matrix3f baseChangeMatrix = EigenUtils::BaseChange(currentTrajectoryMatrixLine(leftFootAngleYaw));

		vRightFootPos = vRightFootPos.transpose()*baseChangeMatrix;
		vLeftFootPos = vLeftFootPos.transpose()*baseChangeMatrix;
		vPelvisPos = vPelvisPos.transpose()*baseChangeMatrix;

		Pe0p = vLeftFootPos;

		//Add the right foot yaw to the pelvis yaw
		vDesiredThetaToPelvis = m_TdToPelvis;
		vDesiredThetaToPelvis(0) = -m_TdToPelvis(0) + currentTrajectoryMatrixLine(leftFootAngleYaw);

		//feet angle offset
		vDesiredThetaToFoot(0) = currentTrajectoryMatrixLine(rightFootAnglePitch);
		vDesiredThetaToFoot(1) = currentTrajectoryMatrixLine(rightFootAngleRoll);
		vDesiredThetaToFoot(2) = currentTrajectoryMatrixLine(rightFootAngleYaw) - currentTrajectoryMatrixLine(pelvisAngleRoll);
	}
	else
	{
		Eigen::Matrix3f baseChangeMatrix = EigenUtils::BaseChange(currentTrajectoryMatrixLine(rightFootAngleYaw));

		vRightFootPos = vRightFootPos.transpose()*baseChangeMatrix;
		vLeftFootPos = vLeftFootPos.transpose()*baseChangeMatrix;
		vPelvisPos = vPelvisPos.transpose()*baseChangeMatrix;

		Pe0p = vRightFootPos;

		//Add the right foot yaw to the pelvis yaw
		vDesiredThetaToPelvis = m_TdToPelvis;
		vDesiredThetaToPelvis(0) = -m_TdToPelvis(0) + currentTrajectoryMatrixLine(rightFootAngleYaw);

		//feet angle offset
		vDesiredThetaToFoot(0) = currentTrajectoryMatrixLine(leftFootAnglePitch);
		vDesiredThetaToFoot(1) = currentTrajectoryMatrixLine(leftFootAngleRoll);
		vDesiredThetaToFoot(2) = currentTrajectoryMatrixLine(leftFootAngleYaw) - currentTrajectoryMatrixLine(pelvisAngleYaw);
	}

	/////////////////////////////////////////////////////////////////
	//Ground foot to pelvis
	/////////////////////////////////////////////////////////////////
	Eigen::Vector3f PeToPelvis = m_DH->MatrixHomogene(DenavitHartenberg::DHSection::ToPelvis).topRightCorner(3,1);//Position of End effector (pelvis) from ground foot

	Eigen::Matrix4f tempPdToPelvis = Eigen::Matrix4f::Identity();
	tempPdToPelvis.topRightCorner(3,1) = vPelvisPos - Pe0p;
	tempPdToPelvis = m_DH->GetPR1() * tempPdToPelvis;
	Eigen::Vector3f PdToPelvis =  tempPdToPelvis.topRightCorner(3,1);						//Position Desired for the end effector (pelvis) from ground foot

	m_DH->UpdateTe(qMotors);

	ePosToPelvis = PdToPelvis - PeToPelvis;															//Error on the Position (pelvis) from ground foot
	eThetaToPelvis = vDesiredThetaToPelvis - m_DH->GetTeToPelvis();									//Error on the Angle (pelvis) from ground foot

	/////////////////////////////////////////////////////////////////
	//Pelvis to moving foot
	/////////////////////////////////////////////////////////////////
	Eigen::Matrix4f tempPeToPelvis = Eigen::Matrix4f::Identity();
	tempPeToPelvis.topRightCorner(3,1) = PeToPelvis;
	tempPeToPelvis = m_DH->GetRP1() * tempPeToPelvis;
	Eigen::Vector3f PeToPelvisP = tempPeToPelvis.topRightCorner(3,1) + Pe0p;

	Eigen::Vector3f PeToFoot = m_DH->MatrixHomogene(DenavitHartenberg::DHSection::ToFoot).topRightCorner(3,1);

	Eigen::Matrix4f tempPdToFoot = Eigen::Matrix4f::Identity();
	if(currentTrajectoryMatrixLine(groundedFoot) == DenavitHartenberg::Leg::GroundLeft)
	{
		tempPdToFoot.col(3) = Eigen::Vector4f(vRightFootPos(0)-PeToPelvisP(0),
				vRightFootPos(1)-PeToPelvisP(1), vRightFootPos(2)-PeToPelvisP(2), 1);
	}
	else
	{
		tempPdToFoot.col(3) = Eigen::Vector4f(vLeftFootPos(0)-PeToPelvisP(0),
				vLeftFootPos(1)-PeToPelvisP(1), vLeftFootPos(2)-PeToPelvisP(2), 1);
	}

	tempPdToFoot = tempPdToFoot * m_DH->GetPR2Fin();
	Eigen::Vector3f PdToFoot =  tempPdToFoot.topRightCorner(3,1); 									//Position Desired for the end effector (foot) for moving foot

	ePosToFoot = PdToFoot - PeToFoot;																//Error on the Position (foot) for moving foot
	eThetaToFoot = vDesiredThetaToFoot - m_DH->GetTeToFoot();										//Error on the Angle (foot) for moving foot
}

