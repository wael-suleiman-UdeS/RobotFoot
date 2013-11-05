#include "MotionControl.h"

#include "Trajectory.h"
#include "DenavitHartenberg.h"
#include "EigenUtils.h"

#include "../../ThirdParty/Eigen/Dense"


//**********************for tests*****************//
#include <iostream>
#include <fstream>
#include <cstdlib>
using namespace std;
//***************************************************//

namespace
{
	const float ANGLE1 = -0.35;
	const float ANGLE2 = 0.7;
}

MotionControl::MotionControl()
: m_distanceThreshold(0.002f)
, m_angleThreshold(1)
, m_nbIterationMax(1)
{
	m_TdToPelvis = Eigen::Vector3f(0,0,0);
	m_TdToFoot = Eigen::Vector3f(0,0,0);
	m_q.resize(12);
	m_q << 0.0f, ANGLE1, ANGLE2, ANGLE1, 0.0f, 0.0f, 0.0f, 0.0f, ANGLE1, ANGLE2, ANGLE1, 0.0f;
}

MotionControl::~MotionControl()
{

}

void MotionControl::Walk(Eigen::MatrixXf trajectoryMatrix)
{
	float damping = 0.9f;

	DenavitHartenberg DH_LeftToRight(m_q, DenavitHartenberg::GroundLeft);
	DenavitHartenberg DH_RightToLeft(m_q, DenavitHartenberg::GroundRight);

	DenavitHartenberg* DH;

	//*****************write to a file for tests************************************//
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
	//*****************************************************************************//

	for(int i = 0; i < trajectoryMatrix.rows(); ++i)
	{
		bool calculationDone = false;
		int NbIterations = 0;

		if(trajectoryMatrix(i, 9) == DenavitHartenberg::Leg::GroundLeft) //1 is left foot fixed
			DH = &DH_LeftToRight;
		else
			DH = &DH_RightToLeft;

		Eigen::Vector3f ePosToPelvis, ePosToFoot, eThetaToPelvis, eThetaToFoot;
		while(!calculationDone)
		{
			NbIterations++;

			Eigen::MatrixXf eclipseWatchSuck = m_q;

			DH->Update(m_q);

			CalculateError(ePosToPelvis, eThetaToPelvis, ePosToFoot, eThetaToFoot, trajectoryMatrix, DH, i);

			Eigen::MatrixXf jacobienne = DH->Jacobian(DenavitHartenberg::DHSection::ToPelvis, 1);
			Eigen::MatrixXf J1inv = EigenUtils::PseudoInverse(jacobienne.topRows(3));
			Eigen::MatrixXf J2inv = EigenUtils::PseudoInverse(jacobienne.bottomRows(3));

			//*****************write to a file for tests************************************//
			myfile << DH->positionMatrix << endl;
			//*****************************************************************************//

			Eigen::MatrixXf jacobienne2 = DH->Jacobian(DenavitHartenberg::DHSection::ToFoot, 1);
			Eigen::MatrixXf J3inv = EigenUtils::PseudoInverse(jacobienne2.topRows(3));
			Eigen::MatrixXf J4inv = EigenUtils::PseudoInverse(jacobienne2.bottomRows(3));

			Eigen::VectorXf tachePriorite1 = J1inv * ePosToPelvis; 														//Position au sol
			Eigen::VectorXf tachePriorite2 = J2inv * (eThetaToPelvis - (jacobienne.bottomRows(3) * tachePriorite1)); 	//Angle au sol
			Eigen::VectorXf tachePriorite3 = J3inv * ePosToFoot; 														//Position en mouvement
			Eigen::VectorXf tachePriorite4 = J4inv * (eThetaToFoot - (jacobienne2.bottomRows(3) * tachePriorite3)); 	//Angle en mouvement

			if(trajectoryMatrix(i, 9) == DenavitHartenberg::Leg::GroundLeft)
			{
				m_q.reverseInPlace();
				m_q.head(6) = m_q.head(6) + damping * (tachePriorite1 + tachePriorite2);
				m_q.tail(6) = m_q.tail(6) + damping * (tachePriorite3 + tachePriorite4);
				m_q.reverseInPlace();
			}
			else
			{
				m_q.head(6) = m_q.head(6) + damping * (tachePriorite1 + tachePriorite2);
				m_q.tail(6) = m_q.tail(6) + damping * (tachePriorite3 + tachePriorite4);
			}

			DH->Update(m_q);

			CalculateError(ePosToPelvis, ePosToFoot, eThetaToPelvis, eThetaToFoot, trajectoryMatrix, DH, i);

			calculationDone = ePosToPelvis.norm() < m_distanceThreshold && ePosToFoot.norm() < m_distanceThreshold &&
					abs(eThetaToPelvis(0)) < m_angleThreshold && abs(eThetaToPelvis(1)) < m_angleThreshold && abs(eThetaToPelvis(2)) < m_angleThreshold &&
					abs(eThetaToFoot(0)) < m_angleThreshold && abs(eThetaToFoot(1)) < m_angleThreshold && abs(eThetaToFoot(2)) < m_angleThreshold;


			//*******************A enlever plus tard*************************//
			calculationDone = NbIterations >= m_nbIterationMax;

			qToDisplay.tail(6) = m_q.tail(6);
			m_q.reverseInPlace();
			qToDisplay.head(6) = m_q.tail(6);
			m_q.reverseInPlace();

			myfileQ << qToDisplay.transpose() << endl;
			myfileEPOS1 << ePosToPelvis << endl;
			myfileEPOS2 << ePosToFoot << endl;
			myfileETHETA1 << eThetaToPelvis << endl;
			myfileETHETA2 << eThetaToFoot << endl;
		}
	}

	//*****************write to a file for tests************************************//
	myfile.close();
	myfileQ.close();
	myfileEPOS1.close();
	myfileEPOS2.close();
	myfileETHETA1.close();
	myfileETHETA2.close();
	//*****************************************************************************//
}

//Check right left*****************************************
void MotionControl::CalculateError(Eigen::Vector3f& ePosToPelvis, Eigen::Vector3f& eThetaToPelvis, Eigen::Vector3f& ePosToFoot,
		Eigen::Vector3f& eThetaToFoot, Eigen::MatrixXf& trajectoryMatrix, DenavitHartenberg* DH, int i)
{
	/////////////////////////////////////////////////////////////////
	//Ground foot to pelvis
	/////////////////////////////////////////////////////////////////
	Eigen::Vector3f PeToPelvis = DH->MatrixHomogene(DenavitHartenberg::DHSection::ToPelvis).topRightCorner(3,1);//Position of End effector (pelvis) from ground foot

	Eigen::Matrix4f tempPdToPelvis = Eigen::Matrix4f::Identity();
	tempPdToPelvis.col(3) = Eigen::Vector4f(trajectoryMatrix(i,10), trajectoryMatrix(i,11), trajectoryMatrix(i,12), 1);
	tempPdToPelvis = DH->GetPR1() * tempPdToPelvis * DH->GetPR1Fin();
	Eigen::Vector3f PdToPelvis =  tempPdToPelvis.topRightCorner(3,1); 								//Position Desired for the end effector (pelvis) from ground foot

	DH->UpdateTe(m_q);

	ePosToPelvis = PdToPelvis - PeToPelvis;															//Error on the Position (pelvis) from ground foot
	eThetaToPelvis = m_TdToPelvis - DH->GetTeToPelvis();											//Error on the Angle (pelvis) from ground foot

	/////////////////////////////////////////////////////////////////
	//Pelvis to moving foot
	/////////////////////////////////////////////////////////////////
	Eigen::Matrix4f tempPeToPelvis = Eigen::Matrix4f::Identity();
	tempPeToPelvis.topRightCorner(3,1) = PeToPelvis;
	tempPeToPelvis = DH->GetRP1() * tempPeToPelvis;
	Eigen::Vector3f PeToPelvisP = tempPeToPelvis.topRightCorner(3,1);

	Eigen::Matrix4f tempPeToFootp = Eigen::Matrix4f::Identity();
	tempPeToFootp.col(3) = DH->MatrixHomogene(DenavitHartenberg::DHSection::ToFoot).col(3);
	tempPeToFootp = DH->GetRP2() * tempPeToFootp;
	Eigen::Vector3f PeToFootp = PeToPelvisP + tempPeToFootp.topRightCorner(3,1);

	Eigen::Matrix4f tempPeToFoot = Eigen::Matrix4f::Identity();
	tempPeToFoot.topRightCorner(3,1) = PeToFootp - PeToPelvisP;
	tempPeToFoot = tempPeToFoot * DH->GetPR2Fin();
	Eigen::Vector3f PeToFoot = tempPeToFoot.topRightCorner(3,1); 									//Position of End effector (foot) for moving foot

	Eigen::Matrix4f tempPdToFoot = Eigen::Matrix4f::Identity();
	if(trajectoryMatrix(i, groundedFoot) == DenavitHartenberg::Leg::GroundLeft)
	{
		tempPdToFoot.col(3) = Eigen::Vector4f(trajectoryMatrix(i,rightFootPosX)-PeToPelvisP(0),
				trajectoryMatrix(i,rightFootPosY)-PeToPelvisP(1), trajectoryMatrix(i,rightFootPosZ)-PeToPelvisP(2), 1);
	}
	else
	{
		float x = trajectoryMatrix(i,leftFootPosX);
		tempPdToFoot.col(3) = Eigen::Vector4f(trajectoryMatrix(i,leftFootPosX)-PeToPelvisP(0),
				trajectoryMatrix(i,leftFootPosY)-PeToPelvisP(1), trajectoryMatrix(i,leftFootPosZ)-PeToPelvisP(2), 1);
	}

	tempPdToFoot = tempPdToFoot * DH->GetPR2Fin();
	Eigen::Vector3f PdToFoot =  tempPdToFoot.topRightCorner(3,1); 									//Position Desired for the end effector (foot) for moving foot

	ePosToFoot = PdToFoot - PeToFoot;																//Error on the Position (foot) for moving foot
	eThetaToFoot = m_TdToFoot - DH->GetTeToFoot();													//Error on the Angle (foot) for moving foot
}

