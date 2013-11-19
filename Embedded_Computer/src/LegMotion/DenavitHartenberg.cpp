#include "DenavitHartenberg.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace
{
	const float L4 = 0.093f;
	const float L5 = 0.093f;
	const float LTX = 0.037f;	//Repere de robotFoot, correspond a LTZ sur darwin
	const float LTZ = 0.122f;	// " " LTY
	const float LTY = 0.005f;	// " " LTX
	const float LF = 0.037f;
}

DenavitHartenberg::DenavitHartenberg(Eigen::VectorXf q, Leg grounedLeg)
{
	m_groundedFoot = grounedLeg;
	Init(q);

	positionMatrix.resize(3,7);
}

DenavitHartenberg::~DenavitHartenberg()
{}

void DenavitHartenberg::Init(Eigen::VectorXf q)
{
	//Matrice de rotation du repère de robot vers le repère global
	m_RP_1 << 0.0f, 1.0f, 0.0f, LTX,
			0.0f, 0.0f, 1.0f, 0.0f,
			1.0f, 0.0f, 0.0f, LTZ,
			0.0f, 0.0f, 0.0f, 1.0f;

	m_PR_1 << 0.0f, 0.0f, 1.0f, -LTZ,
			1.0f, 0.0f, 0.0f, -LTX,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f;

	m_PR_2_fin << 1.0f, 0.0f, 0.0f, LTX,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, LTZ,
			0.0f, 0.0f, 0.0f, 1.0f;


	m_DHtoPelvis = Eigen::MatrixXf::Zero(6,4);
	m_DHtoFoot = Eigen::MatrixXf::Zero(6,4);
	if(m_groundedFoot == GroundLeft)
	{
		//Left foot to pelvis
		m_DHtoPelvis(1,0) = L5;
		m_DHtoPelvis(2,0) = L4;
		m_DHtoPelvis(0,1) = -M_PI_2;
		m_DHtoPelvis(3,1) = M_PI_2;
		m_DHtoPelvis(4,1) = M_PI_2;
		m_DHtoPelvis(0,3) = q(11);
		m_DHtoPelvis(1,3) = q(10);
		m_DHtoPelvis(2,3) = q(9);
		m_DHtoPelvis(3,3) = q(8);
		m_DHtoPelvis(4,3) = q(7)+M_PI_2;
		m_DHtoPelvis(5,3) = q(6);

		//Pelvis to right foot
		m_DHtoFoot(2,0) = L4;
		m_DHtoFoot(3,0) = L5;
		m_DHtoFoot(0,1) = -M_PI_2;
		m_DHtoFoot(1,1) = M_PI_2;
		m_DHtoFoot(4,1) = -M_PI_2;
		m_DHtoFoot(0,3) = q(5);
		m_DHtoFoot(1,3) = q(4)+M_PI_2;
		m_DHtoFoot(2,3) = q(3);
		m_DHtoFoot(3,3) = q(2);
		m_DHtoFoot(4,3) = q(1);
		m_DHtoFoot(5,3) = q(0);

		//*******Left to right
		m_PR_2_fin(0, 3) = -LTX;
	}
	else
	{
		//Right foot to pelvis
		m_DHtoPelvis(1,0) = L5;
		m_DHtoPelvis(2,0) = L4;
		m_DHtoPelvis(0,1) = -M_PI_2;
		m_DHtoPelvis(3,1) = M_PI_2;
		m_DHtoPelvis(4,1) = -M_PI_2;
		m_DHtoPelvis(0,3) = q(0);
		m_DHtoPelvis(1,3) = q(1);
		m_DHtoPelvis(2,3) = q(2);
		m_DHtoPelvis(3,3) = q(3);
		m_DHtoPelvis(4,3) = q(4)-M_PI_2;
		m_DHtoPelvis(5,3) = q(5);

		//Pelvis to left foot
		m_DHtoFoot(2,0) = -L4;
		m_DHtoFoot(3,0) = -L5;
		m_DHtoFoot(0,1) = M_PI_2;
		m_DHtoFoot(1,1) = -M_PI_2;
		m_DHtoFoot(4,1) = M_PI_2;
		m_DHtoFoot(0,3) = q(6);
		m_DHtoFoot(1,3) = q(7)+M_PI_2;
		m_DHtoFoot(2,3) = q(8);
		m_DHtoFoot(3,3) = q(9);
		m_DHtoFoot(4,3) = q(10);
		m_DHtoFoot(5,3) = q(11);

		//*******Right to left
		m_RP_1(0, 1) = -1.0f;
		m_RP_1(0, 3) = -LTX;
		m_RP_1(1, 2) = -1.0f;

		m_PR_1(1, 0) = -1.0f;
		m_PR_1(2, 1) = -1.0f;
	}
}

void DenavitHartenberg::Update(Eigen::VectorXf q)
{
	if(m_groundedFoot == GroundLeft)
	{
		//Left foot to pelvis
		m_DHtoPelvis(0,3) = q(11);
		m_DHtoPelvis(1,3) = q(10);
		m_DHtoPelvis(2,3) = q(9);
		m_DHtoPelvis(3,3) = q(8);
		m_DHtoPelvis(4,3) = q(7)+M_PI_2;
		m_DHtoPelvis(5,3) = q(6);

		//Pelvis to right foot
		m_DHtoFoot(0,3) = q(5);
		m_DHtoFoot(1,3) = q(4)+M_PI_2;
		m_DHtoFoot(2,3) = q(3);
		m_DHtoFoot(3,3) = q(2);
		m_DHtoFoot(4,3) = q(1);
		m_DHtoFoot(5,3) = q(0);
	}
	else
	{
		//Right foot to pelvis
		m_DHtoPelvis(0,3) = q(0);
		m_DHtoPelvis(1,3) = q(1);
		m_DHtoPelvis(2,3) = q(2);
		m_DHtoPelvis(3,3) = q(3);
		m_DHtoPelvis(4,3) = q(4)-M_PI_2;
		m_DHtoPelvis(5,3) = q(5);

		//Pelvis to left foot
		m_DHtoFoot(0,3) = q(6);
		m_DHtoFoot(1,3) = q(7)+M_PI_2;
		m_DHtoFoot(2,3) = q(8);
		m_DHtoFoot(3,3) = q(9);
		m_DHtoFoot(4,3) = q(10);
		m_DHtoFoot(5,3) = q(11);
	}
}

void DenavitHartenberg::UpdateTe(Eigen::VectorXf q)
{
	if(m_groundedFoot == GroundLeft)
	{
		m_TeToPelvis = Eigen::Vector3f(q(6), q(10)+q(9)+q(8), q(11)+q(7));
		m_TeToFoot = Eigen::Vector3f(q(3)+q(2)+q(1), q(4)+q(0), q(5));
	}
	else
	{
		m_TeToPelvis = Eigen::Vector3f(q(5), q(1)+q(2)+q(3), q(0)+q(4));
		m_TeToFoot = Eigen::Vector3f(-q(8)-q(9)-q(10), -q(7)-q(11), q(6));
	}
}

Eigen::Matrix4f DenavitHartenberg::MatrixHomogene(DHSection section)
{
	if(section == ToPelvis)
		return MatrixHomogene(m_DHtoPelvis);
	else
		return MatrixHomogene(m_DHtoFoot);
}

Eigen::Matrix4f DenavitHartenberg::MatrixHomogene(Eigen::MatrixXf& DH)
{
	//find better names, MF & A
	Eigen::Matrix4f A = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f MF = Eigen::Matrix4f::Identity();

	for(int i = 0; i < DH.rows(); ++i)
	{
		A(0,0) = cos(DH(i,3)); A(0,1) = -sin(DH(i,3))*cos(DH(i,1)); A(0,2) = sin(DH(i,3))*sin(DH(i,1)); A(0,3) = DH(i,0)*cos(DH(i,3));
		A(1,0) = sin(DH(i,3)); A(1,1) = cos(DH(i,3))*cos(DH(i,1)); A(1,2) = -cos(DH(i,3))*sin(DH(i,1)); A(1,3) = DH(i,0)*sin(DH(i,3));
		A(2,0) = 0; 		   A(2,1) = sin(DH(i,1)); 				A(2,2) = cos(DH(i,1)); 				A(2,3) = DH(i,2);

		MF *= A;
	}

	return MF;
}

Eigen::Matrix4f DenavitHartenberg::MatrixHomogene_Vector(Eigen::VectorXf vector)
{
	//find better names for A
	Eigen::Matrix4f A = Eigen::Matrix4f::Identity();

	A(0,0) = cos(vector(3)); A(0,1) = -sin(vector(3))*cos(vector(1)); A(0,2) = sin(vector(3))*sin(vector(1)); A(0,3) = vector(0)*cos(vector(3));
	A(1,0) = sin(vector(3)); A(1,1) = cos(vector(3))*cos(vector(1)); A(1,2) = -cos(vector(3))*sin(vector(1)); A(1,3) = vector(0)*sin(vector(3));
	A(2,0) = 0; 		     A(2,1) = sin(vector(1)); 				 A(2,2) = cos(vector(1)); 				  A(2,3) = vector(2);

	return A;
}

//Return choice:
//1: complete
//2: position
//3: rotation
Eigen::MatrixXf DenavitHartenberg::Jacobian(DHSection section, int returnChoice)
{
	Eigen::MatrixXf* currentDH;
	if(section == ToPelvis)
		currentDH = &m_DHtoPelvis;
	else
		currentDH = &m_DHtoFoot;

	Eigen::Matrix4f A01 = MatrixHomogene_Vector(currentDH->row(0));
	Eigen::Matrix4f A12 = MatrixHomogene_Vector(currentDH->row(1));
	Eigen::Matrix4f A23 = MatrixHomogene_Vector(currentDH->row(2));
	Eigen::Matrix4f A34 = MatrixHomogene_Vector(currentDH->row(3));
	Eigen::Matrix4f A45 = MatrixHomogene_Vector(currentDH->row(4));
	Eigen::Matrix4f A56 = MatrixHomogene_Vector(currentDH->row(5));

	Eigen::Matrix4f A02 = A01*A12;
	Eigen::Matrix4f A03 = A02*A23;
	Eigen::Matrix4f A04 = A03*A34;
	Eigen::Matrix4f A05 = A04*A45;
	Eigen::Matrix4f A06 = A05*A56;

	//*****************write to a file for tests************************************//

	Eigen::Vector3f first = Eigen::Vector3f::Constant(0.0f);

	Eigen::Vector3f posA01;
	posA01 << A01(0,2), A01(1,2), A01(2,2);


	Eigen::Vector3f posA02;
	posA02 << A02(0,2), A02(1,2), A02(2,2);

	Eigen::Vector3f posA03;
	posA03 << A03(0,2), A03(1,2), A03(2,2);

	Eigen::Vector3f posA04;
	posA04 << A04(0,2), A04(1,2), A04(2,2);

	Eigen::Vector3f posA05;
	posA05 << A05(0,2), A05(1,2), A05(2,2);

	Eigen::Vector3f posA06;
	posA06 << A06(0,2), A06(1,2), A06(2,2);

	positionMatrix << first, posA01, posA02, posA03, posA04, posA05, posA06;

	//*****************************************************************************//

	Eigen::Vector4f Z1(0,0,1,0);
	Eigen::MatrixXf Z(4,6);
	Z << Z1, A01.col(2), A02.col(2), A03.col(2), A04.col(2), A05.col(2);
	Z.conservativeResize(3, Z.cols());

	Eigen::MatrixXf P(4,7);
	P << Eigen::Vector4f::Zero(), A01.col(3), A02.col(3), A03.col(3), A04.col(3), A05.col(3), A06.col(3);
	P.conservativeResize(3, P.cols());

	Eigen::MatrixXf Jposition(3,6);
	Eigen::MatrixXf Jrotation = Z;//Clearer but Useless...
	Eigen::Vector3f zCol;
	Eigen::Vector3f deltaP;
	for(int i = 0; i < 6;++i)
	{
		zCol = Z.col(i);
		deltaP = P.col(6)-P.col(i);
		Jposition.col(i) = zCol.cross(deltaP);
	}

	if(returnChoice > 2)
		return Jrotation;
	else if(returnChoice < 2)
	{
		Eigen::MatrixXf J(6,6);
		J << Jposition, Jrotation;
		return J;
	}
	else
		return Jposition;
}

