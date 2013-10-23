#include "Kinematics.h"

#define _USE_MATH_DEFINES
#include <math.h>

Kinematics::Kinematics()
{}

Kinematics::~Kinematics()
{}


//NOT TESTED
Eigen::MatrixXf Kinematics::UpdateDH(float L4, float L5, Eigen::VectorXf q)
{
	Eigen::MatrixXf DH = Eigen::MatrixXf::Zero(6,4);
	DH(0,1) = M_PI_2;
	DH(0,3) = q(1);
	DH(1,0) = L4;
	DH(1,3) = q(2);
	DH(2,0) = L5;
	DH(2,3) = q(3);
	DH(0,1) = -M_PI_2;
	DH(0,1) = q(4);
	DH(0,1) = M_PI_2;
	DH(0,1) = q(5)+M_PI_2;
	DH(0,1) = q(6);

	return DH;
}

//NOT TESTED
Eigen::Matrix4f Kinematics::MatrixHomogene(Eigen::MatrixXf DH)
{
	//find better names, MF & A
	Eigen::Matrix4f A = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f MF = Eigen::Matrix4f::Identity();

	for(int i = 0; i < DH.innerSize(); ++i)
	{
		A(0,0) = cos(DH(i,3)); A(0,1) = -sin(DH(i,3))*cos(DH(i,1)); A(0,2) = sin(DH(i,3))*sin(DH(i,1)); A(0,3) = DH(i,0)*cos(DH(i,3));
		A(0,0) = sin(DH(i,3)); A(0,1) = cos(DH(i,3))*cos(DH(i,1)); A(0,2) = -cos(DH(i,3))*sin(DH(i,1)); A(0,3) = DH(i,0)*sin(DH(i,3));
		A(0,0) = 0; 		   A(0,1) = sin(DH(i,1)); 				A(0,2) = cos(DH(i,1)); 				A(0,3) = DH(i,2);

		MF *= A;
	}

	return MF;
}

//NOT TESTED
Eigen::MatrixXf Kinematics::Jacobian(Eigen::MatrixXf DH, int returnChoice)
{
	Eigen::Matrix4f A01 = MatrixHomogene(DH.row(0));
	Eigen::Matrix4f A12 = MatrixHomogene(DH.row(1));
	Eigen::Matrix4f A23 = MatrixHomogene(DH.row(2));
	Eigen::Matrix4f A34 = MatrixHomogene(DH.row(3));
	Eigen::Matrix4f A45 = MatrixHomogene(DH.row(4));
	//Eigen::Matrix4f A56 = MatrixHomogene(DH.row(5)); //not used

	Eigen::Matrix4f A02 = A01*A12;
	Eigen::Matrix4f A03 = A02*A23;
	Eigen::Matrix4f A04 = A03*A34;
	Eigen::Matrix4f A05 = A04*A45;
	//Eigen::Matrix4f A06 = A05*A56; //not used

	Eigen::Vector4f Z1(0,0,1,0);
	Eigen::MatrixXf Z(4,6);
	Z << Z1, A01.col(2), A02.col(2), A03.col(2), A04.col(2), A05.col(2);
	Z.resize(3, Z.cols());

	Eigen::MatrixXf P(4,7);
	P << Eigen::Vector4f::Zero(), A01.col(3), A02.col(3), A03.col(3), A04.col(3), A05.col(3);
	P.resize(3, P.cols());

	Eigen::MatrixXf Jposition(3,6);
	Eigen::MatrixXf Jrotation = Z;//Clearer but Useless...

	//CROSS error: static assertion failed: "THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE"
	//Eigen::VectorXf test = Z.zCol(0).cross(P.col(6)-P.col(0));

	//Correct, but lame
	//Eigen::Vector3f zCol = Z.col(0);
	//Eigen::Vector3f deltaP = P.col(6)-P.col(0);
	//Eigen::VectorXf test = zCol.cross(deltaP);

	//Make this work!
	//Jposition << Z.col(0).cross(P.col(6)-P.col(0)), Z.col(1).cross(P.col(6)-P.col(1)), Z.col(2).cross(P.col(6)-P.col(2)), Z.col(3).cross(P.col(6)-P.col(3), Z.col(4).cross(P.col(6)-P.col(4)), Z.col(5).cross(P.col(6)-P.col(5)));

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

