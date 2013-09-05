/**
 * \file WalkStatus.cpp
 * \brief Implementation d'un pas sur le robot DarwinOP
 * \author Mathieu Drapeau, Camille Hebert, Antoine Rioux et Maxime Tetrault.
 * \version 0.1
 * \date 27 mars 2013
 *
 * Implementation en c++ d"un pas sur la plateforme DarwinOP a tester
 * durant la validation du cour GEN 744.
 *
 */

//#define DARWINTEST
//#define DEBUGTEST
#define LOGGING

#include "WalkStatus.h"

#include "matrixpinv.h"

#ifdef DARWINTEST
#include "LinuxDARwIn.h"
#include "JointData.h"
#include "MX28.h"
#endif

#include <iostream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <math.h>

#ifdef DARWINTEST
using namespace Robot;
#endif

namespace
{

const int DH_LENGTH = 4;
const int DDL_NUMBER = 6;
const int MH_SIZE = 4;

const double PI = 3.1415926535897932384626433832795;
//const double PI = 2*asin(1);
//const double SQRT2 = sqrt(2);

const double hipOffsetY = .037;    //OP, measured
const double hipOffsetZ = .096;    //OP, Calculated from spec
const double hipOffsetX = .008;    //OP, Calculated from spec
const double thighLength = .0930;  //OP, spec
const double tibiaLength = .0930;  //OP, spec
const double footHeight = .0335;   //OP, spec
const double kneeOffsetX = .025;   //OP (Could Change)
//const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
const double aThigh = -atan(kneeOffsetX/thighLength);
//const double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
const double aTibia = -atan(kneeOffsetX/tibiaLength);

const double TF = 3.0;
const double XF = 0.03;
const double ZMAX = 0.02;

#ifdef DARWINTEST
LinuxCM730 linux_cm730("/dev/ttyUSB0");
  CM730 cm730(&linux_cm730);
#endif

}

WalkStatus::WalkStatus() : _tf(TF)
{
  initQ();
  initD();
  initDH();
  
  initRot();
  
  initPePdTeTd();
  initAllTrajParam(XF, ZMAX); 
#ifdef DARWINTEST
  
  if(cm730.Connect() == false)
  {
      printf("Fail to connect CM-730!\n");
  }

  cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);

  cm730.WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_P_GAIN, 8, 0);
  cm730.WriteByte(JointData::ID_L_SHOULDER_ROLL,  MX28::P_P_GAIN, 8, 0);
  cm730.WriteByte(JointData::ID_L_ELBOW,          MX28::P_P_GAIN, 8, 0);



  cm730.WriteWord(JointData::ID_R_HIP_YAW,    MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_R_HIP_ROLL,   MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_R_HIP_PITCH,  MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_R_KNEE,       MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_R_ANKLE_PITCH,MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_TORQUE_ENABLE, 0, 0);

  cm730.WriteWord(JointData::ID_L_HIP_YAW,    MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_L_HIP_ROLL,   MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_L_HIP_PITCH,  MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_L_KNEE,       MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_L_ANKLE_PITCH,MX28::P_TORQUE_ENABLE, 0, 0);
  cm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_TORQUE_ENABLE, 0, 0);



  cm730.WriteByte(JointData::ID_R_HIP_YAW,    MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_HIP_ROLL,   MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_HIP_PITCH,  MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_KNEE,       MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_ANKLE_PITCH,MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_ANKLE_ROLL, MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);

  cm730.WriteByte(JointData::ID_R_HIP_YAW,    MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_HIP_ROLL,   MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_HIP_PITCH,  MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_KNEE,       MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_ANKLE_PITCH,MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_ANKLE_ROLL, MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);

  cm730.WriteByte(JointData::ID_R_HIP_YAW,    MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_HIP_ROLL,   MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_HIP_PITCH,  MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_KNEE,       MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_ANKLE_PITCH,MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_R_ANKLE_ROLL, MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);



  cm730.WriteByte(JointData::ID_L_HIP_YAW,    MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_HIP_ROLL,   MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_HIP_PITCH,  MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_KNEE,       MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_ANKLE_PITCH,MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_ANKLE_ROLL, MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);

  cm730.WriteByte(JointData::ID_L_HIP_YAW,    MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_HIP_ROLL,   MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_HIP_PITCH,  MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_KNEE,       MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_ANKLE_PITCH,MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_ANKLE_ROLL, MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);

  cm730.WriteByte(JointData::ID_L_HIP_YAW,    MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_HIP_ROLL,   MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_HIP_PITCH,  MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_KNEE,       MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_ANKLE_PITCH,MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
  cm730.WriteByte(JointData::ID_L_ANKLE_ROLL, MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);

  
#endif
}

WalkStatus::WalkStatus( const double tf )
{
  _tf = tf;
  initQ();
  initD();
  initDH();
  
  initRot();
  
  initPePdTeTd();
  initAllTrajParam(XF, ZMAX); 
  
}


WalkStatus::~WalkStatus()
{}

void WalkStatus::printDH()
{
  std::cout << "Printing actual DH matrix" << std::endl;
  std::cout << DH << std::endl;
}

void WalkStatus::printQ()
{
  std::cout << "Printing Q array" << std::endl;
  std::cout << q << std::endl;
}

void WalkStatus::printD()
{
  std::cout << "Printing D array" << std::endl;
  std::cout << d << std::endl;
}

void WalkStatus::printPePdTeTd()
{
  std::cout << "Printing Pe" << std::endl;
  std::cout << Pe << std::endl << std::endl;
  
  std::cout << "Printing Pd" << std::endl;
  std::cout << Pd << std::endl << std::endl;
  
  std::cout << "Printing Te" << std::endl;
  std::cout << Te << std::endl << std::endl;
  
  std::cout << "Printing Td" << std::endl;
  std::cout << Td << std::endl << std::endl;
}

void WalkStatus::printTrajParam()
{
  std::cout << "Printing xTrajParam" << std::endl;
  for( int i = 0; i < 4; i++ )
  {
    std::cout << xTrajParam[i] << ' ';
  }
  std::cout << std::endl;
  
  std::cout << "Printing yTrajParam" << std::endl;
  for( int i = 0; i < 4; i++ )
  {
    std::cout << yTrajParam[i] << ' ';
  }
  std::cout << std::endl;
  
  std::cout << "Printing zTrajParam1" << std::endl;
  for( int i = 0; i < 4; i++ )
  {
    std::cout << zTrajParam1[i] << ' ';
  }
  std::cout << std::endl;

  std::cout << "Printing zTrajParam2" << std::endl;
  for( int i = 0; i < 4; i++ )
  {
    std::cout << zTrajParam2[i] << ' ';
  }
  std::cout << std::endl;
}

Eigen::MatrixXd WalkStatus::generateMH( const int num1, const int num2 )
{
  Eigen::MatrixXd mh = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd tmp(4,4);
  
  if( num1 >= num2 || num2 > DDL_NUMBER || num1 < 0 )
  {
    return mh;
  }
  
  for( int i = num1; i < num2; i++ )
  {
    const double cosBeta = cos(DH(i,3));
    const double cosAlpha = cos(DH(i,1));
    const double sinBeta = sin(DH(i,3));
    const double sinAlpha = sin(DH(i,1));
    
    const double cosA = cosBeta * DH(i,0);
    const double sinA = sinBeta * DH(i,0);
    const double dist = DH(i,2);
    
    tmp(0,0) = cosBeta;
    tmp(0,1) = -sinBeta*cosAlpha;
    tmp(0,2) = sinBeta*sinAlpha;
    tmp(0,3) = cosA;
   
    tmp(1,0) = sinBeta;
    tmp(1,1) = cosBeta*cosAlpha;
    tmp(1,2) = -cosBeta*sinAlpha;
    tmp(1,3) = sinA;
    
    tmp(2,0) = 0.0;
    tmp(2,1) = sinAlpha;
    tmp(2,2) = cosAlpha;
    tmp(2,3) = dist;
    
    tmp(3,0) = 0.0;
    tmp(3,1) = 0.0;
    tmp(3,2) = 0.0;
    tmp(3,3) = 1.0;
    
    mh = mh*tmp;    
  }
  
  return mh;
}

void WalkStatus::generateJacobian( Eigen::MatrixXd& J1, Eigen::MatrixXd& J2 )
{
  Eigen::MatrixXd A01(4,4), A12(4,4), A23(4,4), A34(4,4), A45(4,4), A56(4,4),
                  A02(4,4), A03(4,4), A04(4,4), A05(4,4), A06(4,4);		  
  Eigen::Vector3d Z[6], P[7], tmp;		  

  A01 = generateMH(0,1);
  A12 = generateMH(1,2);
  A23 = generateMH(2,3);
  A34 = generateMH(3,4);
  A45 = generateMH(4,5);
  A56 = generateMH(5,6);
  
  A02 = A01*A12;
  A03 = A02*A23;
  A04 = A03*A34;
  A05 = A04*A45;
  A06 = A05*A56;
  
  Z[0](0) = 0.0; Z[0](1) = 0.0; Z[0](2) = 1.0;
  extractColumnFromMatrix( 2, Z[1], A01 );
  extractColumnFromMatrix( 2, Z[2], A02 );
  extractColumnFromMatrix( 2, Z[3], A03 );
  extractColumnFromMatrix( 2, Z[4], A04 );
  extractColumnFromMatrix( 2, Z[5], A05 );
  
  P[0](0) = 0.0; P[0](1) = 0.0; P[0](2) = 0.0;
  extractColumnFromMatrix( 3, P[1], A01 );
  extractColumnFromMatrix( 3, P[2], A02 );
  extractColumnFromMatrix( 3, P[3], A03 );
  extractColumnFromMatrix( 3, P[4], A04 );
  extractColumnFromMatrix( 3, P[5], A05 );
  extractColumnFromMatrix( 3, P[6], A06 );
  
  for( int i = 0; i < 6; i++ )
  {
      tmp = Z[i].cross(P[6]-P[i]);
      for( int j = 0; j < 3; j ++ )
      {
	J1(j,i) = tmp(j);
	J2(j,i) = Z[i](j);
      }
  }
  
  /*
  std::cout << A01 << std::endl << std::endl
            << A12 << std::endl << std::endl
            << A23 << std::endl << std::endl
            << A34 << std::endl << std::endl
            << A45 << std::endl << std::endl
            << A56 << std::endl << std::endl
           
            << A01 << std::endl << std::endl
            << A02 << std::endl << std::endl
            << A03 << std::endl << std::endl
            << A04 << std::endl << std::endl
            << A05 << std::endl << std::endl
            << A06 << std::endl << std::endl
            
            << Z[0] << std::endl << std::endl
            << Z[1] << std::endl << std::endl
            << Z[2] << std::endl << std::endl
            << Z[3] << std::endl << std::endl
            << Z[4] << std::endl << std::endl
            << Z[5] << std::endl << std::endl
	    
	    << P[0] << std::endl << std::endl
            << P[1] << std::endl << std::endl
            << P[2] << std::endl << std::endl
            << P[3] << std::endl << std::endl
            << P[4] << std::endl << std::endl
            << P[5] << std::endl << std::endl
            << P[6] << std::endl << std::endl
	    
	    << J1 << std::endl << std:: endl
	    << J2 << std::endl << std:: endl;
  */ 
}

void WalkStatus::Process( const double time )
{
  std::vector<double> vec;
  Process( time, vec, false );
}

void WalkStatus::Process( const double time,
		          const std::vector<double>& pos,
		          bool isPosValid/* = true*/ )
{
  const double k1(0.9), k2(0.001);
  Eigen::Vector3d ePos, eTheta;
  Eigen::MatrixXd J1(3,6), J2(3,6), J1inv(3,6), J2inv(3,6);
  
  // Read actual Servormotor position (update q())
  UpdateServoPosition();
  if( isPosValid )
  {
    UpdateQWithVector( pos );
  } 

  // Update DH
  updateDH();

  // Update actual Position and Angle (with DH and q())
  updatePeTe();  

  // Update Trajectory
  UpdatePd( time );
  
  // Calculate Position error
  ePos = Pd - Pe;

  // Calculate Angle Error
  eTheta = Td-Te;  
  
  // Calculate Jacobian
  generateJacobian( J1, J2 );
  J1inv = DLS_inverse(J1,k2);
  J2inv = DLS_inverse(J2,k2);
  
  // Calculate wanted motor position 
  Eigen::MatrixXd priorite1 = J1inv * ePos;
  Eigen::MatrixXd priorite2 = J2inv * (eTheta - (J2*priorite1));

  q = q + k1*(priorite1.transpose() + priorite2.transpose());

  // Send Motor desired value (need q())
  // rightOnly = true
  SendMotorValue( true );

  
  
  
  /*
  std::cout << "Time" << std::endl
            << time << std::endl << std::endl
  
            << "Pd" << std::endl
            << Pd << std::endl << std::endl
            
            << "ePos" << std::endl
            << ePos << std::endl << std::endl
            
            << "eTheta" << std::endl
            << eTheta << std::endl << std::endl
            
            << "DH" << std::endl
            << DH << std::endl << std::endl
            
            << "J1" << std::endl
            << J1 << std::endl << std::endl
            
            << "J1inv" << std::endl
            << J1inv << std::endl << std::endl
            
            << "J2" << std::endl
            << J2 << std::endl << std::endl
            
            << "J2inv" << std::endl
            << J2inv << std::endl << std::endl
            
            << "priorite1" << std::endl
            << priorite1 << std::endl << std::endl
            
            << "priorite2" << std::endl
            << priorite2 << std::endl << std::endl
            
            << "q" << std::endl
            << q << std::endl << std::endl
            
            << "Pe" << std::endl
            << Pe << std::endl << std::endl
            
            << "Te" << std::endl
            << Te << std::endl << std::endl;
	    */
	    
#ifdef LOGGING
  LogToFile( "log.txt", q );
#endif
  
}

void WalkStatus::extractColumnFromMatrix( const int col, Eigen::Vector3d& vector, const Eigen::MatrixXd& matrix )
{  
  vector(0) = matrix(0,col);
  vector(1) = matrix(1,col);
  vector(2) = matrix(2,col);
}



void WalkStatus::initDH()
{
  DH = Eigen::MatrixXd(DDL_NUMBER,DH_LENGTH);
  
  //a
  DH(0,0) = 0.0;
  DH(1,0) = d(0,1);
  DH(2,0) = d(0,2);
  DH(3,0) = d(0,3);
  DH(4,0) = d(0,4);
  DH(5,0) = d(0,5);
  
  //alpha
  DH(0,1) = PI/2;
  DH(1,1) = -PI/2;
  DH(2,1) = 0.0;
  DH(3,1) = 0.0;
  DH(4,1) = PI/2;
  DH(5,1) = 0.0;
  
  //d
  DH(0,2) = d(0,0);
  DH(1,2) = 0.0;
  DH(2,2) = 0.0;
  DH(3,2) = 0.0;
  DH(4,2) = 0.0;
  DH(5,2) = 0.0;
 
  //teta
  updateDH();
}

void WalkStatus::updateDH()
{
  DH(0,3) = q(0,0) + PI/2;
  DH(1,3) = q(0,1) - PI/2;
  DH(2,3) = q(0,2);
  DH(3,3) = q(0,3);
  DH(4,3) = q(0,4);
  DH(5,3) = q(0,5);
}

void WalkStatus::initQ()
{
  q = Eigen::MatrixXd(1,DDL_NUMBER);
  // JointData::ID_R_HIP_YAW
  q(0,0) = 0.0;
  // JointData::ID_R_HIP_ROLL
  q(0,1) = 0.0;
  // JointData::ID_R_HIP_PITCH
  q(0,2) = aThigh;
  // JointData::ID_R_KNEE
  q(0,3) = -aThigh-aTibia;
  //JointData::ID_R_ANKLE_PITCH
  q(0,4) = aTibia;
  //JoinData::ID_L_ANKLE_ROLL
  q(0,5) = 0.0;
}

void WalkStatus::initD()
{
  d = Eigen::MatrixXd(1,DDL_NUMBER);
  
  d(0,0) = 0.0;
  d(0,1) = 0.0;
  d(0,2) = thighLength;
  d(0,3) = tibiaLength;
  d(0,4) = 0.0;
  d(0,5) = footHeight;
}

void WalkStatus::initRot()
{
  Rot = Eigen::MatrixXd(4,4);
  
  Rot(0,0) = 0.0;
  Rot(0,1) = 0.0;
  Rot(0,2) = -1.0;
  Rot(0,3) = 0.0;
 
  Rot(1,0) = 0.0;
  Rot(1,1) = 1.0;
  Rot(1,2) = 0.0;
  Rot(1,3) = 0.0;
 
  Rot(2,0) = 1.0;
  Rot(2,1) = 0.0;
  Rot(2,2) = 0.0;
  Rot(2,3) = 0.0;
  
  Rot(3,0) = 0.0;
  Rot(3,1) = 0.0;
  Rot(3,2) = 0.0;
  Rot(3,3) = 1.0;
}

void WalkStatus::initPePdTeTd()
{
  updatePeTe();
  Pd = Pe;
  Td(0) = 0.0;
  Td(1) = 0.0;
  Td(2) = 0.0;
}

void WalkStatus::updatePeTe()
{
  Eigen::MatrixXd MH = generateMH(0,6) * Rot;
  Pe(0) = MH(0,3);
  Pe(1) = MH(1,3);
  Pe(2) = MH(2,3);
  
  Te(0) = q(1)+q(5);
  Te(1) = q(2)+q(3)+q(4);
  Te(2) = q(0);
}

void WalkStatus::initAllTrajParam( const double Xf, const double zMax )
{
 // TODO: change with object parameter.
  double xi(Pe(0)),yi(Pe(1)),zi(Pe(2));
  double xf(Xf),yf(0.0),zf(Pe(2));
  
  calculateTrajParam( xTrajParam, xi, xf, _tf );
  calculateTrajParam( yTrajParam, yi, yf, _tf );
  calculateTrajParam( zTrajParam1, zi, zf + zMax, _tf/2 );
  calculateTrajParam( zTrajParam2, zf + zMax, zf, _tf/2 );
}

void WalkStatus::calculateTrajParam( double trajParam[4], double xi, double xf, double tf )
{
  trajParam[0] = (2.0*(xi-xf))/(tf*tf*tf);
  trajParam[1] = -(3.0/2.0)*tf*trajParam[0];
  trajParam[2] = 0.0;
  trajParam[3] = xi;  
}

void WalkStatus::UpdatePd( double time )
{
  if( time > _tf )
  {
     time = _tf;
  }
  double time2 = time*time;
  double time3 = time2*time;

  // X
  Pd(0) = xTrajParam[0]*time3 + xTrajParam[1]*time2 + xTrajParam[2]*time + xTrajParam[3];
  // Y
  Pd(1) = yTrajParam[0]*time3 + yTrajParam[1]*time2 + yTrajParam[2]*time + yTrajParam[3];
  // Z
  if( time <= _tf/2 )
  {
     Pd(2) = zTrajParam1[0]*time3 + zTrajParam1[1]*time2 + zTrajParam1[2]*time + zTrajParam1[3];
  }
  else
  {
     time = time - _tf/2;
     time2 = time*time;
     time3 = time2*time;
     Pd(2) = zTrajParam2[0]*time3 + zTrajParam2[1]*time2 + zTrajParam2[2]*time + zTrajParam2[3];
  } 
}

void WalkStatus::UpdateServoPosition()
{
#ifdef DARWINTEST
  int value;
  if(cm730.ReadWord(JointData::ID_R_HIP_YAW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
     q(0,0) = DegreeToRadian( MX28::Value2Angle(value) );
  }
  if(cm730.ReadWord(JointData::ID_R_HIP_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
     q(0,1) = DegreeToRadian( MX28::Value2Angle(value) );
  }
  if(cm730.ReadWord(JointData::ID_R_HIP_PITCH, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
     q(0,2) = DegreeToRadian( MX28::Value2Angle(value) );
  }
  if(cm730.ReadWord(JointData::ID_R_KNEE, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
     q(0,3) = DegreeToRadian( MX28::Value2Angle(value) );
  }
  if(cm730.ReadWord(JointData::ID_R_ANKLE_PITCH, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
     q(0,4) = DegreeToRadian( MX28::Value2Angle(value) );
  }
  if(cm730.ReadWord(JointData::ID_R_ANKLE_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
  {
     q(0,5) = DegreeToRadian( MX28::Value2Angle(value) );
  }
#endif
}
  

void WalkStatus::SendMotorValue( const bool rightOnly )
{
#ifdef DARWINTEST
  cm730.WriteWord(JointData::ID_R_HIP_YAW, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,0))), 0);
  cm730.WriteWord(JointData::ID_R_HIP_ROLL, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,1))), 0);
  cm730.WriteWord(JointData::ID_R_HIP_PITCH, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,2))), 0);
  cm730.WriteWord(JointData::ID_R_KNEE, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,3))), 0);
  cm730.WriteWord(JointData::ID_R_ANKLE_PITCH, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,4))), 0);
  cm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,5))), 0);

  if(!rightOnly)
  {
     cm730.WriteWord(JointData::ID_L_HIP_YAW, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,0))), 0);
     cm730.WriteWord(JointData::ID_L_HIP_ROLL, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,1))), 0);
     cm730.WriteWord(JointData::ID_L_HIP_PITCH, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,2))), 0);
     cm730.WriteWord(JointData::ID_L_KNEE, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,3))), 0);
     cm730.WriteWord(JointData::ID_L_ANKLE_PITCH, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,4))), 0);
     cm730.WriteWord(JointData::ID_L_ANKLE_ROLL, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(RadianToDegree(q(0,5))), 0);
  }
#endif
}

int WalkStatus::getMotorValue( const int ID )
{
  int i(0);
  #ifdef DARWINTEST
  if( ID == JointData::ID_R_HIP_YAW)
  {
    i = 0;
  }
  else if( ID == JointData::ID_R_HIP_ROLL)
  {
    i = 1;
  }
  else if( ID == JointData::ID_R_HIP_PITCH)
  {
    i = 2;
  }
  else if( ID == JointData::ID_R_KNEE)
  {
    i = 3;
  }
  else if( ID == JointData::ID_R_ANKLE_PITCH)
  {
    i = 4;
  }
  else if( ID == JointData::ID_R_ANKLE_ROLL)
  {
    i = 5;
  }
  
  return MX28::Angle2Value(RadianToDegree(q(0,i)));
  #endif

  return i;
}

void WalkStatus::initMotorValue()
{
  Eigen::MatrixXd desiredQ = q;
  Eigen::MatrixXd dtQ(1,6);
  // Update q with motor value.

#ifdef DEBUGTEST
  std::cout << q << std::endl << std:: endl; 
#endif

#ifndef DARWINTEST
  for( int i = 0; i < DDL_NUMBER; i ++ )
  {
     q(0,i) = 0;
  }
#endif

#ifdef DEBUGTEST
  std::cout << q << std::endl << std:: endl; 
#endif 

  UpdateServoPosition();
  
  double tf = 5.0;
  double dt = 0.016;

  for( int i = 0; i < DDL_NUMBER; i ++ )
  {
     dtQ(0,i) = (desiredQ(0,i) - q(0,i))/(tf/dt);
  }

  for(double t = 0.0; t < tf; t+=dt)
  {
     for( int i = 0; i < DDL_NUMBER; i ++ )
     {
        q(0,i) = q(0,i) + dtQ(0,i);
     }
    // rightOnly = false
    SendMotorValue( false );
    usleep(dt*1000*1000);

#ifdef DEBUGTEST
    std::cout << q << std::endl << std:: endl; 
#endif

  }
}

void WalkStatus::getMotorPosition( std::vector<double>& pos )
{
  pos.clear();
  
  for( int i = 0; i < DDL_NUMBER; i ++ )
  {
    if( i == 4 )
      pos.push_back( RadianToDegree( -q(0,i) ) );
    else
      pos.push_back( RadianToDegree( q(0,i) ) );
  }
}

double WalkStatus::RadianToDegree( const double radian )
{
  return radian*180.0/PI;
}

double WalkStatus::DegreeToRadian( const double degree )
{
  return degree*PI/180.0;
}

void WalkStatus::LogToFile( const std::string fileName,
			     const Eigen::MatrixXd& q )
  {
  const static Eigen::MatrixXd leftMotor = -q;
  static bool isFirstRun = true;
  std::ofstream outfile;
  if(isFirstRun) 
  { 
     isFirstRun = false;
     outfile.open(fileName.c_str());     
  }
  else
  {
     outfile.open(fileName.c_str(), std::fstream::app);
  }/*
  std::cout << std::endl << "Time : " << time << std::endl << "Pd : " << Pd 
            << std::endl <<"Pe : " << Pe << std::endl <<"ePos :  " << ePos << std::endl;*/
  for( int i = 0; i < DDL_NUMBER; i ++ )
  {
    if( i == 4 )
     outfile << -RadianToDegree(q(0,i)) << ' ';
    else
      outfile << RadianToDegree(q(0,i)) << ' ';
  }
  for( int i = 0; i < DDL_NUMBER; i ++ )
  {
    if( i == 4 )
     outfile << -RadianToDegree(leftMotor(0,i)) << ' ';
    else
      outfile << RadianToDegree(leftMotor(0,i)) << ' ';
  }
  outfile << std::endl;
  
  outfile.close();
}

void WalkStatus::UpdateQWithVector( const std::vector<double>& pos )
{
  const int vectorSize = pos.size();
  for( int i = 0; i < DDL_NUMBER; i ++ )
  {
    if( i < vectorSize )
    {
      if( i == 4 )
        q(0,i) = -DegreeToRadian( pos[i] );
      else
        q(0,i) = DegreeToRadian( pos[i] );
    }     
  }
}