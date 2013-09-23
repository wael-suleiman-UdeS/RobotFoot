//APP GEI 744, Copyright 2013, Wael Suleiman

#include <stdio.h>
#include <stdlib.h>

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/SVD"
using namespace Eigen;


MatrixXd DLS_LE(MatrixXd& A, VectorXd& b) {


	return MatrixXd(A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b));
}

MatrixXd DLS_inverse(MatrixXd& A, double epsilon= 1E-2) {

  MatrixXd AT= A.transpose();
  int rows=A.rows();
  MatrixXd I= MatrixXd::Identity(rows,rows);
  MatrixXd DLS_A=A*AT + epsilon*epsilon*I;
  
  return MatrixXd(AT*DLS_A.inverse());
}

