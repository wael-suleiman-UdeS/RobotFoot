#include "WalkStatus.h"

#include "Eigen/Dense"

#include <iostream>


int main()
{
  WalkStatus test;

  test.initMotorValue();
  usleep(3*1000*1000);
  double dt = 0.016;
  double tf = 3.0;
  for( double time = 0.0; time < tf+dt; time += dt )
  {
    test.Process(time);
    usleep(dt*1000*1000);
    std::cout << time;      
  }
}
 
