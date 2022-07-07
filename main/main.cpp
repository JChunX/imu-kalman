//#define EIGEN_NO_MALLOC
#include <iostream>
#include <Eigen/Dense>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
using Eigen::MatrixXd;
 
extern "C" void app_main(void)
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
}