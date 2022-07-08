//#define EIGEN_NO_MALLOC

#define SDA GPIO_NUM_21
#define SCL GPIO_NUM_22
#define CLOCK 100000

#include <iostream>
#include <functional>
#include <cmath>
#include <Eigen/Dense>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kalman_filter.h"
#include "MPU.hpp"        
#include "mpu/math.hpp"  
#include "mpu/types.hpp"  
#include "I2Cbus.hpp"

Eigen::Matrix<double, 9, 9> getA(double dt);
 
extern "C" void app_main(void)
{
    printf("Hello world!\n");
    i2c0.begin(SDA, SCL, CLOCK);  
    printf("0\n");
    MPU_t MPU;    
    printf("1\n"); 
    MPU.setBus(i2c0); 
    printf("2\n");
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
    printf("3\n");
    MPU.testConnection();
    printf("4\n");
    MPU.initialize(); 
    printf("MPU initialized\n");
    MPU.setSampleRate(250);
    MPU.setAccelFullScale(mpud::ACCEL_FS_4G);
    MPU.setGyroFullScale(mpud::GYRO_FS_500DPS);
    MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ);
    MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);

    mpud::raw_axes_t accelRaw;   
    mpud::raw_axes_t gyroRaw;   
    printf("MPU configured\n");

    Eigen::Vector<double, 9> x_init = Eigen::Vector<double, 9>::Zero();
    Eigen::Matrix<double, 9, 9> P_init = Eigen::Matrix<double, 9, 9>::Identity();Eigen::Matrix<double, 9, 9> Q_init = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 3, 3> R_init = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
    for (int i = 0; i<3; i++)
    {
        H(i,i+6) = 1;
    }
    
    std::function<Eigen::Matrix<double, 9, 9>(double)> A_ptr {&getA};

    KalmanFilter<9,3> kf(x_init, P_init, Q_init, R_init, A_ptr, H);

    printf("Kalman filter initialized\n");

    while (1)
    {
        MPU.acceleration(&accelRaw);  // fetch raw data from the registers
        MPU.rotation(&gyroRaw);       // fetch raw data from the registers
        printf("accel: %+d %+d %+d\n", accelRaw.x, accelRaw.y, accelRaw.z);
        printf("gyro: %+d %+d %+d\n", gyroRaw[0], gyroRaw[1], gyroRaw[2]);
    }
}

Eigen::Matrix<double, 9, 9> getA(double dt)
{
    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();

    for (int i = 0; i < 6; i++)
    {
        A(i,i+3) = dt;
    }

    for (int i = 0; i < 3; i++)
    {
        A(i,i+6) = 0.5 * pow(dt,2);
    }

    return A;
}
