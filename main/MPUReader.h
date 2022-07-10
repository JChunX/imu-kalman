#ifndef MPU_READER_H
#define MPU_READER_H

#include <Eigen/Dense>
#include "esp_err.h"
#include "esp_log.h"
#include "MPU.hpp"        
#include "mpu/math.hpp"  
#include "mpu/types.hpp"  
#include "I2Cbus.hpp"


class MPUReader
{
private:
    MPU_t MPU;

    constexpr gpio_num_t SDA = GPIO_NUM_21;
    constexpr gpio_num_t SCL = GPIO_NUM_22;
    constexpr uint32_t CLOCK_SPEED = 400000;
    constexpr uint16_t kSampleRate = 500;

    constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
    constexpr mpud::gyro_fs_t kGyroFS = mpud::GYRO_FS_500DPS;
    constexpr mpud::dlpf_t kDLPF = mpud::DLPF_98HZ;
    constexpr uint16_t kFIFOPacketSize = 12;
    constexpr float kDeltaTime = 1.f / kSampleRate;

    bool new_accel_data = false;
    bool new_gyro_data = false;
    Eigen::Vector3d accel_meas{0, 0, 0};
    Eigen::Vector3d gyro_meas{0, 0, 0};

    bool calib_available = false;
    Eigen::Vector3d accel_calib_weights{1, 1, 1};
    Eigen::Vector3d accel_calib_biases{0, 0, 0};
    Eigen::Vector3d gyro_offsets{0, 0, 0};

    void LinearLeastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::VectorXd& x);
    
public:

    MPUReader();
    void Calibrate();
    void IMUReadTask();
    Eigen::Vector3d GetAccel()
    Eigen::Vector3d GetGyro()
    float GetDeltaTime() { return kDeltaTime; }
}

#endif // MPU_READER_H