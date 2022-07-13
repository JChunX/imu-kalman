#ifndef MPU_READER_H
#define MPU_READER_H

#include <Eigen/Dense>
#include "esp_err.h"
#include "esp_log.h"
#include "MPU.hpp"        
#include "mpu/math.hpp"  
#include "mpu/types.hpp"  
#include "I2Cbus.hpp"

struct IMUMeasurement {
    Eigen::Vector3d accel_meas;
    Eigen::Vector3d gyro_meas;
};

class MPUReader
{
private:
    MPU_t MPU;

    static constexpr gpio_num_t SDA = GPIO_NUM_21;
    static constexpr gpio_num_t SCL = GPIO_NUM_22;

    static constexpr uint32_t CLOCK_SPEED = 400000;
    static constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
    static constexpr mpud::gyro_fs_t kGyroFS = mpud::GYRO_FS_500DPS;
    static constexpr mpud::dlpf_t kDLPF = mpud::DLPF_98HZ;
    static constexpr uint16_t kFIFOPacketSize = 12;

    static constexpr const char* TAG = "MPUReader";

    bool calib_available = false;
    Eigen::Vector3d accel_calib_weights{1, 1, 1};
    Eigen::Vector3d accel_calib_biases{0, 0, 0};
    Eigen::Vector3d gyro_offsets{0, 0, 0};

    QueueHandle_t imu_measurement_queue;
    
    uint16_t kSampleRate = 100;

    void LinearLeastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::VectorXd& x);

    static void IMUReadTaskImpl(void* _this) { ((MPUReader*)_this)->IMUReadTask(); };
    void IMUReadTask();
    
public:
    MPUReader(QueueHandle_t imu_measurement_queue);
    MPUReader(QueueHandle_t imu_measurement_queue, uint16_t sample_rate);
    esp_err_t Configure();
    esp_err_t Configure(mpud::accel_fs_t kAccelFS,
                        mpud::gyro_fs_t kGyroFS,
                        mpud::dlpf_t kDLPF,
                        uint16_t kSampleRate);
    void Calibrate();
    void SetCalibration(
        const Eigen::Vector3d& accel_calib_weights,
        const Eigen::Vector3d& accel_calib_biases,
        const Eigen::Vector3d& gyro_offsets);
    //https://stackoverflow.com/questions/45831114/c-freertos-task-invalid-use-of-non-static-member-function
    void StartReadTask() { xTaskCreate(this->IMUReadTaskImpl, "IMUReadTask", 1024*14, this, 6, NULL); };
    float GetDeltaTime() { return 1.f / kSampleRate; };
};

#endif // MPU_READER_H