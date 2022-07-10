//#define EIGEN_NO_MALLOC

#include <iostream>
#include <functional>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "KalmanFilter.h"
#include "MPU.hpp"        
#include "mpu/math.hpp"  
#include "mpu/types.hpp"  
#include "I2Cbus.hpp"

static constexpr gpio_num_t SDA = GPIO_NUM_21;
static constexpr gpio_num_t SCL = GPIO_NUM_22;
static constexpr uint32_t CLOCK_SPEED = 400000;
static constexpr uint16_t kSampleRate = 500;
static const char* TAG = "imu_kalman";
static constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
static constexpr mpud::gyro_fs_t kGyroFS   = mpud::GYRO_FS_500DPS;
static constexpr mpud::dlpf_t kDLPF        = mpud::DLPF_98HZ;

mpud::raw_axes_t accelRaw;   
mpud::raw_axes_t gyroRaw;  
mpud::float_axes_t accel;
mpud::float_axes_t gyroRPS; 

Eigen::Vector3d accel_meas{0, 0, 0};
Eigen::Quaterniond q{1, 0, 0, 0};
float roll{0}, pitch{0}, yaw{0};

Eigen::Vector3d Ex{1, 0, 0}, Ey{0, 1, 0}, Ez{0, 0, 1};
Eigen::Vector3d g{0, 0, -9.81};

Eigen::Matrix<double, 9, 9> get_A(double dt);
static void mpu_task(void *);
static void kalman_task(void *);
static void log_task(void *);
Eigen::Vector3d to_euler(const Eigen::Quaterniond& q);

MPU_t MPU;  
 
extern "C" void app_main(void)
{
    fflush(stdout);

    i2c0.begin(SDA, SCL, CLOCK_SPEED);    

    xTaskCreate(
        kalman_task,
        "KalmanFilter",
        6 * 1024,
        nullptr,
        7,
        nullptr
    );

    xTaskCreate(
        mpu_task,
        "MPUTask",
        4 * 1024,
        nullptr,
        6,
        nullptr
    );

    xTaskCreate(
        log_task,
        "LogTask",
        2 * 1024,
        nullptr,
        5,
        nullptr
    );
}

Eigen::Matrix<double, 9, 9> get_A(double dt)
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

static void mpu_task(void *)
{
    MPU.setBus(i2c0); 
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);

    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    ESP_ERROR_CHECK(MPU.initialize()); 

    mpud::selftest_t retSelfTest;
    while (esp_err_t err = MPU.selfTest(&retSelfTest)) {
        ESP_LOGE(TAG, "Failed to perform MPU Self-Test, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU Self-Test result: Gyro=%s Accel=%s",  
             (retSelfTest & mpud::SELF_TEST_GYRO_FAIL ? "FAIL" : "OK"),
             (retSelfTest & mpud::SELF_TEST_ACCEL_FAIL ? "FAIL" : "OK"));

    ESP_ERROR_CHECK(MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY));
    ESP_ERROR_CHECK(MPU.setAccelFullScale(kAccelFS));
    ESP_ERROR_CHECK(MPU.setGyroFullScale(kGyroFS));
    ESP_ERROR_CHECK(MPU.setSampleRate(kSampleRate));
    ESP_ERROR_CHECK(MPU.setDigitalLowPassFilter(kDLPF));

    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));
    constexpr uint16_t kFIFOPacketSize = 12;

    while (1)
    {
        // Check FIFO count
        uint16_t fifocount = MPU.getFIFOCount();
        if (esp_err_t err = MPU.lastError()) {
            ESP_LOGE(TAG, "Error reading fifo count, %#X", err);
            MPU.resetFIFO();
            continue;
        }
        if (fifocount == 0) {
            continue;
        }
        if (fifocount > kFIFOPacketSize * 2) {
            if (!(fifocount % kFIFOPacketSize)) {
                ESP_LOGE(TAG, "Sample Rate too high!, not keeping up the pace!, count: %d", fifocount);
            }
            else {
                ESP_LOGE(TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", kFIFOPacketSize, fifocount);
            }
            MPU.resetFIFO();
            continue;
        }
        // Burst read data from FIFO
        uint8_t buffer[kFIFOPacketSize];
        if (esp_err_t err = MPU.readFIFO(kFIFOPacketSize, buffer)) {
            ESP_LOGE(TAG, "Error reading sensor data, %#X", err);
            MPU.resetFIFO();
            continue;
        }

        accelRaw.x = buffer[0] << 8 | buffer[1];
        accelRaw.y = buffer[2] << 8 | buffer[3];
        accelRaw.z = buffer[4] << 8 | buffer[5];
        gyroRaw.x  = buffer[6] << 8 | buffer[7];
        gyroRaw.y  = buffer[8] << 8 | buffer[9];
        gyroRaw.z  = buffer[10] << 8 | buffer[11];

        accel = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
        gyroRPS = mpud::gyroRadPerSec(gyroRaw, mpud::GYRO_FS_500DPS); 

        constexpr float kDeltaTime = 1.f / kSampleRate;

        // gyro integration w/ quaternions
        Eigen::Vector3d accel_vec = 9.81 * Eigen::Vector3d{accel.x, accel.y, accel.z};
        Eigen::Vector3d gyro_vec = Eigen::Vector3d{gyroRPS.x, gyroRPS.y, gyroRPS.z};

        double angle = gyro_vec.norm() * kDeltaTime;
        Eigen::Vector3d axis = gyro_vec.normalized();
        Eigen::Quaterniond q_delta(Eigen::AngleAxisd(angle, axis));
        q = q * q_delta;

        // tilt correction using complementary filter
        Eigen::Quaterniond q_accel_body(0, accel_vec.x(), accel_vec.y(), accel_vec.z());
        Eigen::Quaterniond q_accel_world = q * q_accel_body * q.inverse();

        Eigen::Vector3d v_accel_world(q_accel_world.x()/q_accel_world.norm(), 
                                      q_accel_world.y()/q_accel_world.norm(), 
                                      q_accel_world.z()/q_accel_world.norm());
        Eigen::Vector3d n = v_accel_world.cross(Eigen::Vector3d(0, 0, 1));
        double phi = std::acos(v_accel_world.dot(Eigen::Vector3d(0, 0, 1)));
        double alpha = 0;//std::max((1-0.1)/(9.81*2) * accel_vec.norm()+0.1, 0.0);
        Eigen::Quaterniond q_tilt(Eigen::AngleAxisd((1-alpha) * phi, n));
        Eigen::Quaterniond q_c = q_tilt * q;
        Eigen::Vector3d euler = to_euler(q_c);
        roll = euler(1) * 180 / M_PI;
        pitch = euler(0) * 180 / M_PI;
        yaw = -euler(2) * 180 / M_PI;

        if (yaw > 180.f)
            yaw -= 360.f;
        else if (yaw < -180.f)
            yaw += 360.f;

        q_accel_world = q_c * q_accel_body * q_c.inverse();
        accel_meas = q_accel_world.vec() + g;
        
        vTaskDelay(1000 / (2 * portTICK_PERIOD_MS * kSampleRate));
    }

    vTaskDelete(nullptr);
}

static void kalman_task(void *)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    Eigen::Vector<double, 9> x_init = Eigen::Vector<double, 9>::Zero();
    Eigen::Matrix<double, 9, 9> P_init = Eigen::Matrix<double, 9, 9>::Identity();Eigen::Matrix<double, 9, 9> Q_init = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 3, 3> R_init = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
    for (int i = 0; i<3; i++)
    {
        H(i,i+6) = 1;
    }
    
    std::function<Eigen::Matrix<double, 9, 9>(double)> A_ptr {&get_A};

    KalmanFilter<9,3> kf(x_init, P_init, Q_init, R_init, A_ptr, H);


    vTaskDelete(nullptr);
}

static void log_task(void *)
{
    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "==========================");
        Eigen::Vector3d accel_vec = 9.81 * Eigen::Vector3d{accel.x, accel.y, accel.z};
        printf("%+.2f %+.2f %+.2f ", accel_vec(0), accel_vec(1), accel_vec(2));
        printf("%+.2f %+.2f %+.2f\n", accel_meas(0), accel_meas(1), accel_meas(2));
        //printf("gyrodps: %+.2f %+.2f %+.2f ", gyroRPS.x*180/M_PI, gyroRPS.y*180/M_PI, gyroRPS.z*180/M_PI);
        //printf("roll: %+.2f pitch: %+.2f yaw: %+.2f\n", roll, pitch, yaw);
    }

}

Eigen::Vector3d to_euler(const Eigen::Quaterniond& q) {
    // https://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen
    Eigen::Vector3d angles;
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}