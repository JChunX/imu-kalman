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

#include "MPU.hpp"   
#include "mpu/math.hpp"  
#include "mpu/types.hpp"  
#include "I2Cbus.hpp"
#include "KalmanFilter.h"
#include "MPUReader.h"


static const char* TAG = "imu_kalman";

Eigen::Vector3d accel_meas{0, 0, 0};
Eigen::Vector3d gyro_meas{0, 0, 0};
Eigen::Quaterniond q{1, 0, 0, 0};
float roll{0}, pitch{0}, yaw{0};

Eigen::Vector3d g{0, 0, -9.81};
QueueHandle_t imu_measurement_queue;
const uint16_t kSampleRate = 100;
const double dt = 1.0 / kSampleRate;

Eigen::Matrix<double, 9, 9> get_A(double dt);
void state_est_task(void *);
void log_task(void *);
Eigen::Vector3d to_euler(const Eigen::Quaterniond& q);
 
extern "C" void app_main(void)
{

    fflush(stdout);
    printf("Hello world!\n");

    imu_measurement_queue = xQueueCreate(10, sizeof(IMUMeasurement));

    MPUReader mpu_reader = MPUReader(imu_measurement_queue, kSampleRate);

    mpu_reader.StartReadTask();
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    //mpu_reader.Calibrate();
    mpu_reader.SetCalibration(
        Eigen::Vector3d{0.991080, 0.994683, 0.969937}, 
        Eigen::Vector3d{0.800342, 0.209465, 0.643287}, 
        Eigen::Vector3d{0, 0, 0}
    );

    xTaskCreate(state_est_task,"EstTask",12 * 1024,nullptr,8,nullptr);
    xTaskCreate(log_task,"LogTask",2 * 1024,nullptr,3,nullptr);

    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
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

void state_est_task(void *)
{
    Eigen::Vector3d accel_meas_raw;
    Eigen::Vector3d gyro_meas_raw;
    while (1)
    {
        vTaskDelay(1);
        IMUMeasurement imu_meas;
        if(xQueueReceive(imu_measurement_queue,&imu_meas,0 ) == pdTRUE)
        {
            accel_meas_raw = imu_meas.accel_meas;
            gyro_meas_raw = imu_meas.gyro_meas;

            // gyro integration w/ quaternions
            double angle = gyro_meas_raw.norm() * dt;
            Eigen::Vector3d axis = gyro_meas_raw.normalized();
            Eigen::Quaterniond q_delta(Eigen::AngleAxisd(angle, axis));
            q = q * q_delta;

            // tilt correction using complementary filter
            Eigen::Quaterniond q_accel_body(0, accel_meas_raw.x(), accel_meas_raw.y(), accel_meas_raw.z());
            Eigen::Quaterniond q_accel_world = q * q_accel_body * q.inverse();

            Eigen::Vector3d v_accel_world(
                q_accel_world.x()/q_accel_world.norm(), 
                q_accel_world.y()/q_accel_world.norm(), 
                q_accel_world.z()/q_accel_world.norm()
            );
            Eigen::Vector3d n = v_accel_world.cross(Eigen::Vector3d(0, 0, 1));
            double phi = std::acos(v_accel_world.dot(Eigen::Vector3d(0, 0, 1)));
            double alpha = 0.0;//std::max((1-0.1)/(9.81*2) * accel_meas.norm()+0.1, 0.0);
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
            accel_meas = accel_meas_raw;//q_accel_world.vec() + g;
            gyro_meas = gyro_meas_raw;
        }
    }

    vTaskDelete(nullptr);
}

void log_task(void *)
{
    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "==========================");
        printf("%+.2f %+.2f %+.2f ", accel_meas(0), accel_meas(1), accel_meas(2));
        printf("%+.2f %+.2f %+.2f ", gyro_meas(0), gyro_meas(1), gyro_meas(2));
        printf("%+.2f %+.2f %+.2f\n", roll, pitch, yaw);
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