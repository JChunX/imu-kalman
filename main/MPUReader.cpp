#include "MPUReader.h"

MPUReader::MPUReader()
{
    i2c0.begin(SDA, SCL, CLOCK_SPEED); 
    MPU.setBus(i2c0); 
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  

    while (esp_err_t err = MPU.testConnection()) 
    {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    ESP_ERROR_CHECK(MPU.initialize()); 

    mpud::selftest_t retSelfTest;
    while (esp_err_t err = MPU.selfTest(&retSelfTest)) 
    {
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
}

void MPUReader::Calibrate()
{
    ESP_LOGI(TAG, "Beginning Calibration");
    int samples_per_axis = 50;
    char axes[] = {'x', 'y', 'z'};
    std::map<char, std::vector<int>> position_affinities = {
        {'x', {3,4,5}},
        {'y', {5,6,1}},
        {'z', {1,2,3}}
    };

    Eigen::VectorXd ground_truth;
    for (int i = 0; i < samples_per_axis*3; i++)
    {
        if (i < samples_per_axis)
        {
            ground_truth(i) = 9.81;
        }
        else if (i < samples_per_axis*2)
        {
            ground_truth(i) = -9.81;
        }
        else
        {
            ground_truth(i) = 0;
        }
    }
    Eigen::Vector<double, samples_per_axis*3> data_x;
    Eigen::Vector<double, samples_per_axis*3> data_y;
    Eigen::Vector<double, samples_per_axis*3> data_z;

    for (int i=0; i<6; i++)
    {
        ESP_LOGI(TAG, "Move IMU to position %d", i+1);
        vTaskDelay(2500 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Collecting %d samples", samples_per_axis);
        int sample_count = 0;
        while (sample_count < samples_per_axis)
        {
            if (new_accel_data)
            {
                Eigen::Vector3d accel_meas = GetAccel();
                switch (i)
                {
                    case 0:
                        data_z(sample_count) = accel_meas(2);
                        data_y(sample_count+samples_per_axis*2) = accel_meas(1);
                        break;
                    case 1:
                        data_z(sample_count + samples_per_axis) = accel_meas(2);
                        break;
                    case 2:
                        data_x(sample_count) = accel_meas(0);
                        data_z(sample_count + samples_per_axis*2) = accel_meas(2);
                        break;
                    case 3:
                        data_x(sample_count + samples_per_axis) = accel_meas(0);
                        break;
                    case 4:
                        data_y(sample_count) = accel_meas(1);
                        data_x(sample_count + samples_per_axis*2) = accel_meas(0);
                        break;
                    case 5:
                        data_y(sample_count + samples_per_axis) = accel_meas(1);
                        break;
                    default:
                        break;
                }
                sample_count++;
            }
        }
    }

    Eigen::MatrixXd Ax;
    for (int i = 0; i < samples_per_axis*3; i++)
    {
        Ax(i, 0) = 1;
        Ax(i, 1) = data_x(i);
    }
    Eigen::VectorXd beta_x{0,1};
    LinearLeastSquares(Ax, ground_truth, beta_x); 

    Eigen::MatrixXd Ay;
    for (int i = 0; i < samples_per_axis*3; i++)
    {
        Ay(i, 0) = 1;
        Ay(i, 1) = data_y(i);
    }
    Eigen::VectorXd beta_y{0,1};
    LinearLeastSquares(Ay, ground_truth, beta_y);

    Eigen::MatrixXd Az;
    for (int i = 0; i < samples_per_axis*3; i++)
    {
        Az(i, 0) = 1;
        Az(i, 1) = data_z(i);
    }
    Eigen::VectorXd beta_z{0,1};
    LinearLeastSquares(Az, ground_truth, beta_z);

    accel_calib_weights = {beta_x(1), beta_y(1), beta_z(1)};
    accel_calib_biases = {beta_x(0), beta_y(0), beta_z(0)};
    ESP_LOGI(TAG, "Calibration complete");

    calib_available = true;
}

void IMUReadTask()
{
    while (1)
    {
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

        mpud::raw_axes_t accel_raw;   
        mpud::raw_axes_t gyro_raw; 

        accel_raw.x = buffer[0] << 8 | buffer[1];
        accel_raw.y = buffer[2] << 8 | buffer[3];
        accel_raw.z = buffer[4] << 8 | buffer[5];
        gyro_raw.x  = buffer[6] << 8 | buffer[7];
        gyro_raw.y  = buffer[8] << 8 | buffer[9];
        gyro_raw.z  = buffer[10] << 8 | buffer[11];

        mpud::float_axes_t accel_raw_g;
        mpud::float_axes_t gyroRPS; 

        accel_raw_g = mpud::accelGravity(accel_raw, mpud::ACCEL_FS_4G);
        gyroRPS = mpud::gyroRadPerSec(gyro_raw, mpud::GYRO_FS_500DPS); 

        if (calib_available)
        {
            accel_meas = accel_calib_weights.dot(9.81 * Eigen::Vector3d
            (
                accel_raw_g.x, 
                accel_raw_g.y,
                accel_raw_g.z
            )) + accel_calib_biases;

            gyro_meas = Eigen::Vector3d(gyroRPS.x, gyroRPS.y, gyroRPS.z) - gyro_offsets;
        }
        else
        {
            accel_meas = 9.81 * Eigen::Vector3d(accel_raw_g.x, accel_raw_g.y, accel_raw_g.z);
            gyro_meas = Eigen::Vector3d(gyroRPS.x, gyroRPS.y, gyroRPS.z);
        }

        new_accel_data = true;
        new_gyro_data = true;
    }
}

Eigen::Vector3d MPUReader::GetAccel()
{
    new_accel_data = false;
    return accel_meas;
}

Eigen::Vector3d MPUReader::GetGyro()
{
    new_gyro_data = false;
    return gyro_meas;
}

void MPUReader::LinearLeastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::VectorXd& x)
{
    x = A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);
}