#include "MPUReader.h"

MPUReader::MPUReader(QueueHandle_t imu_measurement_queue) 
    : imu_measurement_queue(imu_measurement_queue)
{

}

esp_err_t MPUReader::Configure()
{
    return Configure(kAccelFS, kGyroFS, kDLPF, kSampleRate);
}

esp_err_t MPUReader::Configure(mpud::accel_fs_t accelFS, mpud::gyro_fs_t gyroFS, mpud::dlpf_t dlpf, uint16_t sampleRate)
{
    ESP_ERROR_CHECK(MPU.setAccelFullScale(accelFS));
    ESP_ERROR_CHECK(MPU.setGyroFullScale(gyroFS));
    ESP_ERROR_CHECK(MPU.setSampleRate(sampleRate));
    ESP_ERROR_CHECK(MPU.setDigitalLowPassFilter(dlpf));
    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));
    vTaskDelay(200 / portTICK_PERIOD_MS);
    return ESP_OK;
}

void MPUReader::Calibrate()
{
    constexpr uint16_t sample_rate = 1000;
    constexpr mpud::dlpf_t DLPF = mpud::DLPF_188HZ;
    constexpr mpud::accel_fs_t AccelFS = mpud::ACCEL_FS_16G;
    constexpr mpud::gyro_fs_t GyroFS   = mpud::GYRO_FS_250DPS;

    Configure(AccelFS, GyroFS, DLPF, sample_rate);

    ESP_LOGI(TAG, "Beginning Calibration");
    constexpr int samples_per_axis = 50;
    constexpr int samples_per_axis_3x = samples_per_axis * 3;

    Eigen::VectorXd ground_truth;
    for (int i = 0; i < samples_per_axis_3x; i++)
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
    Eigen::Vector<double, samples_per_axis_3x> data_x;
    Eigen::Vector<double, samples_per_axis_3x> data_y;
    Eigen::Vector<double, samples_per_axis_3x> data_z;

    for (int i=0; i<6; i++)
    {
        ESP_LOGI(TAG, "Move IMU to position %d", i+1);
        vTaskDelay(2500 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Collecting %d samples", samples_per_axis);
        int sample_count = 0;
        while (sample_count < samples_per_axis)
        {
            IMUMeasurement imu_meas;
            if (xQueueReceive(imu_measurement_queue,&imu_meas,0) == pdTRUE)
            {
                Eigen::Vector3d accel_meas = imu_meas.accel_meas;
                switch (i)
                {
                    case 0:
                        data_z[sample_count] = accel_meas(2);
                        data_y[sample_count+samples_per_axis*2] = accel_meas(1);
                        break;
                    case 1:
                        data_z[sample_count + samples_per_axis] = accel_meas(2);
                        break;
                    case 2:
                        data_x[sample_count] = accel_meas(0);
                        data_z[sample_count + samples_per_axis*2] = accel_meas(2);
                        break;
                    case 3:
                        data_x[sample_count + samples_per_axis] = accel_meas(0);
                        break;
                    case 4:
                        data_y[sample_count] = accel_meas(1);
                        data_x[sample_count + samples_per_axis*2] = accel_meas(0);
                        break;
                    case 5:
                        data_y[sample_count + samples_per_axis] = accel_meas(1);
                        break;
                    default:
                        break;
                }
                sample_count++;
            }
        }
    }

    Eigen::MatrixXd Ax;
    for (int i = 0; i < samples_per_axis_3x; i++)
    {
        Ax(i, 0) = 1;
        Ax(i, 1) = data_x[i];
    }
    Eigen::VectorXd beta_x{0,1};
    LinearLeastSquares(Ax, ground_truth, beta_x); 

    Eigen::MatrixXd Ay;
    for (int i = 0; i < samples_per_axis_3x; i++)
    {
        Ay(i, 0) = 1;
        Ay(i, 1) = data_y[i];
    }
    Eigen::VectorXd beta_y{0,1};
    LinearLeastSquares(Ay, ground_truth, beta_y);

    Eigen::MatrixXd Az;
    for (int i = 0; i < samples_per_axis_3x; i++)
    {
        Az(i, 0) = 1;
        Az(i, 1) = data_z[i];
    }
    Eigen::VectorXd beta_z{0,1};
    LinearLeastSquares(Az, ground_truth, beta_z);

    accel_calib_weights = {beta_x(1), beta_y(1), beta_z(1)};
    accel_calib_biases = {beta_x(0), beta_y(0), beta_z(0)};

    ESP_LOGI(TAG, "Calibrating Gyro..");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    int gyro_samples = 500;
    gyro_offsets = {0,0,0};
    int gyro_sample_count = 0;
    while(1)
    {
        IMUMeasurement imu_meas;
        if (xQueueReceive(imu_measurement_queue,&imu_meas,0) == pdTRUE)
        {
            Eigen::Vector3d gyro_meas = imu_meas.gyro_meas;
            gyro_offsets += gyro_meas;
        }
        gyro_sample_count++;
        if (gyro_sample_count >= gyro_samples)
        {
            break;
        }
    }
    gyro_offsets /= gyro_samples;

    ESP_LOGI(TAG, "Calibration complete");

    Configure(); // return to default settings

    calib_available = true;
}

void MPUReader::IMUReadTask()
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

    Configure();

    ESP_ERROR_CHECK(MPU.resetFIFO());

    Eigen::Vector3d accel_meas{0, 0, 0};
    Eigen::Vector3d gyro_meas{0, 0, 0};
    mpud::raw_axes_t accel_raw;   
    mpud::raw_axes_t gyro_raw; 
    mpud::float_axes_t accel_raw_g;
    mpud::float_axes_t gyroRPS; 

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

        accel_raw.x = buffer[0] << 8 | buffer[1];
        accel_raw.y = buffer[2] << 8 | buffer[3];
        accel_raw.z = buffer[4] << 8 | buffer[5];
        gyro_raw.x  = buffer[6] << 8 | buffer[7];
        gyro_raw.y  = buffer[8] << 8 | buffer[9];
        gyro_raw.z  = buffer[10] << 8 | buffer[11];

        accel_raw_g = mpud::accelGravity(accel_raw, mpud::ACCEL_FS_4G);
        gyroRPS = mpud::gyroRadPerSec(gyro_raw, mpud::GYRO_FS_500DPS); 

        if (calib_available)
        {
            accel_meas = 9.81 * Eigen::Vector3d
            (
                accel_raw_g.x * accel_calib_weights[0], 
                accel_raw_g.y * accel_calib_weights[1],
                accel_raw_g.z * accel_calib_weights[2]
            );

            accel_meas += accel_calib_biases;

            gyro_meas = Eigen::Vector3d(gyroRPS.x, gyroRPS.y, gyroRPS.z) - gyro_offsets;
        }
        else
        {
            accel_meas = 9.81 * Eigen::Vector3d(accel_raw_g.x, accel_raw_g.y, accel_raw_g.z);
            gyro_meas = Eigen::Vector3d(gyroRPS.x, gyroRPS.y, gyroRPS.z);
        }

        IMUMeasurement imu_meas;
        imu_meas.accel_meas = accel_meas;
        imu_meas.gyro_meas = gyro_meas;

        if(xQueueSend(imu_measurement_queue,&imu_meas,0) != pdTRUE)
        {
            printf("queue full\n");
        }

        vTaskDelay(1000 / (2 * portTICK_PERIOD_MS * kSampleRate));
    }
}

void MPUReader::LinearLeastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::VectorXd& x)
{
    //https://blog.fearcat.in/a?ID=01650-bd7947cb-98f1-4729-ab1b-e1b521ef9b68
    x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}
