
#include "Arduino.h"
#include "libraries/BMI088.h"
#include "Nav.h"
#include "Data.h"
#include "Quaternion.h"
Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro gyro(Wire, 0x68);

float accel_raw[3] = {0, 0, 0};
float accel_raw_prev[3] = {0, 0, 0};
float gyro_raw[3] = {0, 0, 0}; // rad/sec
float g_bias[3] = {0, 0, 0};

// Quaternion Stuff
float q_body_mag = 0;
float q_gyro[4] = {0, 0, 0, 0};
float q[4] = {1, 0, 0, 0};
float q_body[4] = {1, 0, 0, 0};
float q_grad[4] = {0, 0, 0, 0};
float omega[3] = {0, 0, 0};
float theta;

bool first_gyro_reading = true;
unsigned long gyro_current_time = 0;
unsigned long gyro_past_time = 0;
float gyro_dt = 0;

// Yaw pitch roll of rocket
float ypr[3] = {0, 0, 0};

float vel_local[3] = {0, 0, 0};
bool firstAccelReading = true;
float accel_dt = 0;
unsigned long accel_current_time = 0;
unsigned long accel_past_time = 0;

Quaternion localAccelQuat;
Quaternion worldAccelQuat;
Quaternion orientation(1,0,0,0);
float worldAccelArray[4] = {0,0,0,0};
float worldAccelAngles[3] = {0, 0, 0};



void zeroGyroscope()
{
    q_gyro[0] = 0;
    q_gyro[1] = 0;
    q_gyro[2] = 0;
    q_gyro[3] = 0;
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
    q_body[0] = 1;
    q_body[1] = 0;
    q_body[2] = 0;
    q_body[3] = 0;
    q_grad[0] = 0;
    q_grad[1] = 0;
    q_grad[2] = 0;
    q_grad[3] = 0;
    omega[0] = 0;
    omega[1] = 0;
    omega[2] = 0;
    ypr[0] = 0;
    ypr[1] = 0;
    ypr[2] = 0;
}

void quatToEuler(float *qBody, float *ypr);

void getGyroBiases()
{
    int count = 0;
    const int averageAmount = 500;
    while (count < averageAmount)
    {
        gyro.readSensor();
        g_bias[0] += gyro.getGyroX_rads();
        g_bias[1] += gyro.getGyroY_rads();
        g_bias[2] += gyro.getGyroZ_rads();
        count += 1;
        Serial.print(".");
        delay(10);
    }
    Serial.println();
    g_bias[0] /= (float)averageAmount;
    g_bias[1] /= (float)averageAmount;
    g_bias[2] /= (float)averageAmount;

    Serial.print(g_bias[0],8);
    Serial.print(" ");
    Serial.print(g_bias[1],8);
    Serial.print(" ");
    Serial.print(g_bias[2],8);
    Serial.println(" ");
}

bool initIMU()
{
    if (accel.begin() < 0)
    {
        return 0;
    }
    if (gyro.begin() < 0)
    {
        return 0;
    }
    Serial.println("Getting Gyro Biases...");
    zeroGyroscope();
    //getGyroBiases();
    g_bias[0] = 0.004;
    g_bias[1] = -0.0009;
    g_bias[2] = 0.00001;
    zeroGyroscope();
    data.worldVx = 0;
    data.worldVy = 0;
    data.worldVz = 0;
    return 1;
}

void getAccel()
{
    accel_current_time = micros();
    accel.readSensor();
    accel_raw[0] = accel.getAccelX_mss();
    accel_raw[1] = accel.getAccelY_mss();
    accel_raw[2] = accel.getAccelZ_mss();
    if (!firstAccelReading)
    {

        accel_dt = (float)(accel_current_time - accel_past_time) / 1000000.0f;
        vel_local[0] += accel_raw[0] * accel_dt;
        vel_local[1] += accel_raw[1] * accel_dt;
        vel_local[2] += accel_raw[2] * accel_dt;
    }

    accel_raw_prev[0] = accel_raw[0];
    accel_raw_prev[1] = accel_raw[1];
    accel_raw_prev[2] = accel_raw[2];

    data.ax = accel_raw[0];
    data.ay = accel_raw[1];
    data.az = accel_raw[2];

    firstAccelReading = false;
    accel_past_time = accel_current_time;
}



void getYPR()
{
    gyro_current_time = micros();
    gyro.readSensor();

    if (!first_gyro_reading)
    {

        omega[0] = gyro.getGyroX_rads() - g_bias[0];
        omega[1] = gyro.getGyroY_rads() - g_bias[1];
        omega[2] = gyro.getGyroZ_rads() - g_bias[2];

        data.gx = omega[0];
        data.gy = omega[1];
        data.gz = omega[2];

        q_body_mag = sqrt(sq(omega[0]) + sq(omega[1]) + sq(omega[2]));
        gyro_dt = ((gyro_current_time - gyro_past_time) / 1000000.0);

        theta = q_body_mag * gyro_dt;
        q_gyro[0] = cos(theta / 2);
        q_gyro[1] = -(omega[0] / q_body_mag * sin(theta / 2));
        q_gyro[2] = -(omega[1] / q_body_mag * sin(theta / 2));
        q_gyro[3] = -(omega[2] / q_body_mag * sin(theta / 2));

        q[0] = q_body[0];
        q[1] = q_body[1];
        q[2] = q_body[2];
        q[3] = q_body[3];

        q_body[0] = q_gyro[0] * q[0] - q_gyro[1] * q[1] - q_gyro[2] * q[2] - q_gyro[3] * q[3];
        q_body[1] = q_gyro[0] * q[1] + q_gyro[1] * q[0] + q_gyro[2] * q[3] - q_gyro[3] * q[2];
        q_body[2] = q_gyro[0] * q[2] - q_gyro[1] * q[3] + q_gyro[2] * q[0] + q_gyro[3] * q[1];
        q_body[3] = q_gyro[0] * q[3] + q_gyro[1] * q[2] - q_gyro[2] * q[1] + q_gyro[3] * q[0];

        
        // For getting world frame acceleration
        float norm = sqrtf(powf(omega[2], 2) + powf(omega[1], 2) + powf(omega[0], 2));
        norm = copysignf(max(abs(norm), 1e-9), norm); // NO DIVIDE BY 0
        orientation *= from_axis_angle(gyro_dt * norm, omega[0] / norm, omega[1] / norm, omega[2] / norm);
        localAccelQuat = Quaternion(0, data.ax, data.ay, data.az);
        worldAccelQuat = orientation.rotate(localAccelQuat);
        data.worldAx = worldAccelQuat.b;
        data.worldAy = worldAccelQuat.c;
        data.worldAz = worldAccelQuat.d;

       

        quatToEuler(q_body, ypr);
        data.yaw = ypr[0];
        data.pitch = ypr[1];
        data.roll = ypr[2];
    }
    first_gyro_reading = false;
    gyro_past_time = gyro_current_time;
}

void quatToEuler(float *qBody, float *ypr)
{
    double sinr_cosp = 2 * (q_body[0] * q_body[1] + q_body[2] * q_body[3]);
    double cosr_cosp = 1 - 2 * (q_body[1] * q_body[1] + q_body[2] * q_body[2]);
    ypr[2] = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    double sinp = 2 * (q_body[0] * q_body[2] - q_body[1] * q_body[3]);
    if (sinp >= 1)
        ypr[1] = 90;
    else if (sinp <= -1)
        ypr[1] = -90;
    else
        ypr[1] = asin(sinp) * RAD_TO_DEG;

    double siny_cosp = 2 * (q_body[0] * q_body[3] + q_body[1] * q_body[2]);
    double cosy_cosp = 1 - 2 * (q_body[2] * q_body[2] + q_body[3] * q_body[3]);
    ypr[0] = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}