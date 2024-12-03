# Lab 3 - Sensor Fusion: Complementary Filter and Kalman Filter

In this lab, you will combine the readings from the accelerometer and gyroscope to calculate the roll and pitch angles using two popular sensor fusion techniques: **Complementary Filter** and **Kalman Filter**. You will also have the opportunity to observe and compare the performance of both methods in a real-world application.

## Key Concepts:
- **Sensor Fusion**: The process of combining data from multiple sensors to improve accuracy and reliability. In this lab, we will combine accelerometer and gyroscope data to calculate the roll and pitch angles.
- **Complementary Filter**: A simple, efficient filter that combines high-pass filtered gyroscope data and low-pass filtered accelerometer data to estimate angles. 
- **Kalman Filter**: A more advanced technique that estimates the true value by considering the noise in both the accelerometer and gyroscope data. It uses a recursive algorithm to combine sensor measurements with predicted values.

## Things to note
- The code for displaying the results of both the complementary filter and Kalman filter has been commented out for clarity and to allow you to focus on one filter at a time.
- It is recommended to display one filter’s output at a time to better understand their behavior and performance.
## Code for Sensor Fusion:
### Complementary Filter and Kalman Filter:
```c
// Complementary Filter
CF_pitch = 0.1 * pitch_acc + 0.9 * (CF_pitch + imu.gyro[0] * dt);
CF_roll = 0.1 * roll_acc + 0.9 * (CF_roll + imu.gyro[1] * dt);
// Display the complementary filter results (commented out in code)
// sprintf(sbuf, "%.2f,%.2f,%.2f\r\n", CF_roll, roll_gyro, roll_acc);
// HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);

// Kalman Filter
roll_KF = roll_KF + (-imu.gyro[1]) * dt;
pitch_KF = pitch_KF + imu.gyro[0] * dt;

roll_var_g = roll_var_g + dt * dt * var_gyro;
pitch_var_g = pitch_var_g + dt * dt * var_gyro;

KG_roll = roll_var_g / (roll_var_g + var_acc);
KG_pitch = pitch_var_g / (pitch_var_g + var_acc);

roll_KF = roll_KF + KG_roll * (roll_acc - roll_KF);
pitch_KF = pitch_KF + KG_pitch * (pitch_acc - pitch_KF);

roll_var_g = (1 - KG_roll) * roll_var_g;
pitch_var_g = (1 - KG_pitch) * pitch_var_g;

// Display the Kalman filter results (commented out in code)
// sprintf(sbuf, "%.2f,%.2f,%.2f\r\n", pitch_KF, pitch_gyro, pitch_acc);
// sprintf(sbuf, "%.2f,%.2f,%.2f\r\n", roll_KF, roll_gyro, roll_acc);
// HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);
