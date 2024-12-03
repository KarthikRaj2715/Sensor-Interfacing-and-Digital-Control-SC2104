# Lab 2 - IMU Readings (Accelerometer and Gyroscope)

In this lab, you will learn how to retrieve data from the accelerometer and gyroscope and display it using the SerialPlot software. The key focus of this lab is to demonstrate how to interpret the accelerometer and gyroscope data for applications such as orientation and motion sensing.

## Key Concepts:
- **Accelerometer**: Measures acceleration in the X, Y, and Z directions, which can be used to calculate pitch and roll angles.
- **Gyroscope**: Measures angular velocity around the X, Y, and Z axes, which can be used to track rotational motion.

## Things to Take Note:
1. The X and Y directions are indicated on the board. Use these directions to calculate the pitch and roll angles.
2. In Lab 1, SPI was used for communication; however, from Lab 2 onwards, we will be using I2C for communication with the sensors.
3. To display the readings clearly, it is advisable to show the accelerometer data first, followed by the gyroscope data, or vice versa. I have already commented out the code for displaying the gyroscope readings, so by default, the accelerometer data is shown. To display the gyroscope data, simply comment out the accelerometer reading code and uncomment the code for the gyroscope.

## Code for Displaying the Accelerometer Data:
```c
// Calculate angles from accelerometer data
roll_acc = atan2 (imu.acc[0]/9.8, imu.acc[2]/9.8) *(180/M_PI);
pitch_acc = atan2(imu.acc[1]/9.8, imu.acc[2]/9.8)*(180/M_PI);

sprintf(sbuf, "%.2f, %.2f\r\n", pitch_acc, roll_acc);
HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);

## Code for Displaying the Gyroscope Data:
```c
// Gyroscope
// roll_gyro = roll_gyro - imu.gyro[1] * dt;
// pitch_gyro = pitch_gyro - imu.gyro[0] * dt;
// yaw_gyro = yaw_gyro - imu.gyro[2] * dt;

sprintf(sbuf, "%.2f, %.2f, %.2f\r\n", pitch_gyro, roll_gyro, yaw_gyro);
HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);
