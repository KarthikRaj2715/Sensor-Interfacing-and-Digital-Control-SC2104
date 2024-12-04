# Lab 5 - Self-Leveling Platform

In this lab, you will integrate the knowledge and skills acquired in previous labs to design and implement a **self-leveling platform**. The goal is to use the **IMU** (Inertial Measurement Unit) to measure the **roll** and **pitch** angles of the platform and adjust the **servo motors** dynamically to maintain a level orientation. This is achieved using a **digital PID control strategy**, ensuring precision and stability.

## Key Objectives
1. **IMU Integration**: 
   - Use the accelerometer and gyroscope to measure the roll and pitch angles in real-time.
   
2. **PID Control**:
   - Apply a digital PID control strategy to compute and send precise adjustments to the servo motors based on the angle deviations.

3. **Servo Motor Control**:
   - Continuously adjust the servo motor positions to counteract disturbances and maintain a level platform.

## Tasks
1. Interface the IMU with the microcontroller to retrieve accurate roll and pitch measurements.
2. Implement a digital PID control algorithm to calculate corrections based on the measured angles.
3. Control the servo motors to stabilize the platform based on PID outputs.
4. Observe and fine-tune the PID parameters to achieve smooth and stable leveling.

## Things to Note
- **Tuning the PID Parameters**:
  - Start with a high proportional gain (**P**) to observe immediate corrections.
  - Gradually adjust the integral (**I**) and derivative (**D**) gains to optimize the response and minimize oscillations.
- Ensure that the servo motors are properly calibrated and aligned with the platform.

## Expected Outcome
By the end of this lab, you should have a functional self-leveling platform that demonstrates:
- Effective integration of IMU and servo motor control.
- Stability and accuracy in maintaining a level platform, even under external disturbances.
- A clear understanding of the practical application of PID control in dynamic systems.

## Conclusion
This lab consolidates the knowledge gained from previous labs and introduces a practical application of embedded system design, sensor integration, and control strategies. Successfully completing this lab will enhance your expertise in real-time control systems and sensor-based automation.
