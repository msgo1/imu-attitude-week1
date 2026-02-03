# Week 1 Report

- Implemented a 2-axis complementary filter (gyro integration + accel correction) for roll/pitch estimation.
- Observed +/-180° jumps in roll due to angle wrapping from atan2; fixed it by aligning measurement angles to the prediction branch (wrap-to-pi).
- Result: roll/pitch estimates closely track ground-truth in a synthetic IMU scenario.