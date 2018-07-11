## Project: Building an Estimator

---
_[x, y, z, x_dot, y_dot, z_dot, yaw]_ - EKF
_[p, q, r]_ - Gyro measurements 
_[roll, pitch]_ - Complementary Filter

*u = [xddot_Bfr, yddot_Bfr, zddot_Bfr, yaw_dot]* 
As control imput use Accelerometer -> [xddot_Bfr, yddot_Bfr, zddot_Bfr]
                    Rate Gyro -> yaw_dot
                    
### Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

#### The calculated standard deviation should correctly capture  of approximately 68% the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.

1. Used pandas library to get sample standard deviation over requested axis. (`ExtractingStd.py`):
```python

    def find_std(csv_name):
        df = pd.read_csv(csv_name, delimiter = ',', header = 0, usecols = [1], parse_dates=True)
        data_std = df.std()
        return data_std
    ```
  
Quad.GPS.X   | Quad.IMU.AX 
-------------|-------------   
0.717478     |  0.511142

### Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

#### The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.

1. Improve a complementary filter-type attitude filter by replacing a small-angle approximation integration method, so that the current attitude estimate (rollEst, pitchEst and ekfState(6)) are used to integrate the body rates (`p = gyro.x`, `q = gyro.y`) into new Euler angles - nonlinear complementary filter.
    
    1. use the `Quaternion` class, which has a `FromEuler123_RPY` function for creating an attitude `attEst` quaternion from Euler Roll/Pitch\Yaw. 
    2. Quaternion function `IntegrateBodyRate_fast` defines dq to be the quaternion that consists of the measurement of the angular rates from the IMU in the body frame (`p = gyro.x`, `q = gyro.y`, `r = gyro.z`),and give a predicted quaternion, `predictedQt = dq * qt`
    3. Extract predicted roll, pitch, yaw.

    ```cpp
        Quaternion<float> attEst = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
        Quaternion<float> predictedQt = attEst.IntegrateBodyRate_fast(gyro.x, gyro.y, gyro.z, dtIMU / 2.f);
        float predictedPitch = predictedQt.Pitch();
        float predictedRoll = predictedQt.Roll();
        ekfState(6) = predictedQt.Yaw();
  ```

2. The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers
Why use 
```cpp
    accelPitch = atan2f(-accel.x, 9.81f);
```
instead of 
```cpp
    accelPitch = asinf(accel.x, 9.81f);
```
???



### Implement all of the elements of the prediction step for the estimator.

#### The prediction step should include the state update element (`PredictState()` function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.

1. dt is the time duration for which you should predict. It will be very short (on the order of 1ms), so simplistic integration methods are fine here

### Implement lateral position control in C++.

#### The controller should use the local NE position and velocity to generate a commanded local acceleration

```cpp
    if (velCmd.magXY() > maxSpeedXY)
    {
        velCmd = velCmd * maxSpeedXY / velCmd.magXY();
    }
    V3F acc_p_term = kpPosXY * (posCmd - pos);
    V3F acc_v_term = kpVelXY * (velCmd - vel);
    accelCmd = acc_p_term + acc_v_term + accelCmdFF;
    
    if (accelCmd.magXY() > maxAccelXY)
    {
        accelCmd *= maxAccelXY / accelCmd.magXY();
    }
```

### Implement yaw control in C++.

#### The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

```cpp
    yawCmd = fmodf(yawCmd, F_PI);
    float yaw_err = yawCmd - yaw;
    if (yaw_err > F_PI)
    {
        yaw_err -= 2 * F_PI;
    }
    else if (yaw_err < - F_PI)
    {
        yaw_err += 2 * F_PI;
    }
    yawRateCmd = kpYaw * yaw_err;
```

### Implement calculating the motor commands given commanded thrust and moments in C++.

#### The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

```cpp
    float omega_sqr[4];
    float lx = L * cos(M_PI_4);
    float ly = L * sin(M_PI_4);
    float c_bar = collThrustCmd;
    float p_bar = momentCmd.x / lx;
    float q_bar = momentCmd.y / ly;
    float r_bar = - momentCmd.z / kappa;

    omega_sqr[0] = (c_bar + p_bar + r_bar + q_bar) / 4.f;
    omega_sqr[1] = (c_bar - p_bar - r_bar + q_bar) / 4.f;
    omega_sqr[3] = (c_bar - p_bar + r_bar - q_bar) / 4.f;
    omega_sqr[2] = (c_bar + p_bar - r_bar - q_bar) / 4.f;

    cmd.desiredThrustsN[0] = omega_sqr[0];
    cmd.desiredThrustsN[1] = omega_sqr[1]; // front right
    cmd.desiredThrustsN[2] = omega_sqr[2]; // rear left
    cmd.desiredThrustsN[3] = omega_sqr[3]; // rear rightt
```

### Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.

#### Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used).

- [x] Attitude Control Scenario:
![2_AttitudeControl](./control_im/AttitudeControlScenario.png)

- [x] Position Control Scenario: 
![3_PositionControl](./control_im/PositionControlScenario.png)

- [x] Nonidealities: 
![Nonidealities Video](./control_im/NonidealitiesScenario.mov)

- [x] Trajectory Following:
![Nonidealities Video](./control_im/TrajectoryFollowing.mov)


# Extra Challenges: 

`FigureEightFF.txt` is provided with velocity and acceleration information. Though passing all the tests, I still can't reach the smooth Quad2 behavior from ProjectReadme :confused:

