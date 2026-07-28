## Mohan's proposed Solution: The 1000Hz Flight Control Loop Pipeline
**Architecture:** Cascaded PID with a Linear Mixing Matrix. 
**Execution:** Pinned to ESP32 FreeRTOS Core 1. Target loop time: `< 1.0 ms` (1000Hz).

This pipeline represents the entire digital nervous system of the drone. It must execute the following 5 steps sequentially without interruption.

### Step 1: State Estimation (Where are we?)
*   **Action:** Read raw Gyroscope ($\omega$) and Accelerometer ($a$) data from the BMI160 IMU via SPI. Apply the Madgwick Quaternion Filter to fuse the sensors into a clean 3D orientation.
*   **The Math:** Gradient descent optimization using Quaternions to correct fast, drifting gyro integration with the slow, noisy gravity vector.
*   `q_dot = 0.5 * (q ⊗ ω) - β * (∇f / |∇f|)`

### Step 2: The Outer Loop (The "Angle" P-Controller)
*   **Action:** Compare the drone's *Current Angle* to the *Desired Angle* (commanded by the pilot via ESP-NOW joystick).
*   **The Math:** A pure Proportional ($P$) calculation. It maps the absolute geometric angle error into a desired rotational speed (degrees per second).
*   `Rate_Target = Kp_angle * (Angle_Desired - Angle_Current)`

### Step 3: The Inner Loop (The 3 "Rate" PIDs)
*   **Action:** The core brain. It compares the `Rate_Target` from Step 2 against the *actual* rotation rate currently being measured by the Gyroscope, and calculates the corrective forces.
*   **The Math:** Three independent PID equations (for Roll, Pitch, and Yaw) calculating the abstract corrective torques required to hit the target rate.
*   `Torque_Output = (Kp * e) + (Ki * ∫e dt) + (Kd * de/dt)` *(where e = Rate_Target - Gyro_Rate)*

### Step 4: Actuator Allocation (The Mixing Matrix)
*   **Action:** Transforms the 3 abstract torque forces from the PID controllers into 4 physical motor speed commands. 
*   **The Math:** A linear combination matrix that solves cross-coupling by distributing the Roll, Pitch, and Yaw forces symmetrically across the True-X frame.
*   `M1 (Front Left)  = Throttle + PID_Pitch + PID_Roll - PID_Yaw`
*   `M2 (Front Right) = Throttle + PID_Pitch - PID_Roll + PID_Yaw`
*   `M3 (Back Right)  = Throttle - PID_Pitch - PID_Roll - PID_Yaw`
*   `M4 (Back Left)   = Throttle - PID_Pitch + PID_Roll + PID_Yaw`

### Step 5: Hardware Output (DSHOT)
*   **Action:** Send the 4 calculated motor values (0% - 100%) to the Electronic Speed Controllers (ESCs).
*   **The Math/Logic:** Convert the float values to 16-bit DSHOT digital packets and blast them using the ESP32's DMA (Direct Memory Access) and RMT peripherals so the CPU does not freeze. 
*   `DSHOT_Packet = (Motor_Value << 5) ⊕ CRC_Checksum`