import mujoco
import mujoco.viewer
import time
import numpy as np
import math
import threading
# run by typing "mjpython flight_sim.py" in terminal

# ==========================================
# 1. DEFINE THE DRONE IN MJCF (MuJoCo XML)
# ==========================================
# Instead of a separate file, we define our environment right here.
# when changing time step, don't forget to make change in PID control equation too
xml_string = """
<mujoco>
  <option gravity="0 0 -9.81" timestep="0.004"/> <worldbody>
    <light pos="0 0 5" dir="0 0 -1"/>
    <geom type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"/> <body name="drone" pos="0 0 2">
      <freejoint name="free_joint"/> <geom type="box" size="0.05 0.05 0.025" mass="0.5" rgba="0.3 0.3 0.3 1"/>
      
      <site name="motor_1" pos="0.1 -0.1 0.025" type="cylinder" size="0.015 0.02" rgba="1 0 0 1"/>
      <site name="motor_2" pos="0.1 0.1 0.025"  type="cylinder" size="0.015 0.02" rgba="0 0 1 1"/>
      <site name="motor_3" pos="-0.1 0.1 0.025" type="cylinder" size="0.015 0.02" rgba="1 0 0 1"/>
      <site name="motor_4" pos="-0.1 -0.1 0.025" type="cylinder" size="0.015 0.02" rgba="0 0 1 1"/>

    </body>
  </worldbody>
  <actuator>
    <motor name="motor_1" site="motor_1" gear="0 0 1 0 0 -0.05"/>
    <motor name="motor_2" site="motor_2" gear="0 0 1 0 0 0.05"/>
    <motor name="motor_3" site="motor_3" gear="0 0 1 0 0 -0.05"/>
    <motor name="motor_4" site="motor_4" gear="0 0 1 0 0 0.05"/>
    <motor name="torque_roll" joint="free_joint" gear="0 0 0 1 0 0"/>
    <motor name="torque_pitch" joint="free_joint" gear="0 0 0 0 1 0"/>
    <motor name="torque_yaw" joint="free_joint" gear="0 0 0 0 0 1"/>
  </actuator>
</mujoco>
"""
# Motor 1 -> front right, Motor 2 -> front left, Motor 3 -> back left, motor 4 ->back right

# ==========================================
# 2. INITIALIZE THE ENGINE
# ==========================================
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)

# ==========================================
# 3. THE FLIGHT CONTROL LOOP
# ==========================================
print("MuJoCo Engine Running. Close the window to exit.")

# --- CONSTANTS FOR GYROSCOPIC MATH ---
# J_r: Rotational inertia of a single motor bell and propeller (kg * m^2)
# (e.g., 0.000015 is a realistic estimation for a standard 5-inch racing prop)
J_r = 0.000015  

# Thrust constant (k): Used to estimate RPM from thrust. (Thrust = k * RPM^2)
k_thrust = 0.0001 

# Motor spin directions (1 for Counter-Clockwise, -1 for Clockwise)
# In a standard X-frame: FR and BL spin CCW. FL and BR spin CW.
MOTOR_SPIN = {
    0: 1,   # Front-Right (CCW)
    1: -1,  # Front-Left (CW)
    2: 1,  # Back-Left (CCW)
    3: -1    # Back-Right (CW)
}
# PID constants
Kp = [0.7, 0.7, 0.05] # [roll, pitch, yaw]
Kd = [0.4, 0.4, 0.023]
Ki = [0.0005, 0.0005, 0.001]

target_angle = [-0.1,0.2,2] # [roll, pitch, yaw]
error_sum = [0,0,0]
pid_torque = [0,0,0]

# The drone chassis is 0.5kg. Gravity is 9.81.
# Total force needed to hover = 4.905 Newtons.
# Divided by 4 motors = ~1.226 N per motor.
throttle = 1.24
drone_angle = [0,0,0] # roll pitch yaw

last_render_time = time.time()

# live pid tuner
def dynamic_pid_tuner():
    global Kp, Ki, Kd
    
    print("\n" + "="*50)
    print("LIVE PID TUNER ACTIVE")
    print("Type three numbers separated by spaces to update Kp, Ki, Kd.")
    print("Example: 1.5 0.01 0.2")
    print("="*50 + "\n")
    
    while True:
        try:
            print("Choose which PID constant to change. Type in the initial letter: ")
            selected_constant = input()

            print("Choose the direction of PID being changed (type 0 for roll, 1 for pitch, 2 for yaw): ")
            d = int(input())
            
            print("type in the number to be changed to: ")
            num = float(input())
            if selected_constant == "p":
                Kp[d] = num
            elif selected_constant == "i":
                Ki[d] = num
                error_sum = 0
            elif selected_constant == "d":
                Kd[d] = num
            try:
                print(f">>> UPDATED GAINS: Kp={Kp} | Ki={Ki} | Kd={Kd}")
            except:
                print("error printing")
        except ValueError:
            print("Error: Invalid input. Please enter numbers only.")
        except Exception as e:
            pass # Failsafe to prevent the background thread from crashing

# Launch the background listener
listener_thread = threading.Thread(target=dynamic_pid_tuner, daemon=True)
listener_thread.start()

last_sim_time = data.time

# Launch the blazing fast native Mac viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # 1. RESET DETECTION
        # If the current simulation time is less than the last recorded time,
        # the user pressed the Reset button or hit Backspace in the viewer UI.
        if data.time < last_sim_time:
            error_sum = [0.0, 0.0, 0.0]  # Clear the integral memory instantly
        # Update our time tracker for the next frame iteration
        last_sim_time = data.time

        # --- Read Sensors (Orientation and Velocity) ---
        # data.qpos contains [X, Y, Z, Quat_W, Quat_X, Quat_Y, Quat_Z]
        # data.qvel contains [Vel_X, Vel_Y, Vel_Z, Roll_rate, Pitch_rate, Yaw_rate]
        pos = data.qpos[0:3]
        quat = data.qpos[3:7]
        vel = data.qvel[0:3]
        angular_vel = data.qvel[3:6]

        w = quat[0]
        x = quat[1]
        y = quat[2]
        z = quat[3]

        #Converting quaternions to roll pitch yaw
        # Calculate Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        drone_angle[0] = math.atan2(sinr_cosp, cosr_cosp)

        # Calculate Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        # Use math.copysign to prevent NaN errors from floating point inaccuracies
        if abs(sinp) >= 1:
            drone_angle[1] = math.copysign(math.pi / 2, sinp)
        else:
            drone_angle[1] = math.asin(sinp)

        # Calculate Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        drone_angle[2] = math.atan2(siny_cosp, cosy_cosp)
        
        # --- Flight Controller (Thrust Math) ---
        for i in range(3):
            raw_error = target_angle[i] - drone_angle[i]
            # Wrap it to the shortest path [-pi, pi]
            error = (raw_error + math.pi) % (2 * math.pi) - math.pi
            error_sum[i] = error_sum[i]*0.95 + error
            pid_torque[i] = Kp[i] * error - Kd[i] * angular_vel[i] + Ki[i] * error_sum[i] * 0.04 # multiplied by 0.04 the time passed between receiving sensor input and calculating output

        thrust_fr = throttle - pid_torque[1] - pid_torque[0] - pid_torque[2]
        thrust_fl = throttle - pid_torque[1] + pid_torque[0] + pid_torque[2]
        thrust_bl = throttle + pid_torque[1] + pid_torque[0] - pid_torque[2]
        thrust_br = throttle + pid_torque[1] - pid_torque[0] + pid_torque[2]
        # thrust_fr = thrust_fl = thrust_bl = thrust_br = throttle


        # Assume we have: thrust_fr, thrust_fl, thrust_br, thrust_bl
        motor_forces = {0: thrust_fr, 1: thrust_fl, 2: thrust_bl, 3: thrust_br}
        
        # GYROSCOPIC PRECESSION CALCULATION
        # drone's propeller is simplified to abstract point capable of producing linear force, so the physics engine does not account for gyroscopic precession.
        total_gyro_torque = np.array([0.0, 0.0, 0.0])
        
        for motor_index, thrust in motor_forces.items():
            # Prevent negative thrust math errors
            thrust = max(thrust, 0)
            thrust = min(thrust, 4 * throttle)
            
            # Estimate the motor's rotational speed (Omega) in rad/s
            # Since Thrust = k * Omega^2, then Omega = sqrt(Thrust / k)
            omega_motor = np.sqrt(thrust / k_thrust)
            
            # Calculate Angular Momentum vector (L) for this specific prop
            # The prop spins around the local Z-axis [0, 0, 1]
            spin_dir = MOTOR_SPIN[motor_index]
            L_prop = np.array([0, 0, J_r * omega_motor * spin_dir])
            
            # THE CROSS PRODUCT: Torque = body_rotation x L_prop
            gyro_torque = - np.cross(angular_vel, L_prop)
            
            # Add this motor's precession to the drone's total precession
            total_gyro_torque += gyro_torque
            
            # Apply linear thrust on drone
            data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_{motor_index + 1}")] = float(thrust)

        # Apply gyroscopic torque
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "torque_roll")] = total_gyro_torque.tolist()[0]
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "torque_pitch")] = total_gyro_torque.tolist()[1]
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "torque_yaw")] = total_gyro_torque.tolist()[2]

        # Tick the simulation forward by 0.004 seconds
        mujoco.mj_step(model, data)

        # --- HOLOGRAM ---
        # 1. Reset the user geometry count (clears the previous frame's text)
        viewer.user_scn.ngeom = 0
        
        # 2. Position of label
        label_pos_vel = np.array([0, 0, 0.5])
        label_pos_alt = np.array([0, 0, 0])
        
        # 3. Create a blank geometry explicitly for hosting text
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[0],
            type=mujoco.mjtGeom.mjGEOM_NONE, # No physical shape
            size=[0, 0, 0],
            pos=label_pos_vel,
            mat=np.eye(3).flatten(),
            rgba=np.array([0, 1, 0, 1])      # Green text [R, G, B, Alpha]
        )
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[1],
            type=mujoco.mjtGeom.mjGEOM_NONE, 
            size=[0, 0, 0],
            pos=label_pos_alt,
            mat=np.eye(3).flatten(),
            rgba=np.array([0, 1, 0, 1])      
        )
        
        # 4. Inject your dynamic telemetry string
        alt = data.body("drone").xpos[2]
        vel = np.linalg.norm(data.qvel[0:3]) # Calculate absolute speed
        viewer.user_scn.geoms[0].label = f"AngularVel: {[round(drone_angle[x],5) for x in range(3)]} rad/s"
        viewer.user_scn.geoms[1].label = f"Altitude: {round(alt,3)}m, Velocity: {round(vel,3)}m/s"
        
        # 5. Tell the viewer we added 1 geometry
        viewer.user_scn.ngeom = 2

        # Update the visual rendering
        if (time.time() - last_render_time) > 1.0/60.0:
          viewer.sync()
          last_render_time = time.time()

        # Ensure the simulation doesn't run at 10,000x speed
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)