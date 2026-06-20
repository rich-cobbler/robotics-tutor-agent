import os
import time
import math
import argparse
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
from PIL import Image

# 1. Create the Robot URDF dynamically
def create_robot_urdf():
    urdf_content = """<?xml version="1.0"?>
<robot name="diff_drive_robot">
  <!-- Chassis base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.1 0.4 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.12"/>
    </inertial>
  </link>

  <!-- Left wheel link -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="dark_grey">
        <color rgba="0.2 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.1 0.19 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right wheel link -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="dark_grey"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.1 -0.19 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Caster wheel (passive front wheel for stability) -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="continuous">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.18 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Ultrasonic sensor visual representation (Yellow cylinder pointing forward) -->
  <link name="sensor_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 1.5708 0"/>
      <geometry>
        <cylinder length="0.08" radius="0.02"/>
      </geometry>
      <material name="yellow">
        <color rgba="0.9 0.9 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <origin xyz="0.25 0 0.05" rpy="0 0 0"/>
  </joint>
</robot>
"""
    with open("robot.urdf", "w") as f:
        f.write(urdf_content)
    print("robot.urdf created.")

# 2. Run Simulation Function
def run_simulation(gui_mode=False, make_gif=True, pid_params=None, obstacle_x=2.5, label="Simulation", dynamic_obstacle=False, avoid_obstacle=False):
    # Connect to PyBullet
    connection_mode = p.GUI if gui_mode else p.DIRECT
    physics_client = p.connect(connection_mode)
    p.setGravity(0, 0, -9.81)
    
    # Load PyBullet default plane and search path
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")
    
    # Load robot model
    robot_id = p.loadURDF("robot.urdf", basePosition=[0.0, 0.0, 0.15], baseOrientation=[0, 0, 0, 1], flags=p.URDF_MAINTAIN_LINK_ORDER)
    
    # Obstacle configuration
    obstacle_radius = 0.2
    obstacle_height = 0.6
    
    # If dynamic, start obstacle further away
    start_obs_x = 4.0 if dynamic_obstacle else obstacle_x
    obstacle_pos = [start_obs_x, 0.0, obstacle_height / 2.0]
    
    col_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=obstacle_radius, height=obstacle_height)
    vis_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=obstacle_radius, length=obstacle_height, rgbaColor=[0.9, 0.1, 0.1, 1.0])
    
    obstacle_id = p.createMultiBody(
        baseMass=0,  # static
        baseCollisionShapeIndex=col_shape,
        baseVisualShapeIndex=vis_shape,
        basePosition=obstacle_pos
    )
    
    # Joint indexes for wheels and caster
    num_joints = p.getNumJoints(robot_id)
    left_wheel_joint = -1
    right_wheel_joint = -1
    caster_joint = -1
    
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode('utf-8')
        if "left_wheel" in name:
            left_wheel_joint = i
        elif "right_wheel" in name:
            right_wheel_joint = i
        elif "caster" in name:
            caster_joint = i
            
    # Set lateral friction dynamically
    p.changeDynamics(robot_id, left_wheel_joint, lateralFriction=1.5)
    p.changeDynamics(robot_id, right_wheel_joint, lateralFriction=1.5)
    if caster_joint != -1:
        p.changeDynamics(robot_id, caster_joint, lateralFriction=0.0)
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    
    # Disable default motor parameters for velocity control
    p.setJointMotorControl2(robot_id, left_wheel_joint, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(robot_id, right_wheel_joint, p.VELOCITY_CONTROL, force=0)
    if caster_joint != -1:
        p.setJointMotorControl2(robot_id, caster_joint, p.VELOCITY_CONTROL, force=0)
    
    # Simulation settings
    time_step = 1.0 / 240.0
    p.setTimeStep(time_step)
    
    # Run variables (extended for avoidance trajectory to complete)
    duration = 15.0 if avoid_obstacle else (12.0 if dynamic_obstacle else 10.0)
    num_steps = int(duration / time_step)
    
    # Data recording lists
    time_history = []
    distance_history = []
    speed_history = []
    obstacle_speed_history = []
    
    # 2D Path Tracking for avoidance visualization
    x_history = []
    y_history = []
    omega_history = []  # angular velocity
    
    frames = []
    frame_interval = 12  # Render frame every 12 simulation steps
    
    # Default Target Values
    target_speed_rad = 3.0  # rad/s wheel rotation velocity (approx 0.3 m/s)
    wheel_radius = 0.1
    wheel_base = 0.38  # distance between wheels in URDF (0.19 * 2)
    stop_distance_threshold = 1.0  # meter
    
    # PID gains for distance holding (static/dynamic stopping)
    if pid_params is None:
        Kp = 4.5
        Ki = 0.15
        Kd = 0.25
    else:
        Kp = pid_params.get("Kp", 4.5)
        Ki = pid_params.get("Ki", 0.15)
        Kd = pid_params.get("Kd", 0.25)
        
    # PID control state variables
    prev_error = 0.0
    pid_i = 0.0
    braking_zone = 2.0  
    
    # Sensor specs
    sensor_max_range = 5.0  # meters
    ray_angles = [-15.0, -7.5, 0.0, 7.5, 15.0]
    
    stopped_detected = False
    stop_time = -1.0
    final_distance = -1.0
    low_speed_counter = 0
    
    # Dynamic obstacle parameters
    obs_speed = 0.15  
    obs_stop_x = 1.6  
    
    # Path planning/avoidance parameters (Artificial Potential Field)
    goal_pos = [5.0, 0.0]  # Goal coordinate
    K_att = 1.2           # Attraction gain
    K_rep = 0.28          # Repulsion gain
    d_influence = 1.8     # Distance of obstacle influence
    
    # PD steering gains for angle tracking
    Kp_steer = 6.0
    Kd_steer = 0.5
    prev_heading_error = 0.0
    
    print(f"\n--- Running {label} Simulation (Kp={Kp}, Ki={Ki}, Kd={Kd}, Avoid={avoid_obstacle}) ---")
    for step in range(num_steps):
        current_time = step * time_step
        
        # 1. Update Dynamic Obstacle position
        current_obs_speed = 0.0
        if dynamic_obstacle:
            obs_x = 4.0 - obs_speed * current_time
            if obs_x < obs_stop_x:
                obs_x = obs_stop_x
                current_obs_speed = 0.0
            else:
                current_obs_speed = -obs_speed
            p.resetBasePositionAndOrientation(obstacle_id, [obs_x, 0.0, obstacle_height / 2.0], [0, 0, 0, 1])
            
        obstacle_speed_history.append(current_obs_speed)
        
        # 2. Get robot position and orientation
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        rot_mat = p.getMatrixFromQuaternion(orn)
        
        r0, r1, r2 = rot_mat[0:3]
        r3, r4, r5 = rot_mat[3:6]
        r6, r7, r8 = rot_mat[6:9]
        
        # Sensor local offset: [0.25, 0.0, 0.05] (front bumper of the chassis)
        sensor_offset = [0.25, 0.0, 0.05]
        sensor_pos = [
            pos[0] + r0 * sensor_offset[0] + r1 * sensor_offset[1] + r2 * sensor_offset[2],
            pos[1] + r3 * sensor_offset[0] + r4 * sensor_offset[1] + r5 * sensor_offset[2],
            pos[2] + r6 * sensor_offset[0] + r7 * sensor_offset[1] + r8 * sensor_offset[2]
        ]
        
        # Calculate multiple rays
        ray_starts = []
        ray_ends = []
        ray_vectors_local = []
        
        for angle_deg in ray_angles:
            angle_rad = math.radians(angle_deg)
            x_local = math.cos(angle_rad)
            y_local = math.sin(angle_rad)
            z_local = 0.0
            ray_vectors_local.append([x_local, y_local])
            
            ray_dir_world = [
                r0 * x_local + r1 * y_local + r2 * z_local,
                r3 * x_local + r4 * y_local + r5 * z_local,
                r6 * x_local + r7 * y_local + r8 * z_local
            ]
            
            ray_end = [
                sensor_pos[0] + ray_dir_world[0] * sensor_max_range,
                sensor_pos[1] + ray_dir_world[1] * sensor_max_range,
                sensor_pos[2] + ray_dir_world[2] * sensor_max_range
            ]
            
            ray_starts.append(sensor_pos)
            ray_ends.append(ray_end)
            
        # Execute ray test batch
        ray_results = p.rayTestBatch(ray_starts, ray_ends)
        
        # Track distances for each ray (needed for potential field directional repulsion)
        ray_distances = []
        min_distance = sensor_max_range
        for i, res in enumerate(ray_results):
            hit_obj_id, hit_link, hit_fraction, hit_pos, hit_norm = res
            if hit_obj_id != -1:
                dist = hit_fraction * sensor_max_range
            else:
                dist = sensor_max_range
            ray_distances.append(dist)
            if dist < min_distance:
                min_distance = dist
                
        distance = min_distance
        
        # Draw sensor rays in GUI mode
        if gui_mode and step % 15 == 0:
            for i, ray_end in enumerate(ray_ends):
                color = [1, 0, 0] if ray_distances[i] <= stop_distance_threshold else [0, 1, 0]
                p.addUserDebugLine(sensor_pos, ray_end, lineColorRGB=color, lifeTime=0.12)
                
        # 3. Get robot velocities
        linear_vel, angular_vel = p.getBaseVelocity(robot_id)
        forward_vector = [r0, r3, r6]
        forward_speed = sum(v * f for v, f in zip(linear_vel, forward_vector))
        
        # 4. Control Logic
        if avoid_obstacle:
            # --- Obstacle Avoidance via Artificial Potential Field (APF) ---
            # A. Calculate Attractive Force (pointing to goal [5.0, 0.0])
            dx_goal = goal_pos[0] - pos[0]
            dy_goal = goal_pos[1] - pos[1]
            dist_to_goal = math.sqrt(dx_goal**2 + dy_goal**2)
            
            if dist_to_goal > 0:
                F_att = [K_att * (dx_goal / dist_to_goal), K_att * (dy_goal / dist_to_goal)]
            else:
                F_att = [0.0, 0.0]
                
            # B. Calculate Repulsive Force (pushing away from detected ray hits)
            F_rep = [0.0, 0.0]
            for i, dist in enumerate(ray_distances):
                if dist < d_influence:
                    # Direction of force is opposite of ray local direction, then rotated to world coordinates
                    # Local repulsive vector: [-cos(theta), -sin(theta)]
                    local_rep_dir = [-ray_vectors_local[i][0], -ray_vectors_local[i][1]]
                    
                    # Convert local repulsive direction to World coordinates
                    world_rep_dir = [
                        r0 * local_rep_dir[0] + r1 * local_rep_dir[1],
                        r3 * local_rep_dir[0] + r4 * local_rep_dir[1]
                    ]
                    
                    # Magnitude of repulsion (standard potential field math)
                    rep_magnitude = K_rep * (1.0 / dist - 1.0 / d_influence) * (1.0 / (dist**2))
                    F_rep[0] += rep_magnitude * world_rep_dir[0]
                    F_rep[1] += rep_magnitude * world_rep_dir[1]
            
            # C. Combine forces to find target heading
            F_total = [F_att[0] + F_rep[0], F_att[1] + F_rep[1]]
            
            # Prevent local minima deadlocks by introducing a small asymmetry bias 
            # if the repulsive force directly opposes the goal attraction
            if abs(F_total[0]) < 0.05 and abs(F_total[1]) < 0.05 and dist_to_goal > 0.3:
                F_total[1] += 0.15  # Turn slightly to the side to break symmetry
                
            # Calculate target angle in world frame
            target_yaw = math.atan2(F_total[1], F_total[0])
            
            # Get current yaw from quaternion
            # Euler angles: yaw is rotation around Z axis
            siny_cosp = 2 * (orn[3] * orn[2] + orn[0] * orn[1])
            cosy_cosp = 1 - 2 * (orn[1] * orn[1] + orn[2] * orn[2])
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Calculate heading error normalized to [-pi, pi]
            heading_error = target_yaw - current_yaw
            while heading_error > math.pi: heading_error -= 2 * math.pi
            while heading_error < -math.pi: heading_error += 2 * math.pi
            
            # D. PD Steering Controller for wheel alignment
            dt = time_step
            d_heading_error = (heading_error - prev_heading_error) / dt if step > 0 else 0.0
            steer_signal = Kp_steer * heading_error + Kd_steer * d_heading_error
            prev_heading_error = heading_error
            
            # E. Speed controller (Cruise speed slows down when turning or near goal)
            max_v = 0.35  # m/s
            v_cmd = max_v * math.cos(heading_error)  # Slow down on sharp angles
            v_cmd = max(0.0, v_cmd)  # Prevent moving backward during search
            
            # Soft stop at goal
            if dist_to_goal < 0.15:
                v_cmd = 0.0
                steer_signal = 0.0
                if not stopped_detected:
                    stopped_detected = True
                    stop_time = current_time
                    final_distance = dist_to_goal
                    print(f"[{label}] Goal reached successfully at {stop_time:.3f}s!")
            elif dist_to_goal < 0.8:
                # Linearly decrease speed close to goal
                v_cmd *= (dist_to_goal / 0.8)
                
            # F. Map velocities to wheel velocity commands
            # v = (v_r + v_l) / 2
            # omega = (v_r - v_l) / W
            # ==> v_l = v - (omega * W) / 2,  v_r = v + (omega * W) / 2
            v_l = v_cmd - (steer_signal * wheel_base) / 2.0
            v_r = v_cmd + (steer_signal * wheel_base) / 2.0
            
            # Convert linear speeds of wheels to rotational speeds (rad/s) and apply joint direction (negative target velocity)
            left_wheel_vel = -v_l / wheel_radius
            right_wheel_vel = -v_r / wheel_radius
            
        else:
            # --- Standard Distance Holding / Brake Control (Static & Dynamic) ---
            error = distance - stop_distance_threshold
            if distance <= braking_zone:
                dt = time_step
                pid_p = error
                pid_i += error * dt
                pid_d = (error - prev_error) / dt if step > 0 else 0.0
                
                control_signal = Kp * pid_p + Ki * pid_i + Kd * pid_d
                
                if dynamic_obstacle:
                    control_signal = max(-target_speed_rad, min(control_signal, target_speed_rad))
                else:
                    control_signal = max(0.0, min(control_signal, target_speed_rad))
                    
                wheel_vel = -control_signal
                prev_error = error
                
                if abs(forward_speed) < 0.005:
                    low_speed_counter += 1
                    if low_speed_counter >= 30 and not stopped_detected:
                        stopped_detected = True
                        stop_time = current_time - (30 * time_step)
                        final_distance = distance
                        print(f"[{label}] Stabilized stop at {stop_time:.3f}s, distance: {final_distance:.3f}m")
                else:
                    low_speed_counter = 0
            else:
                wheel_vel = -target_speed_rad
                prev_error = error
                pid_i = 0.0
                low_speed_counter = 0
                
            left_wheel_vel = wheel_vel
            right_wheel_vel = wheel_vel
            
        # Apply motor control
        p.setJointMotorControl2(robot_id, left_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=left_wheel_vel, force=15.0)
        p.setJointMotorControl2(robot_id, right_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=right_wheel_vel, force=15.0)
        
        # Step simulation
        p.stepSimulation()
        
        # Record data
        time_history.append(current_time)
        distance_history.append(distance)
        speed_history.append(forward_speed)
        x_history.append(pos[0])
        y_history.append(pos[1])
        omega_history.append(angular_vel[2])
        
        # 5. Capture Camera Images for GIF
        if make_gif and step % frame_interval == 0:
            # camera target adjusts based on mode
            if avoid_obstacle:
                cam_target = [2.5, 0.0, 0.1]
                cam_dist = 5.0
                cam_yaw = 0.0    # Top-down angle or high-perspective is best for path visual
                cam_pitch = -45
            else:
                cam_target = [1.75, 0.0, 0.2]
                cam_dist = 4.5
                cam_yaw = 55
                cam_pitch = -20
                
            view_mat = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=cam_target,
                distance=cam_dist,
                yaw=cam_yaw,
                pitch=cam_pitch,
                roll=0,
                upAxisIndex=2
            )
            proj_mat = p.computeProjectionMatrixFOV(
                fov=55,
                aspect=4.0/3.0,
                nearVal=0.1,
                farVal=12.0
            )
            
            w, h, rgb, depth, seg = p.getCameraImage(
                width=320,
                height=240,
                viewMatrix=view_mat,
                projectionMatrix=proj_mat,
                renderer=p.ER_TINY_RENDERER
            )
            
            rgb_array = np.reshape(rgb, (h, w, 4))
            rgb_img = Image.fromarray(rgb_array[:, :, :3].astype(np.uint8))
            frames.append(rgb_img)
            
        if gui_mode:
            time.sleep(time_step)
            
    p.disconnect()
    
    # Save animated GIF
    if make_gif and len(frames) > 0:
        gif_suffix = "_avoid" if avoid_obstacle else ("_dynamic" if dynamic_obstacle else "")
        gif_path = f"simulation{gif_suffix}.gif"
        frames[0].save(
            gif_path,
            save_all=True,
            append_images=frames[1:],
            duration=int(frame_interval * time_step * 1000),
            loop=0
        )
        print(f"[{label}] Simulation GIF saved to {os.path.abspath(gif_path)}")
        
    return {
        "time": np.array(time_history),
        "distance": np.array(distance_history),
        "speed": np.array(speed_history),
        "obs_speed": np.array(obstacle_speed_history) if dynamic_obstacle else None,
        "x": np.array(x_history),
        "y": np.array(y_history),
        "omega": np.array(omega_history),
        "stop_time": stop_time if stopped_detected else -1.0,
        "final_distance": final_distance if stopped_detected else distance
    }

# 3. Plot Comparison of PID Parameters
def plot_pid_comparison(results):
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 1, 1)
    for name, data in results.items():
        plt.plot(data["time"], data["distance"], color=data["color"], linewidth=2.0, label=f'{name} (Kp={data["Kp"]}, Kd={data["Kd"]})')
        if data["stop_time"] > 0:
            plt.axvline(x=data["stop_time"], color=data["color"], linestyle=':', alpha=0.7)
            
    plt.axhline(y=1.0, color='red', linestyle='--', linewidth=1.5, label='Stop Threshold (1.0m)')
    plt.title('PID Response Comparison: Distance to Obstacle', fontsize=12, fontweight='bold')
    plt.ylabel('Distance (meters)', fontsize=10)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend(loc='upper right')
    
    plt.subplot(2, 1, 2)
    for name, data in results.items():
        plt.plot(data["time"], data["speed"], color=data["color"], linewidth=2.0, label=name)
        if data["stop_time"] > 0:
            plt.axvline(x=data["stop_time"], color=data["color"], linestyle=':', alpha=0.7)
            
    plt.title('PID Response Comparison: Robot Speed', fontsize=12, fontweight='bold')
    plt.xlabel('Time (seconds)', fontsize=10)
    plt.ylabel('Speed (m/s)', fontsize=10)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend(loc='upper right')
    
    plt.tight_layout()
    plot_path = "pid_comparison.png"
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"PID comparison plot saved to {os.path.abspath(plot_path)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PyBullet Differential Drive Robot Obstacle Avoidance Simulation")
    parser.add_argument("--gui", action="store_true", help="Run with PyBullet GUI interface")
    parser.add_argument("--no-gif", action="store_true", help="Disable generating simulation GIF")
    parser.add_argument("--compare-pid", action="store_true", help="Run simulation with different PID parameters and generate comparison plots")
    parser.add_argument("--dynamic", action="store_true", help="Run the simulation with a moving dynamic obstacle approaching the robot")
    parser.add_argument("--avoid", action="store_true", help="Run the simulation with Artificial Potential Field obstacle avoidance steering toward goal [5.0, 0.0]")
    args = parser.parse_args()
    
    create_robot_urdf()
    
    if args.compare_pid:
        pid_configs = {
            "Underdamped": {"Kp": 9.0, "Ki": 0.3, "Kd": 0.02, "color": "tab:orange"},
            "Overdamped": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.8, "color": "tab:purple"},
            "Critically Damped": {"Kp": 4.5, "Ki": 0.15, "Kd": 0.25, "color": "tab:green"}
        }
        
        comparison_results = {}
        for name, params in pid_configs.items():
            sim_res = run_simulation(gui_mode=False, make_gif=False, pid_params=params, label=name)
            comparison_results[name] = {
                "time": sim_res["time"],
                "distance": sim_res["distance"],
                "speed": sim_res["speed"],
                "stop_time": sim_res["stop_time"],
                "final_distance": sim_res["final_distance"],
                "Kp": params["Kp"],
                "Kd": params["Kd"],
                "color": params["color"]
            }
            
        plot_pid_comparison(comparison_results)
    elif args.avoid:
        # Run potential-field obstacle avoidance steering toward goal
        # Set static obstacle exactly in the middle of the path at [2.5, 0.0]
        res = run_simulation(gui_mode=args.gui, make_gif=not args.no_gif, label="APF Obstacle Avoidance", obstacle_x=2.5, avoid_obstacle=True)
        
        # Plot 2D path map and velocities
        plt.figure(figsize=(10, 8))
        
        # 1. 2D Path Map Plot
        plt.subplot(2, 1, 1)
        plt.plot(res["x"], res["y"], color='tab:blue', linewidth=2.5, label='Robot Trajectory')
        
        # Plot Obstacle (represented as a circle of radius 0.2)
        obstacle_circle = plt.Circle((2.5, 0.0), 0.2, color='tab:red', alpha=0.6, label='Obstacle (Radius 0.2m)')
        plt.gca().add_patch(obstacle_circle)
        
        # Plot Goal and Start points
        plt.plot(0.0, 0.0, marker='o', color='tab:green', markersize=8, label='Start (0,0)')
        plt.plot(5.0, 0.0, marker='*', color='gold', markersize=12, markeredgecolor='black', label='Goal (5.0, 0)')
        
        plt.title('2D Path Trajectory: APF Obstacle Avoidance', fontsize=12, fontweight='bold')
        plt.xlabel('X (meters)', fontsize=10)
        plt.ylabel('Y (meters)', fontsize=10)
        plt.xlim(-0.5, 5.5)
        plt.ylim(-1.0, 1.0)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.legend(loc='upper left')
        
        # 2. Velocity Profile Plot
        plt.subplot(2, 1, 2)
        plt.plot(res["time"], res["speed"], color='tab:green', linewidth=2, label='Linear Velocity (m/s)')
        plt.plot(res["time"], res["omega"], color='tab:orange', linewidth=1.5, linestyle='--', label='Angular Velocity (rad/s)')
        if res["stop_time"] > 0:
            plt.axvline(x=res["stop_time"], color='black', linestyle=':', label=f'Goal Reached ({res["stop_time"]:.3f}s)')
        plt.title('Velocity Profile: Linear & Angular Velocities', fontsize=12, fontweight='bold')
        plt.xlabel('Time (seconds)', fontsize=10)
        plt.ylabel('Velocity', fontsize=10)
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.legend(loc='upper right')
        
        plt.tight_layout()
        plot_path = "results_avoid.png"
        plt.savefig(plot_path, dpi=150)
        plt.close()
        print(f"Avoidance results plot saved to {os.path.abspath(plot_path)}")
        
    elif args.dynamic:
        # Run dynamic obstacle simulation
        res = run_simulation(gui_mode=args.gui, make_gif=not args.no_gif, label="Dynamic Obstacle PID", dynamic_obstacle=True)
        
        # Save standard dynamic plot
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot(res["time"], res["distance"], color='tab:blue', linewidth=2, label='Detected Distance (m)')
        plt.axhline(y=1.0, color='tab:red', linestyle='--', label='Keep Distance Threshold (1.0m)')
        if res["stop_time"] > 0:
            plt.axvline(x=res["stop_time"], color='black', linestyle=':', label=f'Stabilized ({res["stop_time"]:.3f}s)')
        plt.title('Dynamic Obstacle Interaction: Distance & Speeds over Time')
        plt.ylabel('Distance (meters)')
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.legend()
        
        plt.subplot(2, 1, 2)
        plt.plot(res["time"], res["speed"], color='tab:green', linewidth=2, label='Robot Forward Speed (m/s)')
        plt.plot(res["time"], res["obs_speed"], color='tab:red', linestyle='--', linewidth=1.5, label='Obstacle Speed (m/s)')
        if res["stop_time"] > 0:
            plt.axvline(x=res["stop_time"], color='black', linestyle=':')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Velocity (m/s)')
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.legend()
        
        plt.tight_layout()
        plot_path = "results_dynamic.png"
        plt.savefig(plot_path, dpi=150)
        plt.close()
        print(f"Dynamic results plot saved to {os.path.abspath(plot_path)}")
    else:
        # Run standard static simulation using default (Critically Damped) PID parameters
        res = run_simulation(gui_mode=args.gui, make_gif=not args.no_gif, label="Default PID", dynamic_obstacle=False)
        
        # Save standard plot
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot(res["time"], res["distance"], color='tab:blue', linewidth=2, label='Detected Distance (m)')
        plt.axhline(y=1.0, color='tab:red', linestyle='--', label='Stop Threshold (1.0m)')
        if res["stop_time"] > 0:
            plt.axvline(x=res["stop_time"], color='black', linestyle=':', label=f'Stop Trigger ({res["stop_time"]:.3f}s)')
        plt.title('Obstacle Distance & Robot Speed over Time (Critically Damped PID)')
        plt.ylabel('Distance (meters)')
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.legend()
        
        plt.subplot(2, 1, 2)
        plt.plot(res["time"], res["speed"], color='tab:green', linewidth=2, label='Forward Speed (m/s)')
        if res["stop_time"] > 0:
            plt.axvline(x=res["stop_time"], color='black', linestyle=':')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Speed (m/s)')
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.legend()
        
        plt.tight_layout()
        plot_path = "results.png"
        plt.savefig(plot_path, dpi=150)
        plt.close()
        print(f"Results plot saved to {os.path.abspath(plot_path)}")
