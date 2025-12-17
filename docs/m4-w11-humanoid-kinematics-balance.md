# Module 4: Vision-Language-Action (VLA)

## Week 11: Humanoid Kinematics, Bipedal Locomotion, and Balance Control

### 1. Introduction to Humanoid Kinematics

Humanoid kinematics robot ke joints aur links ke darmiyan motion aur position ke relationship ko study karta hai. Yeh do main types mein divide hota hai: Forward Kinematics (FK) aur Inverse Kinematics (IK). FK joints ke angles se end-effector (jaise hath) ki position calculate karta hai, jab ke IK end-effector ki desired position se required joint angles ko calculate karta hai. Humanoid robots mein, complex structure ki wajah se kinematics buhat challenging ho jata hai.

**Technical Content:**
- **Forward Kinematics (FK)**: Denavit-Hartenberg (DH) parameters, transformation matrices.
- **Inverse Kinematics (IK)**: Analytical vs. Numerical (iterative) solutions, Jacobian matrix, redundancy resolution.
- Humanoid-specific challenges: High degrees of freedom (DoF), singularity avoidance.

**Code Example (Python - Conceptual Forward Kinematics for a 2-DOF Arm):**
```python
import numpy as np

def forward_kinematics_2dof(l1, l2, theta1, theta2):
    """
    Calculates the end-effector position for a simple 2-DOF planar arm.
    l1, l2: lengths of the two links
    theta1, theta2: joint angles in radians (relative to previous link)
    Returns (x, y) coordinates of the end-effector.
    """
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

# Example usage:
# length1 = 1.0
# length2 = 1.0
# angle1 = np.pi / 4 # 45 degrees
# angle2 = np.pi / 2 # 90 degrees relative to link 1
# end_effector_pos = forward_kinematics_2dof(length1, length2, angle1, angle2)
# print(f"End-effector position: {end_effector_pos}")
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A simplified 2-DOF robot arm showing link lengths and joint angles for FK. Another image showing an end-effector and target for IK.]
```

### 2. Bipedal Locomotion

Bipedal locomotion (do paon par chalna) humanoid robots ke liye ek complex task hai kyunki ismein continuous balance aur stability maintain karna hota hai jab ke robot movement kar raha ho. Ismein gait generation, foot placement, aur dynamic balance control shamil hain.

**Technical Content:**
- **Gait Generation**: Walking patterns, stride length, step frequency.
- **Zero Moment Point (ZMP)**: Stability criterion for bipedal robots.
- **Center of Mass (CoM)**: CoM trajectory planning for stable walking.
- Whole-body control: Multiple joints ko coordinate karna locomotion ke liye.

**Code Example (Python - Conceptual ZMP Calculation for a Simple Biped):**
```python
import numpy as np

def calculate_zmp_simple(masses, positions, accelerations, gravity=[0, 0, -9.81]):
    """
    Conceptual calculation of ZMP for a simple multi-link robot.
    This is a highly simplified model. Real ZMP calculation is much more complex.

    masses: list of masses for each link/segment
    positions: list of (x, y, z) positions for each link's CoM
    accelerations: list of (ax, ay, az) accelerations for each link's CoM
    gravity: gravity vector
    Returns (zmp_x, zmp_y).
    """
    total_mass = sum(masses)
    force_x_sum = 0.0
    force_y_sum = 0.0
    moment_x_sum = 0.0
    moment_y_sum = 0.0

    for m, pos, acc in zip(masses, positions, accelerations):
        # Forces due to gravity and acceleration
        fx = m * acc[0]
        fy = m * acc[1]
        fz = m * (acc[2] - gravity[2]) # Accel in z, minus gravity

        force_x_sum += fx
        force_y_sum += fy

        # Moments about X and Y axes (simplified)
        moment_x_sum += pos[1] * fz - pos[2] * fy
        moment_y_sum += pos[2] * fx - pos[0] * fz

    if force_y_sum == 0 or force_x_sum == 0: # Avoid division by zero
        # In a real scenario, handle this with proper foot contact detection
        print("Warning: Forces are zero, ZMP calculation might be ill-defined.")
        return 0.0, 0.0

    # Simplified ZMP calculation assuming a planar ground
    zmp_x = -moment_y_sum / force_z_sum if force_z_sum != 0 else 0.0
    zmp_y = moment_x_sum / force_z_sum if force_z_sum != 0 else 0.0

    return zmp_x, zmp_y

# Example usage:
# link_masses = [5.0, 2.0, 2.0] # Torso, left leg, right leg
# link_positions = [
#     np.array([0.0, 0.0, 0.8]),  # Torso CoM
#     np.array([0.1, 0.0, 0.4]),  # Left leg CoM
#     np.array([-0.1, 0.0, 0.4])   # Right leg CoM
# ]
# link_accelerations = [
#     np.array([0.0, 0.0, 0.0]),
#     np.array([0.0, 0.0, 0.0]),
#     np.array([0.0, 0.0, 0.0])
# ]
# zmp_point = calculate_zmp_simple(link_masses, link_positions, link_accelerations)
# print(f"Calculated ZMP: {zmp_point}")
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A humanoid robot in a walking pose, with its ZMP and CoM projected onto the ground plane.]
```

**Hardware Context:**
- **Jetson Orin Nano**: Bipedal locomotion ke low-level control loops (e.g., joint position/velocity control, balance reflexes) ko Jetson Orin Nano par efficiently run kiya ja sakta hai, jahan low-latency aur real-time processing critical hai.

### 3. Balance Control Strategies

Humanoid robots ke liye balance control ek continuous aur dynamic process hai. Ismein robot ki stability ko maintain karne ke liye sensor feedback aur control algorithms ka istemal kiya jata hai, khas kar ke external disturbances (e.g., pushes) ya uneven terrain par chalte waqt.

**Technical Content:**
- **Reactive Control**: IMU data aur force/torque sensors (FT sensors) ka istemal karke immediate corrections apply karna.
- **Model Predictive Control (MPC)**: Robot dynamics aur future trajectory ko consider karte hue optimal control actions calculate karna.
- **Whole-body Balance Control**: Body ke sabhi joints aur contact forces ko use karke stability achieve karna.
- Fall detection aur recovery strategies.

**Code Example (Python - Conceptual PID Controller for Balance):**
```python
class PIDController:
    def __init__(self, kp, ki, kd, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits # (min, max)

        self._integral = 0
        self._last_error = 0
        self._last_time = None

    def update(self, setpoint, measurement, current_time):
        if self._last_time is None:
            self._last_time = current_time
            return 0.0

        dt = current_time - self._last_time
        if dt == 0:
            return self._last_output # Avoid division by zero

        error = setpoint - measurement

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self._integral += error * dt
        i_term = self.ki * self._integral

        # Derivative term
        d_term = self.kd * (error - self._last_error) / dt

        output = p_term + i_term + d_term

        # Apply output limits
        if self.output_limits:
            output = max(self.output_limits[0], min(self.output_limits[1], output))

        self._last_error = error
        self._last_time = current_time
        self._last_output = output

        return output

# Example usage for balancing a simple inverted pendulum (conceptual)
# target_angle = 0.0 # upright
# current_angle = 10.0 # degrees, initial tilt
# current_angular_velocity = 0.0

# pid_controller = PIDController(kp=0.5, ki=0.01, kd=0.1, output_limits=(-10, 10))
# current_time = 0.0
# control_effort = 0.0

# for _ in range(100):
#     # Simulate some dynamics (very simplified)
#     current_angular_velocity += control_effort * 0.01
#     current_angle += current_angular_velocity * 0.01

#     current_time += 0.01
#     control_effort = pid_controller.update(target_angle, current_angle, current_time)
#     # print(f"Time: {current_time:.2f}, Angle: {current_angle:.2f}, Control: {control_effort:.2f}")

# print("Conceptual PID balance control simulation finished.")
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A control loop diagram for humanoid balance, showing sensor inputs (IMU, FT sensors), control algorithm, and motor outputs.]
```

**Hardware Context:**
- **RTX GPU**: Model Predictive Control (MPC) ya deep reinforcement learning (DRL) par based advanced balance control algorithms ko train karne aur real-time inference ke liye RTX GPUs ka istemal kiya ja sakta hai. Yeh GPUs complex optimization problems ko quickly solve kar sakte hain.
- **Jetson Orin Nano**: IMU aur force/torque sensor data ko process karne aur PID jaise traditional control algorithms ko run karne ke liye Jetson Orin Nano ideal hai. Iski low-latency processing capabilities real-time balance corrections ke liye critical hain.