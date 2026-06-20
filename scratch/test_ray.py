import pybullet as p
import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("robot.urdf")

# Find joints
num_joints = p.getNumJoints(robot_id)
left_wheel_joint = -1
for i in range(num_joints):
    name = p.getJointInfo(robot_id, i)[1].decode('utf-8')
    if "left_wheel" in name:
        left_wheel_joint = i

print("Before changeDynamics:")
info = p.getDynamicsInfo(robot_id, left_wheel_joint)
print("Mass:", info[0])
print("Lateral Friction:", info[1])
print("Inertia Diagonal:", info[2])

# Change dynamics
p.changeDynamics(robot_id, left_wheel_joint, lateralFriction=1.5, rollingFriction=0.05)
p.changeDynamics(plane_id, -1, lateralFriction=1.0)

print("\nAfter changeDynamics:")
info = p.getDynamicsInfo(robot_id, left_wheel_joint)
print("Mass:", info[0])
print("Lateral Friction:", info[1])
print("Rolling Friction:", info[7])

p.disconnect()
