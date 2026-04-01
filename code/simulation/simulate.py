"""
FaceHugger Quadruped Robot - PyBullet Standing Simulation

Loads the FaceHugger URDF and holds a stable standing stance.
Specs from project proposal:
  - 12-DOF: 4 legs x 3 joints (shoulder yaw, hip pitch, knee pitch)
  - Link lengths: L1=80mm, L2=75mm, L3=90mm
  - Servo: DSS-M15S, 72g, 15 kg*cm, 270 deg
  - Default stance: hip ~40deg, knee ~60deg
"""

import pybullet as p
import pybullet_data
import time
import math
import os

URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "facehugger.urdf")

# Stance angles (radians) - from proposal torque widget
HIP_ANGLE = math.radians(40)
KNEE_ANGLE = math.radians(-60)  # negative = bending down
SHOULDER_ANGLE = 0.0  # straight out

# Joint name -> target angle mapping
STANCE = {
    "shoulder": SHOULDER_ANGLE,
    "hip": HIP_ANGLE,
    "knee": KNEE_ANGLE,
}


def main():
    # Connect to GUI
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Configure gravity
    p.setGravity(0, 0, -9.81)

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Load robot - spawn high enough for legs to unfold underneath
    start_height = 0.20  # meters
    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=[0, 0, start_height],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
    )

    # Build joint index map
    joint_map = {}
    num_joints = p.getNumJoints(robot_id)
    print(f"\nFaceHugger loaded: {num_joints} joints")
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode("utf-8")
        joint_type = info[2]
        print(f"  Joint {i}: {name} (type={joint_type})")
        joint_map[name] = i

    # Set stance angles and enable position control
    for joint_name, joint_idx in joint_map.items():
        info = p.getJointInfo(robot_id, joint_idx)
        if info[2] == p.JOINT_FIXED:
            continue

        # Determine target angle from joint name
        if "shoulder" in joint_name:
            target = STANCE["shoulder"]
        elif "hip" in joint_name:
            target = STANCE["hip"]
        elif "knee" in joint_name:
            target = STANCE["knee"]
        else:
            target = 0.0

        # Set initial position
        p.resetJointState(robot_id, joint_idx, target)

        # Enable position control with servo-like force
        max_force = 1.47  # 15 kg*cm ≈ 1.47 N*m
        p.setJointMotorControl2(
            robot_id,
            joint_idx,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=max_force,
            maxVelocity=5.0,
        )

    # Configure camera
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.1],
    )

    # Set simulation parameters
    p.setTimeStep(1.0 / 240.0)
    p.setRealTimeSimulation(0)

    # Enable foot friction
    for joint_name, joint_idx in joint_map.items():
        if "foot" in joint_name:
            p.changeDynamics(robot_id, joint_idx, lateralFriction=1.0, restitution=0.1)

    print("\nSimulation running - press Ctrl+C to exit")
    print("Camera: scroll to zoom, middle-click to pan, right-click to rotate")

    try:
        while p.isConnected(physics_client):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if p.isConnected(physics_client):
            p.disconnect()


if __name__ == "__main__":
    main()
