"""Look at facehugger.urdf in PyBullet's GUI. That's it.

  python view_urdf.py

Mouse: left-drag to orbit, ctrl-left-drag to pan, scroll to zoom.
Close the window or Ctrl+C to quit.
"""

import math
import os
import time

import pybullet as p
import pybullet_data

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "generated", "facehugger.urdf")
TIMESTEP = 1.0 / 240.0

STANCE_RAD = {
    "shoulder": 0.0,
    "hip": math.radians(-40.0),
    "knee": math.radians(-60.0),
}


def main():
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    p.setTimeStep(TIMESTEP)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF(URDF_PATH, basePosition=[0, 0, 0.10], useFixedBase=True)

    ji = {p.getJointInfo(robot, i)[1].decode(): i
          for i in range(p.getNumJoints(robot))}
    for prefix in ("fl", "br", "fr", "bl"):
        for joint, val in STANCE_RAD.items():
            p.resetJointState(robot, ji[f"{prefix}_{joint}_joint"], val)

    p.resetDebugVisualizerCamera(
        cameraDistance=0.6, cameraYaw=45.0, cameraPitch=-25.0,
        cameraTargetPosition=[0, 0, 0.10],
    )

    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(TIMESTEP)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
