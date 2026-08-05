from __future__ import annotations

import argparse
import math
import time

import pybullet as p
import pybullet_data

CONTROLLED_JOINTS = list(range(7))
END_EFFECTOR_LINK_INDEX = 6
SIMULATION_TIMESTEP = 1 / 240

def calculate_distance(point_a: list[float], point_b: list[float]) -> float:
    """Return the Euclidean distance between two 3D points."""
    dx = point_a[0] - point_b[0]
    dy = point_a[1] - point_b[1]
    dz = point_a[2] - point_b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)

def main() -> None:
    parser = argparse.ArgumentParser(description="Move KUKA iiwa end effector toward a target using inverse kinematics.")
    parser.add_argument("--headless", action="store_true", help="Run without opening the PyBullet GUI.")
    parser.add_argument("--duration", type=float, default=5.0, help="Number of seconds to run the demo.")
    args = parser.parse_args()
    connection_mode = p.DIRECT if args.headless else p.GUI
    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError("Could not connect to the PyBullet physics server.")

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(SIMULATION_TIMESTEP)

        p.loadURDF("plane.urdf")
        robot_id = p.loadURDF(
            "kuka_iiwa/model.urdf",
            basePosition=[0, 0, 0],
            useFixedBase=True,
        )

        print("Loaded KUKA iiwa robot.")
        print(f"Robot id: {robot_id}")
        #===============================================================================================================================================#
        #target_position is the 3D goal point in metres: [x, y, z]
        #createVisualShape(...) creates the red sphere appearance
        #createMultiBody(...) places that sphere in the simulation
        #baseMass=0 makes it static, so it does not fall
        target_position = [0.45, 0.0, 0.75]

        target_visual_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.04,
            rgbaColor=[1.0, 0.0, 0.0, 1.0],
        )

        target_body_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=target_visual_id,
            basePosition=target_position,
        )

        print(f"Target id: {target_body_id}")
        print(f"Target position: {target_position}")
        #================================================================================================================================================#
        #ik_solution = p.calculateInverseKinematics(bodyUniqueId=robot_id,endEffectorLinkIndex=END_EFFECTOR_LINK_INDEX,targetPosition=target_position,)
        #++++ Solver Constraints ++++#
        # lower joint limits
        # upper joint limits
        # joint ranges
        # preferred resting pose
        # number of solver iterations
        # tolerance/residual threshold
        #++++++++++++++++++++++++++++#
        ik_solution = p.calculateInverseKinematics(
            bodyUniqueId=robot_id,
            endEffectorLinkIndex=END_EFFECTOR_LINK_INDEX,
            targetPosition=target_position,
            maxNumIterations=100,
            residualThreshold=1e-5,
        )
        print("IK solution:", [round(value, 3) for value in ik_solution])
        #================================================================================================================================================#
        #start_time records when the demo begins
        # the while loop keeps the simulation alive
        # elapsed_time measures how many seconds have passed
        # break stops the demo after args.duration
        # p.stepSimulation() advances the physics
        # time.sleep(...) slows GUI mode to real-time speed
        start_time = time.perf_counter()

        while p.isConnected():
            elapsed_time = time.perf_counter() - start_time
            if elapsed_time >= args.duration:
                break
            for joint_index, target_angle in zip(CONTROLLED_JOINTS, ik_solution):
                p.setJointMotorControl2(
                    bodyUniqueId=robot_id,
                    jointIndex=joint_index,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_angle,
                    force=300,
                )
            p.stepSimulation()

            if not args.headless:
                time.sleep(SIMULATION_TIMESTEP)
        #================================================================================================================================================#
        final_link_state = p.getLinkState(
        robot_id,
        END_EFFECTOR_LINK_INDEX,
        computeForwardKinematics=True,
        )
        final_end_effector_position = final_link_state[4]
        final_distance = calculate_distance(
            list(final_end_effector_position),
            target_position,
            )

        print(
             "Final end-effector position:",
                [round(value, 3) for value in final_end_effector_position],
        )
        print(f"Final distance to target: {final_distance:.4f} m")

    finally:
        if p.isConnected():
            p.disconnect()

if __name__ == "__main__":
    main()