print("Starting PyBullet test...")

import time

import pybullet as p #the physics simulator
import pybullet_data #access to robot/object files


def main():
    p.connect(p.GUI) # starts PyBullet with a visible window
    # the other option is p.DIRECT, which runs the simulation without a window, usually used in PPO training.
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81) #sets gravity x,y = 0 , z = -9.81, so objects fall downwards

    p.loadURDF("plane.urdf") #loads the ground plane, flat
    # A .urdf file describes a robot or object: its links, joints, shapes, masses, and collision geometry.
    p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0], useFixedBase=True)
    #loads KUKA robot arm at origin, with base fixed to the ground.

    while True:
        p.stepSimulation()
        time.sleep(1 / 240)
    #simulation loop


if __name__ == "__main__":
    main()