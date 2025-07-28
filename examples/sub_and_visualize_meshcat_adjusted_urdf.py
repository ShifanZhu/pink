#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Upkie wheeled biped bending its knees."""

import os
import time

import numpy as np
import pinocchio as pin
import qpsolvers

import meshcat_shapes
import pink
from pink import solve_ik
from pink.barriers import PositionBarrier
from pink.tasks import FrameTask, PostureTask
from pink.utils import custom_configuration_vector
from pink.visualization import start_meshcat_visualizer

import lcm

from skeleton_lcm import skeleton_lcm
from scipy.spatial.transform import Rotation as R

link_map = {
    # "PELVIS": "base",
    # "SPINE_NAVEL": "TopLumbar",
    # "SPINE_CHEST": "Chest",
    "NECK": "Neck",
    # "CLAVICLE_LEFT": "jChestLeftShoulder_rotx", #! There seems no proper associated link in URDF
    # "SHOULDER_LEFT": "LeftUpperArm",
    # "ELBOW_LEFT": "LeftForeArm",
    "WRIST_LEFT": "LeftHand",
    # "HAND_LEFT": "LeftHandCOM",
    "HIP_LEFT": "LeftUpperLeg",
    # "KNEE_LEFT": "LeftLowerLeg",
    "ANKLE_LEFT": "LeftFoot",
    # "FOOT_LEFT": "LeftToe",
    # "CLAVICLE_RIGHT": "RightShoulder",
    # "SHOULDER_RIGHT": "RightUpperArm",
    # "ELBOW_RIGHT": "RightForeArm",
    "WRIST_RIGHT": "RightHand",
    # "HAND_RIGHT": "RightHandCOM",
    "HIP_RIGHT": "RightUpperLeg",
    # "KNEE_RIGHT": "RightLowerLeg",
    "ANKLE_RIGHT": "RightFoot",
    # "FOOT_RIGHT": "RightToe",
    # "HEAD": "Head",
}

joints = [
    "PELVIS",
    "SPINE_NAVEL",
    "SPINE_CHEST",
    "NECK",
    "CLAVICLE_LEFT",
    "SHOULDER_LEFT",
    "ELBOW_LEFT",
    "WRIST_LEFT",
    "HAND_LEFT",
    "HANDTIP_LEFT",
    "THUMB_LEFT",
    "CLAVICLE_RIGHT",
    "SHOULDER_RIGHT",
    "ELBOW_RIGHT",
    "WRIST_RIGHT",
    "HAND_RIGHT",
    "HANDTIP_RIGHT",
    "THUMB_RIGHT",
    "HIP_LEFT",
    "KNEE_LEFT",
    "ANKLE_LEFT",
    "FOOT_LEFT",
    "HIP_RIGHT",
    "KNEE_RIGHT",
    "ANKLE_RIGHT",
    "FOOT_RIGHT",
    "HEAD",
    "NOSE",
    "EYE_LEFT",
    "EAR_LEFT",
    "EYE_RIGHT",
    "EAR_RIGHT"
]

position_costs = {
    "PELVIS": 1.0,
    "SPINE_NAVEL": 0.8,
    "SPINE_CHEST": 0.8,
    "NECK": 0.1,
    "HEAD": 0.6,
    "CLAVICLE_LEFT": 0.5,
    "SHOULDER_LEFT": 1.0,
    "ELBOW_LEFT": 0.8,
    "WRIST_LEFT": 0.1,
    "HAND_LEFT": 0.4,
    "HIP_LEFT": 1.0,
    "KNEE_LEFT": 0.8,
    "ANKLE_LEFT": 0.6,
    "FOOT_LEFT": 0.4,
    "CLAVICLE_RIGHT": 0.5,
    "SHOULDER_RIGHT": 1.0,
    "ELBOW_RIGHT": 0.8,
    "WRIST_RIGHT": 0.6,
    "HAND_RIGHT": 0.4,
    "HIP_RIGHT": 1.0,
    "KNEE_RIGHT": 0.8,
    "ANKLE_RIGHT": 0.6,
    "FOOT_RIGHT": 0.4,

    # You can skip or set 0.0 for unused/irrelevant joints
}

def get_skeleton_frame_from_msg(msg):
    # frame = {
    #     joints[idx]: msg.joint_positions[idx] + msg.joint_orientations[idx]
    #     for idx in range(len(joints))
    # }
    frame = {}

    frame["Timestamp"] = msg.id * 33333
    # frame["Timestamp"] = 0

    # Convert all joint positions to local pelvis frame
    pelvis_pos = np.array(msg.joint_positions[0])
    pelvis_ori = msg.joint_orientations[0]  # (w, x, y, z) order 

    if len(pelvis_ori) == 4:
      # reorder to (x, y, z, w)
      quat = [pelvis_ori[1], pelvis_ori[2], pelvis_ori[3], pelvis_ori[0]]
      pelvis_rot = R.from_quat(quat).as_matrix()
    else:
      print("Warning: Invalid pelvis orientation length, using identity rotation")
      pelvis_rot = np.eye(3)

    for idx in range(len(joints)):
      # Subtract pelvis position and rotate into pelvis frame
      joint_pos = (np.array(msg.joint_positions[idx]) - np.array(pelvis_pos)) / 1000.0 # Convert to meters
      joint_pos_local = pelvis_rot.T @ joint_pos # Convert to local pelvis(base) frame
      #! Somehow I need to convert from (y, z, x) to (x, y, z) order
      joint_pos_local_converted = [joint_pos_local[1], joint_pos_local[2], joint_pos_local[0]]

      # Compute joint orientation relative to pelvis
      joint_ori = msg.joint_orientations[idx]
      quat = [joint_ori[1], joint_ori[2], joint_ori[3], joint_ori[0]] if len(joint_ori) == 4 else [0, 0, 0, 1]
      joint_rot = R.from_quat(quat).as_matrix()
      # Relative orientation: joint_rot in pelvis frame
      rel_rot = pelvis_rot.T @ joint_rot
      rel_quat = R.from_matrix(rel_rot).as_quat()  # (x, y, z, w)
      frame[joints[idx]] = joint_pos_local_converted + rel_quat.tolist()

    # print(frame)

    return frame

try:
    from loop_rate_limiters import RateLimiter

except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "Examples use loop rate limiters, "
        "try `[conda|pip] install loop-rate-limiters`"
    ) from exc

try:
    from robot_descriptions.loaders.pinocchio import load_robot_description
    from pinocchio.robot_wrapper import RobotWrapper

except ModuleNotFoundError:
    raise ModuleNotFoundError(
        "Examples need robot_descriptions, "
        "try `[conda|pip] install robot_descriptions`"
    )

if __name__ == "__main__":
    global last_timestamp
    last_timestamp = None

    # robot = load_robot_description(
    #     "skeleton_description", root_joint=pin.JointModelFreeFlyer()
    # )
    
    # Replace this with your actual URDF file path
    urdf_path = "/home/s/repos/human-model-generator/code/models/humanModels/male.urdf"
    
    # Optional: specify mesh directory if URDF uses relative paths
    model_path = os.path.dirname(urdf_path)

    # Load the model using Pinocchio's URDF parser
    robot = RobotWrapper.BuildFromURDF(
        filename=urdf_path,
        package_dirs=[model_path],  # optional for mesh resolution
        root_joint=pin.JointModelFreeFlyer()
    )

    # Initialize visualization
    viz = start_meshcat_visualizer(robot)
    viewer = viz.viewer

    print(viewer)

    for body in link_map.values():
        if (body == "base"):
            meshcat_shapes.frame(viewer[body], opacity=1.0, axis_length=0.3, axis_thickness=0.01)
            meshcat_shapes.frame(viewer[body + "_target"], opacity=0.5, axis_length=0.4, axis_thickness=0.005)
        else:
            meshcat_shapes.frame(viewer[body], opacity=1.0, axis_length=0.2, axis_thickness=0.01)
            meshcat_shapes.frame(viewer[body + "_target"], opacity=0.5, axis_length=0.3, axis_thickness=0.005)


    q_ref = np.zeros(robot.nq)
    # # q_ref[2] = 1.72
    # # q_ref[6] = 1.0

    # q_ref[8] = 1.0
    # q_ref[9] = 0.2
    # q_ref[17] = -1

    tasks = {
        joint: FrameTask(
            link_map[joint],
            position_cost=position_costs[joint],
            orientation_cost=0.2,
        ) for joint in link_map.keys()
    }


    # tasks["posture"] = PostureTask(
    #     cost=1e-3,
    # )

    # tasks["posture"].set_target(
    #     custom_configuration_vector(robot, jRightKnee_rotz=0.01, jLeftKnee_rotz=0.01)
    # )
    
    configuration = pink.Configuration(robot.model, robot.data, q_ref)

    targets = {}

    for task_name, task in tasks.items():
        if type(task) is FrameTask:
            task.set_target_from_configuration(configuration)
            targets[task_name] = task.transform_target_to_world

    # Select QP solver
    solver = qpsolvers.available_solvers[0]

    if "daqp" in qpsolvers.available_solvers:
        solver = "daqp"

    def lcm_skeleton_handler(channel, data):
        global last_timestamp
        msg = skeleton_lcm.decode(data)
        frame = get_skeleton_frame_from_msg(msg)
        # print("Received skeleton frame", frame)
        
        if "Timestamp" not in frame:
            return

        # print(frame)

        timestamp = frame["Timestamp"]

        if last_timestamp is None:
            last_timestamp = timestamp
            return

        dt = (timestamp - last_timestamp) / 1e6
        # print(dt)
        last_timestamp = timestamp

        # print(frame)

        # Update task targets
        for task, target in targets.items():
            # target.translation[1] = (frame[task][0] - frame["PELVIS"][0]) / 1000
            # target.translation[2] = -(frame[task][1] - frame["PELVIS"][1]) / 1000
            # target.translation[0] = -(frame[task][2] - frame["PELVIS"][2]) / 1000
            # target.translation[0] = frame[task][1] # x = y
            # target.translation[1] = frame[task][2] # y = z
            # target.translation[2] = frame[task][0] # z = x
            target.translation[0] = frame[task][0]
            target.translation[1] = frame[task][1]
            target.translation[2] = frame[task][2]
            print("task", task, "target", target.translation, "actual", configuration.get_transform_frame_to_world(link_map[task]).translation)
        
        # Update visualization frames
        for joint, body in link_map.items():
            viewer[body + "_target"].set_transform(targets[joint].np)
            # pass
        
        for body in link_map.values():
            viewer[body].set_transform(
                configuration.get_transform_frame_to_world(body).np
            )
            # pass

        # print(configuration.q)

        # Compute velocity and integrate it into next configuration
        velocity = solve_ik(configuration, tasks.values(), dt, solver=solver, safety_break=False)
        # velocity = solve_ik(configuration, tasks.values(), dt, solver=solver, barriers=barriers)

        configuration.integrate_inplace(velocity, dt)

        # print(configuration.q)

        # exit(1)

        # Visualize result at fixed FPS
        viz.display(configuration.q)

    lc = lcm.LCM()
    lc.subscribe("SKELETON", lcm_skeleton_handler)

    while True:
        lc.handle()
        time.sleep(0.01)

    