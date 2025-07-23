#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Upkie wheeled biped bending its knees."""

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

def read_skeleton_frames(filename):
    frames = []

    with open(filename, "r") as file:
        frame = {}

        for line in file:
            line = line.replace(" ", "")
            line = line.replace("\n", "")
            label, data = line.split(":")
            parts = data.split(",")

            try:
                # Timestamp is always the first entry in a frame
                if label == "Timestamp":
                    if frame != {}:
                        frames.append(frame)

                    frame = { label: int(parts[0]) }

                else:
                    frame[label] = [ float(p) for p in parts ]

            except:
                pass

    return frames

try:
    from loop_rate_limiters import RateLimiter

except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "Examples use loop rate limiters, "
        "try `[conda|pip] install loop-rate-limiters`"
    ) from exc

try:
    from robot_descriptions.loaders.pinocchio import load_robot_description

except ModuleNotFoundError:
    raise ModuleNotFoundError(
        "Examples need robot_descriptions, "
        "try `[conda|pip] install robot_descriptions`"
    )

link_map = {
    "PELVIS": "base",
    "SPINE_NAVEL": "LowerTrunk",
    "SPINE_CHEST": "UpperTrunk",
    "NECK": "Neck",
    "HEAD": "Head",
    "CLAVICLE_LEFT": "LeftShoulder",
    "SHOULDER_LEFT": "LeftUpperArm",
    "ELBOW_LEFT": "LeftForeArm",
    "WRIST_LEFT": "LeftHand",
    "HAND_LEFT": "LeftHandCOM",
    "HIP_LEFT": "LeftUpperLeg",
    "KNEE_LEFT": "LeftLowerLeg",
    "ANKLE_LEFT": "LeftFoot",
    "FOOT_LEFT": "LeftToe",
    "CLAVICLE_RIGHT": "RightShoulder",
    "SHOULDER_RIGHT": "RightUpperArm",
    "ELBOW_RIGHT": "RightForeArm",
    "WRIST_RIGHT": "RightHand",
    "HAND_RIGHT": "RightHandCOM",
    "HIP_RIGHT": "RightUpperLeg",
    "KNEE_RIGHT": "RightLowerLeg",
    "ANKLE_RIGHT": "RightFoot",
    "FOOT_RIGHT": "RightToe",
}

if __name__ == "__main__":
    last_timestamp = None

    robot = load_robot_description(
        "skeleton_description", root_joint=pin.JointModelFreeFlyer()
    )

    # Initialize visualization
    viz = start_meshcat_visualizer(robot)
    viewer = viz.viewer

    print(viewer)

    for body in link_map.values():
        meshcat_shapes.frame(viewer[body], opacity=1.0)
        meshcat_shapes.frame(viewer[body + "_target"], opacity=0.5)

    q_ref = np.zeros(robot.nq)
    # # q_ref[2] = 1.72
    # # q_ref[6] = 1.0

    # q_ref[8] = 1.0
    # q_ref[9] = 0.2
    # q_ref[17] = -1

    tasks = {
        joint: FrameTask(
            link_map[joint],
            position_cost=1.0,
            orientation_cost=0.0,
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

    rate = RateLimiter(frequency=100, warn=False)

    for frame in read_skeleton_frames("data/standingtoT.txt"):
    # for frame in read_skeleton_frames("data/skeleton1.txt"):
        if "Timestamp" not in frame:
            continue

        # print(frame)

        timestamp = frame["Timestamp"]

        if last_timestamp is None:
            last_timestamp = timestamp

        dt = (timestamp - last_timestamp) / 1e6
        last_timestamp = timestamp

        # print(frame)

        # Update task targets
        for task, target in targets.items():
            target.translation[1] = (frame[task][0] - frame["PELVIS"][0]) / 1000
            target.translation[2] = -(frame[task][1] - frame["PELVIS"][1]) / 1000
            target.translation[0] = -(frame[task][2] - frame["PELVIS"][2]) / 1000

            # print(task, target.translation)
        
        # Update visualization frames
        for joint, body in link_map.items():
            viewer[body + "_target"].set_transform(targets[joint].np)
        
        for body in link_map.values():
            viewer[body].set_transform(
                configuration.get_transform_frame_to_world(body).np
            )

        # Compute velocity and integrate it into next configuration
        velocity = solve_ik(configuration, tasks.values(), dt, solver=solver)
        # velocity = solve_ik(configuration, tasks.values(), dt, solver=solver, barriers=barriers)

        configuration.integrate_inplace(velocity, dt)

        # Visualize result at fixed FPS
        viz.display(configuration.q)
        rate.sleep()