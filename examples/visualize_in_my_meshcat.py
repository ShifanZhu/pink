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


if __name__ == "__main__":
    robot = load_robot_description(
        "skeleton_description", root_joint=pin.JointModelFreeFlyer()
    )

    # Initialize visualization
    viz = start_meshcat_visualizer(robot)
    viewer = viz.viewer
    meshcat_shapes.frame(viewer["left_contact_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["right_contact_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["left_contact"], opacity=1.0)
    meshcat_shapes.frame(viewer["right_contact"], opacity=1.0)

    q_ref = np.zeros(robot.nq)
    # q_ref[2] = 1.72
    # q_ref[6] = 1.0
    q_ref[8] = 1.0
    q_ref[9] = 0.2
    q_ref[17] = -1

    tasks = {
        "base": FrameTask(
            "base",
            position_cost=1.0,  # [cost] / [m]
            orientation_cost=1.0,  # [cost] / [rad]
        ),
        "left_contact": FrameTask(
            "left_contact",
            position_cost=[1, 1, 1],  # [cost] / [m]
            orientation_cost=0.0,  # [cost] / [rad]
        ),
        "right_contact": FrameTask(
            "right_contact",
            position_cost=[1, 1, 1],  # [cost] / [m]
            orientation_cost=0.0,  # [cost] / [rad]
        ),
        "posture": PostureTask(
            cost=1e-3,  # [cost] / [rad]
        ),
    }

    tasks["posture"].set_target(
        custom_configuration_vector(robot, jRightKnee_rotz=0.01, jLeftKnee_rotz=0.01)
        # custom_configuration_vector(robot)
    )

    left_foot_barrier = PositionBarrier(
        "left_contact",
        indices=[2],
        p_min=np.array([-0.9]),
        gain=np.array([100.0]),
        safe_displacement_gain=1.0,
    )
    right_foot_barrier = PositionBarrier(
        "right_contact",
        indices=[2],
        p_min=np.array([-0.9]),
        gain=np.array([100.0]),
        safe_displacement_gain=1.0,
    )
    barriers = [left_foot_barrier, right_foot_barrier]

    # configuration = pink.Configuration(robot.model, robot.data, robot.q0)
    configuration = pink.Configuration(robot.model, robot.data, q_ref)
    for body, task in tasks.items():
        if type(task) is FrameTask:
            task.set_target_from_configuration(configuration)
    # viz.display(configuration.q)

    # tasks["posture"].set_target_from_configuration(configuration)

    left_contact_target = tasks["left_contact"].transform_target_to_world
    right_contact_target = tasks["right_contact"].transform_target_to_world

    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "daqp" in qpsolvers.available_solvers:
        solver = "daqp"


    rate = RateLimiter(frequency=200.0, warn=False)
    dt = rate.period
    t = 0.0  # [s]
    while True:
        # Update task targets
        left_contact_target.translation[2] += (0.1 * np.sin(t) * dt)
        right_contact_target.translation[2] += (0.1 * np.sin(t) * dt)
        
        # Update visualization frames
        viewer["left_contact_target"].set_transform(left_contact_target.np)
        viewer["right_contact_target"].set_transform(right_contact_target.np)
        # viewer["base_target"].set_transform(base_target.np)
        for body in ["left_contact", "right_contact"]:
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
        t += dt