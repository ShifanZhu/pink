#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Upkie wheeled biped bending its knees."""

import numpy as np
import pinocchio as pin
import qpsolvers

import lcm

from skeleton_lcm import skeleton_lcm

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

def read_skeleton_frames_lcm(filename):
    frames = {}

    log = lcm.EventLog(filename, "r")
    
    for i, event in enumerate(log):
        data = skeleton_lcm.decode(event.data)

        frames[i] = {
            joints[idx]: data.joint_positions[idx] + data.joint_orientations[idx]
            for idx in range(len(joints))
        }

    for i, frame in frames.items():
        frame["Timestamp"] = i * 0.033

    return list(frames.values())

sums = {
    "RightForeArm": 0,
    "RightUpperArm": 0,
    "RightLowerLeg": 0,
    "RightUpperLeg": 0,
    "RightHand": 0,
    "Neck": 0,
    "Chest": 0,
    "TopLumbar": 0,
    "MidLumbar": 0,
    "BottomLumbar": 0
}

counts = {
    "RightForeArm": 0,
    "RightUpperArm": 0,
    "RightLowerLeg": 0,
    "RightUpperLeg": 0,
    "RightHand": 0,
    "Neck": 0,
    "Chest": 0,
    "TopLumbar": 0,
    "MidLumbar": 0,
    "BottomLumbar": 0
}

if __name__ == "__main__":
    frames = read_skeleton_frames_lcm("data/upper_body_track.log")
    frame = frames[0]
    H = 1.8
    
    # print("RightForeArm", np.linalg.norm(np.array(frame["ELBOW_RIGHT"])[:3] - np.array(frame["WRIST_RIGHT"])[:3]) / H / 1000)
    # print("RightUpperArm", np.linalg.norm(np.array(frame["SHOULDER_RIGHT"])[:3] - np.array(frame["ELBOW_RIGHT"])[:3]) / H / 1000)
    # print("RightLowerLeg", np.linalg.norm(np.array(frame["KNEE_RIGHT"])[:3] - np.array(frame["ANKLE_RIGHT"])[:3]) / H / 1000)
    # print("RightUpperLeg", np.linalg.norm(np.array(frame["HIP_RIGHT"])[:3] - np.array(frame["KNEE_RIGHT"])[:3]) / H / 1000)
    # print("RightHand", np.linalg.norm(np.array(frame["HAND_RIGHT"])[:3] - np.array(frame["HANDTIP_RIGHT"])[:3]) / H / 1000)

    for frame in frames[:100]:
        sums["RightForeArm"] += np.linalg.norm(np.array(frame["ELBOW_RIGHT"])[:3] - np.array(frame["WRIST_RIGHT"])[:3]) / H / 1000
        sums["RightUpperArm"] += np.linalg.norm(np.array(frame["SHOULDER_RIGHT"])[:3] - np.array(frame["ELBOW_RIGHT"])[:3]) / H / 1000
        sums["RightLowerLeg"] += np.linalg.norm(np.array(frame["KNEE_RIGHT"])[:3] - np.array(frame["ANKLE_RIGHT"])[:3]) / H / 1000
        sums["RightUpperLeg"] += np.linalg.norm(np.array(frame["HIP_RIGHT"])[:3] - np.array(frame["KNEE_RIGHT"])[:3]) / H / 1000
        sums["RightHand"] += np.linalg.norm(np.array(frame["HAND_RIGHT"])[:3] - np.array(frame["HANDTIP_RIGHT"])[:3]) / H / 1000
        sums["Neck"] += np.linalg.norm(np.array(frame["NECK"])[:3] - np.array(frame["HEAD"])[:3]) / H / 1000
        sums["Chest"] += np.linalg.norm(np.array(frame["SPINE_CHEST"])[:3] - np.array(frame["SPINE_NAVEL"])[:3]) / H / 1000
        sums["MidLumbar"] += np.linalg.norm(np.array(frame["SPINE_NAVEL"])[:3] - np.array(frame["PELVIS"])[:3]) / H / 1000


        counts["RightForeArm"] += 1
        counts["RightUpperArm"] += 1
        counts["RightLowerLeg"] += 1
        counts["RightUpperLeg"] += 1
        counts["RightHand"] += 1
        counts["Neck"] += 1
        counts["Chest"] += 1
        counts["MidLumbar"] += 1

    

    print("RightForeArm", sums["RightForeArm"] / counts["RightForeArm"])
    print("RightUpperArm", sums["RightUpperArm"] / counts["RightUpperArm"])
    print("RightLowerLeg", sums["RightLowerLeg"] / counts["RightLowerLeg"])
    print("RightUpperLeg", sums["RightUpperLeg"] / counts["RightUpperLeg"])
    print("RightHand", sums["RightHand"] / counts["RightHand"])
    print("Neck", sums["Neck"] / counts["Neck"])
    print("Chest", sums["Chest"] / counts["Chest"] * 3)
    print("MidLumbar", sums["MidLumbar"] / counts["MidLumbar"])