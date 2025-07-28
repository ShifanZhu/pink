import lcm
from skeleton_lcm import skeleton_lcm
import time


def read_skeleton_frames_lcm(filename):
    frames = {}

    log = lcm.EventLog(filename, "r")
    
    for i, event in enumerate(log):
        data = skeleton_lcm.decode(event.data)
        print()

        print(len(data.joint_positions), len(data.joint_orientations))

        frames[i] = {
            joints[idx]: data.joint_positions[idx] + data.joint_orientations[idx]
            for idx in range(len(joints))
        }

    for i, frame in frames.items():
        frame["Timestamp"] = i * 0.033

    print(frames)

    return list(frames.values())

if __name__ == "__main__":
    # log = lcm.EventLog("data/T_shape_for_scaling.log", "r")
    # log = lcm.EventLog("data/T_shape_for_scaling2.log", "r")
    # log = lcm.EventLog("data/T_shape_parallel.log", "r")
    # log = lcm.EventLog("data/T_shape.log", "r")
    log = lcm.EventLog("data/upper_body_track.log", "r")
    # log = lcm.EventLog("data/lower_body_track.log", "r")
    lc = lcm.LCM()

    last_message = None
    
    for i, event in enumerate(log):
        msg = skeleton_lcm.decode(event.data)
        msg.id = i
        lc.publish("SKELETON", msg.encode())
        print("Sent frame", i)
        if False and i == 1:
            print("Spinning for a long time to visualize the first frame")
            while True:
                lc.publish("SKELETON", last_message.encode())
                lc.publish("SKELETON", msg.encode())
                time.sleep(0.033)
        else:
            time.sleep(0.033)
            # time.sleep(0.003)

        last_message = msg
