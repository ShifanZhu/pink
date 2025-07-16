# position is first 3, orientation is last 4

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
                if label == "Timestamp":
                    frames.append(frame)
                    frame = { label: int(parts[0]) }

                else:
                    frame[label] = [ float(p) for p in parts ]

            except:
                pass

    return frames

if __name__ == "__main__":
    frames = read_skeleton_frames("skeleton.txt")
    print(frames)