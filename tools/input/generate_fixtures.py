import copy
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "core/src/test/resources/input/synthetic"


def vec(x=0.0, y=0.0, z=0.0):
    return {"x": x, "y": y, "z": z}


def rotation(axis, angle):
    s = math.sin(angle / 2)
    return dict(zip(("x", "y", "z", "w"), [v * s for v in axis] + [math.cos(angle / 2)]))


def hand(x):
    return {
        "palm": {"position": vec(x, 0.3, -0.55), "rotation": rotation((1, 0, 0), 0)},
        "curls": [0.9] * 4,
        "fingerTips": [vec(x, 0.3, -0.64)] * 2,
    }


def frame(t):
    return {
        "timestampS": t, "sampleTimestampS": t,
        "left": hand(-0.23), "right": hand(0.23),
        "hmd": {"position": vec(0, 0.8, 0), "rotation": rotation((1, 0, 0), 0)},
        "coordinateSpace": "rig",
    }


def make(name):
    frames = []
    for index in range(1, 201):
        f = frame(index * 0.02)
        a = min(1.0, max(0.0, (index - 40) / 80))
        if name.startswith("steer"):
            angle = a * (0.25 if name.endswith("right") else -0.25)
            for side in ("left", "right"):
                x = f[side]["palm"]["position"]["x"]
                f[side]["palm"]["position"] = vec(x * math.cos(angle), 0.3, -0.55 + x * math.sin(angle))
                f[side]["palm"]["rotation"] = rotation((0, 1, 0), -angle)
        if name.startswith("lean"):
            angle = a * (0.25 if name.endswith("right") else -0.25)
            for side in ("left", "right"):
                x = f[side]["palm"]["position"]["x"]
                f[side]["palm"]["position"] = vec(x * math.cos(angle), 0.3 - x * math.sin(angle), -0.55)
                f[side]["palm"]["rotation"] = rotation((0, 0, -1), angle)
        if name == "throttle":
            f["right"]["palm"]["rotation"] = rotation((1, 0, 0), a * 0.45)
            if index >= 160:
                f["right"]["curls"] = [0.0] * 4
        if name == "brake" and index >= 50:
            pull = min(1.0, max(0.0, (index - 85) / 60))
            f["right"]["curls"] = [0.05 + 0.5 * pull] * 2 + [0.9, 0.9]
            f["right"]["fingerTips"] = [vec(0.23, 0.3, -0.64 + 0.05 * pull)] * 2
        if name in ("left-lost", "both-lost") and index >= 80:
            f["left"] = None
        if name == "both-lost" and index >= 80:
            f["right"] = None
        frames.append(f)
    return frames


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    config = json.loads((ROOT / "assets/input-defaults.json").read_text())
    for name in ("neutral", "steer-left", "steer-right", "lean-left", "lean-right", "throttle", "brake", "left-lost", "both-lost"):
        header = {
            "schema": 1, "source": "synthetic", "config": config, "frames": [],
            "calibration": {"snapshot": frame(0), "rig": {"origin": vec(), "yawRad": 0.0}},
        }
        path = OUTPUT / f"{name}.jsonl"
        path.write_text("\n".join(json.dumps(item, separators=(",", ":")) for item in [header] + make(name)) + "\n")
        print(path)


if __name__ == "__main__":
    main()
