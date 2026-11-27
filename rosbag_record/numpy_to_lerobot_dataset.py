#!/usr/bin/env python3
'''
    Taken straight out of claude. Use at own risk!
'''

"""Convert per-episode npz files (from the ROS 2 bag exporter) to a LeRobotDataset."""
import argparse
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def to_rgb(images, encoding):
    enc = encoding.lower()
    if enc == "rgb8":
        return images
    if enc == "bgr8":
        return images[..., ::-1]
    if enc == "mono8":
        return np.repeat(images[..., None], 3, axis=-1)
    if enc in ("rgba8", "bgra8"):
        rgb = images[..., :3]
        return rgb[..., ::-1] if enc == "bgra8" else rgb
    raise ValueError(f"Unsupported image encoding for training: {encoding}")


def align(ep, fps):
    """Return (state, action, images, n_frames) on a fixed-rate grid."""
    cmd_ts = ep["command_timestamp_ns"]
    st_ts = ep["state_timestamp_ns"]
    img_ts = ep["image_timestamp_ns"] if "images" in ep else None

    obs_starts = [st_ts[0]] + ([img_ts[0]] if img_ts is not None else [])
    obs_ends = [st_ts[-1]] + ([img_ts[-1]] if img_ts is not None else [])
    t0 = max(obs_starts)
    t_end = min(min(obs_ends), cmd_ts[-1])

    n = int((t_end - t0) / 1e9 * fps)
    grid = t0 + (np.arange(n) * (1e9 / fps)).astype(np.int64)

    def latest_before(ts):  # causal: last sample at or before t
        return np.clip(np.searchsorted(ts, grid, side="right") - 1, 0, len(ts) - 1)

    state = ep["states"][latest_before(st_ts)]
    act_idx = np.clip(np.searchsorted(cmd_ts, grid, side="left"), 0, len(cmd_ts) - 1)
    action = ep["commands"][act_idx]

    images = None
    if img_ts is not None:
        images = to_rgb(ep["images"], str(ep["image_encoding"]))[latest_before(img_ts)]
    return state.astype(np.float32), action.astype(np.float32), images, n


def main():
    p = argparse.ArgumentParser()
    p.add_argument("npz_dir", type=Path)
    p.add_argument("--repo-id", required=True)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--task", required=True, help="e.g. 'pick up the cube'")
    p.add_argument("--camera-name", default="front")
    args = p.parse_args()

    files = sorted(args.npz_dir.glob("*.npz"))
    first = np.load(files[0])
    if "states" not in first:
        raise SystemExit("npz has no 'states'. Re-export with --state-topic.")

    state_dim = first["states"].shape[1]
    action_dim = first["commands"].shape[1]
    features = {
        "observation.state": {"dtype": "float32", "shape": (state_dim,),
                              "names": [f"s{i}" for i in range(state_dim)]},
        "action": {"dtype": "float32", "shape": (action_dim,),
                   "names": [f"a{i}" for i in range(action_dim)]},
    }
    has_images = "images" in first
    if has_images:
        h, w = first["images"].shape[1:3]
        features[f"observation.images.{args.camera_name}"] = {
            "dtype": "video", "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id, fps=args.fps, features=features, robot_type="my_robot",
    )

    for f in files:
        ep = dict(np.load(f))  # load every array ONCE
        state, action, images, n = align(ep, args.fps)
        if n < 2:
            print(f"skipping {f.name}: too short"); continue
        for t in range(n):
            frame = {
                "observation.state": state[t],
                "action": action[t],
                "task": args.task,
            }
            if images is not None:
                frame[f"observation.images.{args.camera_name}"] = np.ascontiguousarray(images[t])
            dataset.add_frame(frame)
        dataset.save_episode()
        print(f"{f.name}: {n} frames")

    # Some LeRobot versions require this to flush video/parquet writers:
    if hasattr(dataset, "finalize"):
        dataset.finalize()


if __name__ == "__main__":
    main()