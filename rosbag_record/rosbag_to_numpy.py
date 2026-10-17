#!/usr/bin/env python3
"""
Export one Float32/Float64MultiArray topic from a ROS 2 bag to NumPy.
Example:
    python rosbag_to_numpy.py ~/arm_demos/pickup_001 \
        --output ~/arm_demos/pickup_001_commands.npz

Export commands and states together, preserving their separate timestamps:
    python rosbag_to_numpy.py test.db3 --state-topic /arm_curr_angles \
        --output episode_001.npz
"""

import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


SUPPORTED_TYPES = {
    "std_msgs/msg/Float32MultiArray",
    "std_msgs/msg/Float64MultiArray",
}


def read_topic(bag_path, topic):
    """Return fixed-width numeric rows and their original recording timestamps."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            available = ", ".join(sorted(reader.topics)) or "(none)"
            raise ValueError(f"Topic {topic!r} is absent. Available topics: {available}")

        msgtypes = {c.msgtype for c in connections}
        if len(msgtypes) != 1 or not msgtypes <= SUPPORTED_TYPES:
            raise ValueError(
                "Expected one Float32MultiArray or Float64MultiArray type; "
                f"found {sorted(msgtypes)}. Images and JointState need different extraction."
            )

        rows = []
        timestamps = []
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            row = np.asarray(message.data).copy()
            if rows and row.shape != rows[0].shape:
                raise ValueError("Channel count changes between messages; cannot stack rows.")
            if not np.isfinite(row).all():
                raise ValueError(f"Non-finite value at recording timestamp {timestamp}.")
            rows.append(row)
            timestamps.append(timestamp)

    if not rows:
        raise ValueError(f"Topic {topic!r} has no recorded messages.")
    if rows[0].size == 0:
        raise ValueError(f"Topic {topic!r} contains empty data arrays.")

    timestamp_ns = np.asarray(timestamps, dtype=np.int64)
    return {
        "values": np.stack(rows),
        "timestamp_ns": timestamp_ns,
        "time_s": (timestamp_ns - timestamp_ns[0]) / 1_000_000_000.0,
        "topic": np.asarray(topic),
        "msgtype": np.asarray(next(iter(msgtypes))),
    }


def read_episode(bag_path, command_topic, state_topic):
    """Bundle two streams with a common time origin, without resampling them."""
    commands = read_topic(bag_path, command_topic)
    states = read_topic(bag_path, state_topic)
    origin = min(int(commands["timestamp_ns"][0]), int(states["timestamp_ns"][0]))
    return {
        "commands": commands["values"],
        "states": states["values"],
        "command_timestamp_ns": commands["timestamp_ns"],
        "state_timestamp_ns": states["timestamp_ns"],
        "command_time_s": (commands["timestamp_ns"] - origin) / 1_000_000_000.0,
        "state_time_s": (states["timestamp_ns"] - origin) / 1_000_000_000.0,
        "time_origin_ns": np.asarray(origin, dtype=np.int64),
        "command_topic": commands["topic"],
        "state_topic": states["topic"],
        "command_msgtype": commands["msgtype"],
        "state_msgtype": states["msgtype"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory or .db3 file")
    parser.add_argument("--topic", default="/safe_arm_target_joints")
    parser.add_argument(
        "--state-topic",
        help="Also export this numeric state topic alongside --topic in one archive",
    )
    parser.add_argument("--output", required=True, type=Path, help="New .npz output file")
    args = parser.parse_args()
    output = args.output.expanduser()
    if output.suffix != ".npz":
        parser.error("--output must end in .npz")
    if output.exists():
        parser.error(f"Output already exists: {output}. Choose a new filename.")

    try:
        if args.state_topic:
            arrays = read_episode(args.bag.expanduser(), args.topic, args.state_topic)
        else:
            arrays = read_topic(args.bag.expanduser(), args.topic)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("xb") as stream:
            np.savez_compressed(stream, **arrays)
    except Exception as error:
        parser.exit(1, f"Export failed: {error}\n")

    if args.state_topic:
        print(f"Saved commands {arrays['commands'].shape} and states {arrays['states'].shape} to {output}")
        print("Streams retain separate timestamps; time_s arrays share the same origin.")
    else:
        print(f"Saved {arrays['values'].shape} values to {output}")
    print("Rows = messages; columns = channels. Units and ordering are unchanged.")
    print("timestamp_ns contains bag recording times, not sensor acquisition times.")


if __name__ == "__main__":
    main()
