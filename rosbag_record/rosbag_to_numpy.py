#!/usr/bin/env python3
"""
Export one Float32/Float64MultiArray topic from a ROS 2 bag to NumPy.
"""

import argparse
import re
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


SUPPORTED_TYPES = {
    "std_msgs/msg/Float32MultiArray",
    "std_msgs/msg/Float64MultiArray",
}

IMAGE_ENCODING_ALIASES = {
    "rgb8": ("u", 8, 3),
    "bgr8": ("u", 8, 3),
    "rgba8": ("u", 8, 4),
    "bgra8": ("u", 8, 4),
    "mono8": ("u", 8, 1),
    "mono16": ("u", 16, 1),
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
                f"found {sorted(msgtypes)}. Use --camera-topic for sensor_msgs/Image."
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


def _image_encoding(encoding):
    encoding_lower = encoding.lower()
    if encoding_lower in IMAGE_ENCODING_ALIASES:
        return IMAGE_ENCODING_ALIASES[encoding_lower]

    match = re.fullmatch(r"(8|16|32|64)([USF])C([1-4])", encoding.upper())
    if match is None:
        raise ValueError(f"Unsupported sensor_msgs/Image encoding {encoding!r}.")
    bits, kind, channels = match.groups()
    return {"U": "u", "S": "i", "F": "f"}[kind], int(bits), int(channels)


def read_image_topic(bag_path, topic):
    """Return decoded image frames and their original recording timestamps."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            available = ", ".join(sorted(reader.topics)) or "(none)"
            raise ValueError(f"Topic {topic!r} is absent. Available topics: {available}")

        msgtypes = {c.msgtype for c in connections}
        expected_type = "sensor_msgs/msg/Image"
        if msgtypes != {expected_type}:
            raise ValueError(
                f"Expected {expected_type} on camera topic {topic!r}; "
                f"found {sorted(msgtypes)}."
            )

        frames = []
        timestamps = []
        image_encoding = None
        image_shape = None
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            encoding = message.encoding
            kind, bits, channels = _image_encoding(encoding)
            dtype = np.dtype(f"{kind}{bits // 8}")
            if bits > 8:
                dtype = dtype.newbyteorder(">" if message.is_bigendian else "<")

            height, width, step = message.height, message.width, message.step
            packed_row_bytes = width * channels * dtype.itemsize
            if height <= 0 or width <= 0 or step < packed_row_bytes:
                raise ValueError(
                    f"Invalid image dimensions or row step at recording timestamp {timestamp}."
                )
            expected_data_size = height * step
            if len(message.data) != expected_data_size:
                raise ValueError(
                    f"Image data size mismatch at recording timestamp {timestamp}: "
                    f"expected {expected_data_size} bytes, got {len(message.data)}."
                )

            rows = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
            pixels = rows[:, :packed_row_bytes].copy().view(dtype)
            frame = pixels.reshape(height, width, channels)
            if channels == 1:
                frame = frame[:, :, 0]
            frame = frame.astype(dtype.name, copy=False)

            if image_encoding is not None and encoding != image_encoding:
                raise ValueError("Image encoding changes between messages; cannot stack frames.")
            if image_shape is not None and frame.shape != image_shape:
                raise ValueError("Image dimensions change between messages; cannot stack frames.")
            image_encoding = encoding
            image_shape = frame.shape
            frames.append(frame)
            timestamps.append(timestamp)

    if not frames:
        raise ValueError(f"Topic {topic!r} has no recorded images.")

    timestamp_ns = np.asarray(timestamps, dtype=np.int64)
    return {
        "images": np.stack(frames),
        "timestamp_ns": timestamp_ns,
        "topic": np.asarray(topic),
        "msgtype": np.asarray(expected_type),
        "encoding": np.asarray(image_encoding),
    }


def read_episode(bag_path, command_topic, state_topic=None, camera_topic=None):
    """Bundle command, optional state, and optional camera streams on one time origin."""
    commands = read_topic(bag_path, command_topic)
    streams = {"command": commands}
    if state_topic:
        streams["state"] = read_topic(bag_path, state_topic)
    if camera_topic:
        streams["camera"] = read_image_topic(bag_path, camera_topic)

    origin = min(int(stream["timestamp_ns"][0]) for stream in streams.values())
    arrays = {
        "commands": commands["values"],
        "command_timestamp_ns": commands["timestamp_ns"],
        "command_time_s": (commands["timestamp_ns"] - origin) / 1_000_000_000.0,
        "time_origin_ns": np.asarray(origin, dtype=np.int64),
        "command_topic": commands["topic"],
        "command_msgtype": commands["msgtype"],
    }
    if state_topic:
        states = streams["state"]
        arrays.update({
            "states": states["values"],
            "state_timestamp_ns": states["timestamp_ns"],
            "state_time_s": (states["timestamp_ns"] - origin) / 1_000_000_000.0,
            "state_topic": states["topic"],
            "state_msgtype": states["msgtype"],
        })
    if camera_topic:
        camera = streams["camera"]
        arrays.update({
            "images": camera["images"],
            "image_timestamp_ns": camera["timestamp_ns"],
            "image_time_s": (camera["timestamp_ns"] - origin) / 1_000_000_000.0,
            "camera_topic": camera["topic"],
            "camera_msgtype": camera["msgtype"],
            "image_encoding": camera["encoding"],
        })
    return arrays


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory or .db3 file")
    parser.add_argument("--topic", default="/safe_arm_target_joints")
    parser.add_argument(
        "--state-topic",
        help="Also export this numeric state topic alongside --topic in one archive",
    )
    parser.add_argument(
        "--camera-topic",
        help="Also export this sensor_msgs/Image topic as an image array",
    )
    parser.add_argument("--output", required=True, type=Path, help="New .npz output file")
    args = parser.parse_args()
    output = args.output.expanduser()
    if output.suffix != ".npz":
        parser.error("--output must end in .npz")
    if output.exists():
        parser.error(f"Output already exists: {output}. Choose a new filename.")

    try:
        if args.state_topic or args.camera_topic:
            arrays = read_episode(
                args.bag.expanduser(), args.topic, args.state_topic, args.camera_topic
            )
        else:
            arrays = read_topic(args.bag.expanduser(), args.topic)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("xb") as stream:
            np.savez_compressed(stream, **arrays)
    except Exception as error:
        parser.exit(1, f"Export failed: {error}\n")

    if "values" in arrays:
        print(f"Saved {arrays['values'].shape} values to {output}")
        print("Rows = messages; columns = channels. Units and ordering are unchanged.")
    else:
        print(f"Saved commands {arrays['commands'].shape} to {output}")
        if "states" in arrays:
            print(f"Saved states {arrays['states'].shape}")
        if "images" in arrays:
            print(
                f"Saved images {arrays['images'].shape} "
                f"({arrays['image_encoding'].item()} encoding)"
            )
        print("Streams retain separate timestamps; time_s arrays share the same origin.")
    print("timestamp_ns contains bag recording times, not sensor acquisition times.")


if __name__ == "__main__":
    main()
