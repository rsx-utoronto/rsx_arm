# RosBag to NumPy Commands

To convert a .db3 file to a numpy array to save the states and commands:
```bash
python rosbag_to_numpy.py <input_filename>.db3 --state-topic /arm_curr_angles --output <output_filename>.npz
```

To include commands only:
```bash
python rosbag_to_numpy.py <input_filename>.db3 --output <output_filename>.npz
```

To include the camera images alongside commands and states:
```bash
python rosbag_to_numpy.py <input_filename>.db3 \
  --state-topic /arm_curr_angles \
  --camera-topic /camera/camera/color/image_raw \
  --output <output_filename>.npz
```

The archive contains `images` with shape `(frames, height, width, channels)` for
color images (or `(frames, height, width)` for single-channel images), plus
`image_timestamp_ns` and `image_time_s`. `image_encoding` records the ROS image
encoding, such as `rgb8` or `bgr8`. Image and state/command streams keep their
own timestamps and are not resampled.
