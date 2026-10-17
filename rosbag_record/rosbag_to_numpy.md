# RosBag to NumPy Commands

To convert a .db3 file to a numpy array to save the states and commands:
```bash
python rosbag_to_numpy.py <input_filename>.db3 --state-topic /arm_curr_angles --output <output_filename>.npz
```

To include commands only:
```bash
python rosbag_to_numpy.py <input_filename>.db3 --output <output_filename>.npz
```