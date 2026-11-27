# Using LeRobot to train/run on the arm

If this is your first time here, please source the setup script **from this folder:**
```bash
source lerobot_env_setup.bash
```

**When conda is installing, DO NOT let it override your terminal prompt (answer "no" when it prompts you).**

If this isn't your first time, run:
```bash
conda activate lerobot_arm
```

This gives you required packages and installs miniforge (conda and mamba). Please take note that **LeRobot uses Python 3.12, so it is not compatible with the rest of our ROS2 code and will use a socket to communicate with it.** Additionally, please use a laptop with a proper Nvidia GPU and drivers for training/inference (so probably the arm laptop). Additionally, this probably **takes 15-20 GB of disk space** so account for that before running. 




