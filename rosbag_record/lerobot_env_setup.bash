#!/usr/bin/bash

# Does literally what it's named:
# - Makes the python venv (we need minimum version 3.12, while 
#   humble uses 3.10 so it should ONLY be used for lerobot)
# - Installs lerobot and other stuff in the env

# PLEASE RUN FROM THIS DIRECTORY lol

# Make sure we aren't in arm_env
_check_clean_env() {
    local bad=0

    # Any conda env other than base (also catches envs activated by path)
    if [[ -n "${CONDA_DEFAULT_ENV:-}" && "${CONDA_DEFAULT_ENV}" != "base" ]]; then
        echo "Error: conda env '${CONDA_DEFAULT_ENV}' is active (only 'base' is allowed). Run 'conda deactivate'." >&2
        bad=1
    fi

    # Any pip/venv/virtualenv/poetry/pipenv env (all of these set VIRTUAL_ENV)
    if [[ -n "${VIRTUAL_ENV:-}" ]]; then
        echo "Error: virtual env '${VIRTUAL_ENV}' is active. Run 'deactivate'." >&2
        bad=1
    fi

    return $bad
}

_check_clean_env || { unset -f _check_clean_env; return 1 2>/dev/null || exit 1; }
unset -f _check_clean_env

# The actual script
cd ../../..
WS_HOME="$(pwd)"
cd ../Downloads

# Get from deadsnakes since it isn't available by default on 22.04
# sudo apt install software-properties-common
# sudo add-apt-repository ppa:deadsnakes/ppa
# sudo apt update
# sudo apt install python3.12 python3.12-venv python3.12-dev

# Some stuff huggingface needs
# wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
# bash Miniforge3-$(uname)-$(uname -m).sh

cd "$WS_HOME/src/rsx_arm/rosbag_record"

# Make the conda env
# eval "$($HOME/miniconda3/bin/conda shell.bash hook)" 
# conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
# conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
# conda create -y -n lerobot_arm python=3.12
# return 0
conda activate lerobot_arm

conda install ffmpeg -c conda-forge

# Make sure we have dependencies first
sudo apt-get install cmake build-essential python3-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev

# Don't know wtf we actually need so just install everything for now lol
pip install 'lerobot[all]'                   # Everything