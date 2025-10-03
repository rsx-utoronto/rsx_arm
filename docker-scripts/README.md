# README - The “Why each script exists” Guide ✨

## Overview 🧭
This folder contains small, single-purpose scripts that make a ROS 2 + Docker workspace feel like a native dev setup. Each script covers one decision you’d otherwise repeat by hand.

---

## Scripts, Purpose, and Typical Use 🧩

### 1) `build.sh` — Build the image with your preferences 🏗️
**Why:** You likely prefer `zsh` or `bash`, and `nvim` or `vscode`. Bake that into the image so containers are consistent for all teammates.

**What it does:**
- 🧑‍💻 Detects host OS → sets `HOST_HOME` to mount your home correctly.
- 🆔 Exports `HOST_UID/GID` to avoid root-owned files.
- 🐳 Runs `docker compose build rsxarm`.

**Use:**
```bash
./build.sh zsh nvim    # or: ./build.sh bash vscode
```

---

### 2) `enter.sh` (profile launcher) — Start the right container for your host 🚀
**Why:** Wayland vs X11 vs macOS vs Windows vs WSL need different Compose services.

**What it does:**
- 🧭 Maps profile → service name:
  - `hyprland → rsxarm-wayland`
  - `x11 → rsxarm-x11`
  - `mac → rsxarm-mac`
  - `windows → rsxarm-win`
  - `wsl → rsxarm-wsl`
  - default `rsxarm`
- ▶️ Starts the service via `docker compose up -d`.

**Use:**
```bash
./enter.sh hyprland
./enter.sh x11
./enter.sh                # default
```

---

### 3) `enter.sh` (exec/attach) — Drop into a running container 💻
**Why:** You want an interactive shell in the container **that’s already running**.

**What it does:**
- 🔎 Finds the container id for the given service (default `rsxarm`).
- 🐚 Prefers `/bin/zsh` when present, otherwise `/bin/bash`.
- 🔗 `docker compose exec` attaches with `-i`.

**Use:**
```bash
./enter.sh          # exec into default service
./enter.sh rsxarm   # or another service name
```

> 💡 Consider renaming one of the two `enter.sh` files:
> - `up.sh` → profile launcher  
> - `enter.sh` → exec/attach

---

### 4) `setup_ws.sh` — One-line “source + build + overlay” 🧰
**Why:** You always need to source ROS, build with consistent flags, and overlay your environment.

**What it does:**
- 📦 Sources `/opt/ros/$ROS_DISTRO/setup.sh`
- 🧱 Builds with `colcon` and:
  - `-DCMAKE_BUILD_TYPE=${BUILD_TYPE:-RelWithDebInfo}`
  - `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` (for IDEs)
- 🧩 Sources your workspace overlay `install/setup.sh`

**Use:**
```bash
./setup_ws.sh
./setup_ws.sh --packages-select your_pkg
```

---

### 5) `deps.sh` — Resolve system dependencies via rosdep 🧯
**Why:** New packages appear in `src/`. Resolve their system deps quickly.

**What it does:**
- 🔄 `apt-get update`
- 📚 `rosdep update`
- ✅ `rosdep check … || rosdep install …` for packages under `src/`

**Use (inside container):**
```bash
./deps.sh
```

---

### 6) `add_alias.sh` — Quality-of-life aliases ⚙️
**Why:** Burn common actions into two keystrokes.

**Installs/updates:**
- 🧪 `workon_arm` → cd to `$WS_DIR`, source ROS distro + venv + overlay
- 🏗️ `build_arm` → `workon_arm && colcon build && colcon test …`
- 🧼 `format_code` → `autopep8 --in-place --recursive src/rsx_arm`

**Use:**
```bash
./add_alias.sh --both   # or --bash / --zsh / (no flag => auto)
# Then open a new shell or: source ~/.bashrc | ~/.zshrc
```

---

### 7) `clean_ws.sh` — Clean build artifacts safely 🧹
**Why:** Start fresh without fat-finger nukes.

**What it does:**
- 🛡️ Validates `$WS_DIR` isn’t empty or `/`
- 🗑️ Removes `build/ install/ log/`

**Use:**
```bash
./clean_ws.sh
```

---

## Environment at a Glance 🌱
- 📂 `WS_DIR=/arm_ros2_ws`
- 🤖 `ROS_DISTRO=humble`
- 🏗️ `BUILD_TYPE=RelWithDebInfo`
- 🏠 `HOST_HOME` auto-detected for Compose
- 👥 `HOST_UID/HOST_GID` auto-detected
- 🧩 Build-time knobs: `SHELL_FLAVOR`, `EDITOR_FLAVOR`

---

## Recommended Flow 🛣️

1. `./build.sh zsh nvim`  
2. `./enter.sh hyprland` (or the profile that fits your host)  
3. `./enter.sh rsxarm-wayland` (exec into the running container)  
4. `./deps.sh` (optional, for missing system deps)  
5. `./setup_ws.sh`  
6. `./add_alias.sh --both` (optional QoL)
