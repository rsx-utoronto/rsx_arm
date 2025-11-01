# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
ARG SHELL_FLAVOR=zsh     # zsh | bash
ARG EDITOR_FLAVOR=nvim   # nvim | vscode
ARG WITH_GUI=1           

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    WS_DIR=/arm_ros2_ws \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    # toggle dotfiles sync on entry
    SYNC_DOTFILES_ON_START=1

# ---------- Base OS deps ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales sudo tzdata ca-certificates \
    git curl wget bash-completion \
    build-essential cmake pkg-config \
    python3-pip python3-venv python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    iproute2 iputils-ping net-tools usbutils \
    less nano vim \
    rsync \
    libpcl-dev \
    ripgrep fd-find wl-clipboard xclip \
 && ln -sfn /usr/bin/fdfind /usr/local/bin/fd \
 && rm -rf /var/lib/apt/lists/*

# --- Go toolchain (official tarball) ---
ARG GO_VERSION=1.24.1
RUN set -eux; \
  apt-get update && apt-get install -y --no-install-recommends ca-certificates curl; \
  curl -fsSL --retry 5 --retry-delay 2 -o /tmp/go.tgz "https://go.dev/dl/go${GO_VERSION}.linux-amd64.tar.gz"; \
  rm -rf /usr/local/go && tar -C /usr/local -xzf /tmp/go.tgz; \
  ln -sfn /usr/local/go/bin/go /usr/local/bin/go; \
  go version; \
  rm -f /tmp/go.tgz; \
  rm -rf /var/lib/apt/lists/*

# --- Lazygit CLI (build from source with modern Go) ---
RUN set -eux; \
  apt-get update && apt-get install -y --no-install-recommends git ca-certificates && \
  GOBIN=/usr/local/bin GOPATH=/root/go GO111MODULE=on go install github.com/jesseduffield/lazygit@latest && \
  lazygit --version; \
  rm -rf /root/go && \
  rm -rf /var/lib/apt/lists/*

# --- Clang toolchain (compiler + LSP + format/tidy) for ROS Nvim ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    clang clangd clang-format clang-tidy lldb lld ccache python3-colcon-mixin \
 && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

# --- Node.js (LTS) for JS/TS-based language servers like Pyright ---
ARG NODE_VERSION=20.17.0
RUN set -eux; \
  apt-get update && apt-get install -y --no-install-recommends ca-certificates curl xz-utils; \
  arch="$(dpkg --print-architecture)"; \
  case "$arch" in \
    amd64) node_arch="x64" ;; \
    arm64) node_arch="arm64" ;; \
    *) echo "Unsupported arch: $arch"; exit 1 ;; \
  esac; \
  curl -fsSL --retry 5 --retry-delay 2 -o /tmp/node.tgz "https://nodejs.org/dist/v${NODE_VERSION}/node-v${NODE_VERSION}-linux-${node_arch}.tar.xz"; \
  tar -C /usr/local -xJf /tmp/node.tgz; \
  ln -sfn "/usr/local/node-v${NODE_VERSION}-linux-${node_arch}" /usr/local/node; \
  ln -sfn /usr/local/node/bin/node /usr/local/bin/node; \
  ln -sfn /usr/local/node/bin/npm  /usr/local/bin/npm; \
  ln -sfn /usr/local/node/bin/npx  /usr/local/bin/npx; \
  node -v && npm -v; \
  rm -f /tmp/node.tgz; \
  rm -rf /var/lib/apt/lists/*

# --- pynvim (Python provider) and Pyright LSP ---
# Make Mason-installed tools available for any user at interactive shell
RUN printf 'export PATH="$HOME/.local/share/nvim/mason/bin:$HOME/.local/bin:$PATH"\n' >/etc/profile.d/10-mason-path.sh
RUN apt-get update && apt-get install -y --no-install-recommends python3-pynvim \
 && rm -rf /var/lib/apt/lists/*
RUN npm install -g pyright

# --- Python tooling pin + project deps (keeps colcon happy) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-rclpy \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-evdev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --upgrade pip \
 && python3 -m pip install "setuptools>=70" wheel \
 && python3 -m pip install --no-cache-dir --root-user-action=ignore --upgrade-strategy only-if-needed -r /tmp/requirements.txt


# Make Node & Mason bins available in all interactive shells + hard symlinks for pyright
RUN set -eux; \
  printf '%s\n' \
    '# Dev tool paths for all users' \
    'export PATH="/usr/local/node/bin:$HOME/.local/share/nvim/mason/bin:$HOME/.local/bin:$PATH"' \
    > /etc/profile.d/dev_paths.sh && \
  chmod 0644 /etc/profile.d/dev_paths.sh && \
  echo '. /etc/profile.d/dev_paths.sh' >> /etc/bash.bashrc && \
  if [ -f /etc/zsh/zshrc ]; then echo '. /etc/profile.d/dev_paths.sh' >> /etc/zsh/zshrc; fi && \
  ln -sfn /usr/local/node/bin/pyright            /usr/local/bin/pyright && \
  ln -sfn /usr/local/node/bin/pyright-langserver /usr/local/bin/pyright-langserver


# ---------- User ----------
RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p /arm_ros2_ws/src/rsx_arm && chown -R ${USER_UID}:${USER_GID} /arm_ros2_ws

# --- RealSense ROS wrapper + librealsense from ROS servers ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-librealsense2* \
  && rm -rf /var/lib/apt/lists/*

# ---------- Optional shells & editors ----------
ARG NEOVIM_VERSION=0.10.3 
RUN set -eux; \
  if [ "${SHELL_FLAVOR}" = "zsh" ]; then \
    apt-get update && apt-get install -y --no-install-recommends zsh && \
    rm -rf /var/lib/apt/lists/*; \
  fi; \
  if [ "${EDITOR_FLAVOR}" = "nvim" ]; then \
    # --- Neovim via official binary (guaranteed >= 0.10) ---
    apt-get update && apt-get purge -y neovim neovim-runtime || true; \
    apt-get install -y --no-install-recommends ca-certificates curl; \
    url_base="https://github.com/neovim/neovim/releases/download/v${NEOVIM_VERSION}"; \
    curl -fL --retry 5 --retry-delay 2 -o /tmp/nvim-linux64.tar.gz "${url_base}/nvim-linux64.tar.gz"; \
    curl -fL --retry 5 --retry-delay 2 -o /tmp/nvim-linux64.tar.gz.sha256sum "${url_base}/nvim-linux64.tar.gz.sha256sum"; \
    (cd /tmp && sha256sum -c nvim-linux64.tar.gz.sha256sum); \
    tar -xzf /tmp/nvim-linux64.tar.gz -C /opt; \
    ln -sfn /opt/nvim-linux64/bin/nvim /usr/local/bin/nvim; \
    nvim --headless +"lua assert(vim.version().minor>=10, 'Need Neovim >= 0.10')" +qa; \
    echo 'export EDITOR=nvim VISUAL=nvim' >/etc/profile.d/99-editor.sh; \
    rm -f /tmp/nvim-linux64.tar.gz /tmp/nvim-linux64.tar.gz.sha256sum; \
    rm -rf /var/lib/apt/lists/*; \
  fi

# --- VS Code (desktop GUI app) ---
ARG HOST_FLAVOR=linux-x11  # linux-x11 | linux-wayland | mac-xquartz | win-xserver (informational)
RUN set -eux; \
  apt-get update && apt-get install -y --no-install-recommends wget gpg apt-transport-https ca-certificates; \
  wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /usr/share/keyrings/packages.microsoft.gpg; \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" \
    > /etc/apt/sources.list.d/vscode.list; \
  apt-get update && apt-get install -y --no-install-recommends \
    code \
    libasound2 libxkbfile1 libsecret-1-0 libnss3 libx11-xcb1 libxshmfence1 libgbm1 libxrandr2 libxi6 libgtk-3-0; \
  rm -rf /var/lib/apt/lists/*

# Launchers for explicit X11 vs Wayland
RUN printf '%s\n' \
  '#!/usr/bin/env bash' \
  'exec /usr/bin/code --no-sandbox --unity-launch "$@"' \
  > /usr/local/bin/code-x11 && chmod +x /usr/local/bin/code-x11 && \
  printf '%s\n' \
  '#!/usr/bin/env bash' \
  'export ELECTRON_OZONE_PLATFORM_HINT=wayland' \
  'export OZONE_PLATFORM=wayland' \
  'export GTK_BACKEND=wayland' \
  'exec /usr/bin/code --no-sandbox --ozone-platform=wayland --enable-features=WaylandWindowDecorations "$@"' \
  > /usr/local/bin/code-wayland && chmod +x /usr/local/bin/code-wayland

# ---------- rosdep ----------
RUN rosdep init || true
RUN rosdep update

# ---------- Entry + dotfiles sync helpers ----------
COPY .devcontainer/entrypoint.sh /usr/local/bin/ros2_entrypoint.sh
COPY .devcontainer/sync_dotfiles.sh /usr/local/bin/sync_dotfiles.sh
RUN chmod +x /usr/local/bin/ros2_entrypoint.sh /usr/local/bin/sync_dotfiles.sh

# ---------- Auto-source ROS/workspace for interactive shells ----------
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /etc/skel/.bashrc
RUN echo 'if [ -f "$WS_DIR/install/setup.bash" ]; then source "$WS_DIR/install/setup.bash"; fi' >> /etc/skel/.bashrc
RUN if [ "${SHELL_FLAVOR}" = "zsh" ]; then \
      echo 'emulate sh -c "source /opt/ros/$ROS_DISTRO/setup.bash" >/dev/null 2>&1' >> /etc/skel/.zshrc && \
      echo 'if [ -f "$WS_DIR/install/setup.zsh" ]; then source "$WS_DIR/install/setup.zsh"; fi' >> /etc/skel/.zshrc; \
    fi


# --- GLOBAL: source ROS + workspace for ALL interactive shells (bash & zsh)
# Use POSIX-friendly setup.sh so it works in any shell.
RUN mkdir -p /etc/profile.d && \
    cat >/etc/profile.d/ros2_auto.sh <<'EOF'
: "${ROS_DISTRO:=humble}"
# Source ROS distro
if [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then . "/opt/ros/${ROS_DISTRO}/setup.sh"; fi
# Source workspace overlay if present
if [ -n "${WS_DIR}" ] && [ -f "${WS_DIR}/install/setup.sh" ]; then . "${WS_DIR}/install/setup.sh"; fi
EOF
# Ensure interactive shells read it
RUN echo ". /etc/profile.d/ros2_auto.sh" >> /etc/bash.bashrc && \
    if [ -f /etc/zsh/zshrc ]; then echo ". /etc/profile.d/ros2_auto.sh" >> /etc/zsh/zshrc; fi

# --- ROS 2 visualization + sim (Humble) ---
# RViz2 + rqt suite + Gazebo Classic + helpers for graphs & GL
RUN if [ "$WITH_GUI" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        "ros-${ROS_DISTRO}-rviz2" \
        "ros-${ROS_DISTRO}-rqt" "ros-${ROS_DISTRO}-rqt-graph" "ros-${ROS_DISTRO}-rqt-image-view" "ros-${ROS_DISTRO}-rqt-tf-tree" \
        "ros-${ROS_DISTRO}-joint-state-publisher-gui" \
        "ros-${ROS_DISTRO}-gazebo-ros-pkgs" gazebo \
        graphviz python3-pydot \
        qtwayland5 libgl1-mesa-dri mesa-utils \
      && rm -rf /var/lib/apt/lists/*; \
    fi

# --- RViz Wayland fallback shim ---
RUN if [ "$WITH_GUI" = "1" ]; then \
      rviz_bin="/opt/ros/${ROS_DISTRO}/lib/rviz2/rviz2"; \
      if [ -x "${rviz_bin}" ] && [ ! -f "${rviz_bin}.real" ]; then \
        mv "${rviz_bin}" "${rviz_bin}.real"; \
        printf '%s\n' \
          '#!/usr/bin/env bash' \
          'set -euo pipefail' \
          'RVIZ_REAL="/opt/ros/${ROS_DISTRO:-humble}/lib/rviz2/rviz2.real"' \
          'if [ ! -x "$RVIZ_REAL" ]; then' \
          '  echo "rviz2 backend not found at ${RVIZ_REAL}" >&2' \
          '  exit 127' \
          'fi' \
          'if [ -n "${WAYLAND_DISPLAY:-}" ]; then' \
          '  case "${QT_QPA_PLATFORM:-}" in' \
          '    ""|wayland|wayland-egl)' \
          '      export QT_QPA_PLATFORM=xcb' \
          '      ;;' \
          '  esac' \
          'fi' \
          'exec "$RVIZ_REAL" "$@"' \
        > "${rviz_bin}"; \
        chmod +x "${rviz_bin}"; \
      fi; \
    fi

# --- Gazebo Wayland->X11 fallback wrappers (for Hyprland) ---
RUN set -eux; \
  printf '%s\n' '#!/usr/bin/env bash' \
                'export QT_QPA_PLATFORM=xcb' \
                'exec gazebo "$@"' \
    > /usr/local/bin/gazebo_x11 && chmod +x /usr/local/bin/gazebo_x11; \
  printf '%s\n' '#!/usr/bin/env bash' \
                'export QT_QPA_PLATFORM=xcb' \
                'exec gzclient "$@"' \
    > /usr/local/bin/gzclient_x11 && chmod +x /usr/local/bin/gzclient_x11

# --- Auto-alias Gazebo to X11 only when running under Wayland ---
RUN printf '%s\n' \
'# If we are on Wayland, prefer X11 for Gazebo GUI (more stable)' \
'if [ -n "$WAYLAND_DISPLAY" ]; then' \
'  alias gazebo="gazebo_x11"' \
'  alias gzclient="gzclient_x11"' \
'fi' \
> /etc/profile.d/20-gazebo-wayland.sh

# --- GLOBAL: lsd fallback shim so aliases like `alias ls=lsd` still work
RUN cat >/etc/profile.d/lsd_shim.sh <<'EOF'
# Provide lsd fallback if not installed
if ! command -v lsd >/dev/null 2>&1; then
  lsd() { command ls --color=auto "$@"; }
  alias lsd='ls --color=auto'
fi
EOF
RUN echo ". /etc/profile.d/lsd_shim.sh" >> /etc/bash.bashrc && \
    if [ -f /etc/zsh/zshrc ]; then echo ". /etc/profile.d/lsd_shim.sh" >> /etc/zsh/zshrc; fi

# Optional: try to install real lsd (ignore if repo doesn’t have it)
RUN apt-get update && (apt-get install -y --no-install-recommends lsd || true) && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}
WORKDIR ${WS_DIR}
RUN cp -n /etc/skel/.bashrc ~/.bashrc || true; \
    if [ "${SHELL_FLAVOR}" = "zsh" ]; then cp -n /etc/skel/.zshrc ~/.zshrc || true; fi

# Give GUI user access to GPU devices commonly exposed (/dev/dri)
RUN groupadd -f video && groupadd -f render || true && \
    usermod -aG video,render ${USERNAME} || true

SHELL ["/bin/bash", "-lc"]
ENTRYPOINT ["/usr/local/bin/ros2_entrypoint.sh"]
CMD [ "bash" ]
