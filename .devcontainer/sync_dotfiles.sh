#!/usr/bin/env bash
set -euo pipefail

# Copies host dotfiles (mounted at /host_home:ro) into the container HOME.
# It is safe to run repeatedly (rsync overwrites with host versions).

HOST_HOME="/host_home"
DEST_HOME="${HOME}"
EDITOR_FLAVOR="${EDITOR_FLAVOR:-nvim}"
SHELL_FLAVOR="${SHELL_FLAVOR:-zsh}"

copy_file() {
  local src="$1" dest="$2"
  if [ -f "$src" ]; then
    mkdir -p "$(dirname "$dest")"
    rsync -a --delete "$src" "$dest"
    echo "Synced $(basename "$src")"
  fi
}

copy_dir() {
  local src="$1" dest="$2"
  if [ -d "$src" ]; then
    mkdir -p "$dest"
    rsync -a --delete "$src"/ "$dest"/
    echo "Synced $(basename "$src")/"
  fi
}

# --- Shell configs ---
# Always sync both families so you can switch later without rebuild.
# zsh
copy_file "${HOST_HOME}/.zshrc" "${DEST_HOME}/.zshrc"
copy_file "${HOST_HOME}/.zprofile" "${DEST_HOME}/.zprofile"
copy_file "${HOST_HOME}/.zshenv" "${DEST_HOME}/.zshenv"
copy_dir "${HOST_HOME}/.config/zsh" "${DEST_HOME}/.config/zsh"
copy_dir "${HOST_HOME}/.oh-my-zsh" "${DEST_HOME}/.oh-my-zsh"

# bash
copy_file "${HOST_HOME}/.bashrc" "${DEST_HOME}/.bashrc"
copy_file "${HOST_HOME}/.bash_profile" "${DEST_HOME}/.bash_profile"
copy_file "${HOST_HOME}/.bash_aliases" "${DEST_HOME}/.bash_aliases"

# git (handy)
copy_file "${HOST_HOME}/.gitconfig" "${DEST_HOME}/.gitconfig"
copy_file "${HOST_HOME}/.gitignore_global" "${DEST_HOME}/.gitignore_global"

# --- Editors ---
if [ "${EDITOR_FLAVOR}" = "nvim" ]; then
  # Neovim configs
  copy_dir "${HOST_HOME}/.config/nvim" "${DEST_HOME}/.config/nvim"
  # Optional: user chooses whether to sync data/cache; usually regenerated
  copy_dir "${HOST_HOME}/.local/share/nvim" "${DEST_HOME}/.local/share/nvim"
  copy_dir "${HOST_HOME}/.cache/nvim" "${DEST_HOME}/.cache/nvim"
else
  # VS Code (note: for VS Code *Remote* the UI settings live on host;
  # copying here mainly helps code-server or CLI tooling inside container)
  copy_dir "${HOST_HOME}/.config/Code/User" "${DEST_HOME}/.config/Code/User"
  copy_dir "${HOST_HOME}/.vscode" "${DEST_HOME}/.vscode"
fi

# Normalize ownership (in case of root-placed files)
chown -R "$(id -u)":"$(id -g)" "${DEST_HOME}" || true

# Ensure helper functions in the synced zshrc don't abort container startup.
if [ -f "${DEST_HOME}/.zshrc" ]; then
  perl -0pi -e 's/command -v "\$1" >\/dev\/null 2>&1 && "\$@";/command -v "\$1" >\/dev\/null 2>&1 && "\$@" || true;/' "${DEST_HOME}/.zshrc" || true
fi
