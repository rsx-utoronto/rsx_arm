#!/usr/bin/env bash
set -Eeuo pipefail
# Usage:
#   docker-scripts/add_aliases.sh            # auto: update whichever shells exist
#   docker-scripts/add_aliases.sh --bash     # only bash
#   docker-scripts/add_aliases.sh --zsh      # only zsh
#   docker-scripts/add_aliases.sh --both     # bash + zsh
#
# If sourced, reloads your rc in-place. If executed, execs a new interactive shell.

WS_DIR="${WS_DIR:-/arm_ros2_ws}"
: "${ROS_DISTRO:=humble}"
TARGET="auto"

if [[ "${1:-}" == "--bash" ]]; then TARGET="bash"; fi
if [[ "${1:-}" == "--zsh"  ]]; then TARGET="zsh";  fi
if [[ "${1:-}" == "--both" ]]; then TARGET="both"; fi

# Detect if this script is being sourced
is_sourced() {
  # bash
  if [ -n "${BASH_SOURCE:-}" ] && [ "${BASH_SOURCE[0]}" != "$0" ]; then return 0; fi
  # zsh
  if [ -n "${ZSH_EVAL_CONTEXT:-}" ] && [[ "$ZSH_EVAL_CONTEXT" == *":file"* ]]; then return 0; fi
  return 1
}

# Escape chars that are special in sed *replacement* (/ & @ etc.)
# We use @ as the delimiter, so escape @ and &
escape_sed_repl() {
  local s="$1"
  s=${s//\\/\\\\}   # backslash (robustness)
  s=${s//&/\\&}     # & is "whole match"
  s=${s//@/\\@}     # our delimiter
  printf '%s' "$s"
}

# Freeze values into alias bodies at install time
_ws="$WS_DIR"
_ros="$ROS_DISTRO"

workon_body="cd $_ws; \
[ -f /opt/ros/$_ros/setup.sh ] && . /opt/ros/$_ros/setup.sh; \
[ -f $_ws/arm_env/bin/activate ] && . $_ws/arm_env/bin/activate || true; \
[ -f $_ws/install/setup.sh ] && . $_ws/install/setup.sh || true"

build_body="workon_arm && colcon build && colcon test --ctest-args tests --packages-skip arm_msgs && colcon test-result --all --verbose"

format_body="autopep8 --in-place --recursive src/rsx_arm"

write_aliases() {
  local rc="$1"
  local shell_name="$2"

  mkdir -p "$(dirname "$rc")"
  touch "$rc"

  local w_repl b_repl
  w_repl="$(escape_sed_repl "alias workon_arm='$workon_body'")"
  b_repl="$(escape_sed_repl "alias build_arm='$build_body'")"
  f_line="alias format_code='$format_body'"

  if grep -qE '^alias[[:space:]]+workon_arm=' "$rc"; then
    sed -i.bak -E "s@^alias[[:space:]]+workon_arm=.*\$@${w_repl}@g" "$rc"
  else
    printf "\n# --- ROS 2 helpers (%s / %s) ---\n%s\n" "$_ws" "$_ros" "$(printf %s "$w_repl" | sed 's/^alias //')" >> "$rc"
  fi

  if grep -qE '^alias[[:space:]]+build_arm=' "$rc"; then
    sed -i.bak -E "s@^alias[[:space:]]+build_arm=.*\$@${b_repl}@g" "$rc"
  else
    printf "%s\n" "$(printf %s "$b_repl" | sed 's/^alias //')" >> "$rc"
  fi

  if ! grep -qE '^alias[[:space:]]+format_code=' "$rc"; then
    printf "%s\n" "$f_line" >> "$rc"
  fi

  echo "✔ Installed/updated aliases in $rc ($shell_name)"
}

do_bash=false
do_zsh=false
case "$TARGET" in
  bash) do_bash=true ;;
  zsh)  do_zsh=true  ;;
  both) do_bash=true; do_zsh=true ;;
  auto)
    [[ -n "${BASH_VERSION:-}" || -f "$HOME/.bashrc" ]] && do_bash=true
    [[ -n "${ZSH_VERSION:-}"  || -f "$HOME/.zshrc"  ]] && do_zsh=true
    if ! $do_bash && ! $do_zsh; then do_bash=true; fi
  ;;
esac

updated_bash=false
updated_zsh=false
$do_bash && { write_aliases "$HOME/.bashrc" "bash"; updated_bash=true; }
$do_zsh  && { write_aliases "$HOME/.zshrc"  "zsh";  updated_zsh=true; }

# Apply now
if is_sourced; then
  if [[ -n "${ZSH_VERSION:-}" && "$updated_zsh" == true ]]; then
    # shellcheck disable=SC1090
    source "$HOME/.zshrc"
    echo "↻ Reloaded ~/.zshrc in current zsh."
  fi
  if [[ -n "${BASH_VERSION:-}" && "$updated_bash" == true ]]; then
    # shellcheck disable=SC1090
    source "$HOME/.bashrc"
    echo "↻ Reloaded ~/.bashrc in current bash."
  fi
  return 0 2>/dev/null || exit 0
else
  if [ -t 1 ]; then
    if $updated_zsh && command -v zsh >/dev/null 2>&1; then
      echo "→ Spawning a new interactive zsh with aliases ready..."
      exec zsh -i
    elif $updated_bash && command -v bash >/dev/null 2>&1; then
      echo "→ Spawning a new interactive bash with aliases ready..."
      exec bash -i
    elif command -v "${SHELL:-}" >/dev/null 2>&1; then
      echo "→ Spawning a new interactive shell (${SHELL})..."
      exec "${SHELL}" -i
    fi
  fi
  echo
  echo "Reload manually with:  source ~/.zshrc   or   source ~/.bashrc"
  echo "Then test:              workon_arm && build_arm"
  echo
fi
