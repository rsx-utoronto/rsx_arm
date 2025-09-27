param(
  [ValidateSet("base","windows")] [string]$Profile = "windows",
  [ValidateSet("zsh","bash")]     [string]$Shell   = "zsh",
  [ValidateSet("nvim","vscode")]  [string]$Editor  = "nvim"
)
$env:SHELL_FLAVOR = $Shell
$env:EDITOR_FLAVOR = $Editor
docker compose build rsxrover
docker compose up -d ("rsxrover-" + $Profile.Trim())
