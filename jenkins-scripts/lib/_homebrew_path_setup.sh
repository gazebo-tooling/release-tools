#/bin/sh +x
set -e

# exit if brew command is already in the PATH
command -v brew && exit 0

echo brew not in PATH, checking standard locations

# otherwise check in standard brew locations using the precedence order from
# https://docs.brew.sh/Tips-and-Tricks
BREW=$(PATH="/opt/homebrew/bin:/home/linuxbrew/.linuxbrew/bin:/usr/local/bin" command -v brew)
which $BREW && eval "$($BREW shellenv)"

echo brew found at $(command -v brew)
