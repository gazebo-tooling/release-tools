#/bin/sh +x
set -e

# skip logic if brew command is already in the PATH
if ! which -s brew; then

  echo brew not in PATH, checking standard locations

  # otherwise check in standard brew locations using the precedence order from
  # https://docs.brew.sh/Tips-and-Tricks
  BREW=$(PATH="/opt/homebrew/bin:/home/linuxbrew/.linuxbrew/bin:/usr/local/bin" command -v brew)
  which -s $BREW && eval "$($BREW shellenv)"

  echo brew found at $(command -v brew)

fi
