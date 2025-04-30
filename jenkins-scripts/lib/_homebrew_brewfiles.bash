# Verify that the input brew bundle command is one of the allowed commands
# $1 is the brew bundle command string
# returns the command string if valid, otherwise returns a empty string
validate_brew_bundle_cmd () {
  brew_cmd=$1
  IFS=$' ' valid_brew_commands=("brew tap")
  if [[ " ${valid_brew_commands[@]} " =~ " ${brew_cmd} " ]]; then
    echo ${brew_cmd}
  else
    echo ""
  fi
}

# Verify that the input homebrew repo is in the whitelist
# $1 is the homebrew repo to check
# returns the repo string  if valid, otherwise returns a empty string
validate_brew_bundle_repo() {
  local repo=${1} whitelist_file="${SCRIPT_DIR}/tools/homebrew_repo_whitelist"

  if [[ ! -f ${whitelist_file} ]]; then
    # Not found whitelist file
    echo ""
  fi

  IFS=$'\n' repos=($(cat ${whitelist_file}))
  if [[ " ${repos[@]} " =~ " ${repo} " ]]; then
    echo ${repo}
  else
    echo ""
  fi
}

# Verify that the input Brewfile contains only allowed commands and repos in
# our whitelist
# $1 is the Brewfile to check
# returns error message if not valid
validate_brewfile() {
  brewfile=$1
  IFS=$'\n' lines=($(cat $brewfile))
  for i in $(seq ${#lines[*]}); do
    line=${lines[$i-1]}
    IFS=$' ' tokens=($line)
    checked_cmd=false
    for j in "${tokens[@]}"; do
       token=${j}
       # A token that starts with # is a comment. Ignore the rest of the line.
       if [[ "${token}" == \#* ]]; then
         break
       fi
       # The first token in a line should be a brew command
       # Check to make sure the brew command is allowed
       if [[ $checked_cmd == false ]]; then
         local brew_cmd=$(validate_brew_bundle_cmd ${token})
         if [[ ${brew_cmd} == "" ]]; then
           echo "brew bundle command not allowed: ${brew_cmd}"
           return
         fi
         checked_cmd=true
       else
         # The token that follows the brew command may contain the repo name
         # e.g.
         #   tap org/repo
         #   brew org/repo/package
         # homebrew-core packages do not have repo name in them
         # e.g.
         #   brew package
         # Remove quotes and check to make sure the homebrew repo is allowed
         # if specified
         repo=`echo "${token}" | cut -d'"' -f 2`
         validated_repo=""
         if [[ ${brew_cmd} == "tap" ]]; then
           validated_repo=$(validate_brew_bundle_repo ${repo})
         else
           if [[ ${repo} == "*/*" ]]; then
             repo="${token%/*}"
             validated_repo=$(validate_brew_bundle_repo ${repo})
           else
             validated_repo=${repo}
           fi
         fi
         if [[ ${validated_repo} == "" ]]; then
           echo "homebrew repo not allowed: ${repo}"
           return
         fi
       fi
    done
  done
}
