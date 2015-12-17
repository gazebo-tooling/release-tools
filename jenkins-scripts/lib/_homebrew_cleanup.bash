set -e
set +x

# Backup brew executable.
mv /usr/local/bin/brew /tmp/brew

# Clear all installed homebrew packages, links, taps, and kegs
rm -rf /usr/local/Cellar/*
rm -rf /usr/local/bin/*
rm -rf /usr/local/lib/*
rm -rf /usr/local/include/*
rm -rf /usr/local/Library/Taps/*
rm -rf /usr/local/Library/LinkedKegs/*

# Restore brew executable.
mv /tmp/brew /usr/local/bin/brew

# Restore the basic stuff
if [[ -n ${SCRIPT_LIBDIR} ]]; then
  . ${SCRIPT_LIBDIR}/dependencies_archive.sh
elif [[ -n ${SCRIPT_DIR} ]]; then
  . ${SCRIPT_DIR}/lib/dependencies_archive.sh
else
  echo "Can not find the dependencies_archive.sh"
  exit -1
fi

brew update
brew install ${BREW_BASE_DEPENDCIES}
