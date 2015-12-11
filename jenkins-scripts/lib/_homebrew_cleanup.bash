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
