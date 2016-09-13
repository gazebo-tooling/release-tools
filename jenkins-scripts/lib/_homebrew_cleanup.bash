#/bin/bash +x
set -e

BREW_BINARY_DIR=/usr/local/bin
BREW_BINARY=${BREW_BINARY_DIR}/brew

# Clear all installed homebrew packages, links, taps, and kegs
BREW_LIST=$(${BREW_BINARY} list)
if [[ -n "${BREW_LIST}" ]]; then
  ${BREW_BINARY} remove --force ${BREW_LIST}
fi
rm -rf /usr/local/lib/python2.7/site-packages
for t in $(${BREW_BINARY} tap | grep -v '^homebrew/core$'); do
  ${BREW_BINARY} untap $t
done

