support_lintian_calls() {
  if  ( [ ${DISTRO} == 'focal' ] || [ ${DISTRO} == 'buster' ] ) && \
      ( [ ${ARCH} == 'arm64' ] || [ ${ARCH} == 'armhf' ] ); then
    echo false
  else
    echo true
  fi
}
