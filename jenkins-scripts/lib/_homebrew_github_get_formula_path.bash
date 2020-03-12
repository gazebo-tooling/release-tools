# Parameters:
# - PACKAGE_ALIAS [mandatory] name of package including major version
# Return:
# -> FORMULA_PATH

if [ -z ${PACKAGE_ALIAS} ]; then
    echo "PACKAGE_ALIAS variables is empty"
    exit -1
fi

echo '# BEGIN SECTION: check if the formula exists'
echo
if [ -s ${TAP_PREFIX}/${PACKAGE_ALIAS}.rb ]; then
  FORMULA=${TAP_PREFIX}/${PACKAGE_ALIAS}.rb
elif [ -s ${TAP_PREFIX}/Formula/${PACKAGE_ALIAS}.rb ]; then
  FORMULA=${TAP_PREFIX}/Formula/${PACKAGE_ALIAS}.rb
elif [ -s ${TAP_PREFIX}/Aliases/${PACKAGE_ALIAS} ]; then
  FORMULA=${TAP_PREFIX}/Aliases/${PACKAGE_ALIAS}
else
  echo Formula for ${PACKAGE_ALIAS} not found
  [[ -d homebrew-simulation ]] && ls homebrew-simulation/*
  # Mark the build as unstable (using logparser plugin)
  echo "MARK_AS_UNSTABLE"
  exit 0
fi
echo '# END SECTION'

echo '# BEGIN SECTION: export formula path'
export FORMULA_PATH=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.path"`
echo Modifying ${FORMULA_PATH}
echo '# END SECTION'
