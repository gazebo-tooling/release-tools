GAZEBO_RUNTIME_TEST="""
echo '# BEGIN SECTION: test the script'
wget -P /tmp/ https://github.com/osrf/gazebo_models/archive/master.tar.gz
mkdir -p ~/.gazebo/models
tar -xvf /tmp/master.tar.gz -C ~/.gazebo/models --strip 1
rm /tmp/master.tar.gz

TEST_START=\`date +%s\`
timeout --preserve-status 180 gazebo --verbose || true
TEST_END=\`date +%s\`
DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

if [ \$DIFF -lt 180 ]; then
   echo 'The test took less than 180s. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""

GAZEBO_MODEL_INSTALLATION="""
wget -q https://github.com/osrf/gazebo_models/archive/master.tar.gz -O /tmp/master.tar.gz
mkdir -p ~/.gazebo/models
tar -xf /tmp/master.tar.gz -C ~/.gazebo/models --strip 1 >/dev/null 2>&1
rm /tmp/master.tar.gz"""

GZ_SIM_RUNTIME_TEST="""
echo '# BEGIN SECTION: test the script'
TEST_START=\`date +%s\`
timeout --preserve-status 180 ign gazebo -v -r camera_sensor.sdf || true
TEST_END=\`date +%s\`
DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

if [ \$DIFF -lt 180 ]; then
   echo 'The test took less than 180s. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""

# autopkgtest is a mechanism to test the installation of the generated packages
# at the end of the package production.
RUN_AUTOPKGTEST=${RUN_AUTOPKGTEST:-true}

DEBBUILD_AUTOPKGTEST="""
if $RUN_AUTOPKGTEST; then
echo '# BEGIN SECTION: run autopkgtest'
cd $WORKSPACE/pkgs
set +e
sudo autopkgtest --no-auto-control -B *.deb *.dsc -- null
# autopkgtest will return 0 if there are successful tests and 8 if there are no tests
testret=\$?
if [[ \$testret != 0 ]] && [[ \$testret != 8 ]]; then
  echo 'Problem in running autopkgtest: \$testret'
  exit 1
fi
set -e
echo '# END SECTION'
fi
"""

MKBUILD_INSTALL_DEPS="""
echo '# BEGIN SECTION: install build dependencies'
mkdir build-deps
cd build-deps
seconds_waiting=0
while (! \$update_done); do
  sudo DEBIAN_FRONTEND=noninteractive mk-build-deps \
    -r -i debian/control \
    --tool 'apt-get --yes -o Debug::pkgProblemResolver=yes -o  Debug::BuildDeps=yes' \
  && break
  sleep 60 && seconds_waiting=\$((seconds_waiting+60))
  [ \$seconds_waiting -ge \$timeout ] && exit 1
done
cd ..
# clean up files leftover by mk-build-deps
rm -rf build-deps
echo '# END SECTION'
"""
