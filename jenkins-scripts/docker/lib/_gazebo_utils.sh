GAZEBO_RUNTIME_TEST="""
echo '# BEGIN SECTION: test the script'
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
wget -q https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz -O /tmp/default.tar.gz
mkdir -p ~/.gazebo/models
tar -xf /tmp/default.tar.gz -C ~/.gazebo/models --strip 1
rm /tmp/default.tar.gz"""
