VRX_SMOKE_TEST="""
echo '# BEGIN SECTION: smoke test'

TEST_TIMEOUT=180
TEST_START=\$(date +%s)
timeout --preserve-status \$TEST_TIMEOUT roslaunch vrx_gazebo sandisland.launch extra_gazebo_args:=\"--verbose\"
TEST_END=\$(date +%s)
DIFF=\$(expr \$TEST_END - \$TEST_START)

if [ \$DIFF -lt \$TEST_TIMEOUT ]; then
  echo \"The test took less than \$TEST_TIMEOUT. Something bad happened.\"
  Typo
  exit 1
fi

echo 'Smoke testing completed successfully.'
echo '# END SECTION'
"""
