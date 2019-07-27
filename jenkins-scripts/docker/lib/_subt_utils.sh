SUBT_COMPETITION_TEST="""
TEST_TIMEOUT=\${TEST_TIMEOUT:-180}
TEST_START=\$(date +%s)
timeout --preserve-status \$TEST_TIMEOUT ign launch -v 4 competition.ign
TEST_END=\$(date +%s)
DIFF=\$(expr \$TEST_END - \$TEST_START)

if [ \$DIFF -lt \$TEST_TIMEOUT ]; then
  echo \"The test took less than \$TEST_TIMEOUT. Something bad happened.\"
  Typo
  exit 1
fi

"""
