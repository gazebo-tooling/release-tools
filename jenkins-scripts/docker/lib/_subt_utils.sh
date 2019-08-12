SUBT_FUEL_DOWNLOADS="""
# Download virtual stix models
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Edgar Mine Virtual STIX' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Base Station' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Black and Decker Cordless Drill' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Fiducial' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Fire Extinguisher' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/JanSport Backpack Red' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Jersey Barrier' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Rescue Randy Sitting' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Samsung J8 Black' -v 4

# Download the robot models
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 1' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 2' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 3' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 4' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 5' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 1' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 2' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 3' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 4' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 5' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 6' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X3 UAV Config 1' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X3 UAV Config 2' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X3 UAV Config 3' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X3 UAV Config 4' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 1' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 2' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 3' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 4' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 5' -v 4

# Download the tunnel circuit models
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 1' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 2' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 5' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 6' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 7' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile Blocker' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Constrained Tunnel Tile Short' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Constrained Tunnel Tile Tall' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile 4-way Intersection' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile 90-degree Turn' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile Straight' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile Ramp' -v 4
ign fuel download --url 'https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile Vertical Shaft' -v 4
"""

SUBT_COMPETITION_TEST="""

${SUBT_FUEL_DOWNLOADS}

# inject external variable into test scripts
if [ -n "$TEST_TIMEOUT" ]; then
  export TEST_TIMEOUT=${TEST_TIMEOUT}
fi

TEST_TIMEOUT=\${TEST_TIMEOUT:-180}
TEST_TIMEOUT_KILL=\$((TEST_TIMEOUT + 30))
TEST_START=\$(date +%s)
timeout --preserve-status -k \${TEST_TIMEOUT_KILL} \$TEST_TIMEOUT ign launch -v 4 tunnel_circuit_practice.ign 2>&1 | grep -v -e '[QT]' -e '\"\"'
TEST_END=\$(date +%s)
DIFF=\$(expr \$TEST_END - \$TEST_START)

if [ \$DIFF -lt \$TEST_TIMEOUT ]; then
  echo \"The test took less than \$TEST_TIMEOUT. Something bad happened.\"
  exit 1
fi
"""
