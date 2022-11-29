#!/bin/bash

cat > /etc/apt/apt.conf.d/99fixbadproxy <<-EOF
Acquire::http::Pipeline-Depth "0";
Acquire::http::No-Cache=True;
Acquire::BrokenProxy=true;
EOF

sudo rm -fr /var/lib/apt/lists/
sudo apt-get update
