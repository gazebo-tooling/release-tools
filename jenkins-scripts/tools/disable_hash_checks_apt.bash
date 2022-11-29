#!/bin/bash

cat > /etc/apt/apt.conf.d/99fixbadproxy <<-EOF
Acquire::http::Pipeline-Depth "0";
Acquire::http::No-Cache=True;
Acquire::BrokenProxy=true;
EOF

sudo rm  /var/lib/apt/lists/*
sudo rm  /var/lib/apt/lists/partial/*
sudo apt-get update
