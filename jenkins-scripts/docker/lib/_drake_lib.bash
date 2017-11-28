DRAKE_BAZEL_INSTALL="""
echo '# BEGIN SECTION: install bazel'
# Install Bazel. Part of the install_prereq.sh script from Drake. They are not using the latest version of bazel
wget -O /tmp/bazel_0.6.1-linux-x86_64.deb https://github.com/bazelbuild/bazel/releases/download/0.6.1/bazel_0.6.1-linux-x86_64.deb
if echo '5012d064a6e95836db899fec0a2ee2209d2726fae4a79b08c8ceb61049a115cd /tmp/bazel_0.6.1-linux-x86_64.deb' | sha256sum -c -; then
  dpkg -i /tmp/bazel_0.6.1-linux-x86_64.deb
else
  echo 'The Bazel deb does not have the expected SHA256.  Not installing Bazel.'
  exit -1
fi
echo '# END SECTION'
"""

# Bazel test result parsing
cat > ${WORKSPACE}/bazel.parser << DELIM_PARSER
warning /^TIMEOUT: /
DELIM_PARSER
