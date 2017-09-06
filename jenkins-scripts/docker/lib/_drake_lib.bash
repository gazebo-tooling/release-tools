DRAKE_BAZEL_INSTALL="""
echo '# BEGIN SECTION: install bazel'
echo \"deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8\" | tee /etc/apt/sources.list.d/bazel.list
curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
apt-get update
apt-get install -o Dpkg::Options::=\"--force-overwrite\" -y openjdk-8-jdk bazel
echo '# END SECTION'
"""
