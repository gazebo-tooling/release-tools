#/bin/bash

echo "# BEGIN SECTION: Install script dependencies"
apt-cyg update
apt-cyg install wget mercurial tar gawk xz bzip2 
echo '# END SECTION'

echo "# BEGIN SECTION: Install tool dependencies"
apt-cyg install python make cmake gdb git patch unzip pkg-config gcc-g++ libtool fluid
echo '# END SECTION'

echo "# BEGIN SECTION: Install library dependencies"
apt-cyg install libpoco-devel libboost-devel libboost_python-devel libGLU-devel \
                libgtk2.0-devel libcurl-devel libjpeg-devel libfltk-devel \
                libX11-devel libXext-devel libfreetype-devel libxml2-devel libqhull-devel
echo '# END SECTION'

echo "# BEGIN SECTION: Prepare script sources"
SCRIPTS_INSTALL_DIR=/opt/rosscripts
# TODO: At his moment the script is located in a cygwin fixed dir
# need to check it out from the repository
CHECKOUT_PATH=${HOME}/ros_cygwin-git

# Cleanup the installs paths
mkdir -p /opt
rm -fr ${SCRIPTS_INSTALL_DIR}

# Copy script repo into /opt
cp -a ${CHECKOUT_PATH}/autobuild ${SCRIPTS_INSTALL_DIR}
echo '# END SECTION'

echo "# BEGIN SECTION: Run ros_build_isolated.bat"
cd ${SCRIPTS_INSTALL_DIR}
chmod +x *.sh 
./build_ros_isolated.sh
echo "# END SECTION"
