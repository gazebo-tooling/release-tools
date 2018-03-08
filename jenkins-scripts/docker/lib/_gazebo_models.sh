GAZEBO_MODEL_INSTALLATION="""
wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz && \
mkdir -p ~/.gazebo/models
tar -xvf /tmp/default.tar.gz -C ~/.gazebo/models --strip 1
rm /tmp/default.tar.gz"""
