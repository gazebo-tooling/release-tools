GAZEBO_MODEL_INSTALLATION="""
curl http://example.com --output /tmp/default.tar.gz
mkdir -p ~/.gazebo/models
tar -xvf /tmp/default.tar.gz -C ~/.gazebo/models --strip 1
rm /tmp/default.tar.gz"""
