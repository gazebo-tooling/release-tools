# Information on build.osrfoundation.org nodes

## Jenkins node labels

Jenkins labels are mainly created from chef provisioning of the nodes and mainly
used automatically from DSL code when it generates Jenkins jobs. The buildfarm
probably has more labels than the ones in the table, they are probably not being
used (can check this in the Jenkins UI)

| Label name | Description | Requirements |
| -------- | ----------- | ------------ |
| docker   | Node has capabilities to run Docker CI (standard Linux CI) | Linux system with docker installed |
| gpu-reliable | Node has a real GPU able to run simulation for Gazebo | Nvidia card and nvidia-docker installed on Linux |
| huge-memory  | Node has enough RAM to run really demanding RAM compilations | Hardware has no less than 16Gb of RAM and can run abichecker on ign-physics |
| large-memory | Node has enough RAM to run non trivial compilations | Hardware has no less than 16GB of RAM on Linux |
| linux-arm64 | Node has capabilities to run native arm64 code (mostly used in packaging) | Bare-metal ARM machine |
| linux-armhf | Node has capabilities to run native armhf code (mostly used in packaging) | Bare-metal ARM machine |
| osx | Node has capabilities to run native OsX code | Apple system |
| osx_gazebo | Node has capabilities to run Gazebo classic CI and packaging | 'Powerful' Apple system |
| osx\_$distro | Node has capabilities to build code for the distribution $distro | Apple system running $distro |
| swarm | Node was created using swarm plugin in Jenkins | Chef provisioned node |
| win | Node is able to run Windows CI | Windows10 system |
| win_testing | Node is ready to test a new vcpkg snapshot | Windows10 system |

### Node labels for nightlies

For generating nightlies controlling the generation order of every library, build.osrfoundation.org use the approach of [using a single node to process the whole ignition family for a given Ubuntu distribution](https://github.com/ignition-tooling/release-tools/issues/644). Current assignment of nodes is as follow:

| Label name | Description |
| -------- | ----------- |
| linux-nightly-bionic | r2d2  |
| linux-nightly-focal | optimus |
| linux-nightly-jammy | drogon |

## Provision of Node labels

### Agents

Chef provision of node labels is being done by code in the [osrf_jenkins_agent](https://github.com/osrf/osrf_jenkins_agent/)
repository.

Note: check https://github.com/osrf/chef-osrf/issues/136 to see the status of the automation.
