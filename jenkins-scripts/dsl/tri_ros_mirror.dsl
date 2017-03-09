import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'trusty']
def supported_arches  = [ 'amd64' ]

def snapshot_job = job("tri_ros_mirror-create_snapshot")
OSRFLinuxBase.create(snapshot_job)
snapshot_job.with
{
  label "tri_ros_mirror.trusty"

  wrappers {
    preBuildCleanup {
       includePattern('info/*')
      // the sudo does not seems to be able to remove root owned packaged
      deleteCommand('sudo rm -rf %s')
    }
  }

  parameters
  {
    stringParam('SNAPSHOT_TAG', '', 'Optional tag to identify the snapshot')
  }
         
  steps
  {
    shell("""\
      #!/bin/bash -xe

      /bin/bash -xe ./scripts/jenkins-scripts/docker/tri_ros_mirror_create_snapshot.bash
      """.stripIndent())
  }

  publishers
  {
    archiveArtifacts('info/*')
  }
}

def snapshot_publish_job = job("tri_ros_mirror-publish_snapshot")
OSRFLinuxBase.create(snapshot_publish_job)
snapshot_publish_job.with
{
  steps
  {

    label "tri_ros_mirror.trusty"

    parameters
    {
      stringParam('SNAPSHOT_NAME', '', 'Internal snapshot name to publish as external repo')
    }
   
    shell("""\
      #!/bin/bash -xe

      /bin/bash -xe ./scripts/jenkins-scripts/docker/tri_ros_mirror_publish_snapshot.bash
      """.stripIndent())
  }

  publishers
  {
    archiveArtifacts('info/*')
  }
}

supported_distros.each { distro ->
  supported_arches.each { arch ->
    def install_job = job("tri_ros_mirror-install-${distro}-${arch}")
    OSRFLinuxBase.create(install_job)
    install_job.with
    {
      steps
      {
        shell("""\
	      #!/bin/bash -xe

	      export DISTRO=${distro}
              export ARCH=${arch}
	      /bin/bash -xe ./scripts/jenkins-scripts/docker/tri_ros_mirror_desktop_full_install.bash
	      """.stripIndent())
      }
    }
  }
}
