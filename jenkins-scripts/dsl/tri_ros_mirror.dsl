import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'trusty']
def supported_arches  = [ 'amd64' ]

def snapshot_job = job("tri_ros_mirror-create_ROS_repo_snapshot")
OSRFLinuxBase.create(snapshot_job)
job.with
{
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

      /bin/bash -xe ./scripts/jenkins-scripts/tri_ros_mirror/create_snapshot.bash
      """.stripIndent())
  }

  archiveArtifacts('info/*')
}

def snapshot_job = job("tri_ros_mirror-publish_snapshot")
OSRFLinuxBase.create(snapshot_job)
job.with
{
  steps
  {
    parameters
    {
      stringParam('SNAPSHOT_NAME', '', 'Internal snapshot name to publish as external repo')
    }
   
    shell("""\
      #!/bin/bash -xe

      /bin/bash -xe ./scripts/jenkins-scripts/tri_ros_mirror/create_snapshot.bash
      """.stripIndent())
  }
}

supported_distros.each { distro ->
  supported_arches.each { arch ->
    def snapshot_job = job("tri_ros_mirror-install-${distro}-${arch}")
    OSRFLinuxBase.create(snapshot_job)
    job.with
    {
      steps
      {
        shell("""\
	      #!/bin/bash -xe

	      export DISTRO=${distro}
              export ARCH=${arch}
	      /bin/bash -xe ./scripts/jenkins-scripts/tri_ros_mirror/desktop_full_install.bash
	      """.stripIndent())
      }
    }
  }
}
