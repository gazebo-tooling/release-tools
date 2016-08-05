import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'xenial' ]

// LINUX
supported_distros.each { distro ->
  def build_pkg_job = job("uctf-debbuilder")

  build_pkg_job.with
  {
    scm {
      git {
        remote {
	  github('osrf/uctf-debian', 'http')
	  credentials('923bb60c-535f-4fb9-8768-44d4d0cd11dd')
        }

        extensions {
	  cleanBeforeCheckout()
	  relativeTargetDirectory('repo')
	  submoduleOptions {
	    recursive(true)
	  }
        }
      }
    }

    logRotator {
      artifactNumToKeep(10)
    }

    concurrentBuild(true)

    throttleConcurrentBuilds {
      maxPerNode(1)
      maxTotal(5)
    }

    parameters {
      textParam("RELEASE_VERSION", null, "osrfX, OSRF postix version")
      textParam("RELEASE_ARCH_VERSION", null, "~ARCH-X, release version")
    }

    steps {
      shell("""\
            #!/bin/bash -xe

            export LINUX_DISTRO=ubuntu
            export DISTRO=xenial
            export USE_ROS_REPO=true

            /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-git-debbuild.bash
            """.stripIndent())
    }

    publishers
    {
      publishers {
        archiveArtifacts('pkgs/*')
      }
    }
  }
}
