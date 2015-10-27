import _configs_.*
import javaposse.jobdsl.dsl.Job

packages = [ 'gazebo', 'sdformat', 'urdfdom', 'urdfdom-headers' ]

packages.each { pkg ->

  def ci_job = job("${pkg}-debian_ci-default-debian_sid-amd64")

  OSRFLinuxBase.create(ci_job)

  ci_job.with
  {
      def git_repo = "git://anonscm.debian.org/debian-science/packages/${pkg}.git"

      scm {
	git("${git_repo}") {
	  branch('master')
	  subdirectory("${pkg}")
	}
      }

      priority 300

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

              export LINUX_DISTRO=debian
              export DISTRO=sid
              export GIT_REPOSITORY="${git_repo}"

              /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-git-debbuild.bash
              """.stripIndent())
      }
  }
}
