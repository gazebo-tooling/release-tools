import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'xenial' ]
def supported_arches = [ 'amd64' ]

supported_distros.each { distro ->
  supported_arches.each { arch ->
      // --------------------------------------------------------------
      // 1. Create the default ci jobs
      def ardupilot_ci_job = job("gazebo_ardupilot-ci-default-${distro}-${arch}")
      OSRFLinuxCompilation.create(ardupilot_ci_job, false, false)

      ardupilot_ci_job.with
      {
	  label "gpu-reliable-${distro}"

	  scm {
            git {
              remote {
                github('iche033/ardupilot', 'https')
	        branch('gazebo_sitl_irlock')
              }

              extensions {
                cleanBeforeCheckout()
                relativeTargetDirectory('ardupilot')
                submoduleOptions {
                  recursive(true)
                }
	      }
	    }
          }

	  triggers {
	    scm('*/5 * * * *')
	  }

	  steps {
	    shell("""\
		  #!/bin/bash -xe

		  export DISTRO=${distro}
		  export ARCH=${arch}

		  /bin/bash -xe ./scripts/jenkins-scripts/docker/ardupilot-compilation.bash
		  """.stripIndent())
	  }
      }
  }
}

