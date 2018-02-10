import _configs_.*
import javaposse.jobdsl.dsl.Job


/* Create CI jobs for servicesim https://bitbucket.org/osrf/servicesim */

def ci_distro = [ 'xenial' ]
def supported_arches = [ 'amd64' ]

// ## Method adapted from srcsim.dsl
void include_gpu_label(Job job, String distro) {
  job.with {
    // Early testing shows that xenial jobs can be run on a
    // trusty host with good results.
    if (distro == 'xenial')
      label "gpu-reliable-${distro} || gpu-reliable-trusty"
    else
      label "gpu-reliable-${distro}"
  }
}

// ## Method copied from srcsim.dsl
void include_parselog(Job job) {
  job.with {
    publishers {
      consoleParsing {
        globalRules('/var/lib/jenkins/logparser_error_on_roslaunch_failed')
        failBuildOnError()
      }
    }
  }
}

// Add servicesim compilation script to job
void include_compilation_script_step(Job job) {
  job.with {
    steps {
      shell("""
            #!/bin/bash -xe
           export DISTRO=${distro}
           export ARCH=${arch}

           /bin/bash -xe ./scripts/jenkins-scripts/docker/servicesim-compilation.bash
           """.stripIndent())
    }
  }
}

// MAIN CI JOBS
ci_distro.each { distro ->
  supported_arches.each { arch ->
    // 1. Create default branch jobs
    def servicesim_ci_job = job("servicesim-ci-${distro}-${arch}")
    OSRFLinuxCompilation.create(servicesim_ci_job)
    // GPU label and parselog
    include_gpu_label(servicesim_ci_job, distro)
    include_parselog(servicesim_ci_job)

    servicesim_ci_job.with {
      scm {
        hg('https://bitbucket.org/osrf/servicesim') {
          branch('default')
          subdir('servicesim') // Steven!: Is this the repository directory name?
        }
      }

      triggers {
        scm('*/5 * * * *')
      }
    }
    include_compilation_script_step(servicesim_ci_job)


    // 2. Create pull request jobs
    def servicesim_ci_any_job = job("servicesim-ci-pr-any-${distro}-${arch}")
    OSRFLinuxCompilationAny.create(servicesim_ci_any_job,
                                   'https://bitbucket.org/osrf/servicesim')
    // GPU label and parselog
    include_gpu_label(servicesim_ci_job, distro)
    include_parselog(servicesim_ci_job)

    include_compilation_script_step(servicesim_ci_any_job)
  } // end: supported_arches
} // end: ci_distro

