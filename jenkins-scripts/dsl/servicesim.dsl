import _configs_.*
import javaposse.jobdsl.dsl.Job


/* Create CI jobs for servicesim https://bitbucket.org/osrf/servicesim */

def ci_distro = [ 'xenial' ]
def supported_arches = [ 'amd64' ]
def servicesim_packages = [
  'rqt_servicebot_pan_tilt',
  'rqt_servicesim_score',
  'servicebot_2dnav',
  'servicebot_control',
  'servicebot_description',
  'servicesim',
  'servicesim_competition',
  'servicesim_example_python_solution'
]

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
void include_compilation_script_step(Job job, distro, arch) {
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
    // enable testing, disable cppcheck (for now)
    OSRFLinuxCompilation.create(servicesim_ci_job, true, false)
    include_parselog(servicesim_ci_job)

    servicesim_ci_job.with {
      label "gpu-reliable"

      scm {
        hg('https://bitbucket.org/osrf/servicesim') {
          branch('default')
          subdirectory('servicesim')
        }
      }

      triggers {
        scm('*/5 * * * *')
      }
    }
    include_compilation_script_step(servicesim_ci_job, distro, arch)


    // 2. Create pull request jobs
    def servicesim_ci_any_job = job("servicesim-ci-pr-any-${distro}-${arch}")
    // enable testing, disable cppcheck (for now)
    OSRFLinuxCompilationAny.create(servicesim_ci_any_job,
                                   'https://bitbucket.org/osrf/servicesim',
                                   true, false)
    include_parselog(servicesim_ci_job)
    servicesim_ci_any_job.with {
      label "gpu-reliable"
    }

    include_compilation_script_step(servicesim_ci_any_job, distro, arch)
  } // end: supported_arches
} // end: ci_distro


// BLOOM PACKAGE BUILDER JOBS
servicesim_packages.each { pkg ->
  pkg_dashed = pkg.replaceAll("_", "-")

  def build_pkg_job = job("$pkg_dashed-bloom-debbuilder")

  // Use the linux install as base
  OSRFLinuxBuildPkgBase.create(build_pkg_job)
  GenericRemoteToken.create(build_pkg_job)

  build_pkg_job.with
  {
    properties {
      priority 100
    }

    parameters {
      stringParam("PACKAGE","$pkg_dashed","Package name to be built")
        stringParam("VERSION",null,"Packages version to be built")
        stringParam("RELEASE_VERSION", null, "Packages release version")
        stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
        stringParam("DISTRO", "xenial", "Linux release inside LINUX_DISTRO to build packages for")
        stringParam("ARCH", "amd64", "Architecture to build packages for")
        stringParam('ROS_DISTRO', 'kinetic','ROS DISTRO to build pakcages for')
        stringParam("UPLOAD_TO_REPO", 'stable', "OSRF repo name to upload the package to")
        stringParam('UPSTREAM_RELEASE_REPO', 'https://bitbucket.org/osrf/servicesim-release', 'Release repository url')
    }

    steps {
      systemGroovyCommand("""\
          build.setDescription(
            '<b>' + build.buildVariableResolver.resolve('ROS_DISTRO') + '-'
            + build.buildVariableResolver.resolve('VERSION') + '-'
            + build.buildVariableResolver.resolve('RELEASE_VERSION') + '</b>' +
            '(' + build.buildVariableResolver.resolve('LINUX_DISTRO') + '/' +
            build.buildVariableResolver.resolve('DISTRO') + '::' +
            build.buildVariableResolver.resolve('ARCH') + ')' +
            '<br />' +
            'upload to: ' + build.buildVariableResolver.resolve('UPLOAD_TO_REPO') +
            '<br />' +
            'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
          )
    }

    publishers {
      downstreamParameterized {
        trigger('repository_uploader_ng') {
          condition('SUCCESS')
            parameters {
              currentBuild()
                predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
                predefinedProp("PACKAGE_ALIAS", "\${JOB_NAME}")
            }
        }
      }
    }


    steps {
      shell("""\
          #!/bin/bash -xe

          export OSRF_REPOS_TO_USE='stable prerelease'

          /bin/bash -x ./scripts/jenkins-scripts/docker/bloom-debbuild.bash
          """.stripIndent())
    }
  }
}
