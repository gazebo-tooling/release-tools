import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"

// Jobs
def ignition_ci_job = job("_test_job_from_dsl")
OSRFLinuxBase.create(ignition_ci_job)
def ignition_ci_job_mac = job("_test_job_from_dsl_mac")
OSRFOsXBase.create(ignition_ci_job_mac)
def ignition_ci_job_win = job("_test_job_from_dsl_win")
OSRFWinBase.create(ignition_ci_job_win)
def ignition_ci_pr_job = job("_test_pr_job_from_dsl")
OSRFLinuxCompilationAnyGitHub.create(ignition_ci_pr_job,
                                     'gazebosim/testing',
                                     false,
                                     false,
                                     ['main'])

// relative to WORKSPACE
def pkg_sources_dir="pkgs"

def releasepy_job = job("_test_releasepy")
OSRFReleasepy.create(releasepy_job, [DRY_RUN: true])
releasepy_job.with {
      blockOn("_test_repository_uploader") {
        blockLevel('GLOBAL')
        scanQueueFor('ALL')
      }
}

def repo_uploader = job("_test_repository_uploader")
OSRFBase.create(repo_uploader)
repo_uploader.with
{
  label Globals.nontest_label("docker")

  // wrappers {
  //    preBuildCleanup()
  // }

  parameters
  {
    stringParam('PACKAGE','','Package name')
    stringParam('TARBALL_NAME', '', 'Tarball name to upload')
    stringParam('S3_UPLOAD_PATH','', 'S3 path to upload')
    stringParam('UPLOAD_TO_REPO','none','repo to upload')
  }

  steps
  {
    copyArtifacts('_test_gz_source')
    {
      includePatterns("${pkg_sources_dir}/*.tar.*")
      buildSelector {
        upstreamBuild()
      }
    }

    shell("""\
          #!/bin/bash -xe

          # check that the tarball name actually exist

          ls -R \${WORKSPACE}
          test -f \${WORKSPACE}/\${TARBALL_NAME}

          echo "Fake upload of \${TARBALL_NAME} to \${S3_UPLOAD_PATH}"
          """.stripIndent())
  }
}

def gz_source_job = job("_test_gz_source")
OSRFLinuxSourceCreation.create(gz_source_job)
gz_source_job.with
{
  label Globals.nontest_label("docker")

  def package_name="gz-cmake3"
  def canonical_package_name=Globals.get_canonical_package_name(package_name)
  def properties_file="package_name.prop"

  steps {
    shell("""\
          #!/bin/bash -xe

          # Use Jammy/amd64 as base image to generate sources
          export PACKAGE=${package_name}
          export DISTRO=jammy
          export ARCH=amd64
          export SOURCE_REPO_URI=https://github.com/gazebosim/gz-cmake.git

          /bin/bash -x ./scripts/jenkins-scripts/docker/gz-source-generation.bash
          """.stripIndent())

    shell("""\
          #!/bin/bash -xe

          # Export information from the build in properties_files. The tarball extraction helps to
          # deal with changes in the compression of the tarballs.
          tarball=\$(find \${WORKSPACE}/${pkg_sources_dir} \
                       -type f \
                       -name ${canonical_package_name}-\${VERSION}.tar.* \
                       -printf "%f\\n")
          if [[ -z \${tarball} ]] || [[ \$(wc -w <<< \${tarball}) != 1 ]]; then
            echo "Tarball name extraction returned \${tarball} which is not a one word string"
            exit 1
          fi

          echo "PACKAGE=${package_name}" > ${properties_file}
          echo "TARBALL_NAME=\${tarball}" >> ${properties_file}
          """.stripIndent())

  }

  publishers {
    postBuildScripts {
      steps {
        conditionalSteps {
             condition {
              not {
                expression('none|None|^$','${ENV,var="UPLOAD_TO_REPO"}')
                }
              }
              steps {
                // Invoke repository_uploader
                downstreamParameterized {
                  trigger('_test_repository_uploader') {
                    parameters {
                      predefinedProps([PROJECT_NAME_TO_COPY_ARTIFACTS: "\${JOB_NAME}",
                                       S3_UPLOAD_PATH: Globals.s3_upload_tarball_path(package_name),
                                       UPLOAD_TO_REPO: '${UPLOAD_TO_REPO}'])
                      propertiesFile(properties_file)
                                      // PACKAGE
                                      // TARBALL_NAME
                    }
                  }
                }
                downstreamParameterized {
                  trigger('_test_releasepy') {
                    parameters {
                      currentBuild()
                      predefinedProps([PROJECT_NAME_TO_COPY_ARTIFACTS: "\${JOB_NAME}"])
                      propertiesFile(properties_file)
                                      // PACKAGE
                                      // TARBALL_NAME
                    }
                  }
                }
              }
            }
      }
    }
  }
}

// -------------------------------------------------------------------
def outdated_job_runner = job("_test_outdated_job_runner")
OSRFBase.create(outdated_job_runner)
outdated_job_runner.with
{
  label Globals.nontest_label("master")

  steps
  {
    systemGroovyCommand(readFileFromWorkspace('scripts/jenkins-scripts/tools/outdated-job-runner.groovy'))
  }
}
