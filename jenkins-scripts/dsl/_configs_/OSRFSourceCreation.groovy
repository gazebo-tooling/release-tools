package _configs_

import javaposse.jobdsl.dsl.Job
import _configs_.Globals

class OSRFSourceCreation
{
  static String properties_file = "package_name.prop"
  static String package_name = ""

  static void addParameters(Job job, Map default_params = [:])
  {
    package_name = default_params.find{ it.key == "PACKAGE"}?.value

    job.with
    {
      parameters {
        choiceParam('PACKAGE',
                    [default_params.find{ it.key == "PACKAGE"}?.value],
                    "Package name (can not be modified)")
        choiceParam('SOURCE_REPO_URI',
                    [default_params.find{ it.key == "SOURCE_REPO_URI"}?.value],
                    "Software repository URL (can not be modified)")
        stringParam("VERSION",
                    default_params.find{ it.key == "VERSION"}?.value,
                    "Packages version to be built or nightly (enable nightly build mode)")
        stringParam("RELEASE_VERSION",
                    default_params.find{ it.key == "RELEASE_VERSION"}?.value,
                    "For downstream jobs: Packages release version")
        stringParam("RELEASE_REPO_BRANCH",
                    default_params.find{ it.key == "RELEASE_REPO_BRANCH"}?.value,
                    "For downstream jobs: Branch from the -release repo to be used")
        stringParam("UPLOAD_TO_REPO",
                    default_params.find{ it.key == "UPLOAD_TO_REPO"}?.value,
                    "For downstream jobs: OSRF repo name to upload the package to: stable | prerelease | nightly | none (for testing proposes)")
      }
    }
  }

  static void create(Job job, Map default_params = [:], Map default_hidden_params = [:])
  {
    OSRFLinuxBuildPkgBase.create(job)
    GenericRemoteToken.create(job)
    OSRFSourceCreation.addParameters(job, default_params)

    def pkg_sources_dir="pkgs"

    job.with
    {
      wrappers {
        preBuildCleanup()
      }

      properties {
        priority 100
      }

      def canonical_package_name = Globals.get_canonical_package_name(
        default_params.find{ it.key == "PACKAGE"}.value)

      steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b>' + build.buildVariableResolver.resolve('VERSION') + '-' +
          build.buildVariableResolver.resolve('RELEASE_VERSION') + '</b>' +
          '<br />' +
          'branch: ' + build.buildVariableResolver.resolve('RELEASE_REPO_BRANCH') + ' | ' +
          'upload to: ' + build.buildVariableResolver.resolve('UPLOAD_TO_REPO') +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )
        shell("""\
          #!/bin/bash -xe

          # Use Jammy/amd64 as base image to generate sources
          export DISTRO=jammy
          export ARCH=amd64

          /bin/bash -x ./scripts/jenkins-scripts/docker/gz-source-generation.bash
          """.stripIndent()
        )
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

          echo "TARBALL_NAME=\${tarball}" >> ${properties_file}
          """.stripIndent()
        )
      }
    }
  }

  // Useful to inject testing jobs
  static void call_uploader_and_releasepy(Job job,
                                          String repository_uploader_jobname,
                                          String releasepy_jobname)
  {
    job.with
    {
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
                  trigger(repository_uploader_jobname) {
                    parameters {
                      currentBuild()
                      predefinedProps([PROJECT_NAME_TO_COPY_ARTIFACTS: '${JOB_NAME}',
                                       S3_UPLOAD_PATH: Globals.s3_upload_tarball_path(package_name)])
                      propertiesFile(properties_file) // TARBALL_NAME
                    }
                  }
                }
                downstreamParameterized {
                  trigger(releasepy_jobname) {
                    parameters {
                      currentBuild()
                      predefinedProps([PROJECT_NAME_TO_COPY_ARTIFACTS: "\${JOB_NAME}"])
                      propertiesFile(properties_file) // TARBALL_NAME
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}
