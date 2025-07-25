package _configs_

import javaposse.jobdsl.dsl.Job
import _configs_.Globals

class OSRFSourceCreation
{
  static String properties_file = "package_name.prop"

  static void addParameters(Job job, Map default_params = [:])
  {
    job.with
    {
      parameters {
        choiceParam('SOURCE_REPO_URI',
                    [default_params.find{ it.key == "SOURCE_REPO_URI"}?.value],
                    "Software repository URL (can not be modified)")
        stringParam('SOURCE_REPO_REF',
                    default_params.find{ it.key == "SOURCE_REPO_REF"}?.value,
                    "Git branch or tag to build sources from")
        stringParam("VERSION",
                    default_params.find{ it.key == "VERSION"}?.value,
                    "Packages version to be built or nightly (enable nightly build mode)")
        stringParam("OSRF_REPOS_TO_USE",
                    default_params.find{ it.key == "OSRF_REPOS_TO_USE"}?.value,
                    "OSRF repos name to use when building the package")
        stringParam("LINUX_DISTRO",
                    default_params.find{ it.key == "LINUX_DISTRO"}?.value,
                    "Linux distribution to use to generate sources")
        stringParam("DISTRO",
                    default_params.find{ it.key == "DISTRO"}?.value,
                    "Linux release inside LINUX_DISTRO to generate sources on")
        // Not using choiceParam here to support Citadel/Fortress ign-* packages not only
        // gz-* packages
        stringParam('PACKAGE',
                    default_params.find{ it.key == "PACKAGE"}?.value,
                    "For downstream use: Package name")
        stringParam("RELEASE_VERSION",
                    default_params.find{ it.key == "RELEASE_VERSION"}?.value,
                    "For downstream jobs: Packages release version")
        stringParam("RELEASE_REPO_BRANCH",
                    default_params.find{ it.key == "RELEASE_REPO_BRANCH"}?.value,
                    "For downstream jobs: Branch from the -release repo to be used")
        stringParam("UPLOAD_TO_REPO",
                    default_params.find{ it.key == "UPLOAD_TO_REPO"}?.value,
                    "For downstream jobs: OSRF repo name to upload the package to: stable | prerelease | nightly | none (for testing proposes)")
        stringParam("EXTRA_OSRF_REPO",
                    default_params.find{ it.key == "EXTRA_OSRF_REPO"}?.value,
                    "For downstream jobs: OSRF extra repositories to add")
      }
    }
  }

  static void create(Job job, Map default_params = [:], Map default_hidden_params = [:])
  {
    OSRFLinuxBuildPkgBase.create(job)
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

      def s3_download_url_basedir = Globals.s3_download_url_basedir(
        default_params.find{ it.key == "PACKAGE"}?.value)

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

          export ARCH=amd64
          /bin/bash -x ./scripts/jenkins-scripts/docker/gz-source-generation.bash
          """.stripIndent()
        )

        // This can be enabled after the complete deprecation of ign- names
        // that uses ignition aliases.
        // def canonical_package_name = Globals.get_canonical_package_name(
        //  default_params.find{ it.key == "PACKAGE"}.value)
        //
        // then improve the find command below with:
        //  -name ${canonical_package_name}-\${VERSION}.tar.* \
        shell("""\
          #!/bin/bash -xe

          # Export information from the build in properties_files. The tarball extraction helps to
          # deal with changes in the compression of the tarballs.
          tarball=\$(find \${WORKSPACE}/${pkg_sources_dir} \
                       -type f \
                       -name *-\${VERSION}.tar.* \
                       -printf "%f\\n")
          if [[ -z \${tarball} ]] || [[ \$(wc -w <<< \${tarball}) != 1 ]]; then
            # There is one use case that can be valid but failed to find canonical_package_name
            # which are package using _ (underscores) in their name like gz-fuel_tools. Workaround
            # is to just search for the VERSION (not so safety but should work)
            tarball=\$(find \${WORKSPACE}/${pkg_sources_dir} \
                         -type f \
                         -name *-\${VERSION}.tar.* \
                         -printf "%f\\n")
            if [[ -z \${tarball} ]] || [[ \$(wc -w <<< \${tarball}) != 1 ]]; then
              echo "Tarball name extraction returned \${tarball} which is not a one word string"
              exit 1
            fi
          fi

          SOURCE_TARBALL_SHA256=\$(sha256sum \${WORKSPACE}/${pkg_sources_dir}/\${tarball} | awk '{print \$1}')
          echo tarball sha256 is \${SOURCE_TARBALL_SHA256}

          echo "S3_FILES_TO_UPLOAD=\${tarball}" >> ${properties_file}
          echo "SOURCE_TARBALL_URI=$s3_download_url_basedir/\${tarball}" >> ${properties_file}
          echo "SOURCE_TARBALL_SHA256=\${SOURCE_TARBALL_SHA256}" >> ${properties_file}
          """.stripIndent()
        )
      }
    }
  }

  // Useful to inject testing jobs
  static void call_uploader_and_releasepy(Job job,
                                          String package_name,
                                          String repository_uploader_jobname,
                                          String releasepy_jobname)
  {
    job.with
    {
      // This creates a post-build script that conditionally triggers downstream jobs
      // (repository uploader and release.py) only when UPLOAD_TO_REPO is not 'none', 'None', or empty
      configure { project ->
        project / 'publishers' / 'org.jenkinsci.plugins.postbuildscript.PostBuildScript' {
          config {
            scriptFiles()
            groovyScripts()
            buildSteps {
              'org.jenkinsci.plugins.postbuildscript.model.PostBuildStep' {
                results {
                  string('SUCCESS')
                }
                role('BOTH')
                executeOn('BOTH')
                buildSteps {
                  'org.jenkinsci.plugins.conditionalbuildstep.ConditionalBuilder' {
                    runner(class: 'org.jenkins_ci.plugins.run_condition.BuildStepRunner$Fail')
                    runCondition(class: 'org.jenkins_ci.plugins.run_condition.core.AlwaysRun')
                    conditionalbuilders {
                      'hudson.plugins.parameterizedtrigger.TriggerBuilder' {
                        configs {
                          'hudson.plugins.parameterizedtrigger.BlockableBuildTriggerConfig' {
                            configs {
                              'hudson.plugins.parameterizedtrigger.CurrentBuildParameters'()
                              'hudson.plugins.parameterizedtrigger.FileBuildParameters' {
                                propertiesFile(properties_file)
                                failTriggerOnMissing('false')
                                textParamValueOnNewLine('false')
                                useMatrixChild('false')
                                onlyExactRuns('false')
                              }
                              'hudson.plugins.parameterizedtrigger.PredefinedBuildParameters' {
                                properties("""
                                  PROJECT_NAME_TO_COPY_ARTIFACTS=\${JOB_NAME}
                                  PACKAGE_ALIAS=\${PACKAGE_ALIAS}
                                  S3_UPLOAD_PATH=${Globals.s3_releases_dir(package_name)}
                                  """.stripIndent())
                                textParamValueOnNewLine('false')
                              }
                            }
                            projects(repository_uploader_jobname)
                            condition('ALWAYS')
                            triggerWithNoParameters('false')
                            triggerFromChildProjects('false')
                            buildAllNodesWithLabel('false')
                          }
                        }
                      }
                      'hudson.plugins.parameterizedtrigger.TriggerBuilder' {
                        configs {
                          'hudson.plugins.parameterizedtrigger.BlockableBuildTriggerConfig' {
                            configs {
                              'hudson.plugins.parameterizedtrigger.CurrentBuildParameters'()
                              'hudson.plugins.parameterizedtrigger.FileBuildParameters' {
                                propertiesFile(properties_file)
                                failTriggerOnMissing('false')
                                textParamValueOnNewLine('false')
                                useMatrixChild('false')
                                onlyExactRuns('false')
                              }
                              'hudson.plugins.parameterizedtrigger.PredefinedBuildParameters' {
                                properties('PROJECT_NAME_TO_COPY_ARTIFACTS=${JOB_NAME}')
                                textParamValueOnNewLine('false')
                              }
                            }
                            projects(releasepy_jobname)
                            condition('ALWAYS')
                            triggerWithNoParameters('false')
                            triggerFromChildProjects('false')
                            buildAllNodesWithLabel('false')
                          }
                        }
                      }
                    }
                  }
                }
                stopOnFailure('false')
              }
            }
            markBuildUnstable('false')
          }
        }
      }
    }
  }
}
