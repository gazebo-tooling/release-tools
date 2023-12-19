import _configs_.*
import javaposse.jobdsl.dsl.Job
// If failed to import locally be sure of using tools/ scripts
import org.yaml.snakeyaml.Yaml

// GZ COLLECTIONS
arch = 'amd64'

// Jenkins needs the relative path to work and locally the simulation is done
// using a symlink
file = readFileFromWorkspace("scripts/jenkins-scripts/dsl/gz-collections.yaml")
gz_collections_yaml = new Yaml().load(file)

gz_nightly = 'ionic'

String get_debbuilder_name(parsed_yaml_lib, parsed_yaml_packaging)
{
  major_version = parsed_yaml_lib.major_version

  ignore_major_version = parsed_yaml_packaging.linux?.ignore_major_version
  if (ignore_major_version && ignore_major_version.contains(parsed_yaml_lib.name))
    major_version = ""

  return parsed_yaml_lib.name + major_version + "-debbuilder"
}

def DISABLE_TESTS           = false

void generate_install_job(prefix, gz_collection_name, distro, arch)
{
  def install_default_job = job("${prefix}_${gz_collection_name}-install-pkg-${distro}-${arch}")
  OSRFLinuxInstall.create(install_default_job)

  install_default_job.with
  {
    triggers {
      cron(Globals.CRON_EVERY_THREE_DAYS)
    }

    def dev_package = "${prefix}-${gz_collection_name}"
    def job_name = 'gz_launch-install-test-job.bash'

    label Globals.nontest_label("gpu-reliable")

    steps {
     shell("""\
           #!/bin/bash -xe

           export DISTRO=${distro}
           export ARCH=${arch}
           export INSTALL_JOB_PKG=${dev_package}
           export GZDEV_PROJECT_NAME="${dev_package}"
           if [[ ${gz_collection_name} == 'citadel' || ${gz_collection_name} == 'fortress' ]]; then
              export GZ_SIM_RUNTIME_TEST_USE_IGN=true
           fi
           /bin/bash -x ./scripts/jenkins-scripts/docker/${job_name}
           """.stripIndent())
    }
  }
}

// Testing compilation from source
gz_collections_yaml.collections.each { collection ->
  gz_collection_name = collection.name

  if (! collection.packaging.exclude?.contains(gz_collection_name)) {
    // DEBBUILD: linux package builder
    // --------------------------------------------------------------
    def build_pkg_job = job("gz-${gz_collection_name}-debbuilder")
    OSRFLinuxBuildPkg.create(build_pkg_job)
    build_pkg_job.with
    {
      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-ignition-debbuild.bash
              """.stripIndent())
      }
    }
  }


  collection.ci.configs.each { ci_config_name ->
    ci_config = gz_collections_yaml.ci_configs.find { it.name == ci_config_name }
    distro = ci_config.system.version
    arch = ci_config.system.arch

    // This should really include these two conditions to be effective:
    //   ci_config.exclude.all?.contains("gz-" + gz_collection_name)
    //   ci_config.exclude.all?.contains("ign-" + gz_collection_name)
    // However the collection package being processed here are superseed by the migration
    // to yaml efforts. Remove from here when -install- and -win- jobs are implemented
    // in gazebo_libs
    if (ci_config.exclude.all?.contains(gz_collection_name) ||
        ci_config.system.so == 'darwin' ||
        ci_config.system.so == 'windows')
      return

    // INSTALL JOBS:
    // --------------------------------------------------------------
    if ((gz_collection_name == "citadel") || (gz_collection_name == "fortress")) {
      generate_install_job('ignition', gz_collection_name, distro, arch)
    }
    generate_install_job('gz', gz_collection_name, distro, arch)

    // ROS BOOTSTRAP INSTALL JOBS:
    // --------------------------------------------------------------
    def install_ros_bootstrap_job =
    job("ignition_${gz_collection_name}-install-pkg_ros_bootstrap-any-manual")
    OSRFLinuxInstall.create(install_ros_bootstrap_job)
    install_ros_bootstrap_job.with
    {
      parameters {
        stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
        stringParam("DISTRO", distro, "Linux release inside LINUX_DISTRO to build packages for")
        stringParam("ARCH", arch, "Architecture to build packages for")
        labelParam('JENKINS_NODE_TAG') {
          description('Jenkins node or group to run the build on')
          defaultValue('gpu-reliable')
        }
      }

      // Designed to be run manually. No triggers.
      label Globals.nontest_label("gpu-reliable")

      steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b>' + build.buildVariableResolver.resolve('LINUX_DISTRO') + '/' +
                  build.buildVariableResolver.resolve('DISTRO') + '::' +
                  build.buildVariableResolver.resolve('ARCH') + '</b>' +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent())

        shell("""\
             #!/bin/bash -xe


             export INSTALL_JOB_PKG=ignition-${gz_collection_name}
             export USE_ROS_REPO=true
             export ROS_BOOTSTRAP=true
             # needed for arm64 machines and other arch tests
             export ENABLE_GZ_SIM_RUNTIME_TEST=false
             if [[ ${gz_collection_name} == 'citadel' || ${gz_collection_name} == 'fortress' ]]; then
                export GZ_SIM_RUNTIME_TEST_USE_IGN=true
             fi
             if [[ \${JENKINS_NODE_TAG} == 'gpu-reliable' ]]; then
               export ENABLE_GZ_SIM_RUNTIME_TEST=true
             fi
             /bin/bash -x ./scripts/jenkins-scripts/docker/gz_launch-install-test-job.bash
             """.stripIndent())
      }
    }

    // COLCON - Windows
    Globals.gazebodistro_branch = true
    def gz_win_ci_job = job("ign_${gz_collection_name}-ci-win")
    OSRFWinCompilation.create(gz_win_ci_job, false)
    gz_win_ci_job.with
    {
        steps {
          batchFile("""\
                set IGNITION_COLLECTION=${gz_collection_name}
                call "./scripts/jenkins-scripts/lib/ign_collection-base.bat"
                """.stripIndent())
        }
    }
    Globals.gazebodistro_branch = false

    // MAC Brew CI job
    // --------------------------------------------------------------
    def gz_brew_ci_job = job("ignition_${gz_collection_name}-ci-main-homebrew-amd64")
    OSRFBrewCompilation.create(gz_brew_ci_job, DISABLE_TESTS)
    OSRFGitHub.create(gz_brew_ci_job,
                      "gazebosim/gz-${gz_collection_name}",
                      "main",
                      "ign-${gz_collection_name}")
    gz_brew_ci_job.with
    {
        steps {
          shell("""\
                #!/bin/bash -xe

                /bin/bash -xe
                "./scripts/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash" "gz-${gz_collection_name}"
                """.stripIndent())
        }
    }

    // MAC Brew bottle install job
    // --------------------------------------------------------------
    def gz_brew_install_bottle_job = job("ignition_${gz_collection_name}-install_bottle-homebrew-amd64")
    OSRFBrewInstall.create(gz_brew_install_bottle_job)

    gz_brew_install_bottle_job.with
    {
      triggers {
        cron('@daily')
      }

      def bottle_name = "ignition-${gz_collection_name}"

      steps {
       shell("""\
             #!/bin/bash -xe

             /bin/bash -x ./scripts/jenkins-scripts/lib/project-install-homebrew.bash ${bottle_name}
             """.stripIndent())
      }

      publishers
      {
        configure { project ->
          project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
              unstableOnWarning true
              failBuildOnError false
              parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
          }
        }
      }
    }
  }
}

// NIGHTLY GENERATION
def get_nightly_branch(nightly_collection, lib)
{
  return nightly_collection.libs.find { it.name == lib }.repo.current_branch
}

nightly_collection = gz_collections_yaml.collections
  .find { it.name == gz_nightly }

def nightly_scheduler_job = job("ignition-${gz_nightly}-nightly-scheduler")
OSRFUNIXBase.create(nightly_scheduler_job)

nightly_scheduler_job.with
{
  label Globals.nontest_label("master")

  parameters
  {
     stringParam('NIGHTLY_PACKAGES',
                nightly_collection.libs.collect{
                  get_debbuilder_name(it,nightly_collection.packaging)
                    .replace("-debbuilder","")
                }.join(" "),
                'space separated list of packages to build')

     booleanParam('DRY_RUN',false,
                  'run a testing run with no effects')
  }

  triggers {
     cron(Globals.CRON_START_NIGHTLY)
  }

  cmake_branch = get_nightly_branch(nightly_collection, 'gz-cmake')
  common_branch = get_nightly_branch(nightly_collection, 'gz-common')
  fuel_tools_branch = get_nightly_branch(nightly_collection, 'gz-fuel-tools')
  sim_branch = get_nightly_branch(nightly_collection, 'gz-sim')
  gui_branch = get_nightly_branch(nightly_collection, 'gz-gui')
  launch_branch = get_nightly_branch(nightly_collection, 'gz-launch')
  math_branch = get_nightly_branch(nightly_collection, 'gz-math')
  msgs_branch =  get_nightly_branch(nightly_collection, 'gz-msgs')
  physics_branch = get_nightly_branch(nightly_collection, 'gz-physics')
  plugin_branch = get_nightly_branch(nightly_collection, 'gz-plugin')
  rendering_branch = get_nightly_branch(nightly_collection, 'gz-rendering')
  sensors_branch = get_nightly_branch(nightly_collection, 'gz-sensors')
  sdformat_branch = get_nightly_branch(nightly_collection, 'sdformat')
  tools_branch = get_nightly_branch(nightly_collection, 'gz-tools')
  transport_branch = get_nightly_branch(nightly_collection, 'gz-transport')
  utils_branch = get_nightly_branch(nightly_collection, 'gz-utils')

  steps {
    shell("""\
          #!/bin/bash -xe
          set +x # keep password secret
          PASS=\$(cat \$HOME/build_pass)

          dry_run_str=""
          if \$DRY_RUN; then
            dry_run_str="--dry-run"
          fi

          # redirect to not display the password
          for n in \${NIGHTLY_PACKAGES}; do

              if [[ "\${n}" != "\${n/cmake/}" ]]; then
                src_branch="${cmake_branch}"
              elif [[ "\${n}" != "\${n/common/}" ]]; then
                src_branch="${common_branch}"
              elif [[ "\${n}" != "\${n/fuel-tools/}" ]]; then
                src_branch="${fuel_tools_branch}"
              elif  [[ "\${n}" != "\${n/sim/}" ]]; then
                src_branch="${sim_branch}"
              elif  [[ "\${n}" != "\${n/gui/}" ]]; then
                src_branch="${gui_branch}"
              elif [[ "\${n}" != "\${n/launch/}" ]]; then
                src_branch="${launch_branch}"
              elif [[ "\${n}" != "\${n/math/}" ]]; then
                src_branch="${math_branch}"
              elif [[ "\${n}" != "\${n/msgs/}" ]]; then
                src_branch="${msgs_branch}"
              elif [[ "\${n}" != "\${n/physics/}" ]]; then
                src_branch="${physics_branch}"
              elif [[ "\${n}" != "\${n/plugin/}" ]]; then
                src_branch="${plugin_branch}"
              elif [[ "\${n}" != "\${n/rendering/}" ]]; then
                src_branch="${rendering_branch}"
              elif [[ "\${n}" != "\${n/sensors/}" ]]; then
                src_branch="${sensors_branch}"
              elif [[ "\${n}" != "\${n/sdformat/}" ]]; then
                src_branch="${sdformat_branch}"
              elif  [[ "\${n}" != "\${n/sim/}" ]]; then
                src_branch="${sim_branch}"
              elif [[ "\${n}" != "\${n/transport/}" ]]; then
                src_branch="${transport_branch}"
              elif [[ "\${n}" != "\${n/tools/}" ]]; then
                src_branch="${tools_branch}"
              elif [[ "\${n}" != "\${n/utils/}" ]]; then
                src_branch="${utils_branch}"
              else
                src_branch="main"
              fi

              echo "releasing \${n} (from branch \${src_branch})"
              python3 ./scripts/release.py \${dry_run_str} "\${n}" nightly "\${PASS}" --release-repo-branch main --nightly-src-branch \${src_branch} --upload-to-repo nightly > log || echo "MARK_AS_UNSTABLE"
              echo " - done"
          done

          """.stripIndent())
  }

  publishers
  {
     configure { project ->
       project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
          unstableOnWarning true
          failBuildOnError false
          parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
        }
     }
  }
}
