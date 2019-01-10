import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION PACKAGES
ignition_software = [ 'cmake',
                      'common',
                      'fuel-tools',
                      'gazebo',
                      'gui',
                      'math',
                      'msgs',
                      'physics',
                      'plugin',
                      'rendering',
                      'rndf',
                      'sensors',
                      'tools',
                      'transport' ]
// DESC: need gpu/display for tests
ignition_gpu                = [ 'gazebo',
                                'gui',
                                'rendering',
                                'sensors' ]
// DESC: software does not have tests
ignition_no_test            = [ 'tools' ]
// DESC: major series supported and released. The branches get CI, install pkg
// testing and debbuild job.
// No branches in ignition_branches means no released branches (only CI on
// default, ABI check, install pkg)
ignition_branches           = [ 'common'     : [ '1' ],
                                'fuel-tools' : [ '1' ],
                                'math'       : [ '2', '3','4' ],
                                'msgs'       : [ '1' ],
                                'plugin'     : [ '0' ],
                                'transport'  : [ '3','4' ]]
// DESC: prerelease branches are managed as any other supported branches for
// special cases different to major branches: get compilation CI on the branch
// physics/sensors don't need to be included since they use default for gz11
ignition_prerelease_branches = [ 'cmake'     : [ 'gz11' ],
                                 'common'    : [ 'gz11' ],
                                 'gui'       : [ 'gz11' ],
                                 'math'      : [ 'gz11' ],
                                 'msgs'      : [ 'gz11' ],
                                 'plugin'    : [ 'ign-plugin1' ],
                                 'rendering' : [ 'gz11' ],
                                 'transport' : [ 'gz11' ]]
// DESC: versioned names to generate debbuild jobs for special cases that
// don't appear in ignition_branches
ignition_debbuild  = ignition_software + [ 'cmake1','cmake2',
                                           'common2',
                                           'math5',
                                           'msgs0', 'msgs2',
                                           'transport5' ]
// DESC: exclude ignition from generate any install testing job
ignition_no_pkg_yet         = [ 'gazebo',
                                'gui',
                                'physics',
                                'plugin',
                                'rndf',
                                'sensors' ]
// DESC: major versions that has a package in the prerelease repo. Should
// not appear in ignition_no_pkg_yet nor in ignition_branches
ignition_prerelease_pkgs    = [ 'cmake'  : [
                                   '1' : [ 'bionic', 'xenial' ],
                                   '2' : [ 'bionic' ],
                                ],
                                'common' : [
                                   '2' : [ 'bionic', 'xenial' ],
                                   '3' : [ 'bionic' ],
                                ],
                                'gui'    : [
                                   '1':  [ 'bionic' ],
                                ],
                                'math'   : [
                                   '5':  [ 'bionic', 'xenial' ],
                                   '6':  [ 'bionic' ],
                                ],
                                'msgs'   : [
                                   '2':  [ 'bionic', 'xenial' ],
                                   '3':  [ 'bionic' ],
                                ],
                                'rendering' : [
                                   '1': [ 'bionic' ],
                                ],
                                'transport' : [
                                   '5': [ 'bionic', 'xenial' ],
                                   '6': [ 'bionic' ],
                                ]]
// packages using colcon for windows compilation while migrating all them to
// this solution
ignition_colcon_win         = [ 'gui', 'physics', 'rendering', 'sensors' ]

// Main platform using for quick CI
def ci_distro               = Globals.get_ci_distro()
def abi_distro              = Globals.get_abi_distro()
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = Globals.get_other_supported_distros()
def supported_arches        = Globals.get_supported_arches()

all_supported_distros = ci_distro + other_supported_distros

// Map needed to be used in ci_pr
abi_job_names = [:]

Globals.extra_emails = "caguero@osrfoundation.org"

String ci_distro_str = ci_distro[0]

// Map of lists to use in CIWorkflow
ci_pr_any_list = [:]
ignition_software.each { ign_sw ->
  def list_empty = []
  ci_pr_any_list[ign_sw] = list_empty
}

/**
 * Deeply merges the contents of each Map in sources, merging from
 * "right to left" and returning the merged Map.
 *
 * The source maps will not be modified.
 *
 * Original source code: https://gist.github.com/robhruska/4612278
 */
Map merge_maps(Map[] sources) {
    if (sources.length == 0) return [:]
    if (sources.length == 1) return sources[0]

    sources.inject([:]) { result, source ->
        source.each { k, v ->
            result[k] = result[k] instanceof Map ? merge(result[k], v) : v
        }
        result
    }
}

// return major versions supported or empty if just 0,1 series under
// -dev package.
ArrayList supported_branches(String ign_software)
{
  major_versions_registered = ignition_branches["${ign_software}"]

  if (major_versions_registered == null)
    return [ '' ]

  return major_versions_registered
}

// return prerelease branch names
ArrayList prerelease_branches(String ign_software)
{
  pre_branches = ignition_prerelease_branches["${ign_software}"]

  if (pre_branches == null)
    return [ '' ]

  return pre_branches
}

// return all ci branch names
ArrayList all_branches(String ign_software)
{
  List<String> branches = new ArrayList<String>();
  supported_branches("${ign_software}").each { major_version ->
    if ("${major_version}") {
      branches.add("ign-${ign_software}${major_version}")
    }
  }
  branches.add('default')
  prerelease_branches("${ign_software}").each { branch ->
    if ("${branch}") {
      branches.add(branch)
    }
  }
  return branches
}


// return all ci branch names
// Map with the form of: major versions as keys.
// Lists of distros supported as values
Map supported_install_pkg_branches(String ign_software)
{
  major_versions_prerelease = ignition_prerelease_pkgs["${ign_software}"]

  // construct a map of stable packages based on supported_branches and
  // all_supported_distros
  map_of_stable_versions = [:]
  map_of_stable_versions[ign_software] = [:]
  supported_branches(ign_software).each { major_version ->
    new_relation = [:]
    new_relation[major_version] = all_supported_distros
    map_of_stable_versions[ign_software] << new_relation
  }

  if (major_versions_prerelease == null)
    return map_of_stable_versions[ign_software];

  return merge_maps(map_of_stable_versions[ign_software],
                    major_versions_prerelease)
}

void include_gpu_label_if_needed(Job job, String ign_software_name)
{
  job.with
  {
    ignition_gpu.each { ign_each ->
      if (ign_software_name == ign_each)
        label "gpu-reliable"
    }
  }
}

boolean enable_testing(String ign_software_name)
{
  if (ign_software_name in ignition_no_test)
    return false

  return true
}

boolean is_a_colcon_package(String ign_software_name)
{
  if (ign_software_name in ignition_colcon_win)
    return true

  return false
}

// ABI Checker job
// Need to be before the ci-pr_any so the abi job name is defined
ignition_software.each { ign_sw ->
  abi_distro.each { distro ->
    supported_arches.each { arch ->
      abi_job_names[ign_sw] = "ignition_${ign_sw}-abichecker-any_to_any-ubuntu_auto-${arch}"
      def abi_job = job(abi_job_names[ign_sw])
      checkout_subdir = "ign-${ign_sw}"

      OSRFLinuxABI.create(abi_job)
      OSRFBitbucketHg.create(abi_job,
                            "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}/",
                            '${DEST_BRANCH}', checkout_subdir)
      abi_job.with
      {
        steps {
          shell("""\
                #!/bin/bash -xe
                wget https://raw.githubusercontent.com/osrf/bash-yaml/master/yaml.sh -O yaml.sh
                source yaml.sh

                create_variables \${WORKSPACE}/${checkout_subdir}/bitbucket-pipelines.yml

                export DISTRO=${distro}

                if [[ -n \${image} ]]; then
                  echo "Bitbucket pipeline.yml detected. Default DISTRO is ${distro}"
                  export DISTRO=\$(echo \${image} | sed  's/ubuntu://')
                fi

                export ARCH=${arch}
                export ABI_JOB_SOFTWARE_NAME=${checkout_subdir}
                /bin/bash -xe ./scripts/jenkins-scripts/docker/ignition-abichecker.bash
                """.stripIndent())
        } // end of steps
      }  // end of with
    } // end of arch
  } // end of distro
} // end of ignition

// MAIN CI JOBS (check every 5 minutes)
ignition_software.each { ign_sw ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the any job
    def ignition_ci_job_name = "ignition_${ign_sw}-ci-pr_any-ubuntu_auto-${arch}"
    def ignition_ci_any_job = job(ignition_ci_job_name)
    def ignition_checkout_dir = "ign-${ign_sw}"
    OSRFLinuxCompilationAny.create(ignition_ci_any_job,
                                  "https://bitbucket.org/ignitionrobotics/${ignition_checkout_dir}",
                                  enable_testing(ign_sw))
    include_gpu_label_if_needed(ignition_ci_any_job, ign_sw)
    ignition_ci_any_job.with
    {
      steps
      {
         conditionalSteps
         {
           condition
           {
             not {
               expression('${ENV, var="DEST_BRANCH"}', 'default')
             }

             steps {
               downstreamParameterized {
                 trigger(abi_job_names[ign_sw]) {
                   parameters {
                     currentBuild()
                   }
                 }
               }
             }
           }
         }

         shell("""\
              #!/bin/bash -xe
              wget https://raw.githubusercontent.com/osrf/bash-yaml/master/yaml.sh -O yaml.sh
              source yaml.sh

              create_variables \${WORKSPACE}/${ignition_checkout_dir}/bitbucket-pipelines.yml

              export DISTRO=${ci_distro_str}

              if [[ -n \${image} ]]; then
                echo "Bitbucket pipeline.yml detected. Default DISTRO is ${ci_distro}"
                export DISTRO=\$(echo \${image} | sed  's/ubuntu://')
              fi

              export ARCH=${arch}

              /bin/bash -xe ./scripts/jenkins-scripts/docker/ign_${ign_sw}-compilation.bash
              """.stripIndent())
      } // end of steps
    } // end of ci_any_job

    // add ci-pr_any to the list for CIWorkflow
    ci_pr_any_list[ign_sw] << ignition_ci_job_name
  }
}

// INSTALL PACKAGE ALL PLATFORMS / DAILY
ignition_software.each { ign_sw ->
  // Exclusion list
  if (ign_sw in ignition_no_pkg_yet)
    return

  supported_arches.each { arch ->
    supported_install_pkg_branches(ign_sw).each { major_version, supported_distros ->
      supported_distros.each { distro ->

        // only a few release branches support trusty anymore
        if (("${distro}" == "trusty") && !(
            (("${ign_sw}" == "math") && ("${major_version}" == "2")) ||
            (("${ign_sw}" == "math") && ("${major_version}" == "3"))))
          return
        // no bionic for math2 or math3
        if (("${distro}" == "bionic") && (
            (("${ign_sw}" == "math") && ("${major_version}" == "2")) ||
            (("${ign_sw}" == "math") && ("${major_version}" == "3"))))
          return
        // no bionic for transport3
        if (("${distro}" == "bionic") && (
            ("${ign_sw}" == "transport") && ("${major_version}" == "3")))
          return
        // no rndf install
        if ("${ign_sw}" == "rndf")
          return

        extra_repos_str=""
        if ((ign_sw in ignition_prerelease_pkgs) &&
           (major_version in ignition_prerelease_pkgs[ign_sw]) &&
           (distro in ignition_prerelease_pkgs[ign_sw][major_version]))
          extra_repos_str="prerelease"

        // No 1-dev packages, unversioned
        if ("${major_version}" == "1")
          major_version = ""

        // --------------------------------------------------------------
        def install_default_job = job("ignition_${ign_sw}${major_version}-install-pkg-${distro}-${arch}")
        OSRFLinuxInstall.create(install_default_job)
        include_gpu_label_if_needed(install_default_job, ign_sw)

        install_default_job.with
        {
          triggers {
            cron('@daily')
          }

          def dev_package = "libignition-${ign_sw}${major_version}-dev"

          steps {
           shell("""\
                 #!/bin/bash -xe

                 export DISTRO=${distro}
                 export ARCH=${arch}
                 export INSTALL_JOB_PKG=${dev_package}
                 export INSTALL_JOB_REPOS="stable ${extra_repos_str}"
                 /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
                 """.stripIndent())
          }
        }
      }
    }
  }
}

// OTHER CI SUPPORTED JOBS / DAILY
ignition_software.each { ign_sw ->
  all_supported_distros.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      // branches CI job scm@daily
      all_branches("${ign_sw}").each { branch ->
        def ignition_ci_job = job("ignition_${ign_sw}-ci-${branch}-${distro}-${arch}")
        OSRFLinuxCompilation.create(ignition_ci_job, enable_testing(ign_sw))
        OSRFBitbucketHg.create(ignition_ci_job,
                              "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}/",
                              "${branch}", "ign-${ign_sw}")

        include_gpu_label_if_needed(ignition_ci_job, ign_sw)
        ignition_ci_job.with
        {
          triggers {
            scm('@daily')
          }

          // only a few release branches support trusty anymore
          if (("${distro}" == "trusty") && !(
              ("${branch}" == "ign-math2") ||
              ("${branch}" == "ign-math3")))
            disabled()

          // no bionic for transport3
          if (("${distro}" == "bionic") && (
              ("${branch}" == "ign-transport3")))
            disabled()

          // no xenial for ign-physics/sensors/gazebo or plugin default/ign-plugin1
          if (("${distro}" == "xenial") && (
              ("${ign_sw}" == "gazebo") ||
              ("${ign_sw}" == "physics") ||
              ("${ign_sw}" == "plugin" && "${branch}" != "ign-plugin0") ||
              ("${ign_sw}" == "fuel-tools" && "${branch}" != "ign-fuel-tools1") ||
              ("${ign_sw}" == "sensors")))
            disabled()

          // gz11 branches don't work on trusty or xenial
          if (("${branch}" == "gz11") && (
              ("${distro}" == "xenial") ||
              ("${distro}" == "trusty")))
            disabled()

          steps {
            shell("""\
                  #!/bin/bash -xe

                  export DISTRO=${distro}
                  export ARCH=${arch}
                  /bin/bash -xe ./scripts/jenkins-scripts/docker/ign_${ign_sw}-compilation.bash
                  """.stripIndent())
          }
        }
      }
    }
  }
}

// --------------------------------------------------------------
// DEBBUILD: linux package builder
ignition_debbuild.each { ign_sw ->
  supported_branches("${ign_sw}").each { major_version ->
    // No 1-debbuild versions, they use the unversioned job
    if ("${major_version}" == "1")
      major_version = ""

    def build_pkg_job = job("ign-${ign_sw}${major_version}-debbuilder")
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
}

// --------------------------------------------------------------
// BREW: CI jobs

// 1. any job
ignition_software.each { ign_sw ->
  String ignition_brew_ci_any_job_name = "ignition_${ign_sw}-ci-pr_any-homebrew-amd64"
  def ignition_brew_ci_any_job = job(ignition_brew_ci_any_job_name)
  OSRFBrewCompilationAny.create(ignition_brew_ci_any_job,
                                "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}",
                                enable_testing(ign_sw))
  ignition_brew_ci_any_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              export HOMEBREW_SCRIPT="./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-homebrew-amd64.bash"
              if [ -s "\$HOMEBREW_SCRIPT" ]
              then
                /bin/bash -xe "\$HOMEBREW_SCRIPT"
              else
                /bin/bash -xe "./scripts/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash" "ignition-${ign_sw}"
              fi
              """.stripIndent())
      }
  }

  // add ci-pr_any to the list for CIWorkflow
  ci_pr_any_list[ign_sw] << ignition_brew_ci_any_job_name

  // 2. default, release branches
  all_branches("${ign_sw}").each { branch ->
    def ignition_brew_ci_job = job("ignition_${ign_sw}-ci-${branch}-homebrew-amd64")
    OSRFBrewCompilation.create(ignition_brew_ci_job, enable_testing(ign_sw))
    OSRFBitbucketHg.create(ignition_brew_ci_job,
                              "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}/",
                              "${branch}", "ign-${ign_sw}", "HomeBrew")
    ignition_brew_ci_job.with
    {
        triggers {
          scm('@daily')
        }

        steps {
          shell("""\
                #!/bin/bash -xe

                export HOMEBREW_SCRIPT="./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-homebrew-amd64.bash"
                if [ -s "\$HOMEBREW_SCRIPT" ]
                then
                  /bin/bash -xe "\$HOMEBREW_SCRIPT"
                else
                  /bin/bash -xe "./scripts/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash" "ignition-${ign_sw}"
                fi
                """.stripIndent())
        }
    }
  }
}

// --------------------------------------------------------------
// WINDOWS: CI job

// 1. any
ignition_software.each { ign_sw ->

  if (is_a_colcon_package(ign_sw)) {
    // colcon uses long paths and windows has a hard limit of 260 chars. Keep
    // names minimal
    ignition_win_ci_any_job_name = "ign_${ign_sw}-pr-win"
    Globals.gazebodistro_branch = true
  } else {
    ignition_win_ci_any_job_name = "ignition_${ign_sw}-ci-pr_any-windows7-amd64"
    Globals.gazebodistro_branch = false
  }

  def ignition_win_ci_any_job = job(ignition_win_ci_any_job_name)
  OSRFWinCompilationAny.create(ignition_win_ci_any_job,
                               "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}",
                               enable_testing(ign_sw))
  ignition_win_ci_any_job.with
  {
      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }

  // add ci-pr_any to the list for CIWorkflow
  ci_pr_any_list[ign_sw] << ignition_win_ci_any_job_name

  // 2. default, release branches
  all_branches("${ign_sw}").each { branch ->
    if (is_a_colcon_package(ign_sw)) {
      // colcon uses long paths and windows has a hard limit of 260 chars. Keep
      // names minimal
      if (branch == 'default')
        branch_name = "ci"
      else
        branch_name = branch - ign_sw
      ignition_win_ci_job_name = "ign_${ign_sw}-${branch_name}-win"
    } else {
      ignition_win_ci_job_name = "ignition_${ign_sw}-ci-${branch}-windows7-amd64"
    }

    def ignition_win_ci_job = job(ignition_win_ci_job_name)
    OSRFWinCompilation.create(ignition_win_ci_job, enable_testing(ign_sw))
    OSRFBitbucketHg.create(ignition_win_ci_job,
                              "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}/",
                              "${branch}", "ign-${ign_sw}")

    ignition_win_ci_job.with
    {
        triggers {
          scm('@daily')
        }

        steps {
          batchFile("""\
                call "./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-windows-amd64.bat"
                """.stripIndent())
        }
    }
  }
}

// Main CI workflow
ignition_software.each { ign_sw ->
  def String ci_main_name = "ignition_${ign_sw}-ci-pr_any"
  def ign_ci_main = pipelineJob(ci_main_name)
  OSRFCIWorkFlowMultiAny.create(ign_ci_main, ci_pr_any_list[ign_sw])
}
