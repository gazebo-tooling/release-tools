import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION PACKAGES
gz_software = [ 'cmake',
                'common',
                'fuel-tools',
                'gazebo',
                'gui',
                'launch',
                'math',
                'msgs',
                'physics',
                'plugin',
                'rendering',
                'sensors',
                'sim',
                'tools',
                'transport',
                'utils' ]
// DESC: need gpu/display for tests
gz_gpu                = [ 'gazebo',
                          'gui',
                          'rendering',
                          'sim',
                          'sensors' ]
// DESC: software does not support cmake warnings enabled
gz_no_cmake_warnings = [ 'cmake',
                      'common',
                      'fuel-tools',
                      'gazebo',
                      'gui',
                      'launch',
                      'math',
                      'msgs',
                      'physics',
                      'rendering',
                      'sensors',
                      'tools',
                      'transport',
                      'utils' ]
// DESC: software does not have tests
gz_no_test            = [  ]
// DESC: major series supported and released. The branches get CI, install pkg
// testing and debbuild job.
// No branches in gz_branches means no released branches (only CI on
// main, ABI check, install pkg)
ignition_branches           = [ 'cmake'      : [ '2' ],
                                'common'     : [ '1', '3', '4' ],
                                'fuel-tools' : [ '1', '4', '7' ],
                                'gazebo'     : [ '3', '6' ],
                                'gui'        : [ '0', '3', '6' ],
                                'launch'     : [ '2', '5' ],
                                'math'       : [ '4', '6' ],
                                'msgs'       : [ '1', '5', '8' ],
                                'physics'    : [ '2', '5' ],
                                'plugin'     : [ '1' ],
                                'rendering'  : [ '3', '6' ],
                                'sensors'    : [ '3', '6' ],
                                'tools'      : [ '1' ],
                                'transport'  : [ '4', '8', '11' ],
                                'utils'      : [ '1' ]]

gz_branches                 = [ 'cmake'      : [ '3' ],
                                'common'     : [ '5' ],
                                'fuel-tools' : [ '8', '9' ],
                                'gui'        : [ '7', '8' ],
                                'launch'     : [ '6', '7' ],
                                'math'       : [ '7' ],
                                'msgs'       : [ '9', '10'],
                                'physics'    : [ '6', '7' ],
                                'plugin'     : [ '2' ],
                                'rendering'  : [ '7', '8' ],
                                'sensors'    : [ '7', '8' ],
                                'sim'        : [ '7', '8' ],
                                'tools'      : [ '2' ],
                                'transport'  : [ '12', '13' ],
                                'utils'      : [ '2' ]]
// DESC: prerelease branches are managed as any other supported branches for
// special cases different to major branches: get compilation CI on the branch
// physics/sensors don't need to be included since they use main for gz11
gz_prerelease_branches = []
// DESC: versioned names to generate debbuild jobs for special cases that
// don't appear in gz_branches (like nightly builders or 0-debbuild
// jobs for the special cases of foo0 packages)
gz_extra_debbuild = [ 'cmake4',
                      'common6',
                      'fuel-tools10',
                      'gui9',
                      'launch8',
                      'math8',
                      'msgs11',
                      'physics8',
                      'plugin3',
                      'rendering9',
                      'sensors9',
                      'sim9',
                      'transport14',
                      'utils3',

                      'utils1' // see comment https://github.com/gazebo-tooling/release-tools/pull/431#issuecomment-815099918
                    ]
// DESC: exclude ignition from generate any install testing job
gz_no_pkg_yet         = [  ]
// DESC: major versions that has a package in the prerelease repo. Should
// not appear in gz_no_pkg_yet nor in gz_branches
gz_prerelease_pkgs    = [ 'placeholder' : [
                          '1':  [ 'bionic' ]],
                        ]
// packages using colcon for windows compilation while migrating all them to
// this solution
gz_colcon_win         = [ 'cmake',
                          'common',
                          'fuel-tools',
                          'gazebo',
                          'gui',
                          'launch',
                          'math',
                          'msgs',
                          'physics',
                          'plugin',
                          'rendering',
                          'sensors',
                          'tools',
                          'transport',
                          'utils' ]

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

// shell command to inject in all bash steps
GLOBAL_SHELL_CMD=''
GITHUB_SUPPORT_ALL_BRANCHES = []
ENABLE_GITHUB_PR_INTEGRATION = true

String ci_distro_str = ci_distro[0]

// Map of lists to use in CIWorkflow
ci_pr_any_list = [:]
gz_software.each { gz_sw ->
  def list_empty = []
  ci_pr_any_list[gz_sw] = list_empty
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
ArrayList supported_ign_branches(String ign_software)
{
  // sim was not used in ignition
  if (ign_software == 'sim')
    return ['']

  major_versions_registered = ignition_branches["${ign_software}"]

  if (major_versions_registered == null)
    return [ '' ]

  return major_versions_registered
}

// return major versions supported or empty if just 0,1 series under
// -dev package.
ArrayList supported_gz_branches(String gz_software)
{
  // sim was not used in ignition
  if (gz_software == 'gazebo')
    return ['']

  major_versions_registered = gz_branches["${gz_software}"]

  if (major_versions_registered == null)
    return [ '' ]

  return major_versions_registered
}

// return prerelease branch names
ArrayList prerelease_branches(String gz_software)
{
  pre_branches = gz_prerelease_branches["${gz_software}"]

  if (pre_branches == null)
    return [ '' ]

  return pre_branches
}

// return all ci branch names
ArrayList all_branches(String software_name)
{
  List<String> branches = new ArrayList<String>();
  supported_ign_branches("${software_name}").each { major_version ->
    if ("${major_version}") {
      branches.add("ign-${software_name}${major_version}")
    }
  }
  prerelease_branches("${software_name}").each { branch ->
    if ("${branch}") {
      branches.add(branch)
    }
  }
  if (software_name == 'gazebo')
    software_name = 'sim'
  supported_gz_branches("${software_name}").each { major_version ->
    if ("${major_version}") {
      branches.add("gz-${software_name}${major_version}")
    }
  }
  branches.add('main')
  return branches
}


ArrayList all_debbuilders()
{
  List<String> branches = new ArrayList<String>();
  // add all supported branches
  gz_software.each { software_name ->
    supported_ign_branches("${software_name}").each { major_version ->
      if (major_version) {
        // No 1-debbuild versions, they use the unversioned job
        if ("${major_version}" == "0"  || "${major_version}" == "1" )
          major_version = ""

        branches.add("ign-${software_name}${major_version}")
      }
    }
    if (software_name == 'gazebo')
      software_name = 'sim'
    supported_gz_branches("${software_name}").each { major_version ->
      if (major_version) {
        // No 1-debbuild versions, they use the unversioned job
        if ("${major_version}" == "0"  || "${major_version}" == "1" )
          major_version = ""

        branches.add("gz-${software_name}${major_version}")
      }
    }
  }
  // add all extra debbuilders
  gz_extra_debbuild.each { gz_name ->
    // utils1 is still using the ign preffix
    if (gz_name == 'utils1')
      branches.add("ign-${gz_name}")
    else
      branches.add("gz-${gz_name}")
  }

  return branches
}

// return all ci branch names
// Map with the form of: major versions as keys.
// Lists of distros supported as values
Map supported_install_pkg_branches(String gz_software)
{
  major_versions_prerelease = gz_prerelease_pkgs["${gz_software}"]

  // construct a map of stable packages based on supported_gz_branches and
  // all_supported_distros
  map_of_stable_versions = [:]
  map_of_stable_versions[gz_software] = [:]
  (supported_ign_branches(gz_software) + supported_gz_branches(gz_software)).each { major_version ->
    if (! major_version)
      return false  // keep looping
    new_relation = [:]
    new_relation[major_version] = all_supported_distros
    map_of_stable_versions[gz_software] << new_relation
  }

  if (major_versions_prerelease == null)
    return map_of_stable_versions[gz_software]

  return merge_maps(map_of_stable_versions[gz_software],
                    major_versions_prerelease)
}

void include_gpu_label_if_needed(Job job, String gz_software_name)
{
  job.with
  {

    gz_gpu.each { gz_each ->
      if (gz_software_name == gz_each)
      {
        label Globals.nontest_label("gpu-reliable")

        // unstable build if missing valid gpu display
        publishers {
          consoleParsing {
            projectRules('scripts/jenkins-scripts/parser_rules/display_missing.parser')
            unstableOnWarning()
            failBuildOnError(false)
          }
        }
      }
    }
  }
}

boolean enable_cmake_warnings(String gz_software_name)
{
  if (gz_software_name in gz_no_cmake_warnings)
    return false

  return true
}

boolean enable_testing(String gz_software_name)
{
  if (gz_software_name in gz_no_test)
    return false

  return true
}

boolean is_a_colcon_package(String gz_software_name)
{
  if (gz_software_name in gz_colcon_win)
    return true

  return false
}

// Need to be before the ci-pr_any so the abi job name is defined
gz_software.each { gz_sw ->
  supported_arches.each { arch ->
    // 1 Per library and per linux arch
    //   1.1 DEPRECATED Per abi_distro
    //     1.1.1 [job] ABI checker for main branches
    //   1.2 DEPRECATED Per ci_str_distro
    //     1.2.1 [job] Main PR jobs (-ci-pr_any-)
    //   1.3 Per all supported_distros
    //     1.3.1 Per all supported branches on each library
    //       1.3.1.1 [job] Branch jobs -ci-$branch-

    // 1.1.1 ABI checker for main branches
    // --------------------------------------------------------------
    abi_distro.each { distro ->
      // Packages without ABI
      if (gz_sw == 'tools' || gz_sw == 'cmake')
        return

      software_name = gz_sw  // Necessary substitution. gz_sw won't overwrite

      if (gz_sw == 'sim')
        software_name = "gazebo"

      abi_job_names[software_name] = "ignition_${software_name}-abichecker-any_to_any-ubuntu_auto-${arch}"
      def abi_job = job(abi_job_names[software_name])
      GenericAnyJobGitHub.create(abi_job,
                        "gazebosim/ign-${software_name}",
                        all_branches(software_name) - [ 'main'])
      abi_job.with
      {
        description 'Automatic generated job by DSL jenkins. Stub job for migration, not doing any check'
      }  // end of with
    } // end of abi_distro
  } // end of arch
} // end of gz_software

// --------------------------------------------------------------
// DEBBUILD: linux package builder
all_debbuilders().each { debbuilder_name ->
  extra_str = ""
  if (debbuilder_name.contains("gazebo") || debbuilder_name == "transport7")
    extra_str="export NEED_C17_COMPILER=true"

  // Gazebo physics consumes huge amount of memory making arm node to FAIL
  // Force here to use one compilation thread
  if (debbuilder_name.contains("-physics"))
    extra_str += '\nexport MAKE_JOBS=1'

  def build_pkg_job = job("${debbuilder_name}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)
  build_pkg_job.with
  {

      concurrentBuild(true)

      throttleConcurrentBuilds {
        maxPerNode(1)
        maxTotal(8)
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              ${GLOBAL_SHELL_CMD}
              ${extra_str}
              /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-ignition-debbuild.bash
              """.stripIndent())
      }
  }
}

// --------------------------------------------------------------
// WINDOWS: CI job

// Main CI workflow
gz_software.each { gz_sw ->
  if (gz_sw == 'sim')
    return
  def String ci_main_name = "ignition_${gz_sw}-ci-manual_any"
  def gz_ci_main = pipelineJob(ci_main_name)
  OSRFCIWorkFlowMultiAnyGitHub.create(gz_ci_main, ci_pr_any_list[gz_sw])
}
