import _configs_.*
import javaposse.jobdsl.dsl.Job

import org.yaml.snakeyaml.Yaml

// shell command to inject in all bash steps
GLOBAL_SHELL_CMD=''

// GZ COLLECTIONS
ENABLE_CPPCHECK = true

def WRITE_JOB_LOG = System.getenv('WRITE_JOB_LOG') ?: false
logging_list = [:]
logging_list['branch_ci'] = []

// Jenkins needs the relative path to work and locally the simulation is done
// using a symlink
file = readFileFromWorkspace("scripts/jenkins-scripts/dsl/gz-collections.yaml")
gz_collections_yaml = new Yaml().load(file)

void generate_label_by_requirements(job, lib_name, requirements)
{

  if (requirements.nvidia_gpu.contains(lib_name) &&
      requirements.large_memory.contains(lib_name)) {
     println ("ERROR: more than one label is generated by requirements")
     exit(1)
  }

  label = requirements.nvidia_gpu.contains(lib_name) ? "gpu-reliable" : null
  if (! label)
    label = requirements.large_memory.contains(lib_name) ? "large-memory" : null
    if (! label)
      return

  job.with
  {
    label Globals.nontest_label(label)

    if (requirements.nvidia_gpu.contains(lib_name)) {
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

/* Generate the release-tools script name based on lib_name. The ign replacement can be
 * removed after EOL of Bionic */
String cleanup_library_name(lib_name)
{
  return lib_name.replaceAll('-','_').replaceAll('ign_','gz_').replaceAll('gazebo','sim')
}

boolean is_testing_enabled(lib_name, ci_config)
{
  return ! ci_config.tests_disabled?.contains(lib_name)
}

/*
 * Generate an index that facilitates the operations with the yaml values,
 * avoiding to parse them several times.
 *
 * # ci_configs_by_lib index structure:
 *   lib_name : [ ci_config_name : [ .branch .collection ] ]
 *
 *   The index main keys are the lib names (i.e: gz-cmake) and associated them
 *   another map of CI configuration names supported as keys (i.e: jammy) with the
 *   list of associated items composed by a map: branch (and collection) that CI configuration
 *   (i.e [[branch:gz-cmake3, collection: harmonic], [branch: gz-cmake3, collection: garden])
 *   as values. In a graphic;
 *
 *   index[gz-cmake][jammy] -> [ branch: gz-cmake3, collection: garden ,
 *                               branch: gz-cmake3, collection: harmonic]
 *
 * # pkgconf_per_src_inde index structure:
 *   pkg_src_name : [ packaging_config_name : [ .lib_name .collection ] ]
 *
 *   The index main keys are package source names (i.e gz-cmake3 or gz-harmonic and associated them
 *   another map of packaging configuration names supported as keys (i.e: jammy) with the
 *   list of associated items composed by a map: lib_name (canonical name for the source package)
 *   (and collection)
 *
 *   index[gz-cmake3][jammy] -> [ lib_name: gz-cmake, collection: harmonic ]
 */
void generate_ciconfigs_by_lib(config, ciconf_per_lib_index, pkgconf_per_src_index)
{
  config.collections.each { collection ->
    collection.libs.each { lib ->
      def libName = lib.name
      def branch = lib.repo.current_branch
      collection.ci.configs.each { config_name ->
        ciconf_per_lib_index[libName][config_name] = ciconf_per_lib_index[libName][config_name]?: []
        ciconf_per_lib_index[libName][config_name].contains(branch) ?: ciconf_per_lib_index[libName][config_name] << [branch: branch, collection: collection.name]
      }
      def pkg_name = lib.name + lib.major_version
      if (collection.packaging.linux?.ignore_major_version?.contains(libName))
        pkg_name = lib.name
      collection.packaging.configs?.each { config_name ->
        pkgconf_per_src_index[pkg_name][config_name] = pkgconf_per_src_index[pkg_name][config_name]?: []
        pkgconf_per_src_index[pkg_name][config_name] << [ lib_name: libName, collection: collection.name ]
      }
    }
  }
}

void generate_ci_job(gz_ci_job, lib_name, branch, ci_config,
                     extra_cmake = '', extra_test = '', extra_cmd = '')
{
  def script_name_prefix = cleanup_library_name(lib_name)
  def distro = ci_config.system.version
  def arch = ci_config.system.arch
  def pre_setup_script = ci_config.pre_setup_script_hook?.get(lib_name)?.join('\n')
  extra_cmd = [extra_cmd, pre_setup_script].findAll({ it != null }).join()

  OSRFLinuxCompilation.create(gz_ci_job, is_testing_enabled(lib_name, ci_config))
  OSRFGitHub.create(gz_ci_job,
                    "gazebosim/${lib_name}", "${branch}")
  generate_label_by_requirements(gz_ci_job, lib_name, ci_config.requirements)
  gz_ci_job.with
  {
    steps {
      shell("""\
            #!/bin/bash -xe

            ${GLOBAL_SHELL_CMD}
            ${extra_cmd}
            export BUILDING_EXTRA_CMAKE_PARAMS="${extra_cmake}"
            export BUILDING_EXTRA_MAKETEST_PARAMS="${extra_test}"
            export BUILDING_SOFTWARE_DIRECTORY="${lib_name}"
            export DISTRO=${distro}
            export ARCH=${arch}
            /bin/bash -xe ./scripts/jenkins-scripts/docker/${script_name_prefix}-compilation.bash
            """.stripIndent())
    }
  }
}

def ciconf_per_lib_index = [:].withDefault { [:] }
def pkgconf_per_src_index = [:].withDefault { [:] }
generate_ciconfigs_by_lib(gz_collections_yaml, ciconf_per_lib_index, pkgconf_per_src_index)

// Generate PR jobs: 1 per ci configuration on each lib
ciconf_per_lib_index.each { lib_name, lib_configs ->
  lib_configs.each { ci_configs ->
    def config_name = ci_configs.getKey()
    def ci_config = gz_collections_yaml.ci_configs.find{ it.name == config_name }
    def branches_with_collections = ci_configs.getValue()
    def branch_names = branches_with_collections.collect { it.branch }.unique()
    if (ci_config.exclude.all?.contains(lib_name))
      return
    assert(lib_name)
    assert(branch_names)
    assert(ci_config)

    // Main PR jobs (-ci-pr_any-) (pulling check every 5 minutes)
    // --------------------------------------------------------------
    def script_name_prefix = cleanup_library_name(lib_name)
    def distro = ci_config.system.version
    def arch = ci_config.system.arch
    def gz_job_name_prefix = lib_name.replaceAll('-','_')
    def pre_setup_script = ci_config.pre_setup_script_hook?.get(lib_name)?.join('\n')
    def extra_cmd = pre_setup_script ?: ""

    // Main PR jobs (-ci-pr_any-) (pulling check every 5 minutes)
    // --------------------------------------------------------------
    def gz_ci_job_name = "${gz_job_name_prefix}-ci-pr_any-${distro}-${arch}"
    def gz_ci_any_job = job(gz_ci_job_name)
    OSRFLinuxCompilationAnyGitHub.create(gz_ci_any_job,
                                         "gazebosim/${lib_name}",
                                         is_testing_enabled(lib_name, ci_config),
                                         ENABLE_CPPCHECK,
                                         branch_names)
    generate_label_by_requirements(gz_ci_any_job, lib_name, ci_config.requirements)
    gz_ci_any_job.with
    {
      steps
      {
         shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}

              ${GLOBAL_SHELL_CMD}
              ${extra_cmd}

              export BUILDING_SOFTWARE_DIRECTORY=${lib_name}
              export ARCH=${arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/${script_name_prefix}-compilation.bash
              """.stripIndent())
      } // end of steps
    } // end of ci_any_job

    if (! ci_config.exclude.abichecker?.contains(lib_name)) {
      // ABI branch jobs (-ci-abichecker-) for non main branches
      def abi_job_name = "${gz_job_name_prefix}-abichecker-any_to_any-ubuntu-${distro}-${arch}"
      def abi_job = job(abi_job_name)
      OSRFLinuxABIGitHub.create(abi_job)
      GenericAnyJobGitHub.create(abi_job,
                        "gazebosim/${lib_name}",
                        branch_names - [ 'main'])
      generate_label_by_requirements(abi_job, lib_name, ci_config.requirements)
      abi_job.with
      {
        steps {
          shell("""\
                #!/bin/bash -xe

                export DISTRO=${distro}

                ${GLOBAL_SHELL_CMD}
                ${extra_cmd}

                export ARCH=${arch}
                export DEST_BRANCH=\${DEST_BRANCH:-\$ghprbTargetBranch}
                export SRC_BRANCH=\${SRC_BRANCH:-\$ghprbSourceBranch}
                export SRC_REPO=\${SRC_REPO:-\$ghprbAuthorRepoGitUrl}
                export ABI_JOB_SOFTWARE_NAME=${lib_name}
                /bin/bash -xe ./scripts/jenkins-scripts/docker/gz-abichecker.bash
                """.stripIndent())
        } // end of steps
      }  // end of with
    }

    // CI branch jobs (-ci-$branch-) (pulling check every 5 minutes)
    branches_with_collections.each { branch_and_collection ->
      branch_name = branch_and_collection.branch
      def gz_ci_job = job("${gz_job_name_prefix}-ci-${branch_name}-${distro}-${arch}")
      generate_ci_job(gz_ci_job, lib_name, branch_name, ci_config)
      gz_ci_job.with
      {
        triggers {
          scm('@daily')
        }
      }

      logging_list['branch_ci'].add(
        [collection: branch_and_collection.collection,
         job_name: gz_ci_job.name])
    } // end_of_branch
  } //en of lib_configs
} // end of lib

pkgconf_per_src_index.each { pkg_src, pkg_src_configs ->
  pkg_src_configs.each { pkg_src_config ->
    def config_name = pkg_src_config.getKey()
    def pkg_config = gz_collections_yaml.packaging_configs.find{ it.name == config_name }
    // lib_names are the same in all the entries
    def canonical_lib_name = pkg_src_config.getValue()[0].lib_name

    if (pkg_config.exclude?.contains(canonical_lib_name))
      return

    def gz_source_job = job("${pkg_src}-source")
    OSRFSourceCreation.create(gz_source_job, [
      PACKAGE: pkg_src,
      SOURCE_REPO_URI: "https://github.com/gazebosim/${canonical_lib_name}.git"])
    OSRFSourceCreation.call_uploader_and_releasepy(gz_source_job,
      'repository_uploader_packages',
      '_releasepy')
  }
}

if (WRITE_JOB_LOG) {
  File log_file = new File("jobs.txt")
  logging_list.each { log_type, items ->
    items.each { log_file.append("${log_type} ${it.collection} ${it.job_name}\n") }
  }
}
