import _configs_.*
import javaposse.jobdsl.dsl.Job

import org.yaml.snakeyaml.Yaml

// shell command to inject in all bash steps
GLOBAL_SHELL_CMD=''

// GZ COLLECTIONS
ENABLE_CPPCHECK = true

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

boolean is_testing_enabled(lib_name, ci_config)
{
  return ! ci_config.tests_disabled?.contains(lib_name)
}


/*
 * Generate an index that facilitates the operations with the yaml values,
 * avoiding to parse them several times.
 *
 * ci_configs_by_lib index structure:
 *   lib_name : [ ci_config_name : [ branches ] ]
 *
 *   The index main keys are the lib names (i.e: gz-cmake) and associated them
 *   another map of CI configuration names supported as keys (i.e: jammy) with the
 *   list of associated branches for that CI configuration  (i.e [gz-cmake3, gz-cmake4])
 *   as values.
 */
void generate_ciconfigs_by_lib(config, configs_per_lib_index)
{
  config.collections.each { collection ->
    // TODO(jrivero): limit to harmonic for testing proposes
    if (collection.name != 'harmonic')
      return

    collection.libs.each { lib ->
      def libName = lib.name
      def branch = lib.repo.current_branch
      collection.ci.configs.each { config_name ->
        configs_per_lib_index["$libName"]["${config_name}"] = configs_per_lib_index["$libName"]["${config_name}"]?: []
        configs_per_lib_index["$libName"]["${config_name}"].contains(branch) ?: configs_per_lib_index["$libName"]["${config_name}"] << branch
      }
    }
  }
}

def configs_per_lib_index = [:].withDefault { [:] }
generate_ciconfigs_by_lib(gz_collections_yaml, configs_per_lib_index)

// Generate PR jobs: 1 per ci configuration on each lib
configs_per_lib_index.each { lib_name, lib_configs ->
  lib_configs.each { ci_configs ->
    def config_name = ci_configs.getKey()
    def ci_config = gz_collections_yaml.ci_configs.find{ it.name == config_name }
    def branch_names = ci_configs.getValue()
    if (ci_config.exclude.contains(lib_name))
      return
    assert(lib_name)
    assert(branch_names)
    assert(ci_config)

    if (ci_config.exclude.contains(lib_name))
      return

    // 1.2.1 Main PR jobs (-ci-pr_any-) (pulling check every 5 minutes)
    // --------------------------------------------------------------
    def distro = ci_config.system.version
    def arch = ci_config.system.arch
    def gz_job_name_prefix = lib_name.replaceAll('-','_')
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

              export BUILDING_SOFTWARE_DIRECTORY=${lib_name}
              export ARCH=${arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/${gz_job_name_prefix}-compilation.bash
              """.stripIndent())
      } // end of steps
    } // end of ci_any_job
  } //en of lib_configs
} // end of lib
