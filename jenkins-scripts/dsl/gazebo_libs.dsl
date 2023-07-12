import _configs_.*
import javaposse.jobdsl.dsl.Job

import org.yaml.snakeyaml.Yaml

// shell command to inject in all bash steps
GLOBAL_SHELL_CMD=''

// GZ COLLECTIONS
arch = 'amd64'
ENABLE_CPPCHECK = true

// Jenkins needs the relative path to work and locally the simulation is done
// using a symlink
file = readFileFromWorkspace("scripts/jenkins-scripts/dsl/gz-collections.yaml")
gz_collections_yaml = new Yaml().load(file)

boolean include_gpu_label_if_needed(job, lib, nvidia_gpu_libs)
{
  job.with
  {
    if (nvidia_gpu_libs.contains(lib))
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

void generate_platorms_by_lib(config, libVersions)
{
  config.collections.each { collection ->
    def collectionName = collection.name
    def libs = collection.libs

    libs.each { lib ->
      def libName = lib.name
      def branch = lib.repo.current_branch
      collection.ci.platforms.each { platform ->
        libVersions["$libName@$branch"].contains(platform) ?: libVersions["$libName@$branch"] << platform
      }
    }
  }
}

def libVersions = [:].withDefault { [] }
generate_platorms_by_lib(gz_collections_yaml, libVersions)

libVersions.each { lib, ref_distros ->
  def lib_name = lib.split('@')[0]
  def lib_branch = lib.split('@')[1]
  ref_distros.each { distro ->
    def platform_config = gz_collections_yaml.ci_platforms.find{ it.name == distro  }
    assert(lib_name)
    assert(lib_branch)
    assert(platform_config)
    // 1.2.1 Main PR jobs (-ci-pr_any-) (pulling check every 5 minutes)
    // --------------------------------------------------------------
    def gz_job_name_prefix = lib_name.replaceAll('-','_')
    def gz_ci_job_name = "${gz_job_name_prefix}-ci-pr_any-${distro}-${arch}"
    def gz_ci_any_job = job(gz_ci_job_name)
    def enable_testing = platform_config.tests_disabled?.contains(lib_name) ? false : true
    OSRFLinuxCompilationAnyGitHub.create(gz_ci_any_job,
                                         "gazebosim/${lib_name}",
                                         enable_testing,
                                         ENABLE_CPPCHECK,
                                         [ lib_branch ])
    include_gpu_label_if_needed(gz_ci_any_job, lib_name, platform_config.requirements.nvidia_gpu)
    gz_ci_any_job.with
    {
      if (platform_config.requirements.large_memory?.contains(lib_name))
        label Globals.nontest_label("large-memory")

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
  } //en of distro
} // end of lib
