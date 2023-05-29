import _configs_.*
import javaposse.jobdsl.dsl.Job

import org.yaml.snakeyaml.Yaml

Globals.extra_emails = "caguero@osrfoundation.org"
// shell command to inject in all bash steps
GLOBAL_SHELL_CMD=''

// GZ COLLECTIONS
arch = 'amd64'

// Jenkins needs the relative path to work and locally the simulation is done
// using a symlink
file = readFileFromWorkspace("scripts/jenkins-scripts/dsl/gz-collections.yaml")
gz_collections_yaml = new Yaml().load(file)

boolean include_gpu_label_if_needed(job, lib, ci_info)
{
  job.with
  {
    if (ci_info.requirements.nvidia_gpu.contains(lib.name))
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

[ 'harmonic' ].each { collection_name ->
  collection = gz_collections_yaml.collections. find { it.name == collection_name }
  collection.ci.linux.reference_distro.each { distro ->
    collection.libs.findAll { ! collection.ci.exclude.contains(it.name) }.each { lib ->
      // 1.2.1 Main PR jobs (-ci-pr_any-) (pulling check every 5 minutes)
      // --------------------------------------------------------------
      def gz_ci_job_name = "${lib.name}${lib.major_version}-ci-pr_any-${distro}-${arch}"
      def gz_ci_any_job = job(gz_ci_job_name)
      def tests_disabled = (collection.ci.tests_disabled?.contains(lib.name)) ? true : false
      OSRFLinuxCompilationAnyGitHub.create(gz_ci_any_job,
                                          "gazebosim/${lib.name}",
                                           tests_disabled)
      include_gpu_label_if_needed(gz_ci_any_job, lib, collection.ci)
      gz_ci_any_job.with
      {
        if (collection.ci.requirements.large_memory?.contains(lib.name))
          label Globals.nontest_label("large-memory")

        steps
        {
           shell("""\
                #!/bin/bash -xe

                export DISTRO=${distro}

                ${GLOBAL_SHELL_CMD}

                export ARCH=${arch}

                // TODO: CREATE SYMLINKS TO ign-compilation to be gz-compilation
                /bin/bash -xe ./scripts/jenkins-scripts/docker/${lib.name}-compilation.bash
                """.stripIndent())
        } // end of steps
      } // end of ci_any_job
    } //en of lib
  } // end of distro
} // end of collection
