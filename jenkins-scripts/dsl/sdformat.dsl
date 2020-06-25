import _configs_.*
import javaposse.jobdsl.dsl.Job

def sdformat_supported_branches = [ 'sdformat4', 'sdformat6', 'sdformat8' , 'sdformat9' ]
def sdformat_gz11_branches = [ 'sdformat8', 'sdformat9', 'master' ]
// nightly and prereleases
def extra_sdformat_debbuilder = [ 'sdformat7' ]

// Main platform using for quick CI
def ci_distro               = Globals.get_ci_distro()
def abi_distro              = Globals.get_abi_distro()
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = Globals.get_other_supported_distros()
def all_supported_distros   = Globals.get_all_supported_distros()
def supported_arches        = Globals.get_supported_arches()
def experimental_arches     = Globals.get_experimental_arches()

String ci_distro_str = ci_distro[0]
String ci_build_any_job_name_linux = "sdformat-ci-pr_any-ubuntu_auto-amd64"

// Need to be used in ci_pr
String abi_job_name = ''

// Helper function
String get_sdformat_branch_name(String full_branch_name)
{
  String sdf_branch = full_branch_name.replace("ormat",'')

  return sdf_branch
}

// ABI Checker job
// Need to be the before ci-pr_any so the abi job name is defined
abi_branches = sdformat_supported_branches.collect { it -> get_sdformat_branch_name(it) }
abi_distro.each { distro ->
  supported_arches.each { arch ->
    abi_job_name = "_test_jrivero_sdformat-abichecker-any_to_any-ubuntu_auto-${arch}"
    def abi_job = job(abi_job_name)
    OSRFLinuxABIGitHub.create(abi_job)
    GenericAnyJobGitHub.create(abi_job, "j-rivero/sdformat", abi_branches)
    abi_job.with
    {
      steps {
        shell("""\
              #!/bin/bash -xe
              wget https://raw.githubusercontent.com/osrf/bash-yaml/master/yaml.sh -O yaml.sh
              source yaml.sh

              create_variables \${WORKSPACE}/sdformat/bitbucket-pipelines.yml

              export DISTRO=${distro}

              if [[ -n \${image} ]]; then
                echo "Bitbucket pipeline.yml detected. Default DISTRO is ${distro}"
                export DISTRO=\$(echo \${image} | sed  's/ubuntu://')
              fi

              export ARCH=${arch}
              export DEST_BRANCH=\${DEST_BRANCH:-\$ghprbTargetBranch}
              export SRC_BRANCH=\${SRC_BRANCH:-\$ghprbSourceBranch}
              export SRC_REPO=\${SRC_REPO:-\$ghprbAuthorRepoGitUrl}

              /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-abichecker.bash
	      """.stripIndent())
      } // end of steps
    }  // end of with
  } // end of arch
} // end of distro
