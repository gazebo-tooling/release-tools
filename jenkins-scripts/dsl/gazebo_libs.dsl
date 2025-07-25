import _configs_.*
import javaposse.jobdsl.dsl.Job

import org.yaml.snakeyaml.Yaml

// shell command to inject in all bash steps
GLOBAL_SHELL_CMD=''

// GZ COLLECTIONS
ENABLE_CPPCHECK = true
GITHUB_SUPPORT_ALL_BRANCHES = []
ENABLE_GITHUB_PR_INTEGRATION = true
DISABLE_TESTING = false
DISABLE_CMAKE_WARNS = false

def WRITE_JOB_LOG = System.getenv('WRITE_JOB_LOG') ?: false
logging_list = [:].withDefault {[]}

// Jenkins needs the relative path to work and locally the simulation is done
// using a symlink
file = readFileFromWorkspace("scripts/jenkins-scripts/dsl/gz-collections.yaml")
gz_collections_yaml = new Yaml().load(file)

String get_windows_distro_sortname(ci_config)
{
  // return the components initials of the distribution and the version strings counting
  // _ as separator for components.
  return ci_config.system.distribution.split('_').collect { it[0] }.join('') \
          + ci_config.system.version.split('_').collect { it[0] }.join('')
}

void generate_label_by_requirements(job, lib_name, requirements, base_label)
{

  if (requirements.nvidia_gpu.contains(lib_name) &&
      requirements.large_memory.contains(lib_name)) {
     println ("ERROR: more than one label is not supported by requirements")
     exit(1)
  }

  label = requirements.nvidia_gpu.contains(lib_name) ? "gpu-reliable" : null
  if (! label)
    label = requirements.large_memory.contains(lib_name) ? "large-memory" : null
    if (! label)
      return

  job.with
  {
    label Globals.nontest_label("${base_label} && ${label}")

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

boolean are_cmake_warnings_enabled(lib_name, ci_config)
{
  return ! ci_config.cmake_warnings_disabled?.contains(lib_name)
}

/*
 * TODO: deprecated, migrate the pkgconf_per_src index to use new branch_index
 * or generate a new one together with the branch_index
 *
 * Generate an index that facilitates the operations with the yaml values,
 * avoiding to parse them several times.
 *
 * # pkgconf_per_src index structure:
 *   pkg_src_name : [ packaging_config_name : [ .lib_name .collection ] ]
 *
 *   The index main keys are package source names (i.e gz-cmake3 or gz-harmonic and associated them
 *   another map of packaging configuration names supported as keys (i.e: jammy) with the
 *   list of associated items composed by a map: lib_name (canonical name for the source package)
 *   (and collection)
 *
 *   index[gz-cmake3][jammy] -> [ lib_name: gz-cmake, collection: harmonic ]
 */
void generate_ciconfigs_by_lib(config, pkgconf_per_src_index)
{
  config.collections.each { collection ->
    collection.libs.each { lib ->
      def libName = lib.name
      def branch = lib.repo.current_branch
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
  def ws_checkout_dir = lib_name
  extra_cmd = [extra_cmd, pre_setup_script].findAll({ it != null }).join('\n')

  OSRFLinuxCompilation.create(gz_ci_job, is_testing_enabled(lib_name, ci_config))
  OSRFGitHub.create(gz_ci_job,
                    "gazebosim/${lib_name}",
                    branch,
                    ws_checkout_dir)
  generate_label_by_requirements(gz_ci_job, lib_name, ci_config.requirements, 'docker')
  gz_ci_job.with
  {
    steps {
      shell("""\
            #!/bin/bash -xe

            ${GLOBAL_SHELL_CMD}
            ${extra_cmd}
            export BUILDING_EXTRA_CMAKE_PARAMS="${extra_cmake}"
            export BUILDING_EXTRA_MAKETEST_PARAMS="${extra_test}"
            export BUILDING_SOFTWARE_DIRECTORY="${ws_checkout_dir}"
            export DISTRO=${distro}
            export ARCH=${arch}
            /bin/bash -xe ./scripts/jenkins-scripts/docker/${script_name_prefix}-compilation.bash
            """.stripIndent())
    }
  }
}

void generate_asan_ci_job(gz_ci_job, lib_name, branch, ci_config)
{
  generate_ci_job(gz_ci_job, lib_name, branch, ci_config,
                  '-DGZ_SANITIZER=Address',
                  Globals.MAKETEST_SKIP_GZ,
                  'export ASAN_OPTIONS=check_initialization_order=true:strict_init_order=true')
}

void add_brew_shell_build_step(gz_brew_ci_job, lib_name, ws_checkout_dir)
{
  // ignition formulas does not match the lib name, expand the prefix
  lib_name = lib_name.replaceAll(/^ign-/, 'ignition-')
  gz_brew_ci_job.with
  {
    steps {
      shell("""\
            #!/bin/bash -xe

            export PROJECT_PATH="${ws_checkout_dir}"
            /bin/bash -xe ./scripts/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash "${lib_name}"
            """.stripIndent())
      }
  }
}

void generate_brew_ci_job(gz_brew_ci_job, lib_name, branch, ci_config)
{
  def ws_checkout_dir = lib_name
  OSRFBrewCompilation.create(gz_brew_ci_job,
                             is_testing_enabled(lib_name, ci_config),
                             are_cmake_warnings_enabled(lib_name, ci_config))
  OSRFGitHub.create(gz_brew_ci_job,
                    "gazebosim/${lib_name}",
                    branch,
                    ws_checkout_dir)
  add_brew_shell_build_step(gz_brew_ci_job, lib_name, ws_checkout_dir)
}

void add_win_devel_bat_call(gz_win_ci_job, lib_name, ws_checkout_dir, ci_config)
{
  def script_name_prefix = cleanup_library_name(lib_name)
  def conda_env = ci_config.system.version
  gz_win_ci_job.with
  {
    steps {
      batchFile("""\
            set VCS_DIRECTORY=${ws_checkout_dir}
            if "%CONDA_ENV_NAME%" == "" set CONDA_ENV_NAME=${conda_env}
            if not exist "./scripts/conda/envs/%CONDA_ENV_NAME%" (
              echo "Conda environment %CONDA_ENV_NAME% not found"
              exit 1
            )
            call "./scripts/jenkins-scripts/${script_name_prefix}-default-devel-windows-amd64.bat"
            """.stripIndent())
    }
  }
  // Include the labels in the windows job
  generate_label_by_requirements(gz_win_ci_job, lib_name, ci_config.requirements, 'win')
}

void generate_win_ci_job(gz_win_ci_job, lib_name, branch, ci_config)
{
  def ws_checkout_dir = lib_name
  OSRFWinCompilation.create(gz_win_ci_job,
                            is_testing_enabled(lib_name, ci_config),
                            are_cmake_warnings_enabled(lib_name, ci_config))
  OSRFGitHub.create(gz_win_ci_job,
                    "gazebosim/${lib_name}",
                    branch,
                    ws_checkout_dir)
  add_win_devel_bat_call(gz_win_ci_job,
                         lib_name,
                         ws_checkout_dir,
                         ci_config)
}


String get_debbuilder_name(parsed_yaml_lib, parsed_yaml_packaging)
{
  major_version = parsed_yaml_lib.major_version

  ignore_major_version = parsed_yaml_packaging.linux?.ignore_major_version
  if (ignore_major_version && ignore_major_version.contains(parsed_yaml_lib.name))
    major_version = ""

  return parsed_yaml_lib.name + major_version + "-debbuilder"
}

String generate_linux_install(src_name, lib_name, platform, arch)
{
  def script_name_prefix = cleanup_library_name(src_name)
  def job_name = "${script_name_prefix}-install-pkg-${platform}-${arch}"
  def install_default_job = job(job_name)
  OSRFLinuxInstall.create(install_default_job)
  install_default_job.with
  {
    triggers {
      cron(Globals.CRON_EVERY_THREE_DAYS)
    }

    def dev_package = "lib${src_name}-dev"

    steps {
     shell("""\
           #!/bin/bash -xe

           ${GLOBAL_SHELL_CMD}
           export DISTRO=${platform}
           export ARCH=${arch}
           export INSTALL_JOB_PKG=${dev_package}
           export GZDEV_PROJECT_NAME="${src_name}"
           /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
           """.stripIndent())
    }
  }
  return job_name
}

String generate_brew_install(src_name, lib_name, arch)
{
  def script_name_prefix = cleanup_library_name(src_name)
  def job_name = "${script_name_prefix}-install_bottle-homebrew-${arch}"
  def install_default_job = job(job_name)
  OSRFBrewInstall.create(install_default_job)

  install_default_job.with
  {
    triggers {
      cron('@daily')
    }

    steps {
     shell("""\
           #!/bin/bash -xe

           /bin/bash -x ./scripts/jenkins-scripts/lib/project-install-homebrew.bash ${src_name}
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

  return job_name
}

def generate_debbuilder_job(src_name, ArrayList pre_setup_script_hooks)
{
  def extra_cmd = pre_setup_script_hooks.join('\n')
  assert extra_cmd instanceof String

  def build_pkg_job = job("${src_name}-debbuilder")
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
              ${extra_cmd}
              /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-ignition-debbuild.bash
              """.stripIndent())
      }
  }
}

def pkgconf_per_src_index = [:].withDefault { [:] }
generate_ciconfigs_by_lib(gz_collections_yaml, pkgconf_per_src_index)
/*
 *
 * Loop over each collection, inside each collection loop over the ci configurations assigned
 * and finally loop over each collection library listed.
 *
 */
def branch_index = [:].withDefault { [:] }
gz_collections_yaml.collections.each { collection ->
  collection.ci.configs.each { config_name ->
    def ci_config = gz_collections_yaml.ci_configs.find{ it.name == config_name }
    def distro = ci_config.system.version
    def arch = ci_config.system.arch
    def categories_enabled = ci_config.ci_categories_enabled
    collection.libs.each { lib ->
      def lib_name = lib.name
      def branch_name = lib.repo.current_branch
      def gz_job_name_prefix = lib_name.replaceAll('-','_')
      if (ci_config.exclude.all?.contains(lib_name))
        return

      // Build the branch_index while going through all the libraries to avoid
      // looping twice.
      if (ci_config.system.so == 'linux') {
        platform = distro
      } else if (ci_config.system.so == 'darwin') {
        platform = 'homebrew'
      } else if (ci_config.system.so == 'windows') {
        platform = distro
      }
      branch_index[lib_name][platform] = branch_index[lib_name][platform]?: ['pr':[], 'pr_abichecker':[]]
      if (categories_enabled.contains('pr'))
      {
        branch_index[lib_name][platform]['pr'].contains(branch_name) ?:
          branch_index[lib_name][platform]['pr'] << [branch: branch_name, ci_name: config_name]
      }
      if (categories_enabled.contains('pr_abichecker') &&
         (branch_name != 'main') &&
         (! ci_config.exclude.abichecker?.contains(lib_name)))
      {
        branch_index[lib_name][platform]['pr_abichecker'].contains(branch_name) ?:
          branch_index[lib_name][platform]['pr_abichecker'] << [branch: branch_name, ci_name: config_name]
      }

      // Generate jobs for the library entry being parsed
      if (categories_enabled.contains('stable_branches')) {
        if (ci_config.system.so == 'linux') {
          gz_ci_job = job("${gz_job_name_prefix}-ci-${branch_name}-${distro}-${arch}")
          generate_ci_job(gz_ci_job, lib_name, branch_name, ci_config)
          if (categories_enabled.contains('stable_branches_asan'))
          {
            def gz_ci_asan_job =  job("${gz_job_name_prefix}-ci_asan-${branch_name}-${distro}-${arch}")
            generate_asan_ci_job(gz_ci_asan_job, lib_name, branch_name, ci_config)
            gz_ci_asan_job.with
            {
              triggers {
                scm(Globals.CRON_ON_WEEKEND)
              }
            }

            logging_list['asan_ci'].add(
               [collection: collection.name,
               job_name: gz_ci_asan_job.name])
          }
        } else if (ci_config.system.so == 'darwin') {
          gz_ci_job = job("${gz_job_name_prefix}-ci-${branch_name}-homebrew-${arch}")
          generate_brew_ci_job(gz_ci_job, lib_name, branch_name, ci_config)
        } else if (ci_config.system.so == 'windows') {
          branch_number = branch_name - lib_name
          Globals.gazebodistro_branch = true
          distro_sort_name = get_windows_distro_sortname(ci_config)
          gz_ci_job = job("${gz_job_name_prefix}-${branch_number}-${distro_sort_name}win")
          generate_win_ci_job(gz_ci_job, lib_name, branch_name, ci_config)
          Globals.gazebodistro_branch = false
        } else {
          assert false : "Unexpected config.system.so type: ${ci_config.system.so}"
        }

        gz_ci_job.with
        {
          triggers {
            scm('@daily')
          }
        }

        logging_list['branch_ci'].add(
         [collection: collection.name,
           job_name: gz_ci_job.name])
      } // end of daily category enabled
    }
  }
}

branch_index.each { lib_name, distro_configs ->
  distro_configs.each { distro, branches_with_ciconfig ->
    if (branches_with_ciconfig['pr']) {
      def branch_names = branches_with_ciconfig['pr'].collect { it.branch }.unique()
      // Hack that assumes that pre_setup_script_hook, arch and requirements are equal on all
      // ciconfigs associated to a given branch and distro combination
      def config_name = branches_with_ciconfig['pr'].collect { it.ci_name }.unique()[0]
      def ci_config = gz_collections_yaml.ci_configs.find{ it.name == config_name }
      def script_name_prefix = cleanup_library_name(lib_name)
      def gz_job_name_prefix = lib_name.replaceAll('-','_')
      def arch = ci_config.system.arch
      def ws_checkout_dir = lib_name
      if (ci_config.system.so == 'linux')
      {
        def pre_setup_script = ci_config.pre_setup_script_hook?.get(lib_name)?.join('\n')
        def extra_cmd = pre_setup_script ?: ""
        def gz_ci_job_name = "${gz_job_name_prefix}-ci-pr_any-${distro}-${arch}"
        def gz_ci_any_job = job(gz_ci_job_name)
        OSRFLinuxCompilationAnyGitHub.create(gz_ci_any_job,
                                             "gazebosim/${lib_name}",
                                             is_testing_enabled(lib_name, ci_config),
                                             ENABLE_CPPCHECK,
                                             branch_names)
        generate_label_by_requirements(gz_ci_any_job, lib_name, ci_config.requirements, 'docker')
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
      } else if (ci_config.system.so == 'darwin') {
        // --------------------------------------------------------------
        def gz_brew_ci_any_job_name = "${gz_job_name_prefix}-ci-pr_any-homebrew-amd64"
        def gz_brew_ci_any_job = job(gz_brew_ci_any_job_name)
        OSRFBrewCompilationAnyGitHub.create(gz_brew_ci_any_job,
                                            "gazebosim/${lib_name}",
                                            is_testing_enabled(lib_name, ci_config),
                                            branch_names,
                                            ENABLE_GITHUB_PR_INTEGRATION,
                                            are_cmake_warnings_enabled(lib_name, ci_config))
        add_brew_shell_build_step(gz_brew_ci_any_job, lib_name, ws_checkout_dir)
      } else if (ci_config.system.so == 'windows') {
        distro_sort_name = get_windows_distro_sortname(ci_config)
        Globals.gazebodistro_branch = false
        def gz_win_ci_any_job_name = "${gz_job_name_prefix}-pr-${distro_sort_name}win"
        def gz_win_ci_any_job = job(gz_win_ci_any_job_name)
        Globals.gazebodistro_branch = true
        OSRFWinCompilationAnyGitHub.create(gz_win_ci_any_job,
                                            "gazebosim/${lib_name}",
                                            is_testing_enabled(lib_name, ci_config),
                                            branch_names,
                                            ENABLE_GITHUB_PR_INTEGRATION,
                                            are_cmake_warnings_enabled(lib_name, ci_config))
        add_win_devel_bat_call(gz_win_ci_any_job,
                               lib_name,
                               ws_checkout_dir,
                               ci_config)
        Globals.gazebodistro_branch = false
      }
    }

    if (branches_with_ciconfig['pr_abichecker']) {
      def branch_names = branches_with_ciconfig['pr_abichecker'].collect { it.branch }.unique()
      // Hack that assumes that pre_setup_script_hook, arch and requirements are equal on all
      // ciconfigs associated to a given branch and distro combination
      def config_name = branches_with_ciconfig['pr_abichecker'].collect { it.ci_name }.unique()[0]
      def ci_config = gz_collections_yaml.ci_configs.find{ it.name == config_name }
      def pre_setup_script = ci_config.pre_setup_script_hook?.get(lib_name)?.join('\n')
      def extra_cmd = pre_setup_script ?: ""
      def arch = ci_config.system.arch
      def gz_job_name_prefix = lib_name.replaceAll('-','_')
      def abi_job_name = "${gz_job_name_prefix}-abichecker-any_to_any-ubuntu-${distro}-${arch}"
      def abi_job = job(abi_job_name)
      OSRFLinuxABIGitHub.create(abi_job)
      GenericAnyJobGitHub.create(abi_job,
                        "gazebosim/${lib_name}",
                        branch_names)
      generate_label_by_requirements(abi_job, lib_name, ci_config.requirements, 'docker')
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
  }
}

pkgconf_per_src_index.each { pkg_src, pkg_src_configs ->
  // For each entry in the index perform two steps:
  //  1. Generate Linux builders artifacts (-source and -debbuilders)
  //  2. Generate all -ci-install- jobs looping in the index entries

  // 1. Generate Linux builders
  // All entries are the same for canonical_lib_name, pick the first
  def canonical_lib_name = pkg_src_configs.values()[0].lib_name[0]
  def linux_ciconfigs = gz_collections_yaml.packaging_configs.findAll{
    it.name in pkg_src_configs.keySet() &&
    it.system.so == 'linux'}
  // Collect the pre_setup_script_hooks defined in all the linux distributions
  // Each distribution commands are joined in a signle item.
  // Unique + findall { it } should clean up all null values
  def pre_setup_script_hooks = linux_ciconfigs.findAll {
    it.keySet().contains('pre_setup_script_hook')
  }.collect {
    it -> it.pre_setup_script_hook.get(canonical_lib_name)
  }.flatten().unique().findAll { it }

  def exclusion_list = linux_ciconfigs.findAll {
    it.keySet().contains('exclude')
  }.collect{
    it -> it.exclude
  }

  if (linux_ciconfigs && !exclusion_list.contains(canonical_lib_name))
  {
    // - DEBBUILD jobs -------------------------------------------------
    generate_debbuilder_job(pkg_src,
      pre_setup_script_hooks
    )
    // - SOURCE jobs ---------------------------------------------------
    def gz_source_job = job("${pkg_src}-source")
    OSRFSourceCreation.create(gz_source_job, [
      PACKAGE: pkg_src,
      SOURCE_REPO_URI: "https://github.com/gazebosim/${canonical_lib_name}.git"])
    OSRFSourceCreation.call_uploader_and_releasepy(gz_source_job,
      canonical_lib_name,
      'repository_uploader_packages',
      '_releasepy')
  }

  // 2. Generate all -ci-install jobs
  pkg_src_configs.each { pkg_src_config ->
    def config_name = pkg_src_config.getKey()
    def pkg_config = gz_collections_yaml.packaging_configs.find{ it.name == config_name }
    if (pkg_config.exclude?.contains(canonical_lib_name))
      return
    def pkg_system = pkg_config.system
      // - CI-INSTALL jobs ------------------------------------------------
    pkg_system.arch.each { arch ->
      def install_job_name = ""
      if (pkg_system.so == 'linux') {
        install_job_name = generate_linux_install(
          pkg_src,
          canonical_lib_name,
          pkg_system.version,
          arch)
      } else if (pkg_system.so == 'darwin') {
        install_job_name = generate_brew_install(
          pkg_src,
          canonical_lib_name,
          arch)
      } else {
        assert("Unexpected pkg_system.so: " + pkg_system.so)
      }
      pkg_src_config.getValue().each { index_entry ->
        logging_list['install_ci'].add(
          [collection: index_entry.collection,
           job_name: install_job_name])
      }
    }
  }
}

def File log_file
if (WRITE_JOB_LOG) {
  log_file = new File("logs/generated_jobs.txt")
}

def collection_job_names = [:].withDefault {[]}
logging_list.each { log_type, items ->
  items.each {
    collection_job_names[it.collection] << it.job_name
    if (WRITE_JOB_LOG) {
      log_file.append("${log_type} ${it.collection} ${it.job_name}\n") }
    }
}

/*
 * -------------------------------------------------------
 * DASHBOARD VIEWS
 * -------------------------------------------------------
 */
collection_job_names.each { collection_name, job_names ->
  // TODO: change ign by gz when testing is ready
  dashboardView("gz-${collection_name}")
  {
    jobs {
      job_names.each { jobname ->
        name(jobname)
      }
      def collection = gz_collections_yaml.collections.find { it.name == collection_name }
      if (collection.packaging?.linux?.nightly) {
        collection.libs.each { lib ->
          name(get_debbuilder_name(lib, collection.packaging))
        }
      }
    }

    columns {
      status()
      weather()
      name()
      testResult(0)
      lastSuccess()
      lastFailure()
      lastDuration()
      buildButton()

    }

    bottomPortlets {
      jenkinsJobsList {
          displayName('Jenkins jobs list')
      }
    }

    configure { view ->
      view / columns << "hudson.plugins.warnings.WarningsColumn" (plugin: 'warnings@5.0.1')

      def topPortlets = view / NodeBuilder.newInstance().topPortlets {}

      topPortlets << 'hudson.plugins.view.dashboard.core.UnstableJobsPortlet' {
          id createPortletId()
          name 'Failing jobs'
          showOnlyFailedJobs 'true'
          recurse 'false'
      }
    }
  }
}
