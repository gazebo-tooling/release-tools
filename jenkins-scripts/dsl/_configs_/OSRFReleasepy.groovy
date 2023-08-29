package _configs_

import javaposse.jobdsl.dsl.Job
import _configs_.Globals

class OSRFReleasepy
{
  static void create(Job job, Map default_params = [:])
  {
    // Base class for the job
    OSRFUNIXBase.create(job)

    job.with
    {
      label Globals.nontest_label("master")

      parameters
      {
        stringParam("PACKAGE",
                    default_params.find{ it.key == "PACKAGE"}?.value,
                    "Package name to be built")
        stringParam("VERSION",
                    default_params.find{ it.key == "VERSION"}?.value,
                    "Packages version to be built or nightly (enable nightly build mode)")
        stringParam("RELEASE_VERSION",
                    default_params.find{ it.key == "RELEASE_VERSION"}?.value,
                    "Packages release version")
        stringParam("SOURCE_TARBALL_URI",
                    default_params.find{ it.key == "SOURCE_TARBALL_URI"}?.value,
                    "URL to the tarball containing the package sources")
        stringParam("RELEASE_REPO_BRANCH",
                    default_params.find{ it.key == "RELEASE_REPO_BRANCH"}?.value,
                    "Branch from the -release repo to be used")
        stringParam("UPLOAD_TO_REPO",
                    default_params.find{ it.key == "UPLOAD_TO_REPO"}?.value,
                    "OSRF repo name to upload the package to: stable | prerelease | nightly | none (for testing proposes)")
        stringParam("EXTRA_OSRF_REPO",
                    default_params.find{ it.key == "OSRF_REPOS_TO_USE"}?.value,
                    "OSRF repos name to use when building the package")
  
        booleanParam('DRY_RUN',
                    default_params.find{ it.key == "DRY_RUN"}?.value,
                    'run a testing run with no effects')
      }

      steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b>' + build.buildVariableResolver.resolve('PACKAGE') + '/' +
          '' + build.buildVariableResolver.resolve('VERSION') + '-' +
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
            set +x # keep password secret
            PASS=\$(cat \$HOME/build_pass)

            dry_run_str=""
            if \$DRY_RUN; then
              dry_run_str="--dry-run"
            fi

            extra_osrf_repo=""
            if [[ -n \${EXTRA_OSRF_REPO} ]]; then
              extra_osrf_repo="--extra-osrf-repo \${EXTRA_OSRF_REPO}"
            fi

            echo "releasing \${n} (from branch \${src_branch})"
              python3 ./scripts/release.py \${dry_run_str} "\${PACKAGE}" "\${VERSION}" "\${PASS}" \${extra_osrf_repo} \
                      --release-repo-branch \${RELEASE_REPO_BRANCH} \
                      --upload-to-repo \${UPLOAD_TO_REPO} > log || echo "MARK_AS_UNSTABLE"
            echo " - done"
            """.stripIndent())
      }
    }
  }
}
