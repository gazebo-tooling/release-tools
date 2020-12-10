import _configs_.*
import javaposse.jobdsl.dsl.Job

def NO_TESTING = false
def NO_BRANCHES = []
def NO_GITHUB_PR_INTEGRATION = false

def update_vcpkg_snapshot_job = job("_vcpkg_update_snapshot")
OSRFWinBase.create(update_vcpkg_snapshot_job)
update_vcpkg_snapshot_job.with
{
    parameters {
        nodeParam('TARGET_NODE') {
            description('Node to be updated')
        }
    }

    steps
    {
      systemGroovyCommand("""\
        job_description = 'RTOOLS_BRANCH: ' +
            build.buildVariableResolver.resolve('RTOOLS_BRANCH') + '<br />' +
            'TARGET_NODE: ' +
            '<b>' + build.buildvariableresolver.resolve('TARGET_NODE') + '</b>'
        build.setdescription(job_description)
      """.stripIndent())

      batchFile("""\
            call "%WORKSPACE%/scripts/jenkins-scripts/vcpkg-bootstrap.bat
            """.stripIndent())
    }
}


def ignition_testing_software = 'gazebo'
def testing_vcpkg_job = job("_vcpkg_testing_snapshot")
OSRFWinCompilationAnyGitHub.create(testing_vcpkg_job,
                                  "ignitionrobotics/ign-${ignition_testing_software}",
                                  NO_TESTING, NO_BRANCHES, NO_GITHUB_PR_INTEGRATION)
testing_vcpkg_job.with
{
    parameters {
        stringParam('VCPKG_SNAPSHOT', '','vcpkg tag/release to test')
    }

    steps
    {
      label "win_testing"

      batchFile("""\
            call "%WORKSPACE%/scripts/jenkins-scripts/vcpkg-bootstrap.bat" || exit /B %errorlevel%
            call "%WORKSPACE%/scripts/jenkins-scripts/ign_${ignition_testing_software}-default-devel-windows-amd64.bat"
            """.stripIndent())
    }
}
