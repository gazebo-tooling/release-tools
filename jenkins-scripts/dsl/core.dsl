import _configs_.*
import javaposse.jobdsl.dsl.Job

def update_vcpkg_snapshot_job = job("_vcpkg_update_snapshot")
OSRFWinBase.create(update_vcpkg_snapshot_job)
update_vcpkg_snapshot_job.with
{
    steps
    {
      batchFile("""\
            call "./scripts/jenkins-scripts/vcpkg-update-snapshot.bat
            """.stripIndent())
    }
}


def ignition_testing_software = 'gazebo'
def testing_vcpkg_job = job("_vcpkg_testing_snapshot")
OSRFWinCompilationAnyGitHub.create(testing_vcpkg_job,
                                  "ignitionrobotics/ign-${ignition_testing_software}",
                                  false)
update_vcpkg_snapshot_job.with
{
    parameters {
        stringParam('VCPKG_SNAPSHOT', '','vcpkg tag/release to test')
    }

    steps
    {
      batchFile("""\
            call "./scripts/jenkins-scripts/ign-${ignition_testing_software}-default-devel-windows-amd64.bat"
            """.stripIndent())
    }
}
