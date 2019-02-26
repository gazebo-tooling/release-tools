import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION COLLECTIONS
ignition_collections = [ 'acropolis' ]


// Testing compilation from source

ignition_collections.each { ign_collection ->


  def ignition_win_ci_job = job(("ign_${ign_collection}-ci-win")
  Globals.gazebodistro_branch = true
  OSRFWinCompilation.create(ignition_win_ci_job, enable_testing(ign_sw))
  OSRFBitbucketHg.create(ignition_win_ci_job,
                            "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}/",
                            "${branch}", "ign-${ign_sw}")


  OSRFWinCompilationAny.create(ignition_win_ci_any_job,
                               "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}",
                               false)
  ignition_win_ci_any_job.with
  {
      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }



// DEBBUILD: linux package builder
// --------------------------------------------------------------
ignition_collections.each { ign_collection ->
  def build_pkg_job = job("ign-${ign_collection}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)
  build_pkg_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-ignition-debbuild.bash
              """.stripIndent())
      }
  }
}

