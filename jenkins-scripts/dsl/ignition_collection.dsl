import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION COLLECTIONS
ignition_collections = [ 'acropolis' ]

// Testing compilation from source
ignition_collections.each { ign_collection ->
  def ignition_win_ci_job = job("ign_${ign_collection}-ci-win")
  Globals.gazebodistro_branch = true
  OSRFWinCompilation.create(ignition_win_ci_job, false)
  ignition_win_ci_job.with
  {
      steps {
        batchFile("""\
              set IGNITION_COLLECTION=${ign_collection}
              call "./scripts/jenkins-scripts/ign_collection-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
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

