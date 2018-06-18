import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"

// List of repositories that have a counter part -release repo
// under Open Robotics control that host metadata for debian builds
release_repo_debbuilds = [ 'opensplice' ]

release_repo_debbuilds.each { software ->
  // --------------------------------------------------------------
  // 1. Create the deb build job
  def build_pkg_job = job("${software}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)

  build_pkg_job.with
  {
    // use only the most powerful nodes
    label "large-memory"

    steps {
      shell("""\
            #!/bin/bash -xe

            export USE_ROS_REPO=true
            /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-debbuild.bash
            """.stripIndent())
    }
  }
}
