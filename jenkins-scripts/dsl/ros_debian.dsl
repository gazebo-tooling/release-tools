import _configs_.*
import javaposse.jobdsl.dsl.Job


Globals.default_emails = "jrivero@osrfoundation.org, leo@alaxarxa.net, jochen@sprickerhof.de"
supported_arches = [ 'amd64' ]
ros_debian_supported_distros = [ ubuntu : ["trusty"],
                                 debian : [ "jessie","sid"] ]

supported_arches.each { arch ->
  ros_debian_supported_distros.each { linux, distros ->
      distros.each { distro ->
        def ci_job = job("ros_debian_science-ci-default-${distro}-${arch}")
        OSRFLinuxBase.create(ci_job)
        ci_job.with
        {
          authorization
          {
              permission('hudson.model.Item.Build', 'jochen')
              permission('hudson.model.Item.Build', 'leo')
          }

          steps
          {
              triggers {
                cron('@daily')
              }

              shell("""\
                    #!/bin/bash -xe

                    export LINUX_DISTRO="${linux}"
                    export DISTRO="${distro}"
                    export ARCH="${arch}"

                    /bin/bash -xe ./scripts/jenkins-scripts/docker/ros_debian-installation.bash
                    """.stripIndent())
          }
        }
     }
  }
}
