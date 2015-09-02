evaluate(new File("./_configs.groovy"))

def supported_distros = [ 'trusty' ]
def supported_arches = [ 'amd64' ]

supported_distros.each { distro ->
  supported_arches.each { arch ->    

    // Create the default ci jobs
    def ci_default_job = job("gazebo-ci_mentor2_v2-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(ci_default_job)

    ci_default_job.with
    {
        scm {
          hg('http://bitbucket.org/osrf/gazebo','mentor2_v2')
        }

        triggers {
          scm('*/5 * * * *') 
        }

        steps {
          shell("/bin/bash -x ./scripts/jenkins-scripts/docker/_mentor2_compilation.bash")
        }
     }
  }
}
