doesntWork = new File(".").getCanonicalPath()
println "1: " + doesntWork

println "2: " + new File(".").absolutePath

def scriptDir = new File(getClass().protectionDomain.codeSource.location.path).parent
println "3: " + scriptDir

def scriptDir2 = getClass().protectionDomain.codeSource.location.path
println "4: " + scriptDir2

URL scriptUrl = getClass().classLoader.resourceLoader
    .loadGroovySource(getClass().name)

println "5: " + scriptUrl

def current_dir = new File(getClass().protectionDomain.codeSource.location.path).parent
def config_file = current_dir + "_configs.groovy"
evaluate(new File(config_file))

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
