package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - priorioty 300
    - keep only 10 last artifacts
    - concurrent builds
    - parameters:
        - PACKAGE
        - VERSION
        - RELEASE_VERSION
        - DISTRO
        - ARCH
        - SOURCE_TARBALL_URI
        - RELEASE_REPO_BRANCH
        - PACKAGE_ALIAS 
        - UPLOAD_TO_REPO
    - publish artifacts
    - launch repository_ng
*/
class OSRFLinuxBuildPkg extends OSRFLinuxBase

{  
  
  File token_file = new File(build.getEnvVars()['HOME'] + '/remote_token')

  static void create(Job job)
  {
    OSRFLinuxBase.create(job)

    if (! token_file.exists()) {
      println("!!! token file was not found for setting the remote password")
      println("check your filesystem in the jenkins node for: ")
      println(token_file)
      
      System.exit(1)
    }

    job.with
    {
      priority 300

      logRotator {
        artifactNumToKeep(10)
      }

      concurrentBuild(true)

      throttleConcurrentBuilds {
	maxPerNode(1)
	maxTotal(5)
      }

      // remote calls don't have DSL implementation
      configure { project ->
           project {
              authtoken token_file.text()
           }
      }
      
      parameters {
        textParam("PACKAGE",null,"Package name to be built")
        textParam("VERSION",null,"Packages version to be built")
        textParam("RELEASE_VERSION", null, "Packages release version")
        textParam("DISTRO", null, "Ubuntu distribution to build packages for")
        textParam("ARCH", null, "Architecture to build packages for")
        textParam("SOURCE_TARBALL_URI", null, "URL to the tarball containing the package sources")
        textParam("RELEASE_REPO_BRANCH", null, "Branch from the -release repo to be used")
        textParam("PACKAGE_ALIAS ", null, "If not empty, package name to be used instead of PACKAGE")
        textParam("UPLOAD_TO_REPO", null, "OSRF repo name to upload the package to")
      }

      steps {
        systemGroovyCommand("""\
          build.setDescription(
          build.buildVariableResolver.resolve(VERSION); + '-' + 
          build.buildVariableResolver.resolve(RELEASE_VERSION); + '<br \\>' +
          '(' + build.buildVariableResolver.resolve(DISTRO); + '/' + 
                build.buildVariableResolver.resolve(ARCH); + ')' + '<br \\>' + 
          <br \\> +
          RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )
      }

      publishers {
        archiveArtifacts('pkgs/*')

        downstreamParameterized {
	  trigger('repository_uploader_ng') {
	    condition('SUCCESS')
	    currentBuild()
	    parameters {
	      predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
	    }
	  }
        }
      }
    } // end of job
  } // end of method createJob
} // end of class
