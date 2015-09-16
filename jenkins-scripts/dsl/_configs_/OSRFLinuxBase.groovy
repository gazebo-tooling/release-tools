package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
  - run on docker
  - RTOOLS parameter + description tag in job
  - colorize ansi output
*/
class OSRFLinuxBase extends OSRFBase
{
   static void create(Job job)
   {
     OSRFBase.create(job)
     job.with 
     {
         label "docker"
         
         parameters { stringParam('RTOOLS_BRANCH','default','release-tool branch to use') }

         steps 
         {
           systemGroovyCommand("build.setDescription('RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));")
           
           shell("""
#!/bin/bash -x
[[ -d ./scripts ]] &&  rm -fr ./scripts
hg clone http://bitbucket.org/osrf/release-tools scripts -b \${RTOOLS_BRANCH} 
               """)
        }

         wrappers {
           colorizeOutput()
        }
     }
   }
}
