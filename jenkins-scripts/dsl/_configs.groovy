import javaposse.jobdsl.dsl.Job
  
/* 
  Implements:
     - description
*/
public class OSRFBase
{
   static def create(Job job)
   {
     description 'Automatic generated job by DSL jenkins. Please do not edit manually'
   }
}

/*
  Implements:
  - run on docker
  - RTOOLS parameter + description tag in job
  - colorize ansi output
*/
public class OSRFLinuxBase extends OSRFBase
{
   static def create(Job job)
   {
     super.create(job)
     job.with 
     {
         label: docker
         
         parameters { stringParam('RTOOLS_BRANCH','default','release-tool branch to use') }

         steps 
         {
           systemGroovyCommand("build.setDescription('RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));")
           
           shell("""
                 [[ -d ./scripts ]] &&  rm -fr ./scripts
                 hg clone http://bitbucket.org/osrf/release-tools scripts -b ${RTOOLS_BRANCH} 
               """)
         }

         wrappers {
           colorizeOutput()
        }
     }
   }
}

/*
  Implements:
    - compiler warning
*/
public class OSRFLinuxCompilation extends OSRFLinuxBase
{   
  static def create(Job job)
  {
    super.create(job)
    job.with
    {
      publishers {
         warnings(['GNU C Compiler 4 (gcc)'])
      }
    }
  }
}
