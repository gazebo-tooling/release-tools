package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - priorioty 100
    - keep only 15 builds
    - compiler warning
    - test results
    - cppcheck results
*/
class OSRFLinuxCompilation extends OSRFLinuxBase
{   
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)
    job.with
    {
      priority 100

      logRotator {
        numToKeep(15)
      }

      publishers 
      {
         // compilers warnings
         warnings(['GNU C Compiler 4 (gcc)'])

         // junit plugin is not implemented. Use configure for it
         configure { project ->
            project / publishers << 'hudson.tasks.junit.JUnitResultArchiver' {    
                 testResults('build/test_results/*.xml')
                 keepLongStdio false
                 testDataPublishers()             
            }
         }

         // cppcheck is not implemented. Use configure for it
         configure { project ->
	         def cppcheck_header = project / publishers / 'org.jenkinsci.plugins.cppcheck.CppcheckPublisher'
           cppcheck_header / cppcheckconfig {
             pattern('build/cppcheck_results/*.xml')
             ignoreBlankFiles true
             allowNoReport false
           }
         }     
      }
    }
  }
}
