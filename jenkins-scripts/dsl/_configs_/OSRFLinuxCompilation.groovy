package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxBase
  -> Genericompilation

  Implements:
    - compiler warning
*/
class OSRFLinuxCompilation extends OSRFLinuxBase
{
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)

    /* Properties from generic compilations */
    GenericCompilation.create(job)

    job.with
    {
      publishers
      {
         // compilers warnings
         warnings(['GNU C Compiler 4 (gcc)'])

         // cppcheck is not implemented. Use configure for it
         configure { project ->
             project / publishers / 'org.jenkinsci.plugins.cppcheck.CppcheckPublisher' / cppcheckConfig {
               pattern('build/cppcheck_results/*.xml')
               ignoreBlankFiles true
               allowNoReport false

               configSeverityEvaluation {
                     threshold 0
                     newThreshold()
                     failureThreshold()
                 newFailureThreshold()
                 healthy()
                 unHealthy()
                 severityError true
                 severityWarning true
                 severityStyle true
                 severityPerformance true
                 severityInformation true
                 severityNoCategory false
                 severityPortability false
               }

                     configGraph {
                 xSize 500
                 ySize 200
                 numBuildsInGraph 0
                 displayAllErrors true
                 displayErrorSeverity false
                 displayWarningSeverity false
                 displayStyleSeverity false
                 displayPerformanceSeverity false
                 displayInformationSeverity false
                 displayNoCategorySeverity false
                 displayPortabilitySeverity false
               } // end of configGraph
             } // end of cppcheckconfig
           } // end of configure
      } // end of publishers
    } // end of job
  } // end of method createJob
} // end of class
