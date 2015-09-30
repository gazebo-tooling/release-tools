package _configs_

import javaposse.jobdsl.dsl.Job

/*
   -> GenericCompilation

   Implements:
     - test results (build\test_results)
     - email
*/

class GenericUNIXCompilation
{
   static void create(Job job)
   {
 
     GenericCompilation.create(job)

     job.with
     {
        publishers
        {
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
   } // end of create
} // end of class
