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
  static void create(Job job, enable_testing = true,
                              enable_cppcheck = true)
  {
    OSRFLinuxBase.create(job)

    job.with
    {
      // preclean of build/*_results. Please see
      // https://github.com/ignition-tooling/release-tools/issues/75"
      // to know why preBuildCleanup is not working and the use
      // of shell as workaround
      // wrappers {
      //      preBuildCleanup {
      //          includePattern('build/*_results/')
      //          includePattern('*_results/')
      //          deleteDirectories()
      //      }
      // workaround
      steps {
        shell("""\
             #!/bin/bash -xe

             echo "Workaround for cleaning up workspace"
             echo "check: https://github.com/ignition-tooling/release-tools/issues/75"

             sudo rm -fr "\${WORKSPACE}/build/*_results"
             sudo rm -fr "\${WORKSPACE}/*_results"
             """.stripIndent())
      }
    }

    /* Properties from generic compilations */
    GenericCompilation.create(job, enable_testing)

    job.with
    {
      publishers
      {
          configure { project ->
            project / publishers / 'io.jenkins.plugins.analysis.core.steps.IssuesRecorder' {
              analysisTools {
                'io.jenkins.plugins.analysis.warnings.Gcc4' {
                  id()
                  name()
                  pattern()
                  reportEncoding()
                  skipSymbolicLinks(false)
                }
                'io.jenkins.plugins.analysis.warnings.Cmake' {
                  id()
                  name()
                  pattern()
                  reportEncoding()
                  skipSymbolicLinks(false)
                }
              }

              sourceCodeEncoding()
              ignoreQualityGate(false)
              ignoreFailedBuilds(true)
              referenceJobName()
              healthy(0)
              unhealthy(0)
              minimumSeverity {
                name('LOW')
              }
              filters { }
              isEnabledForFailure(false)
              isAggregatingResults(false)
              isBlameDisabled(false)

              qualityGates {
                'io.jenkins.plugins.analysis.core.util.QualityGate' {
                  threshold(1)
                  type('TOTAL')
                  status('WARNING')
                }
              }
            }
         }

         if (enable_cppcheck)
         {
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
                 severityNoCategory true
                 severityPortability true
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
         } // end of enable_cppcheck
      } // end of publishers
    } // end of job
  } // end of method createJob
} // end of class
