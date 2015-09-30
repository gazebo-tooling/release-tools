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
        } // end of publishers
      } // end of job
   } // end of create
} // end of class
