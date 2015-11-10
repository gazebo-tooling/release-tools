package _configs_

import javaposse.jobdsl.dsl.Job

/*
   -> GenericCompilation

   Implements:
     - test results (build\test_results)
*/

class GenericUNIXCompilation
{
   static void create(Job job)
   {
 
     GenericCompilation.create(job)

   } // end of create
} // end of class
