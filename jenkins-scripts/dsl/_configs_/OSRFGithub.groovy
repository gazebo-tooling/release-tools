package _configs_

import javaposse.jobdsl.dsl.Job

class OSRFGithub
{
  static void create(Job job, String repo,
                              String rev    = 'master',
                              String subdir = 'NOT-DEFINED-USE-DEFAULT',
                              String installName = 'Default')
  {
    String software_name = repo.tokenize('/').last()

    if (subdir == 'NOT-DEFINED-USE-DEFAULT')
      subdir = software_name

    job.with
    {
      scm {
        git {
          remote {
            github(repo)
          }
          extensions {
            relativeTargetDirectory(installName)
          }
          branch(rev)
        }
      }
    }
  }
}
