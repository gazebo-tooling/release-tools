package _configs_

import javaposse.jobdsl.dsl.Job

class OSRFGitHub
{
  static void create(Job job, String repo,
                              String rev    = 'master',
                              String subdir = 'NOT-DEFINED-USE-DEFAULT')
  {
    repo = Globals.ign2gz(repo)

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
          branch(rev)
          extensions {
            relativeTargetDirectory(subdir)
          }
        }
      }
    }
  }
}
