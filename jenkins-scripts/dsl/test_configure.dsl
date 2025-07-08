import _configs_.*
import javaposse.jobdsl.dsl.Job

def test_basic = job("_configure_block_basic")
test_basic.with
{
  publishers
  {
    configure { project ->
      project / 'org.jenkinsci.plugins.postbuildscript.PostBuildScript' << {
         config {
           scriptFiles()
           groovyScripts()
            buildSteps {
              'org.jenkinsci.plugins.postbuildscript.model.PostBuildStep' {
                results {
                  string('SUCCESS')
                  string('NOT_BUILT')
                  string('ABORTED')
                  string('FAILURE')
                  string('UNSTABLE')
                }      
                buildSteps {
                  'hudson.tasks.Shell' {
                    command("""echo fee"
                      """.stripIndent())
                    configuredLocalRules()
                  }
                }
                stopOnFailure(false)
              }
            }
          }
        }
    }
  }
}

def test_son = job("_configure_block_son")
test_son.with 
{
    configure { project ->
        project / 'publishers' << 'org.jenkinsci.plugins.postbuildscript.PostBuildScript' {
            config {
                scriptFiles()
                groovyScripts()
                buildSteps {
                    'org.jenkinsci.plugins.postbuildscript.model.PostBuildStep' {
                        results {
                            string('SUCCESS')
                            string('NOT_BUILT')
                            string('ABORTED')
                            string('FAILURE')
                            string('UNSTABLE')
                        }
                        role('BOTH')
                        buildSteps {
                            'hudson.tasks.Shell' {
                                command('echo foo')
                                configuredLocalRules()
                            }
                        }
                    }
                }
                markBuildUnstable(false)
            }
        }
    }
  }
