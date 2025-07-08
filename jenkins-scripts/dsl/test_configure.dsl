import _configs_.*
import javaposse.jobdsl.dsl.Job

def test_basic = job("_configure_block_linuxpkg")
OSRFLinuxBuildPkgBase.create(test_basic)

def test_source = job("_source_creation")
OSRFSourceCreation.create(test_source, [
  PACKAGE: 'foo',
  SOURCE_REPO_URI: "https://github.com/gazebosim/foo.git"])
OSRFSourceCreation.call_uploader_and_releasepy(test_source,
  'foo',
  'repository_uploader_packages',
  '_releasepy')

def unix_base = job("_unixbase_creation")
OSRFUNIXBase.create(unix_base)

def test_son = job("_configure_block_son")
test_son.with 
{
    configure { project ->
        project / 'publishers' << 'org.jenkinsci.plugins.postbuildscript.PostBuildScript'(plugin: 'postbuildscript@3.4.1-695.vf6b_0b_8053979') {
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
                        stopOnFailure(false)
                    }
                }
                markBuildUnstable(false)
            }
        }
    }
}
