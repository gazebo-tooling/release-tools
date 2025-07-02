import _configs_.*
import javaposse.jobdsl.dsl.Job

def DSL_WORKDIR = 'scripts/jenkins-scripts/dsl/'

def dsl_configs = [
  "_dsl_brew_release": "brew_*.dsl",
  "_dsl_core": "core*.dsl",
  "_dsl_debian": "debian*.dsl",
  "_dsl_extra": "extra.dsl",
  "_dsl_gazebo_libs": "gazebo_libs.dsl",
  "_dsl_gzdev": "gzdev*.dsl",
  "_dsl_ignition_collection": "ignition_collection.dsl",
  "_dsl_ros_gz_bridge": "ros_gz_bridge.dsl",
  "_dsl_test": "test.dsl"
]

// Generate all listed _dsl* jobs
dsl_configs.each { job_name, dsl_scripts ->
  OSRFDslBase.create(job(job_name), "${DSL_WORKDIR}${dsl_scripts}")
}

// Generate a triggerer for all _dsl jobs
// This includes a `_dsl_generate_all_dsls` job, which should be created manually
def dsl_triggerer = job("_dsl_trigger_all_dsls")

Globals.rtools_description = false
OSRFBase.create(dsl_triggerer)
dsl_triggerer.with {
  label(Globals.nontest_label("built-in"))

  parameters {
    stringParam('RUN_DESCRIPTION', 'Automatic update',
                'Description about why the run started')
  }

  triggers {
    scm('@daily') {
      ignorePostCommitHooks(false)
    }
  }

  steps {
    systemGroovyCommand("build.setDescription(build.buildVariableResolver.resolve('RUN_DESCRIPTION'));")
  }

  publishers {
    downstreamParameterized {
      trigger((dsl_configs.keySet() + '_dsl_generate_all_dsls') as List) {
        condition('ALWAYS')
        parameters {
          currentBuild()
        }
      }
    }
  }
}
