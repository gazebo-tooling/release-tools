job('roscygwin-ci_daily-cygwin64') {
    label('cygwin')
    
    scm {
        hg('http://bitbucket.org/osrf/release-tools','${RTOOLS_BRANCH}')
    }    
    triggers {
       cron('@daily')
    }    
    parameters {
       stringParam('RTOOLS_BRANCH','default','release-tool branch to use')
    }    
    steps {
       systemGroovyCommand("build.setDescription('RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));")
       shell("jenkins-scripts/cygwin/_ros1_roscygwin_compilation.bash")
    }
    publishers {
       textFinder(/failed/, '', true, false, false)
    }
    wrappers {
        colorizeOutput()
    }
}
