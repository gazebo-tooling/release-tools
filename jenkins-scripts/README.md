# Jenkins scripts

We use [DSL Jenkins plugin](https://plugins.jenkins.io/job-dsl/) to allow us to manage and define the jobs as human readable code. You can find the different job configs under the [`dsl`](./dsl/) folder. 

## Useful links
- [List of installed plugins in Jenkins](https://github.com/osrf/chef-osrf/blob/latest/cookbooks/osrfbuild/attributes/plugins.rb)
- [Jenkins DSL API docs](https://jenkinsci.github.io/job-dsl-plugin/)
- [Jenkins DSL Wiki](https://github.com/jenkinsci/job-dsl-plugin/wiki) 


## Local testing

To test locally the build of the different `dsl` jobs you need the following: 

1. Download `job_dsl-core.standalone.jar` to be able to build the dsl jobs locally.  [Download here](https://repo.jenkins-ci.org/artifactory/releases/org/jenkins-ci/plugins/job-dsl-core/1.77/)
2. Add the downloaded file to a folder not tracked in the repo.
3. In the terminal execute: 
``` bash
java -jar `PATH/job_dsl-core.jar` `PATH/job_name.dsl`
```
For more information go [here](https://github.com/jenkinsci/job-dsl-plugin/wiki/User-Power-Moves#run-a-dsl-script-locally).

## Development workflow

1. Make changes locally and test that it builds correctly.
2. Push changes to a specific branch in `release-tools`
3. Go to seed job for the job you wanna test (usually you would use [`_dsl_test`](https://build.osrfoundation.org/job/_dsl_test/) to not affect the jobs in production) and build with the parameter pointing to your new custom branch in the `RTOOLS_BRANCH` parameter. 
4. After it builds correctly, you will have generated jobs with the changes you implemented. You can use and modify the generated job. 

> WARNING! : Running the _dsl job for a specific job that it's not `test` will modify the configuration for production. You should always aim to utilize `test` jobs. 

## :arrow_forward: Playbook

### XML injection into DSL 
Sometimes we find a situation where we have a plugin installed that it's not mapped to DSL and you may think: How can I use this plugin?. 

There is a feature called [configure blocks](https://github.com/jenkinsci/job-dsl-plugin/wiki/The-Configure-Block) that allows us to represent xml job configs as DSL. [Here](https://github.com/gazebo-tooling/release-tools/blob/9fbfe60133d2b7b8b280b92f7c563dc64c8367a5/jenkins-scripts/dsl/_configs_/OSRFUNIXBase.groovy#LL83C1-L92C10) is an example of it's usage with the retryBuild plugin, where `checkRegexp(true)` gets converted into `<checkRegexp>true</checkRegexp>` and the  hierarchy of the definition is respected, so `checkRegexp` exists as a child of `com.chikli.hudson.plugin.naginator.NaginatorPublisher` in the XML definition.

To check what are the corresponding names for the XML tags you can refer to the plugin documentation or as an alternative you can manually modify the job to add the information you want and then go to `https://build.osrfoundation.com/job/myjob/config.xml` and match the XML there in the DSL config. 
