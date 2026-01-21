# Jenkins scripts

The release-tools repository uses the [DSL Jenkins plugin](https://plugins.jenkins.io/job-dsl/) to allow us to programmatically generate the job configuration (configuration as code).  You can find the different job configs under the [`dsl`](./dsl/) folder.

## Conda local builder for Windows

### Prerequisites

The installation of Visual Studio 2019 needs to be peformed before using the local builder.
Same [ROS 2 instructons](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html#install-visual-studio) are valid here.


### Usage of `local_build.py`

The [local_build.py](local_build.py) script is used to reproduce Jenkins builds for Windows, specifically supporting Pixi builds.

### Running the script

> [!IMPORTANT]
> The script needs to be run from a Windows command pront (Batch/DOS). Can not work under powershell or
> git bash or other shells.

To run the script, use the following command:

```bat
python3 [local_build.py](local_build.py) <jenkins-bat-script> <gz-sources> [--reuse-dependencies-environment] [-j <make_jobs>]
```

### Arguments

- `jenkins-bat-script`: The script to run from the files in release-tools/jenkins-scripts/gz_*.bat
- `sources`: Local checkout of the gazebo library sources
- `--reuse-dependencies-environment` (optional): Reuse the Pixi build environment created in the initial run (useful for testing code changes).
- `-j`, `--jobs` (optional): MAKE_JOBS variable value to apply

### Example

Use case: reproducing a gz-math pull request for the branch my-testing-branch.

```bat
git clone -b my-testing-branch C:\Users\foo\code\gz-math
python3 [local_build.py](local_build.py) [gz_math-default-devel-windows-amd64.bat](gz_math-default-devel-windows-amd64.bat) C:\Users\foo\code\gz-math
```

This command will run [gz_math-default-devel-windows-amd64.bat](gz_math-default-devel-windows-amd64.bat) using the sources from `C:\User\foo\code\gz-math`. It will handle the installation of all the system dependencies
using Pixi (it can take up to 10 minutes) and build all the Gazebo dependencies from source
using colcon. In a second build it builds gz-math with tests using colcon.

When finishes, you can do modifications in C:\Users\foo\code\gz-math and re-run the script
with the `--reuse-dependencies-enviroment` flag enabled to re-use the environment
prepared with external dependencies.

```bat
python3 [local_build.py](local_build.py) [gz_math-default-devel-windows-amd64.bat](gz_math-default-devel-windows-amd64.bat) C:\Users\foo\code\gz-math --reuse-dependencies-enviroment
```

The script will also generate a `.debug_last_build.bat` file that will source the generated Pixi
enviroment and the colcon `install.bat` and leave the user in the colcon workspace root inside
%TMP%. This allows direct debugging without the need to run anything more than colcon and edit
the code in the colcon workspace.

```bat
call .debug_last_build.bat
:: ignore errors related to vs2019 if using other version of MSVC
C:\Users\josel\AppData\Local\Temp\12853\ws> colcon list
 gz-cmake4       ws\src\gz-cmake (ros.cmake)
 gz-plugin3      gz-plugin       (ros.cmake)
 gz-plugin3      ws\src\gz-plugin        (ros.cmake)
 gz-tools2       ws\src\gz-tools (ros.cmake)
 gz-utils3       ws\src\gz-utils (ros.cmake)
 ...
:: edit code in C:\Users\josel\AppData\Local\Temp\12853\ws\src\gz-sim
C:\Users\josel\AppData\Local\Temp\12853\ws> colcon build --packages-select gz-sim9
```

## DSL related

### Useful links
- [List of installed plugins in Jenkins](https://github.com/osrf/chef-osrf/blob/latest/cookbooks/osrfbuild/attributes/plugins.rb)
- [Jenkins DSL API docs](https://jenkinsci.github.io/job-dsl-plugin/)
- [Jenkins DSL Wiki](https://github.com/jenkinsci/job-dsl-plugin/wiki)

### Local testing

To test locally the build of the different `dsl` jobs you need the following:

1. Run the [dsl/tools/setup_local_generation.bash](dsl/tools/setup_local_generation.bash) script to produce the necessary jar files
2. In the terminal execute:
```bash
java -jar <path-to-dsl-tools>/jobdsl.jar <file.dsl>
```
For more information go [here](https://github.com/jenkinsci/job-dsl-plugin/wiki/User-Power-Moves#run-a-dsl-script-locally).

### Development workflow

1. Make changes locally and test that it builds correctly.
2. Push changes to a specific branch in `release-tools`
3. Go to seed job for the job you wanna test (usually you would use [`_dsl_test`](https://build.osrfoundation.org/job/_dsl_test/) to not affect the jobs in production) and build with the parameter pointing to your new custom branch in the `RTOOLS_BRANCH` parameter.
4. After it builds correctly, you will have generated jobs with the changes you implemented. You can use and modify the generated job.

> WARNING! : Running the _dsl job for a specific job that it's not `test` will modify the configuration for production. You should always aim to utilize `test` jobs.

### :arrow_forward: Playbook

#### XML injection into DSL
How to deal with plugins that do not implement the DSL layer and don't provide a DSL API?

There is a feature called [configure blocks](https://github.com/jenkinsci/job-dsl-plugin/wiki/The-Configure-Block) that allows us to represent xml job configs as DSL. [Here](https://github.com/gazebo-tooling/release-tools/blob/9fbfe60133d2b7b8b280b92f7c563dc64c8367a5/jenkins-scripts/dsl/_configs_/OSRFUNIXBase.groovy#LL83C1-L92C10) is an example of its usage with the retryBuild plugin, where `checkRegexp(true)` gets converted into `<checkRegexp>true</checkRegexp>` and the hierarchy of the definition is respected, so `checkRegexp` exists as a child of `com.chikli.hudson.plugin.naginator.NaginatorPublisher` in the XML definition.

To check what are the corresponding names for the XML tags you can refer to the plugin documentation or as an alternative you can manually modify the job to add the information you want and then go to `https://build.osrfoundation.com/job/myjob/config.xml` and match the XML there in the DSL config.
