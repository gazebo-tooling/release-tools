# -release repositories scripts

The directory hosts the scripts designed to work with Gazebo -release
repositories, the release repositories hosting the metadata information for
packaging Debian and Ubuntu. Mostly hosted under
https://github.com/gazebo-release/.

## Scripts available

### bump_major_version.bash

The script will bump the major version of a library for the packaging files
inside a -release repository. Mostly designed to be run when a new -release
repository is forked for a new major release.

*Note: this script is being used from new_gazebo_release_repos script in
this repository which also creates new repositories. You'll probably want
to use that instead since it automates the forking and cloning process.*

#### Usage

Usually forking an existing -release repository, clone it and navigate to the
local clone using a shell is the step zero.

Require `debchange` command from the package `devscripts`. `debchange` requires
`DEBEMAIL` and `DEBFULLNAME` environment variables to be set for using it.

```bash
cd <path-to-releae-repo>
bump_major_version.bash <old_version> <new_version>
```
Changes required involve file names and code inside the configuration files.
The script will create an initial changelog entry with a nightly version lower
than any other expected in the repository.

#### Example

To bump `gz-rendering6` and create a new `gz-rendering7` release repositories,
first fork the `gz-rendering6-release` in GitHub and call the fork `gz-rendering7-release`.

```bash
git clone https://github.com/gz-release/gz-rendering7-release
cd gz-rendering7-release
./path/to/release-tools/release-repo-scripts/bump_major_version 6 7
```

### changelog_spawn.sh

Update all changelogs inside a -release repository to an specific new version.
Used while releasing a new version of any of the libraries or software.

#### Usage

Require `debchange` command from the package `devscripts`. `debchange` requires
`DEBEMAIL` and `DEBFULLNAME` environment variables to be set for using it.

The new version needs to be in the form of `X.Y.Z-R` Where `X.Y.Z` is the version
of the code and `R` is the revision corresponding to the version of packaging
metadata.

```bash
cd <path-to-releae-repo>
changelog_spawn.sh <new_version>
```

#### Example

To bump `ign-rendering7` from the version `7.0.0-1` to prepare the new release of
`7.1.0`:

```bash
git clone https://github.com/gz-release/gz-rendering7-release
cd gz-rendering7-release
./path/to/release-tools/release-repo-scripts/changelog_spawn.sh 7.1.0-1
```

### convert_gazebodistro_to_release.py

Convert a gazebodistro file into a vcs compatible file with the -release repositories
associated to the repositories in the input file.

#### Usage

Output will printed to stdout:

```bash
./path/to/release-tools/release-repo-scripts/convert_gazebodistro_to_release.py <gazbodistro_file>
```

#### Example

To import all release repositories related to gz Harmonic collection:

```bash
./path/to/release-tools/release-repo-scripts/convert_gazebodistro_to_release.py ~/code/gazebodistro/collection-harmonic.yaml  > collection-harmonic-release.yaml
vcs import < collection-harmonic-release.yaml
```

### convert_gazebodistro_to_release_ros_vendor.py

Convert a gazebodistro file into a vcs compatible file with the Gazebo ROS vendor
repositories related to the repositories in the input file. These vendor
repositories follow the naming convention of `package_name_vendor`. The default
branch to set in the vcs new file is `rolling`.

#### Usage

Output will be printed to stdout:

```bash
./path/to/release-tools/release-repo-scripts/convert_gazebodistro_to_release_ros_vendor.py <gazebodistro_file>
```

#### Example

To import all ROS vendor repositories related to gz Ionic collection:

```bash
./path/to/release-tools/release-repo-scripts/convert_gazebodistro_to_release_ros_vendor.py ~/code/gazebodistro/collection-ionic.yaml > collection-ionic-vendor.yaml
vcs import < collection-ionic-vendor.yaml



### new_ubuntu_distribution.bash

The script will create a new directory to host a new Ubuntu distribution inside
the -release repository where it is being run. The new directory will be mostly
a copy of the latest Ubuntu distribution available.

A local clone of the same Debian official package is being done in /tmp to import
some changes from it and produce a diff between the metadata in Debian official
and the one in the local repo.

#### Usage

Require the package `dctrl-tools` installed. The script will perform different
operations to update different metadata.

Some git add operations are performed but there won't be a commit or a permanent
change.

```bash
cd <path-to-release-repo>
./new_ubuntu_distribution <new_distro_name>
```

#### Example

To create the kinetic Ubuntu distribution inside `gz-cmake3-release`:

```bash
git clone https://github.com/gz-release/gz-cmake3-release
cd gz-cmake3-release
./path/to/release-tools/release-repo-scripts/new_ubuntu_distribution.bash kinetic
```

### new_gazebo_release_repos.bash

The script will create new -release repositories as forks from the previous version
and run the `bump_major_version` script in the repository to update all files and
packaging metadata to the new version.

#### Usage

Requires the 'gh' CLI to be installed and the requirements of the `bump_major_version`
script in this repository.

```bash
cd /tmp/
./path/to/release-tools/release-repo-scripts/new_gazebo_release_repos.bash <list_of_new_gazebo_names_space_separated>
```

`list_of_new_gazebo_names_space_separated` is composed from one or more
repository names to be created.

Sequence of actions expected:
 * Create gz-fooY-release if it does not exits
 * Clone to the current directory
 * Pull from main/master branch from previous version, gz repository to
   have the whole history
 * Run the bump_major_version from X to Y
 * Show the results
 * Commit/push all the changes

The -release repositories used by this tool will be created on current directory.

#### Example

To create `gz-rendering7-release` repository copying files in `gz-rendering6-release`:

```bash
cd /tmp/
./path/to/release-tools/release-repo-scripts/new_gazebo_release_repos.bash gz-rendering7
```

### rename_ignition_to_gazebo.bash

Script used for renaming all ignition package names to gz. It creates
transition packages as aliases.

#### Usage

The script uses `dch` from the package `debhelper`. `grep-dctrl` from the `dctrl-tools` package.
The script will make all the changes without making any commit so a manual inspection
can be done after the run.

```bash
cd <release-repo>
./path/to/release-tools/release-repo-scripts/rename_ignition_to_gazebo.bash
```

#### Example

```bash
git clone https://github.com/gazebo-release/gz-cmake3-release
cd gz-cmake3-release
./path/to/release-tools/release-repo-scripts/rename_ignition_to_gazebo.bash
```
