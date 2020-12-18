# Release Tools

This repository holds scripts and tools that are used for testing and releasing
Gazebo Classic and Ignition software.

## Scripts

* [release.py](release.py): Triggers new debian and homebrew releases (major, minor, patch, pre-release...).
* [changelog_spawn.sh](release-repo-scripts/changelog_spawn.sh): Adds changelog information to `*-release` repositories. Must be used before running `release.py`.
* [new_ignition_release_repos.bash](release-repo-scripts/new_ignition_release_repos.bash): Create new `*-release` repositories.

### Making releases

#### One-time configuration

1. `sudo apt install devscripts s3cmd`
1. `s3cmd --configure`
1. `export DEBEMAIL="<username>@openrobotics"`
1. `export DEBFULLNAME="<Your full name>"`

**Note:** You may wish to add the two above exported variables to your `.bashrc`

### For each release

1. Check if this is a good time for the release
    * Ask team if there are any concerns about making the release.
    * Check if there are changes to previous versions that need to be forward-ported.
    * See if there are open PRs against that release branch that could go into the new release.
1. Choose how to bump the version
    * Bump major if the release includes breaking changes - this is usually done for all libraries at around the same time.
    * Bump minor if there are new backwards-compatible features, like new APIs.
    * Bump patch if there are only bug fixes that won't affect the API.
1. Open pull request updating the version on the source code
    * [Example pull request.](https://github.com/ignitionrobotics/ign-physics/pull/132)
    * Update the version on `CMakeLists.txt`.
    * Include a link comparing the current branch to the latest release.
        1. [Example branch comparison.](https://github.com/ignitionrobotics/ign-gazebo/compare/ignition-gazebo3_3.5.0...ign-gazebo3)
        1. Substitute the package version and name that are relevant to your release.
    * Update the changelog.
        1. Use the branch comparison obtained below as a guide for updating the changelog.
    * Update the migration guide as needed.
    * Wait for this pull request to be merged before proceeding.
1. Clone the appropriate release repository from https://github.com/ignition-release.
    * For example, `git clone https://github.com/ignition-release/ign-physics3-release`
1. If the versions of any dependencies have increased in the source code, open a pull request updating them on the release repo
    * [Example pull request.](https://github.com/ignition-release/ign-gazebo3-release/pull/4)
1. Wait for all pull requests opened above to be approved and merged.
1. Clone `release-tools`
    * `git clone https://github.com/ignition-tooling/release-tools`
1. Update the release-repo's changelog:
    1. `cd <path to release-repo>`
    1. `<path to release-tools>/release-repo-scripts/changelog_spawn.sh <version-revision>`
        * Where `<version-revision>` is something like `3.1.0-1` or `4.0.0~pre1-1`
        * You will need to force run the above command with `bash` (by prefacing the command with `bash`) if you are using `sh`.
1. Trigger release:
    1. `cd <path to source code>`
    1. Make sure you're in the branch to be released, after the pull request bumping the version has been merged.
    1. Dry-run `release.py` with the appropriate arguments. Some examples:
        1. New Ignition minor release: `<path to release-tools>/release.py -r 1 ign-physics3 3.1.0 --ignition-repo -a ignition-physics3 secrettoken --dry-run`
        1. Pre-release: `<path to release-tools>/release.py -r 1 ign-physics3 3.0.0~pre1 --ignition-repo -a ignition-physics3 --upload-to-repo=prerelease secrettoken --dry-run`
        1. Classic release: `<path to release-tools>/release.py -r 1 gazebo11 11.2.0 secrettoken --dry-run`
    1. If the dry run succeeds, run the same command again, now without `--dry-run`.
1. Check that:
    * Several `-debbuilder` jobs have been queued in https://build.osrfoundation.org/
    * A pull request was opened to https://github.com/osrf/homebrew-simulation
        1. This pull request may take a minute or two to open.
