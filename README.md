# Release Tools

This repository holds scripts and tools that are used for testing and releasing
Gazebo Classic and Ignition software.

## Scripts

* [release.py](release.py): Triggers new debian and homebrew releases (major, minor, patch, pre-release...).
* [source_changelog.bash](source_changelog.bash): Generates Changelog.md entries newer than a specified tag based on git commit descriptions.

* For -release repository scripts, please see [README.md](release-repo-scripts/README.md)

### Making releases

#### One-time configuration

1. `sudo apt install devscripts s3cmd`
1. `s3cmd --configure`
    * Check with one of the maintainers to get the necessary credentials
1. `export DEBEMAIL="<username>@openrobotics.org"`
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
        1. Use the [source_changelog.bash](source_changelog.bash) script to generate the Changelog.md entries newer than the most recent tag.
            * `cd <path to source code>`
            * `bash <path to release-tools>/source_changelog.bash <previous tag>`
            * e.g. `bash ../release-tools/source_changelog.bash 3.5.0`
        1. Verify the Changelog.md entries using the GitHub branch comparison.
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
        1. New Ignition minor release and Gazebo classic release:
          1. `<path to release-tools>/release.py -r 1 ign-physics3 3.1.0 secrettoken --dry-run`
          1. `<path to release-tools>/release.py -r 1 gazebo11 11.2.0 secrettoken --dry-run`
        1. Pre-release: `<path to release-tools>/release.py -r 1 ign-physics3 3.0.0~pre1 --upload-to-repo=prerelease secrettoken --dry-run`
    1. If the dry run succeeds, run the same command again, now without `--dry-run`.
1. Check that:
    * Several `-debbuilder` jobs have been queued in https://build.osrfoundation.org/ and watch those jobs to see if any of them fail.
      While it would be easier to track the status of these jobs if there was a [dashboard](https://github.com/ignition-tooling/release-tools/issues/295),
      you can watch the page for a specific debbuild, such as https://build.osrfoundation.org/job/ign-gui3-debbuilder/.
        1. If you observe a failure of an Ignition Dome package, check the list of supported
           architectures in [issue #297](https://github.com/ignition-tooling/release-tools/issues/297);
           for example Bionic/armhf and Focal/armhf are not supported for Dome.
           To disable builds for a specific architecture, add a file to the `-release` repository that starts with `.releasepy_NO_ARCH_`
           and append the name of the architecture to be excluded. The file can be added to the
           root of the `-release` repository (like [.releasepy_NO_ARCH_ARM64 in ign-gazebo2-release](https://github.com/ignition-release/ign-gazebo2-release/blob/master/.releasepy_NO_ARCH_ARM64))
           or in a distro-specific sub-folder (like [bionic/.releasepy_NO_ARCH_armhf in ign-launch2-release](https://github.com/ignition-release/ign-launch2-release/blob/master/bionic/.releasepy_NO_ARCH_armhf)).
           The architecture suffix is not case-sensitive.
        1. To check if a debbuild has previously succeeded for a given architecture, you can look at packages.osrfoundation.org
           to see the most recent successful builds. For example, you can see the most recent
           [Ubuntu builds of ignition-gazebo3](https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/i/ignition-gazebo3/)
           or the [Debian builds of ignition-gazebo3](https://packages.osrfoundation.org/gazebo/debian-stable/pool/main/i/ignition-gazebo3/).
        1. If the failure is on a supported architecture, check the source repository for an existing report of this failure and if none
           exists, please report the failure (see [ignitionrobotics/ign-math#161](https://github.com/ignitionrobotics/ign-math/issues/161)
           for an example).
    * A pull request was opened to https://github.com/osrf/homebrew-simulation
        1. This pull request may take a minute or two to open.
        1. Once it is open, make a comment containing the text "build bottle".
           For further details, see the [README at osrf/homebrew-simulation](https://github.com/osrf/homebrew-simulation#to-build-bottles).
