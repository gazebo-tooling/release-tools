# Release Tools

This repository holds scripts and tools that are used for testing and releasing
Gazebo Classic and Ignition software.

## Scripts

* [release.py](release.py): Triggers new debian and homebrew releases (major, minor, patch, pre-release...).
* [changelog_spawn.sh](release-repo-scripts/changelog_spawn.sh): Adds changelog information to `*-release` repositories. Must be used before running `release.py`.
* [new_ignition_release_repos.bash](release-repo-scripts/new_ignition_release_repos.bash): Create new `*-release` repositories.
* [bump_dependency.bash](release-repo-scripts/bump_dependency.bash): Makes all the updates needed when bumping a library's major version for the upcoming collection.

