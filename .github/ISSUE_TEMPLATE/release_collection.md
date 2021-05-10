---
name: Release collection
about: Checklist when releating a new collection
title: "Release [COLLECTION]"
---

<!--

Use this template to keep track of everything that needs to be done to
release a new collection.

When opening PRs, add a link back to this issue for easier tracking.

-->

## During development

- [ ] Bump major version of libraries early in the development cycle (use [bump_libraries](bump_libraries.md) template).
- [ ] Work with artist on logo.

## Feature freeze

- [ ] Choose name for next release.
- [ ] Merge each library forward from previous versions
- [ ] Label pull requests being considered to enter release with [beta](https://github.com/search?q=org%3Aignitionrobotics+label%3Abeta&state=open)
    Pull requests opened against the new collection after then:
    - Bug fixes: will be considered for the initial release and labeled `beta`
    - New features: will not be considered for the initial release.
- [ ] Pre-release libraries as all `beta` labels are merged.
- [ ] Make collection pre-release after all libraries are pre-released.
- [ ] Update public documentation to install from pre-releases instead of nightlies.

## Code freeze

- [ ] Start T-Shirt campaign
- [ ] Hold tutorial party ([example](https://community.gazebosim.org/t/ignition-edifice-tutorial-party-support-open-source-and-get-t-shirts/866))
- [ ] PRs fixing documentation and critical bugs can be merged and pre-released
- As libraries have all their documentation reviewed:
    - [ ] Create stable branches off of main
    - [ ] Make stable releases

## Release day

- [ ] Make collection stable release after all libraries are released.
- [ ] Update `gazebodistro` ([example](https://github.com/ignition-tooling/gazebodistro/pull/31))
- [ ] Make GitHub releases for all the X.0.0 versions ([example](https://github.com/ignitionrobotics/ign-transport/releases/tag/ignition-transport9_9.0.0))
- [ ] Update docs to say release is stable ([example](https://github.com/ignitionrobotics/docs/pull/171))
- [ ] Make announcement

## Post-release

- [ ] Enable Jenkins CI for new stable branches ([example](https://github.com/ignition-tooling/release-tools/pull/299))
- [ ] Remove nightlies for release branches on `gzdev` ([example](https://github.com/ignition-tooling/gzdev/pull/19))
- [ ] Bump all `main` branches to the next major versioni (`X.0.0~pre1`)
    - [ ] Source code ([example](https://github.com/ignitionrobotics/ign-common/pull/193))
    - [ ] Add files to `gazebodistro` ([example](https://github.com/ignition-tooling/gazebodistro/pull/12))
    - [ ] Add aliases to `homebrew-simulation` ([example](https://github.com/osrf/homebrew-simulation/commit/1f8602af6f52e06e0542eebfbdbe97f5f6cf950c))
    - [ ] Create new `-release` repositories (use [this script](https://github.com/ignition-tooling/release-tools/blob/master/release-repo-scripts/new_ignition_release_repos.bash))


If the collection will be officially paired with a ROS 2 distro:

- [ ] Update REP-2000 when ROS 2 distro is added ([example](https://github.com/ros-infrastructure/rep/pull/291))
- [ ] Import stable releases into https://packages.ros.org ([example](https://github.com/ros-infrastructure/reprepro-updater/pull/109))
- [ ] Add new rosdep keys ([example](https://github.com/ros/rosdistro/pull/29176))
- [ ] Bloom-release `ros_ign` with the new version into Rolling before fork ([example](https://github.com/ros/rosdistro/pull/29192))
