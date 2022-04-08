---
name: EOL collection
about: Checklist when a collection reaches end-of-life
title: "EOL collection: <COLLECTION>"
---

<!--

Use this template to keep track of everything that needs to be done when
EOL'ing a new collection.

When opening PRs, add a link back to this issue for easier tracking.

-->

- [ ] Write a post to https://community.gazebosim.org/ announcing the community it has last X weeks for finishing their PRs.
    * <!-- LINK POST HERE -->
- [ ] Check library versions that can be EOL’d on the [versions table](https://github.com/ignitionrobotics/docs/blob/master/tools/versions.md)
    * <!-- LIST LIBRARIES HERE -->
- [ ] Make releases with outstanding changes in each library about to be EOL'ed
    * <!-- LIST PRs HERE -->
- [ ] Archive release repos in [ignition-release](https://github.com/ignition-release) for all libraries after the last release has been made
- [ ] Update `release-tools` after all releases have been made ([example](https://github.com/ignition-tooling/release-tools/pull/369))
    * <!-- LINK PR HERE -->
- [ ] Deprecate homebrew bottles if they aren't deprecated yet (don't disable them) ([example](https://github.com/osrf/homebrew-simulation/pull/1785))
    * <!-- LINK PR HERE -->
- [ ] Remove badges from `homebrew-simulation` README ([example](https://github.com/osrf/homebrew-simulation/pull/1772))
    * <!-- LINK PR HERE -->
- [ ] Remove from buildfarmer ([example](https://github.com/osrf/homebrew-simulation/pull/1785))
    * <!-- LINK PR HERE -->
- [ ] Remove from `pr-collection-labeler` ([example](https://github.com/ignition-tooling/pr-collection-labeler/pull/13))
    * <!-- LINK PR HERE -->
- [ ] Remove from `ros_ign` ([example](https://github.com/ignitionrobotics/ros_ign/pull/199))
    * <!-- LINK PR HERE -->
- [ ] Sync latest binaries to http://packages.ros.org ([example](https://github.com/ros-infrastructure/reprepro-updater/pull/145))
    * <!-- LINK PR HERE -->
- [ ] Remove from `gzdev`'s `ign_docker_env` ([example](https://github.com/ignition-tooling/gzdev/pull/56))
    * <!-- LINK PR HERE -->
- [ ] Close "<collection> support" ticket after final releases ([example](https://github.com/ignition-tooling/release-tools/issues/297#issuecomment-1002232980))
    * <!-- LINK COMMENT HERE -->
- [ ] Update docs after EOL is complete ([example](https://github.com/ignitionrobotics/docs/pull/124))
    * <!-- LINK PR HERE -->
- [ ] Announce EOL on https://community.gazebosim.org/ ([example](https://community.gazebosim.org/t/ignition-blueprint-officially-end-of-life/764))
    * <!-- LINK POST HERE -->

