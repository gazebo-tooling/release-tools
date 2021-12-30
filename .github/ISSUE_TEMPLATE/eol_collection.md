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
- [ ] Check library versions that can be EOL’d on the [versions table](https://github.com/ignitionrobotics/docs/blob/master/tools/versions.md)
    * <!-- LIST LIBRARIES HERE -->
- [ ] Make releases with outstanding changes in each library about to be EOL'ed
- [ ] Archive release repos for all libraries after the last release has been made
- [ ] Update `release-tools` after all releases have been made ([example](https://github.com/ignition-tooling/release-tools/pull/369))
- [ ] Deprecate homebrew bottles (don't disable them) ([example](https://github.com/osrf/homebrew-simulation/pull/1242))
- [ ] Remove from buildfarmer ([dashboard](https://github.com/osrf/buildfarmer/pull/140) / [report](https://github.com/osrf/buildfarmer/pull/146))
- [ ] Remove from `pr-collection-labeler` ([example](https://github.com/ignition-tooling/pr-collection-labeler/pull/13))
- [ ] Remove from `ros_ign` ([example](https://github.com/ignitionrobotics/ros_ign/pull/199))
- [ ] Close Close "<collection> support" ticket after final releases ([example](https://github.com/ignition-tooling/release-tools/issues/297#issuecomment-1002232980))
- [ ] Update docs after EOL is complete ([example](https://github.com/ignitionrobotics/docs/pull/124))
- [ ] Announce EOL on https://community.gazebosim.org/ ([example](https://community.gazebosim.org/t/ignition-blueprint-officially-end-of-life/764))

