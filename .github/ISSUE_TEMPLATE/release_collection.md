---
name: Release collection
about: Checklist when releasing a new collection
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
    * <!-- LINK PRS HERE -->
- [ ] Create stable branches off of `main`
    - [ ] `New branch` button on branches page ([example](https://github.com/gazebosim/sdformat/branches))
    - [ ] Enable CI for stable branches ([example](https://github.com/gazebo-tooling/release-tools/pull/787))
    - [ ] Use stable branch jobs in Jenkins view ([example part 1](https://github.com/gazebo-tooling/release-tools/pull/793) and [part 2](https://github.com/gazebo-tooling/release-tools/pull/806))
- [ ] Pull requests previously targeted at `main`:
    - Bug fixes:
       - Retarget to the new stable branch.
       - Label with [beta](https://github.com/search?q=org%3Agazebosim+label%3Abeta&state=open)
       - They can be merged until code freeze.
    - Backwards-compatible features, either:
       - Retarget them to the new stable branch (but they can only be merged after the stable release), or
       - Keep targeting `main` and backport after the stable release.
    - Breaking features:
       - Keep targeting `main` (i.e. they will be delayed until the next major release)
- [ ] Bump all `main` branches to the next major version (`X.0.0~pre1`)
    - [ ] Source code ([example](https://github.com/gazebosim/gz-common/pull/193))
        * <!-- LINK PRs HERE -->
    - [ ] Add files to `gazebodistro` ([example](https://github.com/gazebo-tooling/gazebodistro/pull/12))
        * <!-- LINK PR HERE -->
    - [ ] Add aliases to `homebrew-simulation` ([example](https://github.com/osrf/homebrew-simulation/commit/1f8602af6f52e06e0542eebfbdbe97f5f6cf950c))
        * <!-- LINK PR HERE -->
    - [ ] Create new `-release` repositories (use [this script](https://github.com/gazebo-tooling/release-tools/blob/master/release-repo-scripts/new_ignition_release_repos.bash))
    - [ ] Enable nightlies for all `main` branches on `gzdev` ([example](https://github.com/gazebo-tooling/gzdev/pull/50))
    - [ ] Execute the tick-tock's "tock" for deprecations ([example](https://github.com/gazebosim/gz-sim/pull/875))
        * <!-- LINK PRs HERE -->
- [ ] Update Changelog and pre-release libraries as all `beta` labels are merged.
    * <!-- LINK PRS HERE -->
- [ ] Make collection pre-release after all libraries are pre-released.
    * <!-- LINK PR HERE -->
- [ ] Update public documentation to install from pre-releases instead of nightlies ([example](https://github.com/gazebosim/docs/pull/196/files#diff-ebeee4adce7cb444f663b59020fb9f43f6f9adf36f63657b8afdeeea7a8530d1)).
    * <!-- LINK PR HERE -->
- [ ] Change nightlies to pre-release for release branches on `gzdev` ([example](https://github.com/gazebo-tooling/gzdev/pull/35))
    * <!-- LINK PR HERE -->

## Code freeze

- [ ] Start T-Shirt campaign
    * <!-- LINK POST HERE -->
- [ ] Hold tutorial party ([example](https://community.gazebosim.org/t/ignition-edifice-tutorial-party-support-open-source-and-get-t-shirts/866))
    * <!-- LINK POST HERE -->
- [ ] Update references to the release install instructions
    * [ ] Versions doc https://github.com/gazebosim/docs/blob/master/tools/versions.md
    * [ ] ROS2 integration https://github.com/gazebosim/docs/blob/master/\<new_release\>/ros2_integration.md
    * [ ] ros_gz README https://github.com/gazebosim/ros_gz/blob/ros2/README.md
- [ ] PRs fixing documentation and critical bugs can be merged and pre-released
- As libraries have all their documentation reviewed:
    - [ ] Make stable releases
        * <!-- LINK PRs HERE -->

## Release day

- [ ] Make collection stable release after all libraries are released.
    * <!-- LINK PR HERE -->
- [ ] Update `gazebodistro` ([example](https://github.com/gazebo-tooling/gazebodistro/pull/31))
    * <!-- LINK PR HERE -->
- [ ] Make GitHub releases for all the X.0.0 versions ([example](https://github.com/gazebosim/gz-transport/releases/tag/ignition-transport9_9.0.0))
- [ ] Update docs to say release is stable ([example](https://github.com/gazebosim/docs/pull/171))
    * <!-- LINK PR HERE -->
- [ ] Make announcement
    * <!-- LINK POST HERE -->

## Post-release

- [ ] Enable Jenkins CI for new stable branches ([example](https://github.com/gazebo-tooling/release-tools/pull/299))
    * <!-- LINK PR HERE -->
- [ ] Remove pre-release for release branches on `gzdev` ([example](https://github.com/gazebo-tooling/gzdev/pull/36))
    * <!-- LINK PR HERE -->
- [ ] Update all repositories to default to the new stable branches.

If the collection will be officially paired with a ROS 2 distro:

- [ ] Update REP-2000 when ROS 2 distro is added ([example](https://github.com/ros-infrastructure/rep/pull/291))
- [ ] Import stable releases into https://packages.ros.org ([example](https://github.com/ros-infrastructure/reprepro-updater/pull/109))
- [ ] Add new rosdep keys ([example](https://github.com/ros/rosdistro/pull/29176))
- [ ] Bloom-release `ros_ign` with the new version into Rolling before fork ([example](https://github.com/ros/rosdistro/pull/29192))

# Status tracker

Repo | Version | Release Branch | Feature frozen | Synced with previous versions? | Prerelease? | Code Frozen | Stable release? | Open PRs | Pending dep release
-- | -- | -- | -- | -- | -- | -- | -- | -- | --
[gz-tools](https://github.com/gazebosim/gz-tools)           |  | gz-toolsN | ✔️ | ✔️  | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-tools/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-cmake](https://github.com/gazebosim/gz-cmake)           |  | gz-cmakeN | ✔️ | ✔️  | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-cmake/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-math](https://github.com/gazebosim/gz-math)             |  | gz-mathN | ✔️ | ✔️  | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-math/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-plugin](https://github.com/gazebosim/gz-plugin)         |  | gz-pluginN | ✔️ | ✔️  | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-plugin/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-utils](https://github.com/gazebosim/gz-utils)           |  | gz-utilsN | ✔️ | ✔️  | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-utils/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-common](https://github.com/gazebosim/gz-common)         |  | gz-commonN | ✔️ | ✔️  | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-common/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-msgs](https://github.com/gazebosim/gz-msgs)             |  | gz-msgsN | ✔️ | ✔️ | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-msgs/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-rendering](https://github.com/gazebosim/gz-rendering)   |  | gz-renderingN | ✔️ | ✔️ | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-rendering/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[sdformat](https://github.com/gazebosim/sdformat)             |  | sdfN | ✔️ | ✔️  | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/sdformat/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-fuel-tools](https://github.com/gazebosim/gz-fuel-tools) |  | gz-fuel-toolsN | ✔️ | ✔️ | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-fuel-tools/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-transport](https://github.com/gazebosim/gz-transport)   |  | gz-transportN | ✔️ | ✔️ | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-transport/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-gui](https://github.com/gazebosim/gz-gui)               |  | gz-guiN | ✔️ | ✔️  | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-gui/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-sensors](https://github.com/gazebosim/gz-sensors)       |  | gz-sensorsN | ✔️ | ✔️ | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-sensors/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-physics](https://github.com/gazebosim/gz-physics)       |  | gz-physicsN | ✔️ | ✔️ | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-physics/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-sim](https://github.com/gazebosim/gz-sim)         |  | gz-simN | ✔️ | ✔️ | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-sim/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-launch](https://github.com/gazebosim/gz-launch)         |  | gz-launchN | ✔️ | ✔️ | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-launch/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[gz-<collection>](https://github.com/gazebosim/gz-<cok>)    |  | main | ✔️ | ✔️  | ✔️ | ✔️ | ✔️ | [PRs](https://github.com/gazebosim/gz-<collection>/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |
[ros_gz](https://github.com/gazebosim/ros_gz)               |  | <distros> | ✔️ | ✔️ | ✔️ |  |  | [PRs](https://github.com/gazebosim/ros_gz/pulls/?q=is%3Apr+is%3Aopen+label%3A%22beta%22) |

Track stable release of metapackage `ignition-<collection>`.

* :hourglass: TODO
* :yellow_circle: waiting in the buildfarm
* :green_circle: in the repo
* :red_circle: requires action
* :black_circle: no release

### Linux

 * **Ubuntu:**
   - [ ] :hourglass: Focal/amd64
   - [ ] :hourglass: Jammy/amd64
   - [ ] :hourglass: Focal/arm64
   - [ ] :hourglass: Jammy/arm64

 * **Debian:**
   - [ ] :hourglass: Buster/i386
   - [ ] :hourglass: Buster/amd64
   - [ ] :hourglass: Sid/\*:
   - [ ] :hourglass: Buster/arm64
   - [ ] :hourglass: Buster/armhf (raspbian)

### MacOSX

* **Brew**
  - [ ] :hourglass: Bottles

### Windows

* **Conda-forge** (community effort)
  - [ ] :hourglass:
