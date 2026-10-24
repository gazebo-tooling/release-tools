# Job generation

## Naming schema for the jobs generated

  * Separator: `-` reserved symbol. The different parts can not use the `-` symbol with the exception of
    the {package_name} section.
  * Parts: chars between separators or between a separator and the beginning or the end of the job name
  * Section: a group of parts that have some meaning about the job goal

### Categories of jobs

  0. Common sections
  1. Jobs affecting releasing
  2. Jobs affecting the continuous integration of Gazebo libraries
  3. Jobs affecting the testing and building of the source code

### 0. Common sections

Parts composing composing the **package_name** section:

`{package_name}`:
 * Parts: 1
   * Name of the gazebo library with its major version (as appearing in CMakeLists) used as a target for the job action.
     Using dashes is aceptable only in this case. Examples: `gz-fuel-tools9-` or `sdformat15-`

Parts composing the **platform** section:
```
{so_variant}-{arch}
```

Due to problems with the max length of paths in the Windows command line, the Windows jobs use a different schema:
 * `win`: windows short-name implying colcon builds on amd64 using vcpkg packages.

**Sections:**

---

`{so_variant}`
 * Parts: 2
   * Platform description relative to the operative system and/or package manager. Examples:
     ubuntu distributions `jammy` and Mac Brew `homebrew`
   * TODO: separate concepts of OS and provisioner or build system

---

`arch`
 * Parts: 1
  * Architecture used in the build. Examples `arm64` or `amd64`


### 1. Jobs affecting releasing

Sections composing the name:

```
{package_name}-{release_stage}
```

**Sections:**

 `{package_name}`: see common sections

---

`{release_stage}`:
 * Parts: 1
   * Release stage for the `{package_name}` defined.

__Knowing actions and their parts__:

 * `debbuilder`: Linux builders for `{package_name}`
 * `source`: job creating the sources when releasing `{package_name}`

### 2. Jobs affecting the testing of the binary packages

```
{package_name}-{testing_type}-{platform}
```

**Sections:**

`{package_name}`: see common sections

---

`{testing_type}`: literals: `-install-pkg-` or `-install_bottle-`

---

`{platform}`: see common sections


### 3. Jobs affecting the testing and building of the source code

Sections composing the name:

```
{lib_name}-{testing_type}-{platform}
```

**Sections:**

`{lib_name}`:
 * Parts: 1
   * Name (without major versions) of the Gazebo library using underscores instead of hyphens.
     Examples: `gz_cmake` or `gz_fuel_tools`

---

`{testing_type}`:
 * Parts: 1..N
   * Part 1: kind of testing job. Examples: ci, install, etc.
   * Part 2..N: modifiers for the kind of testing job in part 1

__Knowing testing types and parts__:

 * `ci`: build and test that runs on
   * `pr_any`: branch/gitref of a repository is defined as a job input parameter. `ci-pr_any-`
   * `{branch}`: branch/gitref explicit name. Examples: `-ci-gz-cmake4-` or `-ci-gz-sim8-`
 * `ci_asan`: build and test with asan sanitizers on
   * `{branch}`: branch/gitref explicit name. `-ci_asan-sdf12-`
     * `{platorm}`: see common sections
 * `abichecker`:
    * `any_to_any-ubuntu`: abi checking job for two branches defined a job input parameters. `-abichecker-any_to_any-ubuntu-`
      * TODO: remove the ubuntu part using a migration strategy for existing PR

Due to problems with the max length of paths in the Windows command line, the Windows jobs use a different schema:
 * `{number}`: Windows short-name implying `-ci-{branch}` being branch library_name+{number}. Example: `-5-` on gz-common will imply branch gz-common5
 * `pr` Windows short-name implying  `-ci-pr_any-`. Example: `-pr-` on gz-physics will imply the PR job on gz-physics

---

`{platform}`: see common sections

