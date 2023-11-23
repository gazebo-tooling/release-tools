# Job generation

## Naming schema for the jobs generated

  * Separator: `-` reserved symbol. The different parts can not use the `-` symbol
  * Parts: chars between separators or between a separator and the beginning or the end of the job name
  * Section: a group of parts that have some meaning about the job goal

### Categories of jobs

  * Jobs affecting the binary packages
  * Jobs affecting the continuous integration of Gazebo libraries

### Jobs affecting the binary packages

Sections composing the name:

```
{package-name}-{action} 
```

**Sections:**

---

`{package_name}`:
 * Parts: 1
   * Name of the binary package used as a target for the job action. Examples: `gz-fuel-tools9-` or `sdformat15-`

---

`{action}`:
 * Parts: 1..N
   * Part 1: kind of actions to perform on the `{package_name}` defined
   * Part 2..N: modifiers for the kind of actions in part 1. Examples: `-install-pkg-` or `-source`

__Knowing actions and their parts__:

 * `debbuilder`: Linux builders for `{package_name}`
 * `source`: job creating the sources when releasing `{package_name}`
 * `install`: check for the correct installation of the Linux `{package_name}` 
   * `pkg`: TODO: remove pkg or merge with install name
     * `{distro}`: Debian/Ubuntu distribution name used by the job. Example: `jammy`
     * `{arch}`: `{package_name}` architecture tested. Example `arm64`
 * `install_bottle`: check for the correct installation of Mac `{package_name}`
   * `{mac_flavour}`: variant of Mac package manager used by the job. Example: `homebrew` 
     * `{arch}`: `{package_name}` architecture tested. Exmaple `amd64`

### Jobs affecting the continuous integration of Gazebo libraries

Sections composing the name:

```
{lib_name}-{testing_type}-{platform} 
```

**Sections:**

`{lib_name}`:
 * Parts: 1 
   * Name (without major versions) of the Gazebo library using underscores instead of hyphens. Examples: `gz_cmake` or `gz_fuel_tools`

---

`{testing_type}`: 
 * Parts: 1..N
   * Part 1: kind of testing job. Examples: ci, install, etc.
   * Part 2..N: modifiers for the kind of testing job in part 1

__Knowing types and parts__:

 * `ci`: build and test that runs on
   * `pr_any`: branch/gitref of a repository is defined as a job input parameter. `ci-pr_any-`
   * `{branch}`: branch/gitref explicit name. Examples: `-ci-gz-cmake4-` or `-ci-gz-sim8-`
 * `ci_asan`: build and test with asan sanitizers on
   * `{branch}`: branch/gitref explicit name. `-ci_asan-sdf12-`
 * `abichecker`:
    * `any_to_any-ubuntu`: abi checking job for two branches defined a job input parameters. `-abichecker-any_to_any-ubuntu-`
      * TODO: remove the ubuntu part using a migration strategy for existing PR

Due to problems with the max length of paths in the Windows command line, the Windows jobs use a different schema:
 * `{number}`: Windows short-name implying `-ci-{branch}` being branch library_name+{number}. Example: `-5-` on gz-common will imply branch gz-common5
 * `pr` Windows short-name implying  `-ci-pr_any-`. Example: `-pr-` on gz-physics will imply the PR job on gz-physics
  
---

TODO: merge this with the binary package platforms

`{platform}` 
 * Parts: 1..2
   * Platform where the testing is done by the job

__Knowing platforms and parts__: 
 
 * `{distro}`: Debian/Ubuntu distribution name used by the job. Example: `jammy`
   * `{arch}`: `{lib_name}` architecture tested. Example `arm64`
 * `{mac_flavour}`: variant of Mac package manager used by the job. Example: `homebrew` 
   * `{arch}`: `{lib_name}` architecture tested. Example `amd64`

Due to problems with the max length of paths in the Windows command line, the Windows jobs use a different schema:
 * `win`: windows short-name implying colcon builds on amd64 using vcpkg packages.
