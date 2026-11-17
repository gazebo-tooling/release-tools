# Scripts for working with Jenkins DSL and gz-collections.yaml

## get_ciconfigs_from_package_and_version.py

Python script to find conda CI configurations for Gazebo packages based on their
name and major version. Parses `gz-collections.yaml` to determine which conda
environments should be used for building specific packages.

**Usage:**
```bash
python get_ciconfigs_from_package_and_version.py gz-rendering 6
python get_ciconfigs_from_package_and_version.py gz-sim 8 --yaml-file custom-collections.yaml
```

Every collection with the package and major version counts, not only the
first one. A collection with empty `ci.configs` whose entry tracks `main`
(Gazebo M) uses the collections testing the package in `main` (rotary).

**Output:** Full details per collection: collection name, CI configs, and conda configuration details.

With `--conda-env` the script returns only the conda environment version
string. This is the mode used by the build system to determine which conda
environment to use. Configs with the same environment collapse into one;
several environments are an error that lists them and asks for
`CONDA_ENV_NAME`.

```bash
python get_ciconfigs_from_package_and_version.py --conda-env gz-rendering 6
# Output: legacy

python get_ciconfigs_from_package_and_version.py --conda-env gz-sim 11
# Output: noble_like (from rotary, gz-sim 11 is in M)

python get_ciconfigs_from_package_and_version.py --conda-env gz-tools 2
# Error (exit 1): harmonic and ionic use legacy_ogre23, jetty uses noble_like
```

**Output:** Single line containing the conda environment version (e.g., `legacy`, `legacy_ogre23`, `noble_like`).

`--os` and `--arch` (the `system.so` and `system.arch` of the ci_configs,
used together) keep only the conda configs of that platform, in both modes.
`windows_library.bat` and the pixi_ci driver use them.

```bash
python get_ciconfigs_from_package_and_version.py --conda-env --os windows --arch amd64 gz-sim 10
# Output: noble_like
```

## DSL 6
python get_ciconfigs_from_package_and_version.py gz-sim 8 --yaml-file custom-collections.yaml
```

**Output:** Collection name and conda configuration details for Windows builds.

## DSL 

DSL is the Jenkins plugins that allows to use code for creating
the different Jenkins jobs and configurations.

### setup_local_generation.bash (jobdsl.jar)

Script use to generate job configuration locally for Jenkins. See
[Jenkins script README](../README.md) 

## gz-collections.yaml

The `gz-collections.yaml` file stores all the metadata corresponding
to the different collections of Gazebo, the libraries that compose
each release and the metadata for creating the buildfarm jobs that
implement the CI and packaging.

The file is useful for scripts that use global information about the
Gazebo libraries.

### get_branch_from_collection_and_package.py

The script returns the branch used by a given library in a given Gazebo
release (also known as collection), both provided as input parameters.

The output is a single line with the branch name. If no match is found,
the output is empty and an explanatory message is sent to stderr.

#### Usage

```
./get_branch_from_collection_and_package.py <collection_name> <lib_name> [path-to-gz-collections.yaml]
```

Use the canonical library name without the major version number in
`<lib_name>`. When `<path-to-gz-collections.yaml>` is omitted, the
`gz-collections.yaml` file next to this directory is used.

#### Example

```
$./get_branch_from_collection_and_package.py jetty gz-cmake
```

That generates the result of:

```
gz-cmake5
```

### get_collections_from_package_and_version.py

The script return the Gazebo releases (also known as collections) that
contains a given library and major version that are provided as input
parameters.

The output is provided as a space separated list in a single line. If
no match is found, the result is an empty string.

#### Usage

```
./get_collections_from_package_and_version.py <lib_name> <major_version> <path-to-gz-collections.yaml>
```

Be sure of not including the major version number in the `<lib_name>`

#### Example

```
$./get_collections_from_package_and_version.py gz-tools 2 ../gz-collections.yaml
```

That generates the result of:

```
harmonic ionic jetty
```
