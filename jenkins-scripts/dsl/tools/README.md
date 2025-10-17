# Scripts for working with Jenkins DSL and gz-collections.yaml

## find_ciconfigs.py

Python script to find conda CI configurations for Gazebo packages based on their
name and major version. Parses `gz-collections.yaml` to determine which conda
environments should be used for building specific packages.

**Usage:**
```bash
python find_ciconfigs.py gz-rendering 6
python find_ciconfigs.py gz-sim 8 --yaml-file custom-collections.yaml
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
$./get_collections_from_package_and_version.py gz-cmake 4 ../gz-collections.yaml
```

That generates the result of:

```
ionic jetty
```
