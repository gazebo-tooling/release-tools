# Conda configurations and tools for Gazebo

This repository hosts the Conda related configurations and files used
by the Gazebo project.

> [!IMPORTANT]
> Currently the Gazebo project is only using Conda for supporting the
> build on Windows. The builds or Conda packages can perfectly work
> on different platforms but the core team is not actively testing
> them.

## The envs/ (environments) directory

The environments directory hosts the Conda environments and Pixi projects
configurations used for different combinations of Gazebo distributions.

## Create environments with Pixi

[Pixi](https://prefix.dev/) can be used to recreate a Gazebo development
environment. Navigate to the right subdirectory under `envs/` where the
`pixi.lock` and `pixi.toml` files are located and run:

```bash
pixi install --locked
```

## Environments available

### The legacy environments

The legacy environments has the goal of replacing the previous vcpkg
installation used by the Gazebo Buildfarm for testing Fortress, Harmonic
and Ionic. The software versions chosen are mostly based on the Ubuntu
Jammy versions.

The legacy enviroments are locked to use always the same exact set of
version dependencies.

Harmonic and Ionic uses the legacy_ogre23 enviroments which is a variant
of what vcpkg packages had providing an upgrade on ogre-next from 2.2 to
2.3.

## Testing new Conda environments in Gazebo

When developing a new Conda environment can be useful to run a -pr- job
with an environment that is available in a branch of release-tools. To
do so the -pr- jobs have the `CONDA_ENV_NAME` parameter that can be set
to the name of the new Conda environment. Remember to pass the
corresponding release-tools branch in `RTOOLS_BRANCH`.
