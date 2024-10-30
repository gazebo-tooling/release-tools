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

### The legacy environment

The legacy environment has the goal of replacing the previous vcpkg
installation used by the Gazebo Buildfarm for testing Garden, Harmonic
and Ionic. The software versions chosen are mostly based on the Ubuntu
Jammy versions.
