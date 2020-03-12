# Release repositories

The release repositories are those which stores the metadata to build the
different binary packages. This metadata can be presented in different ways,
depending on the `BUILD_METHOD` defined.

## BUILD_METHOD definition

The `BUILD_METHOD` variable is deffined in the *build.metadata.bash* file
located in the root of the -release repo. If this file is not present, LEGACY
method is assumed.

The *build.metadata.bash* file will be sourced by the release script so it can
host other bash variables needed by the different methods.

## Build types supported

#### LEGACY

 * **build.metadata.bash config**: none

 * **Release method**: the whole debian/ directory inside the distribution
   directory present in the -release repo (for the distribution being built)
   will be used.

 * **Directory layout**: In the root directory of the -release repo will exist
   one directory for every ubuntu distribution supported:

   ```
   build.metadata.bash
   xenial/
     debian/
       ...
   bionic/
     debian/
       ...
    ```

### OVERWRITE_BASE

Import a LEGACY branch as a base and overwrite its contents with files and patches

 * **build.metadata.bash config**: 
    - `RELEASE_BASE_BRANCH` (LEGACY type branch used as base)

 * **Release method**: 
    - the `RELEASE_BASE_BRANCH` is cloned 
    - the files inside the distribution debian/ directory overwrite the ones in
      the base branch.
    - .patch files inside patches/ directory are applied

 * **Directory layout**: In the root directory of the -release repo will exist
   one directory for every ubuntu distribution supported. And optionally a
   directory patches/ containing .patch files to be used against the base branch

   ```
   build.metadata.bash
   patches/
     foo.patch
   xenial/ 
	   debian/ 
	     changelog 
   bionic/ 
	   debian/
	     rules
	     changelog 
  ```
