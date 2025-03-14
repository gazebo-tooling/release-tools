#!/usr/bin/env python3
import os
import sys
import shutil
import tempfile
import random
import subprocess
from pathlib import Path
import argparse

def main():
    # Get script directory
    script_dir = Path(__file__).parent.absolute()

    # Parse arguments
    parser = argparse.ArgumentParser(
        description="Local build script",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("script", help="The script to run")
    parser.add_argument("sources", help="Local checkout of sources directory")
    parser.add_argument(
        "--reuse-dependencies-environment",
        action="store_true",
        help="Reuse pixi build environment (useful for testing code changes)"
    )
    parser.add_argument(
        "-j", "--jobs",
        type=int,
        default=8,
        help="Number of building threads (default: 8)"
    )
    args = parser.parse_args()
    
    script_path =  Path(script_dir) / Path(args.script)
    src_directory = args.sources
    reuse_dependencies_environment = args.reuse_dependencies_environment
    make_jobs = args.jobs
    
    # Create temp workspace with random name
    workspace = Path(os.environ["TMP"]) / str(random.randint(0, 1000000))
    
    # Unset variables LOCAL_WS LOCAL_WS_BUILD LOCAL_WS_SOFTWARE_DIR
    for var in ["LOCAL_WS", "LOCAL_WS_BUILD", "LOCAL_WS_SOFTWARE_DIR", "VCS_DIRECTORY", "WORKSPACE"]:
        if var in os.environ:
            os.environ.pop(var, None)

    # Set environment variables
    os.environ["WORKSPACE"] = str(workspace)
    os.environ["MAKE_JOBS"] = str(make_jobs)  # Customize the number of building threads
    pixi_project_path = Path(os.environ["TMP"]) / "pixi" / "project"
    os.environ["PIXI_PROJECT_PATH"] = str(pixi_project_path)

    # Check if script exists
    if not Path(script_path).exists():
        print(f"Script {script_path} does not exist", file=sys.stderr)
        sys.exit(1)
    
    # Check if sources exist
    if not Path(src_directory).exists():
        print(f"Sources {src_directory} does not exist", file=sys.stderr)
        sys.exit(1)
    
    # Check that Visual Studio environment is set
    # if "VSCMD_ARG_TGT_ARCH" not in os.environ:
    #    print("Visual Studio environment not set. Please run vcvarsall.bat", file=sys.stderr)
    #    sys.exit(1)
    
    # Set additional environment variables
    os.environ["KEEP_WORKSPACE"] = "1"  # Help with debugging and re-run compilation only
    
    # Create debug last build file path
    dbg_last_build_file = Path(".debug_last_build.bat")
    if dbg_last_build_file.exists():
        dbg_last_build_file.unlink()
    
    # Create workspace and copy files
    workspace.mkdir(exist_ok=True)
    dest_dir = workspace / Path(src_directory).name
    print(f"Copying {src_directory} to {dest_dir}")
    shutil.copytree(src_directory, dest_dir, dirs_exist_ok=True)

    if reuse_dependencies_environment:
        os.environ["REUSE_PIXI_INSTALLATION"] = "1"
    else:
        if "REUSE_PIXI_INSTALLATION" in os.environ:
            del os.environ["REUSE_PIXI_INSTALLATION"]
    
    # Run the script
    result = subprocess.run([script_path], shell=True, check=False)
    
    # Check for errors
    if result.returncode != 0:
        print("Local build failed")
        print(f" ERROR: {result.returncode}")
        
        if "LOCAL_WS" not in os.environ:
            print("LOCAL_WS is not set. Internal script error.")
            sys.exit(1)
    else:
        print("Local build finished successfully")
    
    # Create debug last build file for reproduction
    local_ws = os.environ.get("LOCAL_WS", "")
    with open(dbg_last_build_file, "w") as f:
        f.write(f"call {Path(local_ws) / 'install' / 'setup.bat'}\n")
        f.write(f"call {Path(pixi_project_path) / 'hooks.bat'}\n")
        f.write(f"cd {local_ws}\n")

    print(f"""- Build root is {workspace}
- Build workspace is {local_ws}
Reproduce the call to the last build:
  - Only reusing pixi environment:
    - run '{script_path} {src_directory} --reuse-dependencies-environment'
  - Preparing pixi and colcon and go to the colcon workspace:
    - run 'call {dbg_last_build_file}'""")


if __name__ == "__main__":
    main()