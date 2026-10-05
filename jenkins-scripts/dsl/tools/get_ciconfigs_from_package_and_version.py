#!/usr/bin/env python3
"""
Script to find conda configurations for a given Gazebo package and major version
Usage: python get_ciconfigs_from_package_and_version.py gz-rendering 3
       python get_ciconfigs_from_package_and_version.py --conda-env gz-rendering 6
       python get_ciconfigs_from_package_and_version.py --conda-env --os windows --arch amd64 gz-rendering 6
With --conda-env it returns only the conda environment version string
(e.g., 'legacy', 'noble_like')
"""

import yaml
import sys
import os
import argparse

def find_conda_configs(package_name, major_version, yaml_file_path):
    """
    Find conda configurations for a given package and major version

    Args:
        package_name (str): Name of the package (e.g., 'gz-rendering')
        major_version (int): Major version number
        yaml_file_path (str): Path to gz-collections.yaml file

    Returns:
        dict: Results containing collection name and conda configs
    """

    if not os.path.exists(yaml_file_path):
        raise FileNotFoundError(f"YAML file not found: {yaml_file_path}")

    with open(yaml_file_path, 'r') as f:
        data = yaml.safe_load(f)

    # Find the collection containing the package with specified major version
    found_collection = None
    ci_configs = []

    for collection in data.get('collections', []):
        collection_name = collection.get('name', '')
        libs = collection.get('libs', [])

        # Check if this collection contains our package with the right major version
        for lib in libs:
            if (lib.get('name') == package_name and
                lib.get('major_version') == major_version):
                found_collection = collection_name
                ci_configs = collection.get('ci', {}).get('configs', [])
                break

        if found_collection:
            break

    if not found_collection:
        return {
            'found': False,
            'message': f"Package {package_name} with major version {major_version} not found"
        }

    # Find conda configurations from ci_configs section
    conda_configs = []
    ci_configs_data = data.get('ci_configs', [])

    for config_name in ci_configs:
        for ci_config in ci_configs_data:
            if ci_config.get('name') == config_name:
                system = ci_config.get('system', {})
                if system.get('distribution') == 'conda':
                    conda_configs.append({
                        'name': config_name,
                        'version': system.get('version'),
                        'arch': system.get('arch'),
                        'so': system.get('so')
                    })
                break

    return {
        'found': True,
        'package_name': package_name,
        'major_version': major_version,
        'collection': found_collection,
        'ci_configs': ci_configs,
        'conda_configs': conda_configs
    }

def print_conda_env(result, package_name, major_version):
    """Print the only conda environment version or exit with an error."""
    if not result['conda_configs']:
        print(f"Error: No conda configurations found for {package_name} v{major_version}", file=sys.stderr)
        sys.exit(1)

    if len(result['conda_configs']) > 1:
        print(f"Error: Multiple conda configurations found for {package_name} v{major_version}:", file=sys.stderr)
        for config in result['conda_configs']:
            print(f"  - {config['name']}: {config['version']}", file=sys.stderr)
        sys.exit(1)

    print(result['conda_configs'][0]['version'])

def _ci_config_names(collection):
    return (collection.get('ci') or {}).get('configs') or []

def _tracks_main(lib):
    return (lib.get('repo') or {}).get('current_branch') == 'main'

def find_platform_conda_envs(package_name, major_version, yaml_file_path,
                             so, arch):
    """
    Find the conda environments testing a package and major version on one
    platform (the system.so and system.arch of the ci_configs).

    Unlike find_conda_configs, every collection with the package and major
    version is a candidate, not only the first one. A candidate with empty
    ci configs whose entry tracks main (i.e: Gazebo M before its stable
    branches exist) is replaced by the collections testing the same package
    on main with non-empty ci configs (i.e: rotary).

    Args:
        package_name (str): gz-collections.yaml library name (e.g., 'gz-fuel-tools')
        major_version (int): Major version number
        yaml_file_path (str): Path to gz-collections.yaml file
        so (str): system.so of the ci_configs (e.g., 'windows', 'darwin')
        arch (str): system.arch of the ci_configs (e.g., 'amd64', 'arm64')

    Returns:
        list: one dict (collection, ci_config, version) per matching conda
              ci_config, in gz-collections.yaml order
    """
    with open(yaml_file_path, 'r') as f:
        data = yaml.safe_load(f)

    collections = data.get('collections', [])
    candidates = []
    for collection in collections:
        for lib in collection.get('libs', []):
            if (lib.get('name') != package_name or
                lib.get('major_version') != major_version):
                continue
            if _ci_config_names(collection):
                candidates.append(collection)
            elif _tracks_main(lib):
                candidates.extend(
                    c for c in collections
                    if _ci_config_names(c) and
                    any(l.get('name') == package_name and _tracks_main(l)
                        for l in c.get('libs', [])))
            break

    systems = {c.get('name'): c.get('system', {})
               for c in data.get('ci_configs', [])}
    matches = []
    for collection in candidates:
        for config_name in _ci_config_names(collection):
            system = systems.get(config_name, {})
            if (system.get('distribution') == 'conda' and
                system.get('so') == so and system.get('arch') == arch):
                matches.append({
                    'collection': collection.get('name'),
                    'ci_config': config_name,
                    'version': system.get('version')
                })
    return matches

def print_platform_env(package_name, major_version, yaml_file, so, arch):
    """Print the only conda env for the platform; return the exit code."""
    matches = find_platform_conda_envs(package_name, major_version,
                                       yaml_file, so, arch)
    versions = {m['version'] for m in matches}
    if len(versions) == 1:
        print(versions.pop())
        return 0
    if not matches:
        print(f"Error: No conda configurations found for {package_name} "
              f"v{major_version} on {so}/{arch}", file=sys.stderr)
        return 1
    print(f"Error: Several conda environments found for {package_name} "
          f"v{major_version} on {so}/{arch}:", file=sys.stderr)
    for m in matches:
        print(f"  - {m['collection']}: {m['ci_config']}: {m['version']}",
              file=sys.stderr)
    print("Set CONDA_ENV_NAME to choose one of them", file=sys.stderr)
    return 1

def main():
    parser = argparse.ArgumentParser(description='Find conda configurations for Gazebo packages')
    parser.add_argument('package_name',
                       help='Package name (e.g., gz-rendering)')
    parser.add_argument('major_version', type=int,
                       help='Major version number')
    parser.add_argument('--yaml-file', '-f',
                       default='../gz-collections.yaml',
                       help='Path to gz-collections.yaml file')
    parser.add_argument('--conda-env', action='store_true',
                       help='Print only the conda environment version')
    parser.add_argument('--os', dest='so',
                       help='Only conda configs with this system.so '
                            '(e.g., windows, darwin). Requires --arch '
                            'and --conda-env')
    parser.add_argument('--arch',
                       help='Only conda configs with this system.arch '
                            '(e.g., amd64, arm64). Requires --os '
                            'and --conda-env')

    args = parser.parse_args()
    if (args.so is None) != (args.arch is None):
        parser.error('--os and --arch must be used together')
    if args.so is not None and not args.conda_env:
        parser.error('--os and --arch require --conda-env')

    package_name = args.package_name
    major_version = args.major_version

    # Find the YAML file
    yaml_file = args.yaml_file
    if not os.path.exists(yaml_file):
        # Try relative to script location
        script_dir = os.path.dirname(os.path.abspath(__file__))
        yaml_file = os.path.join(script_dir, args.yaml_file)

        if not os.path.exists(yaml_file):
            print(f"Error: YAML file not found: {args.yaml_file}", file=sys.stderr)
            sys.exit(1)

    try:
        if args.so is not None:
            sys.exit(print_platform_env(package_name, major_version,
                                        yaml_file, args.so, args.arch))

        result = find_conda_configs(package_name, major_version, yaml_file)

        if not result['found']:
            print(result['message'], file=sys.stderr)
            sys.exit(1)

        if args.conda_env:
            print_conda_env(result, package_name, major_version)
            return

        # Print results
        print(f"Collection: {result['collection']}")
        print(f"CI Configs: {', '.join(result['ci_configs'])}")
        
        if result['conda_configs']:
            print("Conda Configurations:")
            for conda_config in result['conda_configs']:
                print(f"  - Name: {conda_config['name']}")
                print(f"    Version: {conda_config['version']}")
                print(f"    Architecture: {conda_config['arch']}")
                print(f"    OS: {conda_config['so']}")
        else:
            print("No conda configurations found for this package.")
            
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()