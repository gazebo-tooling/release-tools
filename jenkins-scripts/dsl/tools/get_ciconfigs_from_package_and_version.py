#!/usr/bin/env python3
"""
Script to find conda configurations for a given Gazebo package and major version
Usage: python get_ciconfigs_from_package_and_version.py gz-rendering 3
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

def main():
    parser = argparse.ArgumentParser(description='Find conda configurations for Gazebo packages')
    parser.add_argument('package_name',
                       help='Package name (e.g., gz-rendering)')
    parser.add_argument('major_version', type=int,
                       help='Major version number')
    parser.add_argument('--yaml-file', '-f',
                       default='../gz-collections.yaml',
                       help='Path to gz-collections.yaml file')

    args = parser.parse_args()

    package_name = args.package_name
    major_version = args.major_version

    # Find the YAML file
    yaml_file = args.yaml_file
    if not os.path.exists(yaml_file):
        # Try relative to script location
        script_dir = os.path.dirname(os.path.abspath(__file__))
        yaml_file = os.path.join(script_dir, args.yaml_file)

        if not os.path.exists(yaml_file):
            print(f"Error: YAML file not found: {args.yaml_file}")
            sys.exit(1)

    try:
        result = find_conda_configs(package_name, major_version, yaml_file)

        if not result['found']:
            print(result['message'], file=sys.stderr)
            sys.exit(1)

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