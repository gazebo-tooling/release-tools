#!/usr/bin/env python3
"""
Script to find conda configurations for a given Gazebo package and major version
Usage: python get_ciconfigs_from_package_and_version.py gz-rendering 3
       python get_ciconfigs_from_package_and_version.py --conda-env gz-rendering 6
       python get_ciconfigs_from_package_and_version.py --conda-env --os windows --arch amd64 gz-rendering 6
With --conda-env it returns only the conda environment version string
(e.g., 'legacy', 'noble_like'). With --os and --arch only the conda
configs of that platform count.
"""

import yaml
import sys
import os
import argparse

def _ci_config_names(collection):
    return (collection.get('ci') or {}).get('configs') or []

def _tracks_main(lib):
    return (lib.get('repo') or {}).get('current_branch') == 'main'

def find_collections(package_name, major_version, data):
    """
    Find the collections testing a package and major version.

    Every collection with the package and major version is returned, not
    only the first one. A collection with empty ci configs whose entry
    tracks main (i.e: Gazebo M before its stable branches exist) is
    replaced by the collections testing the same package on main with
    non-empty ci configs (i.e: rotary).

    Args:
        package_name (str): Name of the package (e.g., 'gz-rendering')
        major_version (int): Major version number
        data (dict): Parsed gz-collections.yaml

    Returns:
        list: collection dicts, in gz-collections.yaml order
    """
    collections = data.get('collections', [])
    found = []
    for collection in collections:
        for lib in collection.get('libs', []):
            if (lib.get('name') != package_name or
                lib.get('major_version') != major_version):
                continue
            if not _ci_config_names(collection) and _tracks_main(lib):
                found.extend(
                    c for c in collections
                    if _ci_config_names(c) and c not in found and
                    any(l.get('name') == package_name and _tracks_main(l)
                        for l in c.get('libs', [])))
            elif collection not in found:
                found.append(collection)
            break
    return found

def find_conda_configs(package_name, major_version, yaml_file_path,
                       so=None, arch=None):
    """
    Find conda configurations for a given package and major version

    Args:
        package_name (str): Name of the package (e.g., 'gz-rendering')
        major_version (int): Major version number
        yaml_file_path (str): Path to gz-collections.yaml file
        so (str): Only conda configs with this system.so (e.g., 'windows',
                  'darwin'). None for every platform
        arch (str): Only conda configs with this system.arch (e.g., 'amd64',
                    'arm64'). None for every platform

    Returns:
        list: one dict (collection, ci_configs, conda_configs) per collection
              from find_collections. Empty if the package and major version
              are not found
    """

    if not os.path.exists(yaml_file_path):
        raise FileNotFoundError(f"YAML file not found: {yaml_file_path}")

    with open(yaml_file_path, 'r') as f:
        data = yaml.safe_load(f)

    systems = {c.get('name'): c.get('system', {})
               for c in data.get('ci_configs', [])}
    results = []
    for collection in find_collections(package_name, major_version, data):
        ci_configs = _ci_config_names(collection)
        conda_configs = []
        for config_name in ci_configs:
            system = systems.get(config_name, {})
            if (system.get('distribution') == 'conda' and
                so in (None, system.get('so')) and
                arch in (None, system.get('arch'))):
                conda_configs.append({
                    'name': config_name,
                    'version': system.get('version'),
                    'arch': system.get('arch'),
                    'so': system.get('so')
                })
        results.append({
            'collection': collection.get('name'),
            'ci_configs': ci_configs,
            'conda_configs': conda_configs
        })
    return results

def print_conda_env(results, package_name, major_version, so, arch):
    """Print the only conda environment version; return the exit code."""
    matches = [(r['collection'], c)
               for r in results for c in r['conda_configs']]
    # Configs with the same env (i.e: daily + PR canary) collapse into one
    versions = {c['version'] for _, c in matches}
    if len(versions) == 1:
        print(versions.pop())
        return 0

    where = f" on {so}/{arch}" if so is not None else ''
    if not matches:
        print(f"Error: No conda configurations found for {package_name} "
              f"v{major_version}{where}", file=sys.stderr)
        return 1
    print(f"Error: Several conda environments found for {package_name} "
          f"v{major_version}{where}:", file=sys.stderr)
    for collection, config in matches:
        print(f"  - {collection}: {config['name']}: {config['version']}",
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
                            '(e.g., windows, darwin). Requires --arch')
    parser.add_argument('--arch',
                       help='Only conda configs with this system.arch '
                            '(e.g., amd64, arm64). Requires --os')

    args = parser.parse_args()
    if (args.so is None) != (args.arch is None):
        parser.error('--os and --arch must be used together')

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
        results = find_conda_configs(package_name, major_version, yaml_file,
                                     args.so, args.arch)

        if not results:
            print(f"Package {package_name} with major version "
                  f"{major_version} not found", file=sys.stderr)
            sys.exit(1)

        if args.conda_env:
            sys.exit(print_conda_env(results, package_name, major_version,
                                     args.so, args.arch))

        # Print results
        for result in results:
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
