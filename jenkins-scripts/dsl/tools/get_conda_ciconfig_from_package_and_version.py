#!/usr/bin/env python3
"""
Wrapper script to get conda environment version for a given Gazebo package and major version
Usage: python get_conda_ciconfig_from_package_and_version.py gz-rendering 6
Returns only the conda environment version string (e.g., 'legacy', 'noble_like')
"""

import sys
import os
import argparse

# Import the main function from the sibling script
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from get_ciconfigs_from_package_and_version import find_conda_configs

def main():
    parser = argparse.ArgumentParser(
        description='Get conda environment version for Gazebo packages'
    )
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
        yaml_file = os.path.join(script_dir, args.yaml_file)

        if not os.path.exists(yaml_file):
            print(f"Error: YAML file not found: {args.yaml_file}", file=sys.stderr)
            sys.exit(1)

    try:
        result = find_conda_configs(package_name, major_version, yaml_file)

        if not result['found']:
            print(result['message'], file=sys.stderr)
            sys.exit(1)

        if not result['conda_configs']:
            print(f"Error: No conda configurations found for {package_name} v{major_version}", file=sys.stderr)
            sys.exit(1)

        if len(result['conda_configs']) > 1:
            print(f"Error: Multiple conda configurations found for {package_name} v{major_version}:", file=sys.stderr)
            for config in result['conda_configs']:
                print(f"  - {config['name']}: {config['version']}", file=sys.stderr)
            sys.exit(1)

        # Output only the conda environment version
        print(result['conda_configs'][0]['version'])

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
