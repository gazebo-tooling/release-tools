#!/usr/bin/python3
import sys
import yaml


# Function to find the collection name based on lib name and major version
def find_collection(data, lib_name, major_version) -> list:
  instances = []

  for collection in data['collections']:
    for lib in collection['libs']:
      if lib['name'] == lib_name and lib.get('major_version') == major_version:
        instances.append(collection['name'])
  return instances


# Function to find the current branches of a lib name and major version
def find_branches(data, lib_name, major_version) -> list:
  branches = []

  for collection in data['collections']:
    for lib in collection['libs']:
      if lib['name'] == lib_name and lib.get('major_version') == major_version:
        branch = lib.get('repo', {}).get('current_branch')
        if branch and branch not in branches:
          branches.append(branch)
  return branches


def get_major_version(version) -> int:
  elements = version.split('.')
  return int(elements[0])


def main() -> int:
  argv = [arg for arg in sys.argv[1:] if arg != '--branches']
  print_branches = len(argv) != len(sys.argv) - 1
  if len(argv) < 3:
    print(f"Usage: {sys.argv[0]} [--branches] <lib_name> <major_version> <collection-yaml-file>")
    return 2

  lib_name = argv[0]
  version = argv[1]
  yaml_file = argv[2]

  with open(yaml_file, 'r') as file:
    data = yaml.safe_load(file)

  find = find_branches if print_branches else find_collection
  names = find(data, lib_name, get_major_version(version))
  if not names:
    return 1
  print(f"{' '.join(names)}")
  return 0


if __name__ == '__main__':
  sys.exit(main())
