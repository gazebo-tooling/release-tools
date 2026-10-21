#!/usr/bin/python3
import os
import sys
import yaml


DEFAULT_YAML_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'gz-collections.yaml')


# Function to find the branch of a library inside a given collection
def find_branch(data, collection_name, lib_name) -> str:
  for collection in data['collections']:
    if collection['name'] != collection_name:
      continue
    for lib in collection['libs']:
      if lib['name'] == lib_name:
        return lib['repo']['current_branch']
    print(f"Library {lib_name} not found in collection {collection_name}",
          file=sys.stderr)
    return ''

  print(f"Collection {collection_name} not found", file=sys.stderr)
  return ''


def main() -> int:
  if len(sys.argv) < 3:
    print(f"Usage: {sys.argv[0]} <collection_name> <lib_name> "
          "[collection-yaml-file]", file=sys.stderr)
    return 2

  collection_name = sys.argv[1]
  lib_name = sys.argv[2]
  yaml_file = sys.argv[3] if len(sys.argv) > 3 else DEFAULT_YAML_FILE

  with open(yaml_file, 'r') as file:
    data = yaml.safe_load(file)

  branch = find_branch(data, collection_name, lib_name)
  if not branch:
    return 1
  print(f"{branch}")
  return 0


if __name__ == '__main__':
  sys.exit(main())
