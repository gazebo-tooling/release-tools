#!/usr/bin/python3

import sys
import yaml


# Function to find the collection name based on lib name and major version
def find_collection(data, lib_name, major_version):
  instances = []

  for collection in data['collections']:
    for lib in collection['libs']:
      if lib['name'] == lib_name and lib['major_version'] == major_version:
        instances.append(collection['name'])
  return instances


def get_major_version(version):
  elements = version.split('.')
  return int(elements[0])


if len(sys.argv) < 3:
  print(f"Usage: {sys.argv[0]} <lib_name> <major_version> <collection-yaml-file>")
  sys.exit(1)

lib_name = sys.argv[1]
version = sys.argv[2]
yaml_file = sys.argv[3]

with open(yaml_file, 'r') as file:
  data = yaml.safe_load(file)

collection_names = find_collection(data, lib_name, get_major_version(version))
print(f"{' '.join(collection_names)}")
