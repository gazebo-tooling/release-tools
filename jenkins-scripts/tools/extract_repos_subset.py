#/usr/bin/env/python3
"""Extract a subset of repositories from a *.repos yaml file

This script loads a .repos yaml file and prints a yaml file
containing a subset of the repositories specified in the file.
"""
import argparse
import sys
import yaml

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument('file', type=str,
                        help = 'Yaml repos file to extract from')
    parser.add_argument('repo_names_to_extract', type=str, nargs='+')

    args = parser.parse_args()

    f = open(args.file, 'r')
    y = yaml.load(f.read())
    extracted_yaml = { 'repositories': {} }
    extracted_repos = extracted_yaml['repositories']

    for repo_name in args.repo_names_to_extract:
        try:
            extracted_repos[repo_name] = y['repositories'][repo_name]
        except KeyError:
            print(f"Error: could not find {repo_name} in {args.file}", file=sys.stderr)
    print(yaml.dump(extracted_yaml))

# ------------------------------------------------
if __name__ == '__main__':
    main()
