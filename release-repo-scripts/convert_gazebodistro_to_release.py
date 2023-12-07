#!/usr/bin/python3

import argparse
import sys

import yaml


def _load_gazebodistro_file(filepath) -> dict:
    with open(filepath, 'r') as file:
        gazebodistro = yaml.load(file, Loader=yaml.FullLoader)

    return gazebodistro


def _convert_to_release(input_yaml) -> None:
    repos = {}
    for repo, repo_info in input_yaml['repositories'].items():
        branch = repo_info['version'].replace('sdf', 'sdformat')
        repo_name = f'{branch}-release'
        repos[repo_name] = {'type': repo_info['type'],
                            'url': f'https://github.com/gazebo-release/{repo_name}',
                            'version': 'main'}
    repos = {'repositories': repos}
    yaml.dump(repos, sys.stdout)


def _init_argparse() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        usage='%(prog)s GAZEBODISTRO_FILE',
        description='Transform gazebodistro files into vcs files for release repos'
    )
    parser.add_argument('input_file', nargs=1, type=str)
    return parser


def main() -> None:
    """Convert gazebodistro files into vcs files for release repositories in the input file."""
    parser = _init_argparse()
    args = parser.parse_args()
    input_yaml = _load_gazebodistro_file(args.input_file[0])
    _convert_to_release(input_yaml)


if __name__ == '__main__':
    main()
