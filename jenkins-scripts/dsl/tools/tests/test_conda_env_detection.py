import subprocess
import sys
from pathlib import Path

import yaml

from get_ciconfigs_from_package_and_version import find_platform_conda_envs

TOOLS = Path(__file__).resolve().parents[1]
SCRIPT = TOOLS / 'get_ciconfigs_from_package_and_version.py'
FIXTURE = Path(__file__).resolve().parent / 'fixtures' / 'collections.yaml'
REAL_YAML = TOOLS.parent / 'gz-collections.yaml'


def detect(*args, yaml_file=FIXTURE):
    return subprocess.run(
        [sys.executable, str(SCRIPT), '--conda-env', '--yaml-file', str(yaml_file),
         *args],
        capture_output=True, text=True)


def test_windows_and_macos_resolve_to_the_same_env():
    windows = detect('--os', 'windows', '--arch', 'amd64', 'gz-math', '9')
    macos = detect('--os', 'darwin', '--arch', 'arm64', 'gz-math', '9')
    assert (windows.returncode, windows.stdout) == (0, 'noble_like\n')
    # osx_conda_noble and osx_conda_noble_canary collapse into one env
    assert (macos.returncode, macos.stdout) == (0, 'noble_like\n')


def test_several_envs_for_one_platform_fail_naming_every_candidate():
    result = detect('--os', 'windows', '--arch', 'amd64', 'gz-tools', '2')
    assert result.returncode == 1
    assert result.stdout == ''
    for line in ('harmonic: win_conda_LO23: legacy_ogre23',
                 'ionic: win_conda_LO23: legacy_ogre23',
                 'jetty: win_conda_noble: noble_like',
                 'CONDA_ENV_NAME'):
        assert line in result.stderr


def test_main_branch_major_falls_back_to_the_collection_testing_main():
    assert find_platform_conda_envs('gz-math', 10, FIXTURE,
                                    'windows', 'amd64') == [
        {'collection': 'rotary', 'ci_config': 'win_conda_noble',
         'version': 'noble_like'}]
    result = detect('--os', 'darwin', '--arch', 'arm64', 'gz-math', '10')
    assert (result.returncode, result.stdout) == (0, 'noble_like\n')


def test_library_name_resolves_and_colcon_name_does_not():
    library = detect('--os', 'windows', '--arch', 'amd64', 'gz-fuel-tools', '9')
    colcon = detect('--os', 'windows', '--arch', 'amd64', 'gz-fuel_tools', '9')
    assert (library.returncode, library.stdout) == (0, 'legacy_ogre23\n')
    assert colcon.returncode == 1
    assert 'No conda configurations found' in colcon.stderr


def test_no_config_for_the_platform_fails():
    result = detect('--os', 'darwin', '--arch', 'arm64', 'gz-fuel-tools', '9')
    assert result.returncode == 1
    assert 'No conda configurations found for gz-fuel-tools v9 on darwin/arm64' in result.stderr


def test_os_without_arch_is_a_usage_error():
    result = detect('--os', 'windows', 'gz-math', '9')
    assert result.returncode == 2
    assert '--os and --arch must be used together' in result.stderr


def test_os_without_conda_env_is_a_usage_error():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--yaml-file', str(FIXTURE),
         '--os', 'windows', '--arch', 'amd64', 'gz-math', '9'],
        capture_output=True, text=True)
    assert result.returncode == 2
    assert '--os and --arch require --conda-env' in result.stderr


def test_without_os_the_behaviour_is_unchanged():
    # First collection only: gz-tools 2 silently gets harmonic's env
    tools = detect('gz-tools', '2')
    assert (tools.returncode, tools.stdout) == (0, 'legacy_ogre23\n')
    # All conda configs of the collection, whatever the OS: an error
    math = detect('gz-math', '9')
    assert math.returncode == 1
    assert 'Multiple conda configurations found for gz-math v9' in math.stderr


def _main_branch_majors(data):
    """Major version of each library in collections with empty ci configs
    that track main (Gazebo M): the major its main branch builds today."""
    return {lib['name']: lib['major_version']
            for c in data['collections']
            if not (c.get('ci') or {}).get('configs')
            for lib in c.get('libs', [])
            if (lib.get('repo') or {}).get('current_branch') == 'main' and
            'major_version' in lib}


def test_real_collections_resolve_to_their_windows_env():
    data = yaml.safe_load(REAL_YAML.read_text())
    systems = {c['name']: c.get('system', {}) for c in data['ci_configs']}
    main_majors = _main_branch_majors(data)
    checked = 0
    for collection in data['collections']:
        envs = {systems[name]['version']
                for name in (collection.get('ci') or {}).get('configs') or []
                if systems[name].get('distribution') == 'conda' and
                systems[name].get('so') == 'windows'}
        if not envs:
            continue
        for lib in collection['libs']:
            major = lib.get('major_version', main_majors.get(lib['name']))
            assert major is not None, \
                f"{collection['name']}: {lib['name']} has no major version"
            found = {m['version'] for m in find_platform_conda_envs(
                lib['name'], major, REAL_YAML, 'windows', 'amd64')}
            # Either resolves to the collection's env or is ambiguous with
            # the collection's env among the candidates
            assert envs <= found, (collection['name'], lib['name'], found)
            checked += 1
    assert checked > 0
