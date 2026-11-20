from pathlib import Path

import pytest
import yaml

from get_ciconfigs_from_package_and_version import find_conda_libraries

TOOLS = Path(__file__).resolve().parents[1]
FIXTURE = Path(__file__).resolve().parent / 'fixtures' / 'conda_libraries.yaml'
REAL_YAML = TOOLS.parent / 'gz-collections.yaml'


def test_libraries_are_the_ones_with_a_job_on_the_platform():
    assert find_conda_libraries(FIXTURE, 'windows', 'amd64') == {
        'gz-both': True,    # any config of the platform is enough
        'gz-cpu': False,    # nvidia_gpu of linux and windows arm64 do not count
        'gz-gpu': True,
        'gz-new': False,    # null requirements
    }


def test_only_conda_configs_count():
    # mac_brew would add gz-cpu, gz-gpu and gz-old and a GPU for gz-both
    assert find_conda_libraries(FIXTURE, 'darwin', 'arm64') == {
        'gz-both': False,
        'gz-new': False,
    }


def test_platform_without_configs_has_no_libraries():
    assert find_conda_libraries(FIXTURE, 'freebsd', 'amd64') == {}


def test_missing_file_fails():
    with pytest.raises(FileNotFoundError, match='YAML file not found'):
        find_conda_libraries(FIXTURE.with_name('missing.yaml'), 'windows',
                             'amd64')


def test_real_collections_have_no_job_for_the_metapackages():
    libraries = find_conda_libraries(REAL_YAML, 'windows', 'amd64')
    assert 'gz-math' in libraries
    assert not {'gz-fortress', 'gz-harmonic', 'gz-ionic', 'gz-jetty',
                'gz-rotary'} & set(libraries)


def test_real_collections_on_macos_have_no_metapackages_and_no_gpu():
    data = yaml.safe_load(REAL_YAML.read_text())
    metapackages = {'gz-' + c['name'] for c in data['collections']}
    libraries = find_conda_libraries(REAL_YAML, 'darwin', 'arm64')
    assert {'gz-math', 'gz-sim'} <= set(libraries)
    assert not metapackages & set(libraries)
    # No GPU requirement on macOS: the agents always have an Apple GPU
    assert not any(libraries.values())
