"""The jobserver arguments ninja and colcon-cmake read from MAKEFLAGS."""

import re

from pixi_ci import jobserver

# colcon-cmake 0.2.28 build.py, _get_make_arguments: a match keeps it from
# passing -jN -lN to ninja
COLCON_CMAKE_JOBS = (r'(?:^|\s)(-?(?:j|l)(?:\s*[0-9]+|\s|$))'
                     r'|(?:^|\s)((?:--)?(?:jobs|load-average)'
                     r'(?:(?:=|\s+)[0-9]+|(?:\s|$)))')


def test_makeflags_name_the_semaphore_after_a_job_count():
    flags = jobserver.makeflags(5, "pixi_ci_jobserver_7")
    assert flags == "-j5 --jobserver-auth=pixi_ci_jobserver_7"
    assert re.findall(COLCON_CMAKE_JOBS, flags)
    # ninja reads the last --jobserver-auth= word; a value that is neither
    # R,W nor fifo:PATH is a Windows semaphore name
    assert flags.split()[-1].split("=", 1) == ["--jobserver-auth",
                                               "pixi_ci_jobserver_7"]


def test_one_ninja_alone_runs_as_many_jobs_as_slots():
    assert jobserver.tokens_for(4) == 3
    assert jobserver.tokens_for(1) == 0
