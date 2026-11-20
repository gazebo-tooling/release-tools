"""The memory report: peaks, summary line, and probes that fail."""

import time
import urllib.request

import pytest

from pixi_ci import memory
from pixi_ci.memory import GIB, PhaseStats, Sample, ToolUse


def sample(commit, available, peak=None, cl=(0, 0, 0), link=(0, 0, 0)):
    return Sample(commit=commit * GIB, commit_limit=20 * GIB,
                  commit_peak=(peak or commit) * GIB, ram=16 * GIB,
                  ram_available=available * GIB,
                  tools={"cl.exe": ToolUse(cl[0], cl[1] * GIB, cl[2] * GIB),
                         "link.exe": ToolUse(link[0], link[1] * GIB,
                                             link[2] * GIB)})


class ListProbe:
    """Returns the given samples, then raises the given error."""

    def __init__(self, samples, error=None):
        self.samples = list(samples)
        self.error = error

    def sample(self):
        if self.samples:
            return self.samples.pop(0)
        raise self.error or AssertionError("no samples left")


def test_summary_keeps_the_peak_of_every_value():
    stats = PhaseStats("gz-sim")
    stats.add(sample(6, 9, cl=(4, 3, 1)))
    stats.add(sample(13, 2, cl=(16, 10, 1.5), link=(1, 2, 2)))
    stats.add(sample(8, 7, cl=(2, 1, 1), link=(2, 3, 2.5)))
    assert stats.summary() == (
        "memory gz-sim: peak commit 13.0 GiB of 20.0 GiB, lowest available "
        "RAM 2.0 GiB of 16.0 GiB, up to 16 cl.exe using 10.0 GiB, the "
        "largest 1.5 GiB, up to 2 link.exe using 3.0 GiB, the largest "
        "2.5 GiB (3 samples every 2 s)")


def test_summary_adds_the_system_peak_the_samples_missed():
    stats = PhaseStats("gz-sim")
    stats.add(sample(6, 9, peak=8))
    stats.add(sample(7, 9, peak=15))
    assert "peak commit 7.0 GiB of 20.0 GiB (system peak 15.0 GiB)" in \
        stats.summary()


def test_summary_ignores_a_system_peak_older_than_the_phase():
    stats = PhaseStats("gz-sim")
    stats.add(sample(6, 9, peak=15))
    stats.add(sample(7, 9, peak=15))
    assert "system peak" not in stats.summary()


def test_summary_leaves_out_tools_that_did_not_run():
    stats = PhaseStats("gz-cmake")
    stats.add(sample(5, 10))
    assert "cl.exe" not in stats.summary()
    assert "link.exe" not in stats.summary()


def test_watch_samples_before_and_after_the_block(capsys):
    probe = ListProbe([sample(5, 10), sample(9, 4)])
    with memory.watch(probe, "gz-math", interval=60):
        pass
    assert capsys.readouterr().out.startswith(
        "memory gz-math: peak commit 9.0 GiB of 20.0 GiB, lowest available "
        "RAM 4.0 GiB of 16.0 GiB")


def test_watch_samples_while_the_block_runs(capsys):
    probe = ListProbe([sample(5, 10)] * 10000)
    deadline = time.monotonic() + 10
    with memory.watch(probe, "gz-math", interval=0.001) as stats:
        while stats.samples < 5 and time.monotonic() < deadline:
            time.sleep(0.001)
    assert stats.samples >= 6


def test_watch_reports_the_summary_when_the_build_fails(capsys):
    probe = ListProbe([sample(5, 10), sample(6, 9)])
    with pytest.raises(RuntimeError, match="colcon failed"):
        with memory.watch(probe, "gz-math", interval=60):
            raise RuntimeError("colcon failed")
    assert capsys.readouterr().out.startswith("memory gz-math: peak commit")


def test_a_failing_probe_does_not_fail_the_build(capsys):
    probe = ListProbe([], error=OSError(5, "access denied"))
    with memory.watch(probe, "gz-math", interval=60):
        pass
    assert capsys.readouterr().out == (
        "memory gz-math: no samples (OSError: [Errno 5] access denied)\n")


def test_a_probe_failing_midway_keeps_what_it_sampled(capsys):
    probe = ListProbe([sample(5, 10)], error=OSError(5, "access denied"))
    with memory.watch(probe, "gz-math", interval=60):
        pass
    out = capsys.readouterr().out
    assert out.startswith("memory gz-math: peak commit 5.0 GiB")
    assert out.rstrip().endswith(
        "; sampling stopped: OSError: [Errno 5] access denied")


def test_a_probe_that_can_not_be_created_reports_why(capsys):
    def factory():
        raise AttributeError("module 'ctypes' has no attribute 'WinDLL'")

    probe = memory.create_probe(factory)
    assert memory.describe_agent(probe) == (
        "agent: unknown (RuntimeError: AttributeError: module 'ctypes' has "
        "no attribute 'WinDLL')")
    with memory.watch(probe, "gz-math", interval=60):
        pass
    assert capsys.readouterr().out.startswith("memory gz-math: no samples")


def test_instance_type_is_none_outside_ec2(monkeypatch):
    class Opener:
        def open(self, request, timeout):
            raise urllib.error.URLError("timed out")

    monkeypatch.setattr(urllib.request, "build_opener", lambda *_: Opener())
    assert memory.aws_instance_type() is None
