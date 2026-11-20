"""Memory use of the agent while colcon builds.

A thread samples the system commit charge, the available RAM and the
compilers and linkers running, and each build phase ends with one summary
line. The report never fails a job: a probe that stops working only prints
why.
"""

import contextlib
import os
import threading
import urllib.request
from dataclasses import dataclass, field

SAMPLE_SECONDS = 2.0
GIB = 1024 ** 3
COMPILER = "cl.exe"
LINKER = "link.exe"


@dataclass
class ToolUse:
    """Processes of one tool (cl.exe, link.exe) in one sample."""
    count: int = 0
    private: int = 0            # bytes committed by all of them together
    largest_peak: int = 0       # largest commit peak of one of them


@dataclass
class Sample:
    commit: int                 # bytes committed in the whole system
    commit_limit: int           # RAM + page files
    commit_peak: int            # highest commit since the system started
    ram: int
    ram_available: int
    tools: dict = field(default_factory=dict)   # name -> ToolUse


class PhaseStats:
    """Peaks of the samples taken during one build phase."""

    def __init__(self, label):
        self.label = label
        self.samples = 0
        self.error = None
        self.first = None
        self.commit = 0
        self.commit_limit = 0
        self.commit_peak = 0
        self.ram = 0
        self.ram_available = None
        self.tools = {}

    def add(self, sample):
        if self.first is None:
            self.first = sample
        self.samples += 1
        self.commit = max(self.commit, sample.commit)
        self.commit_limit = sample.commit_limit
        self.commit_peak = max(self.commit_peak, sample.commit_peak)
        self.ram = sample.ram
        if self.ram_available is None:
            self.ram_available = sample.ram_available
        self.ram_available = min(self.ram_available, sample.ram_available)
        for name, use in sample.tools.items():
            peak = self.tools.setdefault(name, ToolUse())
            peak.count = max(peak.count, use.count)
            peak.private = max(peak.private, use.private)
            peak.largest_peak = max(peak.largest_peak, use.largest_peak)

    def summary(self):
        if self.samples == 0:
            return f"memory {self.label}: no samples ({self.error})"
        commit = f"peak commit {gib(self.commit)} of {gib(self.commit_limit)}"
        # The system peak is exact; the samples can miss a short spike
        if self.commit_peak > max(self.commit, self.first.commit_peak):
            commit += f" (system peak {gib(self.commit_peak)})"
        parts = [commit,
                 f"lowest available RAM {gib(self.ram_available)} of "
                 f"{gib(self.ram)}"]
        for name in (COMPILER, LINKER):
            use = self.tools.get(name)
            if use and use.count:
                parts.append(f"up to {use.count} {name} using "
                             f"{gib(use.private)}, the largest "
                             f"{gib(use.largest_peak)}")
        line = f"memory {self.label}: " + ", ".join(parts)
        line += f" ({self.samples} samples every {SAMPLE_SECONDS:g} s)"
        if self.error is not None:
            line += f"; sampling stopped: {self.error}"
        return line


def gib(size):
    return f"{size / GIB:.1f} GiB"


@contextlib.contextmanager
def watch(probe, label, interval=SAMPLE_SECONDS):
    """Sample the memory while the block runs, then print the summary."""
    stats = PhaseStats(label)
    stop = threading.Event()

    def take():
        if stats.error is None:
            try:
                stats.add(probe.sample())
            except Exception as error:  # the report must not fail the job
                stats.error = f"{type(error).__name__}: {error}"

    def loop():
        while not stop.wait(interval) and stats.error is None:
            take()

    take()
    thread = threading.Thread(target=loop, name=f"memory {label}",
                              daemon=True)
    thread.start()
    try:
        yield stats
    finally:
        stop.set()
        thread.join()
        take()
        print(stats.summary(), flush=True)


class UnavailableProbe:
    """Stands in for a probe that could not be created."""

    def __init__(self, error):
        self.error = error

    def sample(self):
        raise RuntimeError(self.error)

    def describe(self):
        raise RuntimeError(self.error)


def create_probe(factory):
    try:
        return factory()
    except Exception as error:  # the report must not fail the job
        return UnavailableProbe(f"{type(error).__name__}: {error}")


def describe_agent(probe):
    """One line with the instance type, CPUs, RAM and commit limit."""
    try:
        return probe.describe()
    except Exception as error:  # the report must not fail the job
        return f"agent: unknown ({type(error).__name__}: {error})"


def aws_instance_type(timeout=1.0):
    """EC2 instance type from the instance metadata (IMDSv2), or None."""
    base = "http://169.254.169.254/latest"
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        request = urllib.request.Request(
            f"{base}/api/token", method="PUT",
            headers={"X-aws-ec2-metadata-token-ttl-seconds": "60"})
        with opener.open(request, timeout=timeout) as response:
            token = response.read().decode()
        request = urllib.request.Request(
            f"{base}/meta-data/instance-type",
            headers={"X-aws-ec2-metadata-token": token})
        with opener.open(request, timeout=timeout) as response:
            return response.read().decode().strip()
    except OSError:
        return None


class WindowsMemoryProbe:
    """Commit charge and RAM from GetPerformanceInfo, compilers and linkers
    from a process snapshot and GetProcessMemoryInfo. Only ctypes, so the
    bootstrap env needs nothing more."""

    TOOLS = (COMPILER, LINKER)

    def __init__(self):
        # Imported here: the tests import this module on Linux
        import ctypes
        from ctypes import wintypes
        self._ctypes = ctypes
        self._kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

        class PerformanceInformation(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD),
                        ("CommitTotal", ctypes.c_size_t),
                        ("CommitLimit", ctypes.c_size_t),
                        ("CommitPeak", ctypes.c_size_t),
                        ("PhysicalTotal", ctypes.c_size_t),
                        ("PhysicalAvailable", ctypes.c_size_t),
                        ("SystemCache", ctypes.c_size_t),
                        ("KernelTotal", ctypes.c_size_t),
                        ("KernelPaged", ctypes.c_size_t),
                        ("KernelNonpaged", ctypes.c_size_t),
                        ("PageSize", ctypes.c_size_t),
                        ("HandleCount", wintypes.DWORD),
                        ("ProcessCount", wintypes.DWORD),
                        ("ThreadCount", wintypes.DWORD)]

        class ProcessEntry(ctypes.Structure):
            _fields_ = [("dwSize", wintypes.DWORD),
                        ("cntUsage", wintypes.DWORD),
                        ("th32ProcessID", wintypes.DWORD),
                        ("th32DefaultHeapID", ctypes.c_size_t),
                        ("th32ModuleID", wintypes.DWORD),
                        ("cntThreads", wintypes.DWORD),
                        ("th32ParentProcessID", wintypes.DWORD),
                        ("pcPriClassBase", wintypes.LONG),
                        ("dwFlags", wintypes.DWORD),
                        ("szExeFile", wintypes.WCHAR * 260)]

        class ProcessMemoryCounters(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD),
                        ("PageFaultCount", wintypes.DWORD),
                        ("PeakWorkingSetSize", ctypes.c_size_t),
                        ("WorkingSetSize", ctypes.c_size_t),
                        ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                        ("PagefileUsage", ctypes.c_size_t),
                        ("PeakPagefileUsage", ctypes.c_size_t),
                        ("PrivateUsage", ctypes.c_size_t)]

        self._PerformanceInformation = PerformanceInformation
        self._ProcessEntry = ProcessEntry
        self._ProcessMemoryCounters = ProcessMemoryCounters
        k = self._kernel32
        k.K32GetPerformanceInfo.argtypes = [ctypes.c_void_p, wintypes.DWORD]
        k.K32GetPerformanceInfo.restype = wintypes.BOOL
        k.CreateToolhelp32Snapshot.argtypes = [wintypes.DWORD, wintypes.DWORD]
        k.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
        k.Process32FirstW.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
        k.Process32FirstW.restype = wintypes.BOOL
        k.Process32NextW.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
        k.Process32NextW.restype = wintypes.BOOL
        k.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL,
                                  wintypes.DWORD]
        k.OpenProcess.restype = wintypes.HANDLE
        k.K32GetProcessMemoryInfo.argtypes = [wintypes.HANDLE,
                                              ctypes.c_void_p,
                                              wintypes.DWORD]
        k.K32GetProcessMemoryInfo.restype = wintypes.BOOL
        k.CloseHandle.argtypes = [wintypes.HANDLE]
        k.CloseHandle.restype = wintypes.BOOL

    def _error(self, function):
        code = self._ctypes.get_last_error()
        return OSError(code, f"{function} failed: "
                             f"{self._ctypes.FormatError(code)}")

    def _performance(self):
        info = self._PerformanceInformation()
        info.cb = self._ctypes.sizeof(info)
        if not self._kernel32.K32GetPerformanceInfo(
                self._ctypes.byref(info), info.cb):
            raise self._error("GetPerformanceInfo")
        return info

    def _tool_processes(self):
        """(exe name, pid) of the compilers and linkers running."""
        snapshot_process = 0x2              # TH32CS_SNAPPROCESS
        invalid_handle = self._ctypes.c_void_p(-1).value
        snapshot = self._kernel32.CreateToolhelp32Snapshot(snapshot_process, 0)
        if snapshot in (None, invalid_handle):
            raise self._error("CreateToolhelp32Snapshot")
        try:
            entry = self._ProcessEntry()
            entry.dwSize = self._ctypes.sizeof(entry)
            found = []
            ok = self._kernel32.Process32FirstW(snapshot,
                                                self._ctypes.byref(entry))
            while ok:
                name = entry.szExeFile.lower()
                if name in self.TOOLS:
                    found.append((name, entry.th32ProcessID))
                ok = self._kernel32.Process32NextW(snapshot,
                                                   self._ctypes.byref(entry))
            return found
        finally:
            self._kernel32.CloseHandle(snapshot)

    def _process_memory(self, pid):
        """(private bytes, peak private bytes) of pid, or None if it is
        gone or can not be opened."""
        # PROCESS_QUERY_LIMITED_INFORMATION | PROCESS_VM_READ
        access = 0x1000 | 0x0010
        handle = self._kernel32.OpenProcess(access, False, pid)
        if not handle:
            return None
        try:
            counters = self._ProcessMemoryCounters()
            counters.cb = self._ctypes.sizeof(counters)
            if not self._kernel32.K32GetProcessMemoryInfo(
                    handle, self._ctypes.byref(counters), counters.cb):
                return None
            return counters.PrivateUsage, counters.PeakPagefileUsage
        finally:
            self._kernel32.CloseHandle(handle)

    def sample(self):
        info = self._performance()
        page = info.PageSize
        tools = {name: ToolUse() for name in self.TOOLS}
        for name, pid in self._tool_processes():
            memory = self._process_memory(pid)
            if memory is None:
                continue
            private, peak = memory
            use = tools[name]
            use.count += 1
            use.private += private
            use.largest_peak = max(use.largest_peak, peak)
        return Sample(commit=info.CommitTotal * page,
                      commit_limit=info.CommitLimit * page,
                      commit_peak=info.CommitPeak * page,
                      ram=info.PhysicalTotal * page,
                      ram_available=info.PhysicalAvailable * page,
                      tools=tools)

    def describe(self):
        info = self._performance()
        page = info.PageSize
        instance = aws_instance_type() or "not an EC2 instance"
        return (f"agent: {instance}, {os.cpu_count()} CPUs, "
                f"{gib(info.PhysicalTotal * page)} RAM, commit limit "
                f"{gib(info.CommitLimit * page)}")
