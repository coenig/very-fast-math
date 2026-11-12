"""Local worker limits; the campaign coordinator stays outside the limit.

RLIMIT_AS is an inherited hard *per-process virtual-address-space* limit.
A parent watchdog additionally samples summed RSS for the entire worker tree.
This is deliberately not advertised as an atomic cgroup memory limit.
"""
from __future__ import annotations

import math
from pathlib import Path
import re
import subprocess
import time

GIB = 1024 ** 3
OOM = re.compile(r'MemoryError|std::bad_alloc|out of memory|cannot allocate memory|'
                 r'failed to allocate memory|memory exhausted|memory allocation (?:failed|failure)', re.I)


def limit_bytes(memory_gib):
    if memory_gib is None:
        return None
    if not math.isfinite(memory_gib) or memory_gib <= 0:
        raise ValueError('memory limit must be a positive finite number of GiB')
    return max(1, int(memory_gib * GIB))


def install(memory_gib):
    """Call in the worker, before native imports; never preexec_fn in threads."""
    requested = limit_bytes(memory_gib)
    if requested is None:
        return
    import resource
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    limits = [requested] + [v for v in (soft, hard) if v != resource.RLIM_INFINITY]
    cap = min(limits)
    resource.setrlimit(resource.RLIMIT_AS, (cap, cap))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def memory_error(text):
    return bool(OOM.search(text))


def log_memory_error(path):
    """Only read a small diagnostic tail, not the native template expansion."""
    try:
        with Path(path).open('rb') as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - 16384))
            return memory_error(stream.read().decode(errors='replace'))
    except OSError:
        return False


def watch(process, timeout, memory_gib=None, poll_seconds=.2):
    """Wait, record tree RSS, and terminate only this attempt on a limit.

    Track Process objects as well as parentage: solver children can start new
    sessions, and previously observed children can outlive a crashed worker.
    psutil checks creation times before signalling, protecting against PID reuse.
    """
    import psutil
    started = time.monotonic()
    cap = limit_bytes(memory_gib)
    try:
        parent = psutil.Process(process.pid)
    except psutil.NoSuchProcess:
        process.wait()
        return dict(termination_reason=None,returncode=process.returncode,
                    peak_tree_rss_bytes=0,memory_limit_gib=memory_gib,
                    memory_policy='worker_exited_before_first_sample',
                    memory_poll_seconds=poll_seconds,wall_seconds=time.monotonic()-started)
    known = {parent.pid: parent}
    peak = 0
    reason = None

    def sample():
        nonlocal peak
        try:
            known.update((p.pid, p) for p in parent.children(recursive=True))
        except psutil.NoSuchProcess:
            pass
        rss = 0
        for pid, child in list(known.items()):
            try:
                if not child.is_running():
                    known.pop(pid, None)
                    continue
                rss += child.memory_info().rss
            except psutil.NoSuchProcess:
                known.pop(pid, None)
        peak = max(peak, rss)
        return rss

    def cleanup():
        # Freeze known processes before discovering any last children. Do not
        # rely on one process group: native generators/solvers use setsid().
        for _ in range(2):
            sample()
            for child in list(known.values()):
                try: child.suspend()
                except psutil.NoSuchProcess: pass
        for child in reversed(list(known.values())):
            try: child.kill()
            except psutil.NoSuchProcess: pass
        process.wait()

    try:
        while process.poll() is None:
            rss = sample()
            if cap is not None and rss > cap:
                reason = 'memory_limit'
                break
            if time.monotonic() - started >= timeout:
                reason = 'timeout'
                break
            try: process.wait(timeout=min(poll_seconds, max(.001, timeout-(time.monotonic()-started))))
            except subprocess.TimeoutExpired: pass
        # Also clean up observed children after a worker crash/normal exit.
        cleanup()
    except BaseException:
        cleanup()
        raise
    return dict(termination_reason=reason, returncode=process.returncode,
                peak_tree_rss_bytes=peak, memory_limit_gib=memory_gib,
                memory_policy='inherited_per_process_RLIMIT_AS_and_sampled_tree_RSS' if cap else 'unlimited',
                memory_poll_seconds=poll_seconds, wall_seconds=time.monotonic()-started)
