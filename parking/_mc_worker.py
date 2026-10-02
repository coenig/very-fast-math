"""Standalone, killable vfm script runner.

Usage: python3 parking/_mc_worker.py "<vfm-script>"

Run as its own OS process (started in a new session by the GUI) so the parker can kill
it and its nuXmv children once another variant wins the race. Loads libvfm directly and
does not import PyQt, keeping the process light and independent of the GUI.
"""
import sys
import os
import platform
import ctypes
import signal
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _install_orphan_guard():
    """Kill this worker's whole process group (incl. nuXmv children) if the launcher dies.

    The GUI/CLI starts us in a new session (start_new_session) so it can kill the losing
    variants once a winner is found. If that controller crashes or its terminal closes before
    the race resolves, we would otherwise keep model-checking every variant (incl. the slow
    high-section ones) to completion. Watch for reparenting (parent pid changes) and, when it
    happens, SIGKILL the process group so no orphaned nuXmv jobs are left behind.
    """
    if platform.system() == 'Windows':
        return  # No POSIX sessions/getppid; the GUI uses taskkill /T for the tree on Windows.
    initial_ppid = os.getppid()
    if initial_ppid <= 1:
        return  # Already detached on purpose (e.g. nohup); nothing to guard against.

    def _watch():
        while True:
            time.sleep(1.0)
            if os.getppid() != initial_ppid:  # launcher exited -> we got reparented
                try:
                    os.killpg(os.getpgrp(), signal.SIGKILL)
                finally:
                    os._exit(1)

    threading.Thread(target=_watch, daemon=True).start()


def main():
    if len(sys.argv) < 2:
        sys.stderr.write("Missing script argument.\n")
        return 1

    _install_orphan_guard()

    script = sys.argv[1]

    dll_name = 'libvfm.so'
    dll_dir = os.path.join(REPO_ROOT, 'lib')
    if platform.system() == 'Windows':
        dll_name = 'VFM_MAIN_LIB.dll'
        dll_dir = os.path.join(REPO_ROOT, 'bin')
        os.environ['PATH'] = dll_dir + os.pathsep + os.environ.get('PATH', '')

    # Config relative paths (../examples, ../external, ...) resolve from bin/.
    os.chdir(os.path.join(REPO_ROOT, 'bin'))

    lib = ctypes.CDLL(os.path.join(dll_dir, dll_name))
    lib.expandScript.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t]
    lib.expandScript.restype = ctypes.c_char_p

    buf = ctypes.create_string_buffer(4000000)
    res = lib.expandScript(script.encode('utf-8'), buf, ctypes.sizeof(buf))
    sys.stdout.write(res.decode(errors='replace') if res else "")
    return 0


if __name__ == "__main__":
    sys.exit(main())
