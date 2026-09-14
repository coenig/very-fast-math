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

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    if len(sys.argv) < 2:
        sys.stderr.write("Missing script argument.\n")
        return 1

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
