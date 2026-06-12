import os
import re
import math
import hashlib
import shutil
from pathlib import Path
from typing import List, Dict, Optional
import numpy as np
import platform
import ctypes
from contextlib import contextmanager
from ctypes import CDLL


def ensure_empty_file(path: str) -> None:
    p = Path(path)

    if p.exists() and not p.is_file():
        return

    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('w'):
        pass

def min_max_curr(successful_so_far, done_so_far, max_to_expect):
    percent = 100 * successful_so_far / done_so_far
    min_good = 100 * successful_so_far / max_to_expect
    max_good = 100 * (max_to_expect - done_so_far + successful_so_far) / max_to_expect
        
    return str(round(min_good, 1)) + "% <= " + str(round(percent, 1)) + "% <= " + str(round(max_good, 1)) + "%"

# Pure pursuit.
def dpoint_following_angle(dpoint_y, ego_y, heading, ddist, bwd):
    return heading - math.atan((dpoint_y - ego_y) * bwd / ddist)

def maxDifferenceArray(A):
    maxDiff = -1
    for i in range(len(A)):
        for j in range(len(A)):
            maxDiff = max(maxDiff, abs(A[j] - A[i]))
    return maxDiff

def inverseSortingArray(egos_x: List[float]):
    b = True
    for i in range(len(egos_x) - 1):
        b = b and (egos_x[i + 1] < egos_x[i])
    return b


def _latest_nuxmv_runtime_seconds(config_name: str, generated_path_prefix: str):
    """Return latest nuXmv runtime in seconds from mc_runtimes.txt for a given config.

    Parameters:
    - config_name: name of the config directory
    - generated_path_prefix: prefix path where config dirs live
    """
    runtime_file = os.path.join(generated_path_prefix + config_name, 'mc_runtimes.txt')
    if not os.path.exists(runtime_file):
        return np.nan

    try:
        with open(runtime_file, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()
    except OSError:
        return np.nan

    for line in reversed(lines):
        if "nuXmv" not in line:
            continue
        m = re.search(r"(\d+):(\d+):(\d+(?:\.\d+)?)\s+time elapsed", line)
        if m:
            hours = int(m.group(1))
            minutes = int(m.group(2))
            seconds = float(m.group(3))
            return hours * 3600 + minutes * 60 + seconds

    return np.nan
 
def _hash_file(path: str) -> str:
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(8192), b''):
            h.update(chunk)
    return h.hexdigest()

def _snapshot_configs(ucd_config_prios_str: List[str], generated_path_prefix: str, restrict_to: Optional[Dict[str, List[str]]] = None):
    """Return {config_name: {rel_path: md5_hex}} for all config dirs.
    If restrict_to is given, only include rel_paths present in that dict."""
    snap = {}
    for config_name in ucd_config_prios_str:
        config_dir = generated_path_prefix + config_name
        allowed = restrict_to.get(config_name) if restrict_to else None
        file_hashes = {}
        for dirpath, _, filenames in os.walk(config_dir):
            for fname in filenames:
                full_path = os.path.join(dirpath, fname)
                rel_path = os.path.relpath(full_path, config_dir)
                if allowed is not None and rel_path not in allowed:
                    continue
                file_hashes[rel_path] = _hash_file(full_path)
        snap[config_name] = file_hashes
    return snap

def _save_configs_to_archive(seedo: int, subfolder: str, generated_path_prefix: str, ucd_config_prios_str: List[str], restrict_to: Optional[Dict[str, List[str]]] = None):
    """Copy config files into the detailed archive under the given subfolder.
    If restrict_to is given ({config_name: set_of_rel_paths}), only copy those files."""
    archive_path = f'{generated_path_prefix}/detailed_archive/run_{seedo}/{subfolder}/'
    for config_name in ucd_config_prios_str:
        config_dir = generated_path_prefix + config_name
        allowed = restrict_to.get(config_name) if restrict_to else None
        for dirpath, _, filenames in os.walk(config_dir):
            for fname in filenames:
                full_path = os.path.join(dirpath, fname)
                rel_path = os.path.relpath(full_path, config_dir)
                if allowed is not None and rel_path not in allowed:
                    continue
                dest = os.path.join(archive_path + config_name, rel_path)
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                shutil.copy2(full_path, dest)

def archive(seedo: int, global_counter: int, detailed_archive_flag: bool, generated_path_prefix: str, ucd_config_prios_str: List[str], snapshot_hashes: Dict[str, Dict[str, str]]):
    if detailed_archive_flag:
        archive_path = f'{generated_path_prefix}/detailed_archive/run_{seedo}/iteration_{global_counter}/'
        if not os.path.exists(archive_path):
            os.makedirs(archive_path)
        for config_name in ucd_config_prios_str:
            config_dir = generated_path_prefix + config_name
            baseline = snapshot_hashes.get(config_name, {})
            for dirpath, _, filenames in os.walk(config_dir):
                for fname in filenames:
                    full_path = os.path.join(dirpath, fname)
                    rel_path = os.path.relpath(full_path, config_dir)
                    current_hash = _hash_file(full_path)
                    if rel_path not in baseline or baseline[rel_path] != current_hash:
                        dest = os.path.join(archive_path + config_name, rel_path)
                        os.makedirs(os.path.dirname(dest), exist_ok=True)
                        shutil.copy2(full_path, dest)

if platform.system() == 'Windows':
    kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel32.FreeLibrary.argtypes = [ctypes.c_void_p] # The argument is a handle (void pointer)
    kernel32.FreeLibrary.restype = ctypes.c_int      # Returns a BOOL (int)
    
@contextmanager
def clean_library_context(lib_path):
    try:
        lib = ctypes.CDLL(lib_path)
        yield lib
    finally:
        if 'lib' in locals():
            handle = lib._handle
            
            if platform.system() == 'Windows':
                kernel32.FreeLibrary(handle)
            else:
                ctypes.CDLL(None).dlclose(handle)
        else:
            print("Warning: Library object not found for cleanup.")
            input("Press Enter to continue...")
               
@contextmanager
def morty_script_context():
    # Assum Linux until...
    dll_name = 'libvfm.so'
    dll_dir = './lib'
    
    if platform.system() == 'Windows': # ...proven otherwise.
        dll_name = 'VFM_MAIN_LIB.dll'
        dll_dir = os.path.join(os.getcwd(), 'bin', 'Release')
        # Ensure the directory with native DLLs is discoverable on Windows so CDLL can load dependencies.
        # Add bin/Release to PATH if not already present (works on Python 3.7+).
        if dll_dir not in os.environ.get('PATH', ''):
            os.environ['PATH'] = dll_dir + os.pathsep + os.environ.get('PATH', '')
   
    with clean_library_context(dll_dir + '/' + dll_name) as morty_lib:
        morty_lib = CDLL(dll_dir + '/' + dll_name)
        morty_lib.expandScript.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t]
        morty_lib.expandScript.restype = ctypes.c_char_p
        
        yield morty_lib

# --- How to use
# for item in [...]:
#     with morty_script_context() as morty_lib:
#         # morty_lib is already loaded, configured, and will be cleaned up
#         res1 = morty_lib.expandScript(b"call_one", b"data", 1024)
#         res2 = morty_lib.expandScript(b"call_two", b"data", 1024)
