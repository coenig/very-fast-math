"""Persistent nuXmv sessions for live planning.

Each configuration keeps one interactive nuXmv process with its model parsed, flattened and encoded once
(``go_msat``). Per query only the BMC check runs. The per-query observation constraints (the native
``-- INIT CONSTRAINTS`` block) are moved into the checked property via frozen copies of the observed
variables: ``INIT __obs_i = var`` in the static model plus ``__obs_i = value`` in the property. Since a
frozen variable keeps its initial value, ``!(obs & goal)`` is violated at depth k exactly by the paths
that the original model with ``INIT var = value`` and ``INVARSPEC !(goal)`` admits. The ``__`` prefix
keeps the copies out of printed traces.
"""
import os
import platform
import queue
import re
import subprocess
import threading
import time
from pathlib import Path

INIT_BEGIN = '-- INIT CONSTRAINTS'
INIT_END = '-- EO INIT CONSTRAINTS'
SENTINEL = '@@VFM_PERSISTENT_DONE@@'
PROMPT = 'nuXmv > '
_INIT_RE = re.compile(r'^INIT\s+(!?)\s*([A-Za-z_][\w.]*)\s*(?:=\s*(-?\d+))?\s*;$')


def split_live_model(main_smv):
    """Return (header, body, observations, goal) or None if the layout is unsupported.

    header: leading part up to the last preprocessor directive; body: the rest without the observation
    block and the INVARSPEC; observations: [(variable, negated, value or None)]; goal: the X of INVARSPEC !(X).
    """
    lines = main_smv.split('\n')
    header_end = max((i for i, line in enumerate(lines) if line.lstrip().startswith('#')), default=-1)
    header = '\n'.join(lines[:header_end + 1])
    rest = '\n'.join(lines[header_end + 1:])

    begin, end = rest.find(INIT_BEGIN), rest.find(INIT_END)
    if begin == -1 or end == -1 or end < begin:
        return None
    observations = []
    for line in rest[begin + len(INIT_BEGIN):end].split('\n'):
        line = line.strip()
        if not line:
            continue
        match = _INIT_RE.match(line)
        if not match:
            return None
        observations.append((match.group(2), match.group(1) == '!', match.group(3)))
    rest = rest[:begin] + rest[end + len(INIT_END):]

    spec_begin = rest.find('INVARSPEC')
    spec_end = rest.find(';', spec_begin)
    if spec_begin == -1 or spec_end == -1:
        return None
    spec = ' '.join(rest[spec_begin + len('INVARSPEC'):spec_end].split())
    if not (spec.startswith('!(') and spec.endswith(')')) or '"' in spec:
        return None
    body = rest[:spec_begin] + rest[spec_end + 1:]
    for define in re.findall(r'^\s*#\s*define\s+(\w+)', header, re.M):
        if define in body or define in spec:
            return None  # The remainder relies on macro expansion.
    return header, body, observations, spec[2:-1]


def static_model(preprocessed_header, body, observations):
    frozen = ''.join(f'  __obs_{i} : {"boolean" if value is None else "integer"};\n'
                     for i, (_, _, value) in enumerate(observations))
    links = ''.join(f'INIT __obs_{i} = {variable};\n' for i, (variable, _, _) in enumerate(observations))
    return preprocessed_header + '\n' + body + '\nFROZENVAR\n' + frozen + links


def observation_property(observations, goal):
    terms = [f'__obs_{i} = {value}' if value is not None else (f'!__obs_{i}' if negated else f'__obs_{i}')
             for i, (_, negated, value) in enumerate(observations)]
    return '!(' + ' & '.join(terms + ['(' + goal + ')']) + ')'


class NuxmvSession:
    def __init__(self, command, model_path, timeout, env=None):
        self.process = subprocess.Popen(command + ['-int', str(model_path)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        self.lines = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()
        output = self.command('set on_failure_script_quits 0\ngo_msat', timeout)
        if 'rror' in output:
            self.close()
            raise RuntimeError('nuXmv could not build the persistent model:\n' + output[-2000:])

    def _read(self):
        for line in self.process.stdout:
            self.lines.put(line)
        self.process.stdout.close()
        self.lines.put(None)

    def command(self, text, timeout):
        self.process.stdin.write(text + f'\necho "{SENTINEL}"\n')
        self.process.stdin.flush()
        deadline = time.monotonic() + timeout
        output = []
        while True:
            try:
                line = self.lines.get(timeout=max(deadline - time.monotonic(), 0.001))
            except queue.Empty:
                self.close()
                raise TimeoutError(f'nuXmv did not answer within {timeout} s')
            if line is None:
                raise RuntimeError('nuXmv terminated:\n' + ''.join(output)[-2000:])
            if SENTINEL in line:
                return ''.join(output).replace(PROMPT, '')
            output.append(line)

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait()
        try:
            self.process.stdin.close()
        except OSError:
            pass


class PersistentSolverPool:
    """One NuxmvSession per configuration package, rebuilt whenever its static model changes."""

    def __init__(self, external_path, timeout=300):
        system = platform.system()
        folder = Path(external_path).resolve() / ('win32' if system == 'Windows' else 'linux64')
        self.command = [str(folder / ('nuXmv.exe' if system == 'Windows' else 'nuXmv'))]
        self.cpp = [str(folder / 'cpp.exe')] if system == 'Windows' else [os.environ.get('CPP') or 'cpp']
        self.env = None
        if (folder / 'lib').is_dir():
            self.env = dict(os.environ)
            self.env['LD_LIBRARY_PATH'] = str(folder / 'lib') + (':' + os.environ['LD_LIBRARY_PATH']
                                                                 if os.environ.get('LD_LIBRARY_PATH') else '')
        self.timeout = timeout
        self.sessions = {}  # package -> (static model text, session)
        self.headers = {}  # package -> (header + include contents, preprocessed header)

    def _preprocess(self, package, header):
        names = re.findall(r'^\s*#\s*include\s+"([^"]+)"', header, re.M)
        for name in names:
            included, source = package / name, package / name.removesuffix('.smv')
            if not included.exists() or (name.endswith('.smv') and source.exists()
                                         and source.stat().st_mtime > included.stat().st_mtime):
                return None  # E.g. the planner translation is created by the native path.
        includes = ''.join((package / name).read_text() + '\0' for name in names)
        key = header + '\0' + includes
        cached = self.headers.get(package)
        if cached is None or cached[0] != key:
            path = package / 'main_persistent_header.smv'
            path.write_text(header + '\n')
            result = subprocess.run(self.cpp + [path.name], cwd=package, capture_output=True, text=True, env=self.env)
            if result.returncode != 0 or not result.stdout:
                raise RuntimeError('Preprocessing failed:\n' + result.stderr[-2000:])
            cached = (key, result.stdout)
            self.headers[package] = cached
        return cached[1]

    def check(self, package, bmc_command):
        """Run bmc_command (e.g. 'msat_check_invar_bmc -i -a falsification -k 30') for the package's main.smv.

        Returns the nuXmv output, or None if the model layout is unsupported (caller falls back)."""
        package = Path(package)
        parts = split_live_model((package / 'main.smv').read_text())
        if parts is None or '-p' in bmc_command.split():
            return None
        header, body, observations, goal = parts
        preprocessed = self._preprocess(package, header)
        if preprocessed is None:
            return None
        model = static_model(preprocessed, body, observations)
        current = self.sessions.get(package)
        if current is None or current[0] != model:
            if current is not None:
                current[1].close()
                del self.sessions[package]
            path = package / 'main_persistent.smv'
            path.write_text(model)
            self.sessions[package] = (model, NuxmvSession(self.command, path, self.timeout, self.env))
        session = self.sessions[package][1]
        try:
            return session.command(f'{bmc_command} -p "{observation_property(observations, goal)}"', self.timeout)
        except (TimeoutError, RuntimeError):
            session.close()
            del self.sessions[package]
            raise

    def close(self):
        for _, session in self.sessions.values():
            session.close()
        self.sessions.clear()
