from __future__ import annotations

import re
import shutil
import subprocess
from ctypes import CDLL, c_char_p, c_size_t
from pathlib import Path
from typing import Callable

import networkx as nx


class NuXmvConn:
    def __init__(
        self, nuxmv_args: list[str], *, cwd: str | Path | None = None
    ) -> None:
        self.process = subprocess.Popen(
            nuxmv_args,
            cwd=cwd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True,
        )
        self.property_map: dict[str, int] = {}
        try:
            self._read_response()
        except BaseException:
            self.close()
            raise

    def _read_response(self, line_callback: Callable[[str], None] | None = None) -> str:
        if self.process.stdout is None:
            raise RuntimeError("nuXmv stdout is not available")

        output = ""
        pending_line = ""
        while True:
            character = self.process.stdout.read(1)
            if not character:
                raise RuntimeError(
                    "nuXmv stopped before returning a prompt. "
                    f"Output captured: {output}"
                )
            output += character
            if line_callback is not None:
                if character == "\n":
                    line_callback(pending_line)
                    pending_line = ""
                else:
                    pending_line += character
            if output.endswith("nuXmv > "):
                return output[:-len("nuXmv > ")]

    def _send(self, messages: list[str] | str, ignore_output: bool = False) -> None:
        if isinstance(messages, str):
            messages = [messages]
        elif not isinstance(messages, list):
            raise TypeError("nuXmv commands must be a string or list of strings")
        if self.process.stdin is None:
            raise RuntimeError("nuXmv stdin is not available")

        for message in messages:
            self.process.stdin.write(message + "\n")
        self.process.stdin.flush()
        if ignore_output:
            self._read_response()

    def close(self) -> None:
        if self.process.stdin:
            self.process.stdin.close()
        if self.process.stdout:
            self.process.stdout.close()
        self.process.terminate()
        self.process.wait()


class Morty:
    def __init__(self) -> None:
        self.morty_lib = CDLL("lib/libvfm.so")
        self.morty_lib.expandScript.argtypes = [c_char_p, c_char_p, c_size_t]
        self.morty_lib.expandScript.restype = c_char_p
        # self.morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
        # self.morty_lib.morty.restype = c_char_p
        self.args: list[str] = []
        self.nuxmv: NuXmvConn | None = None

    def generate_smv_files(self, args: list[str]) -> None:
        self.args = args
        self.morty_lib.generate_smv_files(";".join(args).encode("utf-8"))
        requested_target = Path(args[5])
        # VFM's native generator strips the leading slash from absolute
        # targets and therefore creates them below the repository root.
        # Locate that native path for Kratos, then move the finished scenario
        # back to the path requested by Python.
        native_target = (
            Path.cwd() / str(requested_target).lstrip("/")
            if requested_target.is_absolute() else requested_target
        )
        target_path = (
            requested_target if requested_target.exists() else native_target
        )
        subprocess.run(
            [
                "external/linux64/kratos",
                "-trans_output_format=nuxmv-module",
                "-trans_enum_mode=symbolic",
                f"-output_file={target_path / 'planner.cpp_combined.k2.smv'}",
                str(target_path / "planner.cpp_combined.k2"),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if target_path != requested_target and target_path.exists():
            requested_target.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(target_path), str(requested_target))
            self.args[5] = str(requested_target)

    def model_check(
        self,
        cex_file: str,
        iteration: int,
        *,
        model_file: str | Path = "main.smv",
        bound: int = 10,
        property_kind: str = "invar",
    ) -> Path | None:
        del iteration
        if bound < 0:
            raise ValueError("model-checking bound must be non-negative")
        if property_kind not in {"invar", "ltl"}:
            raise ValueError("property_kind must be 'invar' or 'ltl'")
        target_path = self.get_target_path()
        model_path = Path(model_file)
        if not model_path.is_absolute():
            model_path = target_path / model_path
        if not model_path.is_file():
            raise FileNotFoundError(f"SMV model does not exist: {model_path}")
        model_path = model_path.resolve()
        self.nuxmv = NuXmvConn(
            ["nuXmv", "-quiet", "-pre", "cpp", "-int", model_path.name],
            cwd=model_path.parent,
        )
        self.nuxmv._send("go_msat")
        setup_output = self.nuxmv._read_response()
        print("SETUPTOUTPUT", setup_output)
        self._raise_on_nuxmv_error(setup_output, "go_msat")

        check_command = (
            f"msat_check_ltlspec_sbmc_inc -k {bound} -n 0"
            if property_kind == "ltl"
            else f"msat_check_invar_bmc -a falsification -k {bound} -n 0"
        )
        self.nuxmv._send(check_command)
        check_output = self.nuxmv._read_response()
        print("OUTP:", check_output)
        self._raise_on_nuxmv_error(
            check_output, check_command.split()[0]
        )
        normalized_output = check_output.lower()
        has_counterexample = bool(
            re.search(r"counter-?example of size\s+\d+ exists", normalized_output)
            or "trace description:" in normalized_output
        )
        if not has_counterexample:
            print(f"No counterexample found for {model_path}")
            return None

        output_cex = (target_path / cex_file).resolve()
        output_cex.unlink(missing_ok=True)
        self.nuxmv._send(f"show_traces -p 6 -o {output_cex} 1")
        trace_output = self.nuxmv._read_response()
        print("OUTP:", trace_output)
        self._raise_on_nuxmv_error(trace_output, "show_traces")
        if not output_cex.is_file():
            raise RuntimeError(
                f"nuXmv reported a counterexample but did not create {output_cex}"
            )
        # Keep NuSMV's native textual trace next to the XML CEX.  This is the
        # format consumed by the existing VFM renderer (and is intentionally
        # emitted by NuSMV rather than reconstructed from XML).
        # nuXmv runs with the model directory as its cwd.  Resolve the output
        # path just as for the XML trace; otherwise a repository-relative
        # target is interpreted a second time below that cwd.
        output_txt = (target_path / "debug_trace_array.txt").resolve()
        output_txt.unlink(missing_ok=True)
        self.nuxmv._send(f"show_traces -o {output_txt} 1")
        text_trace_output = self.nuxmv._read_response()
        print("OUTP:", text_trace_output)
        self._raise_on_nuxmv_error(text_trace_output, "show_traces (text)")
        return output_cex

    @staticmethod
    def _raise_on_nuxmv_error(output: str, command: str) -> None:
        normalized_output = output.lower()
        error_markers = (
            "syntax error",
            "parser error",
            "cannot open",
            "must be flattened",
            "undefined symbol",
        )
        if any(marker in normalized_output for marker in error_markers):
            raise RuntimeError(f"nuXmv {command} failed:\n{output.strip()}")

    def close_model_checker(self) -> None:
        if self.nuxmv is not None:
            self.nuxmv.close()
            self.nuxmv = None

    def get_target_path(self) -> Path:
        if len(self.args) <= 5:
            raise RuntimeError("SMV files have not been generated")
        return Path(self.args[5])

    def block_topology(self, topology: nx.MultiDiGraph, iteration: int) -> None:
        del iteration
        edges_as_variables = []
        for start_node, end_node, data in topology.edges(data=True):
            if start_node == "NONE":
                minus_ones = data.get("data", [])
                for connection, section in minus_ones:
                    edges_as_variables.append(
                        f"env.outgoing_connection_{connection}"
                        f"_of_section_{section} = -1"
                    )
                continue

            connection = data.get("conn_number")
            if connection is None:
                continue
            start = str(start_node)[2:]
            end = str(end_node)[2:]
            edges_as_variables.append(
                f"env.outgoing_connection_{connection}_of_section_{start} = {end}"
            )

        if edges_as_variables:
            expression = " & ".join(edges_as_variables)
            with (self.get_target_path() / "main.smv").open("a") as main_smv:
                main_smv.write(f"INIT !( {expression} )\n")
